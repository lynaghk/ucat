use crate::*;
use core::{marker::PhantomData, ops::Range};
use log::*;

#[derive(Debug)]
pub struct NetworkDevice<D, const GROUP: usize> {
    pub device: D,
    pub address: DeviceAddress,
    pub pdi_offset: PDIOffset,
}

impl<D: Device, const GROUP: usize> NetworkDevice<D, GROUP> {
    pub fn pdi_window(&self) -> Range<usize> {
        let d = &self.device;
        let offset = self.pdi_offset.0 as usize;
        offset..(offset + d.pdi_window_size())
    }
}

#[derive(Debug)]
pub struct Network<Port, const NUM_GROUPS: usize = 1> {
    pub num_devices: usize,
    pub pdi_offsets: [usize; NUM_GROUPS],
    pub port: Port,
}

pub struct Pending;
pub struct Cycled;
pub struct PDI<'buf, State, const GROUP: usize> {
    state: PhantomData<State>,
    pub buf: &'buf mut [u8],
}

impl<'buf, const GROUP: usize> PDI<'buf, Pending, GROUP> {
    pub fn command<D: Device>(&mut self, nd: &NetworkDevice<D, GROUP>, cmd: &D::Command) {
        nd.device.command(&mut self.buf[nd.pdi_window()], cmd);
    }
}

impl<'buf, const GROUP: usize> PDI<'buf, Cycled, GROUP> {
    pub fn status<D: Device>(&self, nd: &NetworkDevice<D, GROUP>) -> D::Status {
        nd.device.status(&self.buf[nd.pdi_window()])
    }
    pub fn reset(self) -> PDI<'buf, Pending, GROUP> {
        self.buf.fill(0);
        PDI {
            state: PhantomData::<Pending>,
            buf: self.buf,
        }
    }
}

impl<Port, const NUM_GROUPS: usize> Network<Port, NUM_GROUPS>
where
    Port: Read + Write,
{
    pub fn new(port: Port) -> Self {
        Network {
            num_devices: 0,
            pdi_offsets: [0; NUM_GROUPS],
            port,
        }
    }

    pub fn num_devices(&self) -> usize {
        self.num_devices
    }

    ////////////////////
    // Convenience methods for common case of single group

    pub async fn add<D: Device>(&mut self, device: D) -> Result<NetworkDevice<D, 0>, Error> {
        self.group_add::<D, 0>(device).await
    }

    pub fn pdi_size(&self) -> usize {
        self.group_pdi_size::<0>()
    }

    pub fn pdi<'b>(&self, buf: &'b mut [u8]) -> PDI<'b, Pending, 0> {
        self.group_pdi::<0>(buf)
    }

    ////////////////////
    // Group methods

    pub async fn group_add<D: Device, const GROUP: usize>(
        &mut self,
        device: D,
    ) -> Result<NetworkDevice<D, GROUP>, Error> {
        self.num_devices += 1;
        let address = DeviceAddress(self.num_devices as u16);
        let group_address = GroupAddress(GROUP as u16);
        let pdi_offset = PDIOffset(self.pdi_offsets[GROUP] as u16);
        let pdi_window_size = device.pdi_window_size();
        self.pdi_offsets[GROUP] += pdi_window_size;
        let d = NetworkDevice {
            device,
            address,
            pdi_offset,
        };

        let mut payload = [0u8; size_of::<GroupAddress>() + size_of::<PDIOffset>()];
        payload[..size_of::<GroupAddress>()].copy_from_slice(&group_address.0.to_le_bytes());
        payload[size_of::<GroupAddress>()..].copy_from_slice(&pdi_offset.0.to_le_bytes());

        write_frame(
            &mut self.port,
            MessageType::Initialize,
            address.to_wire_address(),
            &payload[..],
        )
        .await?;

        // TODO: check device identity / version
        let mut buf = [0; FRAME_SIZE_INITIALIZE];
        read_frame(&mut buf, &mut self.port).await?;

        Ok(d)
    }

    pub fn group_pdi_size<const GROUP: usize>(&self) -> usize {
        self.pdi_offsets[GROUP]
    }

    pub fn group_pdi<'b, const GROUP: usize>(&self, buf: &'b mut [u8]) -> PDI<'b, Pending, GROUP> {
        PDI::<'b, Pending, GROUP> {
            state: PhantomData::<Pending>,
            buf: &mut buf[0..(self.group_pdi_size::<GROUP>())],
        }
    }

    pub async fn cycle<'b, const GROUP: usize>(
        &mut self,
        pdi: PDI<'b, Pending, GROUP>,
    ) -> Result<PDI<'b, Cycled, GROUP>, Error> {
        write_frame(
            &mut self.port,
            MessageType::ProcessUpdate,
            GroupAddress(GROUP as u16).to_wire_address(),
            pdi.buf,
        )
        .await?;

        // Not using read_frame here so that we can 1) break faster on unexpected data and 2) use distinct PDI buffer

        let mut r = CRCReader::new(&mut self.port);
        let mut buf = [0u8; PRE_PAYLOAD_BYTE_COUNT];

        r.read_exact(&mut buf[..])
            .await
            .map_err(ReadExactErrorWrapper::from)?;

        match MessageType::try_from(buf[0]) {
            Err(e) => {
                error!("Bad message type: {}", e);
                Err(crate::Error::BadMessageType)
            }
            Ok(MessageType::ProcessUpdate) => {
                let address = DeviceOrGroupAddressOnWire::from_le_bytes([buf[1], buf[2]]);
                if address != GROUP as i16 {
                    return Err(Error::UnexpectedResponse);
                }

                let payload_length = PayloadLength::from_le_bytes([buf[3], buf[4]]) as usize;
                if payload_length != pdi.buf.len() {
                    return Err(Error::UnexpectedResponse);
                }

                r.read_exact(pdi.buf)
                    .await
                    .map_err(ReadExactErrorWrapper::from)?;

                let mut crc_buf = [0; CRC_BYTE_COUNT];
                r.read_crc(&mut crc_buf).await?;

                Ok(PDI {
                    state: PhantomData::<Cycled>,
                    buf: pdi.buf,
                })
            }
            Ok(_t) => Err(Error::UnexpectedResponse),
        }
    }
}
