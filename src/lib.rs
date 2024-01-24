#![cfg_attr(not(feature = "std"), no_std)]
#![feature(error_in_core)]

pub mod controller;

pub const MAX_FRAME_SIZE: usize = 2048;

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub use const_str::concat_bytes;
pub use heapless::String;
use log::*;
use serde::{Deserialize, Serialize};

// #[derive(Debug, Serialize, Deserialize)]
// pub struct Frame<'a> {
//     pub message_type: MessageType,
//     pub address: Address,
//     pub payload: &'a [u8],
//     pub crc: u32, //CRC of everything above
// }

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct DeviceAddress(pub u16);

impl DeviceAddress {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        DeviceAddress(u16::from_le_bytes(bytes))
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct GroupAddress(pub u16);

impl GroupAddress {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        GroupAddress(u16::from_le_bytes(bytes))
    }
}

type DeviceOrGroupAddressOnWire = i16;

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct PDIOffset(pub u16);

impl PDIOffset {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        PDIOffset(u16::from_le_bytes(bytes))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct DeviceInfo {
    pub name: String<64>,
}

//TODO: should this be copy?
#[derive(Debug, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub enum DeviceState {
    Reset,
    Initialized {
        group_address: GroupAddress,
        pdi_offset: PDIOffset,
    },
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum MessageType {
    // device increments address, no action taken
    Enumerate,

    // device increments address, then acts if address is 0.
    Initialize, // device address, read payload: group address, PDI offset
    Identify,   // device address, write payload: DeviceInfo
    Query,      // device address, read/write payload: (device-specific types)

    // device acts if assigned to this group address
    ProcessUpdate, // group address, read/write payload: PDI

    Init, // only devices send this upstream once when they boot; used to determine if they are end of chain.
}
use MessageType::*;

// devices send this upstream when they boot
// TODO: put version number as "address" in here?
// TODO: generate this at compile time so it's always synced up with frame definition
const INIT_FRAME_BASE: &[u8] = &[
    Init as u8, // Message Type
    0,          /////////////////////
    0,          // i16 address
    0,          /////////////////////
    0,          // u16 payload length = 0
                /////////////////////
                // (no payload)
];

pub const PRE_PAYLOAD_BYTE_COUNT: usize = 1 + 2 + 2;
pub const CRC_BYTE_COUNT: usize = 4;

pub const INIT_FRAME: &[u8] =
    concat_bytes!(INIT_FRAME_BASE, CRC.checksum(INIT_FRAME_BASE).to_le_bytes());

// TODO: Would be nice if this could be done at compile time, but Rust isn't there yet. May need to rely on build.rs --- oh, someone made a crate https://github.com/Eolu/const-gen
// TODO: compare generated code here with a fn where I do all of the slice offset math myself.
// chatgpt gave it a go, but requires dangerous nightly #![feature(generic_const_exprs)] https://chat.openai.com/share/f51804d9-3424-4690-900c-c00b73ccbf5e
pub fn write_frame<'a>(
    buf: &'a mut [u8],
    t: MessageType,
    address: DeviceOrGroupAddressOnWire,
    payload: &[u8],
) -> &'a [u8] {
    let mut n = 0;

    // TODO: is this macro usage horrible or fine? Not obvious if generated code is better or worse than a closure, and the latter has issues with letting CRC read buf.
    macro_rules! write {
        ($bs: expr) => {
            let bs = $bs;
            buf[n..(n + bs.len())].copy_from_slice(bs);
            n += bs.len();
        };
    }

    write!(&[t as u8]);
    write!(&address.to_le_bytes());
    write!(&(payload.len() as u16).to_le_bytes());
    write!(payload);

    let crc = CRC.checksum(&buf[0..n]);
    write!(&crc.to_le_bytes());

    &buf[0..n]
}

#[cfg(feature = "std")]
pub fn frame(t: MessageType, address: DeviceOrGroupAddressOnWire, payload: &[u8]) -> Vec<u8> {
    let mut v = vec![0; MAX_FRAME_SIZE];
    let len = write_frame(&mut v[..], t, address, payload).len();
    v.truncate(len);
    v
}

#[cfg(feature = "std")]
pub fn initialize_frame(
    device_address: DeviceAddress,
    group_address: GroupAddress,
    pdi_offset: PDIOffset,
) -> Vec<u8> {
    let mut payload = vec![];

    payload.extend_from_slice(&group_address.0.to_le_bytes());
    payload.extend_from_slice(&pdi_offset.0.to_le_bytes());

    let address = -(device_address.0 as i16);

    let mut v = vec![0; MAX_FRAME_SIZE];
    let len = write_frame(&mut v[..], MessageType::Initialize, address, &payload[..]).len();
    v.truncate(len);
    v
}

use bbqueue::Producer;

#[derive(Debug, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub enum Error {
    TODO,
    RX,
    CRC,
    BadMessageType,
}

#[derive(Debug, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub enum Action {
    NewState(DeviceState),
}

pub async fn try_parse_frame<R, const N: usize>(
    state: DeviceState,
    mut rx: R,
    bbq: &mut Producer<'_, N>,
    data_available: impl Fn(),
) -> Result<Option<Action>, Error>
where
    R: embedded_io_async::Read,
{
    ////////////
    // TODO: move this stuff to config object
    const CHUNK_SIZE: usize = 32;
    // TODO: derive this from postcard or something for this specific device.
    const PDI_WINDOW_SIZE: u16 = 1;

    let mut read_digest = CRC.digest();
    let mut write_digest = CRC.digest();
    let mut sent_bytes = 0;

    // Ahh, rust, where the types are so complicated you resort to abstraction via syntax macros.
    macro_rules! commit_without_digest {
        ($grant:expr, $n: expr) => {
            $grant.commit($n);
            data_available();
        };
    }

    macro_rules! commit {
        ($grant:expr, $buf: expr) => {
            let n = $buf.len();
            write_digest.update($buf);
            commit_without_digest!($grant, n);
            sent_bytes += n;
        };
    }

    macro_rules! write {
        ($bs: expr) => {{
            let bs = $bs;
            let mut g = bbq.grant_exact(bs.len()).unwrap();
            let buf = g.buf();
            buf[..].copy_from_slice(bs);
            commit!(g, buf);
        }};
    }

    macro_rules! read_exact_without_digest {
        ($buf: expr) => {
            if let Err(e) = embedded_io_async::Read::read_exact(&mut rx, $buf).await {
                error!("Upstream RX Error: {:?}", e);
                return Err(Error::RX);
            }
        };
    }

    macro_rules! read_exact {
        ($buf: expr) => {
            read_exact_without_digest!($buf);
            read_digest.update($buf);
        };
    }

    macro_rules! read {
        ($buf: expr) => {
            match embedded_io_async::Read::read(&mut rx, $buf).await {
                Err(e) => {
                    error!("Upstream RX Error: {:?}", e);
                    return Err(Error::RX);
                }
                Ok(n) => n,
            }
        };
    }

    macro_rules! forward {
        ($n: expr) => {{
            let mut remaining = $n;

            while remaining > 0 {
                let mut g = bbq
                    .grant_exact(core::cmp::min(CHUNK_SIZE, remaining))
                    .unwrap();
                let buf = g.buf();
                let n = read!(buf);
                remaining -= n;
                commit!(g, &buf[0..n]);
            }
        }};
    }

    let mut g = bbq.grant_exact(3).unwrap();
    let buf = g.buf();
    read_exact!(buf);

    match MessageType::try_from(buf[0]) {
        Err(e) => {
            error!("Bad message type: {}", e);
            // TODO: should I abort here or just fall through assuming address and payload follow, then let rest of message get forwarded?
            Err(Error::BadMessageType)
        }

        Ok(t) => {
            use MessageType::*;

            ///////////////////////////////
            // read and send address

            let mut address = i16::from_le_bytes([buf[1], buf[2]]);

            // increment address for device-oriented messages
            if matches!(t, Enumerate | Initialize | Identify | Query) {
                address += 1;
                buf[1..3].copy_from_slice(&address.to_le_bytes())
            }

            commit!(g, buf);

            ///////////////////////////////
            // read and send payload length

            let mut g = bbq.grant_exact(2).unwrap();
            let buf = g.buf();
            read_exact!(buf);
            let payload_length = u16::from_le_bytes([buf[0], buf[1]]);
            commit!(g, buf);

            ///////////////////////////////
            // message-specific handling

            let action = match (t, address, &state) {
                (Enumerate, _, _) => {
                    info!("enumerated");
                    None
                }

                (Initialize, 0, _) => {
                    // payload is group address and pdi offset
                    let mut g = bbq.grant_exact(4).unwrap();
                    let buf = g.buf();
                    read_exact!(buf);
                    let group_address = GroupAddress::from_le_bytes([buf[0], buf[1]]);
                    let pdi_offset = PDIOffset::from_le_bytes([buf[2], buf[3]]);
                    commit!(g, buf);
                    let new_state = DeviceState::Initialized {
                        group_address,
                        pdi_offset,
                    };
                    info!("initialized: {new_state:?}");
                    Some(Action::NewState(new_state))
                }

                (Identify, 0, _) => todo!(),

                (Query, 0, _) => todo!(),

                (
                    ProcessUpdate,
                    group_address,
                    DeviceState::Initialized {
                        group_address: GroupAddress(address),
                        pdi_offset,
                    },
                ) if group_address == (*address as i16) => {
                    debug!("Handling ProcessUpdate");

                    if payload_length < ((pdi_offset.0 as u16) + PDI_WINDOW_SIZE) {
                        error!("process update payload length smaller than pdi_offset + PDI_WINDOW_SIZE");
                        return Err(Error::TODO);
                    }
                    forward!(pdi_offset.0 as usize);

                    // TODO get latest payload for real
                    let pdi_write = [55u8; PDI_WINDOW_SIZE as usize];
                    write!(&pdi_write);

                    let mut pdi_read = [0u8; PDI_WINDOW_SIZE as usize];
                    read_exact!(&mut pdi_read[..]);

                    info!("process update read: {pdi_read:?}");

                    // TODO: who owns PDI buffer?
                    //Some(Action::PDIUpdate)

                    None
                }

                (Initialize | Identify | Query | ProcessUpdate, _, _) => {
                    // message for another device, nothing for us to do
                    None
                }
                (Init, _, _) => {
                    error!("Device recieved Init message from Controller");
                    return Err(Error::TODO);
                }
            };

            /////////////////////////////////////////
            // forward remainder of frame up to CRC
            {
                let remaining = PRE_PAYLOAD_BYTE_COUNT + (payload_length as usize) - sent_bytes;
                debug!("Forwarding {remaining} remaining bytes");
                forward!(remaining)
            }

            ///////////////////////////
            // check and send CRC
            let crc_ok = {
                let mut buf = [0u8; CRC_BYTE_COUNT];
                read_exact_without_digest!(&mut buf);
                let actual = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                let expected = read_digest.finalize();
                let crc_ok = actual == expected;
                if !crc_ok {
                    error!("Bad message CRC, expected {expected} got {actual}");
                }

                let mut g = bbq.grant_exact(CRC_BYTE_COUNT).unwrap();
                g.buf().copy_from_slice(&if crc_ok {
                    // then we should send our new CRC
                    write_digest.finalize().to_le_bytes()
                } else {
                    // if CRC on original message was bad, we should preserve it so that downstream devices don't act on the bad message (which we've already forwarded)
                    buf
                });
                commit_without_digest!(g, CRC_BYTE_COUNT);

                crc_ok
            };

            if crc_ok {
                Ok(action)
            } else {
                Err(Error::CRC)
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn misc() {
        let mut buf = [0u8; MAX_FRAME_SIZE];

        assert_eq!(INIT_FRAME, write_frame(&mut buf, MessageType::Init, 0, &[]));

        assert_eq!(INIT_FRAME_BASE.len(), PRE_PAYLOAD_BYTE_COUNT);
    }

    fn test_parse(state: DeviceState, frame: &[u8]) -> (Result<Option<Action>, Error>, Vec<u8>) {
        smol::block_on(async move {
            let bbq = bbqueue::BBBuffer::<{ 1024 * 8 }>::new();
            let (mut producer, mut consumer) = bbq.try_split().unwrap();

            let result = try_parse_frame(state, frame, &mut producer, || {}).await;

            let mut downstream = vec![];
            while let Ok(g) = consumer.read() {
                let buf = g.buf();
                let n = buf.len();
                downstream.extend_from_slice(&buf);
                g.release(n);
            }

            (result, downstream)
        })
    }

    #[test]
    fn integration() {
        assert_eq!(
            test_parse(
                DeviceState::Reset,
                &frame(MessageType::Enumerate, 0, &[])[..],
            ),
            (Ok(None), frame(MessageType::Enumerate, 1, &[]),)
        );
    }

    use embedded_io_async::{Read, Write};
    use smol::channel::{self, Receiver, Sender};

    use smol::Executor;

    #[derive(Debug)]
    struct MockError;

    #[derive(Debug)]
    struct MockSerial {
        reader: Receiver<u8>,
        writer: Sender<u8>,
    }

    impl embedded_io::Error for MockError {
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }

    impl embedded_io::ErrorType for MockSerial {
        type Error = MockError;
    }

    impl Read for MockSerial {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            // Read data from the channel
            for item in buf.iter_mut() {
                *item = self.reader.recv().await.map_err(|_| MockError)?;
            }
            Ok(buf.len())
        }
    }

    impl Write for MockSerial {
        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            // Write data to the channel
            for &item in buf {
                self.writer.send(item).await.map_err(|_| MockError)?;
            }
            Ok(buf.len())
        }
    }

    trait MockPort: Read<Error = MockError> + Write<Error = MockError> {}
    impl MockPort for MockSerial {}

    async fn device(mut port: impl MockPort) {
        let bbq = bbqueue::BBBuffer::<{ 1024 * 8 }>::new();
        let (mut producer, mut consumer) = bbq.try_split().unwrap();

        let mut state = DeviceState::Reset;
        loop {
            let result = try_parse_frame(state.clone(), &mut port, &mut producer, || {}).await;

            while let Ok(g) = consumer.read() {
                let buf = g.buf();
                let n = buf.len();
                port.write_all(&buf).await.unwrap();
                g.release(n);
            }

            match result {
                Ok(None) => {}
                Ok(Some(action)) => match action {
                    Action::NewState(s) => state = s,
                },
                Err(Error::RX) => {
                    return;
                }

                Err(e) => {
                    error!("Unexpected device error: {e:?}");
                    return;
                }
            }
        }
    }

    async fn controller(mut port: impl MockPort) {
        use controller::*;

        port.write(&frame(MessageType::Enumerate, 0, &[])[..])
            .await
            .unwrap();

        wait_for(&mut port, &frame(MessageType::Enumerate, 1, &[])[..])
            .await
            .unwrap();

        port.write(&frame(MessageType::Enumerate, 0, &[])[..])
            .await
            .unwrap();

        // wait_for(&mut port, &frame(MessageType::Initialize, 1, &[])[..])
        //     .await
        //     .unwrap();
    }

    #[test]
    fn test_controller_device_communication() {
        // env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        //     .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
        //     .init();

        smol::block_on(async {
            let (tx1, rx1) = channel::unbounded();
            let (tx2, rx2) = channel::unbounded();

            let controller_serial = MockSerial {
                reader: rx2,
                writer: tx1,
            };
            let device_serial = MockSerial {
                reader: rx1,
                writer: tx2,
            };

            let ex = Executor::new();

            let controller_task = ex.spawn(controller(controller_serial));
            let _device_task = ex.spawn(device(device_serial));

            // Run the tasks a bit
            for _ in 0..100 {
                ex.try_tick();
            }

            assert!(controller_task.is_finished());
            controller_task.await;
        });
    }
}
