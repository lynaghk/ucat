#![cfg_attr(not(feature = "std"), no_std)]
#![feature(error_in_core)]

pub mod controller;
pub mod device;
mod util;

use core::mem::size_of;

use embedded_io_async::{Read, ReadExactError, Write};
use log::*;
use serde::{Deserialize, Serialize};

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub const MAX_FRAME_SIZE: usize = 2048;
pub const PRE_PAYLOAD_BYTE_COUNT: usize =
    size_of::<MessageType>() + size_of::<DeviceOrGroupAddressOnWire>() + size_of::<PayloadLength>();
pub const MAX_PAYLOAD_LENGTH: usize = MAX_FRAME_SIZE - PRE_PAYLOAD_BYTE_COUNT - CRC_BYTE_COUNT;
pub const CRC_BYTE_COUNT: usize = 4;
pub const BAUD_RATE: usize = 115_200;
pub const FRAME_SIZE_INITIALIZE: usize = PRE_PAYLOAD_BYTE_COUNT + 2 + 2 + CRC_BYTE_COUNT;

pub trait Device {
    type Command;
    type Status;

    fn pdi_window_size(&self) -> usize;
    fn status(&self, pdi_window: &[u8]) -> Self::Status;
    fn command(&self, pdi_window: &mut [u8], cmd: &Self::Command);
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Frame<'a> {
    pub message_type: MessageType,
    pub address: DeviceOrGroupAddressOnWire,
    pub payload: &'a [u8],
    pub crc: u32, //CRC of everything above
}

type DeviceOrGroupAddressOnWire = i16;

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct DeviceAddress(pub u16);

impl DeviceAddress {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        DeviceAddress(u16::from_le_bytes(bytes))
    }
    pub fn to_wire_address(&self) -> i16 {
        -(self.0 as i16)
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct GroupAddress(pub u16);

impl GroupAddress {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        GroupAddress(u16::from_le_bytes(bytes))
    }
    pub fn to_wire_address(&self) -> i16 {
        self.0 as i16
    }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct PDIOffset(pub u16);

impl PDIOffset {
    pub fn from_le_bytes(bytes: [u8; 2]) -> Self {
        PDIOffset(u16::from_le_bytes(bytes))
    }
}

type PayloadLength = u16;

#[derive(Debug, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub struct DeviceInfo {
    pub name: &'static str,
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
    // device increments address, then acts if address is 0.
    Initialize, // device address, read payload: group address, PDI offset, end-of-group bool
    Identify,   // device address, write payload: DeviceInfo
    Query,      // device address, read/write payload: (device-specific types)

    // device acts if assigned to this group address
    ProcessUpdate, // group address, read/write payload: PDI
}

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
    write!(&(payload.len() as PayloadLength).to_le_bytes());
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

struct CRCWriter<W> {
    writer: W,
    digest: crc::Digest<'static, u32>,
}

impl<W: Write> embedded_io::ErrorType for CRCWriter<W> {
    type Error = W::Error;
}

impl<W: Write> CRCWriter<W> {
    pub fn new(writer: W) -> Self {
        Self {
            writer,
            digest: CRC.digest(),
        }
    }
    pub async fn write_crc(
        mut self,
    ) -> Result<(), <CRCWriter<W> as embedded_io::ErrorType>::Error> {
        self.writer
            .write_all(&self.digest.finalize().to_le_bytes())
            .await?;
        Ok(())
    }
}

impl<W: Write> Write for CRCWriter<W> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let n = self.writer.write(buf).await?;
        self.digest.update(&buf[0..n]);
        Ok(n)
    }
}

#[derive(Debug)]
pub enum CRCReaderError<E> {
    CRC,
    IO(E),
}

impl<E: core::fmt::Debug> embedded_io_async::Error for CRCReaderError<E> {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::InvalidData
    }
}

struct CRCReader<R> {
    reader: R,
    digest: crc::Digest<'static, u32>,
}

impl<R: Read> embedded_io::ErrorType for CRCReader<R> {
    type Error = CRCReaderError<R::Error>;
}

impl<R: Read> CRCReader<R> {
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            digest: CRC.digest(),
        }
    }
    pub async fn read_crc(mut self) -> Result<(), Error> {
        let mut buf = [0u8; CRC_BYTE_COUNT];
        self.reader
            .read_exact(&mut buf)
            .await
            .map_err(ReadExactErrorWrapper::from)?;

        if buf == self.digest.finalize().to_le_bytes() {
            Ok(())
        } else {
            // TODO: better error?
            //Error::IO(embedded_io_async::ErrorKind::InvalidData)
            Err(Error::CRC)
        }
    }
}

impl<R: Read> Read for CRCReader<R> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = self.reader.read(buf).await.map_err(CRCReaderError::IO)?;
        self.digest.update(&buf[0..n]);
        Ok(n)
    }
}

pub async fn write_frame2<W: Write>(
    writer: W,
    message_type: MessageType,
    address: DeviceOrGroupAddressOnWire,
    payload: &[u8],
) -> Result<(), Error> {
    let mut w = CRCWriter::new(writer);
    w.write(&[message_type as u8]).await?;
    w.write(&address.to_le_bytes()).await?;
    w.write(&(payload.len() as PayloadLength).to_le_bytes())
        .await?;
    w.write(payload).await?;
    w.write_crc().await?;
    Ok(())
}

//////////////////
//TODO: is there a proper way to do this error stuff?

//use thiserror::Error;

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    UnexpectedResponse,

    //#[error("{0:?}")]
    IO(embedded_io_async::ErrorKind),

    TODO,
    InvalidLength,
    CRC,
    BadMessageType,
    PDILengthMismatch,
}

pub struct ReadExactErrorWrapper<E>(pub ReadExactError<E>);

impl<E: embedded_io_async::Error> From<ReadExactError<E>> for ReadExactErrorWrapper<E> {
    fn from(error: ReadExactError<E>) -> Self {
        ReadExactErrorWrapper(error)
    }
}
impl<E: embedded_io_async::Error> From<ReadExactErrorWrapper<E>> for Error {
    fn from(wrapper: ReadExactErrorWrapper<E>) -> Self {
        match wrapper.0 {
            ReadExactError::UnexpectedEof => Error::UnexpectedResponse,
            ReadExactError::Other(e) => Error::IO(e.kind()),
        }
    }
}

impl<E: embedded_io_async::Error> From<E> for Error {
    fn from(e: E) -> Self {
        Error::IO(e.kind())
    }
}

// impl embedded_io_async::Error for Error {
//     fn kind(&self) -> embedded_io::ErrorKind {
//         match self {
//             Error::UnexpectedResponse => embedded_io_async::ErrorKind::InvalidData,
//             Error::IO(e) => e.kind(),
//             Error::TODO => embedded_io_async::ErrorKind::Other,
//             Error::CRC => embedded_io_async::ErrorKind::InvalidData,
//             Error::BadMessageType => embedded_io_async::ErrorKind::InvalidData,
//         }
//     }
// }

#[derive(Debug, Clone, Eq, PartialEq)]
pub enum Action<'out> {
    NewState(DeviceState),
    // TODO: should this be a reference rather than allocated on the stack?
    NewCommand(&'out [u8]),
}

pub type PayloadIdx = core::ops::Range<usize>;

pub async fn read_frame<R: Read>(frame_buf: &mut [u8], mut r: R) -> Result<&mut [u8], Error> {
    let header = &mut frame_buf[0..PRE_PAYLOAD_BYTE_COUNT];
    r.read_exact(header).await.map_err(ReadExactErrorWrapper)?;
    info!("{:?}", &header);

    let payload_length = PayloadLength::from_le_bytes([header[3], header[4]]) as usize;
    if payload_length > MAX_PAYLOAD_LENGTH {
        return Err(Error::InvalidLength);
    }
    let total_length = PRE_PAYLOAD_BYTE_COUNT + payload_length + CRC_BYTE_COUNT;
    // r.read_exact(&mut frame_buf[PRE_PAYLOAD_BYTE_COUNT..total_length])
    //     .await
    //     .map_err(ReadExactErrorWrapper)?;

    let mut buf = &mut frame_buf[PRE_PAYLOAD_BYTE_COUNT..total_length];
    while !buf.is_empty() {
        info!("{:?}", &buf);
        match r.read(buf).await {
            Ok(0) => break,
            Ok(n) => buf = &mut buf[n..],
            Err(e) => return Err(Error::TODO),
        }
    }

    Ok(&mut frame_buf[0..total_length])
}

/// Reference implementation of frame handling logic.
/// Introduces substantial latency --- buffers entire frame in memory before sending.
pub async fn handle_frame<'command, 'frame, Rxu, Txu, Txd>(
    state: DeviceState,
    latest_status: &[u8],
    command_buf: &'command mut [u8],
    frame_buf: &'frame mut [u8],
    mut rx_upstream: Rxu,
    mut tx_upstream: Txu,
    mut tx_downstream: Txd,
) -> Result<Option<Action<'command>>, Error>
where
    Rxu: Read,
    Txu: Write,
    Txd: Write,
{
    let frame = read_frame(frame_buf, &mut rx_upstream).await?;
    debug!("device read frame: {:?}", frame);

    let Ok(t) = MessageType::try_from(frame[0]) else {
        error!("No message type for: {}", frame[0]);
        // TODO: should I abort here or just fall through assuming address and payload follow, then let rest of message get forwarded?
        return Err(Error::BadMessageType);
    };

    //////////////////////
    //  check CRC

    let crc_offset = frame.len() - CRC_BYTE_COUNT;
    let expected = CRC.checksum(&frame[0..crc_offset]).to_le_bytes();
    if &frame[crc_offset..] != expected {
        error!("Bad message CRC");
        return Err(Error::CRC);
    }

    /////////////////
    // handle address

    let mut address = DeviceOrGroupAddressOnWire::from_le_bytes([frame[1], frame[2]]);

    // increment address for device-oriented messages
    let device_addressed_message = matches!(t, Initialize | Identify | Query);

    if device_addressed_message {
        address += 1;
        frame[1..=2].copy_from_slice(&address.to_le_bytes())
    }

    let payload = &mut frame[PRE_PAYLOAD_BYTE_COUNT..crc_offset];

    use MessageType::*;
    #[derive(Debug)]
    enum WriteDirection {
        Upstream,
        Downstream,
    }
    use WriteDirection::*;

    ///////////////////////////////
    // message-specific handling

    let (write_direction, action) = match (t, address, &state) {
        (Identify, 0, _) => todo!(),
        (Query, 0, _) => todo!(),

        (Initialize, 0, _) => {
            // payload is group address, pdi offset
            let group_address = GroupAddress::from_le_bytes([payload[0], payload[1]]);
            let pdi_offset = PDIOffset::from_le_bytes([payload[2], payload[3]]);
            let new_state = DeviceState::Initialized {
                group_address,
                pdi_offset,
            };
            info!("initialized: {new_state:?}");
            (Upstream, Some(Action::NewState(new_state)))
        }

        (
            ProcessUpdate,
            group_address,
            DeviceState::Initialized {
                group_address: GroupAddress(address),
                pdi_offset,
                ..
            },
        ) if group_address == (*address as i16) => {
            debug!("Handling ProcessUpdate");

            let Some(pdi_window) = payload
                .get_mut((pdi_offset.0 as usize)..(pdi_offset.0 as usize + command_buf.len()))
            else {
                error!("process update payload length smaller than pdi_offset + pdi_window_size");
                return Err(Error::TODO);
            };

            // Read command, write status
            command_buf.copy_from_slice(pdi_window);
            debug!("process update read: {pdi_window:?}");

            pdi_window.copy_from_slice(latest_status);
            debug!("process update wrote: {pdi_window:?}");

            let action = if command_buf.iter().all(|b| *b == 0) {
                None
            } else {
                Some(Action::NewCommand(command_buf))
            };

            let end_of_group = pdi_window.as_ptr_range().end == payload.as_ptr_range().end;
            debug!("end of group: {}", end_of_group);
            let write_direction = if end_of_group { Upstream } else { Downstream };
            (write_direction, action)
        }

        (Initialize | Identify | Query | ProcessUpdate, _, _) => {
            // message for another device, nothing for us to do
            (Downstream, None)
        }
    };

    // Recalculate CRC
    let new_crc = CRC.checksum(&frame[0..crc_offset]).to_le_bytes();
    frame[crc_offset..].copy_from_slice(&new_crc);

    /////////////////////////////////////
    //write frame upstream or downstream
    info!("{:?}", write_direction);
    match write_direction {
        Upstream => tx_upstream.write_all(frame).await?,
        Downstream => tx_downstream.write_all(frame).await?,
    }

    Ok(action)
}

#[cfg(test)]
mod test {
    use super::*;

    use embedded_io::ReadReady;
    use embedded_io_async::{Read, Write};
    use smol::channel::{self, Receiver, Sender};

    use smol::Executor;

    #[derive(Debug)]
    struct MockError;
    impl embedded_io::Error for MockError {
        fn kind(&self) -> embedded_io::ErrorKind {
            embedded_io::ErrorKind::Other
        }
    }

    #[derive(Debug)]
    struct Tx(Sender<u8>);
    #[derive(Debug)]
    struct Rx(Receiver<u8>);

    #[derive(Debug)]
    struct MockSerial {
        tx: Tx,
        rx: Rx,
    }

    impl embedded_io::ErrorType for MockSerial {
        type Error = MockError;
    }
    impl embedded_io::ErrorType for Rx {
        type Error = MockError;
    }
    impl embedded_io::ErrorType for Tx {
        type Error = MockError;
    }

    impl Read for Rx {
        async fn read(&mut self, bs: &mut [u8]) -> Result<usize, Self::Error> {
            for b in bs.iter_mut() {
                *b = self.0.recv().await.map_err(|_| MockError)?;
            }
            Ok(bs.len())
        }
    }
    impl ReadReady for Rx {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(!self.0.is_empty())
        }
    }

    impl Write for Tx {
        async fn write(&mut self, bs: &[u8]) -> Result<usize, Self::Error> {
            for &b in bs {
                self.0.send(b).await.map_err(|_| MockError)?;
            }
            Ok(bs.len())
        }
    }

    impl Read for MockSerial {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.rx.read(buf).await
        }
    }

    impl Write for MockSerial {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx.write(buf).await
        }
    }

    trait MockPort: Read<Error = MockError> + Write<Error = MockError> {}
    impl MockPort for MockSerial {}

    async fn device(
        mut rx_upstream: impl Read + ReadReady,
        mut tx_upstream: impl Write,
        mut rx_downstream: impl Read + ReadReady,
        mut tx_downstream: impl Write,
        pdi_window_size: usize,
        cb: fn(status: &mut [u8], cmd: &[u8]),
    ) {
        let mut state = DeviceState::Reset;

        // TODO: how should device initialize default status?
        // should there be rust protocol around this, or is it just up to device firmware implementors?
        let mut latest_status = vec![0u8; pdi_window_size];
        let mut command_buf = vec![0u8; pdi_window_size];
        let mut frame_buf = vec![0u8; MAX_FRAME_SIZE];
        loop {
            if rx_upstream.read_ready().unwrap() {
                let result = handle_frame(
                    state.clone(),
                    &latest_status,
                    &mut command_buf,
                    &mut frame_buf,
                    &mut rx_upstream,
                    &mut tx_upstream,
                    &mut tx_downstream,
                )
                .await;

                match result {
                    Ok(None) => {}
                    Ok(Some(action)) => match action {
                        Action::NewState(s) => state = s,
                        Action::NewCommand(cmd) => cb(&mut latest_status[..], cmd),
                    },
                    // Err(Error::IO(_)) => {
                    //     return;
                    // }
                    Err(e) => {
                        error!("Unexpected device error: {e:?}");
                        return;
                    }
                }
            }

            // Forward any downstream frames upstream
            if rx_downstream.read_ready().unwrap() {
                let f = read_frame(&mut frame_buf, &mut rx_downstream)
                    .await
                    .unwrap();

                tx_upstream.write_all(f).await.unwrap();
            }

            // let other async tasks in this test run.
            smol::future::yield_now().await;
        }
    }

    async fn controller(mut port: impl MockPort) {
        let mut buf = [0; MAX_FRAME_SIZE];

        use controller::*;

        use test::devices::{counter, echoer};

        // TODO: fancy types to zero-cost prevent devices from being used between different Networks?
        // TODO: how can I avoid needing to annotate NUM_GROUPS = 1 in constructor here?
        let mut network = Network::<_, 1>::new(&mut port);

        // Devices must be added in physical daisy-chain order
        let c1 = network.add(counter::Counter {}).await.unwrap();
        let e1 = network.add(echoer::Echoer {}).await.unwrap();

        ///////////////////
        // Process Update 1

        let mut pdi = network.pdi(&mut buf);

        assert_eq!(pdi.buf.len(), c1.pdi_window().len() + e1.pdi_window().len());

        pdi.command(&c1, &counter::Command::Increment);
        pdi.command(&e1, &echoer::Command::SetValue(17));
        let pdi = network.cycle(pdi).await.unwrap();

        // Even though we sent commands, the statuses returned will be those before those commands were processed by the devices.
        assert!(matches!(pdi.status(&c1), counter::Status::Value(0)));
        assert!(matches!(pdi.status(&e1), echoer::Status::NoValueSet));

        ///////////////////
        // Process Update 2
        let pdi = pdi.reset();
        let pdi = network.cycle(pdi).await.unwrap();

        // TODO: how to improve tests here? When `matches!` fails it doesn't say what the actual value was.
        // Now we expect the results from Process Update 1 to be reported.
        assert!(matches!(pdi.status(&c1), counter::Status::Value(1)));
        assert!(matches!(pdi.status(&e1), echoer::Status::ValueSet(17)));
    }

    #[test]
    fn test_controller_device_communication() {
        use test::devices::{counter, echoer};

        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
            .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
            .init();

        let new_chan = || {
            let (tx, rx) = channel::unbounded();
            (Tx(tx), Rx(rx))
        };

        smol::block_on(async {
            // controller ---a--- device 1 ---b--- device2 ---c--- (empty)
            let (a_u_tx, a_d_rx) = new_chan();
            let (a_d_tx, a_u_rx) = new_chan();
            let (b_u_tx, b_d_rx) = new_chan();
            let (b_d_tx, b_u_rx) = new_chan();
            let (c_u_tx, c_d_rx) = new_chan();
            let (c_d_tx, c_u_rx) = new_chan();

            // no one attached beyond c
            drop((c_d_tx, c_d_rx));

            let ex = Executor::new();

            let controller_task = ex.spawn(controller(MockSerial {
                rx: a_u_rx,
                tx: a_u_tx,
            }));

            let _device1_task = ex.spawn(device(
                a_d_rx,
                a_d_tx,
                b_u_rx,
                b_u_tx,
                (counter::Counter {}).pdi_window_size(),
                |status, cmd: &[u8]| {
                    if let Some(cmd) =
                        postcard::from_bytes::<Option<counter::Command>>(&cmd).unwrap()
                    {
                        let old = postcard::from_bytes::<counter::Status>(status).unwrap();
                        match (old, cmd) {
                            (counter::Status::Value(v), counter::Command::Increment) => {
                                postcard::to_slice(&counter::Status::Value(v + 1), status).unwrap();
                            }
                        }
                    }
                },
            ));

            let _device2_task = ex.spawn(device(
                b_d_rx,
                b_d_tx,
                c_u_rx,
                c_u_tx,
                (echoer::Echoer {}).pdi_window_size(),
                |status, cmd: &[u8]| {
                    if let Some(cmd) =
                        postcard::from_bytes::<Option<echoer::Command>>(&cmd).unwrap()
                    {
                        let new = match cmd {
                            echoer::Command::SetValue(v) => echoer::Status::ValueSet(v),
                            echoer::Command::ClearValue => echoer::Status::NoValueSet,
                        };
                        postcard::to_slice(&new, status).unwrap();
                    }
                },
            ));

            // Run the tasks a bit
            for _ in 0..500 {
                ex.try_tick();
            }

            assert!(controller_task.is_finished());
            // need to await here so that any panic is propagated up and fails the test
            controller_task.await;
        });
    }

    mod devices {
        pub mod echoer {
            use postcard::experimental::max_size::MaxSize;
            use serde::{Deserialize, Serialize};

            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Command {
                SetValue(u8),
                ClearValue,
            }
            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Status {
                NoValueSet,
                ValueSet(u8),
            }

            pub struct Echoer {}
            crate::default_device_impl!(Echoer, Command, Status);
        }

        pub mod counter {
            use postcard::experimental::max_size::MaxSize;
            use serde::{Deserialize, Serialize};

            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Command {
                Increment,
            }
            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Status {
                Value(u8),
            }
            pub struct Counter {}
            crate::default_device_impl!(Counter, Command, Status);
        }
    }
}
