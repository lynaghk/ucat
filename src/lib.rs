#![cfg_attr(not(feature = "std"), no_std)]
#![feature(error_in_core)]

pub mod controller;

pub mod device;

pub const MAX_FRAME_SIZE: usize = 2048;

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub trait Device {
    type Command;
    type Status;

    fn pdi_window_size(&self) -> usize;
    fn status(&self, pdi_window: &[u8]) -> Self::Status;
    fn command(&self, pdi_window: &mut [u8], cmd: &Self::Command);
}

use core::mem::size_of;

pub use const_str::concat_bytes;
use embedded_io_async::{Read, ReadExactError, Write};
pub use heapless::String;
use log::*;
use serde::{Deserialize, Serialize};

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

    Ping, // only devices send this upstream once when they boot; used to determine if they are end of chain.
}
use MessageType::*;

// devices send this upstream when they boot
// TODO: put version number as "address" in here?
// TODO: generate this at compile time so it's always synced up with frame definition
const PING_FRAME_BASE: &[u8] = &[
    Ping as u8, // Message Type
    0,          /////////////////////
    0,          // i16 address
    0,          /////////////////////
    0,          // u16 payload length = 0
                /////////////////////
                // (no payload)
];

pub const PRE_PAYLOAD_BYTE_COUNT: usize =
    size_of::<MessageType>() + size_of::<DeviceOrGroupAddressOnWire>() + size_of::<PayloadLength>();
pub const CRC_BYTE_COUNT: usize = 4;

pub const FRAME_SIZE_INITIALIZE: usize = PRE_PAYLOAD_BYTE_COUNT + 2 + 2 + CRC_BYTE_COUNT;

pub const PING_FRAME: &[u8] =
    concat_bytes!(PING_FRAME_BASE, CRC.checksum(PING_FRAME_BASE).to_le_bytes());

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

use bbqueue::Producer;

//////////////////
//TODO: is there a proper way to do this error stuff?

//use thiserror::Error;

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    UnexpectedResponse,

    //#[error("{0:?}")]
    IO(embedded_io_async::ErrorKind),

    TODO,
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

pub async fn handle_frame<'out, R, const N: usize>(
    state: DeviceState,
    latest_status: &[u8],
    command_buf: &'out mut [u8],
    mut rx: R,
    bbq: &mut Producer<'_, N>,
    data_available: impl Fn(),
) -> Result<Option<Action<'out>>, Error>
where
    R: embedded_io_async::Read,
{
    ////////////
    // TODO: move this stuff to config object. Unify with generic const N bbq size
    const CHUNK_SIZE: usize = 32;

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
                return Err(ReadExactErrorWrapper::from(e).into());
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
                    return Err(e.into());
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
                read_digest.update(&buf[0..n]);
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

            let mut address = DeviceOrGroupAddressOnWire::from_le_bytes([buf[1], buf[2]]);

            // increment address for device-oriented messages
            if matches!(t, Enumerate | Initialize | Identify | Query) {
                address += 1;
                buf[1..3].copy_from_slice(&address.to_le_bytes())
            }

            commit!(g, buf);

            ///////////////////////////////
            // read and send payload length

            let mut g = bbq.grant_exact(size_of::<PayloadLength>()).unwrap();
            let buf = g.buf();
            read_exact!(buf);
            let payload_length = PayloadLength::from_le_bytes([buf[0], buf[1]]);
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

                    if payload_length < (pdi_offset.0 + command_buf.len() as u16) {
                        error!("process update payload length smaller than pdi_offset + pdi_window_size");
                        return Err(Error::TODO);
                    }
                    forward!(pdi_offset.0 as usize);

                    write!(latest_status);

                    read_exact!(command_buf);
                    debug!("process update read: {command_buf:?}");

                    Some(Action::NewCommand(command_buf))
                }

                (Initialize | Identify | Query | ProcessUpdate, _, _) => {
                    // message for another device, nothing for us to do
                    None
                }
                (Ping, _, _) => {
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
                let mut actual = [0u8; CRC_BYTE_COUNT];
                read_exact_without_digest!(&mut actual);
                let expected = read_digest.finalize().to_le_bytes();
                let crc_ok = actual == expected;
                if !crc_ok {
                    error!("Bad message CRC, expected {expected:?} got {actual:?}");
                }

                let mut g = bbq.grant_exact(CRC_BYTE_COUNT).unwrap();
                g.buf().copy_from_slice(&if crc_ok {
                    // then we should send our new CRC
                    write_digest.finalize().to_le_bytes()
                } else {
                    // if CRC on original message was bad, we should preserve it so that downstream devices don't act on the bad message (which we've already forwarded)
                    actual
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

        assert_eq!(PING_FRAME, write_frame(&mut buf, MessageType::Ping, 0, &[]));

        assert_eq!(PING_FRAME_BASE.len(), PRE_PAYLOAD_BYTE_COUNT);
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
        async fn read(&mut self, bs: &mut [u8]) -> Result<usize, Self::Error> {
            for b in bs.iter_mut() {
                *b = self.reader.recv().await.map_err(|_| MockError)?;
            }
            Ok(bs.len())
        }
    }

    impl Write for MockSerial {
        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        async fn write(&mut self, bs: &[u8]) -> Result<usize, Self::Error> {
            for &b in bs {
                self.writer.send(b).await.map_err(|_| MockError)?;
            }
            Ok(bs.len())
        }
    }

    trait MockPort: Read<Error = MockError> + Write<Error = MockError> {}
    impl MockPort for MockSerial {}

    async fn device(
        mut port: impl MockPort,
        pdi_window_size: usize,
        cb: fn(status: &mut [u8], cmd: &[u8]),
    ) {
        let bbq = bbqueue::BBBuffer::<{ 1024 * 8 }>::new();
        let (mut producer, mut consumer) = bbq.try_split().unwrap();

        let mut state = DeviceState::Reset;

        // TODO: how should device initialize default status?
        // should there be rust protocol around this, or is it just up to device firmware implementors?
        let mut latest_status = vec![0u8; pdi_window_size];
        let mut command_buf = vec![0u8; pdi_window_size];
        loop {
            let result = handle_frame(
                state.clone(),
                &latest_status,
                &mut command_buf,
                &mut port,
                &mut producer,
                || {},
            )
            .await;

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
        // Enumerate
        // {
        //     port.write(&frame(MessageType::Enumerate, 0, &[])[..])
        //         .await
        //         .unwrap();

        //     wait_for(
        //         &mut port,
        //         &frame(MessageType::Enumerate, network.num_devices() as i16, &[])[..],
        //     )
        //     .await
        //     .unwrap();
        // }

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

        smol::block_on(async {
            let (tx1, rx1) = channel::unbounded();
            let (tx2, rx2) = channel::unbounded();
            let (tx3, rx3) = channel::unbounded();

            let ex = Executor::new();

            let controller_task = ex.spawn(controller(MockSerial {
                reader: rx3,
                writer: tx1,
            }));

            let _device1_task = ex.spawn(device(
                MockSerial {
                    reader: rx1,
                    writer: tx2,
                },
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
                MockSerial {
                    reader: rx2,
                    writer: tx3,
                },
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
