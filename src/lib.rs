#![cfg_attr(not(feature = "std"), no_std)]
#![feature(error_in_core)]

pub mod controller;

pub const MAX_FRAME_SIZE: usize = 2048;

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

trait Device {
    type Command;
    type Status;

    fn status(&self, pdi: &[u8]) -> Self::Status;
    fn command(&self, pdi: &mut [u8], cmd: Option<&Self::Command>);
}

use core::mem::size_of;

pub use const_str::concat_bytes;
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
        self.0 as i16
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

#[derive(Debug, Clone, Eq, PartialEq)]
pub enum Action<const PDI_WINDOW_SIZE: usize> {
    NewState(DeviceState),
    // TODO: should this be a reference rather than allocated on the stack?
    NewCommand([u8; PDI_WINDOW_SIZE]),
}

pub async fn handle_frame<R, const N: usize, const PDI_WINDOW_SIZE: usize>(
    state: DeviceState,
    latest_status: &[u8; PDI_WINDOW_SIZE],
    mut rx: R,
    bbq: &mut Producer<'_, N>,
    data_available: impl Fn(),
) -> Result<Option<Action<PDI_WINDOW_SIZE>>, Error>
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

                    if payload_length < (pdi_offset.0 + PDI_WINDOW_SIZE as u16) {
                        error!("process update payload length smaller than pdi_offset + PDI_WINDOW_SIZE");
                        return Err(Error::TODO);
                    }
                    forward!(pdi_offset.0 as usize);

                    write!(latest_status);

                    let mut buf = [0u8; PDI_WINDOW_SIZE];
                    read_exact!(&mut buf[..]);
                    debug!("process update read: {buf:?}");

                    Some(Action::NewCommand(buf))
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

    fn test_parse(state: DeviceState, frame: &[u8]) -> (Result<Option<Action<1>>, Error>, Vec<u8>) {
        smol::block_on(async move {
            let bbq = bbqueue::BBBuffer::<{ 1024 * 8 }>::new();
            let (mut producer, mut consumer) = bbq.try_split().unwrap();

            let latest_status = [7u8];

            let result = handle_frame(state, &latest_status, frame, &mut producer, || {}).await;

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

    async fn device<const PDI_WINDOW_SIZE: usize>(
        mut port: impl MockPort,
        cb: fn(status: &mut [u8], cmd: [u8; PDI_WINDOW_SIZE]),
    ) {
        let bbq = bbqueue::BBBuffer::<{ 1024 * 8 }>::new();
        let (mut producer, mut consumer) = bbq.try_split().unwrap();

        let mut state = DeviceState::Reset;
        // TODO: how should device initialize default status?
        // should there be rust protocol around this, or is it just up to device firmware implementors?
        let mut latest_status = [0u8; PDI_WINDOW_SIZE];

        loop {
            let result = handle_frame(
                state.clone(),
                &latest_status,
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
        let mut buf_res = vec![0; MAX_FRAME_SIZE];

        use controller::*;

        use test::devices::{counter, echoer};

        setup_network!(PDI_OFFSETS, (c1, counter), (e1, echoer));

        let mut buf_cmd = vec![0; PDI_SIZE];

        // TODO: make this part of network macro? That would require it to allocate a PDI_SIZE'd buffer rather than letting caller do it.
        // alternatively, generate a closure to populate a buffer?
        c1.command(&mut buf_cmd, None);
        e1.command(&mut buf_cmd, None);

        ///////////////////
        // Enumerate
        {
            port.write(&frame(MessageType::Enumerate, 0, &[])[..])
                .await
                .unwrap();

            wait_for(
                &mut port,
                &frame(MessageType::Enumerate, NUM_DEVICES as i16, &[])[..],
            )
            .await
            .unwrap();
        }

        ///////////////////
        // Initialize

        let group_address = GroupAddress(7);

        for (idx, offset) in PDI_OFFSETS.iter().enumerate() {
            port.write(&initialize_frame(
                DeviceAddress((idx + 1) as u16),
                group_address,
                PDIOffset(*offset as u16),
            ))
            .await
            .unwrap();

            let _f = controller::try_parse_frame(&mut port, &mut buf_res)
                .await
                .unwrap();
        }

        ///////////////////
        // Process Update 1

        c1.command(&mut buf_cmd, Some(&counter::Command::Increment));
        e1.command(&mut buf_cmd, Some(&echoer::Command::SetValue(17)));

        port.write(
            &frame(
                MessageType::ProcessUpdate,
                group_address.to_wire_address(),
                &buf_cmd,
            )[..],
        )
        .await
        .unwrap();

        let f = controller::try_parse_frame(&mut port, &mut buf_res)
            .await
            .unwrap();

        // Even though we sent commands, the statuses returned will be those before those commands were processed by the devices.
        assert!(matches!(c1.status(f.payload), counter::Status::Value(0)));
        assert!(matches!(e1.status(f.payload), echoer::Status::NoValueSet));

        ///////////////////
        // Process Update 2

        c1.command(&mut buf_cmd, None);
        e1.command(&mut buf_cmd, None);

        port.write(
            &frame(
                MessageType::ProcessUpdate,
                group_address.to_wire_address(),
                &buf_cmd,
            )[..],
        )
        .await
        .unwrap();

        let f = controller::try_parse_frame(&mut port, &mut buf_res)
            .await
            .unwrap();

        // TODO: how to improve tests here? When `matches!` fails it doesn't say what the actual value was.
        // Now we expect the results from Process Update 1 to be reported.
        assert!(matches!(c1.status(f.payload), counter::Status::Value(1)));
        assert!(matches!(e1.status(f.payload), echoer::Status::ValueSet(17)));
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
                |status, cmd: [u8; counter::PDI_WINDOW_SIZE]| {
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
                |status, cmd: [u8; echoer::PDI_WINDOW_SIZE]| {
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

        // TODO: Is there a nicer way for this default device impl without a macro? Need generics on modules
        #[macro_export]
        macro_rules! device_impl {
            () => {
                pub struct Device {
                    pub pdi_offset: usize,
                }

                impl crate::Device for Device {
                    type Status = Status;
                    type Command = Command;

                    fn status(&self, pdi: &[u8]) -> Status {
                        postcard::from_bytes(
                            &pdi[self.pdi_offset..(self.pdi_offset + PDI_WINDOW_SIZE)],
                        )
                        .unwrap()
                    }

                    fn command(&self, pdi: &mut [u8], cmd: Option<&Command>) {
                        postcard::to_slice(
                            &cmd,
                            &mut pdi[self.pdi_offset..(self.pdi_offset + PDI_WINDOW_SIZE)],
                        )
                        .unwrap();
                    }
                }
            };
        }

        pub mod echoer {
            use postcard::experimental::max_size::MaxSize;
            use serde::{Deserialize, Serialize};

            // TODO:
            // const fn max(a: usize, b: usize) -> usize {
            //     [a, b][(a < b) as usize]
            // }

            // TODO: core::cmp::max isn't max, so just waste a bunch of bytes. Ugh, Rust.
            pub const PDI_WINDOW_SIZE: usize =
                Option::<Command>::POSTCARD_MAX_SIZE + Status::POSTCARD_MAX_SIZE;

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

            super::super::device_impl!();
        }

        pub mod counter {
            use postcard::experimental::max_size::MaxSize;
            use serde::{Deserialize, Serialize};

            pub const PDI_WINDOW_SIZE: usize =
                Option::<Command>::POSTCARD_MAX_SIZE + Status::POSTCARD_MAX_SIZE;

            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Command {
                Increment,
            }
            #[derive(Debug, Clone, Serialize, Deserialize, MaxSize)]
            pub enum Status {
                Value(u8),
            }

            super::super::device_impl!();
        }
    }
}
