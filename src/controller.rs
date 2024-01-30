use crate::*;

use log::*;

//////////////////
//TODO: is there a proper way to do this error stuff?
use embedded_io_async::ReadExactError;

use thiserror::Error;

#[derive(Error, Debug)]
pub enum Error {
    #[error("Unexpected Response")]
    UnexpectedResponse,

    #[error("{0:?}")]
    CommonError(crate::Error),

    #[error("{0:?}")]
    IO(embedded_io_async::ErrorKind),
}

impl<E: embedded_io_async::Error> From<ReadExactError<E>> for Error {
    fn from(err: ReadExactError<E>) -> Self {
        match err {
            ReadExactError::UnexpectedEof => Error::UnexpectedResponse,
            ReadExactError::Other(e) => Error::IO(e.kind()),
        }
    }
}

impl From<crate::Error> for Error {
    fn from(err: crate::Error) -> Self {
        Error::CommonError(err)
    }
}

pub async fn wait_for<R>(mut r: R, bs: &[u8]) -> Result<(), Error>
where
    R: embedded_io_async::Read,
{
    // TODO: something smarter than putting max frame on the stack just to compare equality later.
    // presumably some kind of chunked read?
    let n = bs.len();
    let mut buf = [0u8; MAX_FRAME_SIZE];
    r.read_exact(&mut buf[..n]).await?;

    if bs != &buf[..n] {
        debug!("Expected: {bs:?}\nGot: {buf:?}");
        return Err(Error::UnexpectedResponse);
    }

    Ok(())
}

pub async fn try_parse_frame<R>(mut r: R, buf: &mut [u8]) -> Result<Frame, Error>
where
    R: embedded_io_async::Read,
{
    r.read_exact(&mut buf[0..PRE_PAYLOAD_BYTE_COUNT]).await?;

    match MessageType::try_from(buf[0]) {
        Err(e) => {
            error!("Bad message type: {}", e);
            return Err(crate::Error::BadMessageType.into());
        }
        Ok(message_type) => {
            let address = DeviceOrGroupAddressOnWire::from_le_bytes([buf[1], buf[2]]);
            let payload_length = PayloadLength::from_le_bytes([buf[3], buf[4]]) as usize;

            r.read_exact(
                &mut buf[PRE_PAYLOAD_BYTE_COUNT
                    ..(PRE_PAYLOAD_BYTE_COUNT + payload_length + CRC_BYTE_COUNT)],
            )
            .await?;

            let payload = &buf[PRE_PAYLOAD_BYTE_COUNT..(PRE_PAYLOAD_BYTE_COUNT + payload_length)];
            let received_crc = &buf[(PRE_PAYLOAD_BYTE_COUNT + payload_length)
                ..(PRE_PAYLOAD_BYTE_COUNT + payload_length + CRC_BYTE_COUNT)];

            let crc = CRC.checksum(&buf[0..(PRE_PAYLOAD_BYTE_COUNT + payload_length)]);

            if received_crc != crc.to_le_bytes() {
                return Err(crate::Error::CRC.into());
            }

            Ok(Frame {
                message_type,
                address,
                payload,
                crc,
            })
        }
    }
}

/// Wait for init frame, discarding non-matching bytes before it.
pub async fn wait_for_init_frame<R>(mut port: R) -> Result<(), R::Error>
where
    R: embedded_io_async::Read,
    // wait for init frame from device
{
    let needle = PING_FRAME;
    const N: usize = PING_FRAME.len();

    let mut buf = [0; 2 * N];
    let mut offset = 0;

    loop {
        let n = port.read(&mut buf[offset..]).await?;
        offset += n;

        if buf[..offset].windows(N).any(|w| w == needle) {
            return Ok(());
        };

        if offset == 2 * N {
            // buffer is full and we still haven't found anything. Drop front half, as we know they can't be part of any match.
            buf.copy_within(N..2 * N, 0);
            offset = 0;
        }
    }
}

// macro_rules! setup_network {
//     ( $( ($name:ident, $mod:ident) ),* ) => {

//         let mut pdi_offset = 0;

//         $(
//           #[allow(unused_variables)]
//           let $name = $mod::Device{pdi_offset};
//           pdi_offset += $mod::PDI_WINDOW_SIZE;
//         )*

//        #[allow(unused_variables)]

//        // TODO: this should be const
//        let pdi_size = pdi_offset;
//     };
// }

// Can people write Rust macros without ChatGPT?
#[macro_export]
macro_rules! setup_network {
    // Entry point: Start the recursion with initial values
    ( $offsets: ident, $( ($name:ident, $mod:ident) ),* ) => {

        const NUM_DEVICES: usize = setup_network!(@count_devices 0, $(($name, $mod)),*);

        const PDI_SIZE: usize = setup_network!(@total_size 0, $(($name, $mod)),*);

        // TODO: figure out how to do this at compile time.
        // using a local requires passing in the $offsets identifier so everything will be in scope for the caller.
        #[allow(non_snake_case)]
        let mut $offsets = [0usize; NUM_DEVICES];

        setup_network!(@run 0, 0, $offsets, $(($name, $mod)),*);

    };

    // Count the number of devices
    (@count_devices $count:expr, ) => {
        $count
    };
    (@count_devices $count:expr, ($name:ident, $mod:ident) $(, ($next_name:ident, $next_mod:ident))*) => {
        setup_network!(@count_devices $count + 1, $(($next_name, $next_mod)),*)
    };

    // Accumulate total PDI size
    (@total_size $pdi_offset:expr, ) => {
        $pdi_offset
    };
    (@total_size $pdi_offset:expr, ($name:ident, $mod:ident) $(, ($next_name:ident, $next_mod:ident))* ) => {
        setup_network!(@total_size ($pdi_offset + $mod::PDI_WINDOW_SIZE), $(($next_name, $next_mod)),*)
    };

    // Setup
    (@run $idx: expr, $pdi_offset: expr, $offsets:ident, ) => {
        // done
    };
    (@run $idx: expr, $pdi_offset: expr, $offsets:ident, ($name:ident, $mod:ident) $(, ($next_name:ident, $next_mod:ident))* ) => {
       #[allow(unused_variables)]
        let $name: $mod::Device = $mod::Device { pdi_offset: $pdi_offset };
        $offsets[$idx] = $pdi_offset;
        setup_network!(@run ($idx + 1), ($pdi_offset + $mod::PDI_WINDOW_SIZE), $offsets, $(($next_name, $next_mod)),*)
    };
}
