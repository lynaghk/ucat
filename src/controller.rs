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
        return Err(Error::UnexpectedResponse);
    }

    Ok(())
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
