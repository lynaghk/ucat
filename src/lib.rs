#![no_std]

pub const MAX_FRAME_SIZE: usize = 2048;

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub use const_str::concat_bytes;
pub use heapless::String;

use serde::{Deserialize, Serialize};

// #[derive(Debug, Serialize, Deserialize)]
// pub struct Frame<'a> {
//     pub message_type: MessageType,
//     pub address: Address,
//     pub payload: &'a [u8],
//     pub crc: u32, //CRC of everything above
// }

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Address(pub i16);

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct PDIOffset(pub u16);

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub name: String<64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeviceState {
    Reset,
    Initialized {
        group_address: Address,
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

// TODO: Would be nice if this could be done at compile time, but Rust isn't there yet. May need to rely on build.rs.
// TODO: compare generated code here with a fn where I do all of the slice offset math myself.
// chatgpt gave it a go, but requires dangerous nightly #![feature(generic_const_exprs)] https://chat.openai.com/share/f51804d9-3424-4690-900c-c00b73ccbf5e
pub fn create_frame<'a>(
    buf: &'a mut [u8],
    t: MessageType,
    address: Address,
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
    write!(&address.0.to_le_bytes());
    write!(&(payload.len() as u16).to_le_bytes());
    write!(payload);

    let crc = CRC.checksum(&buf[0..n]);
    write!(&crc.to_le_bytes());

    &buf[0..n]
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn everything() {
        let mut buf = [0u8; MAX_FRAME_SIZE];

        assert_eq!(
            INIT_FRAME,
            create_frame(&mut buf, MessageType::Init, Address(0), &[])
        );

        assert_eq!(INIT_FRAME_BASE.len(), PRE_PAYLOAD_BYTE_COUNT);
    }
}
