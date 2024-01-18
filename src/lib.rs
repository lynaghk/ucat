#![no_std]

pub const MAX_FRAME_SIZE: usize = 2048;

pub type Digest = crc::Digest<'static, u32>;
pub const CRC: crc::Crc<u32> = crc::Crc::<u32>::new(&crc::CRC_32_CKSUM);

pub use const_str::concat_bytes;
pub use heapless::String;
pub use num_enum::TryFromPrimitive;

use serde::{Deserialize, Serialize};

// #[derive(Debug, Serialize, Deserialize)]
// pub struct Frame<'a> {
//     pub message_type: MessageType,
//     pub address: Address,
//     pub payload: &'a [u8],
//     pub crc: u32, //CRC of everything above
// }

/// devices increment address for some message types
#[derive(Debug, Serialize, Deserialize)]
pub struct Address(pub i16);

#[derive(Debug, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub name: String<64>,
}

#[derive(Debug, Serialize, Deserialize, num_enum::TryFromPrimitive)]
#[repr(u8)]
pub enum MessageType {
    // device increments address, no action taken
    Enumerate,

    // device increments address, then acts if address is 0.
    Initialize, // device address, read payload: group address
    Identify,   // device address, write payload: DeviceInfo
    Query,      // device address, read/write payload: (device-specific types)

    // device acts if assigned to this group address
    ProcessUpdate, // group address, read/write payload: PDI
}
use MessageType::*;

// devices send this upstream when they boot
// TODO: put version number as "address" in here?
// TODO: generate this at compile time so it's always synced up with frame definition
const INIT_FRAME_BASE: &[u8] = &[
    Enumerate as u8,
    /////////////////////
    0, // i16 address
    0,
    /////////////////////
    0, // u16 payload length = 0
    0,
    /////////////////////
    // (no payload)
    //
];

pub const INIT_FRAME: &[u8] =
    concat_bytes!(INIT_FRAME_BASE, CRC.checksum(INIT_FRAME_BASE).to_le_bytes());
