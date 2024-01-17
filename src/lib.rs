#![no_std]

pub const MAX_FRAME_SIZE: usize = 2048;

pub use heapless::String;
pub use num_enum::TryFromPrimitive;

use serde::{Deserialize, Serialize};

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
    // device increments working counter
    Enumerate, // address ignored, payload: empty

    // device increments address, then acts if address is 0.
    Initialize, // device address, read payload: group address
    Identify,   // device address, write payload: DeviceInfo
    Query,      // device address, read/write payload: (device-specific types)

    // device acts if assigned to this group address
    ProcessUpdate, // group address, read/write payload: PDI
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Frame<'a> {
    pub message_type: MessageType,
    pub address: Address,
    pub payload: &'a [u8],
    pub crc: u32, //CRC of everything above
}
