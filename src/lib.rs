#![no_std]

pub const MAX_FRAME_SIZE: usize = 2048;

pub use heapless::String;

#[derive(Debug)]
pub struct GroupAddress(u8);

#[derive(Debug)]
pub struct DeviceAddress(i16);

#[derive(Debug)]
pub struct DeviceInfo {
    pub name: String<64>,
}

#[derive(Debug)]
pub enum Message {
    Enumerate,
    Query(DeviceAddress),
    QueryResponse(DeviceInfo),
    Initialize(DeviceAddress, GroupAddress),
    ProcessUpdate(GroupAddress),
}

#[derive(Debug)]
pub struct Frame {
    pub message: Message,
    pub working_count: u16,
    pub crc: u32,
}

pub fn send() {}

//heapless::Vec

// pub fn count_devices() -> Result<u16> {
//     Frame {
//         command: Command::Enumerate,
// data: Heapless
//     }
// }
