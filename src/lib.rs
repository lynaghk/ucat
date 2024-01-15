#![no_std]

pub use heapless::String;

pub struct GroupAddress(u8);
pub struct DeviceAddress(i16);
pub struct DeviceInfo {
    pub name: String<64>,
}

pub enum Message {
    Enumerate,
    Query(DeviceAddress),
    QueryResponse(DeviceInfo),
    Initialize(DeviceAddress, GroupAddress),
    ProcessUpdate(GroupAddress),
}

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
