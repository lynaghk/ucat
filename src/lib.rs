pub struct GroupAddress(u16);
pub struct DeviceAddress(i16);
pub struct DeviceCount(u16);

pub enum Message {
    Enumerate(DeviceCount),
    Query(DeviceAddress),
    QueryResponse(DeviceInfo),
    Initialize(DeviceAddress, GroupAddress),
    ProcessUpdate(GroupAddress),
}

pub struct Frame {
    pub command: Command,
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
