use ucat::*;

pub fn main() -> anyhow::Result<()> {
    let f = Frame {
        //message_type: MessageType::Enumerate,
        message_type: MessageType::Initialize(DeviceAddress(-5)),
        payload: &[99u8],
        working_count: 123,
        crc: 0,
    };
postcard::from_eio(val)


    let v = postcard::to_stdvec(&f)?;
    dbg!(v);
    Ok(())
}
