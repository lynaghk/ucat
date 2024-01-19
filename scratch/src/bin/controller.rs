use log::*;
use ucat::*;

pub fn wait_for_init_frame(port: &mut Box<dyn serialport::SerialPort>) {
    // wait for init frame from device
    {
        let needle = INIT_FRAME;
        const N: usize = INIT_FRAME.len();

        let mut buf = [0; 2 * N];
        let mut offset = 0;

        loop {
            let n = port.read(&mut buf[offset..]).unwrap();
            offset += n;

            if buf[..offset].windows(N).any(|w| w == needle) {
                // done!
                break;
            };

            if offset == 2 * N {
                // buffer is full and we still haven't found anything. Drop front half, as we know they can't be part of any match.
                buf.copy_within(N..2 * N, 0);
                offset = 0;
            }
        }
    }
}

pub fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
        .init();

    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?;
    port.set_timeout(std::time::Duration::from_millis(1000000000000))?;

    let mut outgoing = [0u8; MAX_FRAME_SIZE];
    let mut incoming = [0u8; MAX_FRAME_SIZE];
    // info!(
    //     "{:?}",
    //     create_frame(&mut outgoing, MessageType::Init, Address(0), &[])
    // );

    // wait_for_init_frame(&mut port);
    // info!("init frame recieved");

    port.write_all(&create_frame(
        &mut outgoing,
        MessageType::Enumerate,
        Address(0),
        &[],
    ))
    .unwrap();

    // TODO: wait for correct response

    let group_address = Address(7);
    let pdi_length = 2;
    let pdi_offset = PDIOffset(1);

    port.write_all(&create_frame(
        &mut outgoing,
        MessageType::Initialize,
        Address(-1),
        &vec![group_address.0.to_le_bytes(), pdi_offset.0.to_le_bytes()].concat(),
    ))
    .unwrap();

    // TODO: wait for correct response

    port.write_all(&create_frame(
        &mut outgoing,
        MessageType::ProcessUpdate,
        group_address,
        &vec![9u8; pdi_length],
    ))
    .unwrap();

    loop {
        let n = port.read(&mut incoming[..]).unwrap();
        info!("{:?}", &incoming[0..n]);
    }
}
