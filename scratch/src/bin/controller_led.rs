use log::*;
use ucat::controller::*;

use ucat::*;

pub fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
        .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
        .init();

    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?;
    port.set_timeout(std::time::Duration::from_millis(1000000000000))?;

    let mut incoming = vec![0; MAX_FRAME_SIZE];

    use ucat::device::led;
    use ucat::device::led::{Color, Light};

    setup_network!(PDI_OFFSETS, (l1, led));

    let mut buf_cmd = vec![0; PDI_SIZE];

    l1.command(&mut buf_cmd, None);

    ///////////////////
    // Enumerate
    {
        port.write_all(&frame(MessageType::Enumerate, 0, &[])[..])
            .unwrap();

        // wait_for(
        //     &mut port,
        //     &frame(MessageType::Enumerate, NUM_DEVICES as i16, &[])[..],
        // )
        // .await
        // .unwrap();
    }

    ///////////////////
    // Initialize

    let group_address = GroupAddress(7);

    for (idx, offset) in PDI_OFFSETS.iter().enumerate() {
        port.write_all(&initialize_frame(
            DeviceAddress((idx + 1) as u16),
            group_address,
            PDIOffset(*offset as u16),
        ))
        .unwrap();

        // let _f = controller::try_parse_frame(&mut port, &mut buf_res)
        //     .await
        //     .unwrap();
    }

    ///////////////////
    // Process Update 1

    l1.command(&mut buf_cmd, Some(&Light::On(Color { r: 10, g: 0, b: 10 })));

    port.write_all(
        &frame(
            MessageType::ProcessUpdate,
            group_address.to_wire_address(),
            &buf_cmd,
        )[..],
    )
    .unwrap();

    // let f = controller::try_parse_frame(&mut port, &mut buf_res)
    //     .await
    //     .unwrap();

    loop {
        let n = port.read(&mut incoming[..]).unwrap();
        info!("{:?}", &incoming[0..n]);
    }
}
