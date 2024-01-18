use log::*;
use ucat::*;
pub fn main() -> anyhow::Result<()> {
    env_logger::init();

    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?;
    port.set_timeout(std::time::Duration::from_millis(10000))?;

    // wait for init frame from device
    {
        let needle = INIT_FRAME;
        const N: usize = INIT_FRAME.len();

        let mut buf = [0; 2 * N];
        let mut offset = 0;

        loop {
            let n = port.read(&mut buf[offset..]).unwrap();
            //debug!("{:?}", &buf[offset..offset + n]);
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

    info!("init frame recieved");

    port.write_all(&INIT_FRAME).unwrap();

    let mut buf = [0u8; 1024];
    loop {
        let n = port.read(&mut buf[..]).unwrap();
        info!("{:?}", &buf[0..n]);
    }

    //debug!("{:?}", &buf[offset..offset + n]);

    // workaround for MacOS issue =( https://github.com/serialport/serialport-rs/issues/117
    std::thread::sleep(std::time::Duration::from_millis(500));
    Ok(())
}
