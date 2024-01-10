use ::schema::Light;
use eui::*;
use std::time::Duration;

pub fn main() -> anyhow::Result<()> {
    env_logger::init();

    let (cmd_tx, mut cmd_rx) = channel::<Light>(64);
    let (status_tx, status_rx) = channel::<Light>(64);

    std::thread::spawn(|| {
        serve_blocking("127.0.0.1:8080", status_rx, cmd_tx);
    });

    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200)
        .timeout(Duration::from_millis(10))
        .open()
        .expect("Failed to open port");

    loop {
        while let Ok(cmd) = cmd_rx.try_recv() {
            println!("{:?}", cmd);
            let mut buf = [0; 64];
            let output = postcard::to_slice_cobs(&cmd, &mut buf)?;
            port.write(&output).expect("Write failed!");
        }

        let _ = status_tx.try_send(Light::Off);

        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
