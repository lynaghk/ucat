pub fn main() -> anyhow::Result<()> {
    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?;
    port.set_timeout(std::time::Duration::from_millis(500))?;

    port.write_all(
        "Hellooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo"
            .as_bytes(),
    )?;
    port.flush()?;

    std::thread::sleep(std::time::Duration::from_millis(500));
    let mut response = vec![0; 1000];
    let n = port.read(&mut response).unwrap();
    dbg!(std::str::from_utf8(&response[0..n]));

    // workaround for MacOS issue =( https://github.com/serialport/serialport-rs/issues/117
    std::thread::sleep(std::time::Duration::from_millis(500));
    Ok(())
}
