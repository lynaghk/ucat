pub fn main() -> anyhow::Result<()> {
    let mut port = serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?;

    port.write_all("Hello".as_bytes())?;
    port.flush()?;

    // workaround for MacOS issue =( https://github.com/serialport/serialport-rs/issues/117
    std::thread::sleep(std::time::Duration::from_millis(500));
    Ok(())
}
