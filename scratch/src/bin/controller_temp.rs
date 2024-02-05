use embedded_io_async::{Read, Write};
use log::*;
use ucat::controller::*;

use ucat::*;

////////////////////////////////////////
// Paperwork to use serial port async

struct Port(Box<dyn serialport::SerialPort>);

#[derive(Debug)]
struct MockError;

impl embedded_io::Error for MockError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_io::ErrorType for Port {
    type Error = MockError;
}

impl Read for Port {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let n = std::io::Read::read(&mut self.0, buf).map_err(|_| MockError)?;
        Ok(n)
    }
}

impl Write for Port {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let n = std::io::Write::write(&mut self.0, buf).map_err(|_| MockError)?;
        Ok(n)
    }
}

use ucat::device::temp_sensor::TempSensor;

pub fn main() -> anyhow::Result<()> {
    smol::block_on(async {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
            .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
            .init();

        let mut port = Port(serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?);
        port.0
            .set_timeout(std::time::Duration::from_millis(1000000000000))?;

        ///////////////////
        // Wait for ping

        //let _ = wait_for_ping_frame(&mut port).await.unwrap();

        let mut buf = [0u8; MAX_FRAME_SIZE];
        let mut network = Network::<_, 1>::new(&mut port);
        let s1 = network.add(TempSensor {}).await.unwrap();

        ///////////////////
        // Process Update 1

        loop {
            let pdi = network.pdi(&mut buf);
            let pdi = network.cycle(pdi).await.unwrap();
            dbg!(pdi.status(&s1));
        }
    })
}
