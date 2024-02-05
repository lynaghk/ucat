use embedded_io_async::{Read, Write};
use log::*;
use ucat::controller::*;

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

use ucat::device::led::{Color, Led, Light};
use ucat::device::temp_sensor::TempSensor;

pub fn main() -> anyhow::Result<()> {
    smol::block_on(async {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
            .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
            .init();

        // Open (blocking) serial port and wrap in `Port` NewType to implement async Read/Write
        let mut port = Port(serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?);

        // A buffer for the data we send to and receive back from the ucat network.
        let mut buf = [0u8; MAX_FRAME_SIZE];

        // Create a new network with a single device group
        let mut network = Network::<_, 1>::new(&mut port);

        // Add the temperature sensor. This will return an error if the first device isn't a temperature sensor.
        let temp = network.add(TempSensor {}).await.unwrap();

        // Add the LED
        let led = network.add(Led {}).await.unwrap();

        // Loop

        let mut n = 0;
        loop {
            // create a new PDI into which we can write commands
            let mut pdi = network.pdi(&mut buf);

            // Instruct the LED to get brighter
            pdi.command(&led, &Light::On(Color { r: n, g: n, b: n }));
            n = n.wrapping_add(1);

            // send this PDI through the network, delivering commands to all devices in the group and returning their statuses.
            // The returned PDI has been "cycled" and Rust will prevent you from accidentally writing any new commands to it.
            let pdi = network.cycle(pdi).await.unwrap();

            info!("The latest temp reading: {:?}", pdi.status(&temp));
        }
    })
}
