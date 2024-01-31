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

pub fn main() -> anyhow::Result<()> {
    smol::block_on(async {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
            .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
            .init();

        let mut port = Port(serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?);
        port.0
            .set_timeout(std::time::Duration::from_millis(1000000000000))?;

        let mut incoming = vec![0; MAX_FRAME_SIZE];

        use ucat::device::led;
        use ucat::device::led::{Color, Light};

        setup_network!(PDI_OFFSETS, (l1, led));

        let mut buf_cmd = vec![0; PDI_SIZE];

        l1.command(&mut buf_cmd, None);

        ///////////////////
        // Wait for ping

        //let _ = wait_for_init_frame(&mut port).await.unwrap();

        ///////////////////
        // Enumerate
        {
            port.write_all(&frame(MessageType::Enumerate, 0, &[])[..])
                .await
                .unwrap();

            wait_for(
                &mut port,
                &frame(MessageType::Enumerate, NUM_DEVICES as i16, &[])[..],
            )
            .await
            .unwrap();
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
            .await
            .unwrap();

            let _f = controller::try_parse_frame(&mut port, &mut incoming)
                .await
                .unwrap();
        }

        ///////////////////
        // Process Update 1
        {
            //l1.command(&mut buf_cmd, Some(&Light::On(Color { r: 10, g: 0, b: 10 })));
            l1.command(&mut buf_cmd, Some(&Light::Off));

            port.write_all(
                &frame(
                    MessageType::ProcessUpdate,
                    group_address.to_wire_address(),
                    &buf_cmd,
                )[..],
            )
            .await
            .unwrap();

            let f = controller::try_parse_frame(&mut port, &mut incoming)
                .await
                .unwrap();

            dbg!(l1.status(f.payload));
        }

        ///////////////////
        // Process Update 2
        {
            //l1.command(&mut buf_cmd, None);
            l1.command(&mut buf_cmd, Some(&Light::On(Color { r: 10, g: 0, b: 10 })));
            port.write_all(
                &frame(
                    MessageType::ProcessUpdate,
                    group_address.to_wire_address(),
                    &buf_cmd,
                )[..],
            )
            .await
            .unwrap();

            let f = controller::try_parse_frame(&mut port, &mut incoming)
                .await
                .unwrap();

            dbg!(l1.status(f.payload));
        }

        loop {
            let n = port.read(&mut incoming[..]).await.unwrap();
            info!("{:?}", &incoming[0..n]);
        }

        Ok(())
    })
}
