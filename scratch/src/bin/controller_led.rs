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

use eui::*;

use ucat::device::led::{Color, Led, Light};

pub fn main() -> anyhow::Result<()> {
    #[eui]
    struct Foo {}

    #[eui]
    #[derive(Debug)]
    struct C {
        a: Light,
        b: Light,
    };

    impl Status for Foo {}
    impl Command for C {}

    let (cmd_tx, mut cmd_rx) = channel::<C>(64);
    let (_status_tx, status_rx) = channel::<Foo>(64);

    std::thread::spawn(|| {
        serve_blocking("127.0.0.1:8080", status_rx, cmd_tx);
    });

    smol::block_on(async {
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("debug"))
            .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
            .init();

        let mut port = Port(serialport::new("/dev/tty.usbserial-AQ0445MZ", 115_200).open()?);
        port.0
            .set_timeout(std::time::Duration::from_millis(1000000000000))?;

        let mut buf = [0u8; MAX_FRAME_SIZE];
        let mut network = Network::<_, 1>::new(&mut port);

        let l1 = network.add(Led {}).await.unwrap();
        debug!("led 1 setup");
        let l2 = network.add(Led {}).await.unwrap();
        debug!("led 2 setup");

        ///////////////////
        // Process Update 1
        let mut pdi = network.pdi(&mut buf);
        pdi.command(&l1, &Light::Off);
        pdi.command(&l2, &Light::Off);
        let pdi = network.cycle(pdi).await.unwrap();
        debug!("pdi 1 cycled");
        dbg!(pdi.status(&l1));

        ///////////////////
        // Process Update 2
        let mut pdi = pdi.reset();
        pdi.command(&l1, &Light::On(Color { r: 10, g: 0, b: 0 }));
        pdi.command(&l2, &Light::On(Color { r: 0, g: 0, b: 10 }));
        let pdi = network.cycle(pdi).await.unwrap();
        debug!("pdi 2 cycled");
        dbg!(pdi.status(&l1));

        ///////////////////
        // Loop

        let mut pdi = pdi.reset();

        loop {
            // let n = port.read(&mut incoming[..]).await.unwrap();
            // info!("{:?}", &incoming[0..n]);

            while let Ok(c) = cmd_rx.try_recv() {
                pdi.command(&l1, &c.a);
                pdi.command(&l2, &c.b);
                pdi = network.cycle(pdi).await.unwrap().reset();
            }
        }
    })
}
