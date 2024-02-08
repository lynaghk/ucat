#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::{Peripherals, UART1, UART2},
    prelude::*,
    Uart, IO,
};
use esp_backtrace as _;

use log::*;
use static_cell::make_static;
use ucat::*;

// TODO: move this into shared schema crate, not ucat.
#[cfg(feature = "led")]
use ucat::device::led::{postcard, Color, Led as DeviceType, Light, PDI_WINDOW_SIZE};

#[cfg(feature = "temp-sensor")]
use ucat::device::temp_sensor::{postcard, TempCelcius, TempSensor as DeviceType, PDI_WINDOW_SIZE};

// Interrupt will be triggered after these many bytes.
// FIFO buffers are 128 bytes by default. All three UART controllers on ESP32-S3 share 1024 × 8 bits of RAM. As Figure 26-3 illustrates, the RAM is divided into 8 blocks, each having 128 × 8 bits.
const RX_INTERRUPT_THRESHOLD: usize = 4;
//const TX_INTERRUPT_THRESHOLD: usize = 32;

type UpstreamUart = UART1;
type DownstreamUart = UART2;

type StatusSignal = &'static Signal<NoopRawMutex, Status>;
type CommandSignal = &'static Signal<NoopRawMutex, Command>;
type Status = <DeviceType as Device>::Status;
type Command = <DeviceType as Device>::Command;

#[embassy_executor::task]
async fn comms(
    mut upstream: Uart<'static, UpstreamUart>,
    mut downstream: Uart<'static, DownstreamUart>,
    status: StatusSignal,
    command: CommandSignal,
) {
    let mut state = DeviceState::Reset;
    let mut status_buf = [0u8; PDI_WINDOW_SIZE];
    let mut command_buf = [0u8; PDI_WINDOW_SIZE];
    let mut frame_buf = [0u8; MAX_FRAME_SIZE];

    let mut last_status = status.wait().await;

    macro_rules! rx_available {
        ($uart: ident) => {{

            // This seems to return the right count for upstream uart, but always has 1 for the downstream uart.
            // If that's noise, I'm not sure why the read doesn't clear it when it times out and the future gets cancelled.
            let count: u16 = $uart::register_block()
                .status()
                .read()
                .rxfifo_cnt()
                .bits()
                .into();
            //info!("rx {count}");
            count != 0

            // $uart::register_block()
            //     .fsm_status()
            //     .read()
            //     .st_urx_out()
            //     .bits()
            //     != 0x0u8


        }};
    }

    let upstream_rx_available = || rx_available!(UpstreamUart);
    let downstream_rx_available = || rx_available!(DownstreamUart);

    // TODO: use rx idle timeout interrupt rather than embassy timeout here.
    let timeout = Duration::from_millis(500);
    //let (mut upstream_tx, mut upstream_rx) = upstream.split();

    loop {
        // TODO: refactor so handle_frame can use most recent status buffer when it's writing out to PDI?
        // Status gets "stale" while waiting for PDI.
        if status.signaled() {
            last_status = status.wait().await;
        }
        postcard::to_slice(&last_status, &mut status_buf).unwrap();

        // TODO: use async select with cancel-safe ReadReady traits on uarts so we can
        // (or just use DMA for upstream forwarding, if possible)

        if upstream_rx_available() {
            let f = handle_frame(
                state.clone(),
                &status_buf,
                &mut command_buf,
                &mut frame_buf,
                &mut upstream_rx,
                &mut upstream_tx,
                &mut downstream,
            );

            let Ok(result) = embassy_time::with_timeout(timeout, f).await else {
                error!("Timed out waiting for upstream message");
                continue;
            };

            match result {
                Err(e) => {
                    error!("Unexpected device error: {e:?}");
                    continue;
                }
                Ok(None) => {}
                Ok(Some(action)) => match action {
                    Action::NewState(s) => state = s,
                    Action::NewCommand(cmd) => {
                        if let Some(cmd) = postcard::from_bytes::<Option<Command>>(&cmd).unwrap() {
                            command.signal(cmd)
                        }
                    }
                },
            }
        }

        if downstream_rx_available() {
            info!("forwarding upstream...");
            let f = read_frame(&mut frame_buf, &mut downstream);
            let Ok(frame) = embassy_time::with_timeout(timeout, f).await else {
                error!("Timed out waiting for downstream message");
                continue;
            };

            embedded_io_async::Write::write_all(&mut upstream_tx, frame.unwrap())
                .await
                .unwrap();
            info!("forwarded upstream");
        }

        // TODO: investigate likely esp hal bug: when downstream_rx_available() I can blocking read a byte but future read never resolves.
        // while upstream_rx_available() {
        //     info!("upstream: {:?}", upstream.read().unwrap());
        // }
        // while downstream_rx_available() {
        //     info!("reading downstream...");
        //     //info!("downstream: {:?}", downstream.read().unwrap());

        //     let mut buf = [0; 1];
        //     embedded_io_async::Read::read_exact(&mut downstream, &mut buf)
        //         .await
        //         .unwrap();
        //     info!("downstream: {:?}", buf);
        // }

        // yield so this loop doesn't starve CPU.
        yield_now().await;
    }
}

#[main]
async fn main(spawner: embassy_executor::Spawner) {
    esp_println::logger::init_logger_from_env();
    info!("Logger is setup");

    let peripherals = Peripherals::take();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer_group0 = esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    //////////////////////////////
    // UART setup

    let (upstream, downstream) = {
        use esp32s3_hal::uart::{config::*, TxRxPins};
        let config = Config {
            baudrate: BAUD_RATE as u32,
            data_bits: DataBits::DataBits8,
            parity: Parity::ParityNone,
            stop_bits: StopBits::STOP1,
        };

        let mut us = Uart::new_with_config(
            peripherals.UART1,
            config,
            Some(TxRxPins::new_tx_rx(
                io.pins.gpio17.into_push_pull_output(),
                io.pins.gpio18.into_floating_input(),
            )),
            &clocks,
        );

        let mut ds = Uart::new_with_config(
            peripherals.UART2,
            config,
            Some(TxRxPins::new_tx_rx(
                io.pins.gpio19.into_push_pull_output(),
                io.pins.gpio20.into_floating_input(),
            )),
            &clocks,
        );

        // Need this to trigger interrupt after 1 idle byte to unblock any read futures.
        us.set_rx_timeout(Some(1)).unwrap();
        ds.set_rx_timeout(Some(1)).unwrap();

        us.set_rx_fifo_full_threshold(RX_INTERRUPT_THRESHOLD as u16)
            .unwrap();
        ds.set_rx_fifo_full_threshold(RX_INTERRUPT_THRESHOLD as u16)
            .unwrap();

        // TODO: esp-hal doesn't seem to have method to set this UART_TXFIFO_EMPTY_THRHD
        // uart1
        //     .set_tx_fifo_empty_threshold(TX_INTERRUPT_THRESHOLD as u16)
        //     .unwrap();

        (us, ds)
    };

    let status_signal: StatusSignal = &*make_static!(Signal::new());
    let command_signal: CommandSignal = &*make_static!(Signal::new());

    /////////////////////////
    // Spawn comms task

    spawner
        .spawn(comms(upstream, downstream, status_signal, command_signal))
        .ok();

    /////////////////////////
    // Device task
    // (Too painful to write out types for async task, so just run here at end of main.)

    #[cfg(feature = "led")]
    {
        use esp32s3_hal::Rmt;
        use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
        use smart_leds::SmartLedsWrite;
        let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
        let pin = io.pins.gpio38;

        let mut led = {
            let ch = rmt.channel0;
            let mut led = SmartLedsAdapter::new(ch, pin, smartLedBuffer!(1));
            move |l: Light| {
                use smart_leds::RGB8;
                let c = match l {
                    device::led::Light::Off => RGB8 { r: 0, g: 0, b: 0 },
                    device::led::Light::On(Color { r, g, b }) => RGB8 { r, g, b },
                };
                let _ = led.write([c].into_iter());
                info!("cmd: {:?}", &l);
                status_signal.signal(l);
            }
        };

        // Set default light value
        led(Light::On(Color { r: 0, g: 10, b: 0 }));

        loop {
            led(command_signal.wait().await);
        }
    }

    #[cfg(feature = "temp-sensor")]
    {
        use esp32s3_hal::{
            dma::DmaPriority,
            dma_descriptors,
            gdma::Gdma,
            spi::{
                master::{prelude::*, Spi},
                SpiMode,
            },
        };

        let dma = Gdma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);

        let mut vcc = io.pins.gpio13.into_push_pull_output();
        vcc.set_high().unwrap();

        let mut gnd = io.pins.gpio14.into_push_pull_output();
        gnd.set_low().unwrap();

        let mut spi = Spi::new(peripherals.SPI3, 4u32.MHz(), SpiMode::Mode0, &clocks)
            .with_miso(io.pins.gpio10)
            .with_cs(io.pins.gpio11)
            .with_sck(io.pins.gpio12)
            .with_dma(dma_channel.configure(
                false,
                &mut descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ));

        let mut buf = [0u8; 2];

        // https://www.analog.com/media/en/technical-documentation/data-sheets/MAX6675.pdf
        let conversion_time = Duration::from_millis(220);

        // wait for initial conversion so we don't read garbage
        Timer::after(conversion_time).await;

        loop {
            // Temp sensor has no commands
            //let _last_command = command_signal.wait().await;

            embedded_hal_async::spi::SpiBus::read(&mut spi, &mut buf)
                .await
                .unwrap();
            Timer::after(Duration::from_millis(500)).await;

            // https://www.analog.com/media/en/technical-documentation/data-sheets/MAX6675.pdf
            let temp_c = 0.25 * (u16::from_be_bytes(buf) >> 3) as f32;
            debug!("temp: {}; bytes: {:?}", temp_c, buf);
            status_signal.signal(TempCelcius(temp_c));
        }
    }
}
