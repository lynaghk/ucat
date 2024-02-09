#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_futures::select::select3;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_io_async::Write;
use esp32s3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::{Peripherals, UART1, UART2},
    prelude::*,
    Uart, IO,
};
use esp_backtrace as _;

#[allow(unused)]
use esp_println::dbg;

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
    upstream: Uart<'static, UpstreamUart>,
    downstream: Uart<'static, DownstreamUart>,
    status: StatusSignal,
    command: CommandSignal,
) {
    let mut state = DeviceState::Reset;
    let mut last_status = status.wait().await;

    let mut status_buf = [0u8; PDI_WINDOW_SIZE];
    let mut command_buf = [0u8; PDI_WINDOW_SIZE];

    // Ideally we would only need one buffer since there's only ever one message on the network at a time.
    // However, I couldn't find a nice way to convince the borrow checker of this --- I'd love to simply await upstream and downstream read futures, but they can't share a mutable reference.
    let mut upstream_rx_buf = [0u8; MAX_FRAME_SIZE];
    let mut downstream_rx_buf = [0u8; MAX_FRAME_SIZE];

    let (mut upstream_tx, mut upstream_rx) = upstream.split();
    let (mut downstream_tx, mut downstream_rx) = downstream.split();

    // sometimes noise gets into the uart buffers after startup, so just dump them.
    while upstream_rx.read().is_ok() {}
    while downstream_rx.read().is_ok() {}

    loop {
        // Serialize latest status message
        if status.signaled() {
            last_status = status.wait().await;
        }
        postcard::to_slice(&last_status, &mut status_buf).unwrap();

        // TODO: use rx idle timeout interrupt rather than embassy timeout here.
        let timeout = Timer::after_millis(500);
        let upstream_frame = read_frame(&mut upstream_rx_buf, &mut upstream_rx);
        let downstream_frame = read_frame(&mut downstream_rx_buf, &mut downstream_rx);
        use embassy_futures::select::Either3::*;

        match select3(timeout, upstream_frame, downstream_frame).await {
            First(_) => {
                error!("Timed out waiting for messages");
                continue;
            }
            Second(x) => match x {
                Err(e) => {
                    error!("upstream frame error: {e:?}");
                    continue;
                }
                Ok(upstream_frame) => {
                    let result =
                        handle_frame(&mut state, &status_buf, &mut command_buf, upstream_frame);

                    let Ok((reply, maybe_command)) = result else {
                        let e = result.unwrap_err();
                        error!("Error handling frame: {e:?}");
                        continue;
                    };

                    match reply {
                        Reply::Upstream(f) => upstream_tx.write_all(f).await.unwrap(),
                        Reply::Downstream(f) => downstream_tx.write_all(f).await.unwrap(),
                    }

                    if let Some(cmd) = maybe_command {
                        if let Some(cmd) = postcard::from_bytes::<Option<Command>>(&cmd).unwrap() {
                            command.signal(cmd)
                        }
                    }
                }
            },
            Third(x) => match x {
                Err(e) => {
                    error!("downstream frame error: {e:?}");
                    continue;
                }
                Ok(downstream_frame) => upstream_tx.write_all(downstream_frame).await.unwrap(),
            },
        }
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
                io.pins.gpio13.into_push_pull_output(),
                io.pins.gpio12.into_floating_input(),
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
