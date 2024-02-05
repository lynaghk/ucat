#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use bbqueue::{BBBuffer, Consumer, Producer};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    clock::ClockControl,
    embassy,
    peripherals::{Peripherals, UART1, UART2},
    prelude::*,
    Uart, UartRx, UartTx, IO,
};
use esp_backtrace as _;
use esp_println::dbg;

use futures::future::FutureExt;
use log::*;
use static_cell::make_static;
use ucat::*;

// Interrupt will be triggered after these many bytes.
// FIFO buffers are 128 bytes by default. All three UART controllers on ESP32-S3 share 1024 × 8 bits of RAM. As Figure 26-3 illustrates, the RAM is divided into 8 blocks, each having 128 × 8 bits.
const RX_INTERRUPT_THRESHOLD: usize = 4;
//const TX_INTERRUPT_THRESHOLD: usize = 32;

const BUFFER_SIZE: usize = ucat::MAX_FRAME_SIZE * 2;

static UPSTREAM: BBBuffer<BUFFER_SIZE> = BBBuffer::new();
static DOWNSTREAM: BBBuffer<BUFFER_SIZE> = BBBuffer::new();

type UpstreamUart = UART1;
type DownstreamUart = UART2;

type QueueProducer = Producer<'static, BUFFER_SIZE>;
type QueueConsumer = Consumer<'static, BUFFER_SIZE>;
type DataAvailableSignal = &'static Signal<NoopRawMutex, ()>;
type StatusSignal = &'static Signal<NoopRawMutex, Status>;
type CommandSignal = &'static Signal<NoopRawMutex, Command>;
type Status = <DeviceType as Device>::Status;
type Command = <DeviceType as Device>::Command;

// TODO: move this into shared schema crate, not ucat.
#[cfg(feature = "led")]
use ucat::device::led::{postcard, Color, Led as DeviceType, Light, PDI_WINDOW_SIZE};

#[cfg(feature = "temp-sensor")]
use ucat::device::temp_sensor::{postcard, TempCelcius, TempSensor as DeviceType, PDI_WINDOW_SIZE};

#[embassy_executor::task]
async fn upstream_reader(
    mut rx: UartRx<'static, UpstreamUart>,
    mut downstream: QueueProducer,
    data_available: DataAvailableSignal,
    status: StatusSignal,
    command: CommandSignal,
) {
    let mut state = DeviceState::Reset;
    let mut status_buf = [0u8; PDI_WINDOW_SIZE];
    let mut command_buf = [0u8; PDI_WINDOW_SIZE];
    let mut last_status = status.wait().await;

    let data_available = || data_available.signal(());

    // TODO: use rx idle timeout interrupt rather than embassy timeout here.
    let timeout = Duration::from_millis(500);
    loop {
        // TODO: refactor so handle_frame can use most recent status buffer when it's writing out to PDI?
        // Status gets "stale" while waiting for PDI.
        if status.signaled() {
            last_status = status.wait().await;
        }
        postcard::to_slice(&last_status, &mut status_buf).unwrap();

        let f = handle_frame(
            state.clone(),
            &status_buf,
            &mut command_buf,
            &mut rx,
            &mut downstream,
            data_available,
        );

        let Ok(result) = embassy_time::with_timeout(timeout, f).await else {
            error!("Timed out waiting for message");
            continue;
        };

        match result {
            Err(e) => {
                error!("Unexpected device error: {e:?}");
                return;
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
}

#[embassy_executor::task]
async fn downstream_writer(
    _tx: UartTx<'static, DownstreamUart>,
    mut _consumer: QueueConsumer,
    _data_available: DataAvailableSignal,
) {
    todo!();
}

#[embassy_executor::task]
async fn downstream_reader(
    mut rx: UartRx<'static, DownstreamUart>,
    mut upstream: QueueProducer,
    data_available: DataAvailableSignal,
) {
    loop {
        let mut g = upstream.grant_exact(RX_INTERRUPT_THRESHOLD).unwrap();
        match embedded_io_async::Read::read(&mut rx, &mut g.buf()).await {
            Ok(n) => {
                g.commit(n);
                data_available.signal(());
            }
            Err(e) => error!("Downstream RX Error: {:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn upstream_writer(
    mut tx: UartTx<'static, UpstreamUart>,
    mut upstream: QueueConsumer,
    data_available: DataAvailableSignal,
) {
    loop {
        if let Ok(g) = upstream.read() {
            match embedded_io_async::Write::write(&mut tx, &g.buf()).await {
                Ok(n) => g.release(n),
                Err(e) => error!("TX Error: {:?}", e),
            }
        } else {
            data_available.wait().await;
            data_available.reset();
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

    let ping_tx_delay = Timer::after(Duration::from_millis(STARTUP_DELAY_MILLIS as u64 / 2));
    let ping_rx_delay = Timer::after(Duration::from_millis(STARTUP_DELAY_MILLIS as u64));

    //////////////////////////////
    // UART setup

    let (upstream_uart, downstream_uart) = {
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

    let (mut utx, urx) = upstream_uart.split();
    let (dtx, mut drx) = downstream_uart.split();

    let uda = &*make_static!(Signal::new());
    let dda = &*make_static!(Signal::new());
    let (up, uc) = UPSTREAM.try_split().unwrap();
    let (dp, dc) = DOWNSTREAM.try_split().unwrap();

    let status_signal: StatusSignal = &*make_static!(Signal::new());
    let command_signal: CommandSignal = &*make_static!(Signal::new());

    ///////////////////////////////////////
    // Detect if device is at end of chain

    // give upstream neighbor time to boot up, then send it init message
    ping_tx_delay.await;
    embedded_io_async::Write::write(&mut utx, &PING_FRAME)
        .await
        .unwrap();

    // wait as long as possible for our downstream neighbor's message
    ping_rx_delay.await;

    let device_at_end = {
        let mut buf = [0; PING_FRAME.len()];
        match embedded_io_async::Read::read_exact(&mut drx, &mut buf).now_or_never() {
            Some(Ok(())) if buf == PING_FRAME => false,
            _ => true,
        }
    };

    info!("Device at end: {device_at_end}");

    /////////////////////////
    // Spawn comms tasks

    if device_at_end {
        spawner
            .spawn(upstream_reader(urx, up, uda, status_signal, command_signal))
            .ok();
        spawner.spawn(upstream_writer(utx, uc, uda)).ok();
    } else {
        spawner
            .spawn(upstream_reader(urx, dp, dda, status_signal, command_signal))
            .ok();
        spawner.spawn(downstream_writer(dtx, dc, dda)).ok();
        spawner.spawn(downstream_reader(drx, up, uda)).ok();
        spawner.spawn(upstream_writer(utx, uc, uda)).ok();
    }

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
