#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(non_camel_case_types)]

use bbqueue::{BBBuffer, Consumer, Producer};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp32s3_hal::{clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, Rmt, IO};
use esp_backtrace as _;
use esp_hal_common::{
    gpio::{GpioPin, Unknown},
    peripherals::{UART1, UART2},
    Uart, UartRx, UartTx,
};
use futures::future::FutureExt;
use log::*;
use smart_leds::SmartLedsWrite;
use static_cell::make_static;

use ucat::{device::led::Light, *};

// TODO: seems to be brutal to try and create generic tasks: https://github.com/embassy-rs/embassy/issues/1837
// if I need to share RMT across tasks, may be best to just Peripherals::steal() ...
#[embassy_executor::task]
async fn led(rmt: Rmt<'static>, pin: GpioPin<Unknown, 38>) {
    use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
    use smart_leds::{
        brightness, gamma,
        hsv::{hsv2rgb, Hsv},
        SmartLedsWrite,
    };

    // let peripherals = Peripherals::steal();
    // let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let ch = rmt.channel0;
    let mut led = SmartLedsAdapter::new(ch, pin, smartLedBuffer!(1));

    loop {
        let mut color = Hsv {
            hue: 0,
            sat: 255,
            val: 255,
        };

        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            let data = [hsv2rgb(color)];
            // When sending to the LED, we do a gamma correction first (see smart_leds
            // documentation for details) and then limit the brightness to 10 out of 255 so
            // that the output it's not too bright.
            led.write(brightness(gamma(data.iter().cloned()), 10))
                .unwrap();

            Timer::after(Duration::from_millis(20)).await;
        }
    }
}

// Interrupt will be triggered after these many bytes.
// FIFO buffers are 128 bytes by default. All three UART controllers on ESP32-S3 share 1024 × 8 bits of RAM. As Figure 26-3 illustrates, the RAM is divided into 8 blocks, each having 128 × 8 bits.
const RX_INTERRUPT_THRESHOLD: usize = 4;
//const TX_INTERRUPT_THRESHOLD: usize = 32;

const BUFFER_SIZE: usize = ucat::MAX_FRAME_SIZE * 2;

static UPSTREAM: BBBuffer<BUFFER_SIZE> = BBBuffer::new();
static DOWNSTREAM: BBBuffer<BUFFER_SIZE> = BBBuffer::new();

type UPSTREAM_UART = UART1;
type DOWNSTREAM_UART = UART2;

type QueueProducer = Producer<'static, BUFFER_SIZE>;
type QueueConsumer = Consumer<'static, BUFFER_SIZE>;
type DataAvailableSignal = &'static Signal<NoopRawMutex, ()>;

#[embassy_executor::task]
async fn upstream_reader(
    mut rx: UartRx<'static, UPSTREAM_UART>,
    mut downstream: QueueProducer,
    data_available: DataAvailableSignal,
    // TODO: factor the actual LED logic. I tried passing in a closure here, but...rust.
    rmt: Rmt<'static>,
    pin: GpioPin<Unknown, 38>,
) {
    // TODO: move this into shared schema crate, not ucat.
    use ucat::device::led::{postcard, Color, Led, Light, PDI_WINDOW_SIZE};

    type Status = <Led as Device>::Status;
    type Command = <Led as Device>::Command;

    let mut state = DeviceState::Reset;
    let mut latest_status = [0u8; PDI_WINDOW_SIZE];
    let mut command_buf = [0u8; PDI_WINDOW_SIZE];

    let data_available = || data_available.signal(());

    let mut led = {
        let ch = rmt.channel0;
        let mut led =
            esp_hal_smartled::SmartLedsAdapter::new(ch, pin, esp_hal_smartled::smartLedBuffer!(1));
        move |l: &Light, latest_status: &mut [u8]| {
            use smart_leds::RGB8;
            let c = match l {
                device::led::Light::Off => RGB8 { r: 0, g: 0, b: 0 },
                device::led::Light::On(Color { r, g, b }) => RGB8 {
                    r: *r,
                    g: *g,
                    b: *b,
                },
            };
            let _ = led.write([c].into_iter());
            info!("cmd: {:?}", l);
            let status: &Status = l;
            postcard::to_slice(status, latest_status).unwrap();
        }
    };

    // set initial status
    led(&Light::On(Color { r: 0, g: 10, b: 0 }), &mut latest_status);

    // TODO: use rx idle timeout interrupt rather than embassy timeout here.
    let timeout = Duration::from_millis(500);
    loop {
        let f = handle_frame(
            state.clone(),
            &latest_status,
            &mut command_buf[..],
            &mut rx,
            &mut downstream,
            data_available,
        );

        match embassy_time::with_timeout(timeout, f).await {
            Err(_) => {
                error!("Timed out waiting for message");
                continue;
            }
            Ok(result) => match result {
                Ok(None) => {}
                Ok(Some(action)) => match action {
                    Action::NewState(s) => state = s,
                    Action::NewCommand(cmd) => {
                        if let Some(cmd) = postcard::from_bytes::<Option<Command>>(&cmd).unwrap() {
                            led(&cmd, &mut latest_status);
                        }
                    }
                },

                Err(e) => {
                    error!("Unexpected device error: {e:?}");
                    return;
                }
            },
        }
    }
}

#[embassy_executor::task]
async fn downstream_writer(
    _tx: UartTx<'static, DOWNSTREAM_UART>,
    mut _consumer: QueueConsumer,
    _data_available: DataAvailableSignal,
) {
    todo!();
}

#[embassy_executor::task]
async fn downstream_reader(
    mut rx: UartRx<'static, DOWNSTREAM_UART>,
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
    mut tx: UartTx<'static, UPSTREAM_UART>,
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

// TODO: how can we detect when rx timeout has occured?
// let timeout = UART1::register_block()
//     .int_ena()
//     .read()
//     .rxfifo_tout_int_ena()
//     .bit_is_set();
// info!("timeout: {timeout}");

// #[embassy_executor::task]
// async fn comms(upstream: FrameChannel, downstream: FrameChannel) {
//     // TODO: check if actually at end of chain
//     let mut at_end = true;

//     //downstream.try_receive()

//     //    let mut state = Init;

//     loop {
//         let f = upstream.receive().await;
//         info!("msg  {:?}", f.message);
//     }
// }

#[main]
async fn main(spawner: embassy_executor::Spawner) {
    esp_println::logger::init_logger_from_env();
    info!("Logger is setup");

    let peripherals = Peripherals::take();
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timer_group0 = esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    let boot_delay = Timer::after(Duration::from_millis(40));

    // UART

    let (upstream_uart, downstream_uart) = {
        use esp_hal_common::uart::{config::*, TxRxPins};
        let config = Config {
            baudrate: 115200,
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

    ///////////////////////////////////////
    // Detect if device is at end of chain

    // give upstream neighbor time to boot up, then send it init message
    Timer::after(Duration::from_millis(20)).await;
    embedded_io_async::Write::write(&mut utx, &PING_FRAME)
        .await
        .unwrap();

    // wait as long as possible for our downstream neighbor's message
    boot_delay.await;

    let device_at_end = {
        let mut buf = [0; PING_FRAME.len()];
        match embedded_io_async::Read::read_exact(&mut drx, &mut buf).now_or_never() {
            Some(Ok(())) if buf == PING_FRAME => false,
            _ => true,
        }
    };

    info!("Device at end: {device_at_end}");

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    let pin = io.pins.gpio38;

    if device_at_end {
        spawner.spawn(upstream_reader(urx, up, uda, rmt, pin)).ok();
        spawner.spawn(upstream_writer(utx, uc, uda)).ok();
    } else {
        spawner.spawn(upstream_reader(urx, dp, dda, rmt, pin)).ok();
        spawner.spawn(downstream_writer(dtx, dc, dda)).ok();
        spawner.spawn(downstream_reader(drx, up, uda)).ok();
        spawner.spawn(upstream_writer(utx, uc, uda)).ok();
    }

    // Blinky LED
    // {
    //     let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    //     spawner.spawn(led(rmt, io.pins.gpio38)).ok();
    // }

    info!("Tasks spawned");

    // let upstream = &*make_static!(Channel::new());
    // let downstream = &*make_static!(Channel::new());
    // spawner.spawn(comms(upstream, downstream)).ok();
}
