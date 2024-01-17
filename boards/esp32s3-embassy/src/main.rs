#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::ops::Deref;

use bbqueue::{BBBuffer, Consumer, Producer};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, Delay, Rmt, IO,
};
use esp_backtrace as _;
use esp_hal_common::{
    gpio::{GpioPin, Unknown},
    peripherals::UART1,
    Uart, UartRx, UartTx,
};
use static_cell::make_static;

use log::*;

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
        info!("Loop...");

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
static BB: BBBuffer<BUFFER_SIZE> = BBBuffer::new();

#[embassy_executor::task]
async fn upstream_writer(
    mut tx: UartTx<'static, UART1>,
    mut queue: Consumer<'static, BUFFER_SIZE>,
    data_available: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        if let Ok(g) = queue.read() {
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

#[embassy_executor::task]
async fn upstream_reader(
    mut rx: UartRx<'static, UART1>,
    mut queue: Producer<'static, BUFFER_SIZE>,
    data_available: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        let mut g = queue.grant_exact(RX_INTERRUPT_THRESHOLD).unwrap();
        match embedded_io_async::Read::read(&mut rx, &mut g.buf()).await {
            Ok(n) => {
                g.commit(n);
                data_available.signal(());
            }
            Err(e) => error!("RX Error: {:?}", e),
        }
    }
}

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

    //let mut delay = Delay::new(&clocks);
    //delay.delay_ms(20u8);

    let timer_group0 = esp32s3_hal::timer::TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0.timer0);

    // Blinky LED
    {
        let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
        spawner.spawn(led(rmt, io.pins.gpio38)).ok();
    }

    // UART
    {
        let mut uart1 = {
            use esp_hal_common::uart::{config::*, TxRxPins};
            let config = Config {
                baudrate: 115200,
                data_bits: DataBits::DataBits8,
                parity: Parity::ParityNone,
                stop_bits: StopBits::STOP1,
            };

            let pins = TxRxPins::new_tx_rx(
                io.pins.gpio17.into_push_pull_output(),
                io.pins.gpio18.into_floating_input(),
            );

            Uart::new_with_config(peripherals.UART1, config, Some(pins), &clocks)
        };
        uart1
            .set_rx_fifo_full_threshold(RX_INTERRUPT_THRESHOLD as u16)
            .unwrap();

        // TODO: esp-hal doesn't seem to have method to set this UART_TXFIFO_EMPTY_THRHD
        // uart1
        //     .set_tx_fifo_empty_threshold(TX_INTERRUPT_THRESHOLD as u16)
        //     .unwrap();

        // Need this to trigger interrupt after 1 idle byte to unblock any read futures.
        uart1.set_rx_timeout(Some(1)).unwrap();

        let (producer, consumer) = BB.try_split().unwrap();
        let (tx, rx) = uart1.split();
        let data_available = &*make_static!(Signal::new());

        spawner
            .spawn(upstream_reader(rx, producer, &data_available))
            .ok();
        spawner
            .spawn(upstream_writer(tx, consumer, &data_available))
            .ok();

        // let upstream = &*make_static!(Channel::new());
        // let downstream = &*make_static!(Channel::new());
        // spawner.spawn(comms(upstream, downstream)).ok();
    }
}
