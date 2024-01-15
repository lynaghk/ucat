#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

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

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use static_cell::make_static;

// rx_fifo_full_threshold. Interrupt will be triggered after this many bytes are read.
const READ_BUF_SIZE: usize = 8;

#[embassy_executor::task]
async fn writer(mut tx: UartTx<'static, UART1>, signal: &'static Signal<NoopRawMutex, usize>) {
    use core::fmt::Write;
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        //write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        write!(&mut tx, "{}", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, UART1>, signal: &'static Signal<NoopRawMutex, usize>) {
    const MAX_BUFFER_SIZE: usize = 10 * READ_BUF_SIZE + 16;

    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                info!("Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;
                signal.signal(len);
            }
            Err(e) => error!("RX Error: {:?}", e),
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
            .set_rx_fifo_full_threshold(READ_BUF_SIZE as u16)
            .unwrap();

        // Need this to trigger interrupt after 1 idle byte so that read goes through.
        uart1.set_rx_timeout(Some(1)).unwrap();

        let (tx, rx) = uart1.split();

        let signal = &*make_static!(Signal::new());

        spawner.spawn(reader(rx, &signal)).ok();
        spawner.spawn(writer(tx, &signal)).ok();
    }
}
