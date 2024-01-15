#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32s3_hal::{
    clock::ClockControl, embassy, peripherals::Peripherals, prelude::*, Delay, Rmt, IO,
};
use esp_backtrace as _;
use esp_hal_common::gpio::{GpioPin, Unknown};
use esp_println::println;

use log::*;

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
        println!("Loop...");

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

#[main]
async fn main(spawner: Spawner) {
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

    let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
    spawner.spawn(led(rmt, io.pins.gpio38)).ok();
}
