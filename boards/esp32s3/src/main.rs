use esp_idf_svc::hal::peripherals::Peripherals;
use log::*;
use std::default::Default;
use std::thread;
use std::time::Duration;

pub enum Light {
    Off,
    On(Color),
}

pub struct Color {
    r: u8,
    g: u8,
    b: u8,
}
use anyhow::Result;
fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Hello, world!");

    let ps = Peripherals::take().unwrap();
    let pins = ps.pins;

    unsafe {
        use std::ptr::null;

        let config = esp_idf_sys::tinyusb_config_t {
            // string_descriptor: NULL,
            // string_descriptor_count: todo!(),
            external_phy: false,
            // configuration_descriptor: null(),
            // self_powered: false,
            ..Default::default()
        };

        esp_idf_sys::tinyusb_driver_install(&config);

        // https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32s3/api-reference/peripherals/usb_device.html
        // TODO: https://github.com/espressif/esp-idf/blob/v5.1.2/examples/peripherals/usb/device/tusb_serial_device/main/tusb_serial_device_main.c
    }

    let mut led = {
        use esp_idf_svc::hal::rmt;
        use esp_idf_svc::hal::rmt::*;

        //WS2812 data format
        //0xff_00_00 blue
        //0x00_ff_00 red
        //0x00_00_ff green

        let mut tx = TxRmtDriver::new(
            ps.rmt.channel0,
            pins.gpio38,
            &config::TransmitConfig::new().clock_divider(1),
        )?;

        fn ns(nanos: u64) -> Duration {
            Duration::from_nanos(nanos)
        }
        let ticks_hz = tx.counter_clock()?;
        let t0h = Pulse::new_with_duration(ticks_hz, rmt::PinState::High, &ns(350))?;
        let t0l = Pulse::new_with_duration(ticks_hz, rmt::PinState::Low, &ns(800))?;
        let t1h = Pulse::new_with_duration(ticks_hz, rmt::PinState::High, &ns(700))?;
        let t1l = Pulse::new_with_duration(ticks_hz, rmt::PinState::Low, &ns(600))?;

        move |l: &Light| {
            let brg = match *l {
                Light::Off => 0,
                Light::On(Color { r, g, b }) => {
                    ((b as u32) << 16) | ((r as u32) << 8) | ((g as u32) << 0)
                }
            };

            let mut signal = FixedLengthSignal::<24>::new();
            for i in 0..24 {
                let bit = 2_u32.pow(i) & brg != 0;
                let (high_pulse, low_pulse) = if bit { (t1h, t1l) } else { (t0h, t0l) };
                signal.set(i as usize, &(high_pulse, low_pulse)).unwrap();
            }
            tx.start(signal).unwrap();
        }
    };

    led(&Light::On(Color { r: 255, g: 0, b: 0 }));

    loop {
        thread::sleep(Duration::from_millis(100));
    }
}
