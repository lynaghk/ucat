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
use static_cell::make_static;

use ucat::*;

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

struct Queue<T> {
    bbq: T,
    data_available: &'static Signal<NoopRawMutex, ()>,
}

type QueueProducer = Queue<Producer<'static, BUFFER_SIZE>>;
type QueueConsumer = Queue<Consumer<'static, BUFFER_SIZE>>;

#[embassy_executor::task]
async fn upstream_reader(mut rx: UartRx<'static, UPSTREAM_UART>, mut output: QueueProducer) {
    // TODO: actual group address
    let group_address = 1;

    'parser: loop {
        let mut digest = CRC.digest();

        // Ahh, rust, where the types are so complicated you resort to abstraction via syntax macros.
        macro_rules! commit {
            ($grant:expr, $buf: expr) => {
                let n = $buf.len();
                digest.update($buf);
                $grant.commit(n);
                output.data_available.signal(());
            };

            (digest) => {
                let mut g = output.bbq.grant_exact(4).unwrap();
                g.buf().copy_from_slice(&digest.finalize().to_le_bytes());
                g.commit(4);
                output.data_available.signal(());
                digest = CRC.digest();
            };
        }

        let mut g = output.bbq.grant_exact(3).unwrap();
        let mut buf = g.buf();

        if let Err(e) = embedded_io_async::Read::read_exact(&mut rx, &mut buf).await {
            error!("Upstream RX Error: {:?}", e);
            continue;
        }

        match MessageType::try_from(buf[0]) {
            Err(e) => {
                error!("Bad message type: {}", e);
                continue 'parser;
            }

            Ok(t) => {
                use MessageType::*;
                let mut address = i16::from_le_bytes([buf[1], buf[2]]);

                // increment address for device-oriented messages
                if matches!(t, Enumerate | Initialize | Identify | Query) {
                    address += 1;
                    buf[1..3].copy_from_slice(&address.to_le_bytes())
                }

                commit!(g, buf);

                match (t, address) {
                    (Enumerate, _) => {
                        // no work for us to do aside from forwarding the rest of the message

                        let mut g = output.bbq.grant_exact(2).unwrap();
                        let mut buf = g.buf();
                        if let Err(e) = embedded_io_async::Read::read_exact(&mut rx, &mut buf).await
                        {
                            error!("Upstream RX Error: {:?}", e);
                            continue 'parser;
                        }
                        commit!(g, buf);

                        // Read CRC
                        let mut buf = [0u8; 4];
                        if let Err(e) = embedded_io_async::Read::read_exact(&mut rx, &mut buf).await
                        {
                            error!("Upstream RX Error: {:?}", e);
                            continue 'parser;
                        }
                        info!("{:?}", buf);
                        let actual = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
                        let expected = digest.clone().finalize();
                        if actual != expected {
                            error!("Bad message CRC, expected {expected} got {actual}");
                        }

                        // well, too late to do anything about it now. send out the one we expected.
                        commit!(digest);

                        info!("enumerated");
                    }
                    (Initialize, 0) => todo!(),
                    (Identify, 0) => todo!(),
                    (Query, 0) => todo!(),
                    (ProcessUpdate, addr) if addr == group_address => todo!(),
                    _ => todo!(),
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn downstream_writer(_tx: UartTx<'static, DOWNSTREAM_UART>, _downstream: QueueConsumer) {
    todo!();
}

#[embassy_executor::task]
async fn downstream_reader(mut rx: UartRx<'static, DOWNSTREAM_UART>, mut upstream: QueueProducer) {
    loop {
        let mut g = upstream.bbq.grant_exact(RX_INTERRUPT_THRESHOLD).unwrap();
        match embedded_io_async::Read::read(&mut rx, &mut g.buf()).await {
            Ok(n) => {
                g.commit(n);
                upstream.data_available.signal(());
            }
            Err(e) => error!("Downstream RX Error: {:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn upstream_writer(mut tx: UartTx<'static, UPSTREAM_UART>, mut upstream: QueueConsumer) {
    loop {
        if let Ok(g) = upstream.bbq.read() {
            match embedded_io_async::Write::write(&mut tx, &g.buf()).await {
                Ok(n) => g.release(n),
                Err(e) => error!("TX Error: {:?}", e),
            }
        } else {
            upstream.data_available.wait().await;
            upstream.data_available.reset();
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

    let make_queue = |bbq: &'static BBBuffer<BUFFER_SIZE>, sig| {
        let (producer, consumer) = bbq.try_split().unwrap();
        (
            QueueProducer {
                bbq: producer,
                data_available: sig,
            },
            QueueConsumer {
                bbq: consumer,
                data_available: sig,
            },
        )
    };

    let (mut utx, urx) = upstream_uart.split();
    let (dtx, mut drx) = downstream_uart.split();

    let (up, uc) = make_queue(&UPSTREAM, &*make_static!(Signal::new()));
    let (dp, dc) = make_queue(&DOWNSTREAM, &*make_static!(Signal::new()));

    ///////////////////////////////////////
    // Detect if device is at end of chain

    // give upstream neighbor time to boot up, then send it init message
    Timer::after(Duration::from_millis(20)).await;
    embedded_io_async::Write::write(&mut utx, &INIT_FRAME)
        .await
        .unwrap();

    // wait as long as possible for our downstream neighbor's message
    boot_delay.await;

    let device_at_end = {
        let mut buf = [0; INIT_FRAME.len()];
        match embedded_io_async::Read::read_exact(&mut drx, &mut buf).now_or_never() {
            Some(Ok(())) if buf == INIT_FRAME => false,
            _ => true,
        }
    };

    info!("Device at end: {device_at_end}");

    if device_at_end {
        spawner.spawn(upstream_reader(urx, up)).ok();
        spawner.spawn(upstream_writer(utx, uc)).ok();
    } else {
        spawner.spawn(upstream_reader(urx, dp)).ok();
        spawner.spawn(downstream_writer(dtx, dc)).ok();
        spawner.spawn(downstream_reader(drx, up)).ok();
        spawner.spawn(upstream_writer(utx, uc)).ok();
    }

    // Blinky LED
    {
        let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
        spawner.spawn(led(rmt, io.pins.gpio38)).ok();
    }

    info!("Initialized");

    // let upstream = &*make_static!(Channel::new());
    // let downstream = &*make_static!(Channel::new());
    // spawner.spawn(comms(upstream, downstream)).ok();
}
