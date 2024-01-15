# ucat: lil' PLC-ish framework.

## Install

cargo install --locked --root ".cargo-installed/" --version 0.10.0 espup
cargo install --locked --root ".cargo-installed/" --version 2.1.0 espflash

espup install --targets esp32s3,esp32c3



## Roadmap

### V0

- nostd ucat device library
- proof of concept networking over UART
- control LEDs on three esp32s3 via eui
- device latency test / protocol

## Notes

to generate cargo demo project

cargo generate esp-rs/esp-idf-template cargo

rust-analyzer must be installed *without* rustup, otherwise there will be toolchain errors.

https://github.com/udoprog/bittle for bitsets?


## Log
### Jan 15 - embassy impl
cargo generate esp-rs/esp-template

Got smart led example working pretty quickly. Binary size is *way* smaller than esp-idf, much faster to flash the board. This seems like the way to go.


### Jan 12 - getting started.

Decided to start with UART for simplicity.
SPI is appealing for speed and, e.g., automatic hardware CRC on stm32, but drivers (esp32, rust HAL) are transaction oriented and so would be awkward to poll or otherwise have master keep clocking waiting for response from downstream.

I'll lean on postcard for all of the device commands and statuses and assume the controller has access to types.
That leaves me with just the protocol definition itself.

After startup, how does a device know if it's at the end of the chain?
Could have every device always start broadcasting "I'm here" messages upstream.
When should they stop?
  - send ack from upstream neighbor. Not sure if extra logic here is useful, ultimately controller needs to know.
  - when controller configures the device. probably this.
  
How can we detect if devices are removed / fail during operation?
Controller timeout waiting for returned packet.

As a general principle I think I should keep the device logic as simple as possible so they can be implemented easily on all sorts of various MCUs and other hardware.

Device states:

(Always)
  - forward any messages upstream
  - analyze incoming message and respond accordingly

- RESET
  - broadcast "I'm here" message upstream every 10 ms
  - wait for group assignment and offset
    
- OPERATIONAL
  - group assigned, can respond to ProcessDataUpdate commands


EtherCAT has ~1500B frame that can contain multiple datagrams.
This seems more complicated than I need now, so lets just do a single datagram at a time with 2kB max size:

Frame
  - command - (postcard sized)
  - data length - u16 (should be u11?)
  - data - ??
  - CRC - u32


Also looked into stm32 "passthrough" DMA --- seems like it'll be tricker than I expected, since the DMA peripheral requires a number of bytes to transfer --- so I'd need to setup a buffer and do a bit of CPU intervention to actually count the bytes transferred and handle the remainder.
