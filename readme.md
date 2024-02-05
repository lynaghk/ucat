# ucat: lil' PLC-ish framework.

## Install

cargo install --locked --root ".cargo-installed/" --version 0.10.0 espup
cargo install --locked --root ".cargo-installed/" --version 2.1.0 espflash

espup install --targets esp32s3,esp32c3

## Protocol invariants

- single frame on network at a time
- pdi_window of all zeros should be interpreted as "no command"
- end device must buffer entire frame in memory so no devices need to worry about simultanious send/recv.

## Roadmap

### V0

- nostd ucat device library
- proof of concept networking over UART
- control LEDs on three esp32s3 via eui
- device latency test / protocol

## Open questions

- dynamic UART baud rate? PING message could contain downstream device's max supported speed. Would need ack from upstream device. Probably too complicated.

## Notes

to generate cargo demo project:

    cargo generate esp-rs/esp-idf-template cargo

rust-analyzer must be installed *without* rustup, otherwise there will be toolchain errors.

https://github.com/udoprog/bittle for bitsets?

esp32s3 has UART_LOOPBACK in hardware --- could use this to avoid DMA entirely!?!

parsing options:
  https://github.com/sharksforarms/deku
  https://docs.rs/binrw/latest/binrw/ - maybe faster than serde? keep retrying on stream.
  
  
## Log
### Feb 2 - API design

I didn't like where the macro complexity was going, so decided to try the controller network setup API again.

If I want to decouple setup/construction from network calls, it's not clear where to store the necessary info.
A central object needs to track the running PDI offsets so it can assign new devices.
But then to initialize the devices later it'll need to know each device's PDI offset so it can tell them.

This means either storing the same info multiple times (in central object and device objects) or making API awkward where you have to pass device objects back in the same order during initialization.

It seems tidier to make the network client mutable and just do async calls during construction.
There'd be unnecessary rework if you re-initialize the network, but I'm not too worried about that.

I might as well have the network hold a reference to the buffer it uses too, so that doesn't need to be passed into every device call.
the devices would just need to get a handle on the network object when reading/writing.

### Jan 26 - API design help

API design for communicating with heterogeneous devices

I'm designing a communications protocol for a chain of heterogeneous embedded devices (different sensors, etc.), but I'm not sure how to design an ergonomic API that's both:

- type-safe
- suitable for no-std usage (i.e., without `dyn` and allocations)

Conceptually things would compile down into something like:

```rust
let mut message = [0u8; A::Length + A::Length + B::Length];

//command device 1 (type A)
A::write_commmand(&mut message[0..1], A::Command::Foo);

//command device 2 (type A)
A::write_commmand(&mut message[1..2], A::Command::Bar);

//command device 3 (type B)
B::write_commmand(&mut message[2..4], B::Command::Baz);

let response = send(&message);

//read statuses
let d1: A::Status = A::read_status(&mut response[0..1]);
let d2: A::Status = A::read_status(&mut response[1..2]);
let d3: B::Status = B::read_status(&mut response[2..4]);
```

The topology (here, a chain of `[A, A, B]`) is known at compile-time, as are the status/command sizes.
Is it possible in Rust to define the topology once (in a function/macro invocation or data structure) and handle all of the buffer size and offset calculations behind the scenes?
Ideally the user-facing code would look something like:

```rust
let mut network = network![A, A, B];
network[0].write(A::Command::Foo);
network[1].write(A::Command::Bar);
network[2].write(B::Command::Baz);
send(&network.message);

let d1: A::Status = network[0].status;
let d2: A::Status = network[1].status;
let d3: B::Status = network[2].status;
```

I've tried to implement with

```rust
trait Device {
    type Message;
    const MESSAGE_LENGTH: usize;
}
```

but immediately run into object safety issues so figured I should ask for help to see if there are other patterns I should be exploring to solve this. Thanks!

, but haven't been able to get anywhere close to a succient API 



### Jan 19 - embassy size
wondered if embassy would even fit on smaller stm32g0.
as of 871d82de their blinky example is 

but removing defmt and adding to cargo.toml

[profile.release]
debug = false
lto = true
opt-level = "s"
incremental = false
codegen-units = 1

and to config.toml

[unstable]
build-std = ["core"]
build-std-features = ["panic_immediate_abort"]

cargo bloat reports 3.3kB, which seems sufficiently small to me.

Though the stmg4 hal blinky example (spin loop) is just 800 bytes.

the usb_serial g0 example is 14.6KiB. cutting it close, but may be workable.

### Jan 18 - handling the end.

how can the device know when it's at the end of the chain and need to loop back?


### Jan 17 - wire protocol

I keep bouncing between:

- fixed size, deterministic protocol that can be (tediously) implemented by hand
- leaning on rust/postcard and "hacking" streaming by repeatedly trying to deserialize, e.g., message header.

the former seems, well, tedious and hard to debug when I inevitably get things wrong.
the latter feels like it'd make streaming potentially clunky/slow and make it harder for others to interop.

In particular, the latency would seem to be nondeterministic if CPU spends a lot of time trying to deserialize a header.

Alternatively I could "waste" some time to reduce jitter, and not try to deserialize until maxsize bytes have arrived (or end-of-packet).

I think I should do my own streaming for now, since at least I'll learn some stuff and have a point of comparison later.

One other question is how to handle CRC errors --- with ethercat the ethernet hardware handles this.
But in my case, since the CRC comes at the end, how do I actually act on it?
I guess I have to "buffer" the command and only "run" it once CRC checks out.

that doesn't help with messages that I'm already streaming along, but it will at least eliminate problems with side effects like setting device outputs.


### Jan 17 - continued uart echoing

Guess it's time to learn about futures and esp32 interrupts more myself.
Here's the plan for implementing echo:

- use bbqueue to pass data from rx to tx
- set fifo empty/full sizes to be half the buffer sizes
- rx task uses embedded io read
- tx task reads up to tx fifo length from bbqueue, writes it and awaits fifo half empty interrupt to do it again --- ohh, I can use embedded async write_all for this too.

### Jan 16 - run into embassy esp32 uart gaps

esp32 has a UART Idle interrupt.
I was hoping to use this for framing my messages.

However, on esp32 embedded_io_async's read method blocks until a UART interrupt AND at least one byte is ready.

asked on matrix https://matrix.to/#/!LdaNPfUfvefOLewEIM:matrix.org/$yNkeQFSt-U57Kexv64v5TzP1_M0bFTHbtpO8TLTqDdM?via=matrix.org&via=mozilla.org&via=arcticfoxes.net

Should I avoid the embedded_io_async methods and construct my own UartRxFuture to await and call drain_fifo myself? Or is there an entirely different way to go about this that would be nicer?


esp-hal/esp-hal-common/src/uart.rs at a23d6a05a6ae91c4a3c436b25919ede86b35683a · esp-rs/esp-hal - GitHub
no_std Hardware Abstraction Layers for ESP32 microcontrollers - esp-rs/esp-hal
lynaghk
Ah, UartRxFuture is not public, so I guess that's not an option. Is there another way I could accomplish this using the existing esp-hal, or do I need to try implementing my own future using interrupt handlers?
lynaghk
Looking at this PR and having a harder think, I don't think I'll get stuck; the idle interrupt will wake the read future and it'll return data. That said, I don't see a way for my task to know if it was the idle interrupt or the max fifo size interrupt that polled it. I was hoping to use the idle frame for, uh, framing. I guess I could check the length of the returned data and see if it's equal to (or larger than?) the max fifo size. That still seems unreliable in the case where the message is a multiple of max fifo size, though.


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


## Lessons / changes 
- Started with PDI_WINDOW_SIZE as const, but made dynamic to support runtime configuration. Rust consts too annoying to shuffle around everywhere.


## Thanks
Thanks to James Waples for helpful discussions on protocol and API design.
