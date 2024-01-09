# ucat: lil' PLC-ish framework.

## Install

cargo install --locked --root ".cargo-installed/" --version 0.10.0 espup
cargo install --locked --root ".cargo-installed/" --version 2.1.0 espflash

espup install --targets esp32s3,esp32c3

## Notes

to generate cargo demo project

cargo generate esp-rs/esp-idf-template cargo

rust-analyzer must be installed *without* rustup, otherwise there will be toolchain errors.


