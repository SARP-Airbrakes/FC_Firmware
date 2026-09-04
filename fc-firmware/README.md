# `fc-firmware`
This crate contains the firmware for the airbrakes flight computer, which
includes the USB-based CLI (see [`usb.rs`](src/usb.rs) and
[`cli.rs`](src/cli.rs)), the flight log ([`log.rs`](src/log.rs)) and sensor and
IC management ([`sensor.rs`](src/sensor.rs) and [`memory.rs`](src/memory.rs)).
There are also integrated tests using
[`embedded-test`](https://docs.rs/embedded-test/latest/embedded_test/).

## Building
This requires Rust nightly and the `thumbv7em-none-eabihf` target.
```
rustup install nightly
rustup target add thumbv7em-none-eabihf
```

From there, the firmware can be built as normal with `cargo build`. The firmware
also uses [`defmt`](https://defmt.ferrous-systems.com/introduction), which at compile-time decides what log levels are compiled
into the firmware. This can be configured with the environment variable
`DEFMT_LOG`. For example: 
```
DEFMT_LOG=trace cargo build
```

## Running
This crate uses [`probe-rs`](https://probe.rs) to interface with and debug the the flight computer
hardware. To use it, connect to the flight computer hardware via an ST-Link, and then use one of two ways to run the firmware:

1. There are Visual Studio Code launch configurations present in
the root directory of the repository for usage with the [`probe-rs` VSCode plugin](https://marketplace.visualstudio.com/items?itemName=probe-rs.probe-rs-debugger) (strongly recommended).
2. The `cargo` runner has been configured to use `probe-rs`. First, make sure
you are in the `fc-firmware` directory; then simply use `cargo run`.

## Connecting
The firmware exposes a USB CDC device. This can be connected to with a serial
monitor such as the in-built one in the Arduino IDE or a \*nix tool such
`screen` or [`tio`](https://github.com/tio/tio). 

```
Airbrakes flight computer firmware (version v0.1.0)
(c) 2026 Society for Advanced Rocket Propulsion
Use the command 'help' to view available commands.

$ help
Commands:
  version  Print version information for the hardware
  erase    Erase portions or all of the on-board flight log memory
  stats    Reports stats from the flight log
  measure  Takes measurements from sensors
$
```

When using `tio`, it is strongly recommended to map `DEL` to `BS`: `tio --map ODELBS /dev/ttyACM0`.