# Airbrakes flight computer firmware
This repository contains the Airbrakes flight computer firmware, including the
drivers ([`bmi088/`](/bmi088/README.md), [`bmp390/`](/bmp390/README.md), and
[`w25qxxxjv`](/w25qxxxjv/README.md)), the implementation of the controller and
sensor filtering ([`controller/`](/controller/README.md) and
[`controller-tests/`](/controller-tests/README.md)), and of course, the firmware
itself ([`fc-firmware/`](/fc-firmware/README.md)).

This is intentionally structured as a monorepo; all drivers are fully separate
from the actual firmware itself and are fully reusable in other codebases. For
details regarding running and testing the firmware, see the 
[`fc-firmware` README](/fc-firmware/README.md).

## Architecture
The firmware is based upon the [Embassy](https://embassy.dev) framework, which
uses Rust `async`/`await` to coordinate multi-tasking. As such, all drivers are
written with
[`embedded-io-async`](https://docs.rs/embedded-io-async/latest/embedded_io_async/)
rather than the more ubiquitous
[`embedded-io`](https://docs.rs/embedded-io/latest/embedded_io/).