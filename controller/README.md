# Controller
This crate contains the finite state machine ([`stage.rs`](src/stage.rs)), the
Kalman filter implementation ([`filter.rs`](src/filter.rs)) and a $C_D$ solver
([`solver.rs`](src/solver.rs)). For more information regarding the theory for
each of these systems, see the 
[SARP wiki](https://sarp-uw.github.io/docs/docs/Payload/Payload/).

## Testing
There are unit tests present in this crate; these can be run with `cargo test`.
For integrated testing, see both the 
[`fc-firmware` crate](/fc-firmware/README.md) and the 
[`controller-tests` crate](/fc-firmware/README.md).