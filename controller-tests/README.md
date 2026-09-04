# Controller testing
This binary crate contains a small command-line interface to test different
components of the [controller](/controller/README.md).

## Usage
```
$ cargo run -- --help
This crate is used for processing past flight data with the different controller
subcomponents for testing.

Usage: controller-tests [OPTIONS] <COMMAND>

Commands:
  filter  Runs the Kalman filter on given data
  help    Print this message or the help of the given subcommand(s)

Options:
  -v             Enables verbose logging
  -h, --help     Print help
  -V, --version  Print version
```