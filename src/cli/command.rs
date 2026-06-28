
use embedded_cli::Command;

use crate::cli::Cli;

#[derive(Command)]
pub enum Base {
    Hello,
    Version,
}

impl Base {

    pub fn process_byte(cli: &mut Cli, b: u8) {
        cli.0.process_byte::<Base, _>(b, &mut Base::processor(|cli, command| {
            match command {
                Base::Hello => {
                    cli.writer().writeln_str("Hello bro");
                },
                Base::Version => {
                    cli.writer().writeln_str("Airbrakes flight software");
                    cli.writer().writeln_str("(c) Society for Advanced Rocket Propulsion");
                    cli.writer().write_str("Version ");
                    cli.writer().writeln_str(env!("CARGO_PKG_VERSION"));
                }
            };
            Ok(())
        }));
    }
}
