
use embedded_cli::Command;

use crate::cli::Cli;

#[derive(Command)]
pub enum Base {
    Hello,
    Version,
}

impl Base {

    pub fn process_byte(cli: &mut Cli, b: u8) {
        let _ = cli.0.process_byte::<Base, _>(b, &mut Base::processor(|cli, command| {
            match command {
                Base::Hello => {
                    let _ = cli.writer().writeln_str("Hello bro");
                },
                Base::Version => {
                    let _ = cli.writer().writeln_str("Airbrakes flight software");
                    let _ = cli.writer().writeln_str("(c) Society for Advanced Rocket Propulsion");
                    let _ = cli.writer().write_str("Version ");
                    let _ = cli.writer().writeln_str(env!("CARGO_PKG_VERSION"));
                }
            };
            Ok(())
        }));
    }
}
