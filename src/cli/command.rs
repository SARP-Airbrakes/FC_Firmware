
use embedded_cli::Command;

use crate::cli::Cli;

#[derive(Command)]
pub enum Base {
    Hello,
}

impl Base {

    pub fn process_byte(cli: &mut Cli, b: u8) {
        cli.0.process_byte::<Base, _>(b, &mut Base::processor(|cli, command| {
            match command {
                Base::Hello => {
                    cli.writer().writeln_str("Hello bro");
                }
            };
            Ok(())
        }));
    }
}
