
use embedded_cli::Command;

pub mod usb;
pub mod writer;

pub const COMMAND_BUFFER_LEN: usize = 64;
pub const HISTORY_BUFFER_LEN: usize = 128;

pub type Cli = embedded_cli::cli::Cli<
    writer::FnWriter,
    core::convert::Infallible,
    [u8; COMMAND_BUFFER_LEN],
    [u8; HISTORY_BUFFER_LEN],
>;


#[derive(Command)]
pub enum Base {
    Hello,
    Version,
}

impl Base {

    pub fn process_byte(cli: &mut Cli, b: u8) {
        let _ = cli.process_byte::<Base, _>(b, &mut Base::processor(|cli, command| {
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
