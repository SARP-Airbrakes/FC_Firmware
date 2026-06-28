
use embedded_cli::cli::CliBuilder;
use rtic::mutex_prelude::*;
use rtic_sync::channel::Sender;
use static_cell::StaticCell;

use crate::{app::cli_process, cli::command::Base};

pub mod usb;
pub mod writer;
pub mod command;

pub const COMMAND_BUFFER_LEN: usize = 64;
pub const HISTORY_BUFFER_LEN: usize = 128;

pub struct Cli(pub embedded_cli::cli::Cli<
    writer::ChannelWriter<'static, { usb::USB_SPLIT_WRITER_LEN }>,
    writer::ChannelWriterError,
    [u8; COMMAND_BUFFER_LEN],
    [u8; HISTORY_BUFFER_LEN],
>);

impl Cli {
    pub fn new(s: Sender<'static, u8, { usb::USB_SPLIT_WRITER_LEN }>) -> Cli {
        static COMMAND_BUFFER: StaticCell<[u8; COMMAND_BUFFER_LEN]> = StaticCell::new();
        static HISTORY_BUFFER: StaticCell<[u8; HISTORY_BUFFER_LEN]> = StaticCell::new();

        let writer = writer::ChannelWriter(s);
        let cli = CliBuilder::default()
            .writer(writer)
            .command_buffer(*COMMAND_BUFFER.init([0u8; COMMAND_BUFFER_LEN]))
            .history_buffer(*HISTORY_BUFFER.init([0u8; HISTORY_BUFFER_LEN]))
            .build()
            .ok()
            .unwrap();

        Cli(cli)
    }
}

pub async fn cli_process(cx: cli_process::Context<'_>, bytes: [u8; 64], count: usize) {
    let mut cli = cx.shared.cli;

    cli.lock(|cli| {
        for i in 0..count {
            Base::process_byte(cli, bytes[i]);
        }
    });
}
