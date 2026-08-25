
use core::{convert::Infallible, slice, sync::atomic::{AtomicUsize, Ordering}};
use defmt::*;
use embedded_cli::cli::CliBuilder;
use ufmt::uwriteln;
use crate::usb::{UsbPipe, USB_READ_PIPE, USB_WRITE_PIPE};

#[derive(embedded_cli::Command)]
enum Base {
    /// Print version information for the hardware.
    Version,
}

#[embassy_executor::task]
pub async fn process_cli() {
    let dropped = AtomicUsize::new(0);
    let writer = PipeWriter::new(&USB_WRITE_PIPE, &dropped);

    let mut cli = CliBuilder::default()
        .writer(writer)
        .build()
        .unwrap();

    loop {
        let dropped = dropped.swap(0, Ordering::AcqRel);
        if dropped > 0 {
            warn!("{} bytes dropped from console!", dropped);
        }

        let mut c = 0u8;
        USB_READ_PIPE.read(slice::from_mut(&mut c)).await;

        let _ = cli.process_byte::<Base, _>(
            c,
            &mut Base::processor(|cli, command| {
                match command {
                    Base::Version => {
                        uwriteln!(cli.writer(), "Airbrakes Flight Computer firmware");
                        uwriteln!(cli.writer(), "(c) 2026 Society for Advanced Rocket Propulsion");
                        uwriteln!(cli.writer(), "v{}", env!("CARGO_PKG_VERSION"));
                    },
                };
                Ok(())
            })
        );

    }
}

/// A blocking writer that wraps around a pipe synchronization primitive.
/// Reports how many bytes have been lost.
struct PipeWriter<'a> {
    pipe: &'static UsbPipe,
    dropped: &'a AtomicUsize,
}

impl<'a> PipeWriter<'a> {

    pub fn new(pipe: &'static UsbPipe, dropped: &'a AtomicUsize) -> Self {
        Self {
            pipe,
            dropped
        }
    }
}

impl<'a> embedded_io::ErrorType for PipeWriter<'a> {
    type Error = Infallible;
}

impl<'a> embedded_io::Write for PipeWriter<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match self.pipe.try_write(buf) {
            Ok(n) => Ok(n),
            Err(_) => {
                self.dropped.fetch_add(buf.len(), Ordering::AcqRel);
                Ok(buf.len()) // report ok nonetheless
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}