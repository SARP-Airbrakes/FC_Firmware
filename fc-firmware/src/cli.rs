
use core::{convert::Infallible, slice, sync::atomic::{AtomicUsize, Ordering}};
use defmt::{unreachable, *};
use embassy_executor::SendSpawner;
use embassy_futures::join::join;
use embassy_sync::{
    mutex::Mutex,
    blocking_mutex::raw::{NoopRawMutex, CriticalSectionRawMutex}, 
    pipe::Pipe
};
use embedded_cli::cli::CliBuilder;
use ufmt::{uwriteln, uWrite};
use crate::{memory::FLIGHT_LOG, usb::{USB_READ_PIPE, USB_WRITE_PIPE, UsbPipe}};

static CLI_WRITE_PIPE: Pipe<CriticalSectionRawMutex, 128> = Pipe::new();

#[derive(embedded_cli::Command)]
enum Base {
    /// Print version information for the hardware.
    Version,
    /// Erase portions or all of the on-board flight log memory.
    Erase {
        /// Erases a specific sector (4kb block of memory) of the flight log memory.
        #[arg(long)]
        sector: Option<u32>,

        /// Erases all memory.
        #[arg(short = "a", long)]
        all: bool
    }
}

#[embassy_executor::task]
pub async fn erase_command(sector: Option<u32>, all: bool) {
    if sector.is_some() && all {
        CLI_WRITE_PIPE.write("  erase: Multiple options selected for erasure.".as_bytes()).await;
        return;
    }
    if let Some(sector) = sector {
        let mut l = FLIGHT_LOG.lock().await;
        let f = l.as_mut().map(|l| l.erase_sector(sector));

        if let Some(f) = f {
            let r = f.await;
            match r {
                Ok(_) => {
                    CLI_WRITE_PIPE.write("  erase: Successfully erased sector.".as_bytes()).await;
                },
                Err(e) => {
                    CLI_WRITE_PIPE.write("  erase: Encountered error (check RTT).".as_bytes()).await;
                    warn!("Failed: {}", e);
                }
            }
        }
    } else if all {
        let mut l = FLIGHT_LOG.lock().await;
        let f = l.as_mut().map(|l| l.erase_chip());

        if let Some(f) = f {
            let r = f.await;
            match r {
                Ok(_) => {
                    CLI_WRITE_PIPE.write("  erase: Successfully erased chip.".as_bytes()).await;
                },
                Err(e) => {
                    CLI_WRITE_PIPE.write("  erase: Encountered error (check RTT).".as_bytes()).await;
                    warn!("Failed: {}", e);
                }
            }
        }
    }


}

#[embassy_executor::task]
pub async fn process_cli(spawner: SendSpawner) {
    static DROPPED: AtomicUsize = AtomicUsize::new(0);
    let writer: PipeWriter<'static> = PipeWriter::new(&USB_WRITE_PIPE, &DROPPED);

    let cli = CliBuilder::default()
        .writer(writer)
        .build()
        .unwrap();
    let cli = Mutex::<NoopRawMutex, _>::new(cli);

    join(
        async {
            loop {
                let dropped = dropped.swap(0, Ordering::AcqRel);
                if dropped > 0 {
                    warn!("{} bytes dropped from console!", dropped);
                }

                let mut c = 0u8;
                USB_READ_PIPE.read(slice::from_mut(&mut c)).await;

                let _ = cli.lock().await.process_byte::<Base, _>(
                    c,
                    &mut Base::processor(|cli, command| {
                        match command {
                            Base::Version => {
                                uwriteln!(cli.writer(), "Airbrakes Flight Computer firmware");
                                uwriteln!(cli.writer(), "(c) 2026 Society for Advanced Rocket Propulsion");
                                uwriteln!(cli.writer(), "v{}", env!("CARGO_PKG_VERSION"));
                            },

                            // This might be the ugliest piece of code in the entire firmware.
                            Base::Erase { sector, all } => {
                                spawner.spawn(unwrap!(erase_command(sector, all)));
                            }
                        };
                        Ok(())
                    })
                );

            }
        }, 
        async {
            loop {
                let mut buf = [0u8; 32];
                let num = CLI_WRITE_PIPE.read(&mut buf).await;
                cli.lock().await.write(|w| {
                    for i in 0..num {
                        w.write_char(buf[i] as char);
                    }
                    Ok(())
                });
            }
        }).await;
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
                Ok(buf.len()) // report ok nonetheless
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}