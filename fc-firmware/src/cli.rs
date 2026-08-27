
use core::{convert::Infallible, slice, sync::atomic::{AtomicUsize, Ordering}};
use defmt::{unreachable, *};
use embassy_executor::SendSpawner;
use embassy_futures::join::join;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, mutex::Mutex, pipe::Pipe, signal::Signal
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
    },
    /// Reports stats from the flight log.
    Stats,
}

struct CliWriter;

impl uWrite for CliWriter {
    type Error = embassy_sync::pipe::TryWriteError;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        USB_WRITE_PIPE.try_write(s.as_bytes()).map(|_| ())
    }
}

#[embassy_executor::task]
pub async fn erase_command(sector: Option<u32>, all: bool) {
    if sector.is_some() && all {
        USB_WRITE_PIPE.write("  erase: Multiple options selected for erasure.".as_bytes()).await;
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
        CLI_WRITE_PIPE.write("  erase: Erasing all. This may take awhile..".as_bytes()).await;

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
pub async fn stats_command() {
    let mut l = FLIGHT_LOG.lock().await;
    if let Some(header) = l.as_mut().map(|l| &l.header) {
        if let Some(name) = &header.flight_name {
            let _ = uwriteln!(CliWriter, "  stats: Current flight name: {}.", name.as_str());
        } else {
            let _ = uwriteln!(CliWriter, "  stats: Current flight name: <untitled>");
        }
        let _ = uwriteln!(
            CliWriter,
            "  stats: Last write was {}ms",
            header.last_write.as_millis()
        );
        let _ = uwriteln!(CliWriter, "  stats: {} packets total", header.packet_count);
    } else {
        let _ = uwriteln!(CliWriter, "  stats: Failed to access the flight log.");
    }
}

#[embassy_executor::task]
pub async fn process_cli(spawner: SendSpawner) {
    let dropped = AtomicUsize::new(0);
    let writer = PipeWriter::new(&USB_WRITE_PIPE, &dropped);

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

                            Base::Erase { sector, all } => {
                                if let Ok(t) = erase_command(sector, all) {
                                    spawner.spawn(t);
                                } else {
                                    uwriteln!(cli.writer(), "Failed to start the erase process.");
                                }
                            },

                            Base::Stats => {
                                if let Ok(t) = stats_command() {
                                    spawner.spawn(t);
                                } else {
                                    uwriteln!(cli.writer(), "Failed to start the stats process.");
                                }
                            }
                        };
                        Ok(())
                    })
                );

            }
        }, 
        async {
            loop {
                let mut buf = [0u8; 64];
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
                self.dropped.fetch_add(buf.len(), Ordering::AcqRel);
                Ok(buf.len()) // report ok nonetheless
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}