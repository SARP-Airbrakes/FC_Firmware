
use core::{convert::Infallible, slice, sync::atomic::{AtomicBool, AtomicUsize, Ordering}};
use defmt::{unreachable, *};
use embassy_executor::SendSpawner;
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex}, mutex::Mutex
};
use heapless::{String, format};
use embedded_cli::cli::CliBuilder;
use crate::{memory::FLIGHT_LOG, usb::{USB_READ_PIPE, USB_WRITE_PIPE, UsbPipe}};

#[derive(embedded_cli::Command)]
enum Base<'a> {
    /// Print version information for the hardware.
    Version,
    /// Erase portions or all of the on-board flight log memory.
    Erase {
        /// Erases a specific sector (4kb block of memory) of the flight log memory.
        #[arg(long)]
        sector: Option<u32>,

        /// Erases all memory.
        #[arg(short = "a", long)]
        all: bool,

        /// After erasing, sets the flight log flight title to this.
        #[arg(long)]
        title: Option<&'a str>
    },
    /// Reports stats from the flight log.
    Stats,
    /// Takes measurements from sensors.
    Measure {
        /// How many samples to take per measurement (1000 by default)
        #[arg(short = 's', long)]
        samples: Option<u32>,

        #[command(subcommand)]
        command: Measure,
    }
}

#[derive(embedded_cli::Command, Clone, Copy)]
enum Measure {
    /// Measures the variance in the pressure measurement of the barometer.
    Baro,
    /// Measures the variance in the x-axis acceleration measurement of the accelerometer.
    Accel,
}

#[inline]
async fn usb_write(string: &str) {
    USB_WRITE_PIPE.write(string.as_bytes()).await;
}

async fn print_version() {
    const VERSION_STRING: &'static str = 
        concat!("Airbrakes flight computer firmware (version v", env!("CARGO_PKG_VERSION"), ")\r\n");
    usb_write(VERSION_STRING).await;
    usb_write("(c) 2026 Society for Advanced Rocket Propulsion\r\n").await;
}

#[embassy_executor::task]
pub async fn handle_command(
    writer: &'static UsbPipe,
    command: &'static Base<'static>,
    mark_done: fn()
) {
    scopeguard::defer! { mark_done(); }

    match *command {
        Base::Version => {
            print_version().await;
        },
        Base::Erase { sector, all, title } => {
            let mut l = FLIGHT_LOG.lock().await;
            if l.is_none() {
                usb_write("Flight log unavailable.\r\n").await;
                return;
            }
            let log = l.as_mut().unwrap();
            if all {
                usb_write("Erasing entire chip...\r\n").await;
                let _ = log.erase_chip().await;
                let _ = log.reset();
                usb_write("Chip erased.\r\n").await;
            } else if let Some(sector) = sector {
                usb_write("Erasing target sector...\r\n").await;
                let _ = log.erase_sector(sector).await;
                usb_write("Sector erased.\r\n").await;
            } else if title.is_none() {
                usb_write("Nothing selected to erase, or title not given.\r\n").await;
                return;
            }
            if let Some(title) = title {
                let string = String::<32, u8>::try_from(title);
                if let Ok(string) = string {
                    log.header.flight_name = Some(string);
                    let _ = log.update_header().await;
                    usb_write("Wrote title: ").await;
                    usb_write(title).await;
                    usb_write("\r\n").await;
                } else {
                    usb_write("Given title too long; not writing.\r\n").await;
                }
            }
        },
        Base::Stats => {
            let mut l = FLIGHT_LOG.lock().await;
            if l.is_none() {
                usb_write("Flight log unavailable.\r\n").await;
                return;
            }
            let log = l.as_mut().unwrap();
            let formatted = format!(
                64; "Flight name: {}\r\n", 
                log.header.flight_name.as_ref().map_or("<untitled>", String::as_str)
            );
            usb_write(formatted.as_ref().map_or(
                "Flight name: <truncated>\r\n",
                String::as_str
            )).await;
            let formatted = format!(
                64; "Total written packets: {}\r\n",
                log.header.packet_count
            );
            usb_write(formatted.as_ref().map_or(
                "Total written packets: <truncated>\r\n",
                String::as_str
            )).await;
            // Floating point formats are much heavier; avoid if possible
            let formatted = format!(
                64; "Last write: {}.{:03}s after boot\r\n",
                log.header.last_write.as_millis() / 1000,
                log.header.last_write.as_millis() % 1000,
            );
            usb_write(formatted.as_ref().map_or(
                "Last write: <error>\r\n",
                String::as_str
            )).await;
        },
        Base::Measure { samples, command } => {
            let mut variance: f32 = 0.0;
            let mut average: f32 = 0.0;
            for i in 0..samples.unwrap_or(1000) {
                let sample = match command {
                    Measure::Baro => {
                        crate::sensor::LATEST_PRESSURE.wait().await
                    },
                    Measure::Accel => {
                        crate::sensor::LATEST_ACCELERATION_Z.wait().await
                    }
                };

                average *= i as f32;
                average += sample;
                average /= i as f32 + 1.0;
                if i > 0 {
                    variance *= i as f32 - 1.0;
                }
                variance += (sample - average) * (sample - average);
                if i > 0 {
                    variance /= i as f32;
                }

                let rounded = average as u32;
                let decimal = ((average - (rounded as f32)) * 10_000.0) as u32;
                let formatted = format!(
                    64; "Sample: {}.{:04} ({}/{})\r\n",
                    rounded,
                    decimal,
                    i + 1,
                    samples.unwrap_or(1000)
                );
                usb_write(formatted.as_ref().map_or(
                    "",
                    String::as_str
                )).await;
            }

            let rounded = variance as u32;
            let decimal = ((variance - (rounded as f32)) * 1_000_000.0) as u32;
            let formatted = format!(
                64; "Measured variance: {}.{:06}\r\n",
                rounded,
                decimal
            );
            usb_write(formatted.as_ref().map_or(
                "Failed to format variance.\r\n",
                String::as_str
            )).await;
        }
    }
}

#[embassy_executor::task]
pub async fn process_cli(spawner: SendSpawner) {
    let dropped = AtomicUsize::new(0);
    let writer = PipeWriter::new(&USB_WRITE_PIPE, &dropped);

    print_version().await;
    USB_WRITE_PIPE.write("Use the command 'help' to view available commands.\r\n".as_bytes()).await;
    USB_WRITE_PIPE.write("\r\n".as_bytes()).await;

    let cli = CliBuilder::default()
        .writer(writer)
        .build()
        .unwrap();
    let cli = Mutex::<NoopRawMutex, _>::new(cli);

    loop {
        let dropped = dropped.swap(0, Ordering::AcqRel);
        if dropped > 0 {
            warn!("{} bytes dropped from console!", dropped);
        }

        let mut c = 0u8;
        USB_READ_PIPE.read(slice::from_mut(&mut c)).await;

        let mut processor = Base::processor(|cli, command| {
            let command_ref = &command;

            static DONE: AtomicBool = AtomicBool::new(false);
            fn mark_done() {
                DONE.store(true, Ordering::Release);
                cortex_m::asm::sev();
            }
            DONE.store(false, Ordering::Release);

            let command = unsafe {
                core::mem::transmute::<&'_ Base<'_>, &'static Base<'static>>(&command_ref)
            };

            spawner.spawn(
                handle_command(&USB_WRITE_PIPE, command, mark_done)
                    .unwrap()
            );

            ::core::assert_eq!(cortex_m::peripheral::SCB::vect_active(), cortex_m::peripheral::scb::VectActive::ThreadMode);
            while !DONE.load(Ordering::Acquire) {
                cortex_m::asm::wfe();
            }
            cli.writer().write_str(""); // to prompt
            Ok(())
        });
        let _ = cli.lock().await.process_byte::<Base, _>(
            c,
            &mut processor
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