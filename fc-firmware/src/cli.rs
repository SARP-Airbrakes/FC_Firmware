//! This module is for the implementation of the CLI, including the processing
//! and commands.
//!
//! The CLI is accessible via a USB CDC ACM device exposed on the USB port on
//! the flight computer hardware. For the implementation of reading from the USB
//! device, see [`crate::usb`] and the type [`UsbPipe`]. Of note in this module
//! is the function [`handle_command`], which is where all commands present on
//! the firmware are handled.
//!
//! Importantly, the commands execute on an
//! [InterruptExecutor](embassy_executor::InterruptExecutor), while the commands
//! are processed on a normal thread-mode
//! [Executor](embassy_executor::Executor); this is so the processing thread can
//! stop entirely while waiting for the commands to execute. See the
//! [`process_cli`] function for more details.
//!
//! For details of usage, please see the Connecting section of the folder-level
//! `README.md`.

use core::{convert::Infallible, slice, sync::atomic::{AtomicBool, AtomicUsize, Ordering}};
use cortex_m::peripheral::scb::VectActive;
use defmt::{unreachable, *};
use embassy_executor::SendSpawner;
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex}, mutex::Mutex
};
use heapless::{String, format};
use embedded_cli::cli::CliBuilder;
use crate::{memory::FLIGHT_LOG, usb::{USB_READ_PIPE, USB_WRITE_PIPE, UsbPipe}};

/// The commands present on the firmware.
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

/// Subcommands of the [Measure](Base::Measure) command.
#[derive(embedded_cli::Command, Clone, Copy)]
enum Measure {
    /// Measures the variance in the pressure measurement of the barometer.
    Baro,
    /// Measures the variance in the x-axis acceleration measurement of the accelerometer.
    Accel,
}

/// Shorthand function for writing a string slice to the USB output pipe
/// directly (see [`USB_WRITE_PIPE`]).
#[inline]
async fn usb_write(string: &str) {
    USB_WRITE_PIPE.write(string.as_bytes()).await;
}

/// Prints a formatted version information of the firmware to the USB output
/// pipe (see [`usb_write`]).
async fn print_version() {
    const VERSION_STRING: &'static str = 
        concat!("Airbrakes flight computer firmware (version v", env!("CARGO_PKG_VERSION"), ")\r\n");
    usb_write(VERSION_STRING).await;
    usb_write("(c) 2026 Society for Advanced Rocket Propulsion\r\n").await;
}

/// This task executes the given command.
///
/// Runs the given function after the command has elapsed execution; this
/// callback is used to resume execution of the CLI processing.
///
/// # Lifetimes
/// The command argument requires a [`'static`] lifetime (due to lifetime bounds
/// from [`embassy_executor::task`]), and thus requires unsafe transmutation.
/// See the body of [`process_cli`] for further detail.
#[embassy_executor::task]
pub async fn handle_command(
    command: Base<'static>,
    mark_done: fn()
) {
    scopeguard::defer! { mark_done(); }

    match command {
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

/// This task takes input from the USB and executes commands when
/// commands are found.
///
/// Importantly, this task fully blocks when executing a command, using the
/// Cortex-M instructions WFE/SEV. This task should have a separate, thread-mode
/// executor (see [`embassy_executor::Executor`]) for it to solely use.
///
/// # Panics
/// Panics if the task executes in a non-thread-mode executor.
#[embassy_executor::task]
pub async fn process_cli(spawner: SendSpawner) {
    match cortex_m::peripheral::SCB::vect_active() {
        !VectActive::ThreadMode => panic!("Cannot process CLI in non-thread-mode executor!"),
        _ => {}
    }

    // For tracking when writing from the blocking writer
    let dropped = AtomicUsize::new(0);
    let writer = PipeWriter::new(&USB_WRITE_PIPE, &dropped);

    print_version().await;
    USB_WRITE_PIPE.write("Use the command 'help' to view available commands.\r\n".as_bytes()).await;
    USB_WRITE_PIPE.write("\r\n".as_bytes()).await;

    // Build the CLI with default buffers and buffer sizes.
    let cli = CliBuilder::default()
        .writer(writer)
        .build()
        .unwrap();

    loop {
        let dropped = dropped.swap(0, Ordering::AcqRel);
        if dropped > 0 {
            warn!("{} bytes dropped from console!", dropped);
        }

        // Read a character at a time
        let mut c = 0u8;
        USB_READ_PIPE.read(slice::from_mut(&mut c)).await;

        // Create a processor
        let mut processor = Base::processor(|cli, command| {
            // Use a thread-safe atomic to delineate when the sleep should end.
            static DONE: AtomicBool = AtomicBool::new(false);
            fn mark_done() {
                DONE.store(true, Ordering::Release);
                cortex_m::asm::sev();
            }

            DONE.store(false, Ordering::Release);

            // We are guaranteed by virtue of the lifetime in
            // [`Base::processor`] that [`command`] and the data that it holds
            // will not be dropped or invalid until after the scope ends. Thus,
            // we can "erase" the lifetime present on the data (artifically
            // extend it to infinity, [`'static`]), so we can properly pass it
            // to the command execution task.
            let command = unsafe {
                core::mem::transmute::<Base<'_>, Base<'static>>(command)
            };

            // Throw it into the spawner
            spawner.spawn(
                handle_command(command, mark_done)
                    .unwrap()
            );

            // Sleep the core until marked done
            while !DONE.load(Ordering::Acquire) {
                cortex_m::asm::wfe();
            }
            
            // Makes the CLI write the prompt after finishing execution.
            cli.writer().write_str("");
            Ok(())
        });

        // Use the CLI processor to process the byte. Automatically executes the
        // command if it parses a valid command.
        let _ = cli.process_byte::<Base, _>(
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

    /// Creates a new [`PipeWriter`] from a given [`UsbPipe`] and dropped
    /// counter.
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
