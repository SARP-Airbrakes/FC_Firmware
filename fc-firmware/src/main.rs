//! This is firmware for the Airbrakes flight computer.
//!
//! There are, in total, three Embassy executors: a thread executor (see
//! [`EXECUTOR_CLI`]), and two interrupt executors ([`EXECUTOR_LOW`] at priority
//! 6, and [`EXECUTOR_HIGH`] at priority 7). The thread executor has the sole
//! purpose of for processing CLI and processing commands. The low priority
//! executor is for the flight log, and the high priority executor is for the
//! USB, sensors, and controller.
//!
//! For information regarding running and testing the firmware, please see the
//! folder-level `README.md`.

#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![feature(never_type)]

mod cli;
mod sensor;
mod memory;
mod usb;

use defmt::*;
use cortex_m_rt::entry;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_stm32::{Peri, interrupt};
use embassy_stm32::peripherals::*;
use embassy_stm32::interrupt::InterruptExt;

use panic_probe as _;
use static_cell::StaticCell;

use crate::cli::process_cli;
use crate::memory::initialize_memory;
use crate::sensor::initialize_i2c;

/// The high-priority executor, for the USB, sensors and controller.
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
/// The low-priority executor for the flight log.
static EXECUTOR_LOW: InterruptExecutor = InterruptExecutor::new();
/// The thread-mode executor for the CLI.
static EXECUTOR_CLI: StaticCell<Executor> = StaticCell::new();
 
/// Spare interrupt used for the low priority interrupt executor.
#[interrupt]
fn SPI2() {
    unsafe {
        EXECUTOR_LOW.on_interrupt();
    }
}

/// Spare interrupt used for the high priority interrupt executor.
#[interrupt]
fn SPI3() {
    unsafe {
        EXECUTOR_HIGH.on_interrupt();
    }
}

/// Task to setup and process the USB CDC ACM device.
#[embassy_executor::task]
async fn process_usb(
    usb: Peri<'static, USB_OTG_FS>,
    dp: Peri<'static, PA12>,
    dm: Peri<'static, PA11>
) {
    usb::setup_usb(usb, dp, dm).await;
}

/// Entry-point of the flight computer firmware.
#[entry]
fn main() -> ! {
    // Initialize peripherals.
    let p = fc_firmware::setup_stm32();

    debug!("Starting flight firmware.");

    // Initialize the low-priority executor.
    interrupt::SPI2.set_priority(interrupt::Priority::P6);
    let low_spawner = EXECUTOR_LOW.start(interrupt::SPI2);
    low_spawner.spawn(unwrap!(initialize_memory(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA2_CH3,
        p.DMA2_CH2,
        p.PA9
    )));

    // Initialize the high-priority executor.
    interrupt::SPI3.set_priority(interrupt::Priority::P7);
    let high_spawner = EXECUTOR_HIGH.start(interrupt::SPI3);
    high_spawner.spawn(unwrap!(process_usb(p.USB_OTG_FS, p.PA12, p.PA11)));
    high_spawner.spawn(unwrap!(initialize_i2c(high_spawner, p.I2C1, p.PB8, p.PB9, p.DMA1_CH6, p.DMA1_CH0)));
 
    // Initialize and run the thread-mode executor.
    EXECUTOR_CLI
        .init_with(Executor::new)
        .run(|spawner| {
            spawner.spawn(unwrap!(process_cli(low_spawner)));
        });
}
