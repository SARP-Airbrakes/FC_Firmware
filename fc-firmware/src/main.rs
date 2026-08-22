
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
use embassy_executor::InterruptExecutor;
use embassy_stm32::{Peri, interrupt};
use embassy_stm32::peripherals::*;
use embassy_stm32::interrupt::InterruptExt;

use crate::cli::process_cli;
use crate::sensor::initialize_i2c;

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
fn SPI2() {
    unsafe {
        EXECUTOR_LOW.on_interrupt();
    }
}

#[interrupt]
fn SPI3() {
    unsafe {
        EXECUTOR_HIGH.on_interrupt();
    }
}

#[embassy_executor::task]
async fn process_usb(
    usb: Peri<'static, USB_OTG_FS>,
    dp: Peri<'static, PA12>,
    dm: Peri<'static, PA11>
) {
    usb::setup_usb(usb, dp, dm).await;
}

#[entry]
fn main() -> ! {
    let p = fc_firmware::setup_stm32();

    debug!("Starting flight firmware.");

    interrupt::SPI2.set_priority(interrupt::Priority::P6);
    let spawner = EXECUTOR_LOW.start(interrupt::SPI2);
    spawner.spawn(unwrap!(process_cli()));

    interrupt::SPI3.set_priority(interrupt::Priority::P7);
    let spawner = EXECUTOR_HIGH.start(interrupt::SPI3);
    spawner.spawn(unwrap!(process_usb(p.USB_OTG_FS, p.PA12, p.PA11)));
    spawner.spawn(unwrap!(initialize_i2c(spawner, p.I2C1, p.PB8, p.PB9)));

    loop {}
}