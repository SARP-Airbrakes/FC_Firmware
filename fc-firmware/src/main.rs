
#![no_main]
#![no_std]
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
use embassy_stm32::{Config, interrupt::InterruptExt, time::mhz};

use panic_probe as _;
use defmt_rtt as _;

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
    let mut cfg = Config::default();

    // Configure clocks
    {
        use embassy_stm32::rcc::*;

        // Closely matched with the solved configuration from CubeMX
        cfg.rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });

        cfg.rcc.pll_src = PllSource::HSE;
        cfg.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL72,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV3), // for 48 MHz clocks
            divr: None, // not using I2S
        });
        cfg.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;

        cfg.rcc.apb1_pre = APBPrescaler::DIV1; // PCLK1 = 16MHz
        cfg.rcc.apb2_pre = APBPrescaler::DIV1; // PCLK2 = 16MHz
        cfg.rcc.ahb_pre = AHBPrescaler::DIV1; // HCLK = 16MHz

        cfg.rcc.sys = Sysclk::HSI;
    }
    let p = embassy_stm32::init(cfg);

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