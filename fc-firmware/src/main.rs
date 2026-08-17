
#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{Config, rcc, time::mhz};
use embassy_time::Timer;

use panic_probe as _;
use defmt_rtt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut cfg = Config::default();
    cfg.rcc.hse = Some(rcc::Hse {
        freq: mhz(16),
        mode: rcc::HseMode::Oscillator,
    });

    let p = embassy_stm32::init(cfg);
    info!("Starting airbrakes firmware.");

    loop {

        info!("Loop!");
        Timer::after_millis(100).await;
    }
}