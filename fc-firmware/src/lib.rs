#![no_std]
#![no_main]

//! This library comprises all setup boilerplate for the airbrakes flight
//! computer hardware.

pub mod log;

use defmt::unwrap;
use embassy_stm32::{
    Config, 
    Peri, 
    Peripherals, 
    gpio, 
    mode,
    time::{khz, mhz}, 
    peripherals::*, 
    spi::{self, Spi},
    i2c::{self, I2c}
};
use embassy_time::Timer;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use w25qxxxjv::{Model, W25qxxxjv};
use core::cell::RefCell;
use static_cell::StaticCell;

use defmt_rtt as _;

pub type I2cBus = Mutex<CriticalSectionRawMutex, RefCell<I2c<'static, mode::Blocking, i2c::mode::Master>>>;

/// Initializes the [`embassy_stm32`] HAL with the flight computer clock
/// configuration.
pub fn setup_stm32() -> Peripherals {
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
    embassy_stm32::init(cfg)
}

/// Sets up the W25Q128JV connected to the board.
pub async fn initialize_w25q128jv<D>(
    spi: Peri<'static, SPI1>,
    sck: Peri<'static, PA5>,
    mosi: Peri<'static, PA7>,
    miso: Peri<'static, PA6>,
    tx_dma: Peri<'static, DMA2_CH3>,
    rx_dma: Peri<'static, DMA2_CH2>,
    flash_cs: Peri<'static, PA9>,
    interrupts: D
) -> W25qxxxjv<'static, Spi<'static, mode::Async, spi::mode::Master>, gpio::Output<'static>, embassy_time::Delay>
where
    D: embassy_stm32::interrupt::typelevel::Binding<embassy_stm32::interrupt::typelevel::DMA2_STREAM2, embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH2>> +
        embassy_stm32::interrupt::typelevel::Binding<embassy_stm32::interrupt::typelevel::DMA2_STREAM3, embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>> +
        'static
{
    static DELAY_CELL: StaticCell<embassy_time::Delay> = StaticCell::new();

    let config = {
        let mut config = spi::Config::default();
        config.frequency = mhz(1);
        config
    };
    let spi = Spi::new(
        spi,
        sck,
        mosi,
        miso,
        tx_dma,
        rx_dma,
        interrupts,
        config
    );

    let delay = DELAY_CELL.init(embassy_time::Delay);
    let mut w25q128jv = W25qxxxjv::new(
        spi,
        gpio::Output::new(
            flash_cs, 
            gpio::Level::High, 
            gpio::Speed::VeryHigh
        ),
        Model::W25q128jv,
        delay
    );
    unwrap!(w25q128jv.init().await);
    w25q128jv
}

pub async fn initialize_i2c_bus(
    i2c: Peri<'static, I2C1>,
    mut scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>
) -> &'static mut I2cBus {
    static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

    // Wiggle the SCL to try and clear any erroneous peripheral states
    {
        let mut out = gpio::Output::new(scl.reborrow(), gpio::Level::Low, gpio::Speed::VeryHigh);
        for _ in 0..5 {
            out.toggle();
            Timer::after_millis(10).await;
        }
    }
    Timer::after_millis(20).await;

    let config = {
        let mut config = i2c::Config::default();
        config.frequency = khz(100);
        config
    };
    let i2c = I2c::new_blocking(i2c, scl, sda, config);
    let i2c = RefCell::new(i2c);
    I2C_BUS.init(I2cBus::new(i2c))
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {

}
