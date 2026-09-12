//! This library comprises the initialization of the Airbrakes flight computer
//! hardware.
//!
//! This includes the clock configuration and each of the peripherals including
//! the [`bmp390`], [`bmi088`], and [`w25qxxxjv`] (in the form of a W25Q128JV on
//! board). The intention with this library is to provide a way for both the
//! firmware (`main.rs`) and the integration tests (`tests/`) to initialize the
//! hardware.
//!
//! For information regarding building and running the firmware, please see the
//! folder-level documentation (`fc-firmware/README.md`).

#![no_std]
#![no_main]

pub mod log;

use defmt::unwrap;
use embassy_stm32::{
    Config, 
    Peri, 
    Peripherals, 
    dma, 
    gpio, 
    i2c::{self, I2c}, 
    interrupt::typelevel, 
    mode, 
    peripherals::*, 
    spi::{self, Spi}, 
    time::{khz, mhz}
};
use embassy_time::Timer;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use w25qxxxjv::{Model, W25qxxxjv};
use static_cell::StaticCell;

use defmt_rtt as _;

/// An I2C device that is synchronized between several Embassy tasks.
pub type I2cBus = Mutex<CriticalSectionRawMutex, I2c<'static, mode::Async, i2c::mode::Master>>;

/// Initializes the [`embassy_stm32`] HAL with the flight computer clock
/// configuration.
///
/// This specifically initializes the HSE to be a 16MHz oscillator (as the
/// latest revision of the PCB is setup) and for no LSE to be present. It also
/// ensures that all of the peripheral clocks are at a stable 16MHz sourced 
/// from the HSE.
pub fn setup_stm32() -> Peripherals {
    let mut cfg = Config::default();

    // Configure clocks
    // This is closely matched with the clock configuration from the original
    // C++ firmware, as solved by the STM32CubeMX configuration.
    {
        use embassy_stm32::rcc::*;

        cfg.rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });

        // Source the peripheral clocks from the HSE
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
/// 
/// Configures the given SPI bus and the DMA (specifically DMA2 on channels 2
/// and 3) to be used solely for interfacing with the W25Q128JV. The parameters
/// are specific to the latest revision of the PCB.
///
/// # Panics
/// This function will panic if the W25Q128JV is not correctly connected, and
/// the device id could not be read or is incorrect (different model). This
/// function will also panic if called more than one time.
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
    D: typelevel::Binding<typelevel::DMA2_STREAM2, dma::InterruptHandler<DMA2_CH2>> +
        typelevel::Binding<typelevel::DMA2_STREAM3, dma::InterruptHandler<DMA2_CH3>> +
        'static
{
    static DELAY_CELL: StaticCell<embassy_time::Delay> = StaticCell::new();

    // Configure with a 1MHz frequency
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

/// Initializes the I2C bus for usage with the two peripheral driver
/// implementations.
///
/// Creates a synchronization primitive ([`embassy_sync::mutex::Mutex`], see
/// [`I2cBus`]) to share the I2C device between Embassy tasks safely. See also
/// [`embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice`] for an
/// implementation of [`embedded_hal_async::i2c::I2c`] to use the [`I2cBus`].
///
/// # Panics
/// This function will panic if called more than once.
pub async fn initialize_i2c_bus<D>(
    i2c: Peri<'static, I2C1>,
    mut scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
    tx_dma: Peri<'static, DMA1_CH6>,
    rx_dma: Peri<'static, DMA1_CH0>,
    interrupts: D,
) -> &'static mut I2cBus 
where
    D: typelevel::Binding<typelevel::I2C1_EV, i2c::EventInterruptHandler<I2C1>> +
        typelevel::Binding<typelevel::I2C1_ER, i2c::ErrorInterruptHandler<I2C1>> +
        typelevel::Binding<typelevel::DMA1_STREAM0, dma::InterruptHandler<DMA1_CH0>> +
        typelevel::Binding<typelevel::DMA1_STREAM6, dma::InterruptHandler<DMA1_CH6>> +
        'static,
{
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
    let i2c = I2c::new(i2c, scl, sda, tx_dma, rx_dma, interrupts, config);
    I2C_BUS.init(I2cBus::new(i2c))
}

#[cfg(test)]
#[doc(hidden)]
#[embedded_test::tests]
mod tests {
    // This has to be here to avoid linker issues.
}
