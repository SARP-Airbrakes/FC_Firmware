use defmt::*;
use embassy_stm32::{Peri, bind_interrupts, dma, gpio, time::mhz, peripherals::*, spi::Spi};
use w25qxxxjv::{Model, W25qxxxjv};
use static_cell::StaticCell;

static DELAY_CELL: StaticCell<embassy_time::Delay> = StaticCell::new();

bind_interrupts!(struct Irqs {
    DMA2_STREAM2 => dma::InterruptHandler<DMA2_CH2>;
    DMA2_STREAM3 => dma::InterruptHandler<DMA2_CH3>;
});

/// Initializes the flash memory for flight log usage.
#[embassy_executor::task]
pub async fn initialize_memory(
    spi: Peri<'static, SPI1>,
    sck: Peri<'static, PA5>,
    mosi: Peri<'static, PA7>,
    miso: Peri<'static, PA6>,
    tx_dma: Peri<'static, DMA2_CH3>,
    rx_dma: Peri<'static, DMA2_CH2>,
    flash_cs: Peri<'static, PA9>,
) {
    let config = {
        let mut config = embassy_stm32::spi::Config::default();
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
        Irqs,
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
}