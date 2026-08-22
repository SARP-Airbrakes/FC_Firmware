
use embassy_stm32::{Peri, bind_interrupts, dma, peripherals::*};

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
    let _w25 = fc_firmware::initialize_w25q128jv(
        spi, 
        sck, 
        mosi, 
        miso, 
        tx_dma, 
        rx_dma, 
        flash_cs,
        Irqs
    ).await;
}