
use embassy_stm32::{Peri, bind_interrupts, dma, gpio, mode::Async, peripherals::*, spi};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use fc_firmware::log::{FlightLog, Packet};

pub static LOG_WRITE_CHANNEL: Channel<CriticalSectionRawMutex, Packet, 8> = Channel::new();
pub static FLIGHT_LOG: Mutex<
    CriticalSectionRawMutex, 
    Option<
        FlightLog<'static, 
            spi::Spi<'static, Async, spi::mode::Master>, 
            gpio::Output<'static>, 
            embassy_time::Delay
        >
    >
> = Mutex::new(None);

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
    let w25 = fc_firmware::initialize_w25q128jv(
        spi, 
        sck, 
        mosi, 
        miso, 
        tx_dma, 
        rx_dma, 
        flash_cs,
        Irqs
    ).await;
    let log = FlightLog::new(w25);
    if let Err(_) = log.read_header().await {
        defmt::debug!("Failed to read log header.");
    }

    // Move log to global mutex.
    { *(FLIGHT_LOG.lock().await) = Some(log); }

    loop {
        let packet = LOG_WRITE_CHANNEL.receive().await;
        FLIGHT_LOG.lock().await.inspect(|mut log| {
            log.push_packet(packet);
        });
    }
}