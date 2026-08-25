

use bmi088::Bmi088;
use bmp390::{Bmp390, PowerCtrl, PowerCtrlMode};

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::SendSpawner;
use embassy_stm32::{Peri, dma, i2c};
use embassy_stm32::bind_interrupts;
use embassy_stm32::peripherals::*;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    DMA1_STREAM0 => dma::InterruptHandler<DMA1_CH0>;
    DMA1_STREAM6 => dma::InterruptHandler<DMA1_CH6>;
});

/// Initializes the I2C1 peripheral and subsequently the sensors connected to it
/// (BMI088, BMP390).
#[embassy_executor::task]
pub async fn initialize_i2c(
    spawner: SendSpawner,
    i2c: Peri<'static, I2C1>,
    scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
    tx_dma: Peri<'static, DMA1_CH6>,
    rx_dma: Peri<'static, DMA1_CH0>,
) {
    let bus = fc_firmware::initialize_i2c_bus(i2c, scl, sda, tx_dma, rx_dma, Irqs).await;

    spawner.spawn(unwrap!(initialize_bmi(bus)));
    spawner.spawn(unwrap!(initialize_bmp(bus)));
}

/// Initializes the BMI088 sensor, and then listens for new data from the
/// device.
#[embassy_executor::task]
async fn initialize_bmi(bus: &'static fc_firmware::I2cBus) {
    let device = I2cDevice::new(bus);
    let mut bmi = Bmi088::new(device);
    unwrap!(bmi.init(&mut embassy_time::Delay {}).await);
    unwrap!(bmi.set_acc_range(bmi088::AccRange::Range6G).await);

    loop {
        Timer::after_secs(1).await;
        if let Ok(m) = bmi.read_acc().await {
            let range = bmi.acc_range();
            debug!("Measurement: {}", m);
            debug!("{} {} {}", m.x_ms2(range), m.y_ms2(range), m.z_ms2(range))
        }
    }
}

/// Initializes the BMP390 sensor.
#[embassy_executor::task]
async fn initialize_bmp(bus: &'static fc_firmware::I2cBus) {
    let device = I2cDevice::new(bus);
    let mut bmp = Bmp390::new(device);
    let coeff = unwrap!(bmp.read_coefficients().await);
    unwrap!(bmp.set_pwr_ctrl(
        PowerCtrl::PressureEnable | 
        PowerCtrl::TemperatureEnable | 
        PowerCtrl::Mode(PowerCtrlMode::Normal)
    ).await);
    debug!("Coefficients: {}", coeff);

    loop {
        Timer::after_secs(1).await;
        if let Ok((p, t)) = bmp.read().await {
            let temp = t.compensate(&coeff);
            let press = p.compensate(&coeff, temp);
            let altitude = bmp390::Pressure::estimate_altitude_hypsometric(press, temp);
            debug!("Temperature measurement: {} C", temp);
            debug!("Pressure measurement: {} Pa", press);
            debug!("Altitude measurement: {} m", altitude);
        }
    }
}