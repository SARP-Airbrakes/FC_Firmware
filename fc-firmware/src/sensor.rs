
use core::cell::RefCell;

use bmi088::Bmi088;
use bmp390::Bmp390;

use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::SendSpawner;
use embassy_stm32::mode::Blocking;
use embassy_stm32::time::khz;
use embassy_stm32::{gpio, i2c};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::*;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Timer;
use static_cell::StaticCell;

type I2c1Bus = Mutex<CriticalSectionRawMutex, RefCell<I2c<'static, Blocking, Master>>>;
static I2C1_BUS: StaticCell<I2c1Bus> = StaticCell::new();

/// Initializes the I2C1 peripheral and subsequently the sensors connected to it
/// (BMI088, BMP390).
#[embassy_executor::task]
pub async fn initialize_i2c(
    spawner: SendSpawner,
    i2c: Peri<'static, I2C1>,
    mut scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
) {

    // Quickly buzz on SCL just to get rid of any unknown state from reboot
    {
        let mut out = gpio::Output::new(scl.reborrow(), gpio::Level::Low, gpio::Speed::VeryHigh);
        for _ in 0..10 {
            out.toggle();
            Timer::after_millis(20).await;
        }
    }

    // Give peripherals some time to adjust
    Timer::after_millis(50).await;

    let config = {
        let mut config = i2c::Config::default();
        config.frequency = khz(100);
        config
    };
    let i2c = I2c::new_blocking(i2c, scl, sda, config);
    let i2c = RefCell::new(i2c);
    let bus = I2C1_BUS.init(Mutex::new(i2c));

    spawner.spawn(unwrap!(initialize_bmi(bus)));
    spawner.spawn(unwrap!(initialize_bmp(bus)));
}

/// Initializes the BMI088 sensor, and then listens for new data from the
/// device.
#[embassy_executor::task]
async fn initialize_bmi(bus: &'static I2c1Bus) {
    let device = I2cDevice::new(bus);
    let mut bmi = Bmi088::new(device);
    unwrap!(bmi.init(&mut embassy_time::Delay {}));
    unwrap!(bmi.set_acc_range(bmi088::AccRange::Range6G));

    loop {
        Timer::after_secs(1).await;
        if let Ok(m) = bmi.read_acc() {
            let range = bmi.acc_range();
            debug!("Measurement: {}", m);
            debug!("{} {} {}", m.x_ms2(range), m.y_ms2(range), m.z_ms2(range))
        }
    }
}

/// Initializes the BMP390 sensor.
#[embassy_executor::task]
async fn initialize_bmp(bus: &'static I2c1Bus) {
    let device = I2cDevice::new(bus);
    let mut bmp = Bmp390::new(device);
    let coeff = unwrap!(bmp.read_coefficients());
    debug!("Coefficients: {}", coeff);
}