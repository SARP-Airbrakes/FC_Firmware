# BMP390
This crate contains an async driver for the BMP390 barometer.

## Example usage
```rust
use embassy_stm32::{Config, Peripherals};
use bmp390::{Bmp390, Pressure, PowerCtrl, PowerCtrlMode};

let p: Peripherals = embassy_stm32::init(Config::default());
let i2c = {
    // Initialize the async I2C bus.
};
let bmp390 = Bmp390::new(i2c);

bmp.set_pwr_ctrl(
    PowerCtrl::PressureEnable |
    PowerCtrl::TemperatureEnable |
    PowerCtrl::Mode(PowerCtrlMode::Normal)
).await?;

// calibration coefficients
let coeff = bmp390.read_coefficients().await?;

if let Ok((press, temp)) = bmp390.read().await {
    // Use the calibration coeff. to calculate the actual value.
    let temp: f32 = temp.compensate(&coeff);
    let press: f32 = press.compensate(&coeff, temp)
    
    // Use the hypsometric equation to estimate altitude.
    let altitude: f32 = Pressure::estimate_altitude_hypsometric(press, temp);
    defmt::info!("Altitude: {} m", altitude);
}
```