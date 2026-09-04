# BMI088
This crate contains an async driver for the BMI088 6-DOF inertial measurement
unit (IMU).

## Example usage
```rust
use embassy_stm32::{Config, Peripherals};
use bmi088::{Bmi088, AccRange};

let p: Peripherals = embassy_stm32::init(Config::default());
let i2c = {
    // Initialize the async I2C bus.
};
let bmi088 = Bmi088::new(i2c);

bmi088.reset_acc(&mut embassy_time::Delay).await?;
bmi088.set_acc_range(AccRange::Range6G).await?;
bmi088.enable_acc(&mut embassy_time::Delay).await?;

let range = bmi088.acc_range();
if let Ok(m) = bmi088.read_acc().await {
    defmt::info!("z acceleration: {}", m.z_ms2(range));
}
```