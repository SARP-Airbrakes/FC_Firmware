# W25QxxxJV
This crate is an async driver for the W25QxxxJV series of NOR flash memory. Supports the W25Q16JV to the W25Q128JV.

## Example usage
```rust
use embassy_stm32::{Config, Peripherals};
use w25qxxxjv::{Model, W25qxxxjv};

let p: Peripherals = embassy_stm32::init(Config::default());
let spi = {
    // Initialize the async SPI interface.
};

// Initialize the CS line.
let flash_cs = gpio::Output::new(
    p.PB9, 
    gpio::Level::High, 
    gpio::Speed::VeryHigh
);

let mut w25q128jv = W25qxxxjv::new(
    spi,
    flash_cs,
    Model::W25q128jv,
    &mut embassy_time::Delay,
);
w25q128jv.init().await?;

```
