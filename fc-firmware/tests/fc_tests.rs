#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

use defmt_rtt as _;

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::{unwrap, debug};
    use bmi088::Bmi088;
    use bmp390::{Bmp390, Coefficients};
    use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
    use embassy_time::Timer;
    use embassy_stm32::{Config, gpio, mode::Blocking, time::{khz, mhz}, i2c::{self, I2c, Master}};
    use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
    use core::cell::RefCell;
    use static_cell::StaticCell;

    type I2cBus = Mutex<CriticalSectionRawMutex, RefCell<I2c<'static, Blocking, Master>>>;
    
    struct State {
        bus: &'static I2cBus,
        bmi: Bmi088<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Blocking, Master>>>,
        bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Blocking, Master>>>,
        bmp_calib: Coefficients
    }

    #[init]
    async fn setup() -> State {
        static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();

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
        let p = embassy_stm32::init(cfg);

        let mut scl = p.PB8;
        let sda = p.PB9;

        {
            let mut out = gpio::Output::new(scl.reborrow(), gpio::Level::Low, gpio::Speed::VeryHigh);
            for _ in 0..10 {
                out.toggle();
                Timer::after_millis(20).await;
            }
        }
        Timer::after_millis(50).await;

        let config = {
            let mut config = i2c::Config::default();
            config.frequency = khz(100);
            config
        };
        let i2c = p.I2C1;
        let i2c = I2c::new_blocking(i2c, scl, sda, config);
        let i2c = RefCell::new(i2c);
        let bus = I2C_BUS.init(Mutex::new(i2c));

        let device = I2cDevice::new(bus);
        let mut bmi = Bmi088::new(device);
        unwrap!(bmi.init(&mut embassy_time::Delay {}));
        unwrap!(bmi.set_acc_range(bmi088::AccRange::Range6G));

        let device = I2cDevice::new(bus);
        let mut bmp = Bmp390::new(device);
        let coeff = unwrap!(bmp.read_coefficients());
        unwrap!(bmp.set_pwr_ctrl(
            bmp390::PowerCtrl::PressureEnable | 
            bmp390::PowerCtrl::TemperatureEnable | 
            bmp390::PowerCtrl::Mode(bmp390::PowerCtrlMode::Normal)
        ));

        State {
            bus,
            bmi,
            bmp,
            bmp_calib: coeff
        }
    }

    #[test]
    fn read_acceleration(mut state: State) {
        let m = state.bmi.read_acc().unwrap();

        // assuming the accelerometer is not moving, check we are within normal bounds
        assert!(m.z_ms2(bmi088::AccRange::Range6G) > 5.0);
        assert!(m.z_ms2(bmi088::AccRange::Range6G) < 15.0);

        // then check if we are in reasonable bounds
        assert!(f32::abs(m.z_ms2(bmi088::AccRange::Range6G) - 9.81) <= 0.5);
    }

    #[test]
    fn read_temperature(mut state: State) {
        let m = state.bmp.read_temperature().unwrap();
        let temp = m.compensate(&state.bmp_calib);
        
        // assuming nominal conditions, check if we are in a vaguely room temperature room
        assert!(temp > 5.0);
        assert!(temp < 35.0);
    }

    #[test]
    fn read_pressure(mut state: State) {
        let (p, t) = state.bmp.read().unwrap();
        let temp = t.compensate(&state.bmp_calib);
        let press = p.compensate(&state.bmp_calib, temp);
        
        // assuming that this test is in the troposphere
        assert!(press < 120_000.0); // 101325 is nominal for sea level
        assert!(press > 22_632.0); // tropopause base pressure
    }

    #[test]
    fn read_altitude_bmp390(mut state: State) {
        let (p, t) = state.bmp.read().unwrap();
        let temp = t.compensate(&state.bmp_calib);
        let press = p.compensate(&state.bmp_calib, temp);
        let altitude = bmp390::Pressure::estimate_altitude_hypsometric(press, temp);

        // assuming this test is on the ground
        assert!(altitude > -250.0);
        assert!(altitude < 11_000.0); // tropopause starts at 11km
    }
}