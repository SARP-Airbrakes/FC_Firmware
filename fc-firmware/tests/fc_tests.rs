#![no_std]
#![no_main]

#![feature(impl_trait_in_assoc_type)]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::{unwrap, info};
    use bmi088::Bmi088;
    use bmp390::{Bmp390, Coefficients};
    use w25qxxxjv::{W25qxxxjv, Model};
    use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
    use embassy_time::Timer;
    use embassy_stm32::{
        bind_interrupts, 
        dma, 
        peripherals, 
        gpio, 
        i2c::{I2c, Master}, 
        mode::Blocking,
        spi::Spi,
    };
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    bind_interrupts!(struct Irqs {
        DMA2_STREAM2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
        DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
    });
    
    struct State {
        w25: W25qxxxjv<'static, Spi<'static, embassy_stm32::mode::Async, embassy_stm32::spi::mode::Master>, gpio::Output<'static>, embassy_time::Delay>,
        bmi: Bmi088<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Blocking, Master>>>,
        bmp: Bmp390<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Blocking, Master>>>,
        bmp_calib: Coefficients
    }

    #[init]
    async fn setup() -> State {
        let p = fc_firmware::setup_stm32();

        let bus = fc_firmware::initialize_i2c_bus(
            p.I2C1,
            p.PB8,
            p.PB9
        ).await;

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
            bmp390::PowerCtrl::Mode(bmp390::PowerCtrlMode::Forced) // just once per test
        ));

        let w25 = fc_firmware::initialize_w25q128jv(
            p.SPI1,
            p.PA5,
            p.PA7,
            p.PA6,
            p.DMA2_CH3,
            p.DMA2_CH2,
            p.PA9,
            Irqs,
        ).await;

        State {
            w25,
            bmi,
            bmp,
            bmp_calib: coeff
        }
    }

    /// Tests reading the acceleration off of the BMI088 connected to the board.
    #[test]
    fn read_acceleration(mut state: State) {
        let m = state.bmi.read_acc().unwrap();

        info!(
            "Acceleration acquired: ({} m/s^2, {} m/s^2, {} m/s^2)", 
            m.x_ms2(bmi088::AccRange::Range6G),
            m.y_ms2(bmi088::AccRange::Range6G),
            m.z_ms2(bmi088::AccRange::Range6G)
        );

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

        info!("Temperature acquired: {} C", temp);
        
        // assuming nominal conditions, check if we are in a vaguely room temperature room
        assert!(temp > 5.0);
        assert!(temp < 35.0);
    }

    #[test]
    fn read_pressure(mut state: State) {
        let (p, t) = state.bmp.read().unwrap();
        let temp = t.compensate(&state.bmp_calib);
        let press = p.compensate(&state.bmp_calib, temp);

        info!("Temperature acquired: {} C", temp);
        info!("Pressure acquired: {} Pa", press);
        
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

        info!("Temperature acquired: {} C", temp);
        info!("Pressure acquired: {} Pa", press);
        info!("Altitude acquired: {} m", altitude);

        // assuming this test is on the ground
        assert!(altitude > -250.0);
        assert!(altitude < 11_000.0); // tropopause starts at 11km
    }

    #[test]
    async fn read_mfr_dev_id(mut state: State) {
        // this is checked in #init() but this is to double check
        let (mfr, dev) = state.w25.read_manufacturer_device_id().await.unwrap();
        assert_eq!(mfr, 0xef);
        assert_eq!(dev, Model::W25q128jv.device_id());
    }
    
    #[test]
    async fn program_and_read_string(mut state: State) {
        const TEST_STRING: &str = "Hello, world! This is a test string.";

        state.w25.erase_sector(0x00).await.unwrap();
        state.w25.page_program(0x00, TEST_STRING.as_bytes()).await.unwrap();

        // give some time
        Timer::after_millis(10).await;

        let mut buf = [0u8; TEST_STRING.len()];      
        state.w25.read_data(0x00, &mut buf).await.unwrap();

        let found = str::from_utf8(&buf).unwrap();

        info!("Got: {}", found);
        info!("Requires: {}", TEST_STRING);
        assert_eq!(found, TEST_STRING);
    }

    #[test]
    async fn write_and_read_string_on_page_boundary(mut state: State) {
        const TEST_STRING: &str = "Lorem ipsum dolor sit amet";
        // writing at address on page boundary
        const ADDRESS: u32 = 0xf0;

        state.w25.erase_sector(ADDRESS).await.unwrap();
        state.w25.write_data(ADDRESS, TEST_STRING.as_bytes()).await.unwrap();

        // give some time
        Timer::after_millis(10).await;

        let mut buf = [0u8; TEST_STRING.len()];      
        state.w25.read_data(ADDRESS, &mut buf).await.unwrap();

        info!("Received: {}", buf);

        let found = str::from_utf8(&buf).unwrap();

        info!("Got: {}", found);
        info!("Requires: {}", TEST_STRING);

        assert_eq!(found, TEST_STRING);
    }

    #[test]
    async fn write_and_read_string_across_multiple_page_boundaries(mut state: State) {
        const TEST_STRING: &str = "
            Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod
            tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim
            veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea
            commodo consequat. Duis aute irure dolor in reprehenderit in voluptate
            velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint
            occaecat cupidatat non proident, sunt in culpa qui officia deserunt
            mollit anim id est laborum. 
        ";
        // writing at address on page boundary
        const ADDRESS: u32 = 0xf0;

        state.w25.erase_sector(ADDRESS).await.unwrap();
        state.w25.write_data(ADDRESS, TEST_STRING.as_bytes()).await.unwrap();

        // give some time
        Timer::after_millis(10).await;

        let mut buf = [0u8; TEST_STRING.len()];      
        state.w25.read_data(ADDRESS, &mut buf).await.unwrap();

        info!("Received: {}", buf);

        let found = str::from_utf8(&buf).unwrap();

        info!("Got: {}", found);
        info!("Requires: {}", TEST_STRING);

        assert_eq!(found, TEST_STRING);
    }

    /// Tests writing just one packet to the flight log.
    #[test]
    async fn flight_log_write_and_read(mut state: State) {
        use fc_firmware::log::{Packet, FlightLog, FlightTime};

        // Erase the first two sectors for purposes of testing.
        unwrap!(state.w25.erase_sector(0x0000).await);
        unwrap!(state.w25.erase_sector(0x1000).await);

        let mut log = FlightLog::new(state.w25);
        unwrap!(log.update_header().await);

        let packet = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 15.0,
            pressure: 101325.0
        };
        unwrap!(log.push_packet(packet.clone()).await);

        Timer::after_millis(10).await;

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet);
    }

    /// Tests writing multiple packets to the flight log and then reading them in sequence.
    #[test]
    async fn flight_log_write_and_read_multiple(mut state: State) {
        use fc_firmware::log::{Packet, FlightLog, FlightTime};

        // Erase the first two sectors for purposes of testing.
        unwrap!(state.w25.erase_sector(0x0000).await);
        unwrap!(state.w25.erase_sector(0x1000).await);

        let mut log = FlightLog::new(state.w25);
        unwrap!(log.update_header().await);

        let packet1 = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 15.0,
            pressure: 101325.0
        };
        let packet2 = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 25.0,
            pressure: 99000.0
        };
        unwrap!(log.push_packet(packet1.clone()).await);
        unwrap!(log.push_packet(packet2.clone()).await);

        Timer::after_millis(10).await;

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet1);

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet2);

        let fail = log.read_next_packet().await;
        assert!(fail.is_err());
    }

    #[test]
    async fn flight_log_reboot_and_read(mut state: State) {
        use fc_firmware::log::{Packet, FlightLog, FlightTime};

        // Erase the first two sectors for purposes of testing.
        unwrap!(state.w25.erase_sector(0x0000).await);
        unwrap!(state.w25.erase_sector(0x1000).await);

        let mut log = FlightLog::new(state.w25);
        unwrap!(log.update_header().await);

        let packet1 = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 15.0,
            pressure: 101325.0
        };
        unwrap!(log.push_packet(packet1.clone()).await);

        Timer::after_millis(10).await;

        // Refresh the FlightLog state entirely by destroying it and remaking
        // it. This is roughly equivalent to rebooting.
        let mut log = FlightLog::new(log.destroy());
        unwrap!(log.read_header().await);

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet1);
    }

    #[test]
    async fn flight_log_reboot_and_write_and_read(mut state: State) {
        use fc_firmware::log::{Packet, FlightLog, FlightTime};

        // Erase the first two sectors for purposes of testing.
        unwrap!(state.w25.erase_sector(0x0000).await);
        unwrap!(state.w25.erase_sector(0x1000).await);

        let mut log = FlightLog::new(state.w25);
        unwrap!(log.update_header().await);

        let packet1 = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 15.0,
            pressure: 101325.0
        };
        unwrap!(log.push_packet(packet1.clone()).await);

        // Make sure that we wrote one packet correctly.
        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet1);

        Timer::after_millis(10).await;

        // Refresh the FlightLog state entirely by destroying it and remaking
        // it. This is roughly equivalent to rebooting.
        let mut log = FlightLog::new(log.destroy());
        unwrap!(log.read_header().await);

        let packet2 = Packet::TemperaturePressure {
            time: FlightTime::now(), 
            temperature: 25.0,
            pressure: 99000.0
        };
        unwrap!(log.push_packet(packet2.clone()).await);

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet1);

        let read_packet = unwrap!(log.read_next_packet().await);
        assert_eq!(read_packet, packet2);

        let fail = log.read_next_packet().await;
        assert!(fail.is_err());
    }
}