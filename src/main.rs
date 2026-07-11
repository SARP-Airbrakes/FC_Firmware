
#![no_main]
#![no_std]

mod bmi088;
mod cli;
mod filter;

use panic_semihosting as _;

use rtic::app;
use rtic_sync::{channel::Receiver, make_channel};
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::gpio::{
    PB2, 
    Output, 
    PushPull
};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Config;
use stm32f4xx_hal::otg_fs::USB;
use stm32f4xx_hal::i2c::{self, I2c, I2c1};
use stm32f4xx_hal::timer;

use bmi088::Bmi088;
use filter::{Filter, Sensor, SensorKick, Measurement, FilterReceiver, filter_process};
use cli::{
    Cli,
    cli_process,
    usb::{usb_fs, usb_write, USB_SPLIT_WRITER_LEN, UsbSerialDevice}
};

type Mono = stm32f4xx_hal::timer::monotonics::MonoTimerUs<pac::TIM2>;

#[app(device = pac, peripherals = true, dispatchers=[SPI2, USART6])]
mod app {

    use super::*;

    #[shared]
    struct Shared {
        serial_device: UsbSerialDevice,
        filter: Filter,
        delay: timer::DelayMs<pac::TIM1>,
        bmi: Sensor,
    }

    #[local]
    struct Local {
        cli: Cli,
        led: PB2<Output<PushPull>>,
        bmi: Bmi088<I2c1>,
        /*
        encoder: RotaryEncoder<
            QuadratureTableMode,
            PC14,
            PC15
        >
        */
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let mut rcc = dp.RCC.freeze(Config::hse(16.MHz()).sysclk(48.MHz()));
        // let mut syscfg = dp.SYSCFG.constrain(&mut rcc);

        dp.TIM2.monotonic_us(&mut cx.core.NVIC, &mut rcc);

        let mut delay = dp.TIM1.delay_ms(&mut rcc);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let led = gpiob.pb2.into_push_pull_output();

        blink::spawn().ok();

        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: rcc.clocks.hclk(),
        };
        let serial_device = crate::cli::usb::initialize_usb(usb);
        let cli = {
            let (s, r) = make_channel!(u8, USB_SPLIT_WRITER_LEN);
            usb_write::spawn(r).unwrap();
            cli::Cli::new(s)
        };

        let sda = gpiob.pb9;
        let scl = gpiob.pb8;
        let i2c = I2c::new(
            dp.I2C1,
            (scl, sda), 
            i2c::Mode::standard(100.kHz()), 
            &mut rcc
        );

        let filter = {
            let (filter, r) = Filter::new();
            filter_process::spawn(r).unwrap();
            filter
        };
        
        let mut bmi = Bmi088::new(i2c);
        bmi.init(&mut delay).unwrap();

        /*
        let gpioc = dp.GPIOC.split(&mut rcc);
        let mut pc14 = gpioc.pc14.into_floating_input();
        let mut pc15 = gpioc.pc15.into_floating_input();

        pc14.make_interrupt_source(&mut syscfg);
        pc14.enable_interrupt(&mut dp.EXTI);
        pc14.trigger_on_edge(&mut dp.EXTI, gpio::Edge::RisingFalling);

        pc15.make_interrupt_source(&mut syscfg);
        pc15.enable_interrupt(&mut dp.EXTI);
        pc15.trigger_on_edge(&mut dp.EXTI, gpio::Edge::RisingFalling);

        let mut encoder = RotaryEncoder::new(pc14, pc15)
            .into_quadrature_table_mode(1);
        */

        (
            Shared {
                serial_device,
                bmi: Sensor::new(20.Hz(), &filter),
                delay,
                filter,
            },
            Local {
                led,
                cli,
                bmi,
            }
        )
    }

    #[idle(shared=[delay, bmi])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.shared.bmi.lock(|bmi| {
                match bmi.kick() {
                    SensorKick::Kick => {
                        bmi_measure::spawn().ok();
                    },
                    _ => {}
                }
            });
            cx.shared.delay.lock(|delay| delay.delay_ms(1));
        }
    }

    extern "Rust" {
        #[task(binds=OTG_FS, shared=[serial_device])]
        fn usb_fs(cx: usb_fs::Context);

        #[task(priority=2, shared=[serial_device])]
        async fn usb_write(cx: usb_write::Context, mut receiver: Receiver<'static, u8, USB_SPLIT_WRITER_LEN>);

        #[task(priority=1, local=[cli])]
        async fn cli_process(cx: cli_process::Context, bytes: [u8; 64], count: usize);

        #[task(priority=1, shared=[filter])]
        async fn filter_process(mut cx: filter_process::Context, mut receiver: FilterReceiver);
    }

    #[task(priority=1, local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis().into()).await;
        }
    }

    #[task(priority=1, shared=[bmi], local=[bmi])]
    async fn bmi_measure(mut cx: bmi_measure::Context) {
        if let Ok(m) = cx.local.bmi.read_acc() {
            cx.shared.bmi.lock(|bmi| {
                // ignore
                bmi.try_send(Measurement::ACC(m)).ok();
            });
        }
    }

}
