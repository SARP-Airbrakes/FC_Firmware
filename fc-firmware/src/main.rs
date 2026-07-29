
#![no_main]
#![no_std]

mod cli;

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
use stm32f4xx_hal::timer;
use embedded_cli::cli::CliBuilder;

use cli::usb::UsbSerialDevice;

type Mono = stm32f4xx_hal::timer::monotonics::MonoTimerUs<pac::TIM2>;

const CLI_PROCESS_LEN: usize = 64;

#[app(device = pac, peripherals = true, dispatchers=[SPI2, USART6])]
mod app {

use rtic_sync::channel::Sender;
use static_cell::StaticCell;

use super::*;

    #[shared]
    struct Shared {
        serial_device: UsbSerialDevice,
        delay: timer::DelayMs<pac::TIM1>,
    }

    #[local]
    struct Local {
        cli: cli::Cli,
        cli_processor_sender: Sender<'static, u8, CLI_PROCESS_LEN>,
        led: PB2<Output<PushPull>>,
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

        /* USB */
        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: rcc.clocks.hclk(),
        };
        let serial_device = UsbSerialDevice::new(usb);

        /* CLI */
        static COMMAND_BUFFER: StaticCell<[u8; cli::COMMAND_BUFFER_LEN]> = StaticCell::new();
        static HISTORY_BUFFER: StaticCell<[u8; cli::HISTORY_BUFFER_LEN]> = StaticCell::new();

        let cli = CliBuilder::default()
            .writer(cli::writer::FnWriter(|buf| { for byte in buf { usb_write::spawn(*byte).ok(); }; }))
            .command_buffer(*COMMAND_BUFFER.init([0u8; cli::COMMAND_BUFFER_LEN]))
            .history_buffer(*HISTORY_BUFFER.init([0u8; cli::HISTORY_BUFFER_LEN]))
            .build()
            .ok()
            .unwrap();

        /* CLI processor */
        let (s, r) = make_channel!(u8, CLI_PROCESS_LEN);
        cli_process::spawn(r).ok();

        /*
        let sda = gpiob.pb9;
        let scl = gpiob.pb8;
        let i2c = I2c::new(
            dp.I2C1,
            (scl, sda), 
            i2c::Mode::standard(100.kHz()), 
            &mut rcc
        );
        */

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
                delay,
            },
            Local {
                led,
                cli_processor_sender: s,
                cli,
            }
        )
    }

    #[idle(shared=[delay])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
        }
    }

    #[task(binds=OTG_FS, shared=[serial_device], local=[cli_processor_sender])]
    fn usb_fs(mut cx: usb_fs::Context) {
        cx.shared.serial_device.lock(|serial_device| {
            serial_device.poll(|buf| {
                for byte in buf {
                    // ehhh this is prolly fine
                    cx.local.cli_processor_sender.try_send(*byte).ok();
                }
            });
        });
    }

    #[task(priority=2, shared=[serial_device])]
    async fn usb_write(mut cx: usb_write::Context, byte: u8) {
        cx.shared.serial_device.lock(|serial_device| {
            serial_device.write(&[byte]);
        });
    }

    #[task(priority=1, local=[cli])]
    async fn cli_process(cx: cli_process::Context, mut r: Receiver<'static, u8, CLI_PROCESS_LEN>) {
        while let Ok(b) = r.recv().await {
            crate::cli::Base::process_byte(cx.local.cli, b);
        }
    }

    #[task(priority=1, local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis().into()).await;
        }
    }

}
