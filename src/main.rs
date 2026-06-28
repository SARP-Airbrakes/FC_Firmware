
#![no_main]
#![no_std]

mod cli;

use panic_semihosting as _;

use rtic::app;
use rtic_sync::{channel::Receiver, make_channel};
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::gpio::{PB2, Output, PushPull};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Config;
use stm32f4xx_hal::otg_fs::USB;

use cli::{
    Cli,
    usb::{usb_fs, usb_write, USB_SPLIT_WRITER_LEN, UsbSerialDevice}
};

type Mono = stm32f4xx_hal::timer::monotonics::MonoTimerUs<pac::TIM2>;

#[app(device = pac, peripherals = true)]
mod app {
    
    use super::*;

    #[shared]
    struct Shared {
        serial_device: UsbSerialDevice,
        cli: Cli,
    }

    #[local]
    struct Local {
        led: PB2<Output<PushPull>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {

        let dp = cx.device;
        let mut rcc = dp.RCC.freeze(Config::hse(16.MHz()).sysclk(48.MHz()));

        dp.TIM2.monotonic_us(&mut cx.core.NVIC, &mut rcc);

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
        let (s, r) = make_channel!(u8, USB_SPLIT_WRITER_LEN);
        let cli = cli::Cli::new(s);

        usb_write::spawn(r).unwrap();
        
        (
            Shared {
                serial_device,
                cli
            },
            Local {
                led,
            }
        )
    }

    extern "Rust" {
        #[task(binds=OTG_FS, shared=[serial_device, cli])]
        fn usb_fs(cx: usb_fs::Context);

        #[task(shared=[serial_device])]
        async fn usb_write(cx: usb_write::Context, mut receiver: Receiver<'static, u8, USB_SPLIT_WRITER_LEN>);
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis().into()).await;
        }
    }

}
