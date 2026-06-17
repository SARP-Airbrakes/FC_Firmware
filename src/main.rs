
#![no_main]
#![no_std]

use panic_halt as _;

use rtic::app;
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::gpio::{PC13, Output, PushPull};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Config;

type Mono = stm32f4xx_hal::timer::monotonics::MonoTimerUs<pac::TIM2>;

#[app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let dp = cx.device;
        let mut rcc = dp.RCC.freeze(Config::hse(25.MHz()).sysclk(48.MHz()));

        dp.TIM2.monotonic_us(&mut cx.core.NVIC, &mut rcc);

        let gpioc = dp.GPIOC.split(&mut rcc);
        let led = gpioc.pc13.into_push_pull_output();

        blink::spawn().ok();

        (
            Shared { },
            Local {
                led
            }
        )
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            cx.local.led.toggle();
            Mono::delay(500.millis().into()).await;
        }
    }

}
