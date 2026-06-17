
#![no_main]
#![no_std]

use panic_halt as _;

use static_cell::{ConstStaticCell, StaticCell};
use rtic::app;
use rtic_monotonics::Monotonic;
use stm32f4xx_hal::gpio::{PC13, Output, PushPull};
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::Config;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

type Mono = stm32f4xx_hal::timer::monotonics::MonoTimerUs<pac::TIM2>;

#[app(device = pac, peripherals = true)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: SerialPort<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        led: PC13<Output<PushPull>>
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        static EP_MEMORY: ConstStaticCell<[u32; 1024]> = ConstStaticCell::new([0; 1024]);
        static USB_BUS: StaticCell<usb_device::bus::UsbBusAllocator<UsbBusType>> =
            StaticCell::new();

        let dp = cx.device;
        let mut rcc = dp.RCC.freeze(Config::hse(25.MHz()).sysclk(48.MHz()));

        dp.TIM2.monotonic_us(&mut cx.core.NVIC, &mut rcc);

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let led = gpioc.pc13.into_push_pull_output();

        blink::spawn().ok();

        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into(),
            pin_dp: gpioa.pa12.into(),
            hclk: rcc.clocks.hclk(),
        };
        let usb_bus = USB_BUS.init(UsbBus::new(usb, EP_MEMORY.take()));
        let usb_serial = SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x483, 0x5740))
            .device_class(usbd_serial::USB_CLASS_CDC)
            .strings(&[StringDescriptors::default()
                .manufacturer("Society for Advanced Rocket Propulsion")
                .product("Airbrakes Flight Computer")
                .serial_number("010")])
            .unwrap()
            .build();

        (
            Shared {
                usb_dev,
                usb_serial
            },
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

    #[task(binds=OTG_FS, shared=[usb_dev, usb_serial])]
    fn usb_fs(cx: usb_fs::Context) {
        let usb_fs::SharedResources {
            mut usb_dev,
            mut usb_serial,
            ..
        } = cx.shared;

        (&mut usb_dev, &mut usb_serial).lock(|usb_dev, usb_serial| {
            if usb_dev.poll(&mut [usb_serial]) {
                let mut buf = [0u8; 64];

                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        let mut write_offset = 0;
                        while write_offset < count {
                            match usb_serial.write(&mut buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        });
    }

}
