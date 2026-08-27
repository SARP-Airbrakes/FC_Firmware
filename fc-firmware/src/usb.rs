
use defmt::{panic, *};
use embassy_futures::{join::join, select::{Either, select}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe};
use embassy_stm32::{Peri, bind_interrupts, peripherals::{self, USB_OTG_FS}, usb::{DmPin, DpPin, Driver, Instance}};
use embassy_usb::{Builder, class::cdc_acm::{CdcAcmClass, Receiver, Sender, State}, driver::EndpointError};
use static_cell::StaticCell;

pub(crate) type UsbPipe = pipe::Pipe<CriticalSectionRawMutex, 512>;
pub(crate) static USB_READ_PIPE: UsbPipe = UsbPipe::new();
pub(crate) static USB_WRITE_PIPE: UsbPipe = UsbPipe::new();

bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Disconnected {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            _ => Disconnected {},
        }
    }
}

static USB_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
static USB_STATE: StaticCell<State> = StaticCell::new();

pub fn setup_usb(
    usb: Peri<'static, USB_OTG_FS>,
    dp: Peri<'static, impl DpPin<USB_OTG_FS>>,
    dm: Peri<'static, impl DmPin<USB_OTG_FS>>
) -> impl Future<Output = (!, !)> {
    let mut config = embassy_stm32::usb::Config::default();

    // The airbrakes are self-powered but PA9 is not connected to VBUS (on the
    // 2025-2026 revision of the PCB).
    config.vbus_detection = false;

    // This mess initializes the four buffers as one big buffer instead.
    let cell = USB_BUFFER.init([0u8; 1024]);
    let (half1, half2) = cell.split_at_mut(512);
    let (ep_buffer, config_descriptor) = half1.split_at_mut(256);
    let (bos_descriptor, control_buf) = half2.split_at_mut(256);

    let driver = Driver::new_fs(usb, Irqs, dp, dm, ep_buffer, config);

    let mut config = embassy_usb::Config::new(0x0483, 0x5740);
    config.manufacturer = Some("Society for Advanced Rocket Propulsion");
    config.product = Some("Airbrakes Flight Computer");
    config.serial_number = Some(env!("CARGO_PKG_VERSION"));

    let state = USB_STATE.init(State::new());
    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [],
        control_buf
    );
    let class = CdcAcmClass::new(&mut builder, state, 64);

    join(
        async {
            let mut usb = builder.build();
            loop {
                usb.run_until_suspend().await;
                usb.wait_resume().await;
            }
        }, 
        async {
            let (mut sender, mut receiver) = class.split();
            loop {
                receiver.wait_connection().await;
                debug!("Got connection.");
                let _ = process_console(&mut sender, &mut receiver).await;
                debug!("Disconnected.");
            }
        }
    )
}

async fn process_console<'d, T: Instance + 'd>(
    sender: &mut Sender<'d, Driver<'d, T>>, 
    receiver: &mut Receiver<'d, Driver<'d, T>>
) -> Result<(), Disconnected> {
    #[allow(unreachable_code, reason = "Async Result return")]
    let res = select(
        async {
            loop {
                let mut buf = [0u8; 64];
                let n = receiver.read_packet(&mut buf).await?;
                USB_READ_PIPE.write(&buf[..n]).await;
            }
            Ok::<(), Disconnected>(())
        },
        async {
            loop {
                let mut buf = [0u8; 64];
                let n = USB_WRITE_PIPE.read(&mut buf).await;
                sender.write_packet(&buf[..n]).await?;
            }
            Ok::<(), Disconnected>(())
        }
    ).await;

    match res {
        Either::First(res) => res,
        Either::Second(res) => res,
    }
}