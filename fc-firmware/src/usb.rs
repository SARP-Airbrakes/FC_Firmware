//! This module is for the initialization and I/O for the USB CDC ACM device
//! exposed via the USB port present on the flight computer hardware.
//!
//! The USB device is used primarily for the CLI (see [`crate::cli`]), which
//! itself is used for interfacing with the flight log.

use defmt::{panic, *};
use embassy_futures::{join::join, select::{Either, select}};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe};
use embassy_stm32::{Peri, bind_interrupts, peripherals::{self, USB_OTG_FS}, usb::{DmPin, DpPin, Driver, Instance}};
use embassy_usb::{Builder, class::cdc_acm::{CdcAcmClass, Receiver, Sender, State}, driver::EndpointError};
use static_cell::StaticCell;

/// A pipe used for writing (as in, upstream back to a connected computer) to
/// the USB device.
pub(crate) type UsbPipe = pipe::Pipe<CriticalSectionRawMutex, 512>;
/// A pipe used for reading (as in, downstream from a connected computer) from
/// the USB device.
pub(crate) type UsbReadPipe = pipe::Pipe<CriticalSectionRawMutex, 64>;

/// The pipe for data read from the USB.
pub(crate) static USB_READ_PIPE: UsbReadPipe = UsbReadPipe::new();
/// The pipe for writing data to the USB.
pub(crate) static USB_WRITE_PIPE: UsbPipe = UsbPipe::new();

bind_interrupts!(struct Irqs {
    OTG_FS => embassy_stm32::usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

/// Unit error type for handling USB disconnections.
struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Disconnected {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            _ => Disconnected {},
        }
    }
}

/// Statically allocated buffer for the multitude of different buffers required
/// for the initialization of the USB FS device.
static USB_BUFFER: StaticCell<[u8; 1024]> = StaticCell::new();
/// The internal USB CDC state, statically allocated.
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

    // Initialize the driver.
    let driver = Driver::new_fs(usb, Irqs, dp, dm, ep_buffer, config);

    // Initialize the USB device with branding.
    // The VID (vendor ID) and PID (product ID) are sourced from the STM32CubeMX
    // default configuration.
    let mut config = embassy_usb::Config::new(0x0483, 0x5740);
    config.manufacturer = Some("Society for Advanced Rocket Propulsion");
    config.product = Some("Airbrakes Flight Computer");
    config.serial_number = Some(env!("CARGO_PKG_VERSION"));

    // Initialize the state.
    let state = USB_STATE.init(State::new());

    // Build the USB device.
    let mut builder = Builder::new(
        driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [],
        control_buf
    );

    // Initialize the USB CDC ACM.
    let class = CdcAcmClass::new(&mut builder, state, 64);

    join(
        // Connect and run the USB device as a USB CDC ACM.
        async {
            let mut usb = builder.build();
            loop {
                usb.run_until_suspend().await;
                usb.wait_resume().await;
            }
        }, 

        // Wait for connection and read and write from the device.
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

/// Take a reader and writer to a USB CDC ACM device and interface with the
/// corresponding pipes until the master device disconnects.
async fn process_console<'d, T: Instance + 'd>(
    sender: &mut Sender<'d, Driver<'d, T>>, 
    receiver: &mut Receiver<'d, Driver<'d, T>>
) -> Result<(), Disconnected> {
    // Wait until either reading or writing lapse due to disconnection.
    #[allow(unreachable_code, reason = "Async Result return")]
    let res = select(
        // Read from the USB device (received bytes downstream from the
        // connected computer).
        async {
            loop {
                let mut buf = [0u8; 64];
                let n = receiver.read_packet(&mut buf).await?;
                USB_READ_PIPE.write(&buf[..n]).await;
            }
            Ok::<(), Disconnected>(())
        },
        // Write to the USB device (go upstream to the connected computer).
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
