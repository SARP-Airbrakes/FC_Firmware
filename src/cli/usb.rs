
use rtic::mutex_prelude::*;
use static_cell::{ConstStaticCell, StaticCell};
use stm32f4xx_hal::otg_fs::{USB, UsbBus, UsbBusType};
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;
use rtic_sync::channel::Receiver;

use crate::{
    app::{usb_fs, usb_write}
};

pub const USB_SPLIT_WRITER_LEN: usize = 64; 
const USB_MANUFACTURER_STRING: &str = "Society for Advanced Rocket Propulsion";
const USB_PRODUCT_STRING: &str = "Airbrakes Flight Computer";
const USB_SERIAL_NUMBER_STRING: &str = env!("CARGO_PKG_VERSION");

pub struct UsbSerialDevice {
    usb_dev: UsbDevice<'static, UsbBusType>,
    usb_serial: SerialPort<'static, UsbBusType>,
}

pub fn initialize_usb(usb: USB) -> UsbSerialDevice {
    static EP_MEMORY: ConstStaticCell<[u32; 1024]> = ConstStaticCell::new([0; 1024]);
    static USB_BUS: StaticCell<usb_device::bus::UsbBusAllocator<UsbBusType>> =
        StaticCell::new();

    let usb_bus = USB_BUS.init(UsbBus::new(usb, EP_MEMORY.take()));
    let usb_serial = SerialPort::new(usb_bus);

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x483, 0x5740))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer(USB_MANUFACTURER_STRING)
            .product(USB_PRODUCT_STRING)
            .serial_number(USB_SERIAL_NUMBER_STRING)])
        .unwrap()
        .build();

    UsbSerialDevice { usb_dev, usb_serial }
}

pub fn usb_fs(cx: usb_fs::Context) {
    let mut serial_device = cx.shared.serial_device;

    serial_device.lock(|serial_device| {
        let usb_dev = &mut serial_device.usb_dev;
        let usb_serial = &mut serial_device.usb_serial;

        if usb_dev.poll(&mut [usb_serial]) {
            let mut buf = [0u8; 64];

            match usb_serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    let _ = crate::app::cli_process::spawn(buf, count);
                }
                _ => {}
            }
        }
    });
}

pub async fn usb_write(cx: usb_write::Context<'_>, mut receiver: Receiver<'static, u8, USB_SPLIT_WRITER_LEN>) {
    let mut serial_device = cx.shared.serial_device;

    while let Ok(b) = receiver.recv().await {
        serial_device.lock(|serial_device| {
            while let Err(_) = serial_device.usb_serial.write(&[b]) {}
        });
    }
}
