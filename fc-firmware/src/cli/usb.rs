
use static_cell::{ConstStaticCell, StaticCell};
use stm32f4xx_hal::otg_fs::{USB, UsbBus, UsbBusType};
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

const USB_MANUFACTURER_STRING: &str = "Society for Advanced Rocket Propulsion";
const USB_PRODUCT_STRING: &str = "Airbrakes Flight Computer";
const USB_SERIAL_NUMBER_STRING: &str = env!("CARGO_PKG_VERSION");

pub struct UsbSerialDevice {
    usb_dev: UsbDevice<'static, UsbBusType>,
    usb_serial: SerialPort<'static, UsbBusType>,
}

impl UsbSerialDevice {
    pub fn new(usb: USB) -> Self {
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

    pub fn poll(&mut self, cb: impl FnOnce(&[u8])) {
        let usb_dev = &mut self.usb_dev;
        let usb_serial = &mut self.usb_serial;

        if usb_dev.poll(&mut [usb_serial]) {
            let mut buf = [0u8; 64];

            match usb_serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    cb(&buf[0..count]);
                }
                _ => {}
            }
        }
    }

    pub fn write(&mut self, buf: &[u8]) {
        while let Err(_) = self.usb_serial.write(buf) {}
    }
}

