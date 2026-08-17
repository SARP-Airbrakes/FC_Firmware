
#![no_main]
#![no_std]
#![feature(impl_trait_in_assoc_type)]

use core::{convert::Infallible, slice, sync::atomic::{AtomicUsize, Ordering}};

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::{join::join, select::{Either, select}};
use embassy_stm32::{Config, bind_interrupts, peripherals, time::mhz, usb::{self, Driver, Instance}};
use embassy_sync::{blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex}, mutex::Mutex, pipe};
use embassy_usb::{Builder, class::cdc_acm::{CdcAcmClass, Receiver, Sender, State}, driver::EndpointError};
use embedded_cli::cli::CliBuilder;
use ufmt::uwriteln;
use panic_probe as _;
use defmt_rtt as _;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

type ConsolePipe = pipe::Pipe<CriticalSectionRawMutex, 64>;
static CONSOLE_READ_PIPE: ConsolePipe = ConsolePipe::new();
static CONSOLE_WRITE_PIPE: ConsolePipe = ConsolePipe::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut cfg = Config::default();

    // Configure clocks
    {
        use embassy_stm32::rcc::*;

        // Closely matched with the solved configuration from CubeMX
        cfg.rcc.hse = Some(Hse {
            freq: mhz(16),
            mode: HseMode::Oscillator,
        });

        cfg.rcc.pll_src = PllSource::HSE;
        cfg.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV8,
            mul: PllMul::MUL72,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV3), // for 48 MHz clocks
            divr: None, // not using I2S
        });
        cfg.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;

        cfg.rcc.apb1_pre = APBPrescaler::DIV1; // PCLK1 = 16MHz
        cfg.rcc.apb2_pre = APBPrescaler::DIV1; // PCLK2 = 16MHz
        cfg.rcc.ahb_pre = AHBPrescaler::DIV1; // HCLK = 16MHz

        cfg.rcc.sys = Sysclk::HSI;
    }
    let p = embassy_stm32::init(cfg);

    let mut ep_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // The airbrakes are self-powered but PA9 is not connected to VBUS (on the
    // 2025-2026 revision of the PCB).
    config.vbus_detection = false;

    let driver = Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, &mut ep_buffer, config);

    let mut config = embassy_usb::Config::new(0x0483, 0x5740);
    config.manufacturer = Some("Society for Advanced Rocket Propulsion");
    config.product = Some("Airbrakes Flight Computer");
    config.serial_number = Some(env!("CARGO_PKG_VERSION"));

    let mut config_descriptor = [0u8; 256];
    let mut bos_descriptor = [0u8; 256];
    let mut control_buf = [0u8; 256];

    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf
    );

    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let (mut sender, mut receiver) = class.split();
    let mut usb = builder.build();
    let usb_fut = usb.run();

    let echo_fut = async {
        loop {
            receiver.wait_connection().await;
            debug!("Got connection.");
            let _ = process_console(&mut sender, &mut receiver).await;
            debug!("Disconnected.");
        }
    };

    spawner.spawn(console_execute().unwrap());

    join(usb_fut, echo_fut).await;
}

#[derive(embedded_cli::Command)]
enum Base {
    Version,
}

#[embassy_executor::task]
async fn console_execute() {
    let dropped = AtomicUsize::new(0);
    let writer = PipeWriter::new(&CONSOLE_WRITE_PIPE, &dropped);

    let mut cli = CliBuilder::default()
        .writer(writer)
        .build()
        .unwrap();

    loop {
        let dropped = dropped.swap(0, Ordering::AcqRel);
        if dropped > 0 {
            warn!("{} bytes dropped from console!", dropped);
        }

        let mut c = 0u8;
        CONSOLE_READ_PIPE.read(slice::from_mut(&mut c)).await;

        let _ = cli.process_byte::<Base, _>(
            c,
            &mut Base::processor(|cli, command| {
                match command {
                    Base::Version => {
                        uwriteln!(cli.writer(), "Airbrakes flight computer");
                        uwriteln!(cli.writer(), "Version {}", env!("CARGO_PKG_VERSION"));
                    },
                    _ => {},
                };
                Ok(())
            })
        );

    }
}

/// A blocking writer that wraps around a pipe synchronization primitive.
/// Reports how many bytes have been lost.
struct PipeWriter<'a> {
    pipe: &'static ConsolePipe,
    dropped: &'a AtomicUsize,
}

impl<'a> PipeWriter<'a> {

    pub fn new(pipe: &'static ConsolePipe, dropped: &'a AtomicUsize) -> Self {
        Self {
            pipe,
            dropped
        }
    }
}

impl<'a> embedded_io::ErrorType for PipeWriter<'a> {
    type Error = Infallible;
}

impl<'a> embedded_io::Write for PipeWriter<'a> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match self.pipe.try_write(buf) {
            Ok(n) => Ok(n),
            Err(_) => {
                self.dropped.fetch_add(buf.len(), Ordering::AcqRel);
                Ok(buf.len()) // report ok nonetheless
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Disconnected {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            _ => Disconnected {},
        }
    }
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
                CONSOLE_READ_PIPE.write(&buf[..n]).await;
            }
            Ok::<(), Disconnected>(())
        },
        async {
            loop {
                let mut buf = [0u8; 64];
                let n = CONSOLE_WRITE_PIPE.read(&mut buf).await;
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