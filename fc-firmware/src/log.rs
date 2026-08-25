use embassy_time::Instant;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi};
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};
use w25qxxxjv::{W25qxxxjv, Wusize};

const LOG_MAGIC_CONSTANT: &'static str = concat!("FLIGHTLOG V1");

/// A header placed at the very start of memory.
#[derive(Serialize, Deserialize)]
struct LogHeader {
    /// Magic constant (see [`LOG_MAGIC_CONSTANT`]).
    magic: [u8; LOG_MAGIC_CONSTANT.len()],
    /// The position of the next write (address on the W25Q128JV).
    write_cursor: Wusize,
    /// Time that the header was last updated.
    last_write: FlightTime,
    /// Total number of packets written.
    packet_count: usize,
}

/// A time during flight, measured in milliseconds since boot.
#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Serialize, Deserialize, Debug, defmt::Format)]
pub struct FlightTime(u64);

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, defmt::Format)]
pub enum Packet {
    TemperaturePressure {
        time: FlightTime,
        /// Temperature in Celsius.
        temperature: f32,
        /// Pressure in Pascals.
        pressure: f32,
    }
}

pub struct FlightLog<'a, S, CS, D> {
    w25: W25qxxxjv<'a, S, CS, D>,
    /// The position of the next read (address on the W25Q128JV).
    read_cursor: Wusize,
    /// Header to be written to the start.
    header: LogHeader,
}

#[derive(Debug, defmt::Format)]
pub enum Error<E> {
    W25(E),
    Serde(postcard::Error),
    Cobs,
    MagicMismatch
}

impl<'a, S, CS, D, SE, PE> FlightLog<'a, S, CS, D>
where
    S: spi::SpiBus<Error = SE>,
    CS: OutputPin<Error = PE>,
    D: DelayNs,
{
    pub fn new(w25: W25qxxxjv<'a, S, CS, D>) -> Self {
        Self {
            w25,
            read_cursor: 0x1000, // packets always start a sector in
            header: LogHeader {
                magic: LOG_MAGIC_CONSTANT.as_bytes().try_into().unwrap_or_default(),
                write_cursor: 0x1000,
                last_write: FlightTime::now(),
                packet_count: 0usize
            }
        }
    }

    pub fn destroy(self) -> W25qxxxjv<'a, S, CS, D> {
        self.w25
    }

    pub async fn erase_chip(&mut self) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_chip().await.map_err(Error::W25)
    }

    pub async fn erase_sector(&mut self, sector: Wusize) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_sector(sector).await.map_err(Error::W25)
    }

    pub fn reset(&mut self) {
        self.header = LogHeader {
            magic: LOG_MAGIC_CONSTANT.as_bytes().try_into().unwrap_or_default(),
            write_cursor: 0x1000,
            last_write: FlightTime::now(),
            packet_count: 0usize
        };
        self.read_cursor = 0x1000;
    }

    pub fn with_w25<T>(&mut self, f: impl FnOnce(&mut W25qxxxjv<'a, S, CS, D>) -> T) -> T {
        f(&mut self.w25)
    }

    pub async fn read_header(&mut self) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        let mut buf = [0u8; 64];
        self.w25.read_data(0x00, &mut buf).await.map_err(Error::W25)?;
        let header = postcard::from_bytes::<LogHeader>(&buf).map_err(Error::Serde)?;
        if header.magic != LOG_MAGIC_CONSTANT.as_bytes() {
            return Err(Error::MagicMismatch);
        }
        self.header = header;
        Ok(())
    }

    pub async fn update_header(&mut self) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_sector(0x00).await.map_err(Error::W25)?;
        let mut buf = [0u8; 64];

        self.header.last_write = FlightTime::now();
        let slice = postcard::to_slice(&self.header, &mut buf).map_err(Error::Serde)?;
        self.w25.write_data(0x00, slice).await.map_err(Error::W25)?;
        Ok(())
    }

    pub async fn read_next_packet(&mut self) -> Result<Packet, Error<w25qxxxjv::Error<SE, PE>>> {
        let mut read_buf = [0u8; 32];
        let mut accumulator = CobsAccumulator::<256>::new();

        loop {
            defmt::debug!("Reading at {:x}", self.read_cursor);
            self.w25.read_data(self.read_cursor, &mut read_buf).await.map_err(Error::W25)?;
            defmt::debug!("Received {}", read_buf);

            let window = &read_buf[..];
            match accumulator.feed(&window) {
                FeedResult::Consumed => {
                    // Move forward and keep reading.
                    // FIXME: Possible issue when we hit the ending boundary of memory.
                    self.read_cursor += read_buf.len() as Wusize;
                },
                FeedResult::DeserError(remaining) => {
                    defmt::debug!("Got a FeedResult::DeserError");
                    self.read_cursor += (read_buf.len() - remaining.len()) as Wusize;
                    return Err(Error::Cobs);
                },
                FeedResult::OverFull(remaining) => {
                    defmt::debug!("Got a FeedResult::OverFull");
                    // Skip the erroneous packet.
                    self.read_cursor += (read_buf.len() - remaining.len()) as Wusize;
                    return Err(Error::Cobs);
                },
                FeedResult::Success { data, remaining } => {
                    self.read_cursor += (read_buf.len() - remaining.len()) as Wusize;
                    return Ok(data);
                }
            }
        }
    }

    pub async fn push_packet(&mut self, packet: Packet) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        let mut buf = [0u8; 64];
        let slice = postcard::to_slice_cobs(&packet, &mut buf).map_err(Error::Serde)?;
        defmt::debug!("Writing ({:x}): {}", self.header.write_cursor, slice);
        self.w25.write_data(self.header.write_cursor, slice).await.map_err(Error::W25)?;

        self.header.write_cursor += slice.len() as Wusize;
        self.header.packet_count += 1;
        self.update_header().await
    }

    pub fn reset_cursor(&mut self) {
        self.read_cursor = 0;
    }
}

impl FlightTime {
    
    pub fn now() -> Self {
        Self(Instant::now().as_millis())
    }
}

impl From<FlightTime> for u64 {
    fn from(value: FlightTime) -> Self {
        value.0
    }
}