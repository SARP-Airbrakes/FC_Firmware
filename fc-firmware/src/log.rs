//! This module comprises the serialization and de-serialization logic for the
//! on-board flight log.
//!
//! The flight log is for the storage and retrieval of flight data (inertial
//! characteristics of the rocket, control and filter values, environmental
//! conditions, etc.) in a uniform, accessible manner. For the relevant rocket
//! requirements, see ARBK-5 and ARBK-6 (located on the SARP Drive).
//!
//! For storage, the on-board W25Q128JV is used. The flight log stores a header
//! in the first 4096 bytes of the flash memory (see [`LogHeader`]). Packets are
//! stored unaligned, have a computable size and read in a stream from the
//! memory. Packets are of several types (see [`Packet`]), instead denoting
//! "events" (deltas in state) rather than the entire state.

use embassy_time::Instant;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi};
use heapless::String;
use postcard::accumulator::{CobsAccumulator, FeedResult};
use serde::{Deserialize, Serialize};
use w25qxxxjv::{W25qxxxjv, Wusize};

/// A constant present in the header to both version the data and detect
/// corruption.
const LOG_MAGIC_CONSTANT: &'static str = concat!("FLIGHTLOG V1");

/// A header placed at the very start of memory, storing persistent state
/// between boots.
#[derive(Serialize, Deserialize)]
pub struct LogHeader {
    /// Magic constant (see [`LOG_MAGIC_CONSTANT`]).
    magic: [u8; LOG_MAGIC_CONSTANT.len()],
    /// The position of the next write (address on the W25Q128JV).
    write_cursor: Wusize,
    /// Time that the header was last updated.
    pub last_write: FlightTime,
    /// Total number of packets written.
    pub packet_count: usize,
    /// Name of the flight, for tracking.
    pub flight_name: Option<String<32, u8>>,
}

impl Default for LogHeader {
    fn default() -> Self {
        Self {
            magic: LOG_MAGIC_CONSTANT.as_bytes().try_into().unwrap_or_default(),
            write_cursor: 0x1000,
            last_write: FlightTime::now(),
            packet_count: 0usize,
            flight_name: None,
        }
    }
}

/// A time during flight, measured in milliseconds since boot.
#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Serialize, Deserialize, Debug, defmt::Format)]
pub struct FlightTime(u64);

/// A packet written to the flash memory; loosely representing an event of
/// interest in flight.
#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, defmt::Format)]
pub enum Packet {
    BarometerMeasurement {
        time: FlightTime,
        /// Temperature in Celsius.
        temperature: f32,
        /// Pressure in Pascals.
        pressure: f32,
        /// Altitude in meters.
        altitude: f32,
    },
    AccelerometerMeasurement {
        time: FlightTime,
        /// X-axis acceleration in m/s^2.
        x: f32,
        /// Y-axis acceleration in m/s^2.
        y: f32,
        /// Z-axis acceleration in m/s^2.
        z: f32,
    },
}

/// A wrapper around the flash memory that handles flight log logic.
pub struct FlightLog<'a, S, CS, D> {
    w25: W25qxxxjv<'a, S, CS, D>,
    /// The position of the next read (address on the W25Q128JV).
    read_cursor: Wusize,
    /// Header to be written to the start.
    pub header: LogHeader,
}

/// Error type for [`FlightLog`].
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
    /// Creates a new flight log from a given flash memory.
    pub fn new(w25: W25qxxxjv<'a, S, CS, D>) -> Self {
        Self {
            w25,
            read_cursor: 0x1000,
            header: Default::default()
        }
    }

    /// Consumes the flight log type, returning the interior flash memory.
    pub fn destroy(self) -> W25qxxxjv<'a, S, CS, D> {
        self.w25
    }

    /// Erases the flash memory.
    ///
    /// Waits for the flash memory to be ready to take commands.
    pub async fn erase_chip(&mut self) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_chip().await.map_err(Error::W25)
    }

    /// Erases a 4-kb "sector" of the flash memory.
    ///
    /// "Sector" is terminology of the W25QxxxJV. Waits for the flash memory to
    /// finish erasing.
    pub async fn erase_sector(&mut self, sector: Wusize) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_sector(sector).await.map_err(Error::W25)
    }

    /// Resets the internal state of the flight log, including the header.
    ///
    /// To avoid data-loss, the header should be read again with
    /// [`Self::read_header`].
    pub fn reset(&mut self) {
        self.header = Default::default();
        self.reset_cursor();
    }

    /// Resets the read cursor of the flight log to the first packet.
    pub fn reset_cursor(&mut self) {
        self.read_cursor = 0x1000;
    }

    /// Reads the header from the flash memory.
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

    /// Writes the header to the flash memory.
    pub async fn update_header(&mut self) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        self.w25.erase_sector(0x00).await.map_err(Error::W25)?;
        let mut buf = [0u8; 64];

        self.header.last_write = FlightTime::now();
        let slice = postcard::to_slice(&self.header, &mut buf).map_err(Error::Serde)?;
        self.w25.write_data(0x00, slice).await.map_err(Error::W25)?;
        Ok(())
    }

    /// Reads the packet at the read cursor from the flash memory.
    ///
    /// Moves the cursor to the next packet, allowing for the next packet to be
    /// read.
    pub async fn read_next_packet(&mut self) -> Result<Packet, Error<w25qxxxjv::Error<SE, PE>>> {
        let mut read_buf = [0u8; 32];
        let mut accumulator = CobsAccumulator::<256>::new();

        loop {
            defmt::debug!("Reading at {:x}", self.read_cursor);

            // Read a little bit from where we are positioned.
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

    /// Writes a packet to the flash memory at the next available space.
    pub async fn push_packet(&mut self, packet: Packet) -> Result<(), Error<w25qxxxjv::Error<SE, PE>>> {
        let mut buf = [0u8; 64];
        let slice = postcard::to_slice_cobs(&packet, &mut buf).map_err(Error::Serde)?;
        defmt::debug!("Writing ({:x}): {}", self.header.write_cursor, slice);
        self.w25.write_data(self.header.write_cursor, slice).await.map_err(Error::W25)?;

        self.header.write_cursor += slice.len() as Wusize;
        self.header.packet_count += 1;
        self.update_header().await
    }
}

impl FlightTime {
    
    /// Creates a [`FlightTime`] for the current millisecond.
    pub fn now() -> Self {
        Self(Instant::now().as_millis())
    }

    /// Converts the [`FlightTime`] into a count of milliseconds.
    pub fn as_millis(self) -> u64 {
        self.0
    }
}

impl From<FlightTime> for u64 {
    fn from(value: FlightTime) -> Self {
        value.0
    }
}
