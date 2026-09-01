use clap::ValueEnum;
use csv::StringRecordsIter;
use std::fs::File;

#[derive(Copy, Clone, ValueEnum)]
pub enum DataFormat {
    /// Auto-detect the flight data type from the .csv header.
    Detect,
    /// Pre-Kalman filter implementation flight data (e.g. April 11th flight data).
    Pre,
    /// Post-Kalman filter implementation flight data (e.g. IREC 2026 flight data).
    Post,
}

pub struct FlightPacket {
    pub time_s: f64,
    pub accel_x_mps2: f32,
    pub accel_y_mps2: f32,
    pub accel_z_mps2: f32,
    pub pressure_pa: f32,
}

#[derive(Debug)]
pub enum Error {
    Csv(csv::Error),
    Detect,
    ParseFloat(std::num::ParseFloatError),
    Parse,
}

pub struct FlightPacketIterator<'a, R> {
    iterator: StringRecordsIter<'a, R>,
    format: DataFormat,
    after: f64,
}

pub struct FlightData<R> {
    reader: csv::Reader<R>,
    format: DataFormat,
    start: csv::Position,
    after: f64,
}

impl From<csv::Error> for Error {
    fn from(value: csv::Error) -> Self {
        Self::Csv(value)
    }
}

impl From<std::num::ParseFloatError> for Error {
    fn from(value: std::num::ParseFloatError) -> Self {
        Self::ParseFloat(value)
    }
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Csv(e) => e.fmt(f),
            Error::Detect => write!(f, "Failed to detect data format from header"),
            Error::ParseFloat(e) => e.fmt(f),
            Error::Parse => write!(f, "Failed to parse flight packets"),
        }
    }
}

impl std::error::Error for Error {}

impl<'a, R: std::io::Read> FlightPacketIterator<'a, R> {
    
    fn new(iterator: StringRecordsIter<'a, R>, format: DataFormat) -> Self {
        Self {
            iterator,
            format,
            after: 0.0
        }
    }

    fn with_after(mut self, after: f64) -> Self {
        self.after = after;
        self
    }

    fn into_packet(&self, record: csv::StringRecord) -> Result<FlightPacket, Error> {
        Ok(FlightPacket {
            time_s: record.get(0).ok_or(Error::Parse)?.parse()?,
            accel_x_mps2: record.get(2).ok_or(Error::Parse)?.parse()?,
            accel_y_mps2: record.get(3).ok_or(Error::Parse)?.parse()?,
            accel_z_mps2: record.get(4).ok_or(Error::Parse)?.parse()?,
            pressure_pa: record.get(match self.format {
                DataFormat::Pre => 15,
                DataFormat::Post => 16,
                _ => return Err(Error::Parse)
            }).ok_or(Error::Parse)?.parse()?,
        })
    }
}

impl<'a, R: std::io::Read> Iterator for FlightPacketIterator<'a, R> {
    type Item = Result<FlightPacket, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let r = self.iterator.next();
            if let Some(r) = r {
                if let Ok(r) = r {
                    let packet = self.into_packet(r);
                    if let Ok(packet) = packet {
                        if packet.time_s >= self.after {
                            return Some(Ok(packet))
                        }
                    } else {
                        return packet.err().map(Err)
                    }
                } else {
                    return r.err().map(Error::Csv).map(Err)
                }
                
            } else { 
                return None
            }
        }
    }
}

impl FlightData<File> {

    pub fn from_flags(flags: crate::DataFlags) -> Result<Self, Error> {
        let mut out = Self {
            reader: csv::Reader::from_path(flags.file).map_err(Error::Csv)?,
            format: flags.input_format,
            start: csv::Position::new(),
            after: flags.after.unwrap_or(0.0),
        };

        match out.format {
            DataFormat::Detect => { out.detect()?; },
            _ => {}
        };
        out.start = out.reader.position().clone();
        Ok(out)
    }
}

impl<R: std::io::Read + std::io::Seek> FlightData<R> {

    /// Automatically detects the format of this flight data
    pub fn detect(&mut self) -> Result<DataFormat, Error> {
        let header = self.reader.headers()?;
        self.format = if header.get(8).ok_or(Error::Detect)? == "acc_altitude_m" {
            log::debug!("Detected version Pre");
            DataFormat::Pre
        } else {
            log::debug!("Detected version Post");
            DataFormat::Post
        };
        Ok(self.format)
    }

    /// Returns a count of how many packets there are in this flight data.
    pub fn count(&mut self) -> usize {
        self.reader.records().count()
    }

    pub fn reset(&mut self) -> Result<(), Error> {
        self.reader.seek(self.start.clone()).map_err(Error::Csv)
    }

    pub fn packets(&mut self) -> FlightPacketIterator<'_, R> {
        FlightPacketIterator::new(
            self.reader.records(), 
            self.format
        ).with_after(self.after)
    }
}