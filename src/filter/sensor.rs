use rtic_sync::channel::{NoReceiver, Sender, TrySendError};
use rtic_monotonics::{Monotonic, TimerQueueBasedMonotonic, fugit::HertzU32 as Hertz};

use crate::{Mono, filter::{
    FILTER_MEASUREMENT_QUEUE_SIZE, Filter, Measurement
}};

pub enum SensorKick {
    Idle,
    Kick
}

pub enum SensorError {
    NoReceiver(NoReceiver<Measurement>),
    TrySendError(TrySendError<Measurement>)
}

pub struct Sensor {
    target_frequency: Hertz,
    last_kick: <Mono as TimerQueueBasedMonotonic>::Instant,
    sender: Sender<'static, Measurement, FILTER_MEASUREMENT_QUEUE_SIZE>
}

impl Sensor {
    pub fn new(target_frequency: Hertz, filter: &Filter) -> Self {
        Sensor {
            target_frequency,
            last_kick: Mono::now(),
            sender: filter.split()
        }
    }

    pub fn kick(&mut self) -> SensorKick {
        let new_kick = Mono::now();
        let duration = new_kick - self.last_kick;
        self.last_kick = new_kick;

        if let Some(target_duration) = self.target_frequency.try_into_duration::<1, 1_000_000>() {
            if duration > target_duration {
                SensorKick::Kick
            } else {
                SensorKick::Idle
            }
        } else {
            SensorKick::Idle
        }
    }

    pub async fn send(&mut self, m: Measurement) -> Result<(), SensorError> {
        self.sender.send(m).await.map_err(SensorError::NoReceiver)
    }

    pub fn try_send(&mut self, m: Measurement) -> Result<(), SensorError> {
        self.sender.try_send(m).map_err(SensorError::TrySendError)
    }
}


