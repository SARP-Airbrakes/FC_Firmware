use rtic::Mutex;
use rtic_sync::{
    channel::{
        Receiver,
        Sender
    },
    make_channel
};

pub use crate::filter::measurement::*;
pub use crate::filter::sensor::*;

use crate::app::filter_process;

mod measurement;
mod sensor;

pub const FILTER_MEASUREMENT_QUEUE_SIZE: usize = 4;

pub type FilterReceiver = Receiver<'static, Measurement, FILTER_MEASUREMENT_QUEUE_SIZE>;
pub type FilterSender = Sender<'static, Measurement, FILTER_MEASUREMENT_QUEUE_SIZE>;

pub struct Filter {
    sender: FilterSender,
}

impl Filter {
    pub fn new() -> (Self, Receiver<'static, Measurement, FILTER_MEASUREMENT_QUEUE_SIZE>) {
        let (s, r) = make_channel!(Measurement, FILTER_MEASUREMENT_QUEUE_SIZE);
        (
            Filter {
                sender: s
            },
            r
        )
    }
    
    pub fn recv(&mut self, m: Measurement) {

    }
    
    pub fn split(&self) -> Sender<'static, Measurement, FILTER_MEASUREMENT_QUEUE_SIZE> {
        self.sender.clone()
    }
}

pub async fn filter_process(mut cx: filter_process::Context<'_>, mut receiver: FilterReceiver) {
    loop {
        if let Ok(m) = receiver.recv().await {
            cx.shared.filter.lock(|filter| {
                filter.recv(m);
            });
        }
    }
}

