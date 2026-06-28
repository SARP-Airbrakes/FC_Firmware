
use rtic_sync::channel::Sender;

pub struct ChannelWriter<'a, const CAPACITY: usize>(pub Sender<'a, u8, CAPACITY>);

impl<'a, const CAPACITY: usize> usbd_serial::embedded_io::Write for ChannelWriter<'a, CAPACITY> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut written: usize = 0;
        for &b in buf {
            match self.0.try_send(b) {
                Ok(()) => written += 1,
                _ => {} // TODO
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<'a, const CAPACITY: usize> usbd_serial::embedded_io::ErrorType for ChannelWriter<'a, CAPACITY> {
    // TODO
    type Error = core::convert::Infallible;
}
