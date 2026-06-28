
use rtic_sync::channel::Sender;

#[derive(Debug)]
pub enum ChannelWriterError {
    Full
}

pub struct ChannelWriter<'a, const CAPACITY: usize>(pub Sender<'a, u8, CAPACITY>);

impl<'a, const CAPACITY: usize> usbd_serial::embedded_io::Write for ChannelWriter<'a, CAPACITY> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let mut written: usize = 0;
        for &b in buf {
            match self.0.try_send(b) {
                Ok(()) => written += 1,
                Err(_) => return Err(ChannelWriterError::Full)
            }
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<'a, const CAPACITY: usize> usbd_serial::embedded_io::ErrorType for ChannelWriter<'a, CAPACITY> {
    type Error = ChannelWriterError;
}

impl usbd_serial::embedded_io::Error for ChannelWriterError {
    fn kind(&self) -> usbd_serial::embedded_io::ErrorKind {
        match self {
            ChannelWriterError::Full => usbd_serial::embedded_io::ErrorKind::WriteZero
        }
    }
}
