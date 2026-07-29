

pub struct FnWriter(pub fn(&[u8]));

impl usbd_serial::embedded_io::Write for FnWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl usbd_serial::embedded_io::ErrorType for FnWriter {
    type Error = core::convert::Infallible;
}

