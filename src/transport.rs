extern crate alloc;

use crate::error::Error;
use alloc::boxed::Box;

#[cfg(feature = "async")]
#[cfg(feature = "shared-bus")]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
#[cfg(feature = "async")]
pub use embedded_hal_async::i2c::I2c as AsyncI2c;

#[cfg_attr(feature = "async", async_trait::async_trait(?Send))]
pub trait Transport {
    /// Write bytes to a device register.
    ///
    /// # Arguments
    ///
    /// - `reg` (`u8`) - Register address to write to.
    /// - `data` (`&[u8]`) - Bytes to write.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok() or `Error::BusError`.
    #[cfg(feature = "async")]
    async fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error>;

    #[cfg(feature = "blocking")]
    fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error>;

    /// Read bytes from a device register.
    ///
    /// # Arguments
    ///
    /// - `reg` (`u8`) - First register address to read from.
    /// - `buf` (`&mut [u8]`) - Destination buffer to fill.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok() or `Error::BusError`.
    #[cfg(feature = "async")]
    async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error>;

    #[cfg(feature = "blocking")]
    fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error>;
}

#[cfg(feature = "async")]
pub struct I2cTransport<I2C> {
    #[cfg(not(feature = "shared-bus"))]
    pub bus: I2C,

    #[cfg(feature = "shared-bus")]
    pub bus: Mutex<CriticalSectionRawMutex, I2C>,

    pub address: u8,
}

#[cfg(feature = "async")]
impl<I2C> I2cTransport<I2C> {
    /// Create a new async I2C transport wrapper using the provided I2C instance and device address.
    ///
    /// # Arguments
    ///
    /// - `i2c` (`I2C`) - Async I2C instance implementing `embedded_hal_async::i2c::I2c`.
    /// - `address` (`u8`) - 7-bit device address.
    ///
    /// # Returns
    ///
    /// - `Self` - A new `I2cTransport` wrapping the `i2c` instance.
    #[cfg(not(feature = "shared-bus"))]
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { bus: i2c, address }
    }

    /// Create a new async I2C transport wrapper using the provided Mutex wrapped I2C instance and
    /// device address.
    ///
    /// # Arguments
    ///
    /// - `i2c` (`Mutex<CriticalSectionRawMutex, I2C>`) - Async Mutex wrapped I2C instance
    /// implementing `embedded_hal_async::i2c::I2c`.
    /// - `address` (`u8`) - 7-bit device address.
    ///
    /// # Returns
    ///
    /// - `Self` - A new `I2cTransport` wrapping the Mutex'd `i2c` instance.
    #[cfg(feature = "shared-bus")]
    pub fn new(i2c: Mutex<CriticalSectionRawMutex, I2C>, address: u8) -> Self {
        Self { bus: i2c, address }
    }
}

#[cfg(feature = "async")]
#[async_trait::async_trait(?Send)]
impl<I2C, E> Transport for I2cTransport<I2C>
where
    I2C: AsyncI2c<Error = E>,
{
    async fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.len() > 255 {
            return Err(Error::BusError);
        }

        let mut buf = [0u8; 256];
        buf[0] = reg;
        buf[1..=data.len()].copy_from_slice(data);

        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .write(self.address, &buf[..=data.len()])
                .await
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus
                .lock()
                .await
                .write(self.address, &buf[..=data.len()])
                .await
                .map_err(|_| Error::BusError)
        }
    }

    async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .write_read(self.address, &[reg], buf)
                .await
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus
                .lock()
                .await
                .write_read(self.address, &[reg], buf)
                .await
                .map_err(|_| Error::BusError)
        }
    }
}

// TODO: SPI transport is intentionally left unimplemented here — contributions welcome.
