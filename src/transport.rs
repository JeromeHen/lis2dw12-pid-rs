extern crate alloc;

#[cfg(all(feature = "blocking", feature = "shared-bus"))]
use core::cell::RefCell;

use crate::error::Error;

#[cfg(feature = "async")]
use alloc::boxed::Box;
#[cfg(feature = "blocking")]
pub use embedded_hal::i2c::I2c as BlockingI2c;
#[cfg(feature = "async")]
pub use embedded_hal_async::i2c::I2c as AsyncI2c;

#[cfg(feature = "blocking")]
pub use embedded_hal::spi::SpiBus as BlockingSpi;
#[cfg(feature = "async")]
pub use embedded_hal_async::spi as AsyncSpi;

#[cfg(all(feature = "blocking", feature = "shared-bus"))]
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
#[cfg(feature = "shared-bus")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(all(feature = "async", feature = "shared-bus"))]
use embassy_sync::mutex::Mutex as AsyncMutex;

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
    #[cfg(feature = "blocking")]
    fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error>;
}

#[cfg(any(
    all(feature = "async", not(feature = "shared-bus")),
    all(feature = "blocking", not(feature = "shared-bus"))
))]
type I2cBus<I2C> = I2C;
#[cfg(all(feature = "async", feature = "shared-bus"))]
type I2cBus<I2C> = &'static AsyncMutex<CriticalSectionRawMutex, Option<I2C>>;
#[cfg(all(feature = "blocking", feature = "shared-bus"))]
type I2cBus<I2C> = &'static BlockingMutex<CriticalSectionRawMutex, RefCell<Option<I2C>>>;

pub struct I2cTransport<I2C: 'static> {
    pub bus: I2cBus<I2C>,
    pub address: u8,
}

impl<I2C> I2cTransport<I2C> {
    /// Create a new `I2cTransport` wrapping the provided I2C bus instance and
    /// 7-bit device `address`.
    /// The concrete type of the `i2c` parameter follows the `I2cBus<I2C>` type
    /// alias and therefore depends on enabled Cargo features (see the
    /// `I2cTransport` documentation for exact mappings).
    ///
    /// # Arguments
    ///
    /// - `bus` (`I2cBus<I2C>`) - The I2C bus instance.
    /// - `address` (`u8`) - The 7-bit device address.
    pub fn new(bus: I2cBus<I2C>, address: u8) -> Self {
        Self { bus, address }
    }

    /// Return the configured 7-bit device address.
    pub fn address(&self) -> u8 {
        self.address
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
            let mut bus = self.bus.lock().await;

            let i2c = bus.as_mut().ok_or(Error::BusError)?;
            i2c.write(self.address, &buf[..=data.len()])
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
            let mut bus = self.bus.lock().await;

            let i2c = bus.as_mut().ok_or(Error::BusError)?;
            i2c.write_read(self.address, &[reg], buf)
                .await
                .map_err(|_| Error::BusError)
        }
    }
}

#[cfg(feature = "blocking")]
impl<I2C, E> Transport for I2cTransport<I2C>
where
    I2C: BlockingI2c<Error = E>,
{
    fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
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
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus.lock(|bus| {
                let mut maybe_i2c = bus.borrow_mut();
                let i2c = maybe_i2c.as_mut().ok_or(Error::BusError)?;

                i2c.write(self.address, &buf[..=data.len()])
                    .map_err(|_| Error::BusError)
            })
        }
    }

    fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .write_read(self.address, &[reg], buf)
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus.lock(|bus| {
                let mut maybe_i2c = bus.borrow_mut();
                let i2c = maybe_i2c.as_mut().ok_or(Error::BusError)?;

                i2c.write_read(self.address, &[reg], buf)
                    .map_err(|_| Error::BusError)
            })
        }
    }
}

#[cfg(any(
    all(feature = "async", not(feature = "shared-bus")),
    all(feature = "blocking", not(feature = "shared-bus"))
))]
type SpiBus<SPI> = SPI;
#[cfg(all(feature = "async", feature = "shared-bus"))]
type SpiBus<SPI> = &'static AsyncMutex<CriticalSectionRawMutex, Option<SPI>>;
#[cfg(all(feature = "blocking", feature = "shared-bus"))]
type SpiBus<SPI> = &'static BlockingMutex<CriticalSectionRawMutex, RefCell<Option<SPI>>>;

pub struct SpiTransport<SPI: 'static> {
    pub bus: SpiBus<SPI>,
}

impl<SPI> SpiTransport<SPI> {
    /// Create a new `SpiTransport` wrapping the provided SPI bus instance.
    /// The concrete type of the `bus` parameter follows the `SpiBus<SPI>` type
    /// alias and therefore depends on enabled Cargo features (see the
    /// `SpiTransport` documentation for exact mappings).
    ///
    /// # Arguments
    ///
    /// - `bus` (`SpiBus<SPI>`) - The SPI bus instance.
    pub fn new(bus: SpiBus<SPI>) -> Self {
        Self { bus }
    }
}

#[cfg(feature = "async")]
#[async_trait::async_trait(?Send)]
impl<SPI, E> Transport for SpiTransport<SPI>
where
    SPI: AsyncSpi::SpiBus<Error = E>,
{
    async fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.len() > 255 {
            return Err(Error::BusError);
        }

        let mut buf = [0u8; 256];
        buf[0] = reg & 0x7F; // device-protocol specific
        buf[1..=data.len()].copy_from_slice(data);

        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .write(&buf[..=data.len()])
                .await
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            let mut bus = self.bus.lock().await;

            let spi = bus.as_mut().ok_or(Error::BusError)?;
            spi.write(&buf[..=data.len()])
                .await
                .map_err(|_| Error::BusError)
        }
    }

    async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        if buf.len() > 255 {
            return Err(Error::BusError);
        }

        let mut tx = [0u8; 256];
        tx[0] = reg | 0x80u8; // device-protocol specific read bit

        let tx_slice = &tx[..=buf.len()];
        let mut rx = [0u8; 256];

        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .transfer(&mut rx[..=buf.len()], tx_slice)
                .await
                .map_err(|_| Error::BusError)?;
            buf.copy_from_slice(&rx[1..=buf.len()]);
            Ok(())
        }

        #[cfg(feature = "shared-bus")]
        {
            let mut bus = self.bus.lock().await;

            let spi = bus.as_mut().ok_or(Error::BusError)?;
            spi.transfer(&mut rx[..=buf.len()], tx_slice)
                .await
                .map_err(|_| Error::BusError)?;
            buf.copy_from_slice(&rx[1..=buf.len()]);
            Ok(())
        }
    }
}

#[cfg(feature = "blocking")]
impl<SPI, E> Transport for SpiTransport<SPI>
where
    SPI: BlockingSpi<u8, Error = E>,
{
    fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        if data.len() > 255 {
            return Err(Error::BusError);
        }

        let mut buf = [0u8; 256];
        buf[0] = reg & 0x7F;
        buf[1..=data.len()].copy_from_slice(data);

        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .write(&buf[..=data.len()])
                .map_err(|_| Error::BusError)
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus.lock(|bus| {
                let mut maybe_spi = bus.borrow_mut();
                let spi = maybe_spi.as_mut().ok_or(Error::BusError)?;

                spi.write(&buf[..=data.len()]).map_err(|_| Error::BusError)
            })
        }
    }

    fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        if buf.len() > 255 {
            return Err(Error::BusError);
        }

        let mut tx = [0u8; 256];
        // same note as above: MSB-as-read flag is device-protocol specific
        tx[0] = reg | 0x80u8;

        let len = buf.len();
        let write = &tx[..=len];
        let mut rx = [0u8; 256];

        #[cfg(not(feature = "shared-bus"))]
        {
            self.bus
                .transfer(&mut rx[..=len], write)
                .map_err(|_| Error::BusError)?;
            buf.copy_from_slice(&rx[1..=len]);
            Ok(())
        }

        #[cfg(feature = "shared-bus")]
        {
            self.bus.lock(|bus| {
                let mut maybe_spi = bus.borrow_mut();
                let spi = maybe_spi.as_mut().ok_or(Error::BusError)?;

                spi.transfer(&mut rx[..=len], write)
                    .map_err(|_| Error::BusError)?;
                buf.copy_from_slice(&rx[1..=len]);
                Ok(())
            })
        }
    }
}
