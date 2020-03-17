#![feature(trait_alias)]

use nb;
use stm32f4xx_hal as hal;

pub trait Read {
    /// Read error
    type Error;

    /// Reads a single word from the serial interface
    fn read(&mut self) -> nb::Result<u8, Self::Error>;

    fn read_exact(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        for i in 0..buffer.len() {
            buffer[i] = nb::block!(self.read())?;
        }

        Ok(())
    }
}

pub trait Write {
    /// Write error
    type Error;

    /// Writes a single byte to the serial interface
    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error>;

    /// Ensures that none of the previously written words are still buffered
    fn flush(&mut self) -> nb::Result<(), Self::Error>;

    fn write_all(&mut self, buffer: &[u8]) -> Result<(), Self::Error> {
        for byte in buffer {
            nb::block!(self.write(byte.clone()))?;
        }

        Ok(())
    }
}

pub trait SerialRead = embedded_hal::serial::Read<u8, Error = hal::serial::Error>;
pub trait SerialWrite = embedded_hal::serial::Write<u8, Error = hal::serial::Error>;

pub struct SerialRx<R: SerialRead>(R);

impl<R: SerialRead> Read for SerialRx<R> {
    type Error = hal::serial::Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.0.read()
    }
}

impl<R: SerialRead> SerialRx<R> {
    pub fn new(rx: R) -> Self {
        SerialRx(rx)
    }
}

pub struct SerialTx<W: SerialWrite>(W);

impl<W: SerialWrite> Write for SerialTx<W> {
    type Error = hal::serial::Error;

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.0.write(byte)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.0.flush()
    }
}

impl<W: SerialWrite> SerialTx<W> {
    pub fn new(tx: W) -> Self {
        SerialTx(tx)
    }
}

pub struct Serial<R: SerialRead, W: SerialWrite> {
    rx: SerialRx<R>,
    tx: SerialTx<W>,
}

impl<R: SerialRead, W: SerialWrite> Serial<R, W> {
    pub fn new(rx: R, tx: W) -> Self {
        Serial {
            rx: SerialRx::new(rx),
            tx: SerialTx::new(tx),
        }
    }
}

impl<R: SerialRead, W: SerialWrite> Read for Serial<R, W> {
    type Error = hal::serial::Error;
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl<R: SerialRead, W: SerialWrite> Write for Serial<R, W> {
    type Error = hal::serial::Error;

    fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}
