//! A platform agnostic driver to interface with the MMA8452Q 3-Axis Accelerometer via I2C.
//! [embedded-hal] and implements the [`Accelerometer` trait][trait]
//! from the `accelerometer` crate.
//! This chip can be found on Sparkfun's MMA8452Q breakout board.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//!

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

mod reg;

use core::fmt::Debug;

#[cfg(feature = "out_f32")]
pub use accelerometer::vector::F32x3;
pub use accelerometer::{RawAccelerometer, Accelerometer, Error, ErrorKind, vector::I16x3};
use cast::u16;
use embedded_hal as hal;
use hal::blocking::i2c::{Write, WriteRead};

use crate::reg::*;
pub use crate::reg::{ FullScale, Mode, Odr};

/// Possible slave addresses
pub enum SlaveAddr {
    /// Default slave address
    Default,
    /// Alternative slave address providing bit value for `A0`
    Alternative(bool),
}

impl SlaveAddr {
    fn addr(self) -> u8 {
        match self {
            SlaveAddr::Default => I2C_SAD,
            SlaveAddr::Alternative(a0) => I2C_SAD | a0 as u8,
        }
    }
}

/// Data status structure,
/// decoded from STATUS_REG register
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,
    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),
    /// ZYXDA bit
    pub zyxda: bool,
    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

/// `MMA8452q` driver
pub struct MMA8452q<I2C> {
    /// The concrete I²C device implementation
    i2c: I2C,
    /// The I²C device slave address
    addr: u8,
    /// Current full-scale
    fs: FullScale,
    /// Current data rate
    odr: Odr,
}

impl<I2C, E> MMA8452q<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Create a new `MMA84` driver from the given `I2C` peripheral
    pub fn new(i2c: I2C, addr: SlaveAddr) -> Result<Self, Error<E>> {
        let mut dev = Self {
            i2c,
            fs: FullScale::G2,
            addr: addr.addr(),
            odr: Odr::Hz200,
        };

        // Ensure we have the correct device ID
        if dev.get_device_id()? != DEVICE_ID {
            ErrorKind::Device.err()?;
        }


        dev.set_odr(Odr::Hz200)?;
        dev.set_fs(FullScale::G2)?;

        Ok(dev)
    }

    /// Destroy driver instance, return `I2C` bus instance
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// `WHO_AM_I` register
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(Register::WHO_AM_I).map_err(Into::into)
    }

    /// Operating mode selection,
    /// `CTRL_REG2`: `MODS` bit,
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.modify_reg(Register::CTRL_REG2 , |v| (v & !MOD) | ((mode as u8) << 0))?;
        Ok(())
    }

    /// Data rate selection,
    /// `CTRL_REG1`: `ODR`
    pub fn set_odr(&mut self, odr: Odr) -> Result<(), Error<E>> {
        self.modify_reg(Register::CTRL_REG1, |v| {
            (v & !ODR_MASK) | ((odr as u8) << 3)
        })?;
        self.odr = odr;
        // By design, when the device from high-resolution configuration (HR) is set to power-down mode (PD),
        // it is recommended to read register REFERENCE (26h) for a complete reset of the filtering block
        // before switching to normal/high-performance mode again.
        /*if let Odr::PowerDown = odr {
            self.get_ref()?;
        }*/
        Ok(())
    }

    /// Set Standby Mode
    ///	Sets the MMA8452 to standby mode. It must be in standby to change most register settings
    pub fn standby(&mut self) -> Result<(), Error<E>> {
        self.reg_reset_bits(Register::CTRL_REG1, STANDBY)?;
        Ok(())
    }

    /// Set Active Mode
    ///	Sets the MMA8452 to active mode. Needs to be in this mode to output data
    pub fn active(&mut self) -> Result<(), Error<E>> {
        self.reg_set_bits(Register::CTRL_REG1, STANDBY)?;
        Ok(())
    }

    /// Check State (ACTIVE or STANDBY)
    ///	Returns true if in Active State, otherwise return false
    pub fn is_active(&mut self) -> Result<bool, Error<E>> {  
        let current_state: u8 = self.read_reg(Register::SYSMOD)?;
        let current_state: bool = (current_state & SYSMOD_ACTIVE) > 0;

        Ok(current_state)
    }

    /// Full-scale selection,
    /// `XYZ_DATA_CFG`: `FS`
    pub fn set_fs(&mut self, fs: FullScale) -> Result<(), Error<E>> {
        self.modify_reg(Register::XYZ_DATA_CFG , |v| (v & !FS_MASK) | ((fs as u8) << 0))?;
        self.fs = fs;
        Ok(())
    }

    /// Reboot memory content,
    /// `CTRL_REG2`: `RST`
    pub fn reboot(&mut self, reboot: bool) -> Result<(), Error<E>> {
        self.reg_xset_bits(Register::CTRL_REG2, RST, reboot)?;
        Ok(())
    }
  
    /// Data status,
    /// `STATUS_REG`: as
    /// DataStatus {zyxor: `ZYXOR`, xyzor: (`XOR`, `YOR`, `ZOR`), zyxda: `ZYXDA`, xyzda: (`XDA`, `YDA`, `ZDA`)}
    pub fn get_status(&mut self) -> Result<DataStatus, Error<E>> {
        let reg = self.read_reg(Register::STATUS)?;
        Ok(DataStatus {
            zyxor: (reg & ZYXOR) != 0,
            xyzor: ((reg & XOR) != 0, (reg & YOR) != 0, (reg & ZOR) != 0),
            zyxda: (reg & ZYXDA) != 0,
            xyzda: ((reg & XDA) != 0, (reg & YDA) != 0, (reg & ZDA) != 0),
        })
    }

    /// Dump registers
    #[cfg(debug_assertions)]
    pub fn dump_regs<W>(&mut self, w: &mut W) -> Result<(), Error<E>>
    where
        W: core::fmt::Write,
    {
        writeln!(
            w,
            "CTRL_REG1 (20h) = {:#010b}",
            self.read_reg(Register::CTRL_REG1)?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG3 (22h) = {:#010b}",
            self.read_reg(Register::CTRL_REG3)?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG4 (23h) = {:#010b}",
            self.read_reg(Register::CTRL_REG4)?
        )
        .unwrap();
        writeln!(
            w,
            "CTRL_REG5 (24h) = {:#010b}",
            self.read_reg(Register::CTRL_REG5)?
        )
        .unwrap();
        Ok(())
    }

    #[inline]
    fn read_reg(&mut self, reg: Register) -> Result<u8, E> {
        let mut buf = [0u8];
        self.i2c.write_read(self.addr, &[reg.addr()], &mut buf)?;
        Ok(buf[0])
    }

    #[inline]
    fn read_regs(&mut self, reg: Register, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c
            .write_read(self.addr, &[reg.addr()], buffer)
    }

    #[inline]
    fn write_reg(&mut self, reg: Register, val: u8) -> Result<(), E> {
        self.i2c.write(self.addr, &[reg.addr(), val])
    }

    #[inline]
    fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), E>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg)?;
        self.write_reg(reg, f(r))?;
        Ok(())
    }

    #[inline]
    fn reg_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), E> {
        self.modify_reg(reg, |v| v | bits)
    }

    #[inline]
    fn reg_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), E> {
        self.modify_reg(reg, |v| v & !bits)
    }

    #[inline]
    fn reg_xset_bits(&mut self, reg: Register, bits: u8, set: bool) -> Result<(), E> {
        if set {
            self.reg_set_bits(reg, bits)
        } else {
            self.reg_reset_bits(reg, bits)
        }
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for MMA8452q<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = E;

    /// Get the raw unscaled acceleration readings from the accelerometer
    fn accel_raw(&mut self) -> Result<I16x3, Error<E>> {
        let mut buf = [0u8; 6];
        self.read_regs(Register::OUT_X_MSB, &mut buf)?;

        Ok(I16x3::new(
            (u16(buf[1]) + (u16(buf[0]) << 8)) as i16,
            (u16(buf[3]) + (u16(buf[2]) << 8)) as i16,
            (u16(buf[5]) + (u16(buf[4]) << 8)) as i16,
        ))
    }
}

#[cfg(feature = "out_f32")]
impl<I2C, E> Accelerometer for MMA8452q<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    type Error = E;

    /// Get the scaled acceleration reading from the accelerometer
    fn accel_norm(&mut self) -> Result<F32x3, Error<E>> {
        let acc: I16x3 = self.accel_raw()?;

        Ok(F32x3::new(
            self.fs.convert_out_i16tof32(acc.x),
            self.fs.convert_out_i16tof32(acc.y),
            self.fs.convert_out_i16tof32(acc.z),
        ))
    }

    /// Get the current sample rate
    fn sample_rate(&mut self) -> Result<f32, Error<E>> {
        match self.odr {
                Odr::Hz800 => Ok(800.0),
                Odr::Hz400 => Ok(400.0),
                Odr::Hz200 => Ok(200.0),
                Odr::Hz100 => Ok(100.0),
                Odr::Hz50 => Ok(50.0),
                Odr::Hz12 => Ok(12.0),
                Odr::Hz6 => Ok(6.0),
                Odr::Hz1 => Ok(2.0),
        }

    }
}
