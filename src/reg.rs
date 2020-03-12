#![allow(non_upper_case_globals)]

#[cfg(feature = "out_f32")]
use cast::f32;

/// I2C slave address
pub const I2C_SAD: u8 = 0x1c;

/// Oversampling Mode
pub enum Mode {
    /// Oversampling Mode
    /// Normal
    Normal = 0b0000_0000,
    /// Low Noise
    LowNoise = 0b0000_0001,
    /// Hi Res
    HighResolution = 0b0000_0010,
    /// Low Power
    LowPower = 0b0000_0011,
}

/// Register mapping
#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
pub enum Register {
	STATUS = 0x00,
	OUT_X_MSB = 0x01,
	OUT_X_LSB = 0x02,
	OUT_Y_MSB = 0x03,
	OUT_Y_LSB = 0x04,
	OUT_Z_MSB = 0x05,
	OUT_Z_LSB = 0x06,
	SYSMOD = 0x0B,
	INT_SOURCE = 0x0C,
	WHO_AM_I = 0x0D,
	XYZ_DATA_CFG = 0x0E,
	HP_FILTER_CUTOFF = 0x0F,
	PL_STATUS = 0x10,
	PL_CFG = 0x11,
	PL_COUNT = 0x12,
	PL_BF_ZCOMP = 0x13,
	P_L_THS_REG = 0x14,
	FF_MT_CFG = 0x15,
	FF_MT_SRC = 0x16,
	FF_MT_THS = 0x17,
	FF_MT_COUNT = 0x18,
	TRANSIENT_CFG = 0x1D,
	TRANSIENT_SRC = 0x1E,
	TRANSIENT_THS = 0x1F,
	TRANSIENT_COUNT = 0x20,
	PULSE_CFG = 0x21,
	PULSE_SRC = 0x22,
	PULSE_THSX = 0x23,
	PULSE_THSY = 0x24,
	PULSE_THSZ = 0x25,
	PULSE_TMLT = 0x26,
	PULSE_LTCY = 0x27,
	PULSE_WIND = 0x28,
	ASLP_COUNT = 0x29,
	CTRL_REG1 = 0x2A,
	CTRL_REG2 = 0x2B,
	CTRL_REG3 = 0x2C,
	CTRL_REG4 = 0x2D,
	CTRL_REG5 = 0x2E,
	OFF_X = 0x2F,
	OFF_Y = 0x30,
	OFF_Z = 0x31
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}



// === WHO_AM_I (0Fh) ===

/// WHO_AM_I device identification register
pub const DEVICE_ID: u8 = 0x2a;

// === CTRL_REG1 ===

pub const ODR_MASK: u8 = 0b0011_1000;


/// Output Data Rate
#[derive(Copy, Clone)]
pub enum Odr {
    /// 800 Hz
    Hz800 = 0,
    /// 400 Hz
    Hz400 = 1,
    /// 200 Hz
    Hz200 = 2,
    /// 100 Hz
    Hz100 = 3,
    /// 50 Hz
    Hz50 = 4,
    /// 12 Hz
    Hz12 = 5,
    /// 6 Hz
    Hz6 = 6,
    /// 1 Hz
    Hz1 = 7,
}

pub const STANDBY: u8 = 0b0000_0001;

pub const SYSMOD_ACTIVE: u8 = 0b0000_0011;


// === XYZ_DATA_CFG ===
pub const FS_MASK: u8 = 0b0000_0011;

/// Full-scale selection
#[derive(Copy, Clone)]
pub enum FullScale {
    /// ±2 g
    G2 = 0,
    /// ±4 g
    G4 = 1,
    /// ±8 g
    G8 = 2,
}

impl FullScale {
    #[cfg(feature = "out_f32")]
    pub(crate) fn convert_out_i16tof32(self, val: i16) -> f32 {
        // g/digit for high-resolution mode (12-bit)
        let sens: f32 = match self {
            Self::G2 => 1024.0,
            Self::G4 => 512.0,
            Self::G8 => 256.0,
        };
        // up to 12-bit data, left-justified
        f32(val >> 4) / sens
    }
    
    /*#[cfg(feature = "out_f32")]
    pub(crate) fn convert_ths_f32tou8(self, val: f32) -> u8 {
        // 1LSb = x g
        let lsb: f32 = match self {
            Self::G2 => 0.016,
            Self::G4 => 0.032,
            Self::G8 => 0.062,
        };
        let f = val / lsb; // .round(); can not be used with no_std for now
        if f < 0.0 {
            0
        } else if f > 127.0 {
            0x7F
        } else {
            f as u8
        }
    }*/
}

// === CTRL_REG2 ===

pub const RST: u8 = 0b0100_0000;
pub const MOD: u8 = 0b0000_0011;

// === STATUS_REG ===

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;

