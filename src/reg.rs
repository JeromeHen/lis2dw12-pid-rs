use bitflags::bitflags;
use num_enum::TryFromPrimitive;

// Register addresses (taken from the C header)
pub const LIS2DW12_OUT_T_L: u8 = 0x0D;
pub const LIS2DW12_OUT_T_H: u8 = 0x0E;
pub const LIS2DW12_WHO_AM_I: u8 = 0x0F;
pub const LIS2DW12_CTRL1: u8 = 0x20;
pub const LIS2DW12_CTRL2: u8 = 0x21;
pub const LIS2DW12_CTRL3: u8 = 0x22;
pub const LIS2DW12_CTRL4_INT1_PAD_CTRL: u8 = 0x23;
pub const LIS2DW12_CTRL5_INT2_PAD_CTRL: u8 = 0x24;
pub const LIS2DW12_CTRL6: u8 = 0x25;
pub const LIS2DW12_OUT_T: u8 = 0x26;
pub const LIS2DW12_STATUS: u8 = 0x27;
pub const LIS2DW12_OUT_X_L: u8 = 0x28;
pub const LIS2DW12_OUT_X_H: u8 = 0x29;
pub const LIS2DW12_OUT_Y_L: u8 = 0x2A;
pub const LIS2DW12_OUT_Y_H: u8 = 0x2B;
pub const LIS2DW12_OUT_Z_L: u8 = 0x2C;
pub const LIS2DW12_OUT_Z_H: u8 = 0x2D;
pub const LIS2DW12_FIFO_CTRL: u8 = 0x2E;
pub const LIS2DW12_FIFO_SAMPLES: u8 = 0x2F;
pub const LIS2DW12_TAP_THS_X: u8 = 0x30;
pub const LIS2DW12_TAP_THS_Y: u8 = 0x31;
pub const LIS2DW12_TAP_THS_Z: u8 = 0x32;
pub const LIS2DW12_INT_DUR: u8 = 0x33;
pub const LIS2DW12_WAKE_UP_THS: u8 = 0x34;
pub const LIS2DW12_WAKE_UP_DUR: u8 = 0x35;
pub const LIS2DW12_FREE_FALL: u8 = 0x36;
pub const LIS2DW12_STATUS_DUP: u8 = 0x37;
pub const LIS2DW12_WAKE_UP_SRC: u8 = 0x38;
pub const LIS2DW12_TAP_SRC: u8 = 0x39;
pub const LIS2DW12_SIXD_SRC: u8 = 0x3A;
pub const LIS2DW12_ALL_INT_SRC: u8 = 0x3B;
pub const LIS2DW12_X_OFS_USR: u8 = 0x3C;
pub const LIS2DW12_Y_OFS_USR: u8 = 0x3D;
pub const LIS2DW12_Z_OFS_USR: u8 = 0x3E;
pub const LIS2DW12_CTRL_REG7: u8 = 0x3F;

/// Device Identification (Who am I)
pub const LIS2DW12_ID: u8 = 0x44;

// Minimal enums & masks used by the Rust API. These are intentionally
// simplified compared to the original C header, mapping direct u8
// representations for enums to keep the async conversions straightforward.

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum Odr {
    Off = 0x00,
    Hz1_6Lp = 0x01,
    Hz12_5 = 0x02,
    Hz25 = 0x03,
    Hz50 = 0x04,
    Hz100 = 0x05,
    Hz200 = 0x06,
    Hz400 = 0x07,
    Hz800 = 0x08,
    Hz1600 = 0x09,
}

bitflags! {pub struct Ctrl4Int1: u8 {
    const INT1_DRDY = 1 << 0;
    const INT1_FTH = 1 << 1;
    const INT1_DIFF5 = 1 << 2;
    const INT1_TAP = 1 << 3;
    const INT1_FF = 1 << 4;
    const INT1_WU = 1 << 5;
    const INT1_SINGLE_TAP = 1 << 6;
    const INT1_6D = 1 << 7;
}
}

bitflags! {pub struct Fds : u8 {
    const LPF_ON_OUT = 0x00; // default, no flags
    const USER_OFFSET_ON_OUT = 1 << 0;
    const HPF_ON_OUT = 1 << 4;
}}

bitflags! {pub struct Ctrl5Int2: u8 {
    const INT2_DRDY = 1 << 0;
    const INT2_FTH = 1 << 1;
    const INT2_DIFF5 = 1 << 2;
    const INT2_OVR = 1 << 3;
    const INT2_DRDY_T = 1 << 4;
    const INT2_BOOT = 1 << 5;
    const INT2_SLEEP_CHG = 1 << 6;
    const INT2_SLEEP_STATE = 1 << 7;
}
}

bitflags! {pub struct CtrlReg7: u8 {
    const LPASS_ON_6D = 1 << 0; // lpass_on6d
    const HP_REF_MODE = 1 << 1; // hp_ref_mode
    const USR_OFF_W = 1 << 2; // usr_off_w
    const USR_OFF_ON_WU = 1 << 3; // usr_off_on_wu
    const USR_OFF_ON_OUT = 1 << 4; // usr_off_on_out
    const INTERRUPTS_ENABLE = 1 << 5; // interrupts_enable
    const INT2_ON_INT1 = 1 << 6; // int2_on_int1
    const DRDY_PULSED = 1 << 7; // drdy_pulsed
}
}

// TAP_THs masks
pub const TAP_THS_X_6D_THS_MASK: u8 = 0x60;
pub const TAP_THS_X_4D_EN_MASK: u8 = 0x80;
pub const TAP_THS_X_TAP_THS_MASK: u8 = 0x1F;
pub const TAP_THS_Y_TAP_THS_MASK: u8 = 0x1F;
pub const TAP_THS_Y_PRIOR_MASK: u8 = 0xE0;
pub const TAP_THS_Z_TAP_THS_MASK: u8 = 0x1F;
pub const TAP_THS_Z_Z_EN_MASK: u8 = 1 << 5;
pub const TAP_THS_Z_Y_EN_MASK: u8 = 1 << 6;
pub const TAP_THS_Z_X_EN_MASK: u8 = 1 << 7;

// INT_DUR masks
pub const INT_DUR_SHOCK_MASK: u8 = 0x03;
pub const INT_DUR_QUIET_MASK: u8 = 0x0C;
pub const INT_DUR_LATENCY_MASK: u8 = 0xF0;

// WAKE_UP masks
pub const WAKE_UP_THS_WK_THS_MASK: u8 = 0x3F;
pub const WAKE_UP_THS_SLEEP_ON_MASK: u8 = 1 << 6;
pub const WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK: u8 = 1 << 7;

pub const WAKE_UP_DUR_SLEEP_DUR_MASK: u8 = 0x0F;
pub const WAKE_UP_DUR_STATIONARY_MASK: u8 = 1 << 4;
pub const WAKE_UP_DUR_WAKE_DUR_MASK: u8 = 0x60;
pub const WAKE_UP_DUR_FF_DUR_MASK: u8 = 1 << 7;

// FREE FALL masks
pub const FREE_FALL_FF_THS_MASK: u8 = 0x07;
pub const FREE_FALL_FF_DUR_MASK: u8 = 0xF8;

// FIFO masks
pub const FIFO_CTRL_FTH_MASK: u8 = 0x1F;
pub const FIFO_CTRL_FMODE_MASK: u8 = 0xE0;
pub const FIFO_SAMPLES_DIFF_MASK: u8 = 0x3F;
pub const FIFO_SAMPLES_OVR_MASK: u8 = 1 << 6;
pub const FIFO_SAMPLES_FTH_MASK: u8 = 1 << 7;

// Little-endian specific masks for CTRL3 where the slp_mode field sits in
// bits 1:0 of the byte. These are used since the driver expects the default
// DRV_BYTE_ORDER to be little-endian.
pub const CTRL3_SLP_MODE_LE_MASK: u8 = 0x03;

// Helper masks for the API-encoded arguments (not actual register bits).
pub const FILTER_PATH_ARG_FDS_MASK: u8 = 0x10; // bit 4 in a filter_path arg
pub const FILTER_PATH_ARG_USR_OFF_ON_OUT_MASK: u8 = 0x01; // bit 0 in a filter_path arg

// When selecting power modes via a compact `val` as in the C driver, bit 4
// encodes low_noise and bits 4..5 encode slp_mode for mapping to CTRL registers.
pub const POWER_MODE_ARG_LOW_NOISE_MASK: u8 = 0x10;
// Activity mode argument masks (val bits passed to act_mode_set/get)
pub const ACT_MODE_ARG_SLEEP_ON_MASK: u8 = 0x01;
pub const ACT_MODE_ARG_STATIONARY_MASK: u8 = 0x02;
pub const POWER_MODE_ARG_SLP_MODE_MASK: u8 = 0x30;
// Mask used to limit ODR values passed via the compact arg (low 4 bits)
pub const ODR_ARG_VAL_MASK: u8 = 0x0F;
// bit5 mask used by the combined FF duration argument
pub const FF_DUR_ARG_MSB_MASK: u8 = 0x20;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UsrOffOnWu {
    HpFeed = 0,
    UserOffsetFeed = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LpassOn6D {
    OdrDiv2Feed = 0,
    Lpf2Feed = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum SelfTest {
    Disable = 0,
    Positive = 1,
    Negative = 2,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UsrOffW {
    Lsb977ug = 0,
    Lsb15mg6 = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum Fs {
    G2 = 0,
    G4 = 1,
    G8 = 2,
    G16 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum BwFilt {
    OdrDiv2 = 0,
    OdrDiv4 = 1,
    OdrDiv10 = 2,
    OdrDiv20 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Sim {
    Spi4Wire = 0,
    Spi3Wire = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum I2cDisable {
    I2cEnable = 0,
    I2cDisable = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CsPuDisc {
    PullUpConnect = 0,
    PullUpDisconnect = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum HLACTIVE {
    ActiveHigh = 0,
    ActiveLow = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Lir {
    Pulsed = 0,
    Latched = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PpOd {
    PushPull = 0,
    OpenDrain = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DrdyPulsed {
    Latched = 0,
    Pulsed = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum Fmode {
    Bypass = 0,
    Fifo = 1,
    StreamToFifo = 3,
    BypassToStream = 4,
    Stream = 6,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum FfThs {
    Tsh5 = 0,
    Tsh7 = 1,
    Tsh8 = 2,
    Tsh10 = 3,
    Tsh11 = 4,
    Tsh13 = 5,
    Tsh15 = 6,
    Tsh16 = 7,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum SixdThs {
    Level0 = 0,
    Level1 = 1,
    Level2 = 2,
    Level3 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum TapShock {
    Dur4 = 0,
    Dur8 = 1,
    Dur16 = 2,
    Dur24 = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum TapQuiet {
    Dur2 = 0,
    Dur4 = 1,
    Dur8 = 2,
    Dur12 = 3,
}

// A newtype for tap duration (4 bits - 0..15)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TapDur(pub u8);

// WakeUp duration composite type
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WkupDur {
    pub sleep_dur: u8,    // 4 bits
    pub stationary: bool, // 1 bit
    pub wake_dur: u8,     // 2 bits
    pub ff_dur: bool,     // 1 bit
}

impl WkupDur {
    /// Pack the composite WkupDur structure into a single register byte.
    ///
    /// # Returns
    ///
    /// - `u8` - Byte value suitable for writing to WAKE_UP_DUR register.
    pub fn pack(&self) -> u8 {
        ((self.sleep_dur & 0x0F) & WAKE_UP_DUR_SLEEP_DUR_MASK)
            | ((self.stationary as u8) << 4 & WAKE_UP_DUR_STATIONARY_MASK)
            | ((self.wake_dur & 0x03) << 5 & WAKE_UP_DUR_WAKE_DUR_MASK)
            | (((self.ff_dur as u8) << 7) & WAKE_UP_DUR_FF_DUR_MASK)
    }
    /// Unpack a WAKE_UP_DUR register byte into a WkupDur structure.
    ///
    /// # Arguments
    ///
    /// - `val` (`u8`) - Raw byte value read from WAKE_UP_DUR register.
    ///
    /// # Returns
    ///
    /// - `WkupDur` - Decoded composite wake-up duration struct.
    pub fn unpack(val: u8) -> WkupDur {
        WkupDur {
            sleep_dur: val & WAKE_UP_DUR_SLEEP_DUR_MASK,
            stationary: (val & WAKE_UP_DUR_STATIONARY_MASK) != 0,
            wake_dur: (val & WAKE_UP_DUR_WAKE_DUR_MASK) >> 5,
            ff_dur: (val & WAKE_UP_DUR_FF_DUR_MASK) != 0,
        }
    }
}

// Free fall duration composite value (6 bits but stored across WAKE_UP_DUR[7] and FREE_FALL[4:0])
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FfDur(pub u8);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WkupTh(pub u8); // 6-bit wake-up threshold (0..63)

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ActSleepDur(pub u8); // 4-bit sleep duration (0..15)

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TapThreshold(pub u8); // 5-bit threshold for tap

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct FifoWatermark(pub u8); // 5-bit watermark

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum SleepOn {
    NoDetection = 0,
    DetectActInact = 1,
    DetectStatMotion = 3,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum TapPrior {
    XYZ = 0,
    YXZ = 1,
    XZY = 2,
    ZYX = 3,
    YZX = 5,
    ZXY = 6,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SingleDoubleTap {
    SingleOnly = 0,
    Both = 1,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, TryFromPrimitive)]
pub enum Mode {
    HighPerformance = 0x04,
    ContLowPower4 = 0x03,
    ContLowPower3 = 0x02,
    ContLowPower2 = 0x01,
    ContLowPower12bit = 0x00,
    HighPerformanceLowNoise = 0x14,
    ContLowPower4LowNoise = 0x13,
    ContLowPower3LowNoise = 0x12,
    ContLowPower2LowNoise = 0x11,
    ContLowPower12bitLowNoise = 0x10,
}

bitflags! {pub struct Ctrl1 : u8 {
    const MODE_MASK = 0b00001100;
    const LP_MODE_MASK = 0b00000011;
    const ODR_MASK = 0b11110000;
}}

bitflags! {pub struct Ctrl2 : u8 {
    const SIM = 1 << 0;
    const I2C_DISABLE = 1 << 1;
    const IF_ADD_INC = 1 << 2;
    const BDU = 1 << 3;
    const CS_PU_DISC = 1 << 4;
    const SOFT_RESET = 1 << 6;
    const BOOT = 1 << 7;
}}

bitflags! {pub struct Ctrl3 : u8 {
    const SLP_MODE_MASK = 0b11000000; // two bits 7:6 depending on endian
    const H_LACTIVE = 1 << 3;
    const LIR = 1 << 4;
    const PP_OD = 1 << 5;
    const ST_MASK = 0b11000000; // self-test bits 7:6 (same mask as SLP_MODE on opposite endianness)
}}

bitflags! {pub struct Ctrl6 : u8 {
    const LOW_NOISE = 1 << 2; // bit2
    const FDS = 1 << 3; // bit3
    const FS_MASK = 0b0011_0000; // bits 4:5
    const BW_FILT_MASK = 0b1100_0000; // bits 6:7
}}

bitflags! {pub struct Status : u8 {
    const DRDY = 1 << 0;
    const FF_IA = 1 << 1;
    const SIXD_IA = 1 << 2;
    const SINGLE_TAP = 1 << 3;
    const DOUBLE_TAP = 1 << 4;
    const SLEEP_STATE = 1 << 5;
    const WU_IA = 1 << 6;
    const FIFO_THS = 1 << 7;
}}

bitflags! {pub struct TapSrc: u8 {
    const Z_TAP = 1 << 0;
    const Y_TAP = 1 << 1;
    const X_TAP = 1 << 2;
    const TAP_SIGN = 1 << 3;
    const DOUBLE_TAP = 1 << 4;
    const SINGLE_TAP = 1 << 5;
    const TAP_IA = 1 << 6;
}
}

bitflags! {pub struct SixdSrc: u8 {
    const XL = 1 << 0;
    const XH = 1 << 1;
    const YL = 1 << 2;
    const YH = 1 << 3;
    const ZL = 1 << 4;
    const ZH = 1 << 5;
    const SIXD_IA = 1 << 6;
}
}
