#[cfg(feature = "async")]
use crate::{error::Error, reg::*, transport::Transport};

pub struct Lis2dw12<T> {
    iface: T,
}

impl<T> Lis2dw12<T> {
    /// Construct a new Lis2dw12 instance, with specified type of transport (async or blocking).
    ///
    /// # Arguments
    ///
    /// - `iface` (`T`) - The type of transport interface (I2C, SPI / async, blocking)
    pub fn new(iface: T) -> Self {
        Lis2dw12 { iface }
    }

    // TODO: add default address for I2C
}

impl<T> Lis2dw12<T> {
    /// Consume the Lis2dw12 driver instance and return the underlying transport instance.
    pub fn destroy(self) -> T {
        self.iface
    }
}

impl Lis2dw12<()> {
    /// Convert raw 16g low-power-1 accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs16_lp1_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.488
    }

    /// Convert raw 16g accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs16_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.488
    }

    /// Convert raw 2g low-power-1 accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs2_lp1_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.061
    }
    /// Convert raw 2g accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs2_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.061
    }

    /// Convert raw 4g low-power-1 accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs4_lp1_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.122
    }

    /// Convert raw 4g accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs4_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.122
    }

    /// Convert raw 8g low-power-1 accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs8_lp1_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.244
    }

    /// Convert raw 8g accelerometer LSB to milli-g.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Acceleration in milli-g.
    pub fn from_fs8_to_mg(lsb: i16) -> f32 {
        (lsb as f32) * 0.244
    }

    /// Convert raw temperature LSB to celsius.
    ///
    /// # Arguments
    ///
    /// - `lsb` (`i16`) - Raw signed 16-bit LSB value from sensor.
    ///
    /// # Returns
    ///
    /// - `f32` - Temperature in celsius.
    pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
        (lsb as f32) / 256.0 + 25.0
    }
}

#[cfg(feature = "async")]
impl<T> Lis2dw12<T>
where
    T: Transport + Send,
{
    /// Read one or more registers from the device.
    ///
    /// # Arguments
    ///
    /// - `reg` (`u8`) - First register address to read from.
    /// - `buf` (`&mut [u8]`) - Destination buffer to fill with register bytes.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok() on success, or `Error::BusError`.
    pub async fn read_reg(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        self.iface
            .read_register(reg, buf)
            .await
            .map_err(|_| Error::BusError)
    }

    /// Write one or more registers on the device.
    ///
    /// # Arguments
    ///
    /// - `reg` (`u8`) - First register address to write to.
    /// - `data` (`&[u8]`) - Bytes to write into registers.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok() on success, or `Error::BusError`.
    pub async fn write_reg(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        self.iface
            .write_register(reg, data)
            .await
            .map_err(|_| Error::BusError)
    }

    /// Set the power mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`Mode`) - The Mode variant to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn power_mode_set(&mut self, val: Mode) -> Result<(), Error> {
        // CTRL1: set LP_MODE bits [1:0] and MODE bits [3:2]
        let mut ctrl1 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL1, &mut ctrl1).await?;
        let mask = Ctrl1::LP_MODE_MASK.bits() | Ctrl1::MODE_MASK.bits();
        ctrl1[0] = (ctrl1[0] & !mask) | ((val as u8) & mask);
        self.write_reg(LIS2DW12_CTRL1, &ctrl1).await?;

        // CTRL6: low_noise bit is at bit2, sourced from val bit 4
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        let low_noise_val = ((val as u8) & POWER_MODE_ARG_LOW_NOISE_MASK) >> 4;
        if low_noise_val != 0 {
            ctrl6[0] |= Ctrl6::LOW_NOISE.bits();
        } else {
            ctrl6[0] &= !Ctrl6::LOW_NOISE.bits();
        }
        self.write_reg(LIS2DW12_CTRL6, &ctrl6).await?;

        Ok(())
    }

    /// Get the power mode.
    ///
    /// # Returns
    ///
    /// - `Result<Mode, Error>` - Mode , or a Bus error.
    pub async fn power_mode_get(&mut self) -> Result<Mode, Error> {
        let mut ctrl1 = [0u8; 1];
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL1, &mut ctrl1).await?;
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        let lp_mode = ctrl1[0] & Ctrl1::LP_MODE_MASK.bits();
        let mode = ctrl1[0] & Ctrl1::MODE_MASK.bits();
        let low_noise = ((ctrl6[0] & Ctrl6::LOW_NOISE.bits()) >> 2) << 4; // move bit2 into bit4
        let val = low_noise | mode | lp_mode;
        Ok(Mode::try_from(val).unwrap_or(Mode::ContLowPower12bit))
    }

    /// Set the data rate.
    ///
    /// # Arguments
    ///
    /// - `val` (`Odr`) - The Odr to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn data_rate_set(&mut self, val: Odr) -> Result<(), Error> {
        // Read CTRL1, set ODR bits (bits 4..7), write back
        let mut buf = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL1, &mut buf).await?;
        let mut ctrl1 = buf[0];

        // Clear ODR (bits 4..7) and set new value
        ctrl1 = (ctrl1 & !Ctrl1::ODR_MASK.bits()) | (((val as u8) & ODR_ARG_VAL_MASK) << 4);
        self.write_reg(LIS2DW12_CTRL1, &[ctrl1]).await?;

        // Also update ctrl3.slp_mode from val bits 4..5 (shifted)
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        ctrl3[0] = (ctrl3[0] & !CTRL3_SLP_MODE_LE_MASK)
            | (((val as u8) & POWER_MODE_ARG_SLP_MODE_MASK) >> 4);
        self.write_reg(LIS2DW12_CTRL3, &ctrl3).await?;

        Ok(())
    }

    /// Get the data rate.
    ///
    /// # Returns
    ///
    /// - `Result<Odr, Error>` - Odr , or a Bus error.
    pub async fn data_rate_get(&mut self) -> Result<Odr, Error> {
        let mut ctrl1 = [0u8; 1];
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL1, &mut ctrl1).await?;
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;

        let odr_raw =
            ((ctrl3[0] & CTRL3_SLP_MODE_LE_MASK) << 4) + ((ctrl1[0] & Ctrl1::ODR_MASK.bits()) >> 4);
        Ok(Odr::try_from(odr_raw).unwrap_or(Odr::Off))
    }

    /// Set the block data update.
    ///
    /// # Arguments
    ///
    /// - `enable` (`bool`) - true to enable, false to disable.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn block_data_update_set(&mut self, enable: bool) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        let mut flags = Ctrl2::from_bits_truncate(ctrl2[0]);
        if enable {
            flags.insert(Ctrl2::BDU);
        } else {
            flags.remove(Ctrl2::BDU);
        }
        ctrl2[0] = flags.bits();
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Get the block data update.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - bool , or a Bus error.
    pub async fn block_data_update_get(&mut self) -> Result<bool, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok(Ctrl2::from_bits_truncate(ctrl2[0]).contains(Ctrl2::BDU))
    }

    /// Set the full scale.
    ///
    /// # Arguments
    ///
    /// - `val` (`Fs`) - The Fs to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn full_scale_set(&mut self, val: Fs) -> Result<(), Error> {
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        // Clear FS bits (bits 4..5) and set new value
        ctrl6[0] =
            (ctrl6[0] & !Ctrl6::FS_MASK.bits()) | (((val as u8) << 4) & Ctrl6::FS_MASK.bits());
        self.write_reg(LIS2DW12_CTRL6, &ctrl6).await?;
        Ok(())
    }

    /// Get the full scale.
    ///
    /// # Returns
    ///
    /// - `Result<Fs, Error>` - Fs , or a Bus error.
    pub async fn full_scale_get(&mut self) -> Result<Fs, Error> {
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        let val = (ctrl6[0] & Ctrl6::FS_MASK.bits()) >> 4;
        Ok(Fs::try_from(val).unwrap_or(Fs::G2))
    }

    /// Get the status reg.
    ///
    /// # Returns
    ///
    /// - `Result<Status, Error>` - Status , or a Bus error.
    pub async fn status_reg_get(&mut self) -> Result<Status, Error> {
        let mut buf = [0u8; 1];
        self.read_reg(LIS2DW12_STATUS, &mut buf).await?;
        Ok(Status::from_bits_truncate(buf[0]))
    }

    /// Get the flag data ready.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - bool , or a Bus error.
    pub async fn flag_data_ready_get(&mut self) -> Result<bool, Error> {
        let reg = self.status_reg_get().await?;
        Ok(reg.contains(Status::DRDY))
    }

    /// Get the all sources.
    ///
    /// # Returns
    ///
    /// - `Result<[u8; 5], Error>` - [u8; 5] , or a Bus error.
    pub async fn all_sources_get(&mut self) -> Result<[u8; 5], Error> {
        let mut buf = [0u8; 5];
        self.read_reg(LIS2DW12_STATUS_DUP, &mut buf).await?;
        Ok(buf)
    }

    /// Set the usr offset x.
    ///
    /// # Arguments
    ///
    /// - `val` (`i8`) - The i8 to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn usr_offset_x_set(&mut self, val: i8) -> Result<(), Error> {
        self.write_reg(LIS2DW12_X_OFS_USR, &[val as u8]).await?;
        Ok(())
    }

    /// Get the usr offset x.
    ///
    /// # Returns
    ///
    /// - `Result<i8, Error>` - i8 , or a Bus error.
    pub async fn usr_offset_x_get(&mut self) -> Result<i8, Error> {
        let mut buff = [0u8; 1];
        self.read_reg(LIS2DW12_X_OFS_USR, &mut buff).await?;
        Ok(buff[0] as i8)
    }

    /// Set the usr offset y.
    ///
    /// # Arguments
    ///
    /// - `val` (`i8`) - The i8 to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn usr_offset_y_set(&mut self, val: i8) -> Result<(), Error> {
        self.write_reg(LIS2DW12_Y_OFS_USR, &[val as u8]).await?;
        Ok(())
    }

    /// Get the usr offset y.
    ///
    /// # Returns
    ///
    /// - `Result<i8, Error>` - i8 , or a Bus error.
    pub async fn usr_offset_y_get(&mut self) -> Result<i8, Error> {
        let mut buff = [0u8; 1];
        self.read_reg(LIS2DW12_Y_OFS_USR, &mut buff).await?;
        Ok(buff[0] as i8)
    }

    /// Set the usr offset z.
    ///
    /// # Arguments
    ///
    /// - `val` (`i8`) - The i8 to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn usr_offset_z_set(&mut self, val: i8) -> Result<(), Error> {
        self.write_reg(LIS2DW12_Z_OFS_USR, &[val as u8]).await?;
        Ok(())
    }

    /// Get the usr offset z.
    ///
    /// # Returns
    ///
    /// - `Result<i8, Error>` - i8 , or a Bus error.
    pub async fn usr_offset_z_get(&mut self) -> Result<i8, Error> {
        let mut buff = [0u8; 1];
        self.read_reg(LIS2DW12_Z_OFS_USR, &mut buff).await?;
        Ok(buff[0] as i8)
    }

    /// Set the offset weight.
    ///
    /// # Arguments
    ///
    /// - `val` (`UsrOffW`) - The UsrOffW to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn offset_weight_set(&mut self, val: UsrOffW) -> Result<(), Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        if val as u8 != 0 {
            reg7[0] |= CtrlReg7::USR_OFF_W.bits()
        } else {
            reg7[0] &= !CtrlReg7::USR_OFF_W.bits()
        };
        self.write_reg(LIS2DW12_CTRL_REG7, &reg7).await?;
        Ok(())
    }

    /// Get the offset weight.
    ///
    /// # Returns
    ///
    /// - `Result<UsrOffW, Error>` - UsrOffW , or a Bus error.
    pub async fn offset_weight_get(&mut self) -> Result<UsrOffW, Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        Ok(if ((reg7[0] >> 2) & 0x01) != 0 {
            UsrOffW::Lsb15mg6
        } else {
            UsrOffW::Lsb977ug
        })
    }

    /// Get the temperature raw.
    ///
    /// # Returns
    ///
    /// - `Result<i16, Error>` - i16 , or a Bus error.
    pub async fn temperature_raw_get(&mut self) -> Result<i16, Error> {
        let mut buff = [0u8; 2];
        self.read_reg(LIS2DW12_OUT_T_L, &mut buff).await?;
        let raw = (buff[1] as i16) << 8 | (buff[0] as i16);
        Ok(raw)
    }

    /// Get the acceleration raw.
    ///
    /// # Returns
    ///
    /// - `Result<[i16; 3], Error>` - [i16; 3] , or a Bus error.
    pub async fn acceleration_raw_get(&mut self) -> Result<[i16; 3], Error> {
        let mut buff = [0u8; 6];
        self.read_reg(LIS2DW12_OUT_X_L, &mut buff).await?;
        let x = (buff[1] as i16) << 8 | (buff[0] as i16);
        let y = (buff[3] as i16) << 8 | (buff[2] as i16);
        let z = (buff[5] as i16) << 8 | (buff[4] as i16);
        Ok([x, y, z])
    }

    /// Read the device id.
    ///
    /// # Returns
    ///
    /// - `Result<u8, Error>` - u8 , or a Bus error.
    pub async fn device_id_get(&mut self) -> Result<u8, Error> {
        let mut buf = [0u8; 1];
        self.read_reg(LIS2DW12_WHO_AM_I, &mut buf).await?;
        Ok(buf[0])
    }

    /// Set the auto increment.
    ///
    /// # Arguments
    ///
    /// - `enable` (`bool`) - true to enable, false to disable.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn auto_increment_set(&mut self, enable: bool) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        if enable {
            ctrl2[0] |= Ctrl2::IF_ADD_INC.bits();
        } else {
            ctrl2[0] &= !Ctrl2::IF_ADD_INC.bits();
        }
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Get the auto increment.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - bool , or a Bus error.
    pub async fn auto_increment_get(&mut self) -> Result<bool, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok((ctrl2[0] & Ctrl2::IF_ADD_INC.bits()) != 0)
    }

    /// Perform a soft reset on the device.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to assert soft-reset, false to clear.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or a Bus error.
    pub async fn reset_set(&mut self, val: bool) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        if val {
            ctrl2[0] |= Ctrl2::SOFT_RESET.bits();
        } else {
            ctrl2[0] &= !Ctrl2::SOFT_RESET.bits();
        }
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Check whether a soft reset is active.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true when soft-reset flag is set, or Bus error.
    pub async fn reset_get(&mut self) -> Result<bool, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok((ctrl2[0] & Ctrl2::SOFT_RESET.bits()) != 0)
    }

    /// Trigger the boot sequence when set.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to trigger boot, false to clear.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok() or Bus error.
    pub async fn boot_set(&mut self, val: bool) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        if val {
            ctrl2[0] |= Ctrl2::BOOT.bits();
        } else {
            ctrl2[0] &= !Ctrl2::BOOT.bits();
        }
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Read whether a boot sequence has been requested.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if boot flag is set, else false; Bus error on failure.
    pub async fn boot_get(&mut self) -> Result<bool, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok((ctrl2[0] & Ctrl2::BOOT.bits()) != 0)
    }

    /// Configure self-test mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`SelfTest`) - Self test selection to apply (Disable/Positive/Negative).
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn self_test_set(&mut self, val: SelfTest) -> Result<(), Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        ctrl3[0] =
            (ctrl3[0] & !Ctrl3::ST_MASK.bits()) | (((val as u8) << 6) & Ctrl3::ST_MASK.bits());
        self.write_reg(LIS2DW12_CTRL3, &ctrl3).await?;
        Ok(())
    }

    /// Get the current self-test configuration.
    ///
    /// # Returns
    ///
    /// - `Result<SelfTest, Error>` - SelfTest enum describing current mode, or Bus error.
    pub async fn self_test_get(&mut self) -> Result<SelfTest, Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        Ok(
            SelfTest::try_from((ctrl3[0] & Ctrl3::ST_MASK.bits()) >> 6)
                .unwrap_or(SelfTest::Disable),
        )
    }

    /// Set the data-ready signalling mode (latched/pulsed).
    ///
    /// # Arguments
    ///
    /// - `mode` (`DrdyPulsed`) - Desired DRDY signalling mode.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn data_ready_mode_set(&mut self, mode: DrdyPulsed) -> Result<(), Error> {
        let mut r7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut r7).await?;
        if (mode as u8) != 0 {
            r7[0] |= CtrlReg7::DRDY_PULSED.bits()
        } else {
            r7[0] &= !CtrlReg7::DRDY_PULSED.bits()
        };
        self.write_reg(LIS2DW12_CTRL_REG7, &r7).await?;
        Ok(())
    }

    /// Get the configured data-ready signalling mode.
    ///
    /// # Returns
    ///
    /// - `Result<DrdyPulsed, Error>` - Current DRDY mode, or Bus error.
    pub async fn data_ready_mode_get(&mut self) -> Result<DrdyPulsed, Error> {
        let mut r7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut r7).await?;
        Ok(if (r7[0] & CtrlReg7::DRDY_PULSED.bits()) != 0 {
            DrdyPulsed::Pulsed
        } else {
            DrdyPulsed::Latched
        })
    }

    /// Configure filter path options (HPF/LPF and output behaviors).
    ///
    /// # Arguments
    ///
    /// - `val` (`Fds`) - Flags for filter selection and output behavior.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn filter_path_set(&mut self, val: Fds) -> Result<(), Error> {
        // val encodes fds in bit4 and usr_off_on_out in ctrl_reg7 bit4
        let mut ctrl6 = [0u8; 1];
        let mut ctrl7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        // FDS stored in Ctrl6::FDS
        let mut flags6 = Ctrl6::from_bits_truncate(ctrl6[0]);
        if val.contains(Fds::HPF_ON_OUT) {
            flags6.insert(Ctrl6::FDS);
        } else {
            flags6.remove(Ctrl6::FDS);
        }
        ctrl6[0] = flags6.bits();
        self.write_reg(LIS2DW12_CTRL6, &ctrl6).await?;

        self.read_reg(LIS2DW12_CTRL_REG7, &mut ctrl7).await?;
        // usr_off_on_out is CtrlReg7::USR_OFF_ON_OUT
        let mut flags7 = CtrlReg7::from_bits_truncate(ctrl7[0]);
        if val.contains(Fds::USER_OFFSET_ON_OUT) {
            flags7.insert(CtrlReg7::USR_OFF_ON_OUT);
        } else {
            flags7.remove(CtrlReg7::USR_OFF_ON_OUT);
        }
        ctrl7[0] = flags7.bits();
        self.write_reg(LIS2DW12_CTRL_REG7, &ctrl7).await?;
        Ok(())
    }

    /// Read the filter path configuration.
    ///
    /// # Returns
    ///
    /// - `Result<Fds, Error>` - Current filter flags and behaviors, or Bus error.
    pub async fn filter_path_get(&mut self) -> Result<Fds, Error> {
        let mut ctrl6 = [0u8; 1];
        let mut ctrl7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        self.read_reg(LIS2DW12_CTRL_REG7, &mut ctrl7).await?;
        let mut fds: Fds = Fds::LPF_ON_OUT;
        if Ctrl6::from_bits_truncate(ctrl6[0]).contains(Ctrl6::FDS) {
            fds.insert(Fds::HPF_ON_OUT);
        }
        let usr_off_on_out =
            if CtrlReg7::from_bits_truncate(ctrl7[0]).contains(CtrlReg7::USR_OFF_ON_OUT) {
                1
            } else {
                0
            };
        if usr_off_on_out != 0 {
            fds.insert(Fds::USER_OFFSET_ON_OUT);
        }
        Ok(fds)
    }

    /// Set the filter bandwidth selection.
    ///
    /// # Arguments
    ///
    /// - `val` (`BwFilt`) - Filter bandwidth selection enum.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn filter_bandwidth_set(&mut self, val: BwFilt) -> Result<(), Error> {
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        ctrl6[0] = (ctrl6[0] & !Ctrl6::BW_FILT_MASK.bits())
            | (((val as u8) << 6) & Ctrl6::BW_FILT_MASK.bits());
        self.write_reg(LIS2DW12_CTRL6, &ctrl6).await?;
        Ok(())
    }

    /// Get the current filter bandwidth selection.
    ///
    /// # Returns
    ///
    /// - `Result<BwFilt, Error>` - Current filter bandwidth enum, or Bus error.
    pub async fn filter_bandwidth_get(&mut self) -> Result<BwFilt, Error> {
        let mut ctrl6 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL6, &mut ctrl6).await?;
        let raw = (ctrl6[0] & Ctrl6::BW_FILT_MASK.bits()) >> 6;
        Ok(BwFilt::try_from(raw).unwrap_or(BwFilt::OdrDiv2))
    }

    /// Set the high-pass filter reference mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable HPF reference mode, false to disable.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn reference_mode_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        let mut flags7 = CtrlReg7::from_bits_truncate(reg7[0]);
        if val {
            flags7.insert(CtrlReg7::HP_REF_MODE);
        } else {
            flags7.remove(CtrlReg7::HP_REF_MODE);
        }
        reg7[0] = flags7.bits();
        self.write_reg(LIS2DW12_CTRL_REG7, &reg7).await?;
        Ok(())
    }

    /// Get whether high-pass filter reference mode is enabled.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if enabled, else false; Bus error on failure.
    pub async fn reference_mode_get(&mut self) -> Result<bool, Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        Ok(CtrlReg7::from_bits_truncate(reg7[0]).contains(CtrlReg7::HP_REF_MODE))
    }

    /// Configure the SPI mode (3-wire/4-wire).
    ///
    /// # Arguments
    ///
    /// - `sim` (`Sim`) - SPI mode selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn spi_mode_set(&mut self, sim: Sim) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        let mut flags2 = Ctrl2::from_bits_truncate(ctrl2[0]);
        if sim as u8 != 0 {
            flags2.insert(Ctrl2::SIM);
        } else {
            flags2.remove(Ctrl2::SIM);
        }
        ctrl2[0] = flags2.bits();
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Get the configured SPI mode.
    ///
    /// # Returns
    ///
    /// - `Result<Sim, Error>` - Current SPI mode, or Bus error.
    pub async fn spi_mode_get(&mut self) -> Result<Sim, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok(if (ctrl2[0] & Ctrl2::SIM.bits()) != 0 {
            Sim::Spi3Wire
        } else {
            Sim::Spi4Wire
        })
    }

    /// Enable or disable the I2C interface.
    ///
    /// # Arguments
    ///
    /// - `val` (`I2cDisable`) - Enable or disable the I2C interface.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn i2c_interface_set(&mut self, val: I2cDisable) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        let mut flags2 = Ctrl2::from_bits_truncate(ctrl2[0]);
        if val as u8 != 0 {
            flags2.insert(Ctrl2::I2C_DISABLE);
        } else {
            flags2.remove(Ctrl2::I2C_DISABLE);
        }
        ctrl2[0] = flags2.bits();
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Get the I2C interface enable/disable state.
    ///
    /// # Returns
    ///
    /// - `Result<I2cDisable, Error>` - Current I2C interface state, or Bus error.
    pub async fn i2c_interface_get(&mut self) -> Result<I2cDisable, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok(if (ctrl2[0] & Ctrl2::I2C_DISABLE.bits()) != 0 {
            I2cDisable::I2cDisable
        } else {
            I2cDisable::I2cEnable
        })
    }

    /// Configure chip-select pull-up/disconnect behavior.
    ///
    /// # Arguments
    ///
    /// - `val` (`CsPuDisc`) - Chip-select pull-up/disconnect selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn cs_mode_set(&mut self, val: CsPuDisc) -> Result<(), Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        let mut flags2 = Ctrl2::from_bits_truncate(ctrl2[0]);
        if val as u8 != 0 {
            flags2.insert(Ctrl2::CS_PU_DISC);
        } else {
            flags2.remove(Ctrl2::CS_PU_DISC);
        }
        ctrl2[0] = flags2.bits();
        self.write_reg(LIS2DW12_CTRL2, &ctrl2).await?;
        Ok(())
    }

    /// Get the chip-select pull-up/disconnect setting.
    ///
    /// # Returns
    ///
    /// - `Result<CsPuDisc, Error>` - Current CS mode, or Bus error.
    pub async fn cs_mode_get(&mut self) -> Result<CsPuDisc, Error> {
        let mut ctrl2 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL2, &mut ctrl2).await?;
        Ok(if (ctrl2[0] & Ctrl2::CS_PU_DISC.bits()) != 0 {
            CsPuDisc::PullUpDisconnect
        } else {
            CsPuDisc::PullUpConnect
        })
    }

    /// Set the pin polarity for active-low/high behavior.
    ///
    /// # Arguments
    ///
    /// - `active_low` (`HLACTIVE`) - Active low or active high selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn pin_polarity_set(&mut self, active_low: HLACTIVE) -> Result<(), Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        if (active_low as u8) != 0 {
            ctrl3[0] |= Ctrl3::H_LACTIVE.bits();
        } else {
            ctrl3[0] &= !Ctrl3::H_LACTIVE.bits();
        }
        self.write_reg(LIS2DW12_CTRL3, &ctrl3).await?;
        Ok(())
    }

    /// Get the pin polarity setting.
    ///
    /// # Returns
    ///
    /// - `Result<HLACTIVE, Error>` - Active polarity, or Bus error.
    pub async fn pin_polarity_get(&mut self) -> Result<HLACTIVE, Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        Ok(if (ctrl3[0] & Ctrl3::H_LACTIVE.bits()) != 0 {
            HLACTIVE::ActiveLow
        } else {
            HLACTIVE::ActiveHigh
        })
    }

    /// Configure interrupt notification as latched or pulsed.
    ///
    /// # Arguments
    ///
    /// - `latched` (`Lir`) - Latched/pulsed selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn int_notification_set(&mut self, latched: Lir) -> Result<(), Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        if (latched as u8) != 0 {
            ctrl3[0] |= Ctrl3::LIR.bits();
        } else {
            ctrl3[0] &= !Ctrl3::LIR.bits();
        }
        self.write_reg(LIS2DW12_CTRL3, &ctrl3).await?;
        Ok(())
    }

    /// Get the configured interrupt notification mode.
    ///
    /// # Returns
    ///
    /// - `Result<Lir, Error>` - Current IRQ notification mode, or Bus error.
    pub async fn int_notification_get(&mut self) -> Result<Lir, Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        Ok(if (ctrl3[0] & Ctrl3::LIR.bits()) != 0 {
            Lir::Latched
        } else {
            Lir::Pulsed
        })
    }

    /// Set the pin output mode (push-pull or open-drain).
    ///
    /// # Arguments
    ///
    /// - `open_drain` (`PpOd`) - Pin output mode selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn pin_mode_set(&mut self, open_drain: PpOd) -> Result<(), Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        let mut flags3 = Ctrl3::from_bits_truncate(ctrl3[0]);
        if (open_drain as u8) != 0 {
            flags3.insert(Ctrl3::PP_OD);
        } else {
            flags3.remove(Ctrl3::PP_OD);
        }
        ctrl3[0] = flags3.bits();
        self.write_reg(LIS2DW12_CTRL3, &ctrl3).await?;
        Ok(())
    }

    /// Get the configured pin output mode.
    ///
    /// # Returns
    ///
    /// - `Result<PpOd, Error>` - Current pin mode, or Bus error.
    pub async fn pin_mode_get(&mut self) -> Result<PpOd, Error> {
        let mut ctrl3 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL3, &mut ctrl3).await?;
        Ok(
            if Ctrl3::from_bits_truncate(ctrl3[0]).contains(Ctrl3::PP_OD) {
                PpOd::OpenDrain
            } else {
                PpOd::PushPull
            },
        )
    }

    /// Configure routing to INT1 pin via CTRL4 settings.
    ///
    /// # Arguments
    ///
    /// - `ctrl4_val` (`Ctrl4Int1`) - CTRL4 bit flags to route to INT1.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn pin_int1_route_set(&mut self, ctrl4_val: Ctrl4Int1) -> Result<(), Error> {
        let mut ctrl5 = [0u8; 1];
        let mut ctrl7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL5_INT2_PAD_CTRL, &mut ctrl5)
            .await?;
        self.read_reg(LIS2DW12_CTRL_REG7, &mut ctrl7).await?;

        // if any bit in ctrl4_val or ctrl5 indicates one of the interrupt sources,
        // set interrupts_enable bit in CTRL_REG7
        let mut flags7 = CtrlReg7::from_bits_truncate(ctrl7[0]);
        let ctrl4_flags = ctrl4_val;
        let ctrl5_flags = Ctrl5Int2::from_bits_truncate(ctrl5[0]);
        if !ctrl4_flags.is_empty()
            || (ctrl5_flags.intersects(Ctrl5Int2::INT2_SLEEP_CHG | Ctrl5Int2::INT2_SLEEP_STATE))
        {
            flags7.insert(CtrlReg7::INTERRUPTS_ENABLE);
        } else {
            flags7.remove(CtrlReg7::INTERRUPTS_ENABLE);
        }
        ctrl7[0] = flags7.bits();

        self.write_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, &[ctrl4_flags.bits()])
            .await?;
        self.write_reg(LIS2DW12_CTRL_REG7, &ctrl7).await?;
        Ok(())
    }

    /// Read which CTRL4 bits are routed to INT1.
    ///
    /// # Returns
    ///
    /// - `Result<Ctrl4Int1, Error>` - CTRL4 bit flags routed to INT1, or Bus error.
    pub async fn pin_int1_route_get(&mut self) -> Result<Ctrl4Int1, Error> {
        let mut ctrl4 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, &mut ctrl4)
            .await?;
        Ok(Ctrl4Int1::from_bits_truncate(ctrl4[0]))
    }

    /// Configure routing to INT2 pin via CTRL5 settings.
    ///
    /// # Arguments
    ///
    /// - `ctrl5_val` (`Ctrl5Int2`) - CTRL5 bit flags to route to INT2.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn pin_int2_route_set(&mut self, ctrl5_val: Ctrl5Int2) -> Result<(), Error> {
        let mut ctrl4 = [0u8; 1];
        let mut ctrl7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, &mut ctrl4)
            .await?;
        self.read_reg(LIS2DW12_CTRL_REG7, &mut ctrl7).await?;

        let mut flags7 = CtrlReg7::from_bits_truncate(ctrl7[0]);
        let ctrl4_flags = Ctrl4Int1::from_bits_truncate(ctrl4[0]);
        let ctrl5_flags = ctrl5_val;
        if !ctrl4_flags.is_empty()
            || (ctrl5_flags.intersects(Ctrl5Int2::INT2_SLEEP_CHG | Ctrl5Int2::INT2_SLEEP_STATE))
        {
            flags7.insert(CtrlReg7::INTERRUPTS_ENABLE);
        } else {
            flags7.remove(CtrlReg7::INTERRUPTS_ENABLE);
        }
        ctrl7[0] = flags7.bits();

        self.write_reg(LIS2DW12_CTRL5_INT2_PAD_CTRL, &[ctrl5_flags.bits()])
            .await?;
        self.write_reg(LIS2DW12_CTRL_REG7, &ctrl7).await?;
        Ok(())
    }

    /// Read which CTRL5 bits are routed to INT2.
    ///
    /// # Returns
    ///
    /// - `Result<Ctrl5Int2, Error>` - CTRL5 bit flags routed to INT2, or Bus error.
    pub async fn pin_int2_route_get(&mut self) -> Result<Ctrl5Int2, Error> {
        let mut ctrl5 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL5_INT2_PAD_CTRL, &mut ctrl5)
            .await?;
        Ok(Ctrl5Int2::from_bits_truncate(ctrl5[0]))
    }

    /// Enable or disable all-on-int1 behaviour.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable all-on-int1, false to disable.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn all_on_int1_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        let mut r7_flags = CtrlReg7::from_bits_truncate(reg7[0]);
        if val {
            r7_flags.insert(CtrlReg7::INT2_ON_INT1);
        } else {
            r7_flags.remove(CtrlReg7::INT2_ON_INT1);
        }
        reg7[0] = r7_flags.bits();
        self.write_reg(LIS2DW12_CTRL_REG7, &reg7).await?;
        Ok(())
    }

    /// Get the all-on-int1 enable state.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true when enabled, else false; Bus error on failure.
    pub async fn all_on_int1_get(&mut self) -> Result<bool, Error> {
        let mut reg7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg7).await?;
        Ok(CtrlReg7::from_bits_truncate(reg7[0]).contains(CtrlReg7::INT2_ON_INT1))
    }

    /// Set the wake-up threshold value.
    ///
    /// # Arguments
    ///
    /// - `val` (`WkupTh`) - Wake-up threshold to set (0..63).
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn wkup_threshold_set(&mut self, val: WkupTh) -> Result<(), Error> {
        self.write_reg(LIS2DW12_WAKE_UP_THS, &[val.0 & WAKE_UP_THS_WK_THS_MASK])
            .await?;
        Ok(())
    }

    /// Read the wake-up threshold value.
    ///
    /// # Returns
    ///
    /// - `Result<WkupTh, Error>` - Current wake-up threshold, or Bus error.
    pub async fn wkup_threshold_get(&mut self) -> Result<WkupTh, Error> {
        let mut v = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_THS, &mut v).await?;
        Ok(WkupTh(v[0] & WAKE_UP_THS_WK_THS_MASK))
    }

    /// Set wake-up duration composite fields.
    ///
    /// # Arguments
    ///
    /// - `val` (`WkupDur`) - Composite wake-up duration value to write.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn wkup_dur_set(&mut self, val: WkupDur) -> Result<(), Error> {
        self.write_reg(LIS2DW12_WAKE_UP_DUR, &[val.pack()]).await?;
        Ok(())
    }

    /// Read the wake-up duration composite field.
    ///
    /// # Returns
    ///
    /// - `Result<WkupDur, Error>` - The decoded wake-up duration fields, or Bus error.
    pub async fn wkup_dur_get(&mut self) -> Result<WkupDur, Error> {
        let mut v = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut v).await?;
        Ok(WkupDur::unpack(v[0]))
    }

    /// Enable or disable feed-data-on-wakeup behavior for user offset.
    ///
    /// # Arguments
    ///
    /// - `val` (`UsrOffOnWu`) - Feed-data-on-wakeup setting.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn wkup_feed_data_set(&mut self, val: UsrOffOnWu) -> Result<(), Error> {
        let mut r7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut r7).await?;
        let mut flags7 = CtrlReg7::from_bits_truncate(r7[0]);
        if val as u8 != 0 {
            flags7.insert(CtrlReg7::USR_OFF_ON_WU)
        } else {
            flags7.remove(CtrlReg7::USR_OFF_ON_WU)
        };
        r7[0] = flags7.bits();
        self.write_reg(LIS2DW12_CTRL_REG7, &r7).await?;
        Ok(())
    }

    /// Get the feed-data-on-wakeup setting for user offset.
    ///
    /// # Returns
    ///
    /// - `Result<UsrOffOnWu, Error>` - Current feed-data-on-wakeup value, or Bus error.
    pub async fn wkup_feed_data_get(&mut self) -> Result<UsrOffOnWu, Error> {
        let mut r7 = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut r7).await?;
        Ok(
            if CtrlReg7::from_bits_truncate(r7[0]).contains(CtrlReg7::USR_OFF_ON_WU) {
                UsrOffOnWu::UserOffsetFeed
            } else {
                UsrOffOnWu::HpFeed
            },
        )
    }

    /// Set activity/inactivity detection mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`SleepOn`) - Activity/inactivity detection mode to set.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn act_mode_set(&mut self, val: SleepOn) -> Result<(), Error> {
        // val bit0 -> sleep_on, bit1 -> stationary
        let mut ths = [0u8; 1];
        let mut dur = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_THS, &mut ths).await?;
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        let bits = val as u8;
        if (bits & ACT_MODE_ARG_SLEEP_ON_MASK) != 0 {
            ths[0] |= WAKE_UP_THS_SLEEP_ON_MASK;
        } else {
            ths[0] &= !WAKE_UP_THS_SLEEP_ON_MASK;
        }
        if (bits & ACT_MODE_ARG_STATIONARY_MASK) != 0 {
            dur[0] |= WAKE_UP_DUR_STATIONARY_MASK;
        } else {
            dur[0] &= !WAKE_UP_DUR_STATIONARY_MASK;
        }
        self.write_reg(LIS2DW12_WAKE_UP_THS, &ths).await?;
        self.write_reg(LIS2DW12_WAKE_UP_DUR, &dur).await?;
        Ok(())
    }

    /// Get the configured activity/inactivity detection mode.
    ///
    /// # Returns
    ///
    /// - `Result<SleepOn, Error>` - Current SleepOn mode, or Bus error.
    pub async fn act_mode_get(&mut self) -> Result<SleepOn, Error> {
        let mut ths = [0u8; 1];
        let mut dur = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_THS, &mut ths).await?;
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        let sleep_on = (ths[0] & WAKE_UP_THS_SLEEP_ON_MASK) >> 6;
        let stationary = (dur[0] & WAKE_UP_DUR_STATIONARY_MASK) >> 4;
        let val = ((stationary << 1) & ACT_MODE_ARG_STATIONARY_MASK)
            | (sleep_on & ACT_MODE_ARG_SLEEP_ON_MASK);
        Ok(SleepOn::try_from(val).unwrap_or(SleepOn::NoDetection))
    }

    /// Set the activity sleep duration.
    ///
    /// # Arguments
    ///
    /// - `val` (`ActSleepDur`) - Sleep duration value to write.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn act_sleep_dur_set(&mut self, val: ActSleepDur) -> Result<(), Error> {
        let mut dur = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        dur[0] = (dur[0] & !WAKE_UP_DUR_SLEEP_DUR_MASK) | (val.0 & WAKE_UP_DUR_SLEEP_DUR_MASK);
        self.write_reg(LIS2DW12_WAKE_UP_DUR, &dur).await?;
        Ok(())
    }

    /// Get the activity sleep duration.
    ///
    /// # Returns
    ///
    /// - `Result<ActSleepDur, Error>` - Current sleep duration, or Bus error.
    pub async fn act_sleep_dur_get(&mut self) -> Result<ActSleepDur, Error> {
        let mut dur = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        Ok(ActSleepDur(dur[0] & WAKE_UP_DUR_SLEEP_DUR_MASK))
    }

    /// Set the tap detection threshold for X axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapThreshold`) - Threshold to apply on X axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_threshold_x_set(&mut self, val: TapThreshold) -> Result<(), Error> {
        self.write_reg(LIS2DW12_TAP_THS_X, &[val.0 & TAP_THS_X_TAP_THS_MASK])
            .await?;
        Ok(())
    }

    /// Get the tap detection threshold for X axis.
    ///
    /// # Returns
    ///
    /// - `Result<TapThreshold, Error>` - Current X-axis tap threshold, or Bus error.
    pub async fn tap_threshold_x_get(&mut self) -> Result<TapThreshold, Error> {
        let mut v = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_X, &mut v).await?;
        Ok(TapThreshold(v[0] & TAP_THS_X_TAP_THS_MASK))
    }

    /// Set the tap detection threshold for Y axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapThreshold`) - Threshold to apply on Y axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_threshold_y_set(&mut self, val: TapThreshold) -> Result<(), Error> {
        self.write_reg(LIS2DW12_TAP_THS_Y, &[val.0 & TAP_THS_Y_TAP_THS_MASK])
            .await?;
        Ok(())
    }

    /// Get the tap detection threshold for Y axis.
    ///
    /// # Returns
    ///
    /// - `Result<TapThreshold, Error>` - Current Y-axis tap threshold, or Bus error.
    pub async fn tap_threshold_y_get(&mut self) -> Result<TapThreshold, Error> {
        let mut v = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Y, &mut v).await?;
        Ok(TapThreshold(v[0] & TAP_THS_Y_TAP_THS_MASK))
    }

    /// Set axis priority when detecting taps.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapPrior`) - Priority selection for tap detection axes.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_axis_priority_set(&mut self, val: TapPrior) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Y, &mut reg).await?;
        reg[0] = (reg[0] & !TAP_THS_Y_PRIOR_MASK) | ((val as u8) << 5 & TAP_THS_Y_PRIOR_MASK);
        self.write_reg(LIS2DW12_TAP_THS_Y, &reg).await?;
        Ok(())
    }

    /// Get the axis priority for tap detection.
    ///
    /// # Returns
    ///
    /// - `Result<TapPrior, Error>` - Current axis priority, or Bus error.
    pub async fn tap_axis_priority_get(&mut self) -> Result<TapPrior, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Y, &mut reg).await?;
        Ok(TapPrior::try_from((reg[0] & TAP_THS_Y_PRIOR_MASK) >> 5).unwrap_or(TapPrior::XYZ))
    }

    /// Set the tap detection threshold for Z axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapThreshold`) - Threshold to apply on Z axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_threshold_z_set(&mut self, val: TapThreshold) -> Result<(), Error> {
        self.write_reg(LIS2DW12_TAP_THS_Z, &[val.0 & TAP_THS_Z_TAP_THS_MASK])
            .await?;
        Ok(())
    }

    /// Get the tap detection threshold for Z axis.
    ///
    /// # Returns
    ///
    /// - `Result<TapThreshold, Error>` - Current Z-axis tap threshold, or Bus error.
    pub async fn tap_threshold_z_get(&mut self) -> Result<TapThreshold, Error> {
        let mut v = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut v).await?;
        Ok(TapThreshold(v[0] & TAP_THS_Z_TAP_THS_MASK))
    }

    /// Enable or disable tap detection on Z axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable, false to disable tap detection on Z axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_detection_on_z_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        if val {
            reg[0] |= TAP_THS_Z_Z_EN_MASK
        } else {
            reg[0] &= !TAP_THS_Z_Z_EN_MASK
        }
        self.write_reg(LIS2DW12_TAP_THS_Z, &reg).await?;
        Ok(())
    }

    /// Read whether tap detection on Z axis is enabled.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if enabled, else false; Bus error on failure.
    pub async fn tap_detection_on_z_get(&mut self) -> Result<bool, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        Ok((reg[0] & TAP_THS_Z_Z_EN_MASK) != 0)
    }

    /// Enable or disable tap detection on Y axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable, false to disable tap detection on Y axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_detection_on_y_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        if val {
            reg[0] |= TAP_THS_Z_Y_EN_MASK
        } else {
            reg[0] &= !TAP_THS_Z_Y_EN_MASK
        }
        self.write_reg(LIS2DW12_TAP_THS_Z, &reg).await?;
        Ok(())
    }

    /// Read whether tap detection on Y axis is enabled.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if enabled, else false; Bus error on failure.
    pub async fn tap_detection_on_y_get(&mut self) -> Result<bool, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        Ok((reg[0] & TAP_THS_Z_Y_EN_MASK) != 0)
    }

    /// Enable or disable tap detection on X axis.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable, false to disable tap detection on X axis.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_detection_on_x_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        if val {
            reg[0] |= TAP_THS_Z_X_EN_MASK
        } else {
            reg[0] &= !TAP_THS_Z_X_EN_MASK
        }
        self.write_reg(LIS2DW12_TAP_THS_Z, &reg).await?;
        Ok(())
    }

    /// Read whether tap detection on X axis is enabled.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if enabled, else false; Bus error on failure.
    pub async fn tap_detection_on_x_get(&mut self) -> Result<bool, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_Z, &mut reg).await?;
        Ok((reg[0] & TAP_THS_Z_X_EN_MASK) != 0)
    }

    /// Set tap shock duration.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapShock`) - Shock duration selection for tap detection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_shock_set(&mut self, val: TapShock) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        reg[0] = (reg[0] & !INT_DUR_SHOCK_MASK) | ((val as u8) & INT_DUR_SHOCK_MASK);
        self.write_reg(LIS2DW12_INT_DUR, &reg).await?;
        Ok(())
    }

    /// Get the configured tap shock duration.
    ///
    /// # Returns
    ///
    /// - `Result<TapShock, Error>` - Current tap shock selection, or Bus error.
    pub async fn tap_shock_get(&mut self) -> Result<TapShock, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        Ok(TapShock::try_from(reg[0] & INT_DUR_SHOCK_MASK).unwrap_or(TapShock::Dur4))
    }

    /// Set tap quiet time selection.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapQuiet`) - Quiet time selection for tap detection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_quiet_set(&mut self, val: TapQuiet) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        reg[0] = (reg[0] & !INT_DUR_QUIET_MASK) | (((val as u8) << 2) & INT_DUR_QUIET_MASK);
        self.write_reg(LIS2DW12_INT_DUR, &reg).await?;
        Ok(())
    }

    /// Get the configured tap quiet time selection.
    ///
    /// # Returns
    ///
    /// - `Result<TapQuiet, Error>` - Current tap quiet selection, or Bus error.
    pub async fn tap_quiet_get(&mut self) -> Result<TapQuiet, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        Ok(TapQuiet::try_from((reg[0] & INT_DUR_QUIET_MASK) >> 2).unwrap_or(TapQuiet::Dur2))
    }

    /// Set the tap duration (latency) value.
    ///
    /// # Arguments
    ///
    /// - `val` (`TapDur`) - Latency/duration for tap detection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_dur_set(&mut self, val: TapDur) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        reg[0] = (reg[0] & !INT_DUR_LATENCY_MASK) | (((val.0) << 4) & INT_DUR_LATENCY_MASK);
        self.write_reg(LIS2DW12_INT_DUR, &reg).await?;
        Ok(())
    }

    /// Get the tap duration (latency).
    ///
    /// # Returns
    ///
    /// - `Result<TapDur, Error>` - Current tap latency value, or Bus error.
    pub async fn tap_dur_get(&mut self) -> Result<TapDur, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_INT_DUR, &mut reg).await?;
        Ok(TapDur((reg[0] & INT_DUR_LATENCY_MASK) >> 4))
    }

    /// Set tap detection mode (single or double).
    ///
    /// # Arguments
    ///
    /// - `val` (`SingleDoubleTap`) - Single/double tap configuration.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn tap_mode_set(&mut self, val: SingleDoubleTap) -> Result<(), Error> {
        // single_double_tap in WAKE_UP_THS bit7
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_THS, &mut reg).await?;
        if val as u8 != 0 {
            reg[0] |= WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK;
        } else {
            reg[0] &= !WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK;
        }
        self.write_reg(LIS2DW12_WAKE_UP_THS, &reg).await?;
        Ok(())
    }

    /// Get the current tap detection mode (single/double).
    ///
    /// # Returns
    ///
    /// - `Result<SingleDoubleTap, Error>` - Current tap mode, or Bus error.
    pub async fn tap_mode_get(&mut self) -> Result<SingleDoubleTap, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_THS, &mut reg).await?;
        Ok(if (reg[0] & WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK) != 0 {
            SingleDoubleTap::Both
        } else {
            SingleDoubleTap::SingleOnly
        })
    }

    /// Read the tap source register (which axes triggered tap).
    ///
    /// # Returns
    ///
    /// - `Result<TapSrc, Error>` - Tap source bitflags, or Bus error.
    pub async fn tap_src_get(&mut self) -> Result<TapSrc, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_SRC, &mut reg).await?;
        Ok(TapSrc::from_bits_truncate(reg[0]))
    }

    /// Set the 6D orientation detection threshold.
    ///
    /// # Arguments
    ///
    /// - `val` (`SixdThs`) - 6D threshold selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn sixd_threshold_set(&mut self, val: SixdThs) -> Result<(), Error> {
        // 6d threshold field is in TAP_THS_X._6d_ths (bits 5:6)
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_X, &mut reg).await?;
        reg[0] = (reg[0] & !TAP_THS_X_6D_THS_MASK) | (((val as u8) << 5) & TAP_THS_X_6D_THS_MASK);
        self.write_reg(LIS2DW12_TAP_THS_X, &reg).await?;
        Ok(())
    }

    /// Get the 6D orientation detection threshold.
    ///
    /// # Returns
    ///
    /// - `Result<SixdThs, Error>` - Current 6D threshold, or Bus error.
    pub async fn sixd_threshold_get(&mut self) -> Result<SixdThs, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_X, &mut reg).await?;
        Ok(SixdThs::try_from((reg[0] & TAP_THS_X_6D_THS_MASK) >> 5).unwrap_or(SixdThs::Level0))
    }

    /// Enable or disable 4D orientation detection mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`bool`) - true to enable 4D, false to disable.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn fourd_mode_set(&mut self, val: bool) -> Result<(), Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_X, &mut reg).await?;
        if val {
            reg[0] |= TAP_THS_X_4D_EN_MASK
        } else {
            reg[0] &= !TAP_THS_X_4D_EN_MASK
        }
        self.write_reg(LIS2DW12_TAP_THS_X, &reg).await?;
        Ok(())
    }

    /// Read whether 4D orientation detection is enabled.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true if enabled, else false; Bus error on failure.
    pub async fn fourd_mode_get(&mut self) -> Result<bool, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_TAP_THS_X, &mut reg).await?;
        Ok((reg[0] & TAP_THS_X_4D_EN_MASK) != 0)
    }

    /// Read the 6D source register describing orientation events.
    ///
    /// # Returns
    ///
    /// - `Result<SixdSrc, Error>` - 6D status bitflags, or Bus error.
    pub async fn sixd_src_get(&mut self) -> Result<SixdSrc, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_SIXD_SRC, &mut reg).await?;
        Ok(SixdSrc::from_bits_truncate(reg[0]))
    }

    /// Set whether to feed low-pass filtered data to the 6D algorithm.
    ///
    /// # Arguments
    ///
    /// - `val` (`LpassOn6D`) - Choice of feed (LPF vs ODR/2 feed).
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn sixd_feed_data_set(&mut self, val: LpassOn6D) -> Result<(), Error> {
        // lpass_on6d is bit 0 in CTRL_REG7
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg).await?;
        let mut flags7 = CtrlReg7::from_bits_truncate(reg[0]);
        if val as u8 != 0 {
            flags7.insert(CtrlReg7::LPASS_ON_6D)
        } else {
            flags7.remove(CtrlReg7::LPASS_ON_6D)
        };
        reg[0] = flags7.bits();
        self.write_reg(LIS2DW12_CTRL_REG7, &reg).await?;
        Ok(())
    }

    /// Get whether 6D detection feeds LPF data or ODR/2 data.
    ///
    /// # Returns
    ///
    /// - `Result<LpassOn6D, Error>` - Current feed selection for 6D, or Bus error.
    pub async fn sixd_feed_data_get(&mut self) -> Result<LpassOn6D, Error> {
        let mut reg = [0u8; 1];
        self.read_reg(LIS2DW12_CTRL_REG7, &mut reg).await?;
        Ok(if (reg[0] & CtrlReg7::LPASS_ON_6D.bits()) != 0 {
            LpassOn6D::Lpf2Feed
        } else {
            LpassOn6D::OdrDiv2Feed
        })
    }

    /// Set the free-fall duration (split across WAKE_UP_DUR and FREE_FALL registers).
    ///
    /// # Arguments
    ///
    /// - `val` (`FfDur`) - Free-fall duration composite value.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn ff_dur_set(&mut self, val: FfDur) -> Result<(), Error> {
        // ff_dur split across WAKE_UP_DUR[7] and FREE_FALL[4:0]
        let mut dur = [0u8; 1];
        let mut ff = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        self.read_reg(LIS2DW12_FREE_FALL, &mut ff).await?;
        let v = val.0;
        dur[0] = (dur[0] & !WAKE_UP_DUR_FF_DUR_MASK)
            | (((v & FF_DUR_ARG_MSB_MASK) << 2) & WAKE_UP_DUR_FF_DUR_MASK);
        ff[0] = (ff[0] & !FREE_FALL_FF_DUR_MASK) | ((v << 3) & FREE_FALL_FF_DUR_MASK);
        self.write_reg(LIS2DW12_WAKE_UP_DUR, &dur).await?;
        self.write_reg(LIS2DW12_FREE_FALL, &ff).await?;
        Ok(())
    }

    /// Read the free-fall duration composite value.
    ///
    /// # Returns
    ///
    /// - `Result<FfDur, Error>` - Free-fall duration value, or Bus error.
    pub async fn ff_dur_get(&mut self) -> Result<FfDur, Error> {
        let mut dur = [0u8; 1];
        let mut ff = [0u8; 1];
        self.read_reg(LIS2DW12_WAKE_UP_DUR, &mut dur).await?;
        self.read_reg(LIS2DW12_FREE_FALL, &mut ff).await?;
        Ok(FfDur(
            (((dur[0] & WAKE_UP_DUR_FF_DUR_MASK) >> 7) << 5)
                | ((ff[0] & FREE_FALL_FF_DUR_MASK) >> 3),
        ))
    }

    /// Set the free-fall threshold configuration.
    ///
    /// # Arguments
    ///
    /// - `val` (`FfThs`) - Free-fall threshold selection.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn ff_threshold_set(&mut self, val: FfThs) -> Result<(), Error> {
        let mut ff = [0u8; 1];
        self.read_reg(LIS2DW12_FREE_FALL, &mut ff).await?;
        ff[0] = (ff[0] & !FREE_FALL_FF_THS_MASK) | ((val as u8) & FREE_FALL_FF_THS_MASK);
        self.write_reg(LIS2DW12_FREE_FALL, &ff).await?;
        Ok(())
    }

    /// Read the currently-configured free-fall threshold.
    ///
    /// # Returns
    ///
    /// - `Result<FfThs, Error>` - Current free-fall threshold enum, or Bus error.
    pub async fn ff_threshold_get(&mut self) -> Result<FfThs, Error> {
        let mut ff = [0u8; 1];
        self.read_reg(LIS2DW12_FREE_FALL, &mut ff).await?;
        Ok(FfThs::try_from(ff[0] & FREE_FALL_FF_THS_MASK).unwrap_or(FfThs::Tsh5))
    }

    /// Set the FIFO watermark threshold.
    ///
    /// # Arguments
    ///
    /// - `val` (`FifoWatermark`) - FIFO watermark value to program.
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn fifo_watermark_set(&mut self, val: FifoWatermark) -> Result<(), Error> {
        let mut f = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_CTRL, &mut f).await?;
        f[0] = (f[0] & !FIFO_CTRL_FTH_MASK) | ((val.0) & FIFO_CTRL_FTH_MASK);
        self.write_reg(LIS2DW12_FIFO_CTRL, &f).await?;
        Ok(())
    }

    /// Read the currently-configured FIFO watermark value.
    ///
    /// # Returns
    ///
    /// - `Result<FifoWatermark, Error>` - FIFO watermark value, or Bus error.
    pub async fn fifo_watermark_get(&mut self) -> Result<FifoWatermark, Error> {
        let mut f = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_CTRL, &mut f).await?;
        Ok(FifoWatermark(f[0] & FIFO_CTRL_FTH_MASK))
    }

    /// Set the FIFO operating mode.
    ///
    /// # Arguments
    ///
    /// - `val` (`Fmode`) - FIFO mode to configure (Bypass/Fifo/Stream, etc.).
    ///
    /// # Returns
    ///
    /// - `Result<(), Error>` - Ok(), or Bus error.
    pub async fn fifo_mode_set(&mut self, val: Fmode) -> Result<(), Error> {
        let mut f = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_CTRL, &mut f).await?;
        f[0] = (f[0] & !FIFO_CTRL_FMODE_MASK) | (((val as u8) << 5) & FIFO_CTRL_FMODE_MASK);
        self.write_reg(LIS2DW12_FIFO_CTRL, &f).await?;
        Ok(())
    }

    /// Read the currently selected FIFO mode.
    ///
    /// # Returns
    ///
    /// - `Result<Fmode, Error>` - Current FIFO mode, or Bus error.
    pub async fn fifo_mode_get(&mut self) -> Result<Fmode, Error> {
        let mut f = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_CTRL, &mut f).await?;
        Ok(Fmode::try_from((f[0] & FIFO_CTRL_FMODE_MASK) >> 5).unwrap_or(Fmode::Bypass))
    }

    /// Read the number of samples currently in the FIFO.
    ///
    /// # Returns
    ///
    /// - `Result<u8, Error>` - FIFO sample count (0..31), or Bus error.
    pub async fn fifo_data_level_get(&mut self) -> Result<u8, Error> {
        let mut s = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_SAMPLES, &mut s).await?;
        Ok(s[0] & FIFO_SAMPLES_DIFF_MASK)
    }

    /// Check whether FIFO overrun has occurred.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true when FIFO overrun, else false; Bus error on failure.
    pub async fn fifo_ovr_flag_get(&mut self) -> Result<bool, Error> {
        let mut s = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_SAMPLES, &mut s).await?;
        Ok((s[0] & FIFO_SAMPLES_OVR_MASK) != 0)
    }

    /// Check whether FIFO watermark has been reached.
    ///
    /// # Returns
    ///
    /// - `Result<bool, Error>` - true when FIFO watermark flag set, else false; Bus error on failure.
    pub async fn fifo_wtm_flag_get(&mut self) -> Result<bool, Error> {
        let mut s = [0u8; 1];
        self.read_reg(LIS2DW12_FIFO_SAMPLES, &mut s).await?;
        Ok((s[0] & FIFO_SAMPLES_FTH_MASK) != 0)
    }

    /// Get WHO_AM_I device identifier.
    ///
    /// # Returns
    ///
    /// - `Result<u8, Error>` - WHO_AM_I register value, or Bus error.
    pub async fn who_am_i(&mut self) -> Result<u8, Error> {
        self.device_id_get().await
    }
}
