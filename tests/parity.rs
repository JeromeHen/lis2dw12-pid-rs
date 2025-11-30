// Parity tests comparing the C driver behaviour with the Rust port

use lis2dw12_pid_rs::error::Error;
use lis2dw12_pid_rs::lis2dw12::Lis2dw12;
use lis2dw12_pid_rs::reg::*;
use lis2dw12_pid_rs::transport::Transport;
use std::ffi::c_void;
use std::ptr;

#[repr(C)]
pub struct StmdevCtx {
    pub write_reg: Option<extern "C" fn(*mut c_void, u8, *const u8, u16) -> i32>,
    pub read_reg: Option<extern "C" fn(*mut c_void, u8, *mut u8, u16) -> i32>,
    pub mdelay: Option<extern "C" fn(u32)>,
    pub handle: *mut c_void,
    pub priv_data: *mut c_void,
}

#[tokio::test]
async fn parity_reference_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // C setter
        lis2dw12_reference_mode_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.reference_mode_set(true).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::HP_REF_MODE.bits(),
            after_rust[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::HP_REF_MODE.bits()
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL_REG7 as usize] = CtrlReg7::HP_REF_MODE.bits();
        let mut c_r = 0u8;
        lis2dw12_reference_mode_get(&ctx as *const _, &mut c_r as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r = dev2.reference_mode_get().await.unwrap();
        assert_eq!(c_r != 0, r);
    }
}

#[tokio::test]
async fn parity_fourd_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_4d_mode_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.fourd_mode_set(true).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_X as usize] & TAP_THS_X_4D_EN_MASK,
            after_rust[LIS2DW12_TAP_THS_X as usize] & TAP_THS_X_4D_EN_MASK
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_TAP_THS_X as usize] = TAP_THS_X_4D_EN_MASK;
        let mut c_m = 0u8;
        lis2dw12_4d_mode_get(&ctx as *const _, &mut c_m as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r = dev2.fourd_mode_get().await.unwrap();
        assert_eq!(c_m != 0, r);
    }
}

#[tokio::test]
async fn parity_tap_shock_quiet_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Shock: set to Dur8 (1), Quiet: set to Dur4 (1)
        lis2dw12_tap_shock_set(&ctx as *const _, 1u8);
        lis2dw12_tap_quiet_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.tap_shock_set(TapShock::Dur8).await.unwrap();
        dev.tap_quiet_set(TapQuiet::Dur4).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_INT_DUR as usize] & INT_DUR_SHOCK_MASK,
            after_rust[LIS2DW12_INT_DUR as usize] & INT_DUR_SHOCK_MASK
        );
        assert_eq!(
            after_c[LIS2DW12_INT_DUR as usize] & INT_DUR_QUIET_MASK,
            after_rust[LIS2DW12_INT_DUR as usize] & INT_DUR_QUIET_MASK
        );

        // getters
        *regs = [0u8; 256];
        regs[LIS2DW12_INT_DUR as usize] = (1u8 & 0x03) | ((1u8 << 2) & INT_DUR_QUIET_MASK);
        let mut c_sh = 0u8;
        lis2dw12_tap_shock_get(&ctx as *const _, &mut c_sh as *mut u8);
        let mut c_qu = 0u8;
        lis2dw12_tap_quiet_get(&ctx as *const _, &mut c_qu as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_sh = dev2.tap_shock_get().await.unwrap();
        let r_qu = dev2.tap_quiet_get().await.unwrap();
        assert_eq!(c_sh, r_sh as u8);
        assert_eq!(c_qu, r_qu as u8);
    }
}

#[tokio::test]
async fn parity_from_fs_lp1_and_lsb_to_celsius() {
    unsafe {
        let c2lp: f32 = lis2dw12_from_fs2_lp1_to_mg(1000i16);
        assert_eq!(c2lp, Lis2dw12::from_fs2_lp1_to_mg(1000));
        let c4lp: f32 = lis2dw12_from_fs4_lp1_to_mg(1000i16);
        assert_eq!(c4lp, Lis2dw12::from_fs4_lp1_to_mg(1000));
        let c8lp: f32 = lis2dw12_from_fs8_lp1_to_mg(1000i16);
        assert_eq!(c8lp, Lis2dw12::from_fs8_lp1_to_mg(1000));
        let c16lp: f32 = lis2dw12_from_fs16_lp1_to_mg(1000i16);
        assert_eq!(c16lp, Lis2dw12::from_fs16_lp1_to_mg(1000));
        let ctemp: f32 = lis2dw12_from_lsb_to_celsius(512i16);
        assert_eq!(ctemp, Lis2dw12::from_lsb_to_celsius(512));
    }
}

#[tokio::test]
async fn parity_device_id_get_who_am_i() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // set WHO_AM_I via C write
        let id: u8 = LIS2DW12_ID;
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_WHO_AM_I, &id as *const u8, 1);
        // C getter
        let mut c_id: u8 = 0;
        lis2dw12_device_id_get(&ctx as *const _, &mut c_id as *mut u8);
        // Rust getter
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        let rust_id = dev.who_am_i().await.unwrap();
        assert_eq!(c_id, rust_id);
    }
}

#[tokio::test]
async fn parity_read_write_reg_passthrough() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Write using Rust
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.write_reg(LIS2DW12_CTRL2, &[0xAB, 0xCD]).await.unwrap();
        let transport_after = dev.destroy();
        let after_rust = transport_after.regs;
        *regs = after_rust; // reflect Rust writes back into the shared C buffer for read via C
        // Read via C read and verify bytes are as written
        let mut read_buf = [0u8; 2];
        lis2dw12_read_reg(&ctx as *const _, LIS2DW12_CTRL2, read_buf.as_mut_ptr(), 2);
        assert_eq!(read_buf[0], after_rust[LIS2DW12_CTRL2 as usize]);
        assert_eq!(read_buf[1], after_rust[LIS2DW12_CTRL2 as usize + 1]);

        // Now write register using C write, then read via Rust read_reg
        *regs = [0u8; 256];
        let data = [0x11u8, 0x22u8];
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_CTRL3, data.as_ptr(), 2);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let mut buf = [0u8; 2];
        dev2.read_reg(LIS2DW12_CTRL3, &mut buf).await.unwrap();
        assert_eq!(buf[0], 0x11u8);
        assert_eq!(buf[1], 0x22u8);
    }
}

#[tokio::test]
async fn parity_pin_int_routing_and_all_on_int1_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set via C: enable INT1 for drdy and fth, INT2 for ovr
        let ctrl4_val: u8 = Ctrl4Int1::INT1_DRDY.bits() | Ctrl4Int1::INT1_FTH.bits();
        let ctrl5_val: u8 = Ctrl5Int2::INT2_OVR.bits();
        lis2dw12_pin_int1_route_set(&ctx as *const _, &ctrl4_val as *const u8);
        lis2dw12_pin_int2_route_set(&ctx as *const _, &ctrl5_val as *const u8);
        lis2dw12_all_on_int1_set(&ctx as *const _, 1u8);
        let after_c = *regs;

        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.pin_int1_route_set(Ctrl4Int1::INT1_DRDY | Ctrl4Int1::INT1_FTH)
            .await
            .unwrap();
        dev.pin_int2_route_set(Ctrl5Int2::INT2_OVR).await.unwrap();
        dev.all_on_int1_set(true).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL4_INT1_PAD_CTRL as usize],
            after_rust[LIS2DW12_CTRL4_INT1_PAD_CTRL as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL5_INT2_PAD_CTRL as usize],
            after_rust[LIS2DW12_CTRL5_INT2_PAD_CTRL as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::INT2_ON_INT1.bits(),
            after_rust[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::INT2_ON_INT1.bits()
        );

        // Getter parity
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL4_INT1_PAD_CTRL as usize] =
            Ctrl4Int1::INT1_DRDY.bits() | Ctrl4Int1::INT1_FTH.bits();
        regs[LIS2DW12_CTRL5_INT2_PAD_CTRL as usize] = Ctrl5Int2::INT2_OVR.bits();
        regs[LIS2DW12_CTRL_REG7 as usize] = CtrlReg7::INT2_ON_INT1.bits();
        let mut c4 = 0u8;
        lis2dw12_pin_int1_route_get(&ctx as *const _, &mut c4 as *mut u8);
        let mut c5 = 0u8;
        lis2dw12_pin_int2_route_get(&ctx as *const _, &mut c5 as *mut u8);
        let mut c_all = 0u8;
        lis2dw12_all_on_int1_get(&ctx as *const _, &mut c_all as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let r4 = dev2.pin_int1_route_get().await.unwrap();
        let r5 = dev2.pin_int2_route_get().await.unwrap();
        let r_all = dev2.all_on_int1_get().await.unwrap();
        assert_eq!(c4, r4.bits());
        assert_eq!(c5, r5.bits());
        assert_eq!(c_all, r_all as u8);
    }
}

#[tokio::test]
async fn parity_power_mode_and_offset_weight_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Use C to set power mode to High Performance Low Noise (0x14) and offset weight to 1
        lis2dw12_power_mode_set(&ctx as *const _, 0x14u8);
        lis2dw12_offset_weight_set(&ctx as *const _, 1u8);
        let after_c = *regs;

        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.power_mode_set(Mode::HighPerformanceLowNoise)
            .await
            .unwrap();
        dev.offset_weight_set(UsrOffW::Lsb15mg6).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL1 as usize]
                & (Ctrl1::LP_MODE_MASK.bits() | Ctrl1::MODE_MASK.bits()),
            after_rust[LIS2DW12_CTRL1 as usize]
                & (Ctrl1::LP_MODE_MASK.bits() | Ctrl1::MODE_MASK.bits())
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL6 as usize] & Ctrl6::LOW_NOISE.bits(),
            after_rust[LIS2DW12_CTRL6 as usize] & Ctrl6::LOW_NOISE.bits()
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::USR_OFF_W.bits(),
            after_rust[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::USR_OFF_W.bits()
        );

        // Getter parity
        *regs = [0u8; 256];
        // Set LP_MODE+MODE bits and low_noise bit and usr_off_w
        regs[LIS2DW12_CTRL1 as usize] =
            0x14 & (Ctrl1::LP_MODE_MASK.bits() | Ctrl1::MODE_MASK.bits());
        regs[LIS2DW12_CTRL6 as usize] = Ctrl6::LOW_NOISE.bits();
        regs[LIS2DW12_CTRL_REG7 as usize] = CtrlReg7::USR_OFF_W.bits();
        let mut c_pm = 0u8;
        lis2dw12_power_mode_get(&ctx as *const _, &mut c_pm as *mut u8);
        let mut c_ow = 0u8;
        lis2dw12_offset_weight_get(&ctx as *const _, &mut c_ow as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let r_pm = dev2.power_mode_get().await.unwrap();
        let r_ow = dev2.offset_weight_get().await.unwrap();
        assert_eq!(c_pm, r_pm as u8);
        assert_eq!(c_ow, r_ow as u8);
    }
}

#[tokio::test]
async fn parity_tap_and_sixd_src_get_and_tap_detection_on_xyz() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set TAP_SRC and SIXD_SRC
        let ts: u8 = TapSrc::SINGLE_TAP.bits();
        let six: u8 = SixdSrc::XL.bits();
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_TAP_SRC, &ts as *const u8, 1);
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_SIXD_SRC, &six as *const u8, 1);
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        let rust_tap = dev.tap_src_get().await.unwrap();
        let rust_6d = dev.sixd_src_get().await.unwrap();
        let mut c_tap: u8 = 0;
        lis2dw12_tap_src_get(&ctx as *const _, &mut c_tap as *mut u8);
        let mut c_6d: u8 = 0;
        lis2dw12_6d_src_get(&ctx as *const _, &mut c_6d as *mut u8);
        assert_eq!(c_tap, rust_tap.bits());
        assert_eq!(c_6d, rust_6d.bits());

        // Test tap detection X/Y/Z set/get
        *regs = [0u8; 256];
        lis2dw12_tap_detection_on_x_set(&ctx as *const _, 1u8);
        lis2dw12_tap_detection_on_y_set(&ctx as *const _, 1u8);
        lis2dw12_tap_detection_on_z_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        dev2.tap_detection_on_x_set(true).await.unwrap();
        dev2.tap_detection_on_y_set(true).await.unwrap();
        dev2.tap_detection_on_z_set(true).await.unwrap();
        let after_rust = dev2.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_Z as usize],
            after_rust[LIS2DW12_TAP_THS_Z as usize]
        );
        // Getter parity
        *regs = [0u8; 256];
        regs[LIS2DW12_TAP_THS_Z as usize] =
            TAP_THS_Z_X_EN_MASK | TAP_THS_Z_Y_EN_MASK | TAP_THS_Z_Z_EN_MASK;
        let mut c_x = 0u8;
        lis2dw12_tap_detection_on_x_get(&ctx as *const _, &mut c_x as *mut u8);
        let mut c_y = 0u8;
        lis2dw12_tap_detection_on_y_get(&ctx as *const _, &mut c_y as *mut u8);
        let mut c_z = 0u8;
        lis2dw12_tap_detection_on_z_get(&ctx as *const _, &mut c_z as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev3 = Lis2dw12::new(transport2);
        let rx = dev3.tap_detection_on_x_get().await.unwrap();
        let ry = dev3.tap_detection_on_y_get().await.unwrap();
        let rz = dev3.tap_detection_on_z_get().await.unwrap();
        assert_eq!(c_x, rx as u8);
        assert_eq!(c_y, ry as u8);
        assert_eq!(c_z, rz as u8);
    }
}

// Re-declared top-level parity tests (moved from inside other test)
#[tokio::test]
async fn parity_data_rate_set_get() {
    let mut regs: Box<[u8; 256]> = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set via C: ODR = 100Hz (0x05)
        let c_val: u8 = 0x05;
        lis2dw12_data_rate_set(&ctx as *const _, c_val);
        let after_c = *regs;

        // Reset and set via Rust
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.data_rate_set(Odr::Hz100).await.unwrap();
        let transport_after = dev.destroy();
        let after_rust = transport_after.regs;

        // Compare registers of interest
        assert_eq!(
            after_c[LIS2DW12_CTRL1 as usize],
            after_rust[LIS2DW12_CTRL1 as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL3 as usize],
            after_rust[LIS2DW12_CTRL3 as usize]
        );

        // Now set registers and compare getters
        *regs = [0u8; 256];
        // ctrl1.odr = 5, ctrl3.slp_mode = 0
        regs[LIS2DW12_CTRL1 as usize] = 5u8 << 4;
        regs[LIS2DW12_CTRL3 as usize] = 0;
        let mut c_out = 0u8;
        lis2dw12_data_rate_get(&ctx as *const _, &mut c_out as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_out = dev2.data_rate_get().await.unwrap();
        assert_eq!(c_out, rust_out as u8);
    }
}

#[tokio::test]
async fn parity_full_scale_and_filter_bw() {
    let mut regs: Box<[u8; 256]> = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // full scale set to 8g (2) and bw_filt to ODR_DIV_10 (2)
        lis2dw12_full_scale_set(&ctx as *const _, 2u8);
        lis2dw12_filter_bandwidth_set(&ctx as *const _, 2u8);
        let after_c = *regs;

        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.full_scale_set(Fs::G8).await.unwrap();
        dev.filter_bandwidth_set(BwFilt::OdrDiv10).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL6 as usize],
            after_rust[LIS2DW12_CTRL6 as usize]
        );

        // getters
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL6 as usize] = (2u8 << 4) | (2u8 << 6);
        let mut c_fs = 0u8;
        lis2dw12_full_scale_get(&ctx as *const _, &mut c_fs as *mut u8);
        let mut c_bw = 0u8;
        lis2dw12_filter_bandwidth_get(&ctx as *const _, &mut c_bw as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_fs = dev2.full_scale_get().await.unwrap();
        let rust_bw = dev2.filter_bandwidth_get().await.unwrap();
        assert_eq!(c_fs, rust_fs as u8);
        assert_eq!(c_bw, rust_bw as u8);
    }
}

#[tokio::test]
async fn parity_block_data_update_and_spi_i2c_cs() {
    let mut regs: Box<[u8; 256]> = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_block_data_update_set(&ctx as *const _, 1u8);
        lis2dw12_spi_mode_set(&ctx as *const _, 1u8);
        lis2dw12_i2c_interface_set(&ctx as *const _, 1u8);
        lis2dw12_cs_mode_set(&ctx as *const _, 1u8);
        let after_c = *regs;

        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.block_data_update_set(true).await.unwrap();
        dev.spi_mode_set(Sim::Spi3Wire).await.unwrap();
        dev.i2c_interface_set(I2cDisable::I2cDisable).await.unwrap();
        dev.cs_mode_set(CsPuDisc::PullUpDisconnect).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL2 as usize],
            after_rust[LIS2DW12_CTRL2 as usize]
        );

        // getters
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL2 as usize] = Ctrl2::BDU.bits()
            | Ctrl2::SIM.bits()
            | Ctrl2::I2C_DISABLE.bits()
            | Ctrl2::CS_PU_DISC.bits();
        let mut c_bdu = 0u8;
        lis2dw12_block_data_update_get(&ctx as *const _, &mut c_bdu as *mut u8);
        let mut c_sim = 0u8;
        lis2dw12_spi_mode_get(&ctx as *const _, &mut c_sim as *mut u8);
        let mut c_i2c = 0u8;
        lis2dw12_i2c_interface_get(&ctx as *const _, &mut c_i2c as *mut u8);
        let mut c_cs = 0u8;
        lis2dw12_cs_mode_get(&ctx as *const _, &mut c_cs as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let r_bdu = dev2.block_data_update_get().await.unwrap();
        let r_sim = dev2.spi_mode_get().await.unwrap();
        let r_i2c = dev2.i2c_interface_get().await.unwrap();
        let r_cs = dev2.cs_mode_get().await.unwrap();
        assert_eq!(c_bdu, r_bdu as u8);
        assert_eq!(c_sim, r_sim as u8);
        assert_eq!(c_i2c, r_i2c as u8);
        assert_eq!(c_cs, r_cs as u8);
    }
}

#[tokio::test]
async fn parity_status_and_drdy() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set status.DRDY via raw register write
        lis2dw12_write_reg(
            &ctx as *const _,
            LIS2DW12_STATUS,
            &([Status::DRDY.bits()])[0] as *const u8,
            1,
        );
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        let st = dev.status_reg_get().await.unwrap();
        assert!(st.contains(Status::DRDY));
        assert!(dev.flag_data_ready_get().await.unwrap());
    }
}

#[tokio::test]
async fn parity_wkup_feed_data_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Call C to set WAKE_UP_THS bit (usr_off_on_wu) by raw write to CTRL_REG7
        let mut reg7 = [0u8; 1];
        reg7[0] = CtrlReg7::USR_OFF_ON_WU.bits();
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_CTRL_REG7, reg7.as_ptr(), 1);
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        assert_eq!(
            dev.wkup_feed_data_get().await.unwrap(),
            UsrOffOnWu::UserOffsetFeed
        );
        // Now set via Rust and read raw reg to compare
        *regs = [0u8; 256];
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        dev2.wkup_feed_data_set(UsrOffOnWu::UserOffsetFeed)
            .await
            .unwrap();
        let transport_after = dev2.destroy();
        assert_eq!(
            transport_after.regs[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::USR_OFF_ON_WU.bits(),
            CtrlReg7::USR_OFF_ON_WU.bits()
        );
    }
}

#[tokio::test]
async fn parity_act_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // C set sleep_on and stationary = DetectActInact
        let arr_t = [WAKE_UP_THS_SLEEP_ON_MASK];
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_WAKE_UP_THS, arr_t.as_ptr(), 1);
        // C set only sleep_on -> DetectActInact
        // lis2dw12_write_reg(&ctx as *const _, LIS2DW12_WAKE_UP_DUR, arr_d.as_ptr(), 1);
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        let mode = dev.act_mode_get().await.unwrap();
        assert!(matches!(mode, SleepOn::DetectActInact));
        // Set via Rust
        *regs = [0u8; 256];
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        // Now set DetectStatMotion (both sleep_on and stationary) via Rust
        dev2.act_mode_set(SleepOn::DetectStatMotion).await.unwrap();
        let reg_after = dev2.destroy().regs;
        assert_ne!(
            reg_after[LIS2DW12_WAKE_UP_THS as usize] & WAKE_UP_THS_SLEEP_ON_MASK,
            0
        );
        assert_ne!(
            reg_after[LIS2DW12_WAKE_UP_DUR as usize] & WAKE_UP_DUR_STATIONARY_MASK,
            0
        );
    }
}

#[tokio::test]
async fn parity_tap_thresholds_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_tap_threshold_x_set(&ctx as *const _, 0x0A_u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.tap_threshold_x_set(TapThreshold(0x0A)).await.unwrap();
        let transport_after = dev.destroy();
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_X as usize],
            transport_after.regs[LIS2DW12_TAP_THS_X as usize]
        );
    }
}

#[tokio::test]
async fn parity_tap_threshold_y_z_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_tap_threshold_y_set(&ctx as *const _, 0x05);
        lis2dw12_tap_threshold_z_set(&ctx as *const _, 0x06);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.tap_threshold_y_set(TapThreshold(0x05)).await.unwrap();
        dev.tap_threshold_z_set(TapThreshold(0x06)).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_Y as usize],
            after_rust[LIS2DW12_TAP_THS_Y as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_Z as usize],
            after_rust[LIS2DW12_TAP_THS_Z as usize]
        );

        // Getters
        *regs = [0u8; 256];
        regs[LIS2DW12_TAP_THS_Y as usize] = 0x05 & TAP_THS_Y_TAP_THS_MASK;
        regs[LIS2DW12_TAP_THS_Z as usize] = 0x06 & TAP_THS_Z_TAP_THS_MASK;
        let mut c_y = 0u8;
        lis2dw12_tap_threshold_y_get(&ctx as *const _, &mut c_y as *mut u8);
        let mut c_z = 0u8;
        lis2dw12_tap_threshold_z_get(&ctx as *const _, &mut c_z as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_y = dev2.tap_threshold_y_get().await.unwrap();
        let r_z = dev2.tap_threshold_z_get().await.unwrap();
        assert_eq!(c_y, r_y.0);
        assert_eq!(c_z, r_z.0);
    }
}

#[tokio::test]
async fn parity_act_sleep_dur_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // set act_sleep_dur via C
        lis2dw12_act_sleep_dur_set(&ctx as *const _, 0x07);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.act_sleep_dur_set(ActSleepDur(0x07)).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_WAKE_UP_DUR as usize] & WAKE_UP_DUR_SLEEP_DUR_MASK,
            after_rust[LIS2DW12_WAKE_UP_DUR as usize] & WAKE_UP_DUR_SLEEP_DUR_MASK
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_WAKE_UP_DUR as usize] = 0x07 & WAKE_UP_DUR_SLEEP_DUR_MASK;
        let mut c_v = 0u8;
        lis2dw12_act_sleep_dur_get(&ctx as *const _, &mut c_v as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_v = dev2.act_sleep_dur_get().await.unwrap();
        assert_eq!(c_v, r_v.0);
    }
}

#[tokio::test]
async fn parity_tap_axis_priority_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_tap_axis_priority_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.tap_axis_priority_set(TapPrior::YXZ).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_Y as usize] & TAP_THS_Y_PRIOR_MASK,
            after_rust[LIS2DW12_TAP_THS_Y as usize] & TAP_THS_Y_PRIOR_MASK
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_TAP_THS_Y as usize] = (1u8 << 5) & TAP_THS_Y_PRIOR_MASK;
        let mut c_v = 0u8;
        lis2dw12_tap_axis_priority_get(&ctx as *const _, &mut c_v as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_v = dev2.tap_axis_priority_get().await.unwrap();
        assert_eq!(c_v, r_v as u8);
    }
}

#[tokio::test]
async fn parity_sixd_feed_and_threshold_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // set s thresh/two via C
        lis2dw12_6d_threshold_set(&ctx as *const _, 2u8);
        lis2dw12_6d_feed_data_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.sixd_threshold_set(SixdThs::Level2).await.unwrap();
        dev.sixd_feed_data_set(LpassOn6D::Lpf2Feed).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_TAP_THS_X as usize] & TAP_THS_X_6D_THS_MASK,
            after_rust[LIS2DW12_TAP_THS_X as usize] & TAP_THS_X_6D_THS_MASK
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::LPASS_ON_6D.bits(),
            after_rust[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::LPASS_ON_6D.bits()
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_TAP_THS_X as usize] = (2u8 << 5) & TAP_THS_X_6D_THS_MASK;
        regs[LIS2DW12_CTRL_REG7 as usize] = CtrlReg7::LPASS_ON_6D.bits();
        let mut c_th = 0u8;
        lis2dw12_6d_threshold_get(&ctx as *const _, &mut c_th as *mut u8);
        let mut c_feed = 0u8;
        lis2dw12_6d_feed_data_get(&ctx as *const _, &mut c_feed as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_th = dev2.sixd_threshold_get().await.unwrap();
        let r_feed = dev2.sixd_feed_data_get().await.unwrap();
        assert_eq!(c_th, r_th as u8);
        assert_eq!(c_feed, r_feed as u8);
    }
}

#[tokio::test]
async fn parity_data_ready_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        lis2dw12_data_ready_mode_set(&ctx as *const _, 1u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.data_ready_mode_set(DrdyPulsed::Pulsed).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::DRDY_PULSED.bits(),
            after_rust[LIS2DW12_CTRL_REG7 as usize] & CtrlReg7::DRDY_PULSED.bits()
        );

        // getter
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL_REG7 as usize] = CtrlReg7::DRDY_PULSED.bits();
        let mut c_mode = 0u8;
        lis2dw12_data_ready_mode_get(&ctx as *const _, &mut c_mode as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_mode = dev2.data_ready_mode_get().await.unwrap();
        assert_eq!(c_mode, r_mode as u8);
    }
}

#[tokio::test]
async fn parity_user_offsets_y_z_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        let v_y: i8 = 33;
        let v_z: i8 = -22;
        let by = v_y as u8;
        let bz = v_z as u8;
        lis2dw12_usr_offset_y_set(&ctx as *const _, &by as *const u8);
        lis2dw12_usr_offset_z_set(&ctx as *const _, &bz as *const u8);
        let after_c = *regs;
        *regs = [0u8; 256];
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        dev.usr_offset_y_set(33i8).await.unwrap();
        dev.usr_offset_z_set(-22i8).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_Y_OFS_USR as usize],
            after_rust[LIS2DW12_Y_OFS_USR as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_Z_OFS_USR as usize],
            after_rust[LIS2DW12_Z_OFS_USR as usize]
        );

        // getters
        *regs = [0u8; 256];
        regs[LIS2DW12_Y_OFS_USR as usize] = by;
        regs[LIS2DW12_Z_OFS_USR as usize] = bz;
        let mut c_y = 0u8;
        lis2dw12_usr_offset_y_get(&ctx as *const _, &mut c_y as *mut u8);
        let mut c_z = 0u8;
        lis2dw12_usr_offset_z_get(&ctx as *const _, &mut c_z as *mut u8);
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        let r_y = dev2.usr_offset_y_get().await.unwrap();
        let r_z = dev2.usr_offset_z_get().await.unwrap();
        assert_eq!(c_y as i8, r_y);
        assert_eq!(c_z as i8, r_z);
    }
}

#[tokio::test]
async fn parity_fifo_level_and_flags() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set FIFO_SAMPLES to sample=10, OVR=1, FTH=1
        let val = 10u8 | FIFO_SAMPLES_OVR_MASK | FIFO_SAMPLES_FTH_MASK;
        lis2dw12_write_reg(
            &ctx as *const _,
            LIS2DW12_FIFO_SAMPLES,
            &val as *const u8,
            1,
        );
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        assert_eq!(dev.fifo_data_level_get().await.unwrap(), 10);
        assert!(dev.fifo_ovr_flag_get().await.unwrap());
        assert!(dev.fifo_wtm_flag_get().await.unwrap());
    }
}

#[tokio::test]
async fn parity_all_sources_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        let srcs: [u8; 5] = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_STATUS_DUP, srcs.as_ptr(), 5);
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        let got = dev.all_sources_get().await.unwrap();
        assert_eq!(got, srcs);
    }
}

#[tokio::test]
async fn parity_usr_offset_x_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        let v: i8 = 42;
        let b = v as u8;
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_X_OFS_USR, &b as *const u8, 1);
        let mut dev = Lis2dw12::new(MemTransport { regs: *regs });
        assert_eq!(dev.usr_offset_x_get().await.unwrap(), v);

        // Rust setter
        *regs = [0u8; 256];
        let mut dev2 = Lis2dw12::new(MemTransport { regs: *regs });
        dev2.usr_offset_x_set(-42i8).await.unwrap();
        let regs_after = dev2.destroy().regs;
        assert_eq!(regs_after[LIS2DW12_X_OFS_USR as usize] as i8, -42i8);
    }
}

#[tokio::test]
async fn parity_conversion_from_fs() {
    // Compare C and Rust conversion functions
    unsafe {
        let c2: f32 = lis2dw12_from_fs2_to_mg(1000i16);
        assert_eq!(c2, Lis2dw12::from_fs2_to_mg(1000));
        let c4: f32 = lis2dw12_from_fs4_to_mg(1000i16);
        assert_eq!(c4, Lis2dw12::from_fs4_to_mg(1000));
        let c8: f32 = lis2dw12_from_fs8_to_mg(1000i16);
        assert_eq!(c8, Lis2dw12::from_fs8_to_mg(1000));
        let c16: f32 = lis2dw12_from_fs16_to_mg(1000i16);
        assert_eq!(c16, Lis2dw12::from_fs16_to_mg(1000));
    }
}

#[allow(dead_code)]
unsafe extern "C" {
    fn lis2dw12_filter_path_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_filter_path_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_wkup_dur_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_wkup_dur_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_act_sleep_dur_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_act_sleep_dur_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_write_reg(ctx: *const StmdevCtx, reg: u8, data: *const u8, len: u16) -> i32;
    fn lis2dw12_read_reg(ctx: *const StmdevCtx, reg: u8, data: *mut u8, len: u16) -> i32;
    fn lis2dw12_data_rate_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_data_rate_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_full_scale_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_full_scale_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_block_data_update_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_block_data_update_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_filter_bandwidth_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_filter_bandwidth_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_spi_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_spi_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_i2c_interface_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_i2c_interface_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_cs_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_cs_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_wkup_threshold_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_wkup_threshold_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_threshold_x_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_threshold_x_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_dur_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_dur_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_threshold_y_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_threshold_y_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_threshold_z_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_threshold_z_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_axis_priority_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_axis_priority_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_fifo_watermark_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_fifo_watermark_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_fifo_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_fifo_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_ff_dur_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_ff_dur_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_ff_threshold_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_ff_threshold_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_auto_increment_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_auto_increment_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_reset_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_reset_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_boot_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_boot_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_self_test_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_self_test_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_data_ready_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_data_ready_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_pin_polarity_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_pin_polarity_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_int_notification_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_int_notification_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_pin_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_pin_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_pin_int1_route_set(ctx: *const StmdevCtx, val: *const u8) -> i32;
    fn lis2dw12_pin_int1_route_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_pin_int2_route_set(ctx: *const StmdevCtx, val: *const u8) -> i32;
    fn lis2dw12_pin_int2_route_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_all_on_int1_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_all_on_int1_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_power_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_power_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_offset_weight_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_offset_weight_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_src_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_6d_src_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_6d_feed_data_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_6d_feed_data_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_6d_threshold_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_6d_threshold_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_detection_on_z_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_detection_on_z_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_detection_on_y_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_detection_on_y_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_detection_on_x_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_detection_on_x_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_temperature_raw_get(ctx: *const StmdevCtx, val: *mut i16) -> i32;
    fn lis2dw12_acceleration_raw_get(ctx: *const StmdevCtx, val: *mut i16) -> i32;
    fn lis2dw12_usr_offset_y_set(ctx: *const StmdevCtx, buff: *const u8) -> i32;
    fn lis2dw12_usr_offset_y_get(ctx: *const StmdevCtx, buff: *mut u8) -> i32;
    fn lis2dw12_usr_offset_z_set(ctx: *const StmdevCtx, buff: *const u8) -> i32;
    fn lis2dw12_usr_offset_z_get(ctx: *const StmdevCtx, buff: *mut u8) -> i32;
    fn lis2dw12_from_fs2_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs4_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs8_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs16_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs2_lp1_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs4_lp1_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs8_lp1_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_fs16_lp1_to_mg(lsb: i16) -> f32;
    fn lis2dw12_from_lsb_to_celsius(lsb: i16) -> f32;
    fn lis2dw12_device_id_get(ctx: *const StmdevCtx, buff: *mut u8) -> i32;
    fn lis2dw12_reference_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_reference_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_4d_mode_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_4d_mode_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_shock_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_shock_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
    fn lis2dw12_tap_quiet_set(ctx: *const StmdevCtx, val: u8) -> i32;
    fn lis2dw12_tap_quiet_get(ctx: *const StmdevCtx, val: *mut u8) -> i32;
}

// C ABI callbacks for the driver to read/write registers.
extern "C" fn c_write_reg(handle: *mut c_void, reg: u8, data: *const u8, len: u16) -> i32 {
    unsafe {
        let regs = &mut *(handle as *mut [u8; 256]);
        let idx = reg as usize;
        for i in 0..len as usize {
            regs[idx + i] = *data.add(i);
        }
    }
    0
}

extern "C" fn c_read_reg(handle: *mut c_void, reg: u8, buf: *mut u8, len: u16) -> i32 {
    unsafe {
        let regs = &*(handle as *mut [u8; 256]);
        let idx = reg as usize;
        for i in 0..len as usize {
            *buf.add(i) = regs[idx + i];
        }
    }
    0
}

struct MemTransport {
    pub regs: [u8; 256],
}

#[async_trait::async_trait(?Send)]
impl Transport for MemTransport {
    async fn write_register(&mut self, reg: u8, data: &[u8]) -> Result<(), Error> {
        let idx = reg as usize;
        let end = idx + data.len();
        self.regs[idx..end].copy_from_slice(data);
        Ok(())
    }

    async fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
        let idx = reg as usize;
        let end = idx + buf.len();
        buf.copy_from_slice(&self.regs[idx..end]);
        Ok(())
    }
}
// Moved tests (data_rate, full_scale, block_data_update) are declared later, after this function

#[tokio::test]
async fn parity_filter_path_set_get() {
    let mut regs: Box<[u8; 256]> = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };

    unsafe {
        // Call C filter_path_set with both HP and user offset bits
        let cval = FILTER_PATH_ARG_FDS_MASK | FILTER_PATH_ARG_USR_OFF_ON_OUT_MASK;
        lis2dw12_filter_path_set(&ctx as *const _, cval);

        // snapshot
        let after_c = *regs;

        // Reset
        *regs = [0u8; 256];

        // Call Rust filter_path_set
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.filter_path_set(Fds::HPF_ON_OUT | Fds::USER_OFFSET_ON_OUT)
            .await
            .unwrap();

        let transport_after = dev.destroy();
        let after_rust = transport_after.regs;

        // Compare register memory for parity
        assert_eq!(after_c, after_rust);

        // Now test getter parity: set registers directly then call get on both
        *regs = [0u8; 256];
        // set CTRL6 FDS bit and CTRL_REG7 USR_OFF_ON_OUT
        let mut ctrl6 = regs[LIS2DW12_CTRL6 as usize];
        ctrl6 |= Ctrl6::FDS.bits();
        regs[LIS2DW12_CTRL6 as usize] = ctrl6;
        let mut r7 = regs[LIS2DW12_CTRL_REG7 as usize];
        r7 |= CtrlReg7::USR_OFF_ON_OUT.bits();
        regs[LIS2DW12_CTRL_REG7 as usize] = r7;

        // C getter
        let mut cval_out: u8 = 0;
        lis2dw12_filter_path_get(&ctx as *const _, &mut cval_out as *mut u8);

        // Rust getter
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_val = dev2.filter_path_get().await.unwrap();

        // Note: C `filter_path_get` maps some combinations to enumerated values and
        // will not return a composite 0x11 (both bits) - it falls back to LPF_ON_OUT
        // (0) for unknown combinations, whereas the Rust API uses bitflags that
        // may represent both flags simultaneously. Ensure both drivers read the
        // same register values and validate their interpretations accordingly.
        assert_eq!(cval_out, 0u8);
        assert!(rust_val.contains(Fds::HPF_ON_OUT));
        assert!(rust_val.contains(Fds::USER_OFFSET_ON_OUT));
    }
}

#[tokio::test]
async fn parity_wkup_dur_set_get() {
    let mut regs: Box<[u8; 256]> = Box::new([0u8; 256]);

    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };

    unsafe {
        // Compose wake-up duration: sleep_dur = 3, stationary = 1, wake_dur = 2, ff_dur = 0
        let wd: u8 = (3 & 0x0F) | ((1u8) << 4) | ((2u8) << 5);
        // Call C set (raw register write)
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_WAKE_UP_DUR, &wd as *const u8, 1);
        let after_c = *regs;

        // Reset
        *regs = [0u8; 256];

        // Rust set
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        let wkd = WkupDur {
            sleep_dur: 3,
            stationary: true,
            wake_dur: 2,
            ff_dur: false,
        };
        dev.wkup_dur_set(wkd).await.unwrap();
        let transport_after = dev.destroy();
        let after_rust = transport_after.regs;
        // Ensure set operation wrote the same register value as the C driver
        assert_eq!(after_c, after_rust);

        // Now test getters: set regs with known value and compare
        *regs = [0u8; 256];
        regs[LIS2DW12_WAKE_UP_DUR as usize] = wd;
        let mut c_out = 0u8;
        lis2dw12_read_reg(
            &ctx as *const _,
            LIS2DW12_WAKE_UP_DUR,
            &mut c_out as *mut u8,
            1,
        );
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_out = dev2.wkup_dur_get().await.unwrap();
        assert_eq!(c_out, rust_out.pack());
    }
}

#[tokio::test]
async fn parity_fifo_watermark_and_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Use C to set FIFO watermark to 0x05 and mode to STREAM_TO_FIFO (3)
        lis2dw12_fifo_watermark_set(&ctx as *const _, 0x05_u8);
        lis2dw12_fifo_mode_set(&ctx as *const _, 3u8);
        let after_c = *regs;

        // Reset regs
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.fifo_watermark_set(FifoWatermark(0x05)).await.unwrap();
        dev.fifo_mode_set(Fmode::StreamToFifo).await.unwrap();
        let after_rust = dev.destroy().regs;

        assert_eq!(
            after_c[LIS2DW12_FIFO_CTRL as usize],
            after_rust[LIS2DW12_FIFO_CTRL as usize]
        );

        // Now test getters: set registers and compare
        *regs = [0u8; 256];
        regs[LIS2DW12_FIFO_CTRL as usize] |= 0x05 & FIFO_CTRL_FTH_MASK;
        regs[LIS2DW12_FIFO_CTRL as usize] |= (3u8 << 5) & FIFO_CTRL_FMODE_MASK;
        let mut c_wm = 0u8;
        lis2dw12_fifo_watermark_get(&ctx as *const _, &mut c_wm as *mut u8);
        let mut c_mode = 0u8;
        lis2dw12_fifo_mode_get(&ctx as *const _, &mut c_mode as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let r_wm = dev2.fifo_watermark_get().await.unwrap();
        let r_mode = dev2.fifo_mode_get().await.unwrap();
        assert_eq!(c_wm, r_wm.0);
        assert_eq!(c_mode, r_mode as u8);
    }
}

#[tokio::test]
async fn parity_ff_dur_and_threshold_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Compose free-fall duration/threshold
        // For example: ff_dur = 0x1B (6-bit sample), ff_ths = 0x03
        let ff_dur_val: u8 = 0x15; // arbitrary test
        let ff_ths_val: u8 = 0x03; // within 3-bit field
        lis2dw12_ff_dur_set(&ctx as *const _, ff_dur_val);
        lis2dw12_ff_threshold_set(&ctx as *const _, ff_ths_val);
        let after_c = *regs;

        // Reset and do via Rust
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.ff_dur_set(FfDur(ff_dur_val)).await.unwrap();
        dev.ff_threshold_set(FfThs::Tsh10).await.unwrap(); // Tsh10 maps to 3
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_FREE_FALL as usize],
            after_rust[LIS2DW12_FREE_FALL as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_WAKE_UP_DUR as usize],
            after_rust[LIS2DW12_WAKE_UP_DUR as usize]
        );

        // Getter comparison: set registers directly and verify
        *regs = [0u8; 256];
        // set FREE_FALL.ff_tsh bits and FREE_FALL.ff_dur + WAKE_UP_DUR bit
        regs[LIS2DW12_FREE_FALL as usize] =
            (ff_ths_val & FREE_FALL_FF_THS_MASK) | ((ff_dur_val << 3) & FREE_FALL_FF_DUR_MASK);
        // Put MSB of ff_dur into wake_up_dur[7]
        regs[LIS2DW12_WAKE_UP_DUR as usize] =
            ((ff_dur_val & FF_DUR_ARG_MSB_MASK) << 2) & WAKE_UP_DUR_FF_DUR_MASK;
        let mut c_ff_dur = 0u8;
        lis2dw12_ff_dur_get(&ctx as *const _, &mut c_ff_dur as *mut u8);
        let mut c_ff_ths = 0u8;
        lis2dw12_ff_threshold_get(&ctx as *const _, &mut c_ff_ths as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_dur = dev2.ff_dur_get().await.unwrap();
        let rust_ths = dev2.ff_threshold_get().await.unwrap();
        assert_eq!(c_ff_dur, rust_dur.0);
        assert_eq!(c_ff_ths, rust_ths as u8);
    }
}

#[tokio::test]
async fn parity_tap_dur_mode_set_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set via C: tap_dur = 4, tap_mode = both single/double
        lis2dw12_tap_dur_set(&ctx as *const _, 4u8);
        lis2dw12_tap_mode_set(&ctx as *const _, 1u8);
        let after_c = *regs;

        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.tap_dur_set(TapDur(4)).await.unwrap();
        dev.tap_mode_set(SingleDoubleTap::Both).await.unwrap();
        let after_rust = dev.destroy().regs;
        assert_eq!(
            after_c[LIS2DW12_INT_DUR as usize],
            after_rust[LIS2DW12_INT_DUR as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_WAKE_UP_THS as usize],
            after_rust[LIS2DW12_WAKE_UP_THS as usize]
        );

        // Getter checks
        *regs = [0u8; 256];
        regs[LIS2DW12_INT_DUR as usize] = (4u8 << 4) & INT_DUR_LATENCY_MASK;
        regs[LIS2DW12_WAKE_UP_THS as usize] = WAKE_UP_THS_SINGLE_DOUBLE_TAP_MASK;
        let mut c_dur = 0u8;
        lis2dw12_tap_dur_get(&ctx as *const _, &mut c_dur as *mut u8);
        let mut c_mode = 0u8;
        lis2dw12_tap_mode_get(&ctx as *const _, &mut c_mode as *mut u8);
        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let rust_dur = dev2.tap_dur_get().await.unwrap();
        let rust_mode = dev2.tap_mode_get().await.unwrap();
        assert_eq!(c_dur, rust_dur.0);
        assert_eq!(c_mode, rust_mode as u8);
    }
}

#[tokio::test]
async fn parity_pins_reset_auto_increment_and_selftest() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };
    unsafe {
        // Set via C: pin polarity active low (1), pin mode open-drain (1), int notification latched (1)
        lis2dw12_pin_polarity_set(&ctx as *const _, 1u8);
        lis2dw12_pin_mode_set(&ctx as *const _, 1u8);
        lis2dw12_int_notification_set(&ctx as *const _, 1u8);
        lis2dw12_auto_increment_set(&ctx as *const _, 1u8);
        lis2dw12_reset_set(&ctx as *const _, 1u8);
        lis2dw12_boot_set(&ctx as *const _, 1u8);
        lis2dw12_self_test_set(&ctx as *const _, 1u8);
        let after_c = *regs;

        // Reset and set via Rust
        *regs = [0u8; 256];
        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        dev.pin_polarity_set(HLACTIVE::ActiveLow).await.unwrap();
        dev.pin_mode_set(PpOd::OpenDrain).await.unwrap();
        dev.int_notification_set(Lir::Latched).await.unwrap();
        dev.auto_increment_set(true).await.unwrap();
        dev.reset_set(true).await.unwrap();
        dev.boot_set(true).await.unwrap();
        dev.self_test_set(SelfTest::Positive).await.unwrap();
        let after_rust = dev.destroy().regs;

        assert_eq!(
            after_c[LIS2DW12_CTRL3 as usize],
            after_rust[LIS2DW12_CTRL3 as usize]
        );
        assert_eq!(
            after_c[LIS2DW12_CTRL2 as usize],
            after_rust[LIS2DW12_CTRL2 as usize]
        );

        // Getter parity: set registers and verify get matches C getters
        *regs = [0u8; 256];
        regs[LIS2DW12_CTRL3 as usize] =
            Ctrl3::H_LACTIVE.bits() | Ctrl3::PP_OD.bits() | Ctrl3::LIR.bits();
        regs[LIS2DW12_CTRL2 as usize] =
            Ctrl2::IF_ADD_INC.bits() | Ctrl2::SOFT_RESET.bits() | Ctrl2::BOOT.bits();
        let mut c_pol = 0u8;
        lis2dw12_pin_polarity_get(&ctx as *const _, &mut c_pol as *mut u8);
        let mut c_pm = 0u8;
        lis2dw12_pin_mode_get(&ctx as *const _, &mut c_pm as *mut u8);
        let mut c_int = 0u8;
        lis2dw12_int_notification_get(&ctx as *const _, &mut c_int as *mut u8);
        let mut c_auto = 0u8;
        lis2dw12_auto_increment_get(&ctx as *const _, &mut c_auto as *mut u8);
        let mut c_reset = 0u8;
        lis2dw12_reset_get(&ctx as *const _, &mut c_reset as *mut u8);
        let mut c_boot = 0u8;
        lis2dw12_boot_get(&ctx as *const _, &mut c_boot as *mut u8);
        let mut c_st = 0u8;
        lis2dw12_self_test_get(&ctx as *const _, &mut c_st as *mut u8);

        let transport2 = MemTransport { regs: *regs };
        let mut dev2 = Lis2dw12::new(transport2);
        let r_pol = dev2.pin_polarity_get().await.unwrap() as u8;
        let r_pm = dev2.pin_mode_get().await.unwrap() as u8;
        let r_int = dev2.int_notification_get().await.unwrap() as u8;
        let r_auto = dev2.auto_increment_get().await.unwrap();
        let r_reset = dev2.reset_get().await.unwrap();
        let r_boot = dev2.boot_get().await.unwrap();
        let r_st = dev2.self_test_get().await.unwrap() as u8;

        assert_eq!(c_pol, r_pol);
        assert_eq!(c_pm, r_pm);
        assert_eq!(c_int, r_int);
        assert_eq!(c_auto, r_auto as u8);
        assert_eq!(c_reset, r_reset as u8);
        assert_eq!(c_boot, r_boot as u8);
        assert_eq!(c_st, r_st);
    }
}

#[tokio::test]
async fn parity_temperature_and_accel_raw_get() {
    let mut regs = Box::new([0u8; 256]);
    let ctx = StmdevCtx {
        write_reg: Some(c_write_reg),
        read_reg: Some(c_read_reg),
        mdelay: None,
        handle: (&mut *regs) as *mut _ as *mut c_void,
        priv_data: ptr::null_mut(),
    };

    unsafe {
        // Write a temperature (16-bit little-endian) to OUT_T_L/H
        let t_raw: i16 = -1234; // signed value to test sign handling
        let t_bytes = t_raw.to_le_bytes();
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_OUT_T_L, t_bytes.as_ptr(), 2);

        // Write acceleration for X/Y/Z as 16-bit little-endian values
        let x_raw: i16 = 1000;
        let y_raw: i16 = -1000;
        let z_raw: i16 = 500;
        let mut accel_bytes = [0u8; 6];
        accel_bytes[0..2].copy_from_slice(&x_raw.to_le_bytes());
        accel_bytes[2..4].copy_from_slice(&y_raw.to_le_bytes());
        accel_bytes[4..6].copy_from_slice(&z_raw.to_le_bytes());
        lis2dw12_write_reg(&ctx as *const _, LIS2DW12_OUT_X_L, accel_bytes.as_ptr(), 6);

        let transport = MemTransport { regs: *regs };
        let mut dev = Lis2dw12::new(transport);
        let got_temp = dev.temperature_raw_get().await.unwrap();
        let got_accel = dev.acceleration_raw_get().await.unwrap();

        // Compare with C functions
        let mut c_temp: i16 = 0;
        lis2dw12_temperature_raw_get(&ctx as *const _, &mut c_temp as *mut i16);
        assert_eq!(c_temp, got_temp);

        let mut c_acc: [i16; 3] = [0; 3];
        lis2dw12_acceleration_raw_get(&ctx as *const _, c_acc.as_mut_ptr());
        assert_eq!(c_acc[0], got_accel[0]);
        assert_eq!(c_acc[1], got_accel[1]);
        assert_eq!(c_acc[2], got_accel[2]);
    }
}
