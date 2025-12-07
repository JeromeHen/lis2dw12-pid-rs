# lis2dw12-pid-rs

[![crates.io](https://img.shields.io/crates/v/lis2dw12-pid-rs.svg)](https://crates.io/crates/lis2dw12-pid-rs) [![docs.rs](https://img.shields.io/docsrs/lis2dw12-pid-rs)](https://docs.rs/lis2dw12-pid-rs) [![CI](https://github.com/JeromeHen/lis2dw12-pid-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/JeromeHen/lis2dw12-pid-rs/actions)
# LIS2DW12 Rust Driver

Pure-Rust, no_std (with alloc) driver for the ST LIS2DW12 3-axis accelerometer.

This crate is a low-level, register-oriented implementation of the LIS2DW12 driver.
It is a manual reimplementation of the original C reference driver 
(maintained in the excluded `lis2dw12-pid/` folder and available at [STMicroelectronics lis2dw12-pid](https://github.com/STMicroelectronics/lis2dw12-pid)) 
and includes parity tests to ensure the behavior matches the C implementation.

The original C driver is licensed under BSD-3-Clause. This Rust crate is licensed under MIT OR Apache-2.0.


Key design goals
- no_std-compatible (uses alloc) so it can be used on constrained targets
- async-by-default API (feature-gated); a `blocking` feature is also available
- small and explicit register-level API that directly mirrors the C driver
- comprehensive tests and host parity tests against the original C driver

Crate features
- `default` = `async` (uses `embedded-hal-async` + `async-trait`)
- `blocking` = blocking transports using `embedded-hal`
  
    Note: the `blocking` feature exists in Cargo.toml but is currently only partially implemented. The crate defines the synchronous `Transport` trait used for blocking builds, but there is no built-in blocking `I2cTransport` implementation yet (the included `I2cTransport` is async-only). SPI support is also TODO. If you need blocking I2C/SPI today you will need to implement a small wrapper that implements the `Transport` trait for your platform — PRs to add a full blocking transport are very welcome.

License: MIT OR Apache-2.0 — see `LICENSE`.

Quick overview
---------------

This crate exposes a single high-level device object: `Lis2dw12<T>` where `T` is the transport implementation. The crate also exposes a small set of register constants, bitflags and helper newtypes in `reg` for working with compact bitfields.

Transport
---------

The crate uses a `Transport` trait to abstract bus access. There are a few feature-dependent transport shapes to be aware of:

- Async mode (default): the `Transport` trait defines async `read_register` / `write_register` methods and `I2cTransport` wraps any `embedded-hal-async` I2C implementation.
- Shared-bus (default): the `shared-bus` feature adds the `embassy-sync` dependency and makes `I2cTransport.bus` a `&Mutex<CriticalSectionRawMutex, RefCell<Option<I2C>>>` so the same underlying I2C instance can be safely shared across multiple drivers/tasks in an async executor. This is useful when multiple peripherals live on the same physical bus and you want task-safe access without cloning the bus.
- Blocking mode (`--features blocking`): the crate defines a synchronous `Transport` trait for blocking builds but the crate currently does not provide built-in blocking `I2cTransport`/SPI implementations. If you need blocking transports, implement the `Transport` trait for your platform or supply a wrapper — PRs to add built-in blocking transports are welcome.

If you need another type of transport logic, implement the `Transport` trait for your type and supply it to `Lis2dw12::new()`.

Usage examples
--------------

These examples are intentionally minimal — supply a working platform-dependent I2C instance (async or blocking) when using in real firmware.

Async examples

```rust
use lis2dw12_pid_rs::lis2dw12::Lis2dw12;
use lis2dw12_pid_rs::transport::I2cTransport;
use lis2dw12_pid_rs::reg::LIS2DW12_ID;

// `i2c` implements `embedded_hal_async::i2c::I2c`
let i2c: YourAsyncI2c = /* ... */;
// Instantiate device with I2C transport
let mut dev = Lis2dw12::new(I2cTransport::new(i2c, 0x19));

// Note: if you enable the `shared-bus` feature, the I2C transport can be setup like this:
// static I2C_BUS: Mutex<CriticalSectionRawMutex, RefCell<Option<YourAsyncI2c>>> =
//     Mutex::new(RefCell::new(None));

// Put the I2C peripheral inside the bus before transport initialization
// *I2C_BUS.lock().await.borrow_mut() = Some(i2c);

// Build device using the shared-bus transport
// let mut dev = Lis2dw12::new(I2cTransport::new(&I2C_BUS, 0x19));

// Reset device
dev.reset_set(true).await.expect("i2c failed");

while dev.reset_get().await.expect("i2c failed") {
    // e.g. embassy_time::Timer::after(Duration::from_millis(10)).await;
}

// Read WHO_AM_I
let id = dev.device_id_get().await.expect("i2c failed");
assert_eq!(id, LIS2DW12_ID);

// Configure full-scale to ±2g
lis2dw12.full_scale_set(Fs::G2).await.unwrap();

// Set continuous low-power mode 2
lis2dw12.power_mode_set(Mode::ContLowPower2).await.unwrap();

// Read raw acceleration
let raw = dev.acceleration_raw_get().await.expect("i2c failed"); // [i16; 3]

// Convert mg -> g
// Convert depends on your full-scale selection and power mode using helpers in the API
let data_g = raw.map(|v| Lis2dw12::from_fs2_to_mg(v) / 1000.0);
```

API highlights
--------------

- Lis2dw12::new(iface) — construct the driver with your transport
- read_reg / write_reg — low-level register operations
- device_id_get() — WHO_AM_I (device id)
- acceleration_raw_get() — read raw X/Y/Z samples
- conversion helpers — from_fs2_to_mg, from_fs4_to_mg, from_lsb_to_celsius, etc.
- numerous register-level setters/getters for power mode, filters, FIFO, taps, wake-up, interrupts and more

Testing & parities
-------------------

The repository includes unit tests that use a small `MemTransport`-style memory-backed transport to emulate device registers. In addition, there are parity tests under `tests/parity.rs` that compile and use the original C reference implementation (contained in `lis2dw12-pid/`) and compare results to the Rust port. `build.rs` will compile that C library for host tests, so running `cargo test` on a host should exercise parity.

Examples & integration
----------------------

This repository focuses on a small, low-level driver for platform authors. For embedded applications, wiring and runtime setup (RTOS/async executor) is platform-specific. Integrating the driver typically looks like:

1. Provide an I2C (or SPI) implementation that implements the crate's `Transport` trait.
2. Construct `Lis2dw12` with your transport and call async methods in an executor (or a blocking transport on blocking builds).

Contributing
------------

Contributions are welcome — PRs that add missing register APIs, improve parity coverage, add a SPI transport implementation (TODO in repo), or improve docs/tests are especially appreciated. See the `lis2dw12-pid/` folder for the original C driver used for parity.

License
-------

The crate is dual-licensed under MIT or Apache-2.0 — see `LICENSE` for details.

Contact / Author
----------------

Author: Jérôme Hendrick <jerome.hendrick@gmail.com>

Thanks for checking out this driver!