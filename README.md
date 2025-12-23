# lis2dw12-pid-rs

[![crates.io](https://img.shields.io/crates/v/lis2dw12-pid-rs.svg)](https://crates.io/crates/lis2dw12-pid-rs)
[![docs.rs](https://docs.rs/lis2dw12-pid-rs/badge.svg)](https://docs.rs/lis2dw12-pid-rs)
[![CI](https://github.com/JeromeHen/lis2dw12-pid-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/JeromeHen/lis2dw12-pid-rs/actions)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20%7C%20Apache--2.0-blue.svg)](./LICENSE)

## LIS2DW12 Rust Driver

Pure-Rust, `no_std` (with `alloc`) driver for the ST LIS2DW12 3-axis accelerometer.

This crate is a low-level, register-oriented implementation derived from the original C reference driver [STMicroelectronics lis2dw12-pid](https://github.com/STMicroelectronics/lis2dw12-pid) (included under `lis2dw12-pid/`). It exposes a compact, register-centric API and a small transport abstraction so you can plug in your preferred I2C or SPI implementation.

This Rust crate is licensed under MIT OR Apache-2.0.

## Design Goals
- `no_std` compatible (with `alloc`)
- Async-by-default API (feature-gated); a `blocking` feature is available
- Small, explicit register-level API that mirrors the C reference behavior
- Transport-agnostic: users provide the concrete bus implementation
- Host parity tests to ensure behavior matches the original C driver

## Crate Features

- `async` — enable asynchronous transport support (enabled by default, asynchronous `Transport` trait)
- `blocking` — enable blocking transport support (synchronous `Transport` trait)
- `shared-bus` — optional shared-bus support when using async transports (uses `embassy-sync` mutex types)

## Quick Overview

High-level device object: `Lis2dw12<T>`, where `T` is a transport implementation. The `reg` module exports register addresses, bitflags and small newtypes (enums and wrappers) for type-safe register manipulation.

Common items:

- `lis2dw12_pid_rs::Lis2dw12` — main driver type
- `lis2dw12_pid_rs::transport::I2cTransport` — provided async/blocking I2C transport adapter
- `lis2dw12_pid_rs::reg::LIS2DW12_I2C_ADDR` — default 7-bit I2C address

## API highlights

Below are common operations you'll use when integrating this driver:

- Construction
    - `Lis2dw12::new(transport)` — create the device instance
    - `Lis2dw12::destroy(self)` — consume driver and return underlying transport

- Device info
    - `device_id_get()` — read `WHO_AM_I` / device id

- Power, mode & filters
    - power/mode setters/getters (power mode, low-noise flags, sleep/standby)
    - `filter_path_set()` / `filter_path_get()` — configure output filter path

- Data & conversions
    - `acceleration_raw_get()` — read raw X/Y/Z samples
    - conversion helpers (e.g. `from_fs16_to_mg`) for converting LSB -> mg

- Interrupts & events
    - wake-up, tap, free-fall, six-d detection setters/getters
    - interrupt routing and status helpers

- FIFO & timestamps
    - FIFO configuration and sample count helpers

The `reg` module contains enums like `Odr`, `Fs`, `Fmode`, and newtypes like `WkupDur`, `TapThreshold` to help pack/unpack bitfields.

## Transport Abstraction

`Lis2dw12<T>` is generic over a `Transport` implementation (see `src/transport.rs`). The crate ships an async/blocking `I2cTransport` and an async/blocking `SpiTransport` that wraps an `embedded-hal-async`-compatible I2C or SPI buses and the default I2C address `reg::LIS2DW12_I2C_ADDR`.

- Without `shared-bus`: pass your concrete I2C implementation directly.
- With `shared-bus`: pass a shared/mutex-wrapped bus or mutex+refcell wrapped (uses `embassy-sync` types when enabled).

## Examples

Minimal I2C async example (replace `...` with your platform's I2C type). You can easily adapt this to blocking or SPI usage by using the corresponding transport types and compile features.

```rust
use lis2dw12_pid_rs::{lis2dw12::Lis2dw12, transport::I2cTransport, reg::LIS2DW12_ID};

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

The blocking usage is similar but requires a blocking `Transport` implementation.

## Testing & Parity

Unit tests use a memory-backed `MemTransport` to emulate registers. Parity tests in `tests/parity.rs` compare the Rust implementation against the original C reference in `lis2dw12-pid/`. `build.rs` will compile the C driver for host tests when running `cargo test` on a host system.

## Contributing

Contributions are welcome — PRs that add missing register APIs, improve parity coverage, or improve docs/examples are appreciated. If you change register mappings or behavior, update `tests/parity.rs` so host parity tests remain meaningful.

## License

Dual-licensed under MIT OR Apache-2.0 — see `LICENSE` for details.

## Contact / Author

Author: Jérôme Hendrick <jerome.hendrick@gmail.com>

Thanks for checking out this driver!