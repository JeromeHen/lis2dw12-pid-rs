# Copilot / AI agent instructions — lis2dw12-pid-rs

Purpose: make it fast for an AI coding agent to be productive in this repo.

Summary (big picture)
- This crate is a pure-Rust driver for the LIS2DW12 accelerometer. It is written
  as a no_std crate (uses alloc) and exposes a clean, low-level register API.
- Key responsibilities are split across files: `src/reg.rs` (register
  constants, bitflags, and small newtypes), `src/lis2dw12.rs` (async API and
  register read/write helpers), `src/transport.rs` (Transport trait + I2C
  transport), and `src/error.rs` (single Error type).
- There is an original C reference implementation under `lis2dw12-pid/` and
  host parity tests in `tests/parity.rs` that compare the Rust implementation
  with the C driver. `build.rs` compiles the C driver into a static lib for
  host tests.

What to know before you change code
- Async-by-default: Cargo.toml enables the `async` feature by default. The
  public API methods on `Lis2dw12` are async (use the `Transport` trait). If
  you add low-level synchronous implementations, follow the existing
  feature-pattern (`blocking` feature).
- Register mapping: high-level API methods mirror C driver behaviour — prefer
  to read or write registers using `read_reg` / `write_reg`, and use the
  `reg` constants and `bitflags` defined in `src/reg.rs`.
- Small helper newtypes and pack/unpack: the code often wraps numeric fields
  (e.g. `TapThreshold(u8)`, `WkupDur`) and provides `pack()`/`unpack()` where a
  composite bitfield maps to a register. Preserve that style when adding
  fields or conversions.
- Tests use a memory-backed `MemTransport` (see `tests/parity.rs`) to emulate
  device registers. The parity tests also call the host C driver via FFI to
  assert behavior equality — keep parity tests in sync when you edit behavior.

Developer workflows (commands you should use)
- Run unit + integration tests on host (builds the C driver, runs tokio tests):

  ```bash
  cargo test
  ```

- Run tests without default features (useful when working on `blocking`):

  ```bash
  cargo test --no-default-features --features blocking
  ```

- Build for an embedded target: `build.rs` purposely skips compiling the C
  driver if `TARGET` contains `thumb`. Cross-compiles should set the
  appropriate target and avoid assuming host-only test behavior.

Key project conventions and gotchas
- Keep the Rust API behaviour consistent with the C driver. Parity tests are
  the canonical examples that explain small differences (e.g., `filter_path_get`
  in C collapses unknown combinations to a single enumerated value, while the
  Rust API exposes bitflags and accepts composite values).
- Error model is intentionally tiny — `Error::BusError` for any I/O failure.
  When adding more fine-grained error handling, consider compatibility with
  current tests and API simplicity.
- The `Transport` trait is the layer of abstraction for bus access. Unit tests
  and parity tests use `MemTransport` — new hardware transports should implement
  `Transport` and follow the same async/feature pattern.
- Avoid modifying `lis2dw12-pid/` unless you know you must update C reference
  behaviour; changes there affect every parity test (and must be built by
  `build.rs` on host builds).

Files you will interact with most
- src/lis2dw12.rs — core device API (many getter/setter methods, async
  implementations). Example: `device_id_get`, `data_rate_set/get`.
- src/reg.rs — register offsets, bitflags, enums, newtypes and pack/unpack
  helpers.
- src/transport.rs — transport trait + I2C transport wrapper used by tests.
- tests/parity.rs — parity tests comparing Rust API vs original C driver.
- build.rs and lis2dw12-pid/* — host test integration with the original driver.

Good first tasks for a bot (examples)
- Implement a SPI transport in `src/transport.rs` similar to the existing
  `I2cTransport` (follow the async/feature gating pattern). Add unit tests and
  if appropriate add a parity test using `MemTransport`.
- Add a new getter/setter pair to `src/lis2dw12.rs` and mirror the mapping in
  `src/reg.rs`. Update `tests/parity.rs` to add host parity checks using the
  C functions (if the C interface exists), or add a `MemTransport`-based test.
- If you refactor bitfield or enum representations, update `tests/parity.rs`
  and ensure `build.rs` plus the locally compiled C lib remain in sync.

If you get stuck — short checklist before asking a human
- Did you run `cargo test` locally to check parity and unit tests?
- If the change affects registers or the C mapping, did you update
  `tests/parity.rs` to ensure parity with the C driver?
- For any added features, are they conditional behind the `features` section
  in `Cargo.toml` and consistent with `no_std` vs host-only logic in `build.rs`?

End: request for feedback
If anything is unclear or you want this doc to mention other workflows (e.g.
CI, cross-compilation, or code generation with bindgen), tell me which area
to expand and I'll iterate. ✅
