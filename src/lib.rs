#![no_std]

// Compile-time assertion: async and blocking features must not be enabled together
#[cfg(all(feature = "async", feature = "blocking"))]
compile_error!("Features 'async' and 'blocking' cannot be enabled at the same time.");

extern crate alloc;

pub mod error;
pub mod lis2dw12;
pub mod reg;
pub mod transport;
