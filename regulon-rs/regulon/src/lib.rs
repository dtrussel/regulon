//! # `regulon`
//!
//! Rust-first implementation of the Regulon PID controller baseline.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-PR-001-RON-PR-022,
//! RON-SR-001-RON-SR-033, RON-QR-001-RON-QR-031
//! **SPDX-License-Identifier:** MIT

#![no_std]
#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
extern crate std;

pub mod error;
pub mod filter;
pub mod pid;
pub mod platform;

pub use error::RonError;
pub use filter::{
    FilterFault, FilterSnapshot, FilterStatus, Lp1, Lp1Config, RateLimiter, RateLimiterConfig,
};
pub use pid::{
    AntiWindupMode, DerivativeMode, FeedForwardConfig, FeedForwardMode, IntegrationMethod,
    NormalizationConfig, NormalizationRange, Pid, PidConfig, PidFault, PidMode, PidSnapshot,
    PidStatus, SafePolicy,
};
pub use platform::RonFloat;
