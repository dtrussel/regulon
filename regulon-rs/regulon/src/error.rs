//! # `error`
//!
//! Error types returned by the Rust-first PID API.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-SR-001, RON-SR-002, RON-SR-010, RON-SR-012, RON-QR-012
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use core::fmt;

use crate::pid::PidFault;

/// Library error returned by public APIs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RonError {
    /// The supplied configuration is internally inconsistent.
    ConfigInvalid(&'static str),
    /// An argument is invalid for the current operation.
    InvalidArgument(&'static str),
    /// A fault has been latched by the controller.
    Fault(PidFault),
}

impl fmt::Display for RonError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ConfigInvalid(message) => {
                write!(formatter, "invalid PID configuration: {message}")
            }
            Self::InvalidArgument(message) => {
                write!(formatter, "invalid PID argument: {message}")
            }
            Self::Fault(fault) => write!(formatter, "PID fault latched: {}", fault.bits()),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RonError {}
