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
    /// A computation produced a non-finite result or met a matrix that is not
    /// positive definite; the component's state was left unchanged.
    Numerical(&'static str),
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
            Self::Numerical(message) => write!(formatter, "numerical failure: {message}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for RonError {}
