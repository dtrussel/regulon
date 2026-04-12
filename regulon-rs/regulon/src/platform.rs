//! # `platform`
//!
//! Shared numeric helpers and compile-time platform assertions.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-PR-010, RON-PR-011, RON-PR-021, RON-QR-001, RON-QR-003
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use core::mem::size_of;

/// Floating-point type used by the library.
#[cfg(feature = "double_precision")]
pub type RonFloat = f64;

/// Floating-point type used by the library.
#[cfg(not(feature = "double_precision"))]
pub type RonFloat = f32;

#[cfg(feature = "double_precision")]
const _: [(); 8] = [(); size_of::<RonFloat>()];

#[cfg(not(feature = "double_precision"))]
const _: [(); 4] = [(); size_of::<RonFloat>()];

/// Smallest practical non-zero magnitude for divisor checks.
pub const DIVISOR_EPSILON: RonFloat = 1.0e-9 as RonFloat;

/// Returns `true` when the value is finite.
#[must_use]
pub fn is_finite(value: RonFloat) -> bool {
    value.is_finite()
}

/// Returns the absolute value.
#[must_use]
pub fn abs(value: RonFloat) -> RonFloat {
    value.abs()
}

/// Clamps a value to a bounded range.
#[must_use]
pub fn clamp(value: RonFloat, min: RonFloat, max: RonFloat) -> RonFloat {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

/// Returns `true` when the values share the same non-zero sign.
#[must_use]
pub fn same_sign_nonzero(lhs: RonFloat, rhs: RonFloat) -> bool {
    (lhs > 0.0 && rhs > 0.0) || (lhs < 0.0 && rhs < 0.0)
}

/// Returns `true` when a value is close enough to zero to reject as divisor.
#[must_use]
pub fn is_near_zero(value: RonFloat) -> bool {
    abs(value) <= DIVISOR_EPSILON
}
