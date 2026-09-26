//! # `platform`
//!
//! Shared numeric helpers and compile-time platform assertions.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-PR-010, RON-PR-011, RON-PR-021, RON-QR-001, RON-QR-003,
//! RON-FR-500, RON-FR-503
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

/// Newton iterations for [`sqrt`]: enough to converge from the initial guess
/// `max(value, 1)` for the finite inputs the modules pass, as in the C
/// `ron_util_sqrt`.
const SQRT_STEPS: u8 = 30;

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

/// Square root by a fixed number of Newton iterations, so `no_std` builds need
/// no math library and the cost is bounded. Callers pass non-negative finite
/// values.
///
/// **Satisfies:** RON-FR-500
#[must_use]
pub fn sqrt(value: RonFloat) -> RonFloat {
    let mut estimate = if value > 1.0 { value } else { 1.0 };
    for _ in 0..SQRT_STEPS {
        estimate = 0.5 * (estimate + (value / estimate));
    }
    estimate
}

/// Returns `-1` for negative values and `1` otherwise (including zero).
///
/// **Satisfies:** RON-FR-503
#[must_use]
pub fn sign_nonzero(value: RonFloat) -> RonFloat {
    if value < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Limits the change from `previous` to `value` to `max_rate * dt`. A
/// non-positive `max_rate` disables limiting. Returns the limited value and
/// whether limiting was applied.
///
/// **Satisfies:** RON-FR-022, RON-FR-703
#[must_use]
pub fn rate_limit(
    value: RonFloat,
    previous: RonFloat,
    max_rate: RonFloat,
    dt: RonFloat,
) -> (RonFloat, bool) {
    if max_rate <= 0.0 {
        return (value, false);
    }
    let max_delta = max_rate * dt;
    let delta = value - previous;
    if delta > max_delta {
        (previous + max_delta, true)
    } else if delta < -max_delta {
        (previous - max_delta, true)
    } else {
        (value, false)
    }
}
