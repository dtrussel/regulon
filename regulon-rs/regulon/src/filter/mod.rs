//! # `filter`
//!
//! Reusable signal conditioning components independent of controller modules.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-100-RON-FR-103, RON-FR-110-RON-FR-111,
//! RON-FR-130-RON-FR-131, RON-PR-001-RON-PR-004, RON-PR-022
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::platform::{abs, is_finite, is_near_zero, RonFloat};

#[cfg(feature = "double_precision")]
const TWO_PI: RonFloat = core::f64::consts::PI * 2.0;

#[cfg(not(feature = "double_precision"))]
const TWO_PI: RonFloat = core::f32::consts::PI * 2.0;

/// Filter fault register.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FilterFault(u8);

impl FilterFault {
    /// No fault bits set.
    pub const NONE: Self = Self(0x00);
    /// Configuration is invalid or unstable.
    pub const CONFIG_INVALID: Self = Self(0x01);
    /// Input sample is not finite.
    pub const INPUT_NOT_FINITE: Self = Self(0x02);

    /// Returns the raw bitfield.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when no fault bits are set.
    #[must_use]
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
}

/// Filter status bitfield.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FilterStatus(u16);

impl FilterStatus {
    /// Normal operation.
    pub const OK: Self = Self(0x0000);
    /// Rate limiting was applied.
    pub const RATE_LIMITED: Self = Self(0x0001);
    /// Fault is latched.
    pub const FAULT: Self = Self(0x0002);

    /// Returns the raw bitfield.
    #[must_use]
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Returns `true` when `other` bits are present.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }
}

impl core::ops::BitOr for FilterStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitOrAssign for FilterStatus {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Read-only filter state snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FilterSnapshot {
    /// Last output sample.
    pub last_output: RonFloat,
    /// Last status value.
    pub status: FilterStatus,
    /// Latched fault value.
    pub fault: FilterFault,
}

/// Configuration for a first-order low-pass filter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Lp1Config {
    /// Direct smoothing-factor configuration.
    Alpha {
        /// Exponential smoothing factor in `(0, 1]`.
        alpha: RonFloat,
        /// Initial output state.
        initial_output: RonFloat,
    },
    /// Cutoff frequency configuration.
    Cutoff {
        /// Filter cutoff frequency in hertz.
        cutoff_hz: RonFloat,
        /// Sample period in seconds used to derive `alpha`.
        sample_period: RonFloat,
        /// Initial output state.
        initial_output: RonFloat,
    },
}

impl Lp1Config {
    /// Returns the initial output state.
    #[must_use]
    pub const fn initial_output(self) -> RonFloat {
        match self {
            Self::Alpha { initial_output, .. } | Self::Cutoff { initial_output, .. } => {
                initial_output
            }
        }
    }

    /// Returns the validated smoothing factor.
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is non-finite or unstable.
    pub fn alpha(self) -> Result<RonFloat, FilterFault> {
        match self {
            Self::Alpha { alpha, .. } => validate_alpha(alpha),
            Self::Cutoff {
                cutoff_hz,
                sample_period,
                ..
            } => {
                if !is_finite(cutoff_hz)
                    || !is_finite(sample_period)
                    || cutoff_hz <= 0.0
                    || sample_period <= 0.0
                    || is_near_zero(sample_period)
                {
                    return Err(FilterFault::CONFIG_INVALID);
                }
                let omega = TWO_PI * cutoff_hz * sample_period;
                validate_alpha(omega / (1.0 + omega))
            }
        }
    }
}

/// First-order low-pass filter instance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lp1 {
    alpha: RonFloat,
    state: FilterSnapshot,
}

impl Lp1 {
    /// Creates a new low-pass filter instance.
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid or unstable.
    pub fn new(config: Lp1Config) -> Result<Self, FilterFault> {
        let alpha = config.alpha()?;
        Ok(Self {
            alpha,
            state: FilterSnapshot {
                last_output: config.initial_output(),
                status: FilterStatus::OK,
                fault: FilterFault::NONE,
            },
        })
    }

    /// Resets the filter output state.
    pub fn reset(&mut self, output: RonFloat) {
        self.state.last_output = output;
        self.state.status = FilterStatus::OK;
        self.state.fault = FilterFault::NONE;
    }

    /// Applies one input sample.
    ///
    /// # Errors
    ///
    /// Returns an error when a fault is latched or the input is non-finite.
    pub fn step(&mut self, input: RonFloat) -> Result<(RonFloat, FilterStatus), FilterFault> {
        if !self.state.fault.is_none() {
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        if !is_finite(input) {
            self.state.fault = FilterFault::INPUT_NOT_FINITE;
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        self.state.last_output =
            (self.alpha * input) + ((1.0 - self.alpha) * self.state.last_output);
        self.state.status = FilterStatus::OK;
        Ok((self.state.last_output, self.state.status))
    }

    /// Returns a state snapshot.
    #[must_use]
    pub const fn state(&self) -> FilterSnapshot {
        self.state
    }
}

/// Standalone asymmetric rate limiter configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateLimiterConfig {
    /// Positive slew-rate limit in units/second.
    pub rise_limit: RonFloat,
    /// Negative slew-rate limit in units/second.
    pub fall_limit: RonFloat,
    /// Initial output state.
    pub initial_output: RonFloat,
}

impl RateLimiterConfig {
    /// Validates the rate-limiter configuration.
    ///
    /// # Errors
    ///
    /// Returns an error when either rate limit is non-finite or non-positive.
    pub fn validate(self) -> Result<(), FilterFault> {
        if !is_finite(self.rise_limit)
            || !is_finite(self.fall_limit)
            || self.rise_limit <= 0.0
            || self.fall_limit <= 0.0
        {
            return Err(FilterFault::CONFIG_INVALID);
        }
        Ok(())
    }
}

/// Standalone asymmetric slew-rate limiter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateLimiter {
    config: RateLimiterConfig,
    state: FilterSnapshot,
}

impl RateLimiter {
    /// Creates a new rate limiter.
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: RateLimiterConfig) -> Result<Self, FilterFault> {
        config.validate()?;
        Ok(Self {
            config,
            state: FilterSnapshot {
                last_output: config.initial_output,
                status: FilterStatus::OK,
                fault: FilterFault::NONE,
            },
        })
    }

    /// Resets the limiter output state.
    pub fn reset(&mut self, output: RonFloat) {
        self.state.last_output = output;
        self.state.status = FilterStatus::OK;
        self.state.fault = FilterFault::NONE;
    }

    /// Applies one input sample with asymmetric slew limits.
    ///
    /// # Errors
    ///
    /// Returns an error when a fault is latched or the input/sample period is invalid.
    pub fn step(
        &mut self,
        input: RonFloat,
        dt: RonFloat,
    ) -> Result<(RonFloat, FilterStatus), FilterFault> {
        if !self.state.fault.is_none() {
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        if !is_finite(input) || !is_finite(dt) || dt <= 0.0 || is_near_zero(dt) {
            self.state.fault = FilterFault::INPUT_NOT_FINITE;
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        let upper = self.state.last_output + (self.config.rise_limit * dt);
        let lower = self.state.last_output - (self.config.fall_limit * dt);
        let output = if input > upper {
            upper
        } else if input < lower {
            lower
        } else {
            input
        };
        self.state.status = if abs(output - input) > (RonFloat::EPSILON * 4.0) {
            FilterStatus::RATE_LIMITED
        } else {
            FilterStatus::OK
        };
        self.state.last_output = output;
        Ok((output, self.state.status))
    }

    /// Returns a state snapshot.
    #[must_use]
    pub const fn state(&self) -> FilterSnapshot {
        self.state
    }
}

fn validate_alpha(alpha: RonFloat) -> Result<RonFloat, FilterFault> {
    if !is_finite(alpha) || alpha <= 0.0 || alpha > 1.0 {
        return Err(FilterFault::CONFIG_INVALID);
    }
    Ok(alpha)
}
