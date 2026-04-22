//! # `pid::config`
//!
//! PID configuration validation and normalization helpers.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-002, RON-FR-010-RON-FR-013, RON-FR-021, RON-FR-033,
//! RON-FR-035, RON-SR-002, RON-SR-020
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use crate::{
    error::RonError,
    platform::{is_finite, is_near_zero, RonFloat},
};

use super::{AntiWindupMode, FeedForwardMode, NormalizationConfig, NormalizationRange, PidConfig};

impl NormalizationConfig {
    /// Validates the normalization configuration.
    ///
    /// # Errors
    ///
    /// Returns an error when either configured range is non-finite or degenerate.
    pub fn validate(&self) -> Result<(), RonError> {
        validate_range(self.input_min, self.input_max, "input normalization range")?;
        validate_range(
            self.output_min,
            self.output_max,
            "output normalization range",
        )?;
        Ok(())
    }

    #[must_use]
    pub(crate) fn normalize_input(self, value: RonFloat) -> RonFloat {
        normalize_to_range(value, self.input_min, self.input_max, self.range)
    }

    #[must_use]
    pub(crate) fn normalize_output(self, value: RonFloat) -> RonFloat {
        normalize_to_range(value, self.output_min, self.output_max, self.range)
    }

    #[must_use]
    pub(crate) fn denormalize_output(self, value: RonFloat) -> RonFloat {
        denormalize_from_range(value, self.output_min, self.output_max, self.range)
    }
}

impl PidConfig {
    /// Validates the full PID configuration.
    ///
    /// # Errors
    ///
    /// Returns an error when any PID parameter is non-finite, out of range, or
    /// logically inconsistent.
    pub fn validate(&self) -> Result<(), RonError> {
        validate_non_negative(self.kp, "kp")?;
        validate_non_negative(self.ki, "ki")?;
        validate_non_negative(self.kd, "kd")?;
        validate_non_negative(self.derivative_filter, "derivative filter")?;
        validate_range(self.output_min, self.output_max, "output limits")?;
        validate_range(self.integral_min, self.integral_max, "integral clamp")?;
        validate_probability(self.proportional_weight, "proportional weight")?;
        validate_probability(self.derivative_weight, "derivative weight")?;
        validate_non_negative_or_disabled(self.rate_limit, "rate limit")?;
        validate_non_negative_or_disabled(
            self.integral_overflow_threshold,
            "integral overflow threshold",
        )?;
        validate_non_negative_or_disabled(
            self.setpoint_reset_threshold,
            "setpoint reset threshold",
        )?;
        validate_non_negative_or_disabled(self.setpoint_filter_tau, "setpoint filter tau")?;
        validate_non_negative(self.feed_forward.static_gain, "feed-forward static gain")?;
        validate_finite(self.safe_value, "safe value")?;
        if matches!(self.feed_forward.mode, FeedForwardMode::Reserved) {
            return Err(RonError::ConfigInvalid(
                "reserved feed-forward modes are not implemented in this sprint",
            ));
        }
        if matches!(self.anti_windup_mode, AntiWindupMode::BackCalculation)
            && (!is_finite(self.anti_windup_tracking_time)
                || self.anti_windup_tracking_time <= 0.0
                || is_near_zero(self.anti_windup_tracking_time))
        {
            return Err(RonError::ConfigInvalid(
                "back-calculation requires a positive tracking time",
            ));
        }
        if let Some(normalization) = self.normalization {
            normalization.validate()?;
        }
        Ok(())
    }

    #[must_use]
    pub(crate) fn map_input(self, value: RonFloat) -> RonFloat {
        match self.normalization {
            Some(normalization) => normalization.normalize_input(value),
            None => value,
        }
    }

    #[must_use]
    pub(crate) fn map_output_from_controller(self, value: RonFloat) -> RonFloat {
        match self.normalization {
            Some(normalization) => normalization.denormalize_output(value),
            None => value,
        }
    }

    #[must_use]
    pub(crate) fn map_output_to_controller(self, value: RonFloat) -> RonFloat {
        match self.normalization {
            Some(normalization) => normalization.normalize_output(value),
            None => value,
        }
    }
}

fn validate_non_negative(value: RonFloat, field: &'static str) -> Result<(), RonError> {
    validate_finite(value, field)?;
    if value < 0.0 {
        return Err(RonError::ConfigInvalid(field));
    }
    Ok(())
}

fn validate_non_negative_or_disabled(value: RonFloat, field: &'static str) -> Result<(), RonError> {
    validate_finite(value, field)?;
    if value < 0.0 {
        return Err(RonError::ConfigInvalid(field));
    }
    Ok(())
}

fn validate_probability(value: RonFloat, field: &'static str) -> Result<(), RonError> {
    validate_finite(value, field)?;
    if !(0.0..=1.0).contains(&value) {
        return Err(RonError::ConfigInvalid(field));
    }
    Ok(())
}

fn validate_finite(value: RonFloat, field: &'static str) -> Result<(), RonError> {
    if !is_finite(value) {
        return Err(RonError::ConfigInvalid(field));
    }
    Ok(())
}

fn validate_range(
    minimum: RonFloat,
    maximum: RonFloat,
    field: &'static str,
) -> Result<(), RonError> {
    validate_finite(minimum, field)?;
    validate_finite(maximum, field)?;
    if maximum <= minimum || is_near_zero(maximum - minimum) {
        return Err(RonError::ConfigInvalid(field));
    }
    Ok(())
}

#[must_use]
fn normalize_to_range(
    value: RonFloat,
    minimum: RonFloat,
    maximum: RonFloat,
    range: NormalizationRange,
) -> RonFloat {
    let unit_value = (value - minimum) / (maximum - minimum);
    match range {
        NormalizationRange::ZeroToOne => unit_value,
        NormalizationRange::NegativeOneToOne => (unit_value * 2.0) - 1.0,
    }
}

#[must_use]
fn denormalize_from_range(
    value: RonFloat,
    minimum: RonFloat,
    maximum: RonFloat,
    range: NormalizationRange,
) -> RonFloat {
    let unit_value = match range {
        NormalizationRange::ZeroToOne => value,
        NormalizationRange::NegativeOneToOne => value.midpoint(1.0),
    };
    (unit_value * (maximum - minimum)) + minimum
}
