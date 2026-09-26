//! # `pid`
//!
//! Idiomatic Rust PID controller API.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-PR-001-RON-PR-022,
//! RON-SR-001-RON-SR-033, RON-QR-001-RON-QR-031
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

mod config;
mod core;
mod types;

#[cfg(test)]
mod tests;

#[cfg(test)]
mod feed_forward_tests;

#[cfg(kani)]
mod proofs;

pub use types::{
    AntiWindupMode, DerivativeMode, FeedForwardConfig, FeedForwardMode, IntegrationMethod,
    NormalizationConfig, NormalizationRange, PidConfig, PidFault, PidMode, PidSnapshot, PidStatus,
    SafePolicy,
};

use crate::{error::RonError, platform::clamp, RonFloat};

use self::types::PidRuntime;

/// PID controller instance with fully encapsulated state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid {
    config: PidConfig,
    state: PidRuntime,
}

impl Pid {
    /// Creates and validates a new PID controller.
    ///
    /// # Errors
    ///
    /// Returns an error when the supplied configuration violates the PID
    /// contract defined by the specifications.
    pub fn new(config: PidConfig) -> Result<Self, RonError> {
        config.validate()?;
        let initial_output = clamp(0.0, config.output_min, config.output_max);
        Ok(Self {
            config,
            state: PidRuntime {
                mode: config.initial_mode,
                output_prev: initial_output,
                ..PidRuntime::default()
            },
        })
    }

    /// Returns the active configuration.
    #[must_use]
    pub const fn configuration(&self) -> PidConfig {
        self.config
    }

    /// Atomically replaces the active configuration after validation.
    ///
    /// # Errors
    ///
    /// Returns an error when the replacement configuration is invalid.
    pub fn update_configuration(&mut self, config: PidConfig) -> Result<(), RonError> {
        config.validate()?;
        self.config = config;
        self.state.output_prev =
            clamp(self.state.output_prev, config.output_min, config.output_max);
        self.state.integral = clamp(
            self.state.integral,
            config.integral_min,
            config.integral_max,
        );
        Ok(())
    }

    /// Resets the dynamic state without changing the configuration or mode.
    pub fn reset(&mut self) {
        let mode = self.state.mode;
        self.state = PidRuntime {
            mode,
            output_prev: clamp(0.0, self.config.output_min, self.config.output_max),
            ..PidRuntime::default()
        };
    }

    /// Executes one PID control step.
    ///
    /// While a fault is latched the step returns it without running, and
    /// [`Pid::output`] gives the safe-state output to apply.
    ///
    /// # Errors
    ///
    /// Returns [`RonError::Fault`] with the latched bits when a fault is
    /// latched or this step latches one (a non-finite setpoint or
    /// measurement, a non-finite output, integral overflow), and
    /// [`RonError::InvalidArgument`] without latching when `dt` is not
    /// positive and finite.
    pub fn step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
    ) -> Result<(RonFloat, PidStatus), RonError> {
        self.run_step(setpoint, measurement, dt, None)
    }

    /// Executes one PID control step with a caller-supplied feed-forward term.
    ///
    /// Only valid when the configured feed-forward mode is
    /// [`FeedForwardMode::External`]; `external_feed_forward` is added to the
    /// PID sum ahead of saturation and rate limiting.
    ///
    /// **Satisfies:** RON-FR-200, RON-FR-201, RON-FR-203
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when the external mode is not
    /// configured, [`RonError::InvalidArgument`] when `external_feed_forward`
    /// is non-finite, and otherwise the same errors as [`Pid::step`].
    pub fn step_with_feed_forward(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
        external_feed_forward: RonFloat,
    ) -> Result<(RonFloat, PidStatus), RonError> {
        self.run_step(setpoint, measurement, dt, Some(external_feed_forward))
    }

    /// Replaces the feed-forward configuration and clears the feed-forward
    /// filter state. A rejected configuration leaves the controller unchanged.
    ///
    /// **Satisfies:** RON-FR-201, RON-FR-202, RON-FR-204
    ///
    /// # Errors
    ///
    /// Returns an error when the feed-forward configuration is invalid.
    pub fn set_feed_forward(&mut self, feed_forward: FeedForwardConfig) -> Result<(), RonError> {
        feed_forward.validate()?;
        self.config.feed_forward = feed_forward;
        self.state.feed_forward_prev = 0.0;
        self.state.ff_setpoint_prev = 0.0;
        self.state.ff_velocity_prev = 0.0;
        self.state.ff_acceleration_prev = 0.0;
        self.state.status &= !PidStatus::FEED_FORWARD_ACTIVE;
        Ok(())
    }

    /// Returns the feed-forward contribution applied by the last step.
    ///
    /// **Satisfies:** RON-FR-205
    #[must_use]
    pub const fn last_feed_forward(&self) -> RonFloat {
        self.state.feed_forward_prev
    }

    fn run_step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
        external_feed_forward: Option<RonFloat>,
    ) -> Result<(RonFloat, PidStatus), RonError> {
        match core::step(
            self.config,
            &mut self.state,
            setpoint,
            measurement,
            dt,
            external_feed_forward,
        ) {
            Ok(result) => Ok(result),
            Err(RonError::Fault(fault)) => {
                self.state.fault |= fault;
                self.state.status |= PidStatus::FAULT;
                Err(RonError::Fault(self.state.fault))
            }
            Err(error) => Err(error),
        }
    }

    /// Updates PID gains atomically.
    ///
    /// # Errors
    ///
    /// Returns an error when the new gain set is invalid.
    pub fn set_gains(&mut self, kp: RonFloat, ki: RonFloat, kd: RonFloat) -> Result<(), RonError> {
        let mut config = self.config;
        config.kp = kp;
        config.ki = ki;
        config.kd = kd;
        self.update_configuration(config)
    }

    /// Updates output limits atomically.
    ///
    /// # Errors
    ///
    /// Returns an error when the new limit range is invalid.
    pub fn set_limits(
        &mut self,
        output_min: RonFloat,
        output_max: RonFloat,
    ) -> Result<(), RonError> {
        let mut config = self.config;
        config.output_min = output_min;
        config.output_max = output_max;
        self.update_configuration(config)
    }

    /// Updates the derivative filter coefficient atomically.
    ///
    /// # Errors
    ///
    /// Returns an error when the new filter coefficient is invalid.
    pub fn set_filter(&mut self, derivative_filter: RonFloat) -> Result<(), RonError> {
        let mut config = self.config;
        config.derivative_filter = derivative_filter;
        self.update_configuration(config)
    }

    /// Updates anti-windup settings atomically.
    ///
    /// # Errors
    ///
    /// Returns an error when the anti-windup selection is invalid.
    pub fn set_anti_windup(
        &mut self,
        anti_windup_mode: AntiWindupMode,
        anti_windup_tracking_time: RonFloat,
    ) -> Result<(), RonError> {
        let mut config = self.config;
        config.anti_windup_mode = anti_windup_mode;
        config.anti_windup_tracking_time = anti_windup_tracking_time;
        self.update_configuration(config)
    }

    /// Updates the operating mode and manual output tracking state.
    ///
    /// # Errors
    ///
    /// Returns an error when the supplied manual output is non-finite.
    pub fn set_mode(&mut self, mode: PidMode, manual_output: RonFloat) -> Result<(), RonError> {
        if !manual_output.is_finite() {
            return Err(RonError::InvalidArgument("manual output must be finite"));
        }
        let clamped_output = clamp(
            manual_output,
            self.config.output_min,
            self.config.output_max,
        );
        match (self.state.mode, mode) {
            (PidMode::Manual, PidMode::Automatic) => {
                self.state.integral = clamp(
                    clamped_output,
                    self.config.integral_min,
                    self.config.integral_max,
                );
                self.state.output_prev = clamped_output;
                self.state.output_unbounded_prev = clamped_output;
            }
            (PidMode::Automatic | PidMode::Manual, PidMode::Manual) => {
                self.state.output_prev = clamped_output;
            }
            (PidMode::Automatic, PidMode::Automatic) => {}
        }
        self.state.mode = mode;
        Ok(())
    }

    /// Preloads the integral accumulator for warm starts.
    ///
    /// # Errors
    ///
    /// Returns an error when the supplied preload is non-finite.
    pub fn set_integral(&mut self, integral: RonFloat) -> Result<(), RonError> {
        if !integral.is_finite() {
            return Err(RonError::InvalidArgument("integral preload must be finite"));
        }
        self.state.integral = clamp(integral, self.config.integral_min, self.config.integral_max);
        Ok(())
    }

    /// Clears the latched fault register.
    pub fn clear_fault(&mut self) {
        core::clear_fault(&mut self.state);
    }

    /// Returns a read-only snapshot of the current internal state.
    #[must_use]
    pub fn state(&self) -> PidSnapshot {
        self.state.snapshot()
    }

    /// Returns the current integral accumulator.
    #[must_use]
    pub const fn integral(&self) -> RonFloat {
        self.state.integral
    }

    /// Returns the last control output.
    #[must_use]
    pub const fn last_output(&self) -> RonFloat {
        self.state.output_prev
    }

    /// Returns the output to apply: the last output, or while a fault is
    /// latched the safe-state output selected by `safe_policy` (clamped to the
    /// output limits). The output history itself is never overwritten.
    ///
    /// **Satisfies:** RON-SR-011
    #[must_use]
    pub fn output(&self) -> RonFloat {
        if self.state.fault.is_none() {
            self.state.output_prev
        } else {
            core::safe_state_output(self.config, self.state.output_prev)
        }
    }

    /// Returns the last filtered derivative value.
    #[must_use]
    pub const fn last_derivative(&self) -> RonFloat {
        self.state.derivative_filtered_prev
    }

    /// Returns the current mode.
    #[must_use]
    pub const fn mode(&self) -> PidMode {
        self.state.mode
    }
}
