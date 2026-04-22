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
    /// # Errors
    ///
    /// Returns an error when an input is invalid or when the controller faults.
    pub fn step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
    ) -> Result<(RonFloat, PidStatus), RonError> {
        match core::step(self.config, &mut self.state, setpoint, measurement, dt) {
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
