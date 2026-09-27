//! # `metrics`
//!
//! Passive control-performance metrics: IAE, ISE, ITAE, peak overshoot, rise
//! time and settling time, accumulated cumulatively or over rolling windows.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-950-RON-FR-954
//! **Tests:** RON-TC-MET-001-RON-TC-MET-007
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    platform::{abs, is_finite, RonFloat},
};

/// Smallest step magnitude for which transient metrics are evaluated; below
/// it the division by the step size is never taken.
const MIN_STEP: RonFloat = 1.0e-6;
/// Fractions of the step marking the rise-time levels.
const RISE_LOW: RonFloat = 0.10;
const RISE_HIGH: RonFloat = 0.90;
/// Percent scaling for overshoot.
const PERCENT: RonFloat = 100.0;

/// Accumulation mode.
///
/// **Satisfies:** RON-FR-952
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MetricsMode {
    /// Integrals accumulate for the whole run.
    Cumulative,
    /// Integrals and transients restart every `window_steps` samples (`> 0`).
    Windowed {
        /// Window length in samples.
        window_steps: u32,
    },
}

/// Metrics configuration.
///
/// **Satisfies:** RON-FR-950, RON-FR-952, RON-FR-954
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricsConfig {
    /// Accumulation mode.
    pub mode: MetricsMode,
    /// Settling band as a fraction of the step, e.g. `0.02`; positive.
    pub band_fraction: RonFloat,
    /// Dwell time within the band that confirms settling, in seconds; `>= 0`.
    pub settle_confirm: RonFloat,
    /// Setpoint change that marks a new step; positive.
    pub step_threshold: RonFloat,
}

impl MetricsConfig {
    /// Validates the configuration.
    ///
    /// **Satisfies:** RON-FR-950, RON-FR-952
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] naming the first invalid field.
    pub fn validate(&self) -> Result<(), RonError> {
        if matches!(self.mode, MetricsMode::Windowed { window_steps: 0 }) {
            return Err(RonError::ConfigInvalid("window length"));
        }
        if !is_finite(self.band_fraction) || self.band_fraction <= 0.0 {
            return Err(RonError::ConfigInvalid("settling band"));
        }
        if !is_finite(self.settle_confirm) || self.settle_confirm < 0.0 {
            return Err(RonError::ConfigInvalid("settle confirmation time"));
        }
        if !is_finite(self.step_threshold) || self.step_threshold <= 0.0 {
            return Err(RonError::ConfigInvalid("step threshold"));
        }
        Ok(())
    }
}

/// Snapshot of the computed metrics.
///
/// **Satisfies:** RON-FR-951
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MetricsResults {
    /// Integral of `|e| dt`.
    pub iae: RonFloat,
    /// Integral of `e² dt`.
    pub ise: RonFloat,
    /// Integral of `t |e| dt`.
    pub itae: RonFloat,
    /// Peak overshoot beyond the target, in percent of the step.
    pub peak_overshoot: RonFloat,
    /// 10 % to 90 % rise time in seconds, once observed.
    pub rise_time: Option<RonFloat>,
    /// Settling time in seconds, once observed.
    pub settling_time: Option<RonFloat>,
}

/// Reference frame of the step the transient metrics are measured against.
///
/// **Satisfies:** RON-FR-954
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StepFrame {
    /// Setpoint captured at the step.
    pub target: RonFloat,
    /// Measurement captured at the step.
    pub reference: RonFloat,
    /// Signed step size, `target - reference`.
    pub size: RonFloat,
}

/// Performance-metrics accumulator. Created disabled.
///
/// **Satisfies:** RON-FR-950, RON-FR-953
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Metrics {
    config: MetricsConfig,
    results: MetricsResults,
    frame: StepFrame,
    elapsed: RonFloat,
    rise_start: Option<RonFloat>,
    in_band_time: RonFloat,
    setpoint_prev: Option<RonFloat>,
    window_counter: u32,
    enabled: bool,
}

impl Metrics {
    /// Creates a disabled accumulator.
    ///
    /// **Satisfies:** RON-FR-950, RON-FR-953
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: MetricsConfig) -> Result<Self, RonError> {
        config.validate()?;
        Ok(Self::fresh(config))
    }

    /// Enables or disables collection. While disabled, [`Metrics::step`] does
    /// no work and changes no state.
    ///
    /// **Satisfies:** RON-FR-953
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Returns `true` while collection is enabled.
    #[must_use]
    pub const fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Clears every running metric, keeping configuration and enable state.
    ///
    /// **Satisfies:** RON-FR-950
    pub fn reset(&mut self) {
        let enabled = self.enabled;
        *self = Self::fresh(self.config);
        self.enabled = enabled;
    }

    /// Accumulates one control step. A setpoint change of at least the step
    /// threshold restarts the transient metrics from that sample.
    ///
    /// **Satisfies:** RON-FR-951-RON-FR-954
    ///
    /// # Errors
    ///
    /// When enabled, returns [`RonError::InvalidArgument`] without changing
    /// state if `dt` is not positive and finite or an input is not finite.
    pub fn step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
    ) -> Result<(), RonError> {
        if !self.enabled {
            return Ok(());
        }
        if !is_finite(dt) || dt <= 0.0 {
            return Err(RonError::InvalidArgument(
                "sample period must be positive and finite",
            ));
        }
        if !is_finite(setpoint) || !is_finite(measurement) {
            return Err(RonError::InvalidArgument("metrics inputs must be finite"));
        }

        if let MetricsMode::Windowed { window_steps } = self.config.mode {
            if self.window_counter >= window_steps {
                self.restart_window();
            }
        }
        let stepped = self
            .setpoint_prev
            .is_none_or(|previous| abs(setpoint - previous) >= self.config.step_threshold);
        if stepped {
            self.frame = StepFrame {
                target: setpoint,
                reference: measurement,
                size: setpoint - measurement,
            };
            self.clear_transient();
        }
        self.setpoint_prev = Some(setpoint);

        let error = setpoint - measurement;
        let error_abs = abs(error);
        self.elapsed += dt;
        self.window_counter = self.window_counter.saturating_add(1);
        self.results.iae += error_abs * dt;
        self.results.ise += (error * error) * dt;
        self.results.itae += (self.elapsed * error_abs) * dt;

        if abs(self.frame.size) > MIN_STEP {
            self.update_rise(measurement);
            self.update_overshoot(measurement);
            self.update_settling(error_abs, dt);
        }
        Ok(())
    }

    /// Returns the current metrics.
    ///
    /// **Satisfies:** RON-FR-951
    #[must_use]
    pub const fn results(&self) -> MetricsResults {
        self.results
    }

    /// Returns the step the transient metrics are measured against.
    ///
    /// **Satisfies:** RON-FR-954
    #[must_use]
    pub const fn step_frame(&self) -> StepFrame {
        self.frame
    }

    /// Zeroed, disabled accumulator for a validated configuration.
    const fn fresh(config: MetricsConfig) -> Self {
        Self {
            config,
            results: MetricsResults {
                iae: 0.0,
                ise: 0.0,
                itae: 0.0,
                peak_overshoot: 0.0,
                rise_time: None,
                settling_time: None,
            },
            frame: StepFrame {
                target: 0.0,
                reference: 0.0,
                size: 0.0,
            },
            elapsed: 0.0,
            rise_start: None,
            in_band_time: 0.0,
            setpoint_prev: None,
            window_counter: 0,
            enabled: false,
        }
    }

    /// **Satisfies:** RON-FR-951, RON-FR-954
    fn clear_transient(&mut self) {
        self.results.peak_overshoot = 0.0;
        self.results.rise_time = None;
        self.results.settling_time = None;
        self.elapsed = 0.0;
        self.rise_start = None;
        self.in_band_time = 0.0;
    }

    /// **Satisfies:** RON-FR-952
    fn restart_window(&mut self) {
        self.results.iae = 0.0;
        self.results.ise = 0.0;
        self.results.itae = 0.0;
        self.window_counter = 0;
        self.clear_transient();
    }

    /// **Satisfies:** RON-FR-951
    fn update_rise(&mut self, measurement: RonFloat) {
        let fraction = (measurement - self.frame.reference) / self.frame.size;
        if self.rise_start.is_none() && fraction >= RISE_LOW {
            self.rise_start = Some(self.elapsed);
        }
        if let Some(start) = self.rise_start {
            if fraction >= RISE_HIGH && self.results.rise_time.is_none() {
                self.results.rise_time = Some(self.elapsed - start);
            }
        }
    }

    /// **Satisfies:** RON-FR-951
    fn update_overshoot(&mut self, measurement: RonFloat) {
        let overshoot = ((measurement - self.frame.target) / self.frame.size) * PERCENT;
        if overshoot > self.results.peak_overshoot {
            self.results.peak_overshoot = overshoot;
        }
    }

    /// **Satisfies:** RON-FR-951
    fn update_settling(&mut self, error_abs: RonFloat, dt: RonFloat) {
        let band = abs(self.frame.size) * self.config.band_fraction;
        if error_abs > band {
            self.in_band_time = 0.0;
            return;
        }
        self.in_band_time += dt;
        let confirmed = self.in_band_time + (0.5 * dt) >= self.config.settle_confirm;
        if confirmed && self.results.settling_time.is_none() {
            self.results.settling_time = Some(self.elapsed);
        }
    }
}
