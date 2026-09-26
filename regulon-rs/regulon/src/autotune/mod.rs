//! # `autotune`
//!
//! Relay-feedback (Åström-Hägglund) auto-tuner. A relay drives the loop into a
//! limit cycle; zero-crossing timing gives the ultimate period `Tu` and the
//! peak-to-peak excursion the ultimate gain `Ku = 4d / (pi * A)`. A tuning rule
//! turns them into PID gains, which reach the controller only through an
//! explicit [`Autotuner::apply`].
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-800-RON-FR-807
//! **Tests:** RON-TC-AT-001-RON-TC-AT-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

#[cfg(kani)]
mod proofs;

use crate::{
    error::RonError,
    pid::{Pid, PidMode},
    platform::{is_finite, RonFloat},
};

#[cfg(feature = "double_precision")]
const PI: RonFloat = core::f64::consts::PI;

#[cfg(not(feature = "double_precision"))]
const PI: RonFloat = core::f32::consts::PI;

/// Zero crossings per full oscillation cycle.
const HALF_PERIODS_PER_CYCLE: u16 = 2;

/// Smallest half-amplitude treated as a real oscillation.
const MIN_AMPLITUDE: RonFloat = 1.0e-6;

/// Tuning rule applied to the measured `Ku` and `Tu`.
///
/// **Satisfies:** RON-FR-803
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TuningRule {
    /// Ziegler-Nichols (classic).
    #[default]
    ZieglerNichols,
    /// Tyreus-Luyben (robust, slow).
    TyreusLuyben,
    /// Some overshoot.
    SomeOvershoot,
    /// No overshoot (conservative).
    NoOvershoot,
}

impl TuningRule {
    /// Returns `(Kp/Ku, Ti/Tu, Td/Tu)` for the rule.
    ///
    /// **Satisfies:** RON-FR-803
    #[must_use]
    pub const fn factors(self) -> (RonFloat, RonFloat, RonFloat) {
        match self {
            Self::ZieglerNichols => (0.60, 0.50, 0.125),
            Self::TyreusLuyben => (0.45, 2.20, 0.158),
            Self::SomeOvershoot => (0.33, 0.50, 0.333),
            Self::NoOvershoot => (0.20, 0.50, 0.333),
        }
    }
}

/// Auto-tune lifecycle phase.
///
/// **Satisfies:** RON-FR-800
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AutotunePhase {
    /// Created, not yet started.
    #[default]
    Idle,
    /// Relay driving; waiting for the first crossing.
    Settling,
    /// Oscillating; timing half-periods.
    Relay,
    /// Estimation complete; results valid.
    Done,
    /// Aborted by the caller, a timeout or insufficient excitation.
    Aborted,
}

/// Relay auto-tuner configuration.
///
/// **Satisfies:** RON-FR-801
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AutotuneConfig {
    /// Relay half-amplitude `d`; positive and finite.
    pub relay_amplitude: RonFloat,
    /// Switching hysteresis band; `>= 0` and finite.
    pub hysteresis: RonFloat,
    /// Output bias the relay swings about; finite.
    pub bias: RonFloat,
    /// Full cycles to observe before estimating; at least 1.
    pub min_cycles: u8,
    /// Abort when the run exceeds this many seconds; positive and finite.
    pub timeout: RonFloat,
    /// Rule applied to `Ku` and `Tu`.
    pub rule: TuningRule,
}

impl AutotuneConfig {
    /// Validates the configuration.
    ///
    /// **Satisfies:** RON-FR-801
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] naming the first invalid field.
    pub fn validate(&self) -> Result<(), RonError> {
        if !is_finite(self.relay_amplitude) || self.relay_amplitude <= 0.0 {
            return Err(RonError::ConfigInvalid("relay amplitude"));
        }
        if !is_finite(self.hysteresis) || self.hysteresis < 0.0 {
            return Err(RonError::ConfigInvalid("hysteresis"));
        }
        if !is_finite(self.bias) {
            return Err(RonError::ConfigInvalid("bias"));
        }
        if self.min_cycles == 0 {
            return Err(RonError::ConfigInvalid("minimum cycles"));
        }
        if !is_finite(self.timeout) || self.timeout <= 0.0 {
            return Err(RonError::ConfigInvalid("timeout"));
        }
        Ok(())
    }
}

/// Measured ultimate gain and period and the derived gains.
///
/// **Satisfies:** RON-FR-802, RON-FR-803, RON-FR-805
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutotuneResults {
    /// Ultimate gain `Ku`.
    pub ultimate_gain: RonFloat,
    /// Ultimate period `Tu` in seconds.
    pub ultimate_period: RonFloat,
    /// Proportional gain.
    pub kp: RonFloat,
    /// Integral gain.
    pub ki: RonFloat,
    /// Derivative gain.
    pub kd: RonFloat,
}

/// Controller context captured at start, restored on abort.
#[derive(Clone, Copy, Debug, PartialEq)]
struct SavedPid {
    kp: RonFloat,
    ki: RonFloat,
    kd: RonFloat,
    mode: PidMode,
}

/// Relay-feedback auto-tuner.
///
/// **Satisfies:** RON-FR-800
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Autotuner {
    config: AutotuneConfig,
    phase: AutotunePhase,
    results: Option<AutotuneResults>,
    saved: Option<SavedPid>,
    relay_prev: RonFloat,
    elapsed: RonFloat,
    since_crossing: RonFloat,
    half_period_sum: RonFloat,
    half_period_count: u16,
    measurement_min: RonFloat,
    measurement_max: RonFloat,
    error_positive: Option<bool>,
}

impl Autotuner {
    /// Creates an idle auto-tuner.
    ///
    /// **Satisfies:** RON-FR-800, RON-FR-801
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: AutotuneConfig) -> Result<Self, RonError> {
        config.validate()?;
        Ok(Self::idle(config))
    }

    /// Starts a run: clears any previous run, captures the controller's gains
    /// and mode for restore and parks it in manual at the bias so it does not
    /// fight the relay. The gains are not modified.
    ///
    /// **Satisfies:** RON-FR-800, RON-FR-804
    ///
    /// # Errors
    ///
    /// Returns an error when the controller rejects the manual handoff.
    pub fn start(&mut self, pid: &mut Pid) -> Result<(), RonError> {
        *self = Self::idle(self.config);
        let config = pid.configuration();
        self.saved = Some(SavedPid {
            kp: config.kp,
            ki: config.ki,
            kd: config.kd,
            mode: pid.mode(),
        });
        self.relay_prev = self.config.bias + self.config.relay_amplitude;
        self.phase = AutotunePhase::Settling;
        pid.set_mode(PidMode::Manual, self.config.bias)
    }

    /// Executes one relay step and returns the relay output, which always lies
    /// in `bias ± relay_amplitude`. After the run ends it returns the bias.
    ///
    /// **Satisfies:** RON-FR-800, RON-FR-802, RON-FR-806
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `dt` is not positive and
    /// finite or an input is not finite, and [`RonError::ConfigInvalid`] when
    /// the run has not been started.
    pub fn step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        dt: RonFloat,
    ) -> Result<RonFloat, RonError> {
        if !is_finite(dt) || dt <= 0.0 {
            return Err(RonError::InvalidArgument(
                "sample period must be positive and finite",
            ));
        }
        if !is_finite(setpoint) || !is_finite(measurement) {
            return Err(RonError::InvalidArgument("auto-tune inputs must be finite"));
        }
        match self.phase {
            AutotunePhase::Done | AutotunePhase::Aborted => return Ok(self.config.bias),
            AutotunePhase::Idle => {
                return Err(RonError::ConfigInvalid("auto-tune run not started"));
            }
            AutotunePhase::Settling | AutotunePhase::Relay => {}
        }

        let error = setpoint - measurement;
        let output = self.relay_output(error);
        let crossed = self.detect_crossing(error);
        self.elapsed += dt;
        if self.phase == AutotunePhase::Settling {
            if crossed {
                self.phase = AutotunePhase::Relay;
                self.since_crossing = 0.0;
                self.measurement_min = measurement;
                self.measurement_max = measurement;
            }
        } else {
            self.track_oscillation(measurement, dt, crossed);
        }
        if matches!(self.phase, AutotunePhase::Settling | AutotunePhase::Relay)
            && self.elapsed > self.config.timeout
        {
            self.phase = AutotunePhase::Aborted;
        }
        Ok(output)
    }

    /// Applies the tuned gains and restores the mode captured at start. This
    /// is the only path that changes the controller's gains.
    ///
    /// **Satisfies:** RON-FR-804
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] unless the run is done, or the
    /// controller's error when it rejects the gains.
    pub fn apply(&self, pid: &mut Pid) -> Result<(), RonError> {
        let (Some(results), Some(saved)) = (self.results, self.saved) else {
            return Err(RonError::ConfigInvalid("auto-tune results not available"));
        };
        pid.set_gains(results.kp, results.ki, results.kd)?;
        pid.set_mode(saved.mode, self.config.bias)
    }

    /// Aborts the run and restores the gains and mode captured at start. A run
    /// that was never started leaves the controller untouched.
    ///
    /// **Satisfies:** RON-FR-807
    ///
    /// # Errors
    ///
    /// Returns the controller's error if it rejects the restored context.
    pub fn abort(&mut self, pid: &mut Pid) -> Result<(), RonError> {
        self.phase = AutotunePhase::Aborted;
        self.results = None;
        match self.saved {
            Some(saved) => {
                pid.set_gains(saved.kp, saved.ki, saved.kd)?;
                pid.set_mode(saved.mode, self.config.bias)
            }
            None => Ok(()),
        }
    }

    /// Returns the measured `Ku`, `Tu` and tuned gains once the run is done.
    ///
    /// **Satisfies:** RON-FR-805
    #[must_use]
    pub const fn results(&self) -> Option<AutotuneResults> {
        self.results
    }

    /// Returns the lifecycle phase.
    #[must_use]
    pub const fn phase(&self) -> AutotunePhase {
        self.phase
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> AutotuneConfig {
        self.config
    }

    const fn idle(config: AutotuneConfig) -> Self {
        Self {
            config,
            phase: AutotunePhase::Idle,
            results: None,
            saved: None,
            relay_prev: 0.0,
            elapsed: 0.0,
            since_crossing: 0.0,
            half_period_sum: 0.0,
            half_period_count: 0,
            measurement_min: 0.0,
            measurement_max: 0.0,
            error_positive: None,
        }
    }

    /// Relay law with hysteresis hold; always within `bias ± d`.
    ///
    /// **Satisfies:** RON-FR-800, RON-FR-806
    fn relay_output(&mut self, error: RonFloat) -> RonFloat {
        let config = self.config;
        let output = if error > config.hysteresis {
            config.bias + config.relay_amplitude
        } else if error < -config.hysteresis {
            config.bias - config.relay_amplitude
        } else {
            self.relay_prev
        };
        self.relay_prev = output;
        output
    }

    /// Reports whether the error changed sign; the first sample only seeds it.
    ///
    /// **Satisfies:** RON-FR-802
    fn detect_crossing(&mut self, error: RonFloat) -> bool {
        let positive = error >= 0.0;
        let crossed = self
            .error_positive
            .is_some_and(|previous| previous != positive);
        self.error_positive = Some(positive);
        crossed
    }

    /// **Satisfies:** RON-FR-802
    fn track_oscillation(&mut self, measurement: RonFloat, dt: RonFloat, crossed: bool) {
        self.since_crossing += dt;
        self.measurement_min = self.measurement_min.min(measurement);
        self.measurement_max = self.measurement_max.max(measurement);
        if crossed {
            self.half_period_sum += self.since_crossing;
            self.half_period_count = self.half_period_count.saturating_add(1);
            self.since_crossing = 0.0;
        }
        let needed = HALF_PERIODS_PER_CYCLE * u16::from(self.config.min_cycles);
        if self.half_period_count >= needed {
            self.estimate();
        }
    }

    /// Estimates `Ku` and `Tu` and derives the gains; an oscillation too
    /// small to measure aborts the run.
    ///
    /// **Satisfies:** RON-FR-802, RON-FR-803
    fn estimate(&mut self) {
        let amplitude = (self.measurement_max - self.measurement_min) * 0.5;
        if amplitude < MIN_AMPLITUDE {
            self.phase = AutotunePhase::Aborted;
            return;
        }
        let ultimate_period = 2.0 * (self.half_period_sum / RonFloat::from(self.half_period_count));
        let ultimate_gain = (4.0 * self.config.relay_amplitude) / (PI * amplitude);
        let (gain_ratio, integral_ratio, derivative_ratio) = self.config.rule.factors();
        let kp = gain_ratio * ultimate_gain;
        self.results = Some(AutotuneResults {
            ultimate_gain,
            ultimate_period,
            kp,
            ki: kp / (integral_ratio * ultimate_period),
            kd: kp * (derivative_ratio * ultimate_period),
        });
        self.phase = AutotunePhase::Done;
    }
}
