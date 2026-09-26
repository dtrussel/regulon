//! # `health`
//!
//! Passive control-loop health monitor. It observes the setpoint, the
//! measurement and the commanded output and latches five independent
//! conditions; it never touches the controller it watches.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-900-RON-FR-905
//! **Tests:** RON-TC-HLTH-001-RON-TC-HLTH-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use core::ops::{BitOr, BitOrAssign};

use crate::{
    error::RonError,
    platform::{abs, is_finite, RonFloat},
};

/// Length of the error-sign window used by oscillation detection.
///
/// **Satisfies:** RON-FR-901
pub const HEALTH_OSC_WINDOW: usize = 32;

/// Largest output change still treated as "not moving".
const STUCK_EPSILON: RonFloat = 1.0e-6;

/// Smallest setpoint change treated as a new setpoint step.
const STEP_EPSILON: RonFloat = 1.0e-6;

/// Sign codes stored in the oscillation window (0 marks an empty slot).
const SIGN_POSITIVE: u8 = 1;
const SIGN_NEGATIVE: u8 = 2;

/// Latched health-condition bitmask.
///
/// **Satisfies:** RON-FR-901
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HealthStatus(u8);

impl HealthStatus {
    /// Healthy: no condition active.
    pub const OK: Self = Self(0x00);
    /// The output has not moved for longer than the stuck time.
    pub const OUTPUT_STUCK: Self = Self(0x01);
    /// The error is large and still growing.
    pub const DIVERGING: Self = Self(0x02);
    /// Error sign changes in the window exceed the threshold.
    pub const OSCILLATING: Self = Self(0x04);
    /// The measurement has stayed within the dead band for too long.
    pub const SENSOR_DROPOUT: Self = Self(0x08);
    /// A steady-state error persists past the settling time.
    pub const SETPOINT_UNREACHABLE: Self = Self(0x10);

    /// Returns the raw bitfield.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when all bits of `other` are set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    /// Returns `true` when no condition is active.
    #[must_use]
    pub const fn is_ok(self) -> bool {
        self.0 == 0
    }
}

impl BitOr for HealthStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for HealthStatus {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

/// Callback invoked with a single condition bit when it first activates.
///
/// **Satisfies:** RON-FR-904
pub type HealthCallback = fn(HealthStatus);

/// Health-monitor thresholds; every condition is tuned independently.
///
/// **Satisfies:** RON-FR-902
#[derive(Clone, Copy, Debug)]
pub struct HealthConfig {
    /// Output-stuck duration threshold in seconds; positive and finite.
    pub stuck_time: RonFloat,
    /// Error magnitude above which a growing error is divergence; `>= 0`.
    pub divergence_threshold: RonFloat,
    /// Sign changes in the window that trip oscillation; below
    /// [`HEALTH_OSC_WINDOW`].
    pub oscillation_count_threshold: u8,
    /// Measurement movement below which the sensor counts as still; `>= 0`.
    pub dead_band: RonFloat,
    /// Sensor-dropout duration threshold in seconds; positive and finite.
    pub dropout_time: RonFloat,
    /// Steady-state error magnitude; `>= 0`.
    pub steady_state_threshold: RonFloat,
    /// Settling-time budget after a setpoint step in seconds; positive.
    pub settling_time: RonFloat,
    /// Optional first-activation callback.
    pub callback: Option<HealthCallback>,
}

impl HealthConfig {
    /// Validates the configuration.
    ///
    /// **Satisfies:** RON-FR-902
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] naming the first invalid field.
    pub fn validate(&self) -> Result<(), RonError> {
        let positive = |value: RonFloat| is_finite(value) && value > 0.0;
        let non_negative = |value: RonFloat| is_finite(value) && value >= 0.0;
        let checks = [
            (positive(self.stuck_time), "stuck time"),
            (
                non_negative(self.divergence_threshold),
                "divergence threshold",
            ),
            (
                usize::from(self.oscillation_count_threshold) < HEALTH_OSC_WINDOW,
                "oscillation count threshold",
            ),
            (non_negative(self.dead_band), "dead band"),
            (positive(self.dropout_time), "dropout time"),
            (
                non_negative(self.steady_state_threshold),
                "steady-state threshold",
            ),
            (positive(self.settling_time), "settling time"),
        ];
        match checks.iter().find(|(valid, _)| !valid) {
            Some((_, field)) => Err(RonError::ConfigInvalid(field)),
            None => Ok(()),
        }
    }
}

/// Control-loop health monitor.
///
/// **Satisfies:** RON-FR-900
#[derive(Clone, Copy, Debug)]
pub struct HealthMonitor {
    config: HealthConfig,
    status: HealthStatus,
    stuck_elapsed: RonFloat,
    dropout_elapsed: RonFloat,
    since_setpoint_step: RonFloat,
    sign_window: [u8; HEALTH_OSC_WINDOW],
    sign_index: usize,
    error_prev: RonFloat,
    measurement_prev: RonFloat,
    output_prev: RonFloat,
    previous_valid: bool,
}

impl HealthMonitor {
    /// Creates a healthy monitor.
    ///
    /// **Satisfies:** RON-FR-900, RON-FR-902
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: HealthConfig) -> Result<Self, RonError> {
        config.validate()?;
        Ok(Self {
            config,
            status: HealthStatus::OK,
            stuck_elapsed: 0.0,
            dropout_elapsed: 0.0,
            since_setpoint_step: 0.0,
            sign_window: [0; HEALTH_OSC_WINDOW],
            sign_index: 0,
            error_prev: 0.0,
            measurement_prev: 0.0,
            output_prev: 0.0,
            previous_valid: false,
        })
    }

    /// Evaluates one control step and returns the conditions that became
    /// active on this step (each also reported once through the callback).
    ///
    /// The monitor only reads its arguments; it cannot affect the controller.
    ///
    /// **Satisfies:** RON-FR-900, RON-FR-901, RON-FR-903-RON-FR-905
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] without changing the monitor when
    /// `dt` is not positive and finite or an input is not finite.
    pub fn step(
        &mut self,
        setpoint: RonFloat,
        measurement: RonFloat,
        output: RonFloat,
        dt: RonFloat,
    ) -> Result<HealthStatus, RonError> {
        if !is_finite(dt) || dt <= 0.0 {
            return Err(RonError::InvalidArgument(
                "sample period must be positive and finite",
            ));
        }
        if !is_finite(setpoint) || !is_finite(measurement) || !is_finite(output) {
            return Err(RonError::InvalidArgument("health inputs must be finite"));
        }
        let error = setpoint - measurement;
        let before = self.status;

        let stuck = self.output_stuck(output, dt);
        self.latch(HealthStatus::OUTPUT_STUCK, stuck);
        let diverging = self.diverging(error);
        self.latch(HealthStatus::DIVERGING, diverging);
        let oscillating = self.oscillating(error);
        self.latch(HealthStatus::OSCILLATING, oscillating);
        let dropout = self.sensor_dropout(measurement, dt);
        self.latch(HealthStatus::SENSOR_DROPOUT, dropout);
        let unreachable = self.setpoint_unreachable(setpoint, error, dt);
        self.latch(HealthStatus::SETPOINT_UNREACHABLE, unreachable);

        self.error_prev = error;
        self.measurement_prev = measurement;
        self.output_prev = output;
        self.previous_valid = true;
        Ok(HealthStatus(self.status.bits() & !before.bits()))
    }

    /// Returns the latched status.
    ///
    /// **Satisfies:** RON-FR-901, RON-FR-905
    #[must_use]
    pub const fn status(&self) -> HealthStatus {
        self.status
    }

    /// Clears every latched condition and resets the detectors, keeping the
    /// configuration.
    ///
    /// **Satisfies:** RON-FR-905
    pub fn clear(&mut self) {
        *self = Self {
            config: self.config,
            status: HealthStatus::OK,
            stuck_elapsed: 0.0,
            dropout_elapsed: 0.0,
            since_setpoint_step: 0.0,
            sign_window: [0; HEALTH_OSC_WINDOW],
            sign_index: 0,
            error_prev: 0.0,
            measurement_prev: 0.0,
            output_prev: 0.0,
            previous_valid: false,
        };
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> HealthConfig {
        self.config
    }

    /// Latches `bit` and fires the callback on its first activation.
    ///
    /// **Satisfies:** RON-FR-904, RON-FR-905
    fn latch(&mut self, bit: HealthStatus, active: bool) {
        if active && !self.status.contains(bit) {
            self.status |= bit;
            if let Some(callback) = self.config.callback {
                callback(bit);
            }
        }
    }

    /// **Satisfies:** RON-FR-901
    fn output_stuck(&mut self, output: RonFloat, dt: RonFloat) -> bool {
        let moved = self.previous_valid && abs(output - self.output_prev) > STUCK_EPSILON;
        self.stuck_elapsed = if moved { 0.0 } else { self.stuck_elapsed + dt };
        duration_reached(self.stuck_elapsed, dt, self.config.stuck_time)
    }

    /// **Satisfies:** RON-FR-901
    fn diverging(&self, error: RonFloat) -> bool {
        let growing = error * (error - self.error_prev) > 0.0;
        abs(error) > self.config.divergence_threshold && growing
    }

    /// **Satisfies:** RON-FR-901
    fn oscillating(&mut self, error: RonFloat) -> bool {
        let sign = if error >= 0.0 {
            SIGN_POSITIVE
        } else {
            SIGN_NEGATIVE
        };
        if let Some(slot) = self.sign_window.get_mut(self.sign_index) {
            *slot = sign;
        }
        self.sign_index = (self.sign_index + 1) % HEALTH_OSC_WINDOW;
        self.sign_changes() > usize::from(self.config.oscillation_count_threshold)
    }

    /// Counts sign changes across the window, oldest to newest. Empty slots
    /// form a prefix of that walk, so only the older slot needs checking.
    fn sign_changes(&self) -> usize {
        (0..HEALTH_OSC_WINDOW - 1)
            .filter(|offset| {
                let older = self.sign_at(self.sign_index + offset);
                older != 0 && older != self.sign_at(self.sign_index + offset + 1)
            })
            .count()
    }

    fn sign_at(&self, position: usize) -> u8 {
        self.sign_window
            .get(position % HEALTH_OSC_WINDOW)
            .copied()
            .unwrap_or(0)
    }

    /// **Satisfies:** RON-FR-901
    fn sensor_dropout(&mut self, measurement: RonFloat, dt: RonFloat) -> bool {
        let moved = self.previous_valid
            && abs(measurement - self.measurement_prev) >= self.config.dead_band;
        self.dropout_elapsed = if moved {
            0.0
        } else {
            self.dropout_elapsed + dt
        };
        duration_reached(self.dropout_elapsed, dt, self.config.dropout_time)
    }

    /// **Satisfies:** RON-FR-901
    fn setpoint_unreachable(&mut self, setpoint: RonFloat, error: RonFloat, dt: RonFloat) -> bool {
        let setpoint_prev = self.error_prev + self.measurement_prev;
        let stepped = self.previous_valid && abs(setpoint - setpoint_prev) > STEP_EPSILON;
        self.since_setpoint_step = if stepped {
            0.0
        } else {
            self.since_setpoint_step + dt
        };
        abs(error) > self.config.steady_state_threshold
            && duration_reached(self.since_setpoint_step, dt, self.config.settling_time)
    }
}

/// Compares an accumulated duration with a threshold, rounded to the nearest
/// sample so the boundary lands on the expected step despite accumulation
/// error.
fn duration_reached(elapsed: RonFloat, dt: RonFloat, threshold: RonFloat) -> bool {
    elapsed + (0.5 * dt) >= threshold
}
