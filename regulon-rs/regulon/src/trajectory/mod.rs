//! # `trajectory`
//!
//! Setpoint trajectory generators: a trapezoidal (velocity- and
//! acceleration-limited) profile and a seven-phase jerk-limited S-curve.
//! Both evaluate incrementally, so each step costs the same regardless of
//! the length of the move.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-500-RON-FR-503, RON-FR-510-RON-FR-515
//! **Tests:** RON-TC-TRAJ-001-RON-TC-TRAJ-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

mod scurve;
mod trapezoidal;

#[cfg(test)]
mod tests;

pub use scurve::{SCurve, SCurveConfig, SCurvePhase, SCurveState, SCURVE_PHASE_COUNT};
pub use trapezoidal::{Trapezoidal, TrapezoidalConfig, TrapezoidalPhase, TrapezoidalState};

use crate::platform::{is_finite, RonFloat};

/// Position tolerance below which a move counts as complete.
const POSITION_TOLERANCE: RonFloat = 1.0e-6;

/// Trajectory fault register. Faults latch until the generator is reset.
///
/// **Satisfies:** RON-FR-512
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TrajectoryFault(u8);

impl TrajectoryFault {
    /// No fault bits set.
    pub const NONE: Self = Self(0x00);
    /// Invalid configuration, target, position or sample period.
    pub const CONFIG_INVALID: Self = Self(0x01);
    /// A computed setpoint was not finite.
    pub const OUTPUT_NOT_FINITE: Self = Self(0x02);

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

/// One trajectory sample.
///
/// **Satisfies:** RON-FR-502, RON-FR-511
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Setpoint {
    /// Position setpoint.
    pub position: RonFloat,
    /// Velocity setpoint.
    pub velocity: RonFloat,
    /// Acceleration setpoint.
    pub acceleration: RonFloat,
    /// Jerk setpoint (always zero for the trapezoidal profile).
    pub jerk: RonFloat,
    /// `true` once the target has been reached.
    pub finished: bool,
}

/// Returns `true` when every value is finite and positive.
fn all_positive_finite(values: &[RonFloat]) -> bool {
    values.iter().all(|value| is_finite(*value) && *value > 0.0)
}

/// Checks a sample period.
fn valid_dt(dt: RonFloat) -> bool {
    is_finite(dt) && dt > 0.0
}
