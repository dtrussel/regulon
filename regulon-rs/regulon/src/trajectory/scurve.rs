//! # `trajectory::scurve`
//!
//! Seven-phase jerk-limited S-curve profile. Acceleration ramps rather than
//! stepping, which excites far less mechanical resonance than a trapezoid.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-510-RON-FR-515
//! **Tests:** RON-TC-TRAJ-005-RON-TC-TRAJ-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{all_positive_finite, valid_dt, Setpoint, TrajectoryFault, POSITION_TOLERANCE};
use crate::platform::{abs, is_finite, sign_nonzero, sqrt, RonFloat};

/// Number of timed phases in an S-curve move.
pub const SCURVE_PHASE_COUNT: usize = 7;

/// Newton iterations for the cube root used to plan short moves.
const CBRT_STEPS: u8 = 18;

/// S-curve profile limits.
///
/// **Satisfies:** RON-FR-510
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SCurveConfig {
    /// Maximum speed; positive and finite.
    pub v_max: RonFloat,
    /// Maximum acceleration; positive and finite.
    pub a_max: RonFloat,
    /// Maximum jerk; positive and finite.
    pub j_max: RonFloat,
}

/// S-curve phase, in execution order.
///
/// **Satisfies:** RON-FR-510, RON-FR-512, RON-FR-513
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SCurvePhase {
    /// Positive jerk, building acceleration.
    JerkPositive1,
    /// Constant acceleration.
    AccelerationHold,
    /// Negative jerk, releasing acceleration.
    JerkNegative1,
    /// Constant velocity.
    ConstantVelocity,
    /// Negative jerk, building deceleration.
    JerkNegative2,
    /// Constant deceleration.
    DecelerationHold,
    /// Positive jerk, releasing deceleration.
    JerkPositive2,
    /// At rest on the target.
    #[default]
    Done,
}

impl SCurvePhase {
    /// Index into [`SCurveState::phase_times`]; `None` for `Done`.
    #[must_use]
    pub const fn index(self) -> Option<usize> {
        match self {
            Self::JerkPositive1 => Some(0),
            Self::AccelerationHold => Some(1),
            Self::JerkNegative1 => Some(2),
            Self::ConstantVelocity => Some(3),
            Self::JerkNegative2 => Some(4),
            Self::DecelerationHold => Some(5),
            Self::JerkPositive2 => Some(6),
            Self::Done => None,
        }
    }

    const fn next(self) -> Self {
        match self {
            Self::JerkPositive1 => Self::AccelerationHold,
            Self::AccelerationHold => Self::JerkNegative1,
            Self::JerkNegative1 => Self::ConstantVelocity,
            Self::ConstantVelocity => Self::JerkNegative2,
            Self::JerkNegative2 => Self::DecelerationHold,
            Self::DecelerationHold => Self::JerkPositive2,
            Self::JerkPositive2 | Self::Done => Self::Done,
        }
    }
}

/// Read-only S-curve generator state.
///
/// **Satisfies:** RON-FR-515
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SCurveState {
    /// Current position.
    pub position: RonFloat,
    /// Current velocity.
    pub velocity: RonFloat,
    /// Current acceleration.
    pub acceleration: RonFloat,
    /// Jerk applied over the last step.
    pub jerk: RonFloat,
    /// Target position.
    pub target: RonFloat,
    /// Direction of the move, `1` or `-1`.
    pub direction: RonFloat,
    /// Planned duration of each phase, indexed by [`SCurvePhase::index`].
    pub phase_times: [RonFloat; SCURVE_PHASE_COUNT],
    /// Time spent in the current phase.
    pub phase_elapsed: RonFloat,
    /// Planned duration of the whole move.
    pub total_time: RonFloat,
    /// Time elapsed since the move started.
    pub elapsed: RonFloat,
    /// Current phase.
    pub phase: SCurvePhase,
    /// Latched fault register.
    pub fault: TrajectoryFault,
    /// `true` while execution is paused.
    pub hold: bool,
    /// `true` once the target has been reached.
    pub finished: bool,
}

impl SCurveState {
    fn phase_time(&self, phase: SCurvePhase) -> RonFloat {
        match phase.index().and_then(|index| self.phase_times.get(index)) {
            Some(time) => *time,
            None => 0.0,
        }
    }
}

/// S-curve trajectory generator.
///
/// **Satisfies:** RON-FR-510
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SCurve {
    config: SCurveConfig,
    state: SCurveState,
}

impl SCurve {
    /// Creates a generator at rest at `position` with no move planned.
    ///
    /// **Satisfies:** RON-FR-510, RON-FR-512
    ///
    /// # Errors
    ///
    /// Returns [`TrajectoryFault::CONFIG_INVALID`] when a limit is not positive
    /// and finite, or `position` is not finite.
    pub fn new(config: SCurveConfig, position: RonFloat) -> Result<Self, TrajectoryFault> {
        if !all_positive_finite(&[config.v_max, config.a_max, config.j_max]) || !is_finite(position)
        {
            return Err(TrajectoryFault::CONFIG_INVALID);
        }
        Ok(Self {
            config,
            state: seed(position),
        })
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> SCurveConfig {
        self.config
    }

    /// Sets a new target and plans a move to it from the current position.
    ///
    /// **Satisfies:** RON-FR-510
    ///
    /// # Errors
    ///
    /// Returns the latched fault, or latches and returns
    /// [`TrajectoryFault::CONFIG_INVALID`] when `target` is not finite.
    pub fn set_target(&mut self, target: RonFloat) -> Result<(), TrajectoryFault> {
        self.check_fault()?;
        if !is_finite(target) {
            return Err(self.latch(TrajectoryFault::CONFIG_INVALID));
        }
        self.state.target = target;
        self.plan();
        Ok(())
    }

    /// Advances the profile by `dt` seconds and returns the new setpoint.
    ///
    /// **Satisfies:** RON-FR-510-RON-FR-512
    ///
    /// # Errors
    ///
    /// Latches and returns [`TrajectoryFault::CONFIG_INVALID`] when `dt` is not
    /// positive and finite, or [`TrajectoryFault::OUTPUT_NOT_FINITE`] when a
    /// setpoint is not finite; returns an already latched fault unchanged.
    pub fn step(&mut self, dt: RonFloat) -> Result<Setpoint, TrajectoryFault> {
        if !valid_dt(dt) {
            return Err(self.latch(TrajectoryFault::CONFIG_INVALID));
        }
        self.check_fault()?;
        if self.state.hold || self.state.finished {
            return Ok(self.setpoint());
        }
        self.integrate(dt);
        if self.state.elapsed >= self.state.total_time - POSITION_TOLERANCE
            || self.state.phase == SCurvePhase::Done
        {
            self.finish();
        }
        let setpoint = self.setpoint();
        if !is_finite(setpoint.position)
            || !is_finite(setpoint.velocity)
            || !is_finite(setpoint.acceleration)
        {
            return Err(self.latch(TrajectoryFault::OUTPUT_NOT_FINITE));
        }
        Ok(setpoint)
    }

    /// Pauses (`true`) or resumes (`false`) the move. While held, `step`
    /// returns the current setpoint unchanged.
    ///
    /// **Satisfies:** RON-FR-513
    ///
    /// # Errors
    ///
    /// Returns the latched fault, if any.
    pub fn set_hold(&mut self, hold: bool) -> Result<(), TrajectoryFault> {
        self.check_fault()?;
        self.state.hold = hold;
        Ok(())
    }

    /// Re-seeds the generator at rest at `position`, clearing the move, hold
    /// and any latched fault, and keeping the configuration.
    ///
    /// **Satisfies:** RON-FR-514
    ///
    /// # Errors
    ///
    /// Returns [`TrajectoryFault::CONFIG_INVALID`] without changing the state
    /// when `position` is not finite.
    pub fn reset(&mut self, position: RonFloat) -> Result<(), TrajectoryFault> {
        if !is_finite(position) {
            return Err(TrajectoryFault::CONFIG_INVALID);
        }
        self.state = seed(position);
        Ok(())
    }

    /// Returns a copy of the complete internal state.
    ///
    /// **Satisfies:** RON-FR-515
    #[must_use]
    pub const fn state(&self) -> SCurveState {
        self.state
    }

    fn check_fault(&self) -> Result<(), TrajectoryFault> {
        if self.state.fault.is_none() {
            Ok(())
        } else {
            Err(self.state.fault)
        }
    }

    fn latch(&mut self, fault: TrajectoryFault) -> TrajectoryFault {
        self.state.fault = fault;
        fault
    }

    const fn setpoint(&self) -> Setpoint {
        Setpoint {
            position: self.state.position,
            velocity: self.state.velocity,
            acceleration: self.state.acceleration,
            jerk: self.state.jerk,
            finished: self.state.finished,
        }
    }

    /// **Satisfies:** RON-FR-512
    fn finish(&mut self) {
        self.state.position = self.state.target;
        self.state.velocity = 0.0;
        self.state.acceleration = 0.0;
        self.state.jerk = 0.0;
        self.state.elapsed = self.state.total_time;
        self.state.phase_elapsed = 0.0;
        self.state.phase = SCurvePhase::Done;
        self.state.finished = true;
    }

    /// Plans symmetric jerk phases of length `t_j` around an optional cruise.
    ///
    /// **Satisfies:** RON-FR-510
    fn plan(&mut self) {
        self.state.phase_times = [0.0; SCURVE_PHASE_COUNT];
        let offset = self.state.target - self.state.position;
        let distance = abs(offset);
        if distance <= POSITION_TOLERANCE {
            self.finish();
            return;
        }
        let config = self.config;
        self.state.direction = sign_nonzero(offset);
        let acceleration_limited = config.a_max / config.j_max;
        let velocity_limited = sqrt(config.v_max / config.j_max);
        let limit = acceleration_limited.min(velocity_limited);
        let short_move = cbrt(distance / (2.0 * config.j_max));
        self.state.phase_elapsed = 0.0;
        self.state.elapsed = 0.0;
        self.state.phase = SCurvePhase::JerkPositive1;
        self.state.finished = false;

        let (jerk_time, cruise_time) = if short_move < limit {
            (short_move, 0.0)
        } else {
            let peak_velocity = config.j_max * limit * limit;
            let ramp_distance = 2.0 * config.j_max * limit * limit * limit;
            (limit, (distance - ramp_distance) / peak_velocity)
        };
        self.state.phase_times = [
            jerk_time,
            0.0,
            jerk_time,
            cruise_time,
            jerk_time,
            0.0,
            jerk_time,
        ];
        self.state.total_time = (4.0 * jerk_time) + cruise_time;
    }

    /// **Satisfies:** RON-FR-511
    fn phase_jerk(&self) -> RonFloat {
        let j_max = self.state.direction * self.config.j_max;
        match self.state.phase {
            SCurvePhase::JerkPositive1 | SCurvePhase::JerkPositive2 => j_max,
            SCurvePhase::JerkNegative1 | SCurvePhase::JerkNegative2 => -j_max,
            SCurvePhase::AccelerationHold
            | SCurvePhase::ConstantVelocity
            | SCurvePhase::DecelerationHold
            | SCurvePhase::Done => 0.0,
        }
    }

    fn advance_phase(&mut self) {
        self.state.phase = self.state.phase.next();
        self.state.phase_elapsed = 0.0;
    }

    /// Skips phases with no planned duration. Bounded by the phase count.
    fn advance_empty_phases(&mut self) {
        for _ in 0..SCURVE_PHASE_COUNT {
            if self.state.phase == SCurvePhase::Done
                || self.state.phase_time(self.state.phase) > POSITION_TOLERANCE
            {
                break;
            }
            self.advance_phase();
        }
    }

    /// Integrates `dt` exactly across phase boundaries. At most one segment
    /// per phase, so the loop is bounded by the phase count.
    ///
    /// **Satisfies:** RON-FR-510, RON-FR-511
    fn integrate(&mut self, dt: RonFloat) {
        let mut remaining = dt;
        for _ in 0..SCURVE_PHASE_COUNT {
            if remaining <= 0.0 {
                break;
            }
            self.advance_empty_phases();
            if self.state.phase == SCurvePhase::Done {
                break;
            }
            let phase_remaining =
                self.state.phase_time(self.state.phase) - self.state.phase_elapsed;
            let h = remaining.min(phase_remaining);
            self.integrate_segment(h, self.phase_jerk());
            self.state.phase_elapsed += h;
            self.state.elapsed += h;
            remaining -= h;
            if phase_remaining - h <= POSITION_TOLERANCE {
                self.advance_phase();
            }
        }
    }

    /// Exact constant-jerk integration over `h` seconds.
    ///
    /// **Satisfies:** RON-FR-510, RON-FR-511
    fn integrate_segment(&mut self, h: RonFloat, jerk: RonFloat) {
        let h2 = h * h;
        let h3 = h2 * h;
        self.state.position +=
            (self.state.velocity * h) + (0.5 * self.state.acceleration * h2) + ((jerk * h3) / 6.0);
        self.state.velocity += (self.state.acceleration * h) + (0.5 * jerk * h2);
        self.state.acceleration += jerk * h;
        self.state.jerk = jerk;
    }
}

/// Cube root by a fixed number of Newton iterations (no math library).
///
/// **Satisfies:** RON-FR-510
fn cbrt(value: RonFloat) -> RonFloat {
    let mut estimate = if value > 1.0 { value } else { 1.0 };
    for _ in 0..CBRT_STEPS {
        estimate = ((2.0 * estimate) + (value / (estimate * estimate))) / 3.0;
    }
    estimate
}

/// State at rest at `position`; shared by `new` and `reset`.
///
/// **Satisfies:** RON-FR-512, RON-FR-514
fn seed(position: RonFloat) -> SCurveState {
    SCurveState {
        position,
        target: position,
        direction: 1.0,
        finished: true,
        ..SCurveState::default()
    }
}
