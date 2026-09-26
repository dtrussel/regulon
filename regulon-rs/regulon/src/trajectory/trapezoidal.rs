//! # `trajectory::trapezoidal`
//!
//! Trapezoidal velocity profile; short moves degenerate to a triangular
//! profile that never reaches `v_max`.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-500-RON-FR-503, RON-FR-512-RON-FR-515
//! **Tests:** RON-TC-TRAJ-001-RON-TC-TRAJ-004, RON-TC-TRAJ-007-RON-TC-TRAJ-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{all_positive_finite, valid_dt, Setpoint, TrajectoryFault, POSITION_TOLERANCE};
use crate::platform::{abs, clamp, is_finite, sign_nonzero, sqrt, RonFloat};

/// Trapezoidal profile limits.
///
/// **Satisfies:** RON-FR-500
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrapezoidalConfig {
    /// Maximum speed; positive and finite.
    pub v_max: RonFloat,
    /// Maximum acceleration; positive and finite.
    pub a_max: RonFloat,
}

/// Trapezoidal profile phase.
///
/// **Satisfies:** RON-FR-500, RON-FR-512, RON-FR-513
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum TrapezoidalPhase {
    /// Accelerating towards the peak speed.
    Accelerate,
    /// Cruising at the peak speed.
    ConstantVelocity,
    /// Decelerating onto the target.
    Decelerate,
    /// Execution paused by [`Trapezoidal::set_hold`].
    Hold,
    /// At rest on the target.
    #[default]
    Done,
}

/// Read-only trapezoidal generator state.
///
/// **Satisfies:** RON-FR-515
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrapezoidalState {
    /// Current position.
    pub position: RonFloat,
    /// Current velocity.
    pub velocity: RonFloat,
    /// Current acceleration.
    pub acceleration: RonFloat,
    /// Target position.
    pub target: RonFloat,
    /// Direction of the move, `1` or `-1`.
    pub direction: RonFloat,
    /// Planned peak speed.
    pub peak_velocity: RonFloat,
    /// Current phase.
    pub phase: TrapezoidalPhase,
    /// Latched fault register.
    pub fault: TrajectoryFault,
    /// `true` while execution is paused.
    pub hold: bool,
    /// `true` once the target has been reached.
    pub finished: bool,
}

/// Trapezoidal trajectory generator.
///
/// **Satisfies:** RON-FR-500
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trapezoidal {
    config: TrapezoidalConfig,
    state: TrapezoidalState,
}

impl Trapezoidal {
    /// Creates a generator at rest at `position` with no move planned.
    ///
    /// **Satisfies:** RON-FR-500, RON-FR-512
    ///
    /// # Errors
    ///
    /// Returns [`TrajectoryFault::CONFIG_INVALID`] when a limit is not positive
    /// and finite, or `position` is not finite.
    pub fn new(config: TrapezoidalConfig, position: RonFloat) -> Result<Self, TrajectoryFault> {
        if !all_positive_finite(&[config.v_max, config.a_max]) || !is_finite(position) {
            return Err(TrajectoryFault::CONFIG_INVALID);
        }
        Ok(Self {
            config,
            state: seed(position),
        })
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> TrapezoidalConfig {
        self.config
    }

    /// Sets a new target and re-plans from the current position and velocity,
    /// so a mid-move change decelerates and reverses rather than jumping.
    ///
    /// **Satisfies:** RON-FR-501, RON-FR-503
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
    /// **Satisfies:** RON-FR-500, RON-FR-502, RON-FR-512
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
        if self.state.hold {
            self.state.phase = TrapezoidalPhase::Hold;
            return Ok(self.setpoint());
        }
        if self.state.finished {
            return Ok(self.setpoint());
        }
        self.integrate(dt);
        self.finish_if_reached(dt);
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
    /// returns the current setpoint unchanged; resuming re-plans from it.
    ///
    /// **Satisfies:** RON-FR-513
    ///
    /// # Errors
    ///
    /// Returns the latched fault, if any.
    pub fn set_hold(&mut self, hold: bool) -> Result<(), TrajectoryFault> {
        self.check_fault()?;
        self.state.hold = hold;
        if hold {
            self.state.phase = TrapezoidalPhase::Hold;
        } else if !self.state.finished {
            self.plan();
        }
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
    pub const fn state(&self) -> TrapezoidalState {
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
            jerk: 0.0,
            finished: self.state.finished,
        }
    }

    /// **Satisfies:** RON-FR-501, RON-FR-503
    fn plan(&mut self) {
        let offset = self.state.target - self.state.position;
        let distance = abs(offset);
        if distance <= POSITION_TOLERANCE {
            self.state.direction = 1.0;
            self.state.peak_velocity = 0.0;
            self.state.phase = TrapezoidalPhase::Done;
            self.state.finished = true;
            return;
        }
        self.state.direction = sign_nonzero(offset);
        self.state.peak_velocity =
            clamp(sqrt(self.config.a_max * distance), 0.0, self.config.v_max);
        self.state.phase = TrapezoidalPhase::Accelerate;
        self.state.finished = false;
    }

    /// **Satisfies:** RON-FR-500, RON-FR-501
    fn integrate(&mut self, dt: RonFloat) {
        let distance = abs(self.state.target - self.state.position);
        let speed_along = self.state.velocity * self.state.direction;
        let braking_distance = if speed_along > 0.0 {
            (speed_along * speed_along) / (2.0 * self.config.a_max)
        } else {
            0.0
        };
        let a_max = self.config.a_max;
        let (acceleration, phase) = if speed_along < 0.0 {
            (self.state.direction * a_max, TrapezoidalPhase::Accelerate)
        } else if distance <= braking_distance + POSITION_TOLERANCE {
            (-self.state.direction * a_max, TrapezoidalPhase::Decelerate)
        } else if speed_along >= self.state.peak_velocity {
            (0.0, TrapezoidalPhase::ConstantVelocity)
        } else {
            (self.state.direction * a_max, TrapezoidalPhase::Accelerate)
        };
        self.state.acceleration = acceleration;
        self.state.phase = phase;
        self.state.velocity += acceleration * dt;

        let next_speed_along = self.state.velocity * self.state.direction;
        if phase == TrapezoidalPhase::Accelerate && next_speed_along > self.state.peak_velocity {
            self.state.velocity = self.state.direction * self.state.peak_velocity;
            self.state.acceleration = 0.0;
        } else if phase == TrapezoidalPhase::Decelerate && next_speed_along < 0.0 {
            self.state.velocity = 0.0;
            self.state.acceleration = 0.0;
        }
        self.state.position += self.state.velocity * dt;
    }

    /// **Satisfies:** RON-FR-512
    fn finish_if_reached(&mut self, dt: RonFloat) {
        let remaining = self.state.direction * (self.state.target - self.state.position);
        if remaining <= POSITION_TOLERANCE
            && (self.state.phase == TrapezoidalPhase::Decelerate
                || abs(self.state.velocity) <= self.config.a_max * dt)
        {
            self.state.position = self.state.target;
            self.state.velocity = 0.0;
            self.state.acceleration = 0.0;
            self.state.phase = TrapezoidalPhase::Done;
            self.state.finished = true;
        }
    }
}

/// State at rest at `position`; shared by `new` and `reset`.
///
/// **Satisfies:** RON-FR-512, RON-FR-514
fn seed(position: RonFloat) -> TrapezoidalState {
    TrapezoidalState {
        position,
        target: position,
        direction: 1.0,
        finished: true,
        ..TrapezoidalState::default()
    }
}
