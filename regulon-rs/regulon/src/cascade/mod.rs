//! # `cascade`
//!
//! Cascade (master/slave) control built from two [`Pid`] loops: the outer
//! loop's output is the inner loop's setpoint.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-400-RON-FR-406
//! **Tests:** RON-TC-CASC-001-RON-TC-CASC-012
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    pid::{AntiWindupMode, Pid, PidConfig, PidFault, PidMode, PidStatus},
    platform::{is_finite, RonFloat},
};

/// Bit offset of the inner loop's status in [`CascadeStatus::bits`].
const INNER_STATUS_SHIFT: u32 = 16;

/// Unified cascade status: one status word per loop.
///
/// **Satisfies:** RON-FR-406
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CascadeStatus {
    /// Outer (master) loop status.
    pub outer: PidStatus,
    /// Inner (slave) loop status.
    pub inner: PidStatus,
}

impl CascadeStatus {
    /// Returns the packed word used by the C API: outer status in bits 0-15,
    /// inner status in bits 16-31.
    ///
    /// **Satisfies:** RON-FR-406
    #[must_use]
    pub const fn bits(self) -> u32 {
        (self.outer.bits() as u32) | ((self.inner.bits() as u32) << INNER_STATUS_SHIFT)
    }
}

/// Cascade controller instance.
///
/// **Satisfies:** RON-FR-400
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cascade {
    outer: Pid,
    inner: Pid,
}

impl Cascade {
    /// Creates a cascade from validated outer and inner configurations.
    ///
    /// The outer output limits bound the inner setpoint (RON-FR-402).
    ///
    /// **Satisfies:** RON-FR-400, RON-FR-402, RON-FR-405
    ///
    /// # Errors
    ///
    /// Returns an error when either configuration is invalid.
    pub fn new(outer: PidConfig, inner: PidConfig) -> Result<Self, RonError> {
        Ok(Self {
            outer: Pid::new(outer)?,
            inner: Pid::new(inner)?,
        })
    }

    /// Executes one cascade step and returns the inner loop's output.
    ///
    /// The outer loop runs first; its (saturated) output becomes the inner
    /// setpoint. When the inner loop saturates and the outer loop uses
    /// back-calculation anti-windup, the outer integrator is corrected by
    /// `dt / T_aw * (u_inner - u_outer)` (RON-FR-403).
    ///
    /// A faulted loop contributes its safe-state output, so both loops always
    /// run; [`Cascade::last_output`] and [`Cascade::status`] report the result
    /// of a failed step.
    ///
    /// **Satisfies:** RON-FR-401-RON-FR-403, RON-FR-406
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] without stepping either loop when
    /// `dt` is not positive and finite. Otherwise returns the loops' errors,
    /// with latched faults of both loops OR-ed together.
    pub fn step(
        &mut self,
        outer_setpoint: RonFloat,
        outer_measurement: RonFloat,
        inner_measurement: RonFloat,
        dt: RonFloat,
    ) -> Result<(RonFloat, CascadeStatus), RonError> {
        if !is_finite(dt) || dt <= 0.0 {
            return Err(RonError::InvalidArgument(
                "sample period must be positive and finite",
            ));
        }
        let outer_result = self.outer.step(outer_setpoint, outer_measurement, dt);
        let (outer_output, outer_status) = loop_result(&self.outer, outer_result);
        let inner_result = self.inner.step(outer_output, inner_measurement, dt);
        let (inner_output, inner_status) = loop_result(&self.inner, inner_result);

        self.apply_cross_anti_windup(outer_output, inner_output, inner_status, dt)?;

        let status = CascadeStatus {
            outer: outer_status,
            inner: inner_status,
        };
        match (outer_result, inner_result) {
            (Ok(_), Ok(_)) => Ok((inner_output, status)),
            (Err(RonError::Fault(outer)), Err(RonError::Fault(inner))) => {
                Err(RonError::Fault(outer | inner))
            }
            (Err(error), _) | (Ok(_), Err(error)) => Err(error),
        }
    }

    /// Switches both loops to `mode` in bumpless order: outer first when
    /// entering manual, inner first when returning to automatic.
    ///
    /// **Satisfies:** RON-FR-404
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] without changing either loop when
    /// a manual value is non-finite.
    pub fn set_mode(
        &mut self,
        mode: PidMode,
        manual_inner: RonFloat,
        manual_outer: RonFloat,
    ) -> Result<(), RonError> {
        if !is_finite(manual_inner) || !is_finite(manual_outer) {
            return Err(RonError::InvalidArgument("manual outputs must be finite"));
        }
        match mode {
            PidMode::Manual => {
                self.outer.set_mode(mode, manual_outer)?;
                self.inner.set_mode(mode, manual_inner)
            }
            PidMode::Automatic => {
                self.inner.set_mode(mode, manual_inner)?;
                self.outer.set_mode(mode, manual_outer)
            }
        }
    }

    /// Returns the current status of both loops.
    ///
    /// **Satisfies:** RON-FR-406
    #[must_use]
    pub fn status(&self) -> CascadeStatus {
        CascadeStatus {
            outer: self.outer.state().status,
            inner: self.inner.state().status,
        }
    }

    /// Returns the latched faults as `(outer, inner)`.
    ///
    /// **Satisfies:** RON-FR-406
    #[must_use]
    pub fn faults(&self) -> (PidFault, PidFault) {
        (self.outer.state().fault, self.inner.state().fault)
    }

    /// Returns the cascade's actuator command: the inner loop's last output,
    /// or its safe-state output while the inner loop is faulted.
    #[must_use]
    pub fn last_output(&self) -> RonFloat {
        self.inner.output()
    }

    /// Clears the fault registers of both loops, keeping dynamic state.
    ///
    /// **Satisfies:** RON-FR-405
    pub fn clear_fault(&mut self) {
        self.outer.clear_fault();
        self.inner.clear_fault();
    }

    /// Resets the dynamic state of both loops, keeping configuration and mode.
    ///
    /// **Satisfies:** RON-FR-405
    pub fn reset(&mut self) {
        self.outer.reset();
        self.inner.reset();
    }

    /// Returns the outer (master) loop.
    #[must_use]
    pub const fn outer(&self) -> &Pid {
        &self.outer
    }

    /// Returns the inner (slave) loop.
    #[must_use]
    pub const fn inner(&self) -> &Pid {
        &self.inner
    }

    /// Returns the outer loop for runtime reconfiguration.
    pub fn outer_mut(&mut self) -> &mut Pid {
        &mut self.outer
    }

    /// Returns the inner loop for runtime reconfiguration.
    pub fn inner_mut(&mut self) -> &mut Pid {
        &mut self.inner
    }

    /// **Satisfies:** RON-FR-403
    fn apply_cross_anti_windup(
        &mut self,
        outer_output: RonFloat,
        inner_output: RonFloat,
        inner_status: PidStatus,
        dt: RonFloat,
    ) -> Result<(), RonError> {
        let config = self.outer.configuration();
        if !inner_status.contains(PidStatus::SATURATED)
            || !matches!(config.anti_windup_mode, AntiWindupMode::BackCalculation)
        {
            return Ok(());
        }
        let correction = (dt / config.anti_windup_tracking_time) * (inner_output - outer_output);
        self.outer.set_integral(self.outer.integral() + correction)
    }
}

/// Output and status a loop contributes, including after a failed step.
fn loop_result(
    pid: &Pid,
    result: Result<(RonFloat, PidStatus), RonError>,
) -> (RonFloat, PidStatus) {
    match result {
        Ok(output) => output,
        Err(_) => (pid.output(), pid.state().status),
    }
}
