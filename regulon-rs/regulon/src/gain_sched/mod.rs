//! # `gain_sched`
//!
//! Gain scheduling: a bounded table of PID configurations indexed by a
//! scheduling variable, applied to a [`Pid`] atomically.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-300-RON-FR-306
//! **Tests:** RON-TC-GS-001-RON-TC-GS-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    pid::{Pid, PidConfig, PidMode},
    platform::{is_finite, RonFloat},
};

/// Maximum number of breakpoints in a gain-scheduling table.
///
/// **Satisfies:** RON-FR-301
pub const GS_MAX_BREAKPOINTS: usize = 16;

/// Transition between breakpoint configurations.
///
/// **Satisfies:** RON-FR-302
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ScheduleMode {
    /// Apply the whole configuration of the breakpoint at or below `sigma`.
    #[default]
    HardSwitch,
    /// Interpolate `kp`, `ki` and `kd` between the bracketing breakpoints.
    /// Every other configuration field must match across the table.
    LinearInterpolation,
}

/// Validated gain-scheduling table with `N` breakpoints.
///
/// The table is immutable once built, so [`GainSchedule::apply`] never has to
/// re-validate it.
///
/// **Satisfies:** RON-FR-300, RON-FR-301
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GainSchedule<const N: usize> {
    breakpoints: [RonFloat; N],
    configs: [PidConfig; N],
    mode: ScheduleMode,
    reset_integral_on_switch: bool,
}

impl<const N: usize> GainSchedule<N> {
    /// Builds and validates a gain-scheduling table.
    ///
    /// **Satisfies:** RON-FR-300, RON-FR-301, RON-FR-306
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when `N` is zero or above
    /// [`GS_MAX_BREAKPOINTS`], a breakpoint is non-finite or not strictly
    /// increasing, a configuration fails [`PidConfig::validate`], or, for
    /// [`ScheduleMode::LinearInterpolation`], neighbouring configurations
    /// differ in anything but their gains.
    pub fn new(
        breakpoints: [RonFloat; N],
        configs: [PidConfig; N],
        mode: ScheduleMode,
        reset_integral_on_switch: bool,
    ) -> Result<Self, RonError> {
        if N == 0 || N > GS_MAX_BREAKPOINTS {
            return Err(RonError::ConfigInvalid("breakpoint count out of range"));
        }
        let mut previous: Option<(RonFloat, &PidConfig)> = None;
        for (sigma, config) in breakpoints.iter().zip(configs.iter()) {
            if !is_finite(*sigma) {
                return Err(RonError::ConfigInvalid("breakpoint must be finite"));
            }
            config.validate()?;
            if let Some((previous_sigma, previous_config)) = previous {
                if *sigma <= previous_sigma {
                    return Err(RonError::ConfigInvalid(
                        "breakpoints must be strictly increasing",
                    ));
                }
                if matches!(mode, ScheduleMode::LinearInterpolation)
                    && !interpolation_compatible(previous_config, config)
                {
                    return Err(RonError::ConfigInvalid(
                        "interpolated configurations may differ only in gains",
                    ));
                }
            }
            previous = Some((*sigma, config));
        }
        Ok(Self {
            breakpoints,
            configs,
            mode,
            reset_integral_on_switch,
        })
    }

    /// Returns the configuration scheduled for `sigma`.
    ///
    /// Values outside the table clamp to the first or last breakpoint.
    ///
    /// **Satisfies:** RON-FR-302
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `sigma` is non-finite.
    pub fn config_at(&self, sigma: RonFloat) -> Result<PidConfig, RonError> {
        if !is_finite(sigma) {
            return Err(RonError::InvalidArgument(
                "scheduling variable must be finite",
            ));
        }
        // Binary search: O(log N) and bounded by GS_MAX_BREAKPOINTS.
        let lower = self
            .breakpoints
            .partition_point(|breakpoint| *breakpoint <= sigma)
            .saturating_sub(1);
        let lower_entry = self.entry(lower)?;
        if matches!(self.mode, ScheduleMode::HardSwitch) || sigma <= lower_entry.0 {
            return Ok(lower_entry.1);
        }
        let Some(upper_entry) = self.entry(lower + 1).ok() else {
            return Ok(lower_entry.1);
        };
        let t = (sigma - lower_entry.0) / (upper_entry.0 - lower_entry.0);
        Ok(PidConfig {
            kp: lerp(lower_entry.1.kp, upper_entry.1.kp, t),
            ki: lerp(lower_entry.1.ki, upper_entry.1.ki, t),
            kd: lerp(lower_entry.1.kd, upper_entry.1.kd, t),
            ..lower_entry.1
        })
    }

    /// Applies the configuration scheduled for `sigma` to `pid`.
    ///
    /// The update is atomic. In [`ScheduleMode::HardSwitch`] with
    /// `reset_integral_on_switch`, an update that changes the configuration
    /// also clears the integrator.
    ///
    /// **Satisfies:** RON-FR-302-RON-FR-305
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `sigma` is non-finite; the
    /// controller is then left unchanged.
    pub fn apply(&self, pid: &mut Pid, sigma: RonFloat) -> Result<(), RonError> {
        let candidate = self.config_at(sigma)?;
        let switched = pid.configuration() != candidate;
        pid.update_configuration(candidate)?;
        if matches!(self.mode, ScheduleMode::HardSwitch)
            && self.reset_integral_on_switch
            && switched
        {
            pid.set_integral(0.0)?;
        }
        Ok(())
    }

    /// Returns the transition mode.
    #[must_use]
    pub const fn mode(&self) -> ScheduleMode {
        self.mode
    }

    fn entry(&self, index: usize) -> Result<(RonFloat, PidConfig), RonError> {
        match (self.breakpoints.get(index), self.configs.get(index)) {
            (Some(sigma), Some(config)) => Ok((*sigma, *config)),
            _ => Err(RonError::ConfigInvalid("breakpoint index out of range")),
        }
    }
}

/// Two configurations can be interpolated when they differ only in their
/// gains (and the construction-time `initial_mode`).
///
/// **Satisfies:** RON-FR-302, RON-FR-303
fn interpolation_compatible(lhs: &PidConfig, rhs: &PidConfig) -> bool {
    without_gains(lhs) == without_gains(rhs)
}

fn without_gains(config: &PidConfig) -> PidConfig {
    PidConfig {
        kp: 0.0,
        ki: 0.0,
        kd: 0.0,
        initial_mode: PidMode::Automatic,
        ..*config
    }
}

/// **Satisfies:** RON-FR-302
fn lerp(lower: RonFloat, upper: RonFloat, t: RonFloat) -> RonFloat {
    lower + ((upper - lower) * t)
}
