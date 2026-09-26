//! # `statespace`
//!
//! Discrete-time state-feedback controller `u = -K x_hat + K_r r` with
//! optional integral augmentation and PID-equivalent saturation and rate
//! limiting. The estimate comes from an embedded [`Estimator`], which the
//! caller advances each cycle through [`StateSpace::estimator_mut`].
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-700-RON-FR-704, RON-SR-012, RON-SR-013
//! **Tests:** RON-TC-SS-001-RON-TC-SS-005, RON-TC-SS-009, RON-TC-SS-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    estimator::{Estimator, EstimatorConfig},
    matrix::{dimension_valid, vector_is_finite},
    pid::{PidFault, PidStatus},
    platform::{clamp, is_finite, rate_limit, RonFloat},
};

/// Integral augmentation for output regulation: the integral of
/// `r - C_out x_hat` is added to the control output.
///
/// **Satisfies:** RON-FR-702
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegralAugmentation<const N: usize> {
    /// Integral gain.
    pub gain: RonFloat,
    /// Regulated-output row.
    pub output_row: [RonFloat; N],
    /// Lower integral clamp.
    pub min: RonFloat,
    /// Upper integral clamp; `>= min`.
    pub max: RonFloat,
}

/// Output limits shared by the state-feedback controllers.
///
/// **Satisfies:** RON-FR-703
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OutputLimits {
    /// Minimum output; below `max`.
    pub min: RonFloat,
    /// Maximum output.
    pub max: RonFloat,
    /// Maximum output change per second; `<= 0` disables rate limiting.
    pub rate_limit: RonFloat,
}

impl OutputLimits {
    /// Validates the limits.
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when a limit is not finite or
    /// `min >= max`.
    pub fn validate(&self) -> Result<(), RonError> {
        if !is_finite(self.min) || !is_finite(self.max) || !is_finite(self.rate_limit) {
            return Err(RonError::ConfigInvalid("output limits must be finite"));
        }
        if self.min >= self.max {
            return Err(RonError::ConfigInvalid(
                "output minimum must be below maximum",
            ));
        }
        Ok(())
    }

    /// Saturates then rate-limits `raw` from `previous`, reporting both in
    /// the status word.
    ///
    /// **Satisfies:** RON-FR-020, RON-FR-022, RON-FR-703
    pub(crate) fn apply(
        &self,
        raw: RonFloat,
        previous: RonFloat,
        dt: RonFloat,
    ) -> (RonFloat, PidStatus) {
        let mut status = PidStatus::OK;
        if raw < self.min || raw > self.max {
            status |= PidStatus::SATURATED;
        }
        let (output, limited) = rate_limit(
            clamp(raw, self.min, self.max),
            previous,
            self.rate_limit,
            dt,
        );
        if limited {
            status |= PidStatus::RATE_LIMITED;
        }
        (output, status)
    }
}

/// State-space controller configuration for `N` states and an estimator with
/// `M` outputs/measurements and `P` inputs.
///
/// **Satisfies:** RON-FR-700-RON-FR-703
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateSpaceConfig<const N: usize, const M: usize, const P: usize> {
    /// State-estimate source.
    pub estimator: EstimatorConfig<N, M, P>,
    /// State-feedback row gain.
    pub k: [RonFloat; N],
    /// Reference pre-gain.
    pub kr: RonFloat,
    /// Optional integral augmentation.
    pub integral: Option<IntegralAugmentation<N>>,
    /// Output limits.
    pub limits: OutputLimits,
}

/// State-feedback controller.
///
/// **Satisfies:** RON-FR-700
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateSpace<const N: usize, const M: usize, const P: usize> {
    estimator: Estimator<N, M, P>,
    k: [RonFloat; N],
    kr: RonFloat,
    integral_config: Option<IntegralAugmentation<N>>,
    limits: OutputLimits,
    integral: RonFloat,
    output_prev: RonFloat,
    fault: PidFault,
}

impl<const N: usize, const M: usize, const P: usize> StateSpace<N, M, P> {
    /// Creates the controller and its estimator.
    ///
    /// **Satisfies:** RON-FR-700, RON-FR-701, RON-FR-723
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] for out-of-range dimensions,
    /// non-finite gains, invalid limits or integral settings, or an invalid
    /// estimator configuration.
    pub fn new(config: StateSpaceConfig<N, M, P>) -> Result<Self, RonError> {
        if !dimension_valid(N, false) {
            return Err(RonError::ConfigInvalid("state dimension out of range"));
        }
        if !vector_is_finite(&config.k) || !is_finite(config.kr) {
            return Err(RonError::ConfigInvalid(
                "state-feedback gains must be finite",
            ));
        }
        config.limits.validate()?;
        if let Some(integral) = config.integral {
            validate_integral(&integral)?;
        }
        Ok(Self {
            estimator: Estimator::new(config.estimator)?,
            k: config.k,
            kr: config.kr,
            integral_config: config.integral,
            limits: config.limits,
            integral: 0.0,
            output_prev: 0.0,
            fault: PidFault::NONE,
        })
    }

    /// Computes one control output from the current estimate. The estimate
    /// is not advanced here; drive the estimator separately each cycle.
    ///
    /// A runtime fault latches (RON-SR-012): it is OR-ed into [`Self::fault`],
    /// the integral and output history are left unchanged (so
    /// [`Self::output`] holds the last output), and every later step returns
    /// the latched fault until [`Self::clear_fault`] or [`Self::reset`].
    ///
    /// **Satisfies:** RON-FR-700, RON-FR-702, RON-FR-703, RON-SR-010,
    /// RON-SR-012, RON-SR-013
    ///
    /// # Errors
    ///
    /// Returns [`RonError::Fault`] with the latched bits:
    /// [`PidFault::INPUT_NOT_FINITE`] when `r` is not finite or `dt` is not
    /// positive and finite, [`PidFault::OUTPUT_NOT_FINITE`] when the control
    /// law overflows, or the bits latched by an earlier step.
    pub fn step(&mut self, r: RonFloat, dt: RonFloat) -> Result<(RonFloat, PidStatus), RonError> {
        if self.fault.is_none() {
            match self.evaluate(r, dt) {
                Ok((output, integral, status)) => {
                    self.integral = integral;
                    self.output_prev = output;
                    return Ok((output, status));
                }
                Err(fault) => self.fault |= fault,
            }
        }
        Err(RonError::Fault(self.fault))
    }

    /// Evaluates one step without changing the controller, returning the
    /// limited output, the advanced integral and the status word.
    ///
    /// **Satisfies:** RON-FR-700, RON-FR-702, RON-FR-703
    fn evaluate(
        &self,
        r: RonFloat,
        dt: RonFloat,
    ) -> Result<(RonFloat, RonFloat, PidStatus), PidFault> {
        if !is_finite(r) || !is_finite(dt) || dt <= 0.0 {
            return Err(PidFault::INPUT_NOT_FINITE);
        }
        let x_hat = self.estimator.state();
        let mut raw = -dot(&self.k, &x_hat) + (self.kr * r);
        let mut integral = self.integral;
        if let Some(augmentation) = self.integral_config {
            let regulation_error = r - dot(&augmentation.output_row, &x_hat);
            integral = clamp(
                integral + (augmentation.gain * dt * regulation_error),
                augmentation.min,
                augmentation.max,
            );
            raw += integral;
        }
        if !is_finite(raw) {
            return Err(PidFault::OUTPUT_NOT_FINITE);
        }
        let (output, status) = self.limits.apply(raw, self.output_prev, dt);
        Ok((output, integral, status))
    }

    /// Clears the latched fault register; the integral, output history and
    /// estimator are left as they were when the fault latched.
    ///
    /// **Satisfies:** RON-SR-012
    pub fn clear_fault(&mut self) {
        self.fault = PidFault::NONE;
    }

    /// Returns the latched fault bits.
    ///
    /// **Satisfies:** RON-SR-013
    #[must_use]
    pub const fn fault(&self) -> PidFault {
        self.fault
    }

    /// Returns the last committed output, which a faulted step holds.
    ///
    /// **Satisfies:** RON-FR-703
    #[must_use]
    pub const fn output(&self) -> RonFloat {
        self.output_prev
    }

    /// Replaces `K` and `K_r` without touching the estimator, integral or
    /// output history.
    ///
    /// **Satisfies:** RON-FR-704
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when a gain is not finite; the
    /// gains are then unchanged.
    pub fn set_gains(&mut self, k: &[RonFloat; N], kr: RonFloat) -> Result<(), RonError> {
        if !vector_is_finite(k) || !is_finite(kr) {
            return Err(RonError::ConfigInvalid(
                "state-feedback gains must be finite",
            ));
        }
        self.k = *k;
        self.kr = kr;
        Ok(())
    }

    /// Clears the integral, output history and latched faults and resets the
    /// estimator.
    ///
    /// **Satisfies:** RON-FR-702, RON-SR-012
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.output_prev = 0.0;
        self.fault = PidFault::NONE;
        self.estimator.reset();
    }

    /// Returns the estimator.
    #[must_use]
    pub const fn estimator(&self) -> &Estimator<N, M, P> {
        &self.estimator
    }

    /// Returns the estimator, to advance it each cycle.
    ///
    /// **Satisfies:** RON-FR-701
    pub fn estimator_mut(&mut self) -> &mut Estimator<N, M, P> {
        &mut self.estimator
    }

    /// Returns the augmented-integral accumulator.
    #[must_use]
    pub const fn integral(&self) -> RonFloat {
        self.integral
    }
}

/// **Satisfies:** RON-FR-702
fn validate_integral<const N: usize>(integral: &IntegralAugmentation<N>) -> Result<(), RonError> {
    if !is_finite(integral.gain)
        || !is_finite(integral.min)
        || !is_finite(integral.max)
        || !vector_is_finite(&integral.output_row)
    {
        return Err(RonError::ConfigInvalid(
            "integral augmentation must be finite",
        ));
    }
    if integral.min > integral.max {
        return Err(RonError::ConfigInvalid("integral minimum above maximum"));
    }
    Ok(())
}

/// **Satisfies:** RON-FR-700, RON-FR-702
pub(crate) fn dot<const N: usize>(lhs: &[RonFloat; N], rhs: &[RonFloat; N]) -> RonFloat {
    lhs.iter()
        .zip(rhs.iter())
        .fold(0.0, |sum, (a, b)| sum + (a * b))
}
