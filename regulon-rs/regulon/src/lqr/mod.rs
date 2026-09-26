//! # `lqr`
//!
//! Discrete-time MIMO linear quadratic regulator `u = -K x_hat + K_r r`. `K`
//! is supplied pre-computed or solved once at construction from the discrete
//! algebraic Riccati equation (DARE) by bounded value iteration.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-730-RON-FR-739
//! **Tests:** RON-TC-LQR-001-RON-TC-LQR-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    estimator::{Estimator, EstimatorConfig},
    matrix::{dimension_valid, vector_is_finite, Matrix},
    pid::PidStatus,
    platform::{abs, clamp, is_finite, RonFloat},
    statespace::OutputLimits,
};

/// Iteration limit used when [`DareConfig::max_iterations`] is 0.
///
/// **Satisfies:** RON-FR-733
pub const DARE_DEFAULT_MAX_ITERATIONS: u16 = 200;

/// DARE problem for `N` states and `U` inputs.
///
/// **Satisfies:** RON-FR-731, RON-FR-733
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DareConfig<const N: usize, const U: usize> {
    /// State transition matrix.
    pub a: Matrix<N, N>,
    /// Input matrix.
    pub b: Matrix<N, U>,
    /// State cost (positive semi-definite).
    pub q: Matrix<N, N>,
    /// Input cost (positive definite).
    pub r: Matrix<U, U>,
    /// Iteration limit; 0 selects [`DARE_DEFAULT_MAX_ITERATIONS`].
    pub max_iterations: u16,
    /// Convergence tolerance on the largest change in `P`; positive. It is
    /// compared with values of `P`'s magnitude, so in single precision a
    /// tolerance near machine epsilon may never be reached.
    pub tolerance: RonFloat,
}

impl<const N: usize, const U: usize> DareConfig<N, U> {
    /// Validates finiteness and the tolerance.
    ///
    /// **Satisfies:** RON-FR-731
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when an entry is not finite or the
    /// tolerance is not positive.
    pub fn validate(&self) -> Result<(), RonError> {
        if !self.a.is_finite() || !self.b.is_finite() || !self.q.is_finite() || !self.r.is_finite()
        {
            return Err(RonError::ConfigInvalid("DARE matrices must be finite"));
        }
        if !is_finite(self.tolerance) || self.tolerance <= 0.0 {
            return Err(RonError::ConfigInvalid("DARE tolerance must be positive"));
        }
        Ok(())
    }
}

/// Converged DARE solution.
///
/// **Satisfies:** RON-FR-733, RON-FR-739
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DareSolution<const N: usize, const U: usize> {
    /// Optimal state-feedback gain.
    pub k: Matrix<U, N>,
    /// Riccati solution (cost-to-go matrix).
    pub p: Matrix<N, N>,
}

/// Solves the DARE by value iteration from `P = Q`:
/// `K = (R + B^T P B)^-1 B^T P A`, `P' = Q + A^T P A - A^T P B K`, until the
/// largest change in `P` is below the tolerance. No Schur decomposition, and
/// the loop is bounded by the iteration limit.
///
/// **Satisfies:** RON-FR-731, RON-FR-733, RON-FR-739, RON-FR-756
///
/// # Errors
///
/// Returns [`RonError::ConfigInvalid`] for an invalid problem and
/// [`RonError::Numerical`] when `R + B^T P B` is not positive definite, an
/// iterate is not finite, or the iteration limit is reached first.
pub fn solve_dare<const N: usize, const U: usize>(
    config: &DareConfig<N, U>,
) -> Result<DareSolution<N, U>, RonError> {
    config.validate()?;
    let limit = if config.max_iterations == 0 {
        DARE_DEFAULT_MAX_ITERATIONS
    } else {
        config.max_iterations
    };
    let (a, b) = (&config.a, &config.b);
    let mut p = config.q;
    for _ in 0..limit {
        let btp = b.transpose().mul(&p);
        let factor = config
            .r
            .add(&btp.mul(b))
            .cholesky()
            .ok_or(RonError::Numerical("R + B^T P B is not positive definite"))?;
        // Solve (R + B^T P B) K = B^T P A column by column.
        let rhs = btp.mul(a).transpose();
        let mut columns = *rhs.rows();
        for column in &mut columns {
            *column = factor.solve(column);
        }
        let k = Matrix::<N, U>::new(columns).transpose();

        let atp = a.transpose().mul(&p);
        let next = config.q.add(&atp.mul(a)).sub(&atp.mul(b).mul(&k));
        if !next.is_finite() {
            return Err(RonError::Numerical("DARE iterate is not finite"));
        }
        if max_abs_difference(&next, &p) < config.tolerance {
            return Ok(DareSolution { k, p: next });
        }
        p = next;
    }
    Err(RonError::Numerical("DARE did not converge"))
}

fn max_abs_difference<const N: usize>(lhs: &Matrix<N, N>, rhs: &Matrix<N, N>) -> RonFloat {
    lhs.rows()
        .iter()
        .flatten()
        .zip(rhs.rows().iter().flatten())
        .fold(0.0, |max, (a, b)| max.max(abs(a - b)))
}

/// Gain source.
///
/// **Satisfies:** RON-FR-732
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LqrGain<const N: usize, const U: usize> {
    /// Caller-supplied gain; no DARE solve.
    Precomputed(Matrix<U, N>),
    /// Gain solved from the DARE at construction.
    Dare(DareConfig<N, U>),
}

/// Per-input integral augmentation: input `j` adds the clamped integral of
/// `r[j] - output_rows[j] x_hat` scaled by `gains[j]`.
///
/// **Satisfies:** RON-FR-735
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqrIntegral<const N: usize, const U: usize> {
    /// Per-input integral gains.
    pub gains: [RonFloat; U],
    /// Regulated-output rows, one per input.
    pub output_rows: Matrix<U, N>,
    /// Per-input lower clamps.
    pub min: [RonFloat; U],
    /// Per-input upper clamps; each `>=` its `min`.
    pub max: [RonFloat; U],
}

/// LQR configuration for `N` states, `U` inputs and an estimator with `Y`
/// outputs/measurements (whose input is the control vector).
///
/// **Satisfies:** RON-FR-730-RON-FR-737
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqrConfig<const N: usize, const U: usize, const Y: usize> {
    /// State-estimate source.
    pub estimator: EstimatorConfig<N, Y, U>,
    /// Gain source.
    pub gain: LqrGain<N, U>,
    /// Reference pre-gain per input.
    pub kr: [RonFloat; U],
    /// Optional integral augmentation.
    pub integral: Option<LqrIntegral<N, U>>,
    /// Per-input output limits.
    pub limits: [OutputLimits; U],
}

/// Linear quadratic regulator.
///
/// **Satisfies:** RON-FR-730
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lqr<const N: usize, const U: usize, const Y: usize> {
    estimator: Estimator<N, Y, U>,
    k: Matrix<U, N>,
    kr: [RonFloat; U],
    dare_solution: Option<Matrix<N, N>>,
    integral_config: Option<LqrIntegral<N, U>>,
    limits: [OutputLimits; U],
    integral: [RonFloat; U],
    output_prev: [RonFloat; U],
}

impl<const N: usize, const U: usize, const Y: usize> Lqr<N, U, Y> {
    /// Validates the configuration, resolves the gain (solving the DARE if
    /// requested) and creates the estimator.
    ///
    /// **Satisfies:** RON-FR-730, RON-FR-732-RON-FR-734, RON-FR-737
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] for an invalid configuration and
    /// the solver's error when the DARE does not converge.
    pub fn new(config: LqrConfig<N, U, Y>) -> Result<Self, RonError> {
        if !dimension_valid(N, false) || !dimension_valid(U, false) {
            return Err(RonError::ConfigInvalid("LQR dimensions out of range"));
        }
        if !vector_is_finite(&config.kr) {
            return Err(RonError::ConfigInvalid("reference gain must be finite"));
        }
        for limits in &config.limits {
            limits.validate()?;
        }
        if let Some(integral) = &config.integral {
            validate_integral(integral)?;
        }
        let (k, dare_solution) = match &config.gain {
            LqrGain::Precomputed(k) => {
                if !k.is_finite() {
                    return Err(RonError::ConfigInvalid("LQR gain must be finite"));
                }
                (*k, None)
            }
            LqrGain::Dare(dare) => {
                let solution = solve_dare(dare)?;
                (solution.k, Some(solution.p))
            }
        };
        Ok(Self {
            estimator: Estimator::new(config.estimator)?,
            k,
            kr: config.kr,
            dare_solution,
            integral_config: config.integral,
            limits: config.limits,
            integral: [0.0; U],
            output_prev: [0.0; U],
        })
    }

    /// Computes one control vector from the current estimate: feedback,
    /// reference, optional integral, then per-input saturation and rate
    /// limiting. The estimate is not advanced here.
    ///
    /// **Satisfies:** RON-FR-730, RON-FR-735, RON-FR-736
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `r` is not finite or `dt` is
    /// not positive and finite, and [`RonError::Numerical`] when the control
    /// law overflows; the controller state is unchanged on error.
    pub fn step(
        &mut self,
        r: &[RonFloat; U],
        dt: RonFloat,
    ) -> Result<([RonFloat; U], PidStatus), RonError> {
        if !vector_is_finite(r) || !is_finite(dt) || dt <= 0.0 {
            return Err(RonError::InvalidArgument(
                "reference must be finite and dt positive",
            ));
        }
        let x_hat = self.estimator.state();
        let feedback = self.k.mul_vec(&x_hat);
        let mut raw = [0.0; U];
        for ((value, fb), (gain, reference)) in raw
            .iter_mut()
            .zip(feedback.iter())
            .zip(self.kr.iter().zip(r.iter()))
        {
            *value = -fb + (gain * reference);
        }
        let mut integral = self.integral;
        if let Some(augmentation) = &self.integral_config {
            let regulated = augmentation.output_rows.mul_vec(&x_hat);
            for j in 0..U {
                integral[j] = clamp(
                    integral[j] + (augmentation.gains[j] * dt * (r[j] - regulated[j])),
                    augmentation.min[j],
                    augmentation.max[j],
                );
                raw[j] += integral[j];
            }
        }
        if !vector_is_finite(&raw) {
            return Err(RonError::Numerical("LQR output is not finite"));
        }
        let mut output = [0.0; U];
        let mut status = PidStatus::OK;
        for j in 0..U {
            let (value, input_status) = self.limits[j].apply(raw[j], self.output_prev[j], dt);
            output[j] = value;
            status |= input_status;
        }
        self.integral = integral;
        self.output_prev = output;
        Ok((output, status))
    }

    /// Replaces `K` and `K_r`, bypassing the DARE: the mechanism for
    /// gain-scheduling an LQR with offline gains. Integral and output
    /// history are kept.
    ///
    /// **Satisfies:** RON-FR-738
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when a gain is not finite; the
    /// gains are then unchanged.
    pub fn set_gains(&mut self, k: &Matrix<U, N>, kr: &[RonFloat; U]) -> Result<(), RonError> {
        if !k.is_finite() || !vector_is_finite(kr) {
            return Err(RonError::ConfigInvalid("LQR gains must be finite"));
        }
        self.k = *k;
        self.kr = *kr;
        Ok(())
    }

    /// Clears the integral and output history and resets the estimator. The
    /// solved gain is kept.
    ///
    /// **Satisfies:** RON-FR-735, RON-FR-736
    pub fn reset(&mut self) {
        self.integral = [0.0; U];
        self.output_prev = [0.0; U];
        self.estimator.reset();
    }

    /// Returns the active feedback gain.
    #[must_use]
    pub const fn gain(&self) -> &Matrix<U, N> {
        &self.k
    }

    /// Returns the DARE solution `P`, or `None` for a pre-computed gain.
    ///
    /// **Satisfies:** RON-FR-739
    #[must_use]
    pub const fn dare_solution(&self) -> Option<&Matrix<N, N>> {
        self.dare_solution.as_ref()
    }

    /// Returns the per-input integral accumulators.
    #[must_use]
    pub const fn integral(&self) -> [RonFloat; U] {
        self.integral
    }

    /// Returns the estimator.
    #[must_use]
    pub const fn estimator(&self) -> &Estimator<N, Y, U> {
        &self.estimator
    }

    /// Returns the estimator, to advance it each cycle.
    ///
    /// **Satisfies:** RON-FR-734
    pub fn estimator_mut(&mut self) -> &mut Estimator<N, Y, U> {
        &mut self.estimator
    }
}

/// **Satisfies:** RON-FR-735
fn validate_integral<const N: usize, const U: usize>(
    integral: &LqrIntegral<N, U>,
) -> Result<(), RonError> {
    if !vector_is_finite(&integral.gains)
        || !vector_is_finite(&integral.min)
        || !vector_is_finite(&integral.max)
        || !integral.output_rows.is_finite()
    {
        return Err(RonError::ConfigInvalid(
            "integral augmentation must be finite",
        ));
    }
    if integral
        .min
        .iter()
        .zip(integral.max.iter())
        .any(|(min, max)| min > max)
    {
        return Err(RonError::ConfigInvalid("integral minimum above maximum"));
    }
    Ok(())
}
