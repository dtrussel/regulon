//! # `lqg`
//!
//! Discrete-time MIMO linear quadratic Gaussian controller: an LQR control
//! law driven by an embedded Kalman estimate, designed independently by the
//! separation principle. The estimator is always the Kalman filter;
//! substituting another estimator is what makes a design no longer LQG.
//!
//! Heap freedom (RON-TC-LQG-010-FV) holds by construction: the crate is
//! `no_std` and never links `alloc`.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-750-RON-FR-759
//! **Tests:** RON-TC-LQG-001-RON-TC-LQG-009, RON-TC-LQG-010-FV
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    kalman::{Kalman, KalmanConfig},
    lqr::{solve_dare, DareConfig},
    matrix::{dimension_valid, vector_is_finite, Matrix},
    pid::PidStatus,
    platform::{is_finite, RonFloat},
    statespace::OutputLimits,
};

/// LQR gain source; the DARE uses the shared `A` and `B`.
///
/// **Satisfies:** RON-FR-756
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LqgGain<const N: usize, const U: usize> {
    /// Caller-supplied feedback gain; no DARE solve.
    Precomputed(Matrix<U, N>),
    /// Gain solved from the DARE at construction.
    Dare {
        /// State cost (positive semi-definite).
        q_cost: Matrix<N, N>,
        /// Input cost (positive definite).
        r_cost: Matrix<U, U>,
        /// Iteration limit; 0 selects the default.
        max_iterations: u16,
        /// Convergence tolerance; positive.
        tolerance: RonFloat,
    },
}

/// LQG configuration for `N` states, `U` inputs and `Y` measurements.
///
/// **Satisfies:** RON-FR-750, RON-FR-751, RON-FR-756, RON-FR-757
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LqgConfig<const N: usize, const U: usize, const Y: usize> {
    /// State transition matrix, shared by the estimator and the DARE.
    pub a: Matrix<N, N>,
    /// Input matrix, shared by the estimator and the DARE.
    pub b: Matrix<N, U>,
    /// Observation matrix.
    pub h: Matrix<Y, N>,
    /// Process-noise covariance.
    pub q_noise: Matrix<N, N>,
    /// Measurement-noise covariance.
    pub r_noise: Matrix<Y, Y>,
    /// Initial estimate.
    pub x0: [RonFloat; N],
    /// Initial covariance.
    pub p0: Matrix<N, N>,
    /// Use the Joseph-form covariance update.
    pub joseph_form: bool,
    /// Fixed steady-state Kalman gain; `None` computes it every update.
    pub kalman_steady_state_gain: Option<Matrix<N, Y>>,
    /// LQR gain source.
    pub gain: LqgGain<N, U>,
    /// Reference pre-gain per input.
    pub kr: [RonFloat; U],
    /// Per-input output limits.
    pub limits: [OutputLimits; U],
}

/// Linear quadratic Gaussian controller.
///
/// **Satisfies:** RON-FR-750, RON-FR-759
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lqg<const N: usize, const U: usize, const Y: usize> {
    kalman: Kalman<N, Y, U>,
    k: Matrix<U, N>,
    kr: [RonFloat; U],
    dare_solution: Option<Matrix<N, N>>,
    limits: [OutputLimits; U],
    output_prev: [RonFloat; U],
}

impl<const N: usize, const U: usize, const Y: usize> Lqg<N, U, Y> {
    /// Validates the configuration, resolves the LQR gain (solving the
    /// control DARE if requested) and creates the Kalman filter from the
    /// noise model; the two designs are independent.
    ///
    /// **Satisfies:** RON-FR-750-RON-FR-752, RON-FR-756
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] for an invalid configuration and
    /// the solver's error when the DARE does not converge.
    pub fn new(config: LqgConfig<N, U, Y>) -> Result<Self, RonError> {
        if !dimension_valid(N, false) || !dimension_valid(U, false) || !dimension_valid(Y, false) {
            return Err(RonError::ConfigInvalid("LQG dimensions out of range"));
        }
        if !config.a.is_finite() || !config.b.is_finite() {
            return Err(RonError::ConfigInvalid(
                "LQG system matrices must be finite",
            ));
        }
        if !vector_is_finite(&config.kr) {
            return Err(RonError::ConfigInvalid("reference gain must be finite"));
        }
        for limits in &config.limits {
            limits.validate()?;
        }
        let (k, dare_solution) = match config.gain {
            LqgGain::Precomputed(k) => {
                if !k.is_finite() {
                    return Err(RonError::ConfigInvalid("LQG gain must be finite"));
                }
                (k, None)
            }
            LqgGain::Dare {
                q_cost,
                r_cost,
                max_iterations,
                tolerance,
            } => {
                let solution = solve_dare(&DareConfig {
                    a: config.a,
                    b: config.b,
                    q: q_cost,
                    r: r_cost,
                    max_iterations,
                    tolerance,
                })?;
                (solution.k, Some(solution.p))
            }
        };
        let kalman = Kalman::new(KalmanConfig {
            a: config.a,
            b: config.b,
            h: config.h,
            q: config.q_noise,
            r: config.r_noise,
            x0: config.x0,
            p0: config.p0,
            joseph_form: config.joseph_form,
            steady_state_gain: config.kalman_steady_state_gain,
        })?;
        Ok(Self {
            kalman,
            k,
            kr: config.kr,
            dare_solution,
            limits: config.limits,
            output_prev: [0.0; U],
        })
    }

    /// Estimator time update; pass the control vector from the previous
    /// [`Lqg::step`].
    ///
    /// **Satisfies:** RON-FR-753
    ///
    /// # Errors
    ///
    /// Returns the Kalman filter's error.
    pub fn predict(&mut self, u: &[RonFloat; U]) -> Result<(), RonError> {
        self.kalman.predict(u)
    }

    /// Estimator measurement update; `None` is a dropout.
    ///
    /// **Satisfies:** RON-FR-754
    ///
    /// # Errors
    ///
    /// Returns the Kalman filter's error.
    pub fn update(&mut self, z: Option<&[RonFloat; Y]>) -> Result<(), RonError> {
        self.kalman.update(z)
    }

    /// Computes `u = -K x_hat + K_r r` from the current estimate, then
    /// per-input saturation and rate limiting. Call [`Lqg::predict`] and
    /// [`Lqg::update`] first.
    ///
    /// **Satisfies:** RON-FR-755, RON-FR-757
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
        let feedback = self.k.mul_vec(&self.kalman.state());
        let mut raw = [0.0; U];
        for ((value, fb), (gain, reference)) in raw
            .iter_mut()
            .zip(feedback.iter())
            .zip(self.kr.iter().zip(r.iter()))
        {
            *value = -fb + (gain * reference);
        }
        if !vector_is_finite(&raw) {
            return Err(RonError::Numerical("LQG output is not finite"));
        }
        let mut output = [0.0; U];
        let mut status = PidStatus::OK;
        for j in 0..U {
            let (value, input_status) = self.limits[j].apply(raw[j], self.output_prev[j], dt);
            output[j] = value;
            status |= input_status;
        }
        self.output_prev = output;
        Ok((output, status))
    }

    /// Clears the output history and resets the Kalman filter to `x0`/`P0`.
    /// Both designs are kept, so no Riccati solve is repeated.
    ///
    /// **Satisfies:** RON-FR-757
    pub fn reset(&mut self) {
        self.output_prev = [0.0; U];
        self.kalman.reset();
    }

    /// Returns the current state estimate.
    ///
    /// **Satisfies:** RON-FR-758
    #[must_use]
    pub const fn state(&self) -> [RonFloat; N] {
        self.kalman.state()
    }

    /// Returns the LQR gain in use.
    #[must_use]
    pub const fn gain(&self) -> &Matrix<U, N> {
        &self.k
    }

    /// Returns the control DARE solution, or `None` for a pre-computed gain.
    #[must_use]
    pub const fn dare_solution(&self) -> Option<&Matrix<N, N>> {
        self.dare_solution.as_ref()
    }

    /// Returns the embedded Kalman filter.
    #[must_use]
    pub const fn kalman(&self) -> &Kalman<N, Y, U> {
        &self.kalman
    }
}
