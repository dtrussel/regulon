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
//! **Satisfies:** RON-FR-750-RON-FR-759, RON-SR-012, RON-SR-013
//! **Tests:** RON-TC-LQG-001-RON-TC-LQG-009, RON-TC-LQG-010-FV, RON-TC-LQG-011
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    kalman::{Kalman, KalmanConfig},
    lqr::{solve_dare, DareConfig, DareSolution},
    matrix::{dimension_valid, vector_is_finite, Matrix},
    pid::{PidFault, PidStatus},
    platform::{is_finite, RonFloat},
    statespace::OutputLimits,
};

/// Gain source. The control DARE uses the shared `A` and `B`; in
/// [`LqgGain::DareBoth`] the estimator DARE uses `A`, `H` and the noise model.
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
    /// LQR gain and steady-state Kalman gain both solved at construction;
    /// [`LqgConfig::kalman_steady_state_gain`] is ignored. Both DAREs use the
    /// same iteration limit and tolerance.
    DareBoth {
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
    /// Ignored (the gain is solved) with [`LqgGain::DareBoth`].
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
    fault: PidFault,
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
        let mut steady_state_gain = config.kalman_steady_state_gain;
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
                let solution =
                    solve_control_dare(&config, q_cost, r_cost, max_iterations, tolerance)?;
                (solution.k, Some(solution.p))
            }
            LqgGain::DareBoth {
                q_cost,
                r_cost,
                max_iterations,
                tolerance,
            } => {
                let solution =
                    solve_control_dare(&config, q_cost, r_cost, max_iterations, tolerance)?;
                steady_state_gain = Some(solve_kalman_gain(&config, max_iterations, tolerance)?);
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
            steady_state_gain,
        })?;
        Ok(Self {
            kalman,
            k,
            kr: config.kr,
            dare_solution,
            limits: config.limits,
            output_prev: [0.0; U],
            fault: PidFault::NONE,
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
    /// A runtime fault latches (RON-SR-012): it is OR-ed into [`Self::fault`],
    /// the output history is left unchanged (so [`Self::output`] holds the
    /// last output), and every later step returns the latched fault until
    /// [`Self::clear_fault`] or [`Self::reset`]. A latched fault does not block
    /// [`Lqg::predict`] or [`Lqg::update`].
    ///
    /// **Satisfies:** RON-FR-755, RON-FR-757, RON-SR-010, RON-SR-012,
    /// RON-SR-013
    ///
    /// # Errors
    ///
    /// Returns [`RonError::Fault`] with the latched bits:
    /// [`PidFault::INPUT_NOT_FINITE`] when `r` is not finite or `dt` is not
    /// positive and finite, [`PidFault::OUTPUT_NOT_FINITE`] when the control
    /// law overflows, or the bits latched by an earlier step.
    pub fn step(
        &mut self,
        r: &[RonFloat; U],
        dt: RonFloat,
    ) -> Result<([RonFloat; U], PidStatus), RonError> {
        if self.fault.is_none() {
            match self.evaluate(r, dt) {
                Ok((output, status)) => {
                    self.output_prev = output;
                    return Ok((output, status));
                }
                Err(fault) => self.fault |= fault,
            }
        }
        Err(RonError::Fault(self.fault))
    }

    /// Evaluates one step without changing the controller, returning the
    /// limited outputs and the status word.
    ///
    /// **Satisfies:** RON-FR-755, RON-FR-757
    fn evaluate(
        &self,
        r: &[RonFloat; U],
        dt: RonFloat,
    ) -> Result<([RonFloat; U], PidStatus), PidFault> {
        if !vector_is_finite(r) || !is_finite(dt) || dt <= 0.0 {
            return Err(PidFault::INPUT_NOT_FINITE);
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
            return Err(PidFault::OUTPUT_NOT_FINITE);
        }
        let mut output = [0.0; U];
        let mut status = PidStatus::OK;
        for j in 0..U {
            let (value, input_status) = self.limits[j].apply(raw[j], self.output_prev[j], dt);
            output[j] = value;
            status |= input_status;
        }
        Ok((output, status))
    }

    /// Clears the latched fault register; the output history and Kalman
    /// filter are left as they were when the fault latched.
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

    /// Returns the last committed output vector, which a faulted step holds.
    ///
    /// **Satisfies:** RON-FR-757
    #[must_use]
    pub const fn output(&self) -> [RonFloat; U] {
        self.output_prev
    }

    /// Clears the output history and latched faults and resets the Kalman
    /// filter to `x0`/`P0`. Both designs are kept, so no Riccati solve is
    /// repeated.
    ///
    /// **Satisfies:** RON-FR-757, RON-SR-012
    pub fn reset(&mut self) {
        self.output_prev = [0.0; U];
        self.fault = PidFault::NONE;
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

/// Solves the control DARE in the shared `A` and `B`.
///
/// **Satisfies:** RON-FR-756
fn solve_control_dare<const N: usize, const U: usize, const Y: usize>(
    config: &LqgConfig<N, U, Y>,
    q_cost: Matrix<N, N>,
    r_cost: Matrix<U, U>,
    max_iterations: u16,
    tolerance: RonFloat,
) -> Result<DareSolution<N, U>, RonError> {
    solve_dare(&DareConfig {
        a: config.a,
        b: config.b,
        q: q_cost,
        r: r_cost,
        max_iterations,
        tolerance,
    })
}

/// Steady-state Kalman gain from the dual (estimator) DARE: the a-priori
/// covariance `P` solves the DARE in `(A^T, H^T, Q_noise, R_noise)`, and
/// `K_f = P H^T (H P H^T + R_noise)^-1` is the gain the time-varying filter
/// converges to. The DARE's own gain is the predictor form `A K_f` and is
/// not used.
///
/// **Satisfies:** RON-FR-752, RON-FR-756
///
/// # Errors
///
/// Returns the solver's error, or [`RonError::Numerical`] when
/// `H P H^T + R_noise` is not positive definite.
fn solve_kalman_gain<const N: usize, const U: usize, const Y: usize>(
    config: &LqgConfig<N, U, Y>,
    max_iterations: u16,
    tolerance: RonFloat,
) -> Result<Matrix<N, Y>, RonError> {
    let p = solve_dare(&DareConfig {
        a: config.a.transpose(),
        b: config.h.transpose(),
        q: config.q_noise,
        r: config.r_noise,
        max_iterations,
        tolerance,
    })?
    .p;
    let hp = config.h.mul(&p);
    let factor = hp
        .mul(&config.h.transpose())
        .add(&config.r_noise)
        .cholesky()
        .ok_or(RonError::Numerical(
            "H P H^T + R_noise is not positive definite",
        ))?;
    // Row i of K_f solves S k = (H P)[:, i] (S and P are symmetric).
    let mut rows = *hp.transpose().rows();
    for row in &mut rows {
        *row = factor.solve(row);
    }
    Ok(Matrix::new(rows))
}
