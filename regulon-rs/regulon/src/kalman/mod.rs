//! # `kalman`
//!
//! Discrete-time linear Kalman filter for `x(k+1) = A x(k) + B u(k) + w(k)`,
//! `z(k) = H x(k) + v(k)` with a predict/update cycle, optional Joseph-form
//! covariance update, steady-state gain mode and measurement dropout.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-600-RON-FR-607
//! **Tests:** RON-TC-KF-001-RON-TC-KF-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    matrix::{dimension_valid, vector_add, vector_is_finite, vector_sub, Matrix},
    platform::RonFloat,
};

/// Kalman filter model for `N` states, `M` measurements and `P` inputs
/// (`P` may be 0).
///
/// **Satisfies:** RON-FR-601, RON-FR-604, RON-FR-606, RON-FR-607
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KalmanConfig<const N: usize, const M: usize, const P: usize> {
    /// State transition matrix.
    pub a: Matrix<N, N>,
    /// Input matrix.
    pub b: Matrix<N, P>,
    /// Measurement matrix.
    pub h: Matrix<M, N>,
    /// Process-noise covariance.
    pub q: Matrix<N, N>,
    /// Measurement-noise covariance.
    pub r: Matrix<M, M>,
    /// Initial estimate.
    pub x0: [RonFloat; N],
    /// Initial covariance.
    pub p0: Matrix<N, N>,
    /// Use the Joseph-form covariance update.
    pub joseph_form: bool,
    /// Fixed steady-state gain; `None` computes the gain every update.
    pub steady_state_gain: Option<Matrix<N, M>>,
}

impl<const N: usize, const M: usize, const P: usize> KalmanConfig<N, M, P> {
    /// Validates dimensions and finiteness.
    ///
    /// **Satisfies:** RON-FR-601, RON-FR-607
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when a dimension is out of range or
    /// an entry is not finite.
    pub fn validate(&self) -> Result<(), RonError> {
        if !dimension_valid(N, false) || !dimension_valid(M, false) || !dimension_valid(P, true) {
            return Err(RonError::ConfigInvalid("Kalman dimensions out of range"));
        }
        let gain_finite = self.steady_state_gain.is_none_or(|gain| gain.is_finite());
        if !self.a.is_finite()
            || !self.b.is_finite()
            || !self.h.is_finite()
            || !self.q.is_finite()
            || !self.r.is_finite()
            || !self.p0.is_finite()
            || !vector_is_finite(&self.x0)
            || !gain_finite
        {
            return Err(RonError::ConfigInvalid("Kalman model must be finite"));
        }
        Ok(())
    }
}

/// Discrete-time linear Kalman filter.
///
/// **Satisfies:** RON-FR-600
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Kalman<const N: usize, const M: usize, const P: usize> {
    config: KalmanConfig<N, M, P>,
    x_hat: [RonFloat; N],
    covariance: Matrix<N, N>,
}

impl<const N: usize, const M: usize, const P: usize> Kalman<N, M, P> {
    /// Creates a filter seeded with `x0` and `P0`.
    ///
    /// **Satisfies:** RON-FR-600, RON-FR-601, RON-FR-607
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: KalmanConfig<N, M, P>) -> Result<Self, RonError> {
        config.validate()?;
        Ok(Self {
            config,
            x_hat: config.x0,
            covariance: config.p0,
        })
    }

    /// Restores the estimate and covariance to `x0` and `P0`.
    ///
    /// **Satisfies:** RON-FR-602
    pub fn reset(&mut self) {
        self.x_hat = self.config.x0;
        self.covariance = self.config.p0;
    }

    /// Time update: `x = A x + B u`, `P = A P A^T + Q`. A predict without an
    /// update is how the filter coasts through a sample with no measurement.
    ///
    /// **Satisfies:** RON-FR-600, RON-FR-602
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `u` is not finite and
    /// [`RonError::Numerical`] when the result would not be finite; the state
    /// is unchanged on error.
    pub fn predict(&mut self, u: &[RonFloat; P]) -> Result<(), RonError> {
        if !vector_is_finite(u) {
            return Err(RonError::InvalidArgument("Kalman input must be finite"));
        }
        let config = &self.config;
        let x_hat = vector_add(&config.a.mul_vec(&self.x_hat), &config.b.mul_vec(u));
        let covariance = config
            .a
            .mul(&self.covariance)
            .mul_transpose(&config.a)
            .add(&config.q);
        self.commit(x_hat, covariance)
    }

    /// Measurement update with `z`, or a dropout when `z` is `None`: the
    /// correction is skipped and the covariance keeps the growth from
    /// predict.
    ///
    /// The gain uses scalar division for `M = 1` and a Cholesky solve for
    /// `M > 1`, never an explicit inverse.
    ///
    /// **Satisfies:** RON-FR-602-RON-FR-606
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `z` is not finite and
    /// [`RonError::Numerical`] when the innovation covariance is not positive
    /// definite or the result would not be finite; the state is unchanged on
    /// error.
    pub fn update(&mut self, z: Option<&[RonFloat; M]>) -> Result<(), RonError> {
        let Some(z) = z else {
            return Ok(());
        };
        if !vector_is_finite(z) {
            return Err(RonError::InvalidArgument(
                "Kalman measurement must be finite",
            ));
        }
        let config = &self.config;
        let gain = match config.steady_state_gain {
            Some(gain) => gain,
            None => self.gain()?,
        };
        let innovation = vector_sub(z, &config.h.mul_vec(&self.x_hat));
        let x_hat = vector_add(&self.x_hat, &gain.mul_vec(&innovation));

        let i_kh = Matrix::<N, N>::identity().sub(&gain.mul(&config.h));
        let mut covariance = i_kh.mul(&self.covariance);
        if config.joseph_form {
            covariance = covariance
                .mul_transpose(&i_kh)
                .add(&gain.mul(&config.r).mul_transpose(&gain));
        }
        self.commit(x_hat, covariance)
    }

    /// Returns the current state estimate.
    ///
    /// **Satisfies:** RON-FR-602
    #[must_use]
    pub const fn state(&self) -> [RonFloat; N] {
        self.x_hat
    }

    /// Returns the current error covariance.
    ///
    /// **Satisfies:** RON-FR-602
    #[must_use]
    pub const fn covariance(&self) -> Matrix<N, N> {
        self.covariance
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> &KalmanConfig<N, M, P> {
        &self.config
    }

    /// `K = P H^T S^-1` with `S = H P H^T + R`.
    ///
    /// **Satisfies:** RON-FR-603
    fn gain(&self) -> Result<Matrix<N, M>, RonError> {
        let config = &self.config;
        let pht = self.covariance.mul_transpose(&config.h);
        let innovation_covariance = config.h.mul(&pht).add(&config.r);
        let not_pd = RonError::Numerical("innovation covariance is not positive definite");
        if M == 1 {
            let s = innovation_covariance.get(0, 0).unwrap_or(0.0);
            if !s.is_finite() || s <= 0.0 {
                return Err(not_pd);
            }
            return Ok(pht.scale(1.0 / s));
        }
        let factor = innovation_covariance.cholesky().ok_or(not_pd)?;
        let mut rows = *pht.rows();
        for row in &mut rows {
            *row = factor.solve(row);
        }
        Ok(Matrix::new(rows))
    }

    fn commit(&mut self, x_hat: [RonFloat; N], covariance: Matrix<N, N>) -> Result<(), RonError> {
        if !vector_is_finite(&x_hat) || !covariance.is_finite() {
            return Err(RonError::Numerical("Kalman state is not finite"));
        }
        self.x_hat = x_hat;
        self.covariance = covariance;
        Ok(())
    }
}
