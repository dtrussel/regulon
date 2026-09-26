//! # `observer`
//!
//! Discrete-time Luenberger state observer
//! `x_hat(k+1) = A x_hat(k) + B u(k) + L (y(k) - C x_hat(k))` with a
//! caller-designed gain `L`.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-720-RON-FR-723
//! **Tests:** RON-TC-SS-006-RON-TC-SS-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    matrix::{dimension_valid, vector_add, vector_is_finite, vector_sub, Matrix},
    platform::RonFloat,
};

/// Observer model for `N` states, `M` outputs and `P` inputs (`P` may be 0).
///
/// **Satisfies:** RON-FR-721, RON-FR-723
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ObserverConfig<const N: usize, const M: usize, const P: usize> {
    /// State transition matrix.
    pub a: Matrix<N, N>,
    /// Input matrix.
    pub b: Matrix<N, P>,
    /// Output matrix.
    pub c: Matrix<M, N>,
    /// Observer gain.
    pub l: Matrix<N, M>,
    /// Initial estimate.
    pub x0: [RonFloat; N],
}

impl<const N: usize, const M: usize, const P: usize> ObserverConfig<N, M, P> {
    /// Validates dimensions and finiteness.
    ///
    /// **Satisfies:** RON-FR-721, RON-FR-723
    ///
    /// # Errors
    ///
    /// Returns [`RonError::ConfigInvalid`] when a dimension is out of range or
    /// an entry is not finite.
    pub fn validate(&self) -> Result<(), RonError> {
        if !dimension_valid(N, false) || !dimension_valid(M, false) || !dimension_valid(P, true) {
            return Err(RonError::ConfigInvalid("observer dimensions out of range"));
        }
        if !self.a.is_finite()
            || !self.b.is_finite()
            || !self.c.is_finite()
            || !self.l.is_finite()
            || !vector_is_finite(&self.x0)
        {
            return Err(RonError::ConfigInvalid("observer model must be finite"));
        }
        Ok(())
    }
}

/// Luenberger observer.
///
/// **Satisfies:** RON-FR-720
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Observer<const N: usize, const M: usize, const P: usize> {
    config: ObserverConfig<N, M, P>,
    x_hat: [RonFloat; N],
}

impl<const N: usize, const M: usize, const P: usize> Observer<N, M, P> {
    /// Creates an observer seeded with `x0`.
    ///
    /// **Satisfies:** RON-FR-721, RON-FR-723
    ///
    /// # Errors
    ///
    /// Returns an error when the configuration is invalid.
    pub fn new(config: ObserverConfig<N, M, P>) -> Result<Self, RonError> {
        config.validate()?;
        Ok(Self {
            config,
            x_hat: config.x0,
        })
    }

    /// Restores the estimate to `x0`.
    ///
    /// **Satisfies:** RON-FR-720
    pub fn reset(&mut self) {
        self.x_hat = self.config.x0;
    }

    /// Advances the estimate by one sample.
    ///
    /// `u` must be the input actually applied to the plant on the previous
    /// step; feeding the upcoming input biases the estimate by one sample.
    ///
    /// **Satisfies:** RON-FR-720
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] when `y` or `u` is not finite, and
    /// [`RonError::Numerical`] when the new estimate would not be finite. On
    /// error the estimate is unchanged.
    pub fn step(&mut self, y: &[RonFloat; M], u: &[RonFloat; P]) -> Result<(), RonError> {
        if !vector_is_finite(y) || !vector_is_finite(u) {
            return Err(RonError::InvalidArgument("observer inputs must be finite"));
        }
        let config = &self.config;
        let innovation = vector_sub(y, &config.c.mul_vec(&self.x_hat));
        let next = vector_add(
            &vector_add(&config.a.mul_vec(&self.x_hat), &config.b.mul_vec(u)),
            &config.l.mul_vec(&innovation),
        );
        if !vector_is_finite(&next) {
            return Err(RonError::Numerical("observer estimate is not finite"));
        }
        self.x_hat = next;
        Ok(())
    }

    /// Returns the current state estimate.
    ///
    /// **Satisfies:** RON-FR-722
    #[must_use]
    pub const fn state(&self) -> [RonFloat; N] {
        self.x_hat
    }

    /// Returns the configuration.
    #[must_use]
    pub const fn config(&self) -> &ObserverConfig<N, M, P> {
        &self.config
    }
}
