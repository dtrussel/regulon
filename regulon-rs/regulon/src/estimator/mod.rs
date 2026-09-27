//! # `estimator`
//!
//! State-estimate source shared by the state-space controller and LQR: a
//! caller-supplied vector, an embedded Luenberger observer or an embedded
//! Kalman filter, selected at construction.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-701, RON-FR-734
//! **Tests:** RON-TC-EST-001-RON-TC-EST-003
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
mod tests;

use crate::{
    error::RonError,
    kalman::{Kalman, KalmanConfig},
    matrix::vector_is_finite,
    observer::{Observer, ObserverConfig},
    platform::RonFloat,
};

/// Estimator source and its configuration, for `N` states, `M`
/// outputs/measurements and `P` inputs.
///
/// **Satisfies:** RON-FR-701, RON-FR-734
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EstimatorConfig<const N: usize, const M: usize, const P: usize> {
    /// The caller supplies the estimate through [`Estimator::set_external`];
    /// this is the initial value.
    External([RonFloat; N]),
    /// Embedded Luenberger observer.
    Luenberger(ObserverConfig<N, M, P>),
    /// Embedded Kalman filter.
    Kalman(KalmanConfig<N, M, P>),
}

/// Selected estimator source.
///
/// **Satisfies:** RON-FR-701, RON-FR-734
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimatorSource {
    /// Caller-supplied estimate.
    External,
    /// Embedded Luenberger observer.
    Luenberger,
    /// Embedded Kalman filter.
    Kalman,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Inner<const N: usize, const M: usize, const P: usize> {
    External {
        initial: [RonFloat; N],
        current: [RonFloat; N],
    },
    Luenberger(Observer<N, M, P>),
    Kalman(Kalman<N, M, P>),
}

/// State estimator.
///
/// **Satisfies:** RON-FR-701, RON-FR-734
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimator<const N: usize, const M: usize, const P: usize> {
    inner: Inner<N, M, P>,
}

const WRONG_SOURCE: RonError =
    RonError::ConfigInvalid("operation not valid for this estimator source");

impl<const N: usize, const M: usize, const P: usize> Estimator<N, M, P> {
    /// Creates the selected estimator.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    ///
    /// # Errors
    ///
    /// Returns the embedded component's configuration error, or
    /// [`RonError::ConfigInvalid`] for a non-finite external estimate.
    pub fn new(config: EstimatorConfig<N, M, P>) -> Result<Self, RonError> {
        let inner = match config {
            EstimatorConfig::External(initial) => {
                if !vector_is_finite(&initial) {
                    return Err(RonError::ConfigInvalid("external estimate must be finite"));
                }
                Inner::External {
                    initial,
                    current: initial,
                }
            }
            EstimatorConfig::Luenberger(config) => Inner::Luenberger(Observer::new(config)?),
            EstimatorConfig::Kalman(config) => Inner::Kalman(Kalman::new(config)?),
        };
        Ok(Self { inner })
    }

    /// Returns the selected source.
    #[must_use]
    pub const fn source(&self) -> EstimatorSource {
        match self.inner {
            Inner::External { .. } => EstimatorSource::External,
            Inner::Luenberger(_) => EstimatorSource::Luenberger,
            Inner::Kalman(_) => EstimatorSource::Kalman,
        }
    }

    /// Restores the embedded estimator (or external vector) to its initial
    /// estimate.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    pub fn reset(&mut self) {
        match &mut self.inner {
            Inner::External { initial, current } => *current = *initial,
            Inner::Luenberger(observer) => observer.reset(),
            Inner::Kalman(kalman) => kalman.reset(),
        }
    }

    /// Supplies the estimate for the external source.
    ///
    /// **Satisfies:** RON-FR-701
    ///
    /// # Errors
    ///
    /// Returns [`RonError::InvalidArgument`] for a non-finite estimate and
    /// [`RonError::ConfigInvalid`] for any other source.
    pub fn set_external(&mut self, estimate: &[RonFloat; N]) -> Result<(), RonError> {
        let Inner::External { current, .. } = &mut self.inner else {
            return Err(WRONG_SOURCE);
        };
        if !vector_is_finite(estimate) {
            return Err(RonError::InvalidArgument(
                "external estimate must be finite",
            ));
        }
        *current = *estimate;
        Ok(())
    }

    /// Advances the embedded observer by one sample.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    ///
    /// # Errors
    ///
    /// Returns the observer's error, or [`RonError::ConfigInvalid`] for any
    /// other source.
    pub fn observer_step(&mut self, y: &[RonFloat; M], u: &[RonFloat; P]) -> Result<(), RonError> {
        match &mut self.inner {
            Inner::Luenberger(observer) => observer.step(y, u),
            Inner::External { .. } | Inner::Kalman(_) => Err(WRONG_SOURCE),
        }
    }

    /// Runs the embedded Kalman filter's time update.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    ///
    /// # Errors
    ///
    /// Returns the filter's error, or [`RonError::ConfigInvalid`] for any
    /// other source.
    pub fn kalman_predict(&mut self, u: &[RonFloat; P]) -> Result<(), RonError> {
        match &mut self.inner {
            Inner::Kalman(kalman) => kalman.predict(u),
            Inner::External { .. } | Inner::Luenberger(_) => Err(WRONG_SOURCE),
        }
    }

    /// Runs the embedded Kalman filter's measurement update; `None` is a
    /// dropout.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    ///
    /// # Errors
    ///
    /// Returns the filter's error, or [`RonError::ConfigInvalid`] for any
    /// other source.
    pub fn kalman_update(&mut self, z: Option<&[RonFloat; M]>) -> Result<(), RonError> {
        match &mut self.inner {
            Inner::Kalman(kalman) => kalman.update(z),
            Inner::External { .. } | Inner::Luenberger(_) => Err(WRONG_SOURCE),
        }
    }

    /// Returns the current state estimate. It is always finite: every source
    /// rejects non-finite estimates before storing them.
    ///
    /// **Satisfies:** RON-FR-701, RON-FR-734
    #[must_use]
    pub const fn state(&self) -> [RonFloat; N] {
        match &self.inner {
            Inner::External { current, .. } => *current,
            Inner::Luenberger(observer) => observer.state(),
            Inner::Kalman(kalman) => kalman.state(),
        }
    }

    /// Returns the embedded observer, if that is the source.
    #[must_use]
    pub const fn observer(&self) -> Option<&Observer<N, M, P>> {
        match &self.inner {
            Inner::Luenberger(observer) => Some(observer),
            Inner::External { .. } | Inner::Kalman(_) => None,
        }
    }

    /// Returns the embedded Kalman filter, if that is the source.
    #[must_use]
    pub const fn kalman(&self) -> Option<&Kalman<N, M, P>> {
        match &self.inner {
            Inner::Kalman(kalman) => Some(kalman),
            Inner::External { .. } | Inner::Luenberger(_) => None,
        }
    }
}
