//! # `estimator::tests`
//!
//! Traceable tests for the shared state-estimator component.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-701, RON-FR-734
//! **Tests:** RON-TC-EST-001-RON-TC-EST-003
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Estimator, EstimatorConfig, EstimatorSource};
use crate::{kalman::KalmanConfig, matrix::Matrix, observer::ObserverConfig, RonError, RonFloat};

fn luenberger() -> ObserverConfig<2, 1, 0> {
    ObserverConfig {
        a: Matrix::identity(),
        b: Matrix::new([[], []]),
        c: Matrix::new([[1.0, 0.0]]),
        l: Matrix::new([[0.5], [0.0]]),
        x0: [1.0, 2.0],
    }
}

fn kalman() -> KalmanConfig<2, 1, 0> {
    KalmanConfig {
        a: Matrix::identity(),
        b: Matrix::new([[], []]),
        h: Matrix::new([[1.0, 0.0]]),
        q: Matrix::zeros(),
        r: Matrix::new([[1.0]]),
        x0: [1.0, 2.0],
        p0: Matrix::identity(),
        joseph_form: false,
        steady_state_gain: None,
    }
}

fn all() -> [Estimator<2, 1, 0>; 3] {
    [
        Estimator::new(EstimatorConfig::External([3.0, 4.0])).unwrap(),
        Estimator::new(EstimatorConfig::Luenberger(luenberger())).unwrap(),
        Estimator::new(EstimatorConfig::Kalman(kalman())).unwrap(),
    ]
}

/// RON-TC-EST-001 | RON-FR-701, RON-FR-734
#[test]
fn ron_tc_est_001() {
    let [mut external, mut observer, mut filter] = all();
    assert_eq!(external.source(), EstimatorSource::External);
    assert_eq!(observer.source(), EstimatorSource::Luenberger);
    assert_eq!(filter.source(), EstimatorSource::Kalman);
    assert!(observer.observer().is_some() && observer.kalman().is_none());
    assert!(filter.kalman().is_some() && filter.observer().is_none());

    let mut bad_observer = luenberger();
    bad_observer.a = Matrix::new([[RonFloat::NAN, 0.0], [0.0, 1.0]]);
    assert!(matches!(
        Estimator::new(EstimatorConfig::Luenberger(bad_observer)),
        Err(RonError::ConfigInvalid(_))
    ));
    let mut bad_filter = kalman();
    bad_filter.r = Matrix::new([[RonFloat::NAN]]);
    assert!(Estimator::new(EstimatorConfig::Kalman(bad_filter)).is_err());
    assert!(Estimator::<2, 1, 0>::new(EstimatorConfig::External([0.0, RonFloat::NAN])).is_err());

    observer.observer_step(&[5.0], &[]).unwrap();
    observer.reset();
    assert_eq!(observer.state(), [1.0, 2.0]);
    filter.kalman_update(Some(&[5.0])).unwrap();
    filter.reset();
    assert_eq!(filter.state(), [1.0, 2.0]);
    external.set_external(&[7.0, 8.0]).unwrap();
    external.reset();
    assert_eq!(external.state(), [3.0, 4.0]);
}

/// RON-TC-EST-002 | RON-FR-701, RON-FR-734
#[test]
fn ron_tc_est_002() {
    let [mut external, mut observer, mut filter] = all();
    observer.observer_step(&[5.0], &[]).unwrap();
    assert!(observer.state()[0] > 1.0);
    filter.kalman_predict(&[]).unwrap();
    filter.kalman_update(Some(&[5.0])).unwrap();
    assert!(filter.state()[0] > 1.0);
    assert!(matches!(
        observer.observer_step(&[RonFloat::NAN], &[]),
        Err(RonError::InvalidArgument(_))
    ));

    let wrong = |result: Result<(), RonError>| {
        assert!(matches!(result, Err(RonError::ConfigInvalid(_))));
    };
    wrong(filter.observer_step(&[5.0], &[]));
    wrong(external.observer_step(&[5.0], &[]));
    wrong(observer.kalman_predict(&[]));
    wrong(external.kalman_predict(&[]));
    wrong(observer.kalman_update(Some(&[5.0])));
    wrong(external.kalman_update(Some(&[5.0])));
    wrong(observer.set_external(&[0.0, 0.0]));
    wrong(filter.set_external(&[0.0, 0.0]));
}

/// RON-TC-EST-003 | RON-FR-701, RON-FR-734
#[test]
fn ron_tc_est_003() {
    let [mut external, observer, filter] = all();
    assert_eq!(external.state(), [3.0, 4.0]);
    assert_eq!(observer.state(), observer.observer().unwrap().state());
    assert_eq!(filter.state(), filter.kalman().unwrap().state());
    external.set_external(&[5.0, 6.0]).unwrap();
    assert_eq!(external.state(), [5.0, 6.0]);
    assert!(matches!(
        external.set_external(&[3.0, RonFloat::NAN]),
        Err(RonError::InvalidArgument(_))
    ));
    assert_eq!(external.state(), [5.0, 6.0]);
}
