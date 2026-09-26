//! # `statespace::tests`
//!
//! Traceable tests for the state-feedback controller.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-700-RON-FR-704, RON-FR-723
//! **Tests:** RON-TC-SS-001-RON-TC-SS-005, RON-TC-SS-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{IntegralAugmentation, OutputLimits, StateSpace, StateSpaceConfig};
use crate::{
    estimator::EstimatorConfig,
    kalman::KalmanConfig,
    matrix::{Matrix, MATRIX_MAX_DIM},
    observer::ObserverConfig,
    pid::PidStatus,
    RonError, RonFloat,
};

const TOL: RonFloat = 1.0e-4;

fn approx_eq(lhs: RonFloat, rhs: RonFloat) {
    assert!((lhs - rhs).abs() <= TOL, "{lhs} != {rhs}");
}

fn limits(min: RonFloat, max: RonFloat, rate_limit: RonFloat) -> OutputLimits {
    OutputLimits {
        min,
        max,
        rate_limit,
    }
}

fn external<const N: usize>(x: [RonFloat; N], k: [RonFloat; N]) -> StateSpaceConfig<N, 1, 0> {
    StateSpaceConfig {
        estimator: EstimatorConfig::External(x),
        k,
        kr: 0.0,
        integral: None,
        limits: limits(-1_000.0, 1_000.0, 0.0),
    }
}

/// RON-TC-SS-001 | RON-FR-700
#[test]
fn ron_tc_ss_001() {
    let mut controller = StateSpace::new(StateSpaceConfig {
        kr: 1.0,
        ..external([3.0, 4.0], [2.0, 1.0])
    })
    .unwrap();
    let (output, status) = controller.step(5.0, 0.01).unwrap();
    approx_eq(output, -5.0);
    assert_eq!(status, PidStatus::OK);
}

/// RON-TC-SS-002 | RON-FR-701
#[test]
fn ron_tc_ss_002() {
    let observer = ObserverConfig::<2, 1, 0> {
        a: Matrix::identity(),
        b: Matrix::new([[], []]),
        c: Matrix::new([[1.0, 0.0]]),
        l: Matrix::zeros(),
        x0: [1.0, 2.0],
    };
    let kalman = KalmanConfig::<2, 1, 0> {
        a: Matrix::identity(),
        b: Matrix::new([[], []]),
        h: Matrix::new([[1.0, 0.0]]),
        q: Matrix::zeros(),
        r: Matrix::new([[1.0]]),
        x0: [1.0, 2.0],
        p0: Matrix::identity(),
        joseph_form: false,
        steady_state_gain: None,
    };
    let base = external([1.0, 2.0], [2.0, 1.0]);
    let mut controllers = [
        StateSpace::new(base).unwrap(),
        StateSpace::new(StateSpaceConfig {
            estimator: EstimatorConfig::Luenberger(observer),
            ..base
        })
        .unwrap(),
        StateSpace::new(StateSpaceConfig {
            estimator: EstimatorConfig::Kalman(kalman),
            ..base
        })
        .unwrap(),
    ];
    for controller in &mut controllers {
        approx_eq(controller.step(0.0, 0.01).unwrap().0, -4.0);
    }
    let [external, luenberger, filter] = &mut controllers;
    luenberger
        .estimator_mut()
        .observer_step(&[1.0], &[])
        .unwrap();
    approx_eq(luenberger.step(0.0, 0.01).unwrap().0, -4.0);
    filter.estimator_mut().kalman_predict(&[]).unwrap();
    filter.estimator_mut().kalman_update(Some(&[1.0])).unwrap();
    filter.reset();
    assert_eq!(filter.estimator().state(), [1.0, 2.0]);
    assert!(filter.estimator_mut().observer_step(&[1.0], &[]).is_err());
    assert!(luenberger.estimator_mut().kalman_predict(&[]).is_err());
    assert!(external.estimator_mut().observer_step(&[1.0], &[]).is_err());
    external.estimator_mut().set_external(&[0.0, 0.0]).unwrap();
    approx_eq(external.step(0.0, 0.01).unwrap().0, 0.0);
}

/// RON-TC-SS-003 | RON-FR-702
#[test]
fn ron_tc_ss_003() {
    let augmented = |max: RonFloat| StateSpaceConfig {
        integral: Some(IntegralAugmentation {
            gain: 1.0,
            output_row: [1.0],
            min: -100.0,
            max,
        }),
        ..external([0.0], [0.0])
    };
    let mut controller = StateSpace::new(augmented(100.0)).unwrap();
    approx_eq(controller.step(2.0, 0.5).unwrap().0, 1.0);
    approx_eq(controller.step(2.0, 0.5).unwrap().0, 2.0);
    controller.reset();
    approx_eq(controller.step(2.0, 0.5).unwrap().0, 1.0);

    let mut clamped = StateSpace::new(augmented(1.5)).unwrap();
    approx_eq(clamped.step(2.0, 0.5).unwrap().0, 1.0);
    approx_eq(clamped.step(2.0, 0.5).unwrap().0, 1.5);
}

/// RON-TC-SS-004 | RON-FR-703
#[test]
fn ron_tc_ss_004() {
    let mut saturating = StateSpace::new(StateSpaceConfig {
        limits: limits(-2.0, 2.0, 0.0),
        ..external([0.0], [1.0])
    })
    .unwrap();
    for (x, expected, saturated) in [(10.0, -2.0, true), (-10.0, 2.0, true), (0.5, -0.5, false)] {
        saturating.estimator_mut().set_external(&[x]).unwrap();
        let (output, status) = saturating.step(0.0, 1.0).unwrap();
        approx_eq(output, expected);
        assert_eq!(status.contains(PidStatus::SATURATED), saturated);
    }

    let mut limited = StateSpace::new(StateSpaceConfig {
        limits: limits(-100.0, 100.0, 1.0),
        ..external([0.0], [1.0])
    })
    .unwrap();
    for (x, expected, rate_limited) in [
        (-5.0, 1.0, true),
        (-5.0, 2.0, true),
        (-2.5, 2.5, false),
        (10.0, 1.5, true),
    ] {
        limited.estimator_mut().set_external(&[x]).unwrap();
        let (output, status) = limited.step(0.0, 1.0).unwrap();
        approx_eq(output, expected);
        assert_eq!(status.contains(PidStatus::RATE_LIMITED), rate_limited);
    }
}

/// RON-TC-SS-005 | RON-FR-704
#[test]
fn ron_tc_ss_005() {
    let mut controller = StateSpace::new(external([3.0, 4.0], [1.0, 0.0])).unwrap();
    approx_eq(controller.step(0.0, 0.01).unwrap().0, -3.0);
    controller.set_gains(&[0.0, 2.0], 1.0).unwrap();
    approx_eq(controller.step(1.0, 0.01).unwrap().0, -7.0);
    assert!(controller
        .set_gains(&[RonFloat::INFINITY, 0.0], 1.0)
        .is_err());
    assert!(controller
        .set_gains(&[0.0, 2.0], RonFloat::INFINITY)
        .is_err());
    approx_eq(controller.step(1.0, 0.01).unwrap().0, -7.0);
}

/// RON-TC-SS-009 | RON-FR-723
#[test]
fn ron_tc_ss_009_validation() {
    let base = external([0.0], [1.0]);
    let augmented = IntegralAugmentation {
        gain: 1.0,
        output_row: [1.0],
        min: -1.0,
        max: 1.0,
    };
    let invalid = [
        StateSpaceConfig {
            k: [RonFloat::INFINITY],
            ..base
        },
        StateSpaceConfig {
            kr: RonFloat::NAN,
            ..base
        },
        StateSpaceConfig {
            limits: limits(RonFloat::INFINITY, 1.0, 0.0),
            ..base
        },
        StateSpaceConfig {
            limits: limits(-1.0, RonFloat::INFINITY, 0.0),
            ..base
        },
        StateSpaceConfig {
            limits: limits(-1.0, 1.0, RonFloat::NAN),
            ..base
        },
        StateSpaceConfig {
            limits: limits(5.0, 1.0, 0.0),
            ..base
        },
        StateSpaceConfig {
            integral: Some(IntegralAugmentation {
                gain: RonFloat::INFINITY,
                ..augmented
            }),
            ..base
        },
        StateSpaceConfig {
            integral: Some(IntegralAugmentation {
                min: RonFloat::INFINITY,
                ..augmented
            }),
            ..base
        },
        StateSpaceConfig {
            integral: Some(IntegralAugmentation {
                min: 2.0,
                max: 1.0,
                ..augmented
            }),
            ..base
        },
        StateSpaceConfig {
            integral: Some(IntegralAugmentation {
                output_row: [RonFloat::NAN],
                ..augmented
            }),
            ..base
        },
        StateSpaceConfig {
            estimator: EstimatorConfig::External([RonFloat::INFINITY]),
            ..base
        },
    ];
    for config in invalid {
        assert!(matches!(
            StateSpace::new(config),
            Err(RonError::ConfigInvalid(_))
        ));
    }
    assert!(StateSpace::new(StateSpaceConfig::<0, 1, 0> {
        estimator: EstimatorConfig::External([]),
        k: [],
        kr: 0.0,
        integral: None,
        limits: limits(-1.0, 1.0, 0.0),
    })
    .is_err());
}

/// RON-TC-SS-009 | RON-FR-723
#[test]
fn ron_tc_ss_009_runtime() {
    let mut largest =
        StateSpace::new(external([1.0; MATRIX_MAX_DIM], [0.5; MATRIX_MAX_DIM])).unwrap();
    assert!(largest.step(0.0, 0.01).is_ok());
    for (r, dt) in [(RonFloat::INFINITY, 0.01), (0.0, RonFloat::NAN), (0.0, 0.0)] {
        assert!(matches!(
            largest.step(r, dt),
            Err(RonError::InvalidArgument(_))
        ));
    }

    let mut overflowing = StateSpace::new(StateSpaceConfig {
        limits: limits(-RonFloat::MAX, RonFloat::MAX, 0.0),
        ..external([RonFloat::MAX], [RonFloat::MAX])
    })
    .unwrap();
    assert!(matches!(
        overflowing.step(0.0, 0.1),
        Err(RonError::Numerical(_))
    ));
}
