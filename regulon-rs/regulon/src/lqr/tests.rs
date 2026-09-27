//! # `lqr::tests`
//!
//! Traceable tests for the linear quadratic regulator.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-730-RON-FR-739
//! **Tests:** RON-TC-LQR-001-RON-TC-LQR-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{solve_dare, DareConfig, Lqr, LqrConfig, LqrGain, LqrIntegral};
use crate::{
    estimator::EstimatorConfig,
    kalman::KalmanConfig,
    matrix::{Matrix, MATRIX_MAX_DIM},
    observer::ObserverConfig,
    pid::{PidFault, PidStatus, SafePolicy},
    statespace::OutputLimits,
    RonError, RonFloat,
};

const TOL: RonFloat = 1.0e-4;

fn approx_eq(lhs: RonFloat, rhs: RonFloat) {
    assert!((lhs - rhs).abs() <= TOL, "{lhs} != {rhs}");
}

const WIDE: OutputLimits = OutputLimits {
    min: -1_000.0,
    max: 1_000.0,
    rate_limit: 0.0,
};

fn external<const N: usize>(x: [RonFloat; N], k: [RonFloat; N]) -> LqrConfig<N, 1, 1> {
    LqrConfig {
        estimator: EstimatorConfig::External(x),
        gain: LqrGain::Precomputed(Matrix::new([k])),
        kr: [0.0],
        integral: None,
        limits: [WIDE],
        safe_policy: SafePolicy::HoldLast,
        safe_value: [0.0],
    }
}

/// Double integrator `x1 += x2`, `x2 += u` with identity state cost.
fn double_integrator() -> DareConfig<2, 1> {
    DareConfig {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::new([[0.0], [1.0]]),
        q: Matrix::identity(),
        r: Matrix::new([[1.0]]),
        max_iterations: 200,
        tolerance: 1.0e-4,
    }
}

/// RON-TC-LQR-001 | RON-FR-730, RON-FR-732
#[test]
fn ron_tc_lqr_001() {
    let mut lqr = Lqr::new(LqrConfig {
        kr: [1.0],
        ..external([0.5, 0.2], [2.0, 1.0])
    })
    .unwrap();
    assert!(lqr.dare_solution().is_none());
    let (u, status) = lqr.step(&[1.0], 0.01).unwrap();
    approx_eq(u[0], -0.2);
    assert_eq!(status, PidStatus::OK);
}

/// RON-TC-LQR-002 | RON-FR-730, RON-FR-734
#[test]
fn ron_tc_lqr_002() {
    let mut lqr = Lqr::new(external([1.0, 2.0], [2.0, 1.0])).unwrap();
    approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -4.0);
    lqr.estimator_mut().set_external(&[3.0, 4.0]).unwrap();
    approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -10.0);
}

/// RON-TC-LQR-003 | RON-FR-731, RON-FR-733, RON-FR-739
#[test]
fn ron_tc_lqr_003() {
    let mut lqr = Lqr::new(LqrConfig {
        gain: LqrGain::Dare(double_integrator()),
        ..external([5.0, 0.0], [0.0, 0.0])
    })
    .unwrap();
    let p = *lqr.dare_solution().unwrap();
    for (i, row) in p.rows().iter().enumerate() {
        assert!(row[i] >= 0.0);
        for (j, value) in row.iter().enumerate() {
            assert!((value - p.rows()[j][i]).abs() <= 1.0e-3);
        }
    }
    // Riccati fixed point: P = Q + A^T P A - A^T P B K.
    let dare = double_integrator();
    let atp = dare.a.transpose().mul(&p);
    let residual = dare
        .q
        .add(&atp.mul(&dare.a))
        .sub(&atp.mul(&dare.b).mul(lqr.gain()))
        .sub(&p);
    assert!(residual
        .rows()
        .iter()
        .flatten()
        .all(|value| value.abs() < 1.0e-2));

    let mut x = [5.0, 0.0];
    for _ in 0..50 {
        lqr.estimator_mut().set_external(&x).unwrap();
        let u = lqr.step(&[0.0], 1.0).unwrap().0;
        x = [x[0] + x[1], x[1] + u[0]];
    }
    assert!(x[0].abs() < 0.5);
    assert_eq!(solve_dare(&double_integrator()).unwrap().k, *lqr.gain());
}

/// RON-TC-LQR-004 | RON-FR-736
#[test]
fn ron_tc_lqr_004() {
    let mut lqr = Lqr::new(LqrConfig {
        limits: [OutputLimits {
            min: -1.0,
            max: 1.0,
            rate_limit: 0.0,
        }],
        ..external([-5.0, 0.0], [1.0, 0.0])
    })
    .unwrap();
    let (u, status) = lqr.step(&[0.0], 1.0).unwrap();
    approx_eq(u[0], 1.0);
    assert!(status.contains(PidStatus::SATURATED));

    let mut limited = Lqr::new(LqrConfig {
        limits: [OutputLimits {
            rate_limit: 1.0,
            ..WIDE
        }],
        ..external([-100.0], [1.0])
    })
    .unwrap();
    let (u, status) = limited.step(&[0.0], 1.0).unwrap();
    approx_eq(u[0], 1.0);
    assert!(status.contains(PidStatus::RATE_LIMITED));
    limited.estimator_mut().set_external(&[100.0]).unwrap();
    assert!(limited
        .step(&[0.0], 1.0)
        .unwrap()
        .1
        .contains(PidStatus::RATE_LIMITED));
    limited.estimator_mut().set_external(&[0.0]).unwrap();
    assert!(!limited
        .step(&[0.0], 1.0)
        .unwrap()
        .1
        .contains(PidStatus::RATE_LIMITED));
}

/// RON-TC-LQR-005 | RON-FR-738
#[test]
fn ron_tc_lqr_005() {
    let mut lqr = Lqr::new(external([3.0, 4.0], [1.0, 0.0])).unwrap();
    approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -3.0);
    lqr.set_gains(&Matrix::new([[0.0, 2.0]]), &[1.0]).unwrap();
    approx_eq(lqr.step(&[1.0], 0.01).unwrap().0[0], -7.0);
    assert!(lqr
        .set_gains(&Matrix::new([[RonFloat::INFINITY, 0.0]]), &[0.0])
        .is_err());
    assert!(lqr
        .set_gains(&Matrix::new([[0.0, 0.0]]), &[RonFloat::NAN])
        .is_err());
    assert_eq!(*lqr.gain(), Matrix::new([[0.0, 2.0]]));
}

/// RON-TC-LQR-006 | RON-FR-736, RON-SR-001
#[test]
fn ron_tc_lqr_006() {
    let mut lqr = Lqr::new(external([0.0], [0.0])).unwrap();
    for (r, dt) in [
        ([0.0], RonFloat::NAN),
        ([0.0], 0.0),
        ([RonFloat::INFINITY], 0.01),
    ] {
        assert_eq!(
            lqr.step(&r, dt),
            Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
        );
        lqr.clear_fault();
    }
    let mut overflowing = Lqr::new(LqrConfig {
        limits: [OutputLimits {
            min: -RonFloat::MAX,
            max: RonFloat::MAX,
            rate_limit: 0.0,
        }],
        ..external([RonFloat::MAX], [RonFloat::MAX])
    })
    .unwrap();
    assert_eq!(
        overflowing.step(&[0.0], 0.1),
        Err(RonError::Fault(PidFault::OUTPUT_NOT_FINITE))
    );
}

/// RON-TC-LQR-007 | RON-FR-735
#[test]
fn ron_tc_lqr_007() {
    let mut lqr = Lqr::new(LqrConfig {
        integral: Some(LqrIntegral {
            gains: [0.8],
            output_rows: Matrix::new([[1.0]]),
            min: [-100.0],
            max: [100.0],
        }),
        limits: [OutputLimits {
            min: -100.0,
            max: 100.0,
            rate_limit: 0.0,
        }],
        ..external([0.0], [2.0])
    })
    .unwrap();
    let mut x = 0.0;
    for _ in 0..500 {
        lqr.estimator_mut().set_external(&[x]).unwrap();
        let u = lqr.step(&[2.0], 0.05).unwrap().0;
        x += u[0] * 0.05;
        assert!((-100.0..=100.0).contains(&lqr.integral()[0]));
    }
    assert!((2.0 - x).abs() < 0.01);
    lqr.reset();
    assert_eq!(lqr.integral(), [0.0]);
}

/// RON-TC-LQR-008 | RON-FR-734
#[test]
fn ron_tc_lqr_008() {
    let observer = ObserverConfig::<2, 1, 1> {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::new([[0.0], [1.0]]),
        c: Matrix::new([[1.0, 0.0]]),
        l: Matrix::zeros(),
        x0: [0.0, 0.0],
    };
    let mut lqr = Lqr::new(LqrConfig {
        estimator: EstimatorConfig::Luenberger(observer),
        ..external([0.0, 0.0], [0.5, 0.5])
    })
    .unwrap();
    let mut u = [0.0];
    for _ in 0..200 {
        lqr.estimator_mut().observer_step(&[1.0], &u).unwrap();
        u = lqr.step(&[0.0], 0.01).unwrap().0;
        assert!((-1_000.0..=1_000.0).contains(&u[0]));
    }
    assert!(lqr.estimator_mut().kalman_predict(&[0.0]).is_err());
    lqr.reset();
    assert_eq!(lqr.estimator().state(), [0.0, 0.0]);

    let mut bad = observer;
    bad.a = Matrix::new([[RonFloat::INFINITY, 1.0], [0.0, 1.0]]);
    assert!(Lqr::new(LqrConfig {
        estimator: EstimatorConfig::Luenberger(bad),
        ..external([0.0, 0.0], [0.5, 0.5])
    })
    .is_err());
}

/// RON-TC-LQR-009 | RON-FR-734
#[test]
fn ron_tc_lqr_009() {
    let kalman = KalmanConfig::<2, 1, 1> {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::zeros(),
        h: Matrix::new([[1.0, 0.0]]),
        q: Matrix::diagonal([0.01, 0.01]),
        r: Matrix::new([[1.0]]),
        x0: [0.0, 0.0],
        p0: Matrix::identity(),
        joseph_form: false,
        steady_state_gain: None,
    };
    let mut lqr = Lqr::new(LqrConfig {
        estimator: EstimatorConfig::Kalman(kalman),
        ..external([0.0, 0.0], [0.5, 0.5])
    })
    .unwrap();
    for _ in 0..200 {
        lqr.estimator_mut().kalman_predict(&[0.0]).unwrap();
        lqr.estimator_mut().kalman_update(Some(&[1.0])).unwrap();
        let u = lqr.step(&[0.0], 0.01).unwrap().0;
        assert!((-1_000.0..=1_000.0).contains(&u[0]));
    }
    assert!((lqr.estimator().state()[0] - 1.0).abs() < 0.5);
    assert!(lqr.estimator_mut().observer_step(&[1.0], &[0.0]).is_err());
    lqr.reset();

    let mut bad = kalman;
    bad.r = Matrix::new([[RonFloat::INFINITY]]);
    assert!(Lqr::new(LqrConfig {
        estimator: EstimatorConfig::Kalman(bad),
        ..external([0.0, 0.0], [0.5, 0.5])
    })
    .is_err());
}

/// RON-TC-LQR-006 | RON-FR-737
#[test]
fn ron_tc_lqr_006_validation() {
    let base = external([0.0], [0.0]);
    let scalar_dare = DareConfig {
        a: Matrix::new([[1.0]]),
        b: Matrix::new([[1.0]]),
        q: Matrix::new([[1.0]]),
        r: Matrix::new([[1.0]]),
        max_iterations: 0,
        tolerance: 1.0e-6,
    };
    let limits = |min: RonFloat, max: RonFloat, rate_limit: RonFloat| {
        [OutputLimits {
            min,
            max,
            rate_limit,
        }]
    };
    let integral = LqrIntegral {
        gains: [1.0],
        output_rows: Matrix::new([[1.0]]),
        min: [-1.0],
        max: [1.0],
    };
    let invalid = [
        LqrConfig {
            kr: [RonFloat::INFINITY],
            ..base
        },
        LqrConfig {
            gain: LqrGain::Precomputed(Matrix::new([[RonFloat::NAN]])),
            ..base
        },
        LqrConfig {
            gain: LqrGain::Dare(DareConfig {
                tolerance: 0.0,
                ..scalar_dare
            }),
            ..base
        },
        LqrConfig {
            gain: LqrGain::Dare(DareConfig {
                tolerance: RonFloat::NAN,
                ..scalar_dare
            }),
            ..base
        },
        LqrConfig {
            gain: LqrGain::Dare(DareConfig {
                q: Matrix::new([[RonFloat::INFINITY]]),
                ..scalar_dare
            }),
            ..base
        },
        LqrConfig {
            gain: LqrGain::Dare(DareConfig {
                a: Matrix::new([[RonFloat::INFINITY]]),
                ..scalar_dare
            }),
            ..base
        },
        LqrConfig {
            gain: LqrGain::Dare(DareConfig {
                b: Matrix::new([[RonFloat::NAN]]),
                ..scalar_dare
            }),
            ..base
        },
        LqrConfig {
            limits: limits(RonFloat::INFINITY, 1.0, 0.0),
            ..base
        },
        LqrConfig {
            limits: limits(-1.0, RonFloat::NAN, 0.0),
            ..base
        },
        LqrConfig {
            limits: limits(-1.0, 1.0, RonFloat::NAN),
            ..base
        },
        LqrConfig {
            limits: limits(5.0, 1.0, 0.0),
            ..base
        },
        LqrConfig {
            integral: Some(LqrIntegral {
                gains: [RonFloat::INFINITY],
                ..integral
            }),
            ..base
        },
        LqrConfig {
            integral: Some(LqrIntegral {
                min: [2.0],
                max: [1.0],
                ..integral
            }),
            ..base
        },
        LqrConfig {
            integral: Some(LqrIntegral {
                output_rows: Matrix::new([[RonFloat::NAN]]),
                ..integral
            }),
            ..base
        },
    ];
    for config in invalid {
        assert!(matches!(Lqr::new(config), Err(RonError::ConfigInvalid(_))));
    }

    // R + B^T P B not positive definite, and an unreachable tolerance.
    for dare in [
        DareConfig {
            b: Matrix::new([[0.0]]),
            r: Matrix::new([[0.0]]),
            tolerance: 1.0e-8,
            ..scalar_dare
        },
        DareConfig {
            max_iterations: 2,
            tolerance: 1.0e-30,
            ..scalar_dare
        },
    ] {
        assert!(matches!(
            Lqr::new(LqrConfig {
                gain: LqrGain::Dare(dare),
                ..base
            }),
            Err(RonError::Numerical(_))
        ));
    }
    assert!(Lqr::new(LqrConfig {
        gain: LqrGain::Dare(scalar_dare),
        ..base
    })
    .is_ok());

    const N: usize = MATRIX_MAX_DIM;
    let mut largest = Lqr::new(LqrConfig::<N, N, 1> {
        estimator: EstimatorConfig::External([1.0; N]),
        gain: LqrGain::Precomputed(Matrix::new([[0.1; N]; N])),
        kr: [0.0; N],
        integral: None,
        limits: [WIDE; N],
        safe_policy: SafePolicy::HoldLast,
        safe_value: [0.0; N],
    })
    .unwrap();
    assert!(largest.step(&[0.0; N], 0.01).is_ok());
    assert!(Lqr::new(LqrConfig::<0, 1, 1> {
        estimator: EstimatorConfig::External([]),
        gain: LqrGain::Precomputed(Matrix::new([[]])),
        kr: [0.0],
        integral: None,
        limits: [WIDE],
        safe_policy: SafePolicy::HoldLast,
        safe_value: [0.0],
    })
    .is_err());
}

/// RON-TC-LQR-011 | RON-FR-736, RON-SR-010, RON-SR-012, RON-SR-013
#[test]
fn ron_tc_lqr_011() {
    let mut lqr = Lqr::new(external([1.0], [1.0])).unwrap();
    approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -1.0);

    // The fault latches and the last output is held.
    lqr.estimator_mut().set_external(&[2.0]).unwrap();
    let latched = Err(RonError::Fault(PidFault::INPUT_NOT_FINITE));
    assert_eq!(lqr.step(&[RonFloat::NAN], 0.01), latched);
    assert_eq!(lqr.fault(), PidFault::INPUT_NOT_FINITE);
    approx_eq(lqr.output()[0], -1.0);

    // Finite inputs do not clear it.
    assert_eq!(lqr.step(&[0.0], 0.01), latched);
    approx_eq(lqr.output()[0], -1.0);

    // An explicit clear resumes normal stepping.
    lqr.clear_fault();
    let (output, status) = lqr.step(&[0.0], 0.01).unwrap();
    approx_eq(output[0], -2.0);
    assert!(!status.contains(PidStatus::FAULT));

    // Reset clears a latched fault too.
    assert_eq!(lqr.step(&[0.0], RonFloat::NAN), latched);
    lqr.reset();
    assert_eq!(lqr.fault(), PidFault::NONE);
    assert!(lqr.step(&[0.0], 0.01).is_ok());
}

/// RON-TC-LQR-012 | RON-FR-736, RON-SR-011
#[test]
fn ron_tc_lqr_012() {
    let limited = OutputLimits {
        min: -5.0,
        max: 5.0,
        rate_limit: 1.0,
    };
    // du_max = 1/s at dt = 0.01 s: the output moves 0.01 per step.
    for (policy, expected) in [
        (SafePolicy::HoldLast, -0.01),
        (SafePolicy::DriveZero, 0.0),
        (SafePolicy::DriveSafeValue, 5.0),
    ] {
        let mut lqr = Lqr::new(LqrConfig {
            limits: [limited],
            safe_policy: policy,
            safe_value: [9.0], // beyond the limits: clamped
            ..external([1.0], [1.0])
        })
        .unwrap();
        approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -0.01);

        // Latched: the policy output, with the history untouched.
        assert!(lqr.step(&[RonFloat::NAN], 0.01).is_err());
        approx_eq(lqr.output()[0], expected);
        assert!(lqr.step(&[0.0], 0.01).is_err());
        approx_eq(lqr.output()[0], expected);

        // After the clear, rate limiting continues from the last output.
        lqr.clear_fault();
        approx_eq(lqr.step(&[0.0], 0.01).unwrap().0[0], -0.02);
    }
    assert!(matches!(
        Lqr::new(LqrConfig {
            safe_value: [RonFloat::INFINITY],
            ..external([1.0], [1.0])
        }),
        Err(RonError::ConfigInvalid(_))
    ));
}
