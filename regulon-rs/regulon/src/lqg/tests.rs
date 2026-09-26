//! # `lqg::tests`
//!
//! Traceable tests for the LQG controller.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-750-RON-FR-759
//! **Tests:** RON-TC-LQG-001-RON-TC-LQG-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Lqg, LqgConfig, LqgGain};
use crate::{
    lqr::{solve_dare, DareConfig},
    matrix::{Matrix, MATRIX_MAX_DIM},
    pid::{PidFault, PidStatus},
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

/// Double integrator with a pre-computed gain `K = [1, 1]`.
fn base() -> LqgConfig<2, 1, 1> {
    LqgConfig {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::new([[0.0], [1.0]]),
        h: Matrix::new([[1.0, 0.0]]),
        q_noise: Matrix::diagonal([0.01, 0.01]),
        r_noise: Matrix::new([[1.0]]),
        x0: [0.0, 0.0],
        p0: Matrix::diagonal([10.0, 10.0]),
        joseph_form: false,
        kalman_steady_state_gain: None,
        gain: LqgGain::Precomputed(Matrix::new([[1.0, 1.0]])),
        kr: [0.0],
        limits: [WIDE],
    }
}

const DARE_GAIN: LqgGain<2, 1> = LqgGain::Dare {
    q_cost: Matrix::new([[1.0, 0.0], [0.0, 1.0]]),
    r_cost: Matrix::new([[1.0]]),
    max_iterations: 200,
    tolerance: 1.0e-4,
};

/// RON-TC-LQG-001 | RON-FR-750, RON-FR-756
#[test]
fn ron_tc_lqg_001() {
    let lqg = Lqg::new(base()).unwrap();
    assert_eq!(lqg.state(), [0.0, 0.0]);
    assert!(lqg.dare_solution().is_none());
    assert_eq!(*lqg.gain(), Matrix::new([[1.0, 1.0]]));
}

/// RON-TC-LQG-002 | RON-FR-753
#[test]
fn ron_tc_lqg_002() {
    let mut lqg = Lqg::new(base()).unwrap();
    lqg.predict(&[1.0]).unwrap();
    approx_eq(lqg.state()[0], 0.0);
    approx_eq(lqg.state()[1], 1.0);
}

/// RON-TC-LQG-003 | RON-FR-754
#[test]
fn ron_tc_lqg_003() {
    let mut lqg = Lqg::new(base()).unwrap();
    lqg.predict(&[0.0]).unwrap();
    let before = lqg.state();
    lqg.update(Some(&[0.5])).unwrap();
    assert!(lqg.state()[0] > before[0]);
}

/// RON-TC-LQG-004 | RON-FR-754
#[test]
fn ron_tc_lqg_004() {
    let mut lqg = Lqg::new(base()).unwrap();
    lqg.predict(&[0.0]).unwrap();
    let before = lqg.state();
    lqg.update(None).unwrap();
    assert_eq!(lqg.state(), before);
}

/// RON-TC-LQG-005 | RON-FR-755
#[test]
fn ron_tc_lqg_005() {
    let mut lqg = Lqg::new(LqgConfig {
        gain: LqgGain::Precomputed(Matrix::new([[0.5, 0.25]])),
        kr: [2.0],
        ..base()
    })
    .unwrap();
    lqg.predict(&[2.0]).unwrap();
    lqg.update(Some(&[1.5])).unwrap();
    let x = lqg.state();
    let (u, _) = lqg.step(&[1.0], 0.01).unwrap();
    approx_eq(u[0], -(0.5 * x[0] + 0.25 * x[1]) + 2.0);
}

/// Both gains solved at construction, with a tight tolerance.
const DARE_BOTH: LqgGain<2, 1> = LqgGain::DareBoth {
    q_cost: Matrix::new([[1.0, 0.0], [0.0, 1.0]]),
    r_cost: Matrix::new([[1.0]]),
    max_iterations: 500,
    tolerance: 1.0e-6,
};

fn near(lhs: RonFloat, rhs: RonFloat) {
    assert!((lhs - rhs).abs() <= 1.0e-3, "{lhs} != {rhs}");
}

/// RON-TC-LQG-006 | RON-FR-756
#[test]
fn ron_tc_lqg_006() {
    // (a) Both gains solved and matching the offline references.
    let lqg = Lqg::new(LqgConfig {
        gain: DARE_BOTH,
        ..base()
    })
    .unwrap();
    near(lqg.gain().rows()[0][0], 0.422_082);
    near(lqg.gain().rows()[0][1], 1.243_929);
    let solved = lqg.kalman().config().steady_state_gain.unwrap();
    near(solved.rows()[0][0], 0.368_686);
    near(solved.rows()[1][0], 0.079_455);

    // (b) A time-varying filter converges to the same gain.
    let mut tv = Lqg::new(LqgConfig {
        gain: DARE_GAIN,
        ..base()
    })
    .unwrap();
    let mut gain = [0.0; 2];
    for _ in 0..200 {
        tv.predict(&[0.0]).unwrap();
        let p = tv.kalman().covariance();
        let s = p.rows()[0][0] + 1.0;
        gain = [p.rows()[0][0] / s, p.rows()[1][0] / s];
        tv.update(Some(&[0.0])).unwrap();
    }
    near(gain[0], solved.rows()[0][0]);
    near(gain[1], solved.rows()[1][0]);

    // (c) The DARE mode still uses a supplied steady-state gain unchanged.
    let steady = Matrix::new([[0.5], [0.2]]);
    let supplied = Lqg::new(LqgConfig {
        gain: DARE_GAIN,
        kalman_steady_state_gain: Some(steady),
        ..base()
    })
    .unwrap();
    assert!(supplied.dare_solution().is_some());
    assert_eq!(supplied.kalman().config().steady_state_gain, Some(steady));

    // (d) Rejected estimator designs.
    for config in [
        LqgConfig {
            h: Matrix::new([[1.0, RonFloat::NAN]]),
            ..base()
        },
        LqgConfig {
            q_noise: Matrix::diagonal([0.01, RonFloat::INFINITY]),
            ..base()
        },
        LqgConfig {
            r_noise: Matrix::new([[RonFloat::NAN]]),
            ..base()
        },
        // R + H Q H^T = -1.
        LqgConfig {
            q_noise: Matrix::diagonal([-2.0, 0.01]),
            ..base()
        },
    ] {
        assert!(Lqg::new(LqgConfig {
            gain: DARE_BOTH,
            ..config
        })
        .is_err());
    }

    // Scalar plant, loose tolerance: the estimator DARE stops after one step
    // on P = -2.1, so H P H^T + R_noise = -1.1 cannot be factored.
    let scalar = Lqg::<1, 1, 1>::new(LqgConfig {
        a: Matrix::new([[1.0]]),
        b: Matrix::new([[1.0]]),
        h: Matrix::new([[1.0]]),
        q_noise: Matrix::new([[-0.6]]),
        r_noise: Matrix::new([[1.0]]),
        x0: [0.0],
        p0: Matrix::new([[1.0]]),
        joseph_form: false,
        kalman_steady_state_gain: None,
        gain: LqgGain::DareBoth {
            q_cost: Matrix::new([[1.0]]),
            r_cost: Matrix::new([[1.0]]),
            max_iterations: 500,
            tolerance: 10.0,
        },
        kr: [0.0],
        limits: [WIDE],
    });
    assert!(matches!(
        scalar,
        Err(RonError::Numerical(
            "H P H^T + R_noise is not positive definite"
        ))
    ));
}

/// RON-TC-LQG-007 | RON-FR-752
#[test]
fn ron_tc_lqg_007() {
    let config = LqgConfig {
        gain: DARE_GAIN,
        ..base()
    };
    let lqg = Lqg::new(config).unwrap();
    let standalone = solve_dare(&DareConfig {
        a: config.a,
        b: config.b,
        q: Matrix::identity(),
        r: Matrix::new([[1.0]]),
        max_iterations: 200,
        tolerance: 1.0e-4,
    })
    .unwrap();
    assert_eq!(*lqg.gain(), standalone.k);
    // Changing the noise model leaves the control gain unchanged.
    let noisier = Lqg::new(LqgConfig {
        q_noise: Matrix::diagonal([1.0, 1.0]),
        r_noise: Matrix::new([[10.0]]),
        ..config
    })
    .unwrap();
    assert_eq!(noisier.gain(), lqg.gain());
}

/// RON-TC-LQG-008 | RON-FR-757
#[test]
fn ron_tc_lqg_008() {
    let mut lqg = Lqg::new(LqgConfig {
        gain: LqgGain::Precomputed(Matrix::new([[5.0, 5.0]])),
        limits: [OutputLimits {
            min: -1.0,
            max: 1.0,
            rate_limit: 0.0,
        }],
        ..base()
    })
    .unwrap();
    lqg.predict(&[0.0]).unwrap();
    lqg.update(Some(&[-100.0])).unwrap();
    let (u, status) = lqg.step(&[0.0], 1.0).unwrap();
    approx_eq(u[0], 1.0);
    assert!(status.contains(PidStatus::SATURATED));

    let mut limited = Lqg::new(LqgConfig {
        gain: LqgGain::Precomputed(Matrix::new([[1.0, 0.0]])),
        limits: [OutputLimits {
            rate_limit: 1.0,
            ..WIDE
        }],
        ..base()
    })
    .unwrap();
    assert!(!limited
        .step(&[0.0], 1.0)
        .unwrap()
        .1
        .contains(PidStatus::RATE_LIMITED));
    for z in [-100.0, 1_000.0] {
        limited.predict(&[0.0]).unwrap();
        limited.update(Some(&[z])).unwrap();
        assert!(limited
            .step(&[0.0], 1.0)
            .unwrap()
            .1
            .contains(PidStatus::RATE_LIMITED));
    }
}

/// RON-TC-LQG-009 | RON-FR-757, RON-SR-001
#[test]
fn ron_tc_lqg_009() {
    let mut lqg = Lqg::new(base()).unwrap();
    for (r, dt) in [
        ([0.0], RonFloat::NAN),
        ([0.0], 0.0),
        ([RonFloat::INFINITY], 0.01),
    ] {
        assert_eq!(
            lqg.step(&r, dt),
            Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
        );
        lqg.clear_fault();
    }
    lqg.predict(&[1.0]).unwrap();
    lqg.reset();
    assert_eq!(lqg.state(), [0.0, 0.0]);

    let mut overflowing = Lqg::new(LqgConfig {
        gain: LqgGain::Precomputed(Matrix::new([[RonFloat::MAX, 0.0]])),
        limits: [OutputLimits {
            min: -RonFloat::MAX,
            max: RonFloat::MAX,
            rate_limit: 0.0,
        }],
        ..base()
    })
    .unwrap();
    overflowing.predict(&[0.0]).unwrap();
    overflowing.update(Some(&[RonFloat::MAX])).unwrap();
    assert_eq!(
        overflowing.step(&[0.0], 0.1),
        Err(RonError::Fault(PidFault::OUTPUT_NOT_FINITE))
    );
}

/// RON-TC-LQG-009 | RON-FR-750, RON-FR-751
#[test]
fn ron_tc_lqg_009_validation() {
    let b = base();
    let dare = |tolerance: RonFloat, q: RonFloat, r: RonFloat| LqgGain::Dare {
        q_cost: Matrix::diagonal([q, 1.0]),
        r_cost: Matrix::new([[r]]),
        max_iterations: 0,
        tolerance,
    };
    let limits = |min: RonFloat, max: RonFloat, rate_limit: RonFloat| {
        [OutputLimits {
            min,
            max,
            rate_limit,
        }]
    };
    let invalid = [
        LqgConfig {
            a: Matrix::new([[RonFloat::INFINITY, 1.0], [0.0, 1.0]]),
            ..b
        },
        LqgConfig {
            b: Matrix::new([[0.0], [RonFloat::NAN]]),
            ..b
        },
        LqgConfig {
            h: Matrix::new([[RonFloat::NAN, 0.0]]),
            ..b
        },
        LqgConfig {
            q_noise: Matrix::diagonal([RonFloat::INFINITY, 0.01]),
            ..b
        },
        LqgConfig {
            r_noise: Matrix::new([[RonFloat::NAN]]),
            ..b
        },
        LqgConfig {
            x0: [RonFloat::INFINITY, 0.0],
            ..b
        },
        LqgConfig {
            p0: Matrix::diagonal([RonFloat::NAN, 1.0]),
            ..b
        },
        LqgConfig {
            kalman_steady_state_gain: Some(Matrix::new([[RonFloat::INFINITY], [0.0]])),
            ..b
        },
        LqgConfig {
            kr: [RonFloat::INFINITY],
            ..b
        },
        LqgConfig {
            gain: LqgGain::Precomputed(Matrix::new([[RonFloat::NAN, 0.0]])),
            ..b
        },
        LqgConfig {
            gain: dare(0.0, 1.0, 1.0),
            ..b
        },
        LqgConfig {
            gain: dare(RonFloat::NAN, 1.0, 1.0),
            ..b
        },
        LqgConfig {
            gain: dare(1.0e-6, 1.0, RonFloat::NAN),
            ..b
        },
        LqgConfig {
            gain: dare(1.0e-6, RonFloat::INFINITY, 1.0),
            ..b
        },
        LqgConfig {
            limits: limits(RonFloat::INFINITY, 1.0, 0.0),
            ..b
        },
        LqgConfig {
            limits: limits(-1.0, RonFloat::NAN, 0.0),
            ..b
        },
        LqgConfig {
            limits: limits(-1.0, 1.0, RonFloat::NAN),
            ..b
        },
        LqgConfig {
            limits: limits(5.0, 1.0, 0.0),
            ..b
        },
    ];
    for config in invalid {
        assert!(matches!(Lqg::new(config), Err(RonError::ConfigInvalid(_))));
    }
    assert!(matches!(
        Lqg::new(LqgConfig {
            b: Matrix::zeros(),
            gain: dare(1.0e-4, 1.0, 0.0),
            ..b
        }),
        Err(RonError::Numerical(_))
    ));

    const N: usize = MATRIX_MAX_DIM;
    let mut largest = Lqg::new(LqgConfig::<N, N, N> {
        a: Matrix::identity(),
        b: Matrix::zeros(),
        h: Matrix::identity(),
        q_noise: Matrix::zeros(),
        r_noise: Matrix::identity(),
        x0: [0.0; N],
        p0: Matrix::identity(),
        joseph_form: false,
        kalman_steady_state_gain: None,
        gain: LqgGain::Precomputed(Matrix::zeros()),
        kr: [0.0; N],
        limits: [WIDE; N],
    })
    .unwrap();
    assert!(largest.step(&[0.0; N], 0.01).is_ok());
}

/// RON-TC-LQG-011 | RON-FR-757, RON-SR-010, RON-SR-012, RON-SR-013
#[test]
fn ron_tc_lqg_011() {
    let mut lqg = Lqg::new(LqgConfig {
        kr: [2.0],
        ..base()
    })
    .unwrap();
    approx_eq(lqg.step(&[1.0], 0.01).unwrap().0[0], 2.0);

    // The fault latches and the last output is held.
    let latched = Err(RonError::Fault(PidFault::INPUT_NOT_FINITE));
    assert_eq!(lqg.step(&[RonFloat::NAN], 0.01), latched);
    assert_eq!(lqg.fault(), PidFault::INPUT_NOT_FINITE);
    approx_eq(lqg.output()[0], 2.0);

    // Finite inputs do not clear it, and the estimator still runs.
    assert_eq!(lqg.step(&[3.0], 0.01), latched);
    approx_eq(lqg.output()[0], 2.0);
    lqg.predict(&[0.0]).unwrap();
    lqg.update(None).unwrap();

    // An explicit clear resumes normal stepping.
    lqg.clear_fault();
    let (output, status) = lqg.step(&[3.0], 0.01).unwrap();
    approx_eq(output[0], 6.0);
    assert!(!status.contains(PidStatus::FAULT));

    // Reset clears a latched fault too.
    assert_eq!(lqg.step(&[3.0], 0.0), latched);
    lqg.reset();
    assert_eq!(lqg.fault(), PidFault::NONE);
    assert!(lqg.step(&[3.0], 0.01).is_ok());
}
