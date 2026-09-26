//! # `kalman::tests`
//!
//! Traceable tests for the Kalman filter.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-600-RON-FR-607
//! **Tests:** RON-TC-KF-001-RON-TC-KF-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Kalman, KalmanConfig};
use crate::{
    matrix::{Matrix, MATRIX_MAX_DIM},
    RonError, RonFloat,
};

const TOL: RonFloat = 0.01;
const TIGHT: RonFloat = 1.0e-4;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

/// Deterministic uniform noise in `[-0.5, 0.5)` (the C suite's LCG).
struct Noise(u32);

impl Noise {
    fn next(&mut self) -> RonFloat {
        self.0 = self.0.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        #[allow(clippy::cast_possible_truncation)]
        let value = (f64::from(self.0) / 4_294_967_296.0 - 0.5) as RonFloat;
        value
    }
}

/// Random-walk scalar model: `A = H = 1`, `Q = 0.01`, `R = 1`, `P0 = 10`.
fn scalar() -> KalmanConfig<1, 1, 0> {
    KalmanConfig {
        a: Matrix::new([[1.0]]),
        b: Matrix::new([[]]),
        h: Matrix::new([[1.0]]),
        q: Matrix::new([[0.01]]),
        r: Matrix::new([[1.0]]),
        x0: [0.0],
        p0: Matrix::new([[10.0]]),
        joseph_form: false,
        steady_state_gain: None,
    }
}

fn two_state_two_measurements(r: Matrix<2, 2>, p0: Matrix<2, 2>) -> KalmanConfig<2, 2, 0> {
    KalmanConfig {
        a: Matrix::identity(),
        b: Matrix::new([[], []]),
        h: Matrix::identity(),
        q: Matrix::zeros(),
        r,
        x0: [0.0, 0.0],
        p0,
        joseph_form: false,
        steady_state_gain: None,
    }
}

fn constant_velocity(joseph_form: bool) -> KalmanConfig<2, 1, 0> {
    KalmanConfig {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::new([[], []]),
        h: Matrix::new([[1.0, 0.0]]),
        q: Matrix::diagonal([0.01, 0.01]),
        r: Matrix::new([[1.0]]),
        x0: [0.0, 1.0],
        p0: Matrix::identity(),
        joseph_form,
        steady_state_gain: None,
    }
}

/// RON-TC-KF-001 | RON-FR-600
#[test]
fn ron_tc_kf_001() {
    let mut filter = Kalman::new(scalar()).unwrap();
    approx_eq(filter.covariance().rows()[0][0], 10.0, TIGHT);
    let mut noise = Noise(1);
    let mut previous = filter.covariance().rows()[0][0];
    for cycle in 0..100 {
        filter.predict(&[]).unwrap();
        filter.update(Some(&[5.0 + noise.next()])).unwrap();
        let variance = filter.covariance().rows()[0][0];
        if cycle < 20 {
            assert!(variance < previous);
        }
        previous = variance;
    }
    assert!((filter.state()[0] - 5.0).abs() < 0.5);
}

/// RON-TC-KF-002 | RON-FR-601
#[test]
fn ron_tc_kf_002() {
    let base = scalar();
    let bad = [
        KalmanConfig {
            a: Matrix::new([[RonFloat::INFINITY]]),
            ..base
        },
        KalmanConfig {
            q: Matrix::new([[RonFloat::NAN]]),
            ..base
        },
        KalmanConfig {
            p0: Matrix::new([[RonFloat::INFINITY]]),
            ..base
        },
        KalmanConfig {
            h: Matrix::new([[RonFloat::NAN]]),
            ..base
        },
        KalmanConfig {
            r: Matrix::new([[RonFloat::INFINITY]]),
            ..base
        },
        KalmanConfig {
            x0: [RonFloat::NAN],
            ..base
        },
        KalmanConfig {
            steady_state_gain: Some(Matrix::new([[RonFloat::NEG_INFINITY]])),
            ..base
        },
    ];
    for config in bad {
        assert!(matches!(
            Kalman::new(config),
            Err(RonError::ConfigInvalid(_))
        ));
    }

    let with_input = KalmanConfig::<2, 2, 1> {
        a: Matrix::identity(),
        b: Matrix::new([[0.5], [0.25]]),
        h: Matrix::identity(),
        q: Matrix::diagonal([0.01, 0.01]),
        r: Matrix::diagonal([1.0, 2.0]),
        x0: [3.0, -1.0],
        p0: Matrix::diagonal([5.0, 7.0]),
        joseph_form: false,
        steady_state_gain: None,
    };
    let filter = Kalman::new(with_input).unwrap();
    assert_eq!(filter.state(), [3.0, -1.0]);
    assert_eq!(filter.covariance(), Matrix::diagonal([5.0, 7.0]));
    assert!(Kalman::new(KalmanConfig {
        b: Matrix::new([[RonFloat::INFINITY], [0.25]]),
        ..with_input
    })
    .is_err());
    const TOO_MANY: usize = MATRIX_MAX_DIM + 1;
    assert!(Kalman::new(KalmanConfig::<1, 1, TOO_MANY> {
        a: base.a,
        b: Matrix::zeros(),
        h: base.h,
        q: base.q,
        r: base.r,
        x0: base.x0,
        p0: base.p0,
        joseph_form: false,
        steady_state_gain: None,
    })
    .is_err());
}

/// RON-TC-KF-003 | RON-FR-602
#[test]
fn ron_tc_kf_003() {
    let mut scalar_filter = Kalman::new(KalmanConfig {
        a: Matrix::new([[2.0]]),
        q: Matrix::zeros(),
        x0: [1.0],
        p0: Matrix::new([[1.0]]),
        ..scalar()
    })
    .unwrap();
    scalar_filter.predict(&[]).unwrap();
    approx_eq(scalar_filter.state()[0], 2.0, TIGHT);
    approx_eq(scalar_filter.covariance().rows()[0][0], 4.0, TIGHT);
    scalar_filter.update(Some(&[3.0])).unwrap();
    approx_eq(scalar_filter.state()[0], 2.8, TIGHT);
    approx_eq(scalar_filter.covariance().rows()[0][0], 0.8, TIGHT);
    scalar_filter.reset();
    assert_eq!(scalar_filter.state(), [1.0]);
    approx_eq(scalar_filter.covariance().rows()[0][0], 1.0, TIGHT);

    let mut filter = Kalman::new(KalmanConfig {
        q: Matrix::zeros(),
        ..constant_velocity(false)
    })
    .unwrap();
    filter.predict(&[]).unwrap();
    assert_eq!(filter.state(), [1.0, 1.0]);
    assert_eq!(filter.covariance(), Matrix::new([[2.0, 1.0], [1.0, 1.0]]));
    filter.update(Some(&[2.0])).unwrap();
    let x = filter.state();
    approx_eq(x[0], 5.0 / 3.0, TOL);
    approx_eq(x[1], 4.0 / 3.0, TOL);
    let p = filter.covariance();
    for (row, expected) in p.rows().iter().zip([[2.0, 1.0], [1.0, 2.0]]) {
        for (value, e) in row.iter().zip(expected) {
            approx_eq(*value, e / 3.0, TOL);
        }
    }
    filter.update(None).unwrap();
    approx_eq(filter.state()[0], 5.0 / 3.0, TOL);
}

/// RON-TC-KF-004 | RON-FR-603
#[test]
fn ron_tc_kf_004() {
    let mut diagonal = Kalman::new(two_state_two_measurements(
        Matrix::diagonal([2.0, 3.0]),
        Matrix::diagonal([4.0, 9.0]),
    ))
    .unwrap();
    diagonal.predict(&[]).unwrap();
    diagonal.update(Some(&[1.0, 1.0])).unwrap();
    approx_eq(diagonal.state()[0], 2.0 / 3.0, TOL);
    approx_eq(diagonal.state()[1], 0.75, TOL);
    approx_eq(diagonal.covariance().rows()[0][0], 4.0 / 3.0, TOL);
    approx_eq(diagonal.covariance().rows()[1][1], 2.25, TOL);

    let mut coupled = Kalman::new(two_state_two_measurements(
        Matrix::identity(),
        Matrix::new([[2.0, 1.0], [1.0, 2.0]]),
    ))
    .unwrap();
    coupled.predict(&[]).unwrap();
    coupled.update(Some(&[1.0, 0.0])).unwrap();
    approx_eq(coupled.state()[0], 0.625, TOL);
    approx_eq(coupled.state()[1], 0.125, TOL);
    let p = coupled.covariance();
    approx_eq(p.rows()[0][0], 0.625, TOL);
    approx_eq(p.rows()[0][1], 0.125, TOL);
    approx_eq(p.rows()[1][0], 0.125, TOL);
    approx_eq(p.rows()[1][1], 0.625, TOL);

    let mut indefinite = Kalman::new(two_state_two_measurements(
        Matrix::new([[1.0, 2.0], [2.0, 1.0]]),
        Matrix::zeros(),
    ))
    .unwrap();
    indefinite.predict(&[]).unwrap();
    assert!(matches!(
        indefinite.update(Some(&[1.0, 1.0])),
        Err(RonError::Numerical(_))
    ));
    assert_eq!(indefinite.state(), [0.0, 0.0]);

    let mut zero_noise = Kalman::new(KalmanConfig {
        r: Matrix::new([[0.0]]),
        q: Matrix::zeros(),
        p0: Matrix::zeros(),
        ..scalar()
    })
    .unwrap();
    zero_noise.predict(&[]).unwrap();
    assert!(matches!(
        zero_noise.update(Some(&[1.0])),
        Err(RonError::Numerical(_))
    ));
}

/// RON-TC-KF-005 | RON-FR-604
#[test]
fn ron_tc_kf_005() {
    let mut standard = Kalman::new(constant_velocity(false)).unwrap();
    let mut joseph = Kalman::new(constant_velocity(true)).unwrap();
    let (mut noise_a, mut noise_b) = (Noise(7), Noise(7));
    for _ in 0..30 {
        standard.predict(&[]).unwrap();
        joseph.predict(&[]).unwrap();
        standard.update(Some(&[2.0 + noise_a.next()])).unwrap();
        joseph.update(Some(&[2.0 + noise_b.next()])).unwrap();
    }
    for (a, b) in standard.state().iter().zip(joseph.state().iter()) {
        approx_eq(*a, *b, TOL);
    }
    let (ps, pj) = (standard.covariance(), joseph.covariance());
    for (row_s, row_j) in ps.rows().iter().zip(pj.rows().iter()) {
        for (a, b) in row_s.iter().zip(row_j.iter()) {
            approx_eq(*a, *b, TOL);
        }
    }
    approx_eq(pj.rows()[0][1], pj.rows()[1][0], TIGHT);

    let mut coupled = Kalman::new(KalmanConfig {
        joseph_form: true,
        ..two_state_two_measurements(Matrix::identity(), Matrix::new([[2.0, 1.0], [1.0, 2.0]]))
    })
    .unwrap();
    coupled.predict(&[]).unwrap();
    coupled.update(Some(&[1.0, 0.0])).unwrap();
    approx_eq(coupled.state()[0], 0.625, TOL);
    approx_eq(coupled.state()[1], 0.125, TOL);
    let p = coupled.covariance();
    approx_eq(p.rows()[0][0], p.rows()[1][1], TIGHT);
    approx_eq(p.rows()[0][1], p.rows()[1][0], TIGHT);
}

/// RON-TC-KF-006 | RON-FR-605
#[test]
fn ron_tc_kf_006() {
    let mut filter = Kalman::new(scalar()).unwrap();
    let mut noise = Noise(3);
    for _ in 0..50 {
        filter.predict(&[]).unwrap();
        filter.update(Some(&[5.0 + noise.next()])).unwrap();
    }
    let converged = filter.state()[0];
    let mut previous = filter.covariance().rows()[0][0];
    for _ in 0..10 {
        filter.predict(&[]).unwrap();
        filter.update(None).unwrap();
        approx_eq(filter.state()[0], converged, TIGHT);
        let variance = filter.covariance().rows()[0][0];
        assert!(variance > previous);
        previous = variance;
    }
}

/// RON-TC-KF-007 | RON-FR-606
#[test]
fn ron_tc_kf_007() {
    let k_inf = 0.095_124_9;
    let mut filter = Kalman::new(KalmanConfig {
        steady_state_gain: Some(Matrix::new([[k_inf]])),
        ..scalar()
    })
    .unwrap();
    filter.predict(&[]).unwrap();
    filter.update(Some(&[5.0])).unwrap();
    approx_eq(filter.state()[0], k_inf * 5.0, TIGHT);
    let mut previous = filter.state()[0];
    for _ in 0..100 {
        filter.predict(&[]).unwrap();
        filter.update(Some(&[5.0])).unwrap();
        assert!(filter.state()[0] >= previous);
        previous = filter.state()[0];
    }
    assert!((filter.state()[0] - 5.0).abs() < 0.5);
}

/// RON-TC-KF-008 | RON-FR-607
#[test]
fn ron_tc_kf_008() {
    const N: usize = MATRIX_MAX_DIM;
    let mut largest = Kalman::new(KalmanConfig::<N, N, N> {
        a: Matrix::identity(),
        b: Matrix::identity().scale(0.1),
        h: Matrix::identity(),
        q: Matrix::identity().scale(0.01),
        r: Matrix::identity(),
        x0: [0.0; N],
        p0: Matrix::identity(),
        joseph_form: false,
        steady_state_gain: None,
    })
    .unwrap();
    for _ in 0..5 {
        largest.predict(&[1.0; N]).unwrap();
        largest.update(Some(&[0.5; N])).unwrap();
    }
    assert!(largest.state().iter().all(|value| value.is_finite()));

    let mut input = [1.0; N];
    input[0] = RonFloat::NAN;
    assert!(matches!(
        largest.predict(&input),
        Err(RonError::InvalidArgument(_))
    ));
    let mut measurement = [0.5; N];
    measurement[0] = RonFloat::INFINITY;
    assert!(largest.update(Some(&measurement)).is_err());
    assert!(largest.update(None).is_ok());

    let mut diverging = Kalman::new(KalmanConfig {
        a: Matrix::new([[1.0e30]]),
        q: Matrix::zeros(),
        x0: [1.0],
        p0: Matrix::new([[1.0]]),
        ..scalar()
    })
    .unwrap();
    let mut result = Ok(());
    for _ in 0..40 {
        result = diverging.predict(&[]);
        if result.is_err() {
            break;
        }
    }
    assert!(matches!(result, Err(RonError::Numerical(_))));
    assert!(diverging.state()[0].is_finite());
    assert!(diverging.covariance().is_finite());
}
