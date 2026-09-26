//! # `observer::tests`
//!
//! Traceable tests for the Luenberger observer.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-720-RON-FR-723
//! **Tests:** RON-TC-SS-006-RON-TC-SS-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Observer, ObserverConfig};
use crate::{
    matrix::{Matrix, MATRIX_MAX_DIM},
    RonError, RonFloat,
};

const TOL: RonFloat = 1.0e-4;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

/// `A = [[1,1],[0,1]]`, `B = [[0],[1]]`, `C = [[1,0]]`, `L = [[0.5],[0.2]]`.
fn config() -> ObserverConfig<2, 1, 1> {
    ObserverConfig {
        a: Matrix::new([[1.0, 1.0], [0.0, 1.0]]),
        b: Matrix::new([[0.0], [1.0]]),
        c: Matrix::new([[1.0, 0.0]]),
        l: Matrix::new([[0.5], [0.2]]),
        x0: [0.0, 0.0],
    }
}

/// RON-TC-SS-006 | RON-FR-720
#[test]
fn ron_tc_ss_006() {
    let mut observer = Observer::new(config()).unwrap();
    observer.step(&[1.0], &[0.0]).unwrap();
    let x = observer.state();
    approx_eq(x[0], 0.5, TOL);
    approx_eq(x[1], 0.2, TOL);
    observer.step(&[1.0], &[0.0]).unwrap();
    let x = observer.state();
    approx_eq(x[0], 0.95, TOL);
    approx_eq(x[1], 0.3, TOL);
    observer.reset();
    assert_eq!(observer.state(), [0.0, 0.0]);
}

/// RON-TC-SS-007 | RON-FR-721
#[test]
fn ron_tc_ss_007_convergence() {
    let mut observer = Observer::new(ObserverConfig::<1, 1, 0> {
        a: Matrix::new([[1.0]]),
        b: Matrix::new([[]]),
        c: Matrix::new([[1.0]]),
        l: Matrix::new([[0.5]]),
        x0: [0.0],
    })
    .unwrap();
    for _ in 0..40 {
        observer.step(&[5.0], &[]).unwrap();
    }
    approx_eq(observer.state()[0], 5.0, 0.001);
}

/// RON-TC-SS-007 | RON-FR-721
#[test]
fn ron_tc_ss_007_validation() {
    let base = config();
    let mut bad_a = base;
    bad_a.a = Matrix::new([[RonFloat::INFINITY, 1.0], [0.0, 1.0]]);
    let mut bad_c = base;
    bad_c.c = Matrix::new([[RonFloat::NAN, 0.0]]);
    let mut bad_l = base;
    bad_l.l = Matrix::new([[RonFloat::INFINITY], [0.2]]);
    let mut bad_b = base;
    bad_b.b = Matrix::new([[0.0], [RonFloat::NAN]]);
    let mut bad_x0 = base;
    bad_x0.x0 = [RonFloat::INFINITY, 0.0];
    for config in [bad_a, bad_c, bad_l, bad_b, bad_x0] {
        assert!(matches!(
            Observer::new(config),
            Err(RonError::ConfigInvalid(_))
        ));
    }
}

/// RON-TC-SS-008 | RON-FR-722
#[test]
fn ron_tc_ss_008() {
    let mut observer = Observer::new(config()).unwrap();
    assert!(matches!(
        observer.step(&[RonFloat::INFINITY], &[0.0]),
        Err(RonError::InvalidArgument(_))
    ));
    assert!(observer.step(&[1.0], &[RonFloat::NAN]).is_err());
    assert_eq!(observer.state(), [0.0, 0.0]);
    observer.step(&[1.0], &[0.0]).unwrap();
    let x = observer.state();
    approx_eq(x[0], 0.5, TOL);
    approx_eq(x[1], 0.2, TOL);
}

/// RON-TC-SS-009 | RON-FR-723
#[test]
fn ron_tc_ss_009() {
    let mut largest =
        Observer::new(
            ObserverConfig::<MATRIX_MAX_DIM, MATRIX_MAX_DIM, MATRIX_MAX_DIM> {
                a: Matrix::identity(),
                b: Matrix::zeros(),
                c: Matrix::identity(),
                l: Matrix::identity().scale(0.1),
                x0: [0.0; MATRIX_MAX_DIM],
            },
        )
        .unwrap();
    largest
        .step(&[1.0; MATRIX_MAX_DIM], &[0.0; MATRIX_MAX_DIM])
        .unwrap();
    approx_eq(largest.state()[0], 0.1, TOL);

    assert!(Observer::new(ObserverConfig::<0, 1, 0> {
        a: Matrix::new([]),
        b: Matrix::new([]),
        c: Matrix::new([[]]),
        l: Matrix::new([]),
        x0: [],
    })
    .is_err());
    const TOO_BIG: usize = MATRIX_MAX_DIM + 1;
    assert!(Observer::new(ObserverConfig::<1, 1, TOO_BIG> {
        a: Matrix::new([[1.0]]),
        b: Matrix::zeros(),
        c: Matrix::new([[1.0]]),
        l: Matrix::new([[0.5]]),
        x0: [0.0],
    })
    .is_err());

    let mut diverging = Observer::new(ObserverConfig::<1, 1, 0> {
        a: Matrix::new([[1.0e30]]),
        b: Matrix::new([[]]),
        c: Matrix::new([[1.0]]),
        l: Matrix::new([[0.0]]),
        x0: [1.0e30],
    })
    .unwrap();
    let mut result = Ok(());
    for _ in 0..20 {
        result = diverging.step(&[0.0], &[]);
        if result.is_err() {
            break;
        }
    }
    assert!(matches!(result, Err(RonError::Numerical(_))));
    assert!(diverging.state()[0].is_finite());
}
