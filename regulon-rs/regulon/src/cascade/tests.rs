//! # `cascade::tests`
//!
//! Traceable tests for cascade control.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-400-RON-FR-406
//! **Tests:** RON-TC-CASC-001-RON-TC-CASC-012
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Cascade, CascadeStatus};
use crate::{
    pid::{AntiWindupMode, PidConfig, PidFault, PidMode, PidStatus},
    RonError, RonFloat,
};

const TIGHT: RonFloat = 4.0 * RonFloat::EPSILON;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

fn loop_config(kp: RonFloat, limit: RonFloat) -> PidConfig {
    PidConfig {
        output_min: -limit,
        output_max: limit,
        integral_min: -100.0,
        integral_max: 100.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(kp, 0.0, 0.0)
    }
}

/// RON-TC-CASC-001 | RON-FR-400
#[test]
fn ron_tc_casc_001() {
    let cascade = Cascade::new(loop_config(2.0, 100.0), loop_config(3.0, 100.0)).unwrap();
    approx_eq(cascade.outer().configuration().kp, 2.0, RonFloat::EPSILON);
    approx_eq(cascade.inner().configuration().kp, 3.0, RonFloat::EPSILON);
}

/// RON-TC-CASC-002 | RON-FR-400
#[test]
fn ron_tc_casc_002() {
    let valid = loop_config(1.0, 100.0);
    let degenerate = PidConfig {
        output_min: 10.0,
        output_max: 10.0,
        ..valid
    };
    assert!(matches!(
        Cascade::new(degenerate, valid),
        Err(RonError::ConfigInvalid(_))
    ));
    assert!(matches!(
        Cascade::new(valid, degenerate),
        Err(RonError::ConfigInvalid(_))
    ));
}

/// RON-TC-CASC-003 | RON-FR-401
#[test]
fn ron_tc_casc_003() {
    let mut cascade = Cascade::new(loop_config(1.0, 50.0), loop_config(2.0, 100.0)).unwrap();
    let (output, status) = cascade.step(10.0, 0.0, 0.0, 0.01).unwrap();
    approx_eq(output, 20.0, TIGHT);
    assert_eq!(status, CascadeStatus::default());
}

/// RON-TC-CASC-004 | RON-FR-402
#[test]
fn ron_tc_casc_004() {
    let mut cascade = Cascade::new(loop_config(100.0, 5.0), loop_config(1.0, 100.0)).unwrap();
    let (output, status) = cascade.step(10.0, 0.0, 0.0, 0.01).unwrap();
    approx_eq(output, 5.0, TIGHT);
    assert!(status.outer.contains(PidStatus::SATURATED));
}

/// RON-TC-CASC-005 | RON-FR-406
#[test]
fn ron_tc_casc_005() {
    let mut cascade = Cascade::new(loop_config(1.0, 1_000.0), loop_config(50.0, 2.0)).unwrap();
    let (_, status) = cascade.step(1.0, 0.0, 0.0, 0.01).unwrap();
    assert!(status.inner.contains(PidStatus::SATURATED));
    assert!(!status.outer.contains(PidStatus::SATURATED));
    let packed = status.bits();
    assert_eq!(packed & 0xFFFF, u32::from(status.outer.bits()));
    assert_eq!(packed >> 16, u32::from(status.inner.bits()));
}

/// RON-TC-CASC-006 | RON-FR-403
#[test]
fn ron_tc_casc_006() {
    let outer_aw = PidConfig {
        ki: 10.0,
        integral_min: -500.0,
        integral_max: 500.0,
        anti_windup_mode: AntiWindupMode::BackCalculation,
        anti_windup_tracking_time: 0.1,
        ..loop_config(1.0, 100.0)
    };
    let outer_plain = PidConfig {
        anti_windup_mode: AntiWindupMode::Disabled,
        ..outer_aw
    };
    let inner = loop_config(1.0, 0.5);
    let mut with_aw = Cascade::new(outer_aw, inner).unwrap();
    let mut without_aw = Cascade::new(outer_plain, inner).unwrap();
    for _ in 0..20 {
        let _ = with_aw.step(10.0, 0.0, 0.0, 0.01);
        let _ = without_aw.step(10.0, 0.0, 0.0, 0.01);
    }
    assert!(with_aw.outer().integral().abs() < without_aw.outer().integral().abs());
}

/// RON-TC-CASC-007 | RON-FR-404
#[test]
fn ron_tc_casc_007() {
    let config = loop_config(1.0, 100.0);
    let mut cascade = Cascade::new(config, config).unwrap();
    for _ in 0..5 {
        let _ = cascade.step(5.0, 0.0, 0.0, 0.01).unwrap();
    }
    cascade.set_mode(PidMode::Manual, 3.0, 7.0).unwrap();
    assert_eq!(cascade.outer().mode(), PidMode::Manual);
    assert_eq!(cascade.inner().mode(), PidMode::Manual);
    let (output, status) = cascade.step(5.0, 0.0, 0.0, 0.01).unwrap();
    approx_eq(output, 3.0, TIGHT);
    assert!(status.inner.contains(PidStatus::MANUAL_MODE));
    assert!(status.outer.contains(PidStatus::MANUAL_MODE));
}

/// RON-TC-CASC-008 | RON-FR-404
#[test]
fn ron_tc_casc_008() {
    let config = loop_config(1.0, 100.0);
    let mut cascade = Cascade::new(config, config).unwrap();
    cascade.set_mode(PidMode::Manual, 4.0, 8.0).unwrap();
    let (manual, _) = cascade.step(5.0, 0.0, 0.0, 0.01).unwrap();
    approx_eq(manual, 4.0, TIGHT);
    cascade.set_mode(PidMode::Automatic, 4.0, 8.0).unwrap();
    assert_eq!(cascade.outer().mode(), PidMode::Automatic);
    assert_eq!(cascade.inner().mode(), PidMode::Automatic);
    let (automatic, _) = cascade.step(8.0, 8.0, 8.0, 0.01).unwrap();
    approx_eq(automatic, 4.0, TIGHT);
}

/// RON-TC-CASC-009 | RON-FR-401, RON-FR-405
#[test]
fn ron_tc_casc_009() {
    let config = loop_config(1.0, 100.0);
    let mut cascade = Cascade::new(config, config).unwrap();
    let before = cascade;
    for dt in [0.0, -0.001, RonFloat::NAN, RonFloat::INFINITY] {
        assert!(matches!(
            cascade.step(0.0, 0.0, 0.0, dt),
            Err(RonError::InvalidArgument(_))
        ));
    }
    for (inner, outer) in [
        (RonFloat::NAN, 0.0),
        (0.0, RonFloat::NAN),
        (RonFloat::NEG_INFINITY, 0.0),
    ] {
        assert!(matches!(
            cascade.set_mode(PidMode::Manual, inner, outer),
            Err(RonError::InvalidArgument(_))
        ));
    }
    assert_eq!(cascade, before);
}

/// RON-TC-CASC-010 | RON-FR-406
#[test]
fn ron_tc_casc_010() {
    let mut cascade = Cascade::new(loop_config(1.0, 1_000.0), loop_config(50.0, 2.0)).unwrap();
    let (_, step_status) = cascade.step(1.0, 0.0, 0.0, 0.01).unwrap();
    assert_eq!(cascade.status(), step_status);
    assert_eq!(cascade.faults(), (PidFault::NONE, PidFault::NONE));

    let result = cascade.step(RonFloat::NAN, 0.0, 0.0, 0.01);
    assert_eq!(result, Err(RonError::Fault(PidFault::INPUT_NOT_FINITE)));
    assert!(!cascade.faults().0.is_none());
    assert!(cascade.faults().1.is_none());
    assert!(cascade.status().outer.contains(PidStatus::FAULT));
}

/// RON-TC-CASC-011 | RON-FR-405
#[test]
fn ron_tc_casc_011() {
    let config = loop_config(1.0, 100.0);
    let mut cascade = Cascade::new(config, config).unwrap();
    let _ = cascade.outer_mut().step(RonFloat::NAN, 0.0, 0.01);
    let _ = cascade.inner_mut().step(RonFloat::NAN, 0.0, 0.01);
    let (outer, inner) = cascade.faults();
    assert!(!outer.is_none() && !inner.is_none());
    assert_eq!(
        cascade.step(1.0, 0.0, 0.0, 0.01),
        Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
    );
    cascade.clear_fault();
    assert_eq!(cascade.faults(), (PidFault::NONE, PidFault::NONE));
    assert!(cascade.step(1.0, 0.0, 0.0, 0.01).is_ok());
}

/// RON-TC-CASC-012 | RON-FR-405
#[test]
fn ron_tc_casc_012() {
    let config = PidConfig {
        ki: 5.0,
        ..loop_config(1.0, 100.0)
    };
    let mut cascade = Cascade::new(config, config).unwrap();
    for _ in 0..10 {
        let _ = cascade.step(5.0, 0.0, 0.0, 0.01).unwrap();
    }
    assert!(cascade.outer().integral().abs() > 0.0);
    assert!(cascade.inner().integral().abs() > 0.0);
    cascade.reset();
    for pid in [cascade.outer(), cascade.inner()] {
        approx_eq(pid.integral(), 0.0, RonFloat::EPSILON);
        approx_eq(pid.last_output(), 0.0, RonFloat::EPSILON);
        approx_eq(pid.last_derivative(), 0.0, RonFloat::EPSILON);
    }
    assert_eq!(cascade.outer().configuration(), config);
}
