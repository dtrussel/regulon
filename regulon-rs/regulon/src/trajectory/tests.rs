//! # `trajectory::tests`
//!
//! Traceable tests for the trapezoidal and S-curve generators.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-500-RON-FR-503, RON-FR-510-RON-FR-515
//! **Tests:** RON-TC-TRAJ-001-RON-TC-TRAJ-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{
    SCurve, SCurveConfig, SCurvePhase, Setpoint, TrajectoryFault, Trapezoidal, TrapezoidalConfig,
    TrapezoidalPhase,
};
use crate::RonFloat;

const TOL: RonFloat = 0.001;
const MARGIN: RonFloat = 0.0001;
const DT: RonFloat = 0.01;
const MAX_STEPS: u16 = 5_000;

fn approx_eq(lhs: RonFloat, rhs: RonFloat) {
    assert!((lhs - rhs).abs() <= TOL, "{lhs} != {rhs}");
}

fn trap_config() -> TrapezoidalConfig {
    TrapezoidalConfig {
        v_max: 1.0,
        a_max: 2.0,
    }
}

fn scurve_config() -> SCurveConfig {
    SCurveConfig {
        v_max: 1.5,
        a_max: 2.0,
        j_max: 8.0,
    }
}

fn trap_to(start: RonFloat, target: RonFloat) -> Trapezoidal {
    let mut trap = Trapezoidal::new(trap_config(), start).unwrap();
    trap.set_target(target).unwrap();
    trap
}

fn scurve_to(config: SCurveConfig, start: RonFloat, target: RonFloat) -> SCurve {
    let mut scurve = SCurve::new(config, start).unwrap();
    scurve.set_target(target).unwrap();
    scurve
}

fn run_trap_to_done(trap: &mut Trapezoidal) -> Setpoint {
    let limits = trap.config();
    for _ in 0..MAX_STEPS {
        let setpoint = trap.step(DT).unwrap();
        assert!(setpoint.velocity.abs() <= limits.v_max + MARGIN);
        assert!(setpoint.acceleration.abs() <= limits.a_max + MARGIN);
        if setpoint.finished {
            return setpoint;
        }
    }
    panic!("trapezoidal move did not finish");
}

fn run_scurve_to_done(scurve: &mut SCurve) -> Setpoint {
    let limits = scurve.config();
    for _ in 0..MAX_STEPS {
        let setpoint = scurve.step(DT).unwrap();
        assert!(setpoint.velocity.abs() <= limits.v_max + MARGIN);
        assert!(setpoint.acceleration.abs() <= limits.a_max + MARGIN);
        assert!(setpoint.jerk.abs() <= limits.j_max + MARGIN);
        if setpoint.finished {
            return setpoint;
        }
    }
    panic!("S-curve move did not finish");
}

/// RON-TC-TRAJ-001 | RON-FR-500
#[test]
fn ron_tc_traj_001() {
    let mut trap = trap_to(0.0, 1.0);
    let done = run_trap_to_done(&mut trap);
    approx_eq(done.position, 1.0);
    approx_eq(done.velocity, 0.0);
    assert_eq!(trap.state().phase, TrapezoidalPhase::Done);
}

/// RON-TC-TRAJ-001, RON-TC-TRAJ-005 | RON-FR-500, RON-FR-510
#[test]
fn ron_tc_traj_001_config_validation() {
    let bad = [
        0.0,
        -1.0,
        RonFloat::NAN,
        RonFloat::INFINITY,
        RonFloat::NEG_INFINITY,
    ];
    for value in bad {
        let trap = trap_config();
        assert!(Trapezoidal::new(
            TrapezoidalConfig {
                v_max: value,
                ..trap
            },
            0.0
        )
        .is_err());
        assert!(Trapezoidal::new(
            TrapezoidalConfig {
                a_max: value,
                ..trap
            },
            0.0
        )
        .is_err());
        let scurve = scurve_config();
        assert!(SCurve::new(
            SCurveConfig {
                v_max: value,
                ..scurve
            },
            0.0
        )
        .is_err());
        assert!(SCurve::new(
            SCurveConfig {
                a_max: value,
                ..scurve
            },
            0.0
        )
        .is_err());
        assert!(SCurve::new(
            SCurveConfig {
                j_max: value,
                ..scurve
            },
            0.0
        )
        .is_err());
    }
    for position in [RonFloat::NAN, RonFloat::INFINITY] {
        assert_eq!(
            Trapezoidal::new(trap_config(), position),
            Err(TrajectoryFault::CONFIG_INVALID)
        );
        assert_eq!(
            SCurve::new(scurve_config(), position),
            Err(TrajectoryFault::CONFIG_INVALID)
        );
    }
}

/// RON-TC-TRAJ-002 | RON-FR-501
#[test]
fn ron_tc_traj_002() {
    let mut trap = Trapezoidal::new(
        TrapezoidalConfig {
            v_max: 10.0,
            a_max: 2.0,
        },
        0.0,
    )
    .unwrap();
    trap.set_target(0.1).unwrap();
    let mut peak: RonFloat = 0.0;
    let mut last = Setpoint::default();
    for _ in 0..MAX_STEPS {
        last = trap.step(DT).unwrap();
        peak = peak.max(last.velocity.abs());
        if last.finished {
            break;
        }
    }
    assert!(last.finished);
    assert!(peak < 10.0);
    approx_eq(last.position, 0.1);
    approx_eq(last.velocity, 0.0);
}

/// RON-TC-TRAJ-003 | RON-FR-502
#[test]
fn ron_tc_traj_003() {
    let mut trap = trap_to(0.0, 1.0);
    let mut previous = 0.0;
    for _ in 0..80 {
        let setpoint = trap.step(DT).unwrap();
        assert!(setpoint.position >= previous);
        assert!(setpoint.position.is_finite());
        assert!(setpoint.velocity.abs() <= 1.0 + MARGIN);
        assert!(setpoint.acceleration.abs() <= 2.0 + MARGIN);
        previous = setpoint.position;
        if setpoint.finished {
            break;
        }
    }
}

/// RON-TC-TRAJ-004 | RON-FR-503
#[test]
fn ron_tc_traj_004() {
    for (first, second, steps) in [(2.0, 1.0, 40), (2.0, -0.5, 25)] {
        let mut trap = trap_to(0.0, first);
        let mut before = Setpoint::default();
        for _ in 0..steps {
            before = trap.step(DT).unwrap();
        }
        trap.set_target(second).unwrap();
        let after = trap.step(DT).unwrap();
        assert!((after.velocity - before.velocity).abs() <= (2.0 * DT) + MARGIN);
        approx_eq(run_trap_to_done(&mut trap).position, second);
    }
    let mut reverse = trap_to(1.0, -0.25);
    let done = run_trap_to_done(&mut reverse);
    approx_eq(done.position, -0.25);
    approx_eq(done.velocity, 0.0);
}

/// RON-TC-TRAJ-005 | RON-FR-510
#[test]
fn ron_tc_traj_005() {
    let mut scurve = scurve_to(scurve_config(), 0.0, 1.0);
    let done = run_scurve_to_done(&mut scurve);
    approx_eq(done.position, 1.0);
    approx_eq(done.velocity, 0.0);
    assert_eq!(scurve.state().phase, SCurvePhase::Done);
}

/// RON-TC-TRAJ-005 | RON-FR-510
#[test]
fn ron_tc_traj_005_short_and_cruise_moves() {
    let cruise = SCurvePhase::ConstantVelocity.index().unwrap();
    let mut short = scurve_to(scurve_config(), 0.0, 0.01);
    approx_eq(short.state().phase_times[cruise], 0.0);
    approx_eq(run_scurve_to_done(&mut short).position, 0.01);

    let long_config = SCurveConfig {
        v_max: 8.0,
        a_max: 16.0,
        j_max: 2.0,
    };
    let mut long = scurve_to(long_config, 2.0, -38.0);
    assert!(long.state().phase_times[cruise] > 0.0);
    approx_eq(run_scurve_to_done(&mut long).position, -38.0);
}

/// RON-TC-TRAJ-006 | RON-FR-511
#[test]
fn ron_tc_traj_006() {
    let mut scurve = scurve_to(scurve_config(), 0.0, 1.0);
    let (mut saw_positive, mut saw_negative) = (false, false);
    for _ in 0..MAX_STEPS {
        let setpoint = scurve.step(DT).unwrap();
        saw_positive |= setpoint.jerk > 0.0;
        saw_negative |= setpoint.jerk < 0.0;
        assert!(setpoint.jerk.abs() <= 8.0 + MARGIN);
        if setpoint.finished {
            break;
        }
    }
    assert!(saw_positive && saw_negative);
}

/// RON-TC-TRAJ-007 | RON-FR-512
#[test]
fn ron_tc_traj_007() {
    let mut trap = trap_to(1.0, 1.0);
    assert!(trap.step(DT).unwrap().finished);
    let mut scurve = scurve_to(scurve_config(), 1.0, 1.0);
    assert!(scurve.step(DT).unwrap().finished);
}

/// RON-TC-TRAJ-008 | RON-FR-513
#[test]
fn ron_tc_traj_008() {
    let mut trap = trap_to(0.0, 1.0);
    let moving = trap.step(DT).unwrap();
    trap.set_hold(true).unwrap();
    let held = trap.step(DT).unwrap();
    assert_eq!(held, moving);
    assert_eq!(trap.state().phase, TrapezoidalPhase::Hold);
    trap.set_hold(false).unwrap();
    approx_eq(run_trap_to_done(&mut trap).position, 1.0);

    let mut scurve = scurve_to(scurve_config(), 0.0, 1.0);
    let moving = scurve.step(DT).unwrap();
    scurve.set_hold(true).unwrap();
    let held = scurve.step(DT).unwrap();
    assert_eq!(held, moving);
    assert!(scurve.state().hold);
    scurve.set_hold(false).unwrap();
    approx_eq(run_scurve_to_done(&mut scurve).position, 1.0);
}

/// RON-TC-TRAJ-009 | RON-FR-514
#[test]
fn ron_tc_traj_009() {
    let mut trap = trap_to(0.0, 1.0);
    let _ = trap.step(DT).unwrap();
    trap.set_hold(true).unwrap();
    trap.reset(2.5).unwrap();
    let state = trap.state();
    approx_eq(state.position, 2.5);
    approx_eq(state.target, 2.5);
    approx_eq(state.velocity, 0.0);
    approx_eq(state.acceleration, 0.0);
    assert!(state.finished && !state.hold);
    assert_eq!(trap.config(), trap_config());
    approx_eq(trap.step(DT).unwrap().position, 2.5);
    assert_eq!(
        trap.reset(RonFloat::NAN),
        Err(TrajectoryFault::CONFIG_INVALID)
    );
    approx_eq(trap.state().position, 2.5);

    let mut scurve = scurve_to(scurve_config(), 0.0, 1.0);
    let _ = scurve.step(DT).unwrap();
    scurve.set_hold(true).unwrap();
    scurve.reset(2.5).unwrap();
    let state = scurve.state();
    approx_eq(state.position, 2.5);
    approx_eq(state.target, 2.5);
    approx_eq(state.velocity, 0.0);
    approx_eq(state.jerk, 0.0);
    assert!(state.finished && !state.hold);
    approx_eq(scurve.step(DT).unwrap().position, 2.5);
    assert_eq!(
        scurve.reset(RonFloat::INFINITY),
        Err(TrajectoryFault::CONFIG_INVALID)
    );
    approx_eq(scurve.state().position, 2.5);
}

/// RON-TC-TRAJ-009 | RON-FR-512, RON-FR-514
#[test]
fn ron_tc_traj_009_faults_latch_until_reset() {
    let mut trap = trap_to(0.0, 0.5);
    for dt in [0.0, RonFloat::INFINITY] {
        assert_eq!(trap.step(dt), Err(TrajectoryFault::CONFIG_INVALID));
    }
    assert_eq!(trap.step(DT), Err(TrajectoryFault::CONFIG_INVALID));
    assert_eq!(trap.set_hold(true), Err(TrajectoryFault::CONFIG_INVALID));
    assert_eq!(trap.set_target(1.0), Err(TrajectoryFault::CONFIG_INVALID));
    trap.reset(0.0).unwrap();
    assert!(trap.step(DT).is_ok());

    let mut trap = Trapezoidal::new(trap_config(), 0.0).unwrap();
    assert_eq!(
        trap.set_target(RonFloat::INFINITY),
        Err(TrajectoryFault::CONFIG_INVALID)
    );
    assert_eq!(trap.state().fault, TrajectoryFault::CONFIG_INVALID);

    let mut scurve = scurve_to(scurve_config(), 0.0, 0.5);
    assert_eq!(scurve.step(0.0), Err(TrajectoryFault::CONFIG_INVALID));
    assert_eq!(scurve.step(DT), Err(TrajectoryFault::CONFIG_INVALID));
    assert_eq!(scurve.set_hold(true), Err(TrajectoryFault::CONFIG_INVALID));
    let mut scurve = SCurve::new(scurve_config(), 0.0).unwrap();
    assert_eq!(
        scurve.set_target(RonFloat::NAN),
        Err(TrajectoryFault::CONFIG_INVALID)
    );
    assert_eq!(scurve.step(DT), Err(TrajectoryFault::CONFIG_INVALID));
    scurve.reset(0.0).unwrap();
    assert!(scurve.state().fault.is_none());
}

/// RON-TC-TRAJ-010 | RON-FR-515
#[test]
fn ron_tc_traj_010() {
    let mut trap = trap_to(0.0, 1.0);
    let mut scurve = scurve_to(scurve_config(), 0.0, 1.0);
    let mut trap_last = Setpoint::default();
    let mut scurve_last = Setpoint::default();
    for _ in 0..5 {
        trap_last = trap.step(DT).unwrap();
        scurve_last = scurve.step(DT).unwrap();
    }
    let trap_state = trap.state();
    assert_eq!(trap_state.position.to_bits(), trap_last.position.to_bits());
    assert_eq!(trap_state.velocity.to_bits(), trap_last.velocity.to_bits());
    approx_eq(trap_state.target, 1.0);
    assert!(trap_state.peak_velocity > 0.0);
    assert!(!trap_state.finished && !trap_state.hold);
    assert!(trap_state.fault.is_none());

    let scurve_state = scurve.state();
    assert_eq!(
        scurve_state.position.to_bits(),
        scurve_last.position.to_bits()
    );
    assert_eq!(scurve_state.jerk.to_bits(), scurve_last.jerk.to_bits());
    approx_eq(scurve_state.elapsed, 5.0 * DT);
    assert!(scurve_state.total_time > scurve_state.elapsed);
    assert!(!scurve_state.finished && !scurve_state.hold);
}
