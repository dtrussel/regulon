//! # `autotune::tests`
//!
//! Traceable tests for the relay auto-tuner.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-800-RON-FR-807
//! **Tests:** RON-TC-AT-001-RON-TC-AT-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{AutotuneConfig, AutotunePhase, AutotuneResults, Autotuner, TuningRule};
use crate::{
    pid::{AntiWindupMode, Pid, PidConfig, PidMode},
    RonError, RonFloat,
};

const DT: RonFloat = 0.001;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

fn tune_config() -> AutotuneConfig {
    AutotuneConfig {
        relay_amplitude: 0.5,
        hysteresis: 0.05,
        bias: 0.0,
        min_cycles: 5,
        timeout: 30.0,
        rule: TuningRule::ZieglerNichols,
    }
}

fn pid(kp: RonFloat, ki: RonFloat, kd: RonFloat) -> Pid {
    Pid::new(PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(kp, ki, kd)
    })
    .unwrap()
}

/// Injects a synthetic sine measurement (open loop). With amplitude
/// `0.5 / pi` and period 0.5 s, `Ku = 4` and `Tu = 0.5` for `d = 0.5`.
fn run_sine(tuner: &mut Autotuner) -> Option<AutotuneResults> {
    let amplitude = 0.5 / core::f64::consts::PI;
    for k in 0_u32..200_000 {
        let time = f64::from(k) * f64::from(DT);
        #[allow(clippy::cast_possible_truncation)]
        let measurement =
            (amplitude * (2.0 * core::f64::consts::PI * time / 0.5).sin()) as RonFloat;
        let _ = tuner.step(0.0, measurement, DT).unwrap();
        if matches!(tuner.phase(), AutotunePhase::Done | AutotunePhase::Aborted) {
            break;
        }
    }
    tuner.results()
}

/// RON-TC-AT-001 | RON-FR-800
#[test]
fn ron_tc_at_001() {
    let config = AutotuneConfig {
        relay_amplitude: 1.0,
        hysteresis: 0.1,
        min_cycles: 3,
        ..tune_config()
    };
    let mut tuner = Autotuner::new(config).unwrap();
    let mut controller = pid(1.0, 0.0, 0.0);
    tuner.start(&mut controller).unwrap();
    assert_eq!(controller.mode(), PidMode::Manual);
    let mut measurement = 0.0;
    for _ in 0..60_000 {
        let output = tuner.step(0.0, measurement, DT).unwrap();
        assert!((-1.0..=1.0).contains(&output));
        measurement += (DT / 0.1) * (output - measurement);
        if tuner.phase() == AutotunePhase::Done {
            break;
        }
    }
    assert_eq!(tuner.phase(), AutotunePhase::Done);
    let results = tuner.results().unwrap();
    assert!(results.ultimate_gain > 0.0);
    assert!(results.ultimate_period > 0.0);
}

/// RON-TC-AT-002 | RON-FR-801
#[test]
fn ron_tc_at_002() {
    let base = tune_config();
    let invalid = [
        AutotuneConfig {
            relay_amplitude: 0.0,
            ..base
        },
        AutotuneConfig {
            relay_amplitude: RonFloat::NAN,
            ..base
        },
        AutotuneConfig {
            hysteresis: -0.1,
            ..base
        },
        AutotuneConfig {
            hysteresis: RonFloat::INFINITY,
            ..base
        },
        AutotuneConfig {
            bias: RonFloat::NAN,
            ..base
        },
        AutotuneConfig {
            min_cycles: 0,
            ..base
        },
        AutotuneConfig {
            timeout: 0.0,
            ..base
        },
        AutotuneConfig {
            timeout: RonFloat::INFINITY,
            ..base
        },
    ];
    for config in invalid {
        assert!(matches!(
            Autotuner::new(config),
            Err(RonError::ConfigInvalid(_))
        ));
    }
    let tuner = Autotuner::new(base).unwrap();
    assert_eq!(tuner.phase(), AutotunePhase::Idle);
    assert_eq!(tuner.results(), None);
}

/// RON-TC-AT-003 | RON-FR-802
#[test]
fn ron_tc_at_003() {
    let mut tuner = Autotuner::new(tune_config()).unwrap();
    tuner.start(&mut pid(1.0, 0.0, 0.0)).unwrap();
    let results = run_sine(&mut tuner).unwrap();
    approx_eq(results.ultimate_gain, 4.0, 0.40);
    approx_eq(results.ultimate_period, 0.5, 0.05);
}

/// RON-TC-AT-003 | RON-FR-802
#[test]
fn ron_tc_at_003_insufficient_excitation() {
    let mut tuner = Autotuner::new(AutotuneConfig {
        min_cycles: 2,
        ..tune_config()
    })
    .unwrap();
    tuner.start(&mut pid(1.0, 0.0, 0.0)).unwrap();
    for k in 0..50 {
        let measurement = if k % 2 == 0 { 1.0e-9 } else { -1.0e-9 };
        let _ = tuner.step(0.0, measurement, DT).unwrap();
        if tuner.phase() == AutotunePhase::Aborted {
            break;
        }
    }
    assert_eq!(tuner.phase(), AutotunePhase::Aborted);
    assert_eq!(tuner.results(), None);
}

/// RON-TC-AT-004 | RON-FR-803
#[test]
fn ron_tc_at_004() {
    let expected = [
        (TuningRule::ZieglerNichols, 0.60, 0.50, 0.125),
        (TuningRule::TyreusLuyben, 0.45, 2.20, 0.158),
        (TuningRule::SomeOvershoot, 0.33, 0.50, 0.333),
        (TuningRule::NoOvershoot, 0.20, 0.50, 0.333),
    ];
    for (rule, kp_factor, ti_factor, td_factor) in expected {
        let mut tuner = Autotuner::new(AutotuneConfig {
            rule,
            ..tune_config()
        })
        .unwrap();
        tuner.start(&mut pid(1.0, 0.0, 0.0)).unwrap();
        let results = run_sine(&mut tuner).unwrap();
        let kp = kp_factor * results.ultimate_gain;
        approx_eq(results.kp, kp, 1.0e-3);
        approx_eq(
            results.ki,
            kp / (ti_factor * results.ultimate_period),
            1.0e-3,
        );
        approx_eq(
            results.kd,
            kp * (td_factor * results.ultimate_period),
            1.0e-3,
        );
    }
}

/// RON-TC-AT-005 | RON-FR-804
#[test]
fn ron_tc_at_005() {
    let mut tuner = Autotuner::new(tune_config()).unwrap();
    let mut controller = pid(1.0, 0.5, 0.1);
    tuner.start(&mut controller).unwrap();
    assert!(matches!(
        tuner.apply(&mut controller),
        Err(RonError::ConfigInvalid(_))
    ));
    let results = run_sine(&mut tuner).unwrap();
    let untouched = controller.configuration();
    approx_eq(untouched.kp, 1.0, RonFloat::EPSILON);
    approx_eq(untouched.ki, 0.5, RonFloat::EPSILON);
    approx_eq(untouched.kd, 0.1, RonFloat::EPSILON);

    tuner.apply(&mut controller).unwrap();
    let tuned = controller.configuration();
    approx_eq(tuned.kp, results.kp, 1.0e-4);
    approx_eq(tuned.ki, results.ki, 1.0e-4);
    approx_eq(tuned.kd, results.kd, 1.0e-4);
    assert_eq!(controller.mode(), PidMode::Automatic);
    approx_eq(tuner.step(0.0, 0.0, DT).unwrap(), 0.0, RonFloat::EPSILON);
}

/// RON-TC-AT-006 | RON-FR-805
#[test]
fn ron_tc_at_006() {
    let mut tuner = Autotuner::new(tune_config()).unwrap();
    tuner.start(&mut pid(1.0, 0.0, 0.0)).unwrap();
    assert_eq!(tuner.results(), None);
    let results = run_sine(&mut tuner).unwrap();
    assert!(results.ultimate_gain > 0.0);
    assert!(results.ultimate_period > 0.0);
    assert_eq!(tuner.results(), Some(results));
}

/// RON-TC-AT-007 | RON-FR-806
#[test]
fn ron_tc_at_007() {
    let config = AutotuneConfig {
        bias: 2.0,
        relay_amplitude: 1.5,
        hysteresis: 0.25,
        ..tune_config()
    };
    let mut tuner = Autotuner::new(config).unwrap();
    assert!(matches!(
        tuner.step(0.0, 0.0, DT),
        Err(RonError::ConfigInvalid(_))
    ));
    tuner.start(&mut pid(1.0, 0.0, 0.0)).unwrap();
    for dt in [0.0, RonFloat::NAN] {
        assert!(matches!(
            tuner.step(0.0, 0.0, dt),
            Err(RonError::InvalidArgument(_))
        ));
    }
    for (setpoint, measurement) in [
        (RonFloat::NAN, 0.0),
        (0.0, RonFloat::INFINITY),
        (0.0, RonFloat::NEG_INFINITY),
    ] {
        assert!(tuner.step(setpoint, measurement, DT).is_err());
    }
    let (mut saw_high, mut saw_low) = (false, false);
    for i in -50_i8..=50 {
        let output = tuner.step(0.0, RonFloat::from(i) * 0.1, DT).unwrap();
        assert!((0.5..=3.5).contains(&output));
        saw_high |= (output - 3.5).abs() < RonFloat::EPSILON;
        saw_low |= (output - 0.5).abs() < RonFloat::EPSILON;
    }
    assert!(saw_high && saw_low);
}

/// RON-TC-AT-008 | RON-FR-807
#[test]
fn ron_tc_at_008() {
    let mut untouched = pid(7.0, 3.0, 1.0);
    let before = untouched;
    let mut idle = Autotuner::new(tune_config()).unwrap();
    idle.abort(&mut untouched).unwrap();
    assert_eq!(untouched, before);
    assert_eq!(idle.phase(), AutotunePhase::Aborted);

    let mut controller = pid(7.0, 3.0, 1.0);
    let mut tuner = Autotuner::new(tune_config()).unwrap();
    tuner.start(&mut controller).unwrap();
    assert_eq!(controller.mode(), PidMode::Manual);
    tuner.abort(&mut controller).unwrap();
    assert_eq!(tuner.phase(), AutotunePhase::Aborted);
    assert_eq!(controller.mode(), PidMode::Automatic);
    let restored = controller.configuration();
    approx_eq(restored.kp, 7.0, RonFloat::EPSILON);
    approx_eq(restored.ki, 3.0, RonFloat::EPSILON);
    approx_eq(restored.kd, 1.0, RonFloat::EPSILON);
    approx_eq(tuner.step(0.0, 0.0, DT).unwrap(), 0.0, RonFloat::EPSILON);

    let mut timed = Autotuner::new(AutotuneConfig {
        timeout: 0.05,
        ..tune_config()
    })
    .unwrap();
    timed.start(&mut pid(7.0, 3.0, 1.0)).unwrap();
    for _ in 0..100 {
        let _ = timed.step(0.0, 5.0, 0.01).unwrap();
        if timed.phase() == AutotunePhase::Aborted {
            break;
        }
    }
    assert_eq!(timed.phase(), AutotunePhase::Aborted);
    assert_eq!(timed.results(), None);
}

/// RON-TC-AT-001 | RON-FR-800
#[test]
fn ron_tc_at_001_restart_clears_previous_run() {
    let mut tuner = Autotuner::new(tune_config()).unwrap();
    let mut controller = pid(1.0, 0.0, 0.0);
    tuner.start(&mut controller).unwrap();
    assert!(run_sine(&mut tuner).is_some());
    tuner.start(&mut controller).unwrap();
    assert_eq!(tuner.phase(), AutotunePhase::Settling);
    assert_eq!(tuner.results(), None);
}
