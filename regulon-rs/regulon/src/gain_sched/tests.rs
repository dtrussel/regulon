//! # `gain_sched::tests`
//!
//! Traceable tests for gain scheduling.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-300-RON-FR-306
//! **Tests:** RON-TC-GS-001-RON-TC-GS-008
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{GainSchedule, ScheduleMode, GS_MAX_BREAKPOINTS};
use crate::{
    pid::{
        AntiWindupMode, FeedForwardConfig, FeedForwardMode, NormalizationConfig, Pid, PidConfig,
    },
    RonError, RonFloat,
};

const TOL: RonFloat = 1.0e-5;

fn approx_eq(lhs: RonFloat, rhs: RonFloat) {
    assert!((lhs - rhs).abs() <= TOL, "{lhs} != {rhs}");
}

fn base(kp: RonFloat) -> PidConfig {
    PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(kp, 0.0, 0.0)
    }
}

fn hard<const N: usize>(sigma: [RonFloat; N], configs: [PidConfig; N]) -> GainSchedule<N> {
    GainSchedule::new(sigma, configs, ScheduleMode::HardSwitch, false).unwrap()
}

fn interp_err(lhs: PidConfig, rhs: PidConfig) -> bool {
    GainSchedule::new(
        [0.0, 1.0],
        [lhs, rhs],
        ScheduleMode::LinearInterpolation,
        false,
    )
    .is_err()
}

/// RON-TC-GS-001 | RON-FR-300
#[test]
fn ron_tc_gs_001() {
    let low = PidConfig {
        output_min: -2.0,
        output_max: 2.0,
        ..base(1.0)
    };
    let high = PidConfig {
        kp: 4.0,
        output_min: -4.0,
        output_max: 4.0,
        ..low
    };
    let table = hard([0.0, 1.0], [low, high]);
    let mut pid = Pid::new(low).unwrap();
    table.apply(&mut pid, 1.5).unwrap();
    assert_eq!(pid.configuration(), high);
    let (output, _) = pid.step(2.0, 0.0, 0.01).unwrap();
    approx_eq(output, 4.0);
}

/// RON-TC-GS-002 | RON-FR-301
#[test]
fn ron_tc_gs_002() {
    let mut sigma = [0.0; GS_MAX_BREAKPOINTS];
    let mut configs = [base(1.0); GS_MAX_BREAKPOINTS];
    for (index, (point, config)) in sigma.iter_mut().zip(configs.iter_mut()).enumerate() {
        let value = RonFloat::from(u8::try_from(index).unwrap());
        *point = value;
        config.kp = 1.0 + value;
    }
    assert!(GainSchedule::new(sigma, configs, ScheduleMode::HardSwitch, false).is_ok());

    let too_many = [base(1.0); GS_MAX_BREAKPOINTS + 1];
    let mut sigma_too_many = [0.0; GS_MAX_BREAKPOINTS + 1];
    for (index, point) in sigma_too_many.iter_mut().enumerate() {
        *point = RonFloat::from(u8::try_from(index).unwrap());
    }
    assert!(matches!(
        GainSchedule::new(sigma_too_many, too_many, ScheduleMode::HardSwitch, false),
        Err(RonError::ConfigInvalid(_))
    ));
}

/// RON-TC-GS-003 | RON-FR-302
#[test]
fn ron_tc_gs_003() {
    let table = hard([0.0, 1.0, 2.0], [base(1.0), base(2.0), base(3.0)]);
    let mut pid = Pid::new(base(1.0)).unwrap();
    for (sigma, expected) in [(0.5, 1.0), (1.5, 2.0), (2.5, 3.0)] {
        table.apply(&mut pid, sigma).unwrap();
        approx_eq(pid.configuration().kp, expected);
        let (output, _) = pid.step(1.0, 0.0, 0.01).unwrap();
        approx_eq(output, expected);
    }
}

/// RON-TC-GS-004 | RON-FR-302
#[test]
fn ron_tc_gs_004() {
    let table = GainSchedule::new(
        [0.0, 1.0],
        [base(1.0), base(3.0)],
        ScheduleMode::LinearInterpolation,
        false,
    )
    .unwrap();
    let mut pid = Pid::new(base(1.0)).unwrap();
    for (sigma, expected) in [(-0.25, 1.0), (0.25, 1.5), (0.75, 2.5), (1.25, 3.0)] {
        table.apply(&mut pid, sigma).unwrap();
        approx_eq(pid.configuration().kp, expected);
    }
}

/// RON-TC-GS-004 | RON-FR-302
#[test]
fn ron_tc_gs_004_single_point_interp_table() {
    let table = GainSchedule::new(
        [0.5],
        [base(1.25)],
        ScheduleMode::LinearInterpolation,
        false,
    )
    .unwrap();
    let mut pid = Pid::new(base(1.25)).unwrap();
    table.apply(&mut pid, 0.75).unwrap();
    approx_eq(pid.configuration().kp, 1.25);
}

/// RON-TC-GS-005 | RON-FR-303
#[test]
fn ron_tc_gs_005() {
    let low = base(0.5);
    let high = PidConfig {
        kp: 3.0,
        output_min: -2.0,
        output_max: 2.0,
        ..low
    };
    let table = hard([0.0, 1.0], [low, high]);
    let mut pid = Pid::new(low).unwrap();
    table.apply(&mut pid, 1.0).unwrap();
    assert_eq!(pid.configuration(), high);

    // A table that fails validation can never be built, let alone applied,
    // and a rejected scheduling variable leaves the controller untouched.
    let invalid = PidConfig {
        anti_windup_mode: AntiWindupMode::BackCalculation,
        ..high
    };
    assert!(interp_err(high, invalid));
    assert!(table.apply(&mut pid, RonFloat::NAN).is_err());
    assert_eq!(pid.configuration(), high);
}

/// RON-TC-GS-006 | RON-FR-304
#[test]
fn ron_tc_gs_006() {
    let table = hard([0.0, 1.0], [base(1.0), base(5.0)]);
    let mut pid = Pid::new(base(1.0)).unwrap();
    let _ = pid.step(1.0, 0.0, 0.01).unwrap();
    let (before, _) = pid.step(1.0, 0.0, 0.01).unwrap();
    approx_eq(before, 1.0);
    table.apply(&mut pid, 1.0).unwrap();
    let (after, _) = pid.step(1.0, 0.0, 0.01).unwrap();
    approx_eq(after, 5.0);
}

/// RON-TC-GS-007 | RON-FR-305
#[test]
fn ron_tc_gs_007() {
    let first = PidConfig {
        ki: 1.0,
        integral_min: -10.0,
        integral_max: 10.0,
        ..base(0.0)
    };
    let second = PidConfig { kp: 2.0, ..first };
    let reset =
        GainSchedule::new([0.0, 1.0], [first, second], ScheduleMode::HardSwitch, true).unwrap();
    let keep = hard([0.0, 1.0], [first, second]);
    let mut reset_pid = Pid::new(first).unwrap();
    let mut keep_pid = Pid::new(first).unwrap();
    reset_pid.set_integral(0.75).unwrap();
    keep_pid.set_integral(0.75).unwrap();

    reset.apply(&mut reset_pid, 0.0).unwrap();
    approx_eq(reset_pid.integral(), 0.75);
    reset.apply(&mut reset_pid, 1.0).unwrap();
    approx_eq(reset_pid.integral(), 0.0);
    keep.apply(&mut keep_pid, 1.0).unwrap();
    approx_eq(keep_pid.integral(), 0.75);
}

/// RON-TC-GS-008 | RON-FR-306
#[test]
fn ron_tc_gs_008() {
    let config = base(1.0);
    let invalid = PidConfig { kp: -1.0, ..config };
    let hard_mode = ScheduleMode::HardSwitch;
    assert!(GainSchedule::<0>::new([], [], hard_mode, false).is_err());
    assert!(GainSchedule::new([1.0, 0.0], [config; 2], hard_mode, false).is_err());
    assert!(GainSchedule::new([0.0, 0.0], [config; 2], hard_mode, false).is_err());
    assert!(GainSchedule::new([0.0, 1.0], [config, invalid], hard_mode, false).is_err());
    assert!(GainSchedule::new([RonFloat::INFINITY], [config], hard_mode, false).is_err());
    assert!(GainSchedule::new([0.0], [invalid], hard_mode, false).is_err());
    assert!(GainSchedule::new([0.0, RonFloat::INFINITY], [config; 2], hard_mode, false).is_err());
    assert!(GainSchedule::new([0.0, RonFloat::NAN], [config; 2], hard_mode, false).is_err());
    assert!(interp_err(
        config,
        PidConfig {
            kp: 3.0,
            anti_windup_mode: AntiWindupMode::BackCalculation,
            ..config
        }
    ));

    let table = hard([0.0, 1.0], [config, base(2.0)]);
    let mut pid = Pid::new(config).unwrap();
    for sigma in [RonFloat::NAN, RonFloat::INFINITY, RonFloat::NEG_INFINITY] {
        assert!(matches!(
            table.apply(&mut pid, sigma),
            Err(RonError::InvalidArgument(_))
        ));
    }
}

/// RON-TC-GS-008 | RON-FR-306
#[test]
fn ron_tc_gs_008_interp_validation_groups() {
    let config = PidConfig {
        derivative_filter: 8.0,
        output_min: -10.0,
        output_max: 10.0,
        rate_limit: 20.0,
        integral_min: -5.0,
        integral_max: 5.0,
        anti_windup_mode: AntiWindupMode::BackCalculation,
        anti_windup_tracking_time: 0.2,
        setpoint_filter_tau: 0.1,
        normalization: Some(NormalizationConfig {
            input_min: -1.0,
            input_max: 1.0,
            output_min: -10.0,
            output_max: 10.0,
            range: crate::pid::NormalizationRange::NegativeOneToOne,
        }),
        integral_overflow_threshold: 100.0,
        setpoint_reset_threshold: 50.0,
        feed_forward: FeedForwardConfig {
            mode: FeedForwardMode::StaticGain,
            gain: 0.5,
            derivative_filter: 4.0,
        },
        ..base(1.0)
    };
    assert!(!interp_err(config, PidConfig { kp: 2.0, ..config }));
    assert!(interp_err(
        config,
        PidConfig {
            derivative_filter: 9.0,
            ..config
        }
    ));
    assert!(interp_err(
        config,
        PidConfig {
            output_min: -9.0,
            ..config
        }
    ));
    let mut shifted = config;
    if let Some(normalization) = shifted.normalization.as_mut() {
        normalization.input_min = -2.0;
    }
    assert!(interp_err(config, shifted));
    let mut velocity = config;
    velocity.feed_forward.mode = FeedForwardMode::Velocity;
    assert!(interp_err(config, velocity));
}
