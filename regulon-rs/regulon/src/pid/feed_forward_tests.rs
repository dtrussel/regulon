//! # `pid::feed_forward_tests`
//!
//! Traceable tests for the PID feed-forward path.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-200-RON-FR-205
//! **Tests:** RON-TC-FF-001-RON-TC-FF-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{
    AntiWindupMode, FeedForwardConfig, FeedForwardMode, Pid, PidConfig, PidFault, PidStatus,
    SafePolicy,
};
use crate::{RonError, RonFloat};

const TIGHT: RonFloat = 4.0 * RonFloat::EPSILON;
const LOOSE: RonFloat = 1.0e-6;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

fn base_config(kp: RonFloat) -> PidConfig {
    PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(kp, 0.0, 0.0)
    }
}

fn ff(mode: FeedForwardMode, gain: RonFloat) -> FeedForwardConfig {
    FeedForwardConfig {
        mode,
        gain,
        derivative_filter: 0.0,
    }
}

fn pid_with(kp: RonFloat, feed_forward: FeedForwardConfig) -> Pid {
    Pid::new(PidConfig {
        feed_forward,
        ..base_config(kp)
    })
    .unwrap()
}

/// RON-TC-FF-001 | RON-FR-200
#[test]
fn ron_tc_ff_001() {
    let mut pid = pid_with(2.0, ff(FeedForwardMode::StaticGain, 0.5));
    let (output, _) = pid.step(2.0, 1.0, 0.01).unwrap();
    approx_eq(output, 3.0, TIGHT);
    approx_eq(pid.last_feed_forward(), 1.0, TIGHT);
}

/// RON-TC-FF-002 | RON-FR-201
#[test]
fn ron_tc_ff_002() {
    let mut pid = pid_with(1.0, ff(FeedForwardMode::StaticGain, 0.5));
    let (output, status) = pid.step(2.0, 0.0, 0.01).unwrap();
    approx_eq(output, 3.0, TIGHT);
    assert!(status.contains(PidStatus::FEED_FORWARD_ACTIVE));
    approx_eq(pid.state().last_feed_forward, 1.0, TIGHT);
}

/// RON-TC-FF-003 | RON-FR-201
#[test]
fn ron_tc_ff_003() {
    let mut pid = pid_with(0.0, ff(FeedForwardMode::Velocity, 0.25));
    let _ = pid.step(1.0, 0.0, 0.1).unwrap();
    let (output, _) = pid.step(1.2, 0.0, 0.1).unwrap();
    approx_eq(output, 0.5, LOOSE);
}

/// RON-TC-FF-004 | RON-FR-201
#[test]
fn ron_tc_ff_004() {
    let mut pid = pid_with(0.0, ff(FeedForwardMode::Acceleration, 0.1));
    let _ = pid.step(0.0, 0.0, 0.1).unwrap();
    let _ = pid.step(0.1, 0.0, 0.1).unwrap();
    let (output, _) = pid.step(0.3, 0.0, 0.1).unwrap();
    approx_eq(output, 1.0, LOOSE);
}

/// RON-TC-FF-005 | RON-FR-201
#[test]
fn ron_tc_ff_005() {
    let mut pid = pid_with(1.0, ff(FeedForwardMode::External, 0.0));
    assert!(matches!(
        pid.step(2.0, 1.0, 0.01),
        Err(RonError::ConfigInvalid(_))
    ));
    let (output, status) = pid.step_with_feed_forward(2.0, 1.0, 0.01, 0.75).unwrap();
    approx_eq(output, 1.75, LOOSE);
    assert!(status.contains(PidStatus::FEED_FORWARD_ACTIVE));
}

/// RON-TC-FF-005 | RON-FR-201
#[test]
fn ron_tc_ff_005_rejects_invalid_external_calls() {
    let mut pid = pid_with(1.0, ff(FeedForwardMode::External, 0.0));
    for external in [RonFloat::NAN, RonFloat::INFINITY, RonFloat::NEG_INFINITY] {
        assert!(matches!(
            pid.step_with_feed_forward(0.0, 0.0, 0.1, external),
            Err(RonError::InvalidArgument(_))
        ));
    }
    for dt in [0.0, RonFloat::INFINITY] {
        assert!(pid.step_with_feed_forward(0.0, 0.0, dt, 0.0).is_err());
    }
    let mut disabled = pid_with(1.0, ff(FeedForwardMode::Disabled, 0.0));
    assert!(matches!(
        disabled.step_with_feed_forward(0.0, 0.0, 0.1, 0.0),
        Err(RonError::ConfigInvalid(_))
    ));
}

/// RON-TC-FF-005 | RON-FR-201
#[test]
fn ron_tc_ff_005_latched_fault_path() {
    let mut pid = Pid::new(PidConfig {
        feed_forward: ff(FeedForwardMode::External, 0.0),
        safe_policy: SafePolicy::DriveZero,
        ..base_config(1.0)
    })
    .unwrap();
    assert_eq!(
        pid.step_with_feed_forward(RonFloat::NAN, 0.0, 0.1, 0.0),
        Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
    );
    assert_eq!(
        pid.step_with_feed_forward(0.0, 0.0, 0.1, 0.0),
        Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
    );
    approx_eq(pid.last_output(), 0.0, LOOSE);
}

/// RON-TC-FF-006 | RON-FR-202
#[test]
fn ron_tc_ff_006() {
    let mut raw = pid_with(0.0, ff(FeedForwardMode::Velocity, 1.0));
    let mut filtered = pid_with(
        0.0,
        FeedForwardConfig {
            derivative_filter: 1.0,
            ..ff(FeedForwardMode::Velocity, 1.0)
        },
    );
    let _ = raw.step(0.0, 0.0, 0.1).unwrap();
    let _ = filtered.step(0.0, 0.0, 0.1).unwrap();
    let (raw_output, _) = raw.step(1.0, 0.0, 0.1).unwrap();
    let (filtered_output, _) = filtered.step(1.0, 0.0, 0.1).unwrap();
    approx_eq(raw.last_derivative(), 0.0, LOOSE);
    approx_eq(filtered.last_derivative(), 0.0, LOOSE);
    assert!(filtered_output > 0.0);
    assert!(filtered_output < raw_output);
}

/// RON-TC-FF-006 | RON-FR-202
#[test]
fn ron_tc_ff_006_config_validation() {
    for mode in [
        FeedForwardMode::Disabled,
        FeedForwardMode::StaticGain,
        FeedForwardMode::Velocity,
        FeedForwardMode::Acceleration,
        FeedForwardMode::External,
    ] {
        assert_eq!(ff(mode, 0.0).validate(), Ok(()));
    }
    assert_eq!(ff(FeedForwardMode::StaticGain, -2.0).validate(), Ok(()));
    for gain in [RonFloat::NAN, RonFloat::INFINITY, RonFloat::NEG_INFINITY] {
        assert!(ff(FeedForwardMode::StaticGain, gain).validate().is_err());
    }
    for bandwidth in [-1.0, RonFloat::NAN] {
        let config = FeedForwardConfig {
            derivative_filter: bandwidth,
            ..ff(FeedForwardMode::StaticGain, 1.0)
        };
        assert!(config.validate().is_err());
        assert!(Pid::new(PidConfig {
            feed_forward: config,
            ..base_config(1.0)
        })
        .is_err());
    }
}

/// RON-TC-FF-007 | RON-FR-203
#[test]
fn ron_tc_ff_007() {
    let mut pid = Pid::new(PidConfig {
        output_max: 5.0,
        rate_limit: 10.0,
        feed_forward: ff(FeedForwardMode::StaticGain, 100.0),
        ..base_config(0.0)
    })
    .unwrap();
    let (output, status) = pid.step(1.0, 0.0, 0.1).unwrap();
    approx_eq(output, 1.0, LOOSE);
    assert!(status.contains(PidStatus::SATURATED));
    assert!(status.contains(PidStatus::RATE_LIMITED));
}

/// RON-TC-FF-008 | RON-FR-204
#[test]
fn ron_tc_ff_008() {
    let plain_config = PidConfig {
        ki: 0.5,
        ..base_config(1.25)
    };
    let mut plain = Pid::new(plain_config).unwrap();
    let mut disabled = Pid::new(PidConfig {
        feed_forward: ff(FeedForwardMode::Disabled, 0.0),
        ..plain_config
    })
    .unwrap();
    for index in 0_u16..1_000 {
        let setpoint = RonFloat::from(index % 17) * 0.1;
        let measurement = RonFloat::from(index % 11) * 0.05;
        let (plain_output, plain_status) = plain.step(setpoint, measurement, 0.01).unwrap();
        let (disabled_output, disabled_status) =
            disabled.step(setpoint, measurement, 0.01).unwrap();
        approx_eq(plain_output, disabled_output, RonFloat::EPSILON);
        assert_eq!(plain_status, disabled_status);
    }
    assert_eq!(disabled.last_feed_forward().to_bits(), 0);
}

/// RON-TC-FF-008 | RON-FR-204
#[test]
fn ron_tc_ff_008_zero_gain_static_is_inactive() {
    let mut pid = pid_with(1.0, ff(FeedForwardMode::StaticGain, 0.0));
    let (output, status) = pid.step(2.0, 1.0, 0.01).unwrap();
    approx_eq(output, 1.0, LOOSE);
    assert!(!status.contains(PidStatus::FEED_FORWARD_ACTIVE));
    approx_eq(pid.last_feed_forward(), 0.0, LOOSE);
}

/// RON-TC-FF-009 | RON-FR-205
#[test]
fn ron_tc_ff_009() {
    let mut pid = pid_with(1.0, FeedForwardConfig::default());
    pid.set_feed_forward(ff(FeedForwardMode::StaticGain, 0.25))
        .unwrap();
    let (_, status) = pid.step(4.0, 1.0, 0.01).unwrap();
    assert!(status.contains(PidStatus::FEED_FORWARD_ACTIVE));
    approx_eq(pid.last_feed_forward(), 1.0, TIGHT);
}

/// RON-TC-FF-009 | RON-FR-205
#[test]
fn ron_tc_ff_009_set_rejects_invalid_and_resets_state() {
    let mut pid = pid_with(1.0, ff(FeedForwardMode::StaticGain, 0.5));
    let _ = pid.step(2.0, 0.0, 0.01).unwrap();
    let before = pid.configuration();
    assert!(pid
        .set_feed_forward(ff(FeedForwardMode::StaticGain, RonFloat::NAN))
        .is_err());
    assert_eq!(pid.configuration(), before);
    approx_eq(pid.last_feed_forward(), 1.0, TIGHT);
    pid.set_feed_forward(ff(FeedForwardMode::Velocity, 1.0))
        .unwrap();
    assert_eq!(pid.last_feed_forward().to_bits(), 0);
    assert!(!pid.state().status.contains(PidStatus::FEED_FORWARD_ACTIVE));
}
