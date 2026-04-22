//! # `pid::tests`
//!
//! Traceable PID and safety tests for the Rust-first baseline.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-SR-001-RON-SR-021
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{
    AntiWindupMode, DerivativeMode, FeedForwardConfig, FeedForwardMode, IntegrationMethod,
    NormalizationConfig, NormalizationRange, Pid, PidConfig, PidFault, PidMode, PidStatus,
    SafePolicy,
};
use crate::{RonError, RonFloat};
use std::format;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

fn base_config() -> PidConfig {
    PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(0.0, 0.0, 0.0)
    }
}

fn kp_only_config(kp: RonFloat) -> PidConfig {
    PidConfig {
        kp,
        ..base_config()
    }
}

/// RON-TC-PID-001 | RON-FR-001
#[test]
fn ron_tc_pid_001() {
    let mut pid = Pid::new(kp_only_config(2.0)).unwrap();
    let (output, status) = pid.step(1.0, 0.0, 0.01).unwrap();
    approx_eq(output, 2.0, 4.0 * RonFloat::EPSILON);
    assert_eq!(status, PidStatus::OK);
}

/// RON-TC-PID-002 | RON-FR-002
#[test]
fn ron_tc_pid_002() {
    let mut parallel = Pid::new(PidConfig {
        kp: 2.0,
        ki: 0.4,
        kd: 0.2,
        ..base_config()
    })
    .unwrap();
    let mut ideal = Pid::new(PidConfig::new_ideal(2.0, 5.0, 0.1)).unwrap();
    for _ in 0..50 {
        let (parallel_output, _) = parallel.step(1.0, 0.5, 0.01).unwrap();
        let (ideal_output, _) = ideal.step(1.0, 0.5, 0.01).unwrap();
        approx_eq(parallel_output, ideal_output, 32.0 * RonFloat::EPSILON);
    }
}

/// RON-TC-PID-004 | RON-FR-004
#[test]
fn ron_tc_pid_004() {
    let mut pid = Pid::new(PidConfig {
        ki: 2.0,
        integration_method: IntegrationMethod::Euler,
        ..base_config()
    })
    .unwrap();
    let (output, _) = pid.step(1.0, 0.0, 0.5).unwrap();
    approx_eq(output, 1.0, 16.0 * RonFloat::EPSILON);
}

/// RON-TC-PID-005 | RON-FR-004
#[test]
fn ron_tc_pid_005() {
    let mut pid = Pid::new(PidConfig {
        ki: 2.0,
        integration_method: IntegrationMethod::Trapezoidal,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(1.0, 0.0, 0.5).unwrap();
    let (output, _) = pid.step(1.0, 0.0, 0.5).unwrap();
    approx_eq(output, 1.5, 16.0 * RonFloat::EPSILON);
}

/// RON-TC-PID-006 | RON-FR-005
#[test]
fn ron_tc_pid_006() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        kd: 2.0,
        derivative_mode: DerivativeMode::OnError,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(0.5, 0.5, 0.01).unwrap();
    let (output, _) = pid.step(1.0, 0.5, 0.01).unwrap();
    assert!(output > 50.0);
}

/// RON-TC-PID-007 | RON-FR-005
#[test]
fn ron_tc_pid_007() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        kd: 2.0,
        derivative_mode: DerivativeMode::OnMeasurement,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(0.5, 0.5, 0.01).unwrap();
    let (output, _) = pid.step(1.0, 0.5, 0.01).unwrap();
    approx_eq(output, 0.5, 64.0 * RonFloat::EPSILON);
}

/// RON-TC-PID-010 | RON-FR-010-RON-FR-013
#[test]
fn ron_tc_pid_010() {
    let normalization = NormalizationConfig {
        input_min: 0.0,
        input_max: 10.0,
        output_min: -100.0,
        output_max: 100.0,
        range: NormalizationRange::NegativeOneToOne,
    };
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        normalization: Some(normalization),
        output_min: -100.0,
        output_max: 100.0,
        ..base_config()
    })
    .unwrap();
    let (output, status) = pid.step(10.0, 5.0, 0.01).unwrap();
    approx_eq(output, 100.0, 0.001);
    assert!(status.contains(PidStatus::NORMALIZED));
}

/// RON-TC-FF-002 | RON-FR-201
#[test]
fn ron_tc_ff_002() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        feed_forward: FeedForwardConfig {
            mode: FeedForwardMode::StaticGain,
            static_gain: 0.5,
        },
        ..base_config()
    })
    .unwrap();
    let (output, status) = pid.step(2.0, 0.0, 0.01).unwrap();
    approx_eq(output, 3.0, 4.0 * RonFloat::EPSILON);
    assert!(status.contains(PidStatus::FEED_FORWARD_ACTIVE));
    approx_eq(pid.state().last_feed_forward, 1.0, 4.0 * RonFloat::EPSILON);
}

/// RON-TC-FF-008 | RON-FR-204
#[test]
fn ron_tc_ff_008() {
    let mut disabled = Pid::new(PidConfig {
        kp: 1.0,
        feed_forward: FeedForwardConfig {
            mode: FeedForwardMode::Disabled,
            static_gain: 0.0,
        },
        ..base_config()
    })
    .unwrap();
    let mut legacy = Pid::new(PidConfig {
        kp: 1.0,
        ..base_config()
    })
    .unwrap();
    for _ in 0..1_000 {
        let (disabled_output, disabled_status) = disabled.step(0.5, 0.1, 0.01).unwrap();
        let (legacy_output, legacy_status) = legacy.step(0.5, 0.1, 0.01).unwrap();
        approx_eq(disabled_output, legacy_output, 4.0 * RonFloat::EPSILON);
        assert_eq!(disabled_status, legacy_status);
    }
    assert_eq!(disabled.state().last_feed_forward, 0.0);
}

/// RON-TC-PID-015 | RON-FR-020
#[test]
fn ron_tc_pid_015() {
    let mut pid = Pid::new(PidConfig {
        kp: 1_000.0,
        output_min: -10.0,
        output_max: 10.0,
        ..base_config()
    })
    .unwrap();
    for _ in 0..20 {
        let (output, status) = pid.step(100.0, 0.0, 0.01).unwrap();
        assert_eq!(output, 10.0);
        assert!(status.contains(PidStatus::SATURATED));
    }
}

/// RON-TC-PID-016 | RON-FR-021
#[test]
fn ron_tc_pid_016() {
    let mut pid = Pid::new(kp_only_config(10.0)).unwrap();
    pid.set_limits(-1.0, 1.0).unwrap();
    let (output, _) = pid.step(1.0, 0.0, 0.01).unwrap();
    assert_eq!(output, 1.0);
}

/// RON-TC-PID-017 | RON-FR-053
#[test]
fn ron_tc_pid_017() {
    let mut pid = Pid::new(base_config()).unwrap();
    pid.set_gains(2.0, 3.0, 4.0).unwrap();
    pid.set_filter(5.0).unwrap();
    pid.set_anti_windup(AntiWindupMode::ConditionalIntegration, 0.2)
        .unwrap();
    let config = pid.configuration();
    assert_eq!(config.kp, 2.0);
    assert_eq!(config.ki, 3.0);
    assert_eq!(config.kd, 4.0);
    assert_eq!(config.derivative_filter, 5.0);
    assert_eq!(
        config.anti_windup_mode,
        AntiWindupMode::ConditionalIntegration
    );
}

/// RON-TC-PID-022 | RON-FR-031
#[test]
fn ron_tc_pid_022() {
    let aw_config = PidConfig {
        kp: 1.0,
        ki: 10.0,
        output_min: -5.0,
        output_max: 5.0,
        anti_windup_mode: AntiWindupMode::BackCalculation,
        anti_windup_tracking_time: 0.1,
        ..base_config()
    };
    let mut pid = Pid::new(aw_config).unwrap();
    let mut no_aw = Pid::new(PidConfig {
        anti_windup_mode: AntiWindupMode::Disabled,
        ..aw_config
    })
    .unwrap();
    for _ in 0..100 {
        let _ = pid.step(100.0, 0.0, 0.01).unwrap();
        let _ = no_aw.step(100.0, 0.0, 0.01).unwrap();
    }
    let mut measurement_aw = 0.0;
    let mut measurement_no_aw = 0.0;
    let mut final_output_aw = 0.0;
    let mut final_output_no_aw = 0.0;
    for _ in 0..100 {
        let (output_aw, _) = pid.step(0.0, measurement_aw, 0.01).unwrap();
        let (output_no_aw, _) = no_aw.step(0.0, measurement_no_aw, 0.01).unwrap();
        measurement_aw += 0.2 * (output_aw - measurement_aw);
        measurement_no_aw += 0.2 * (output_no_aw - measurement_no_aw);
        final_output_aw = output_aw;
        final_output_no_aw = output_no_aw;
    }
    assert!(final_output_aw.abs() < final_output_no_aw.abs());
    assert!(measurement_aw.abs() < measurement_no_aw.abs());
}

/// RON-TC-PID-025 | RON-FR-034
#[test]
fn ron_tc_pid_025() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        ki: 10.0,
        output_min: -5.0,
        output_max: 5.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..base_config()
    })
    .unwrap();
    for _ in 0..100 {
        let _ = pid.step(100.0, 0.0, 0.01).unwrap();
    }
    let mut settling_step = None;
    for step in 0..100 {
        let (output, _) = pid.step(0.0, 0.0, 0.01).unwrap();
        if output.abs() < 0.1 {
            settling_step = Some(step);
            break;
        }
    }
    assert!(settling_step.is_none() || settling_step.unwrap() > 20);
}

/// RON-TC-PID-028 | RON-FR-041
#[test]
fn ron_tc_pid_028() {
    let mut pid = Pid::new(PidConfig {
        kp: 2.0,
        ki: 1.0,
        initial_mode: PidMode::Manual,
        ..base_config()
    })
    .unwrap();
    pid.set_mode(PidMode::Manual, 3.5).unwrap();
    for _ in 0..10 {
        let (output, status) = pid.step(0.0, 0.0, 0.01).unwrap();
        assert_eq!(output, 3.5);
        assert!(status.contains(PidStatus::MANUAL_MODE));
    }
    pid.set_mode(PidMode::Automatic, 3.5).unwrap();
    let (output, _) = pid.step(0.0, 0.0, 0.01).unwrap();
    assert!(((output - 3.5) / 3.5).abs() < 0.05);
}

/// RON-TC-PID-030 | RON-FR-050
#[test]
fn ron_tc_pid_030() {
    let pid = Pid::new(base_config()).unwrap();
    let state = pid.state();
    assert_eq!(state.integral, 0.0);
    assert_eq!(state.last_output, 0.0);
    assert_eq!(state.fault, PidFault::NONE);
}

/// RON-TC-PID-031 | RON-FR-051
#[test]
fn ron_tc_pid_031() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        ki: 1.0,
        kd: 1.0,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(1.0, 0.0, 0.01).unwrap();
    pid.reset();
    let state = pid.state();
    assert_eq!(state.integral, 0.0);
    assert_eq!(state.last_derivative, 0.0);
    assert_eq!(state.last_output, 0.0);
}

/// RON-TC-PID-033 | RON-FR-053
#[test]
fn ron_tc_pid_033() {
    let mut pid = Pid::new(kp_only_config(1.0)).unwrap();
    pid.update_configuration(PidConfig {
        kp: 3.0,
        ki: 0.0,
        kd: 0.0,
        output_min: -2.0,
        output_max: 2.0,
        ..base_config()
    })
    .unwrap();
    let (output, status) = pid.step(1.0, 0.0, 0.01).unwrap();
    assert_eq!(output, 2.0);
    assert!(status.contains(PidStatus::SATURATED));
}

/// RON-TC-PID-034 | RON-FR-052
#[test]
fn ron_tc_pid_034() {
    let mut pid = Pid::new(base_config()).unwrap();
    pid.set_integral(4.0).unwrap();
    assert_eq!(pid.integral(), 4.0);
}

/// RON-TC-PID-037 | RON-FR-061
#[test]
fn ron_tc_pid_037() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        kd: 1.0,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(1.0, 0.0, 0.01).unwrap();
    assert_eq!(pid.mode(), PidMode::Automatic);
    assert!(pid.last_output() > 0.0);
    assert!(pid.last_derivative().is_finite());
}

/// RON-TC-PID-035 | RON-FR-060-RON-FR-062
#[test]
fn ron_tc_pid_035() {
    let mut pid_a_interleaved = Pid::new(PidConfig {
        kp: 1.0,
        ki: 0.1,
        ..base_config()
    })
    .unwrap();
    let mut pid_b_interleaved = Pid::new(PidConfig {
        kp: 3.0,
        ki: 0.5,
        kd: 0.2,
        ..base_config()
    })
    .unwrap();
    let mut pid_a_alone = pid_a_interleaved;
    let mut pid_b_alone = pid_b_interleaved;
    let inputs_a = [(1.0, 0.1), (0.8, 0.2), (1.2, 0.3), (0.5, 0.0)];
    let inputs_b = [(0.2, 0.0), (0.4, 0.1), (0.1, -0.1), (0.0, 0.2)];
    for index in 0..inputs_a.len() {
        let (output_a_interleaved, _) = pid_a_interleaved
            .step(inputs_a[index].0, inputs_a[index].1, 0.01)
            .unwrap();
        let (output_b_interleaved, _) = pid_b_interleaved
            .step(inputs_b[index].0, inputs_b[index].1, 0.01)
            .unwrap();
        let (output_a_alone, _) = pid_a_alone
            .step(inputs_a[index].0, inputs_a[index].1, 0.01)
            .unwrap();
        let (output_b_alone, _) = pid_b_alone
            .step(inputs_b[index].0, inputs_b[index].1, 0.01)
            .unwrap();
        approx_eq(
            output_a_interleaved,
            output_a_alone,
            16.0 * RonFloat::EPSILON,
        );
        approx_eq(
            output_b_interleaved,
            output_b_alone,
            16.0 * RonFloat::EPSILON,
        );
    }
}

/// RON-TC-PID-038 | RON-FR-070
#[test]
fn ron_tc_pid_038() {
    let mut pid = Pid::new(PidConfig {
        kp: 100.0,
        ki: 10.0,
        output_min: -1.0,
        output_max: 1.0,
        rate_limit: 10.0,
        anti_windup_mode: AntiWindupMode::BackCalculation,
        anti_windup_tracking_time: 0.1,
        ..base_config()
    })
    .unwrap();
    let (_, status) = pid.step(1.0, 0.0, 0.01).unwrap();
    assert!(status.contains(PidStatus::SATURATED));
    assert!(status.contains(PidStatus::RATE_LIMITED));
    assert!(status.contains(PidStatus::ANTI_WINDUP_ACTIVE));
}

/// RON-TC-PID-039 | RON-FR-071
#[test]
fn ron_tc_pid_039() {
    let mut pid = Pid::new(PidConfig {
        kp: 1.0,
        ki: 1.0,
        kd: 0.5,
        ..base_config()
    })
    .unwrap();
    let _ = pid.step(1.0, 0.0, 0.01).unwrap();
    let state = pid.state();
    assert!(state.integral > 0.0);
    assert!(state.last_output > 0.0);
}

/// RON-TC-SAFE-007 | RON-SR-010
#[test]
fn ron_tc_safe_007() {
    let mut pid = Pid::new(PidConfig {
        safe_policy: SafePolicy::DriveZero,
        ..base_config()
    })
    .unwrap();
    let fault = pid.step(RonFloat::NAN, 0.0, 0.01).unwrap_err();
    assert_eq!(fault, RonError::Fault(PidFault::INPUT_NOT_FINITE));
    let state = pid.state();
    assert!(state.status.contains(PidStatus::FAULT));

    let invalid = Pid::new(PidConfig {
        output_min: 1.0,
        output_max: 1.0,
        ..base_config()
    });
    assert!(matches!(invalid, Err(RonError::ConfigInvalid(_))));
}

/// RON-TC-SAFE-009 | RON-SR-012
#[test]
fn ron_tc_safe_009() {
    let mut pid = Pid::new(base_config()).unwrap();
    let _ = pid.step(RonFloat::NAN, 0.0, 0.01);
    for _ in 0..5 {
        let result = pid.step(0.0, 0.0, 0.01);
        assert!(matches!(result, Err(RonError::Fault(_))));
    }
    pid.clear_fault();
    let result = pid.step(0.0, 0.0, 0.01);
    assert!(result.is_ok());
}

/// RON-TC-SAFE-011 | RON-SR-020
#[test]
fn ron_tc_safe_011() {
    let mut pid = Pid::new(base_config()).unwrap();
    assert!(matches!(
        pid.step(0.0, RonFloat::INFINITY, 0.01),
        Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
    ));
    pid.clear_fault();
    assert!(matches!(
        pid.step(0.0, 0.0, RonFloat::INFINITY),
        Err(RonError::Fault(PidFault::INPUT_NOT_FINITE))
    ));
}

/// RON-TC-SAFE-012 | RON-SR-021
#[test]
fn ron_tc_safe_012() {
    let mut pid = Pid::new(PidConfig {
        ki: 1.0,
        ..base_config()
    })
    .unwrap();
    for _ in 0..1_000_000 {
        let _ = pid.step(0.001, 0.0, 0.001).unwrap();
    }
    let state = pid.state();
    let true_integral = 1.0;
    let relative_error = ((state.integral - true_integral) / true_integral).abs();
    assert!(relative_error < 0.001);
}

/// RON-TC-SAFE-013 | RON-SR-001-RON-SR-006
#[test]
fn ron_tc_safe_013() {
    let mut pid = Pid::new(base_config()).unwrap();
    assert!(matches!(
        pid.set_mode(PidMode::Manual, RonFloat::NAN),
        Err(RonError::InvalidArgument(_))
    ));
    assert!(matches!(
        pid.set_integral(RonFloat::INFINITY),
        Err(RonError::InvalidArgument(_))
    ));
    let rendered = format!("{}", RonError::InvalidArgument("bad"));
    assert!(rendered.contains("invalid PID argument"));
    let rendered_fault = format!("{}", RonError::Fault(PidFault::CONFIG_INVALID));
    assert!(rendered_fault.contains("PID fault latched"));
}
