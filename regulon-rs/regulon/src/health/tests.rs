//! # `health::tests`
//!
//! Traceable tests for the loop-health monitor.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-900-RON-FR-905
//! **Tests:** RON-TC-HLTH-001-RON-TC-HLTH-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use super::{HealthConfig, HealthMonitor, HealthStatus, HEALTH_OSC_WINDOW};
use crate::{
    pid::{AntiWindupMode, Pid, PidConfig},
    RonError, RonFloat,
};

const DT: RonFloat = 0.01;

fn quiet_config() -> HealthConfig {
    HealthConfig {
        stuck_time: 100.0,
        divergence_threshold: 100.0,
        oscillation_count_threshold: 31,
        dead_band: 1.0e-4,
        dropout_time: 100.0,
        steady_state_threshold: 100.0,
        settling_time: 100.0,
        callback: None,
    }
}

fn step_index(k: u16) -> RonFloat {
    RonFloat::from(k)
}

/// RON-TC-HLTH-001 | RON-FR-900
#[test]
fn ron_tc_hlth_001() {
    let monitor = HealthMonitor::new(quiet_config()).unwrap();
    assert!(monitor.status().is_ok());

    let bad_positive = [
        0.0,
        -1.0,
        RonFloat::NAN,
        RonFloat::INFINITY,
        RonFloat::NEG_INFINITY,
    ];
    let bad_non_negative = [-1.0, RonFloat::NAN, RonFloat::INFINITY];
    let base = quiet_config();
    for value in bad_positive {
        for config in [
            HealthConfig {
                stuck_time: value,
                ..base
            },
            HealthConfig {
                dropout_time: value,
                ..base
            },
            HealthConfig {
                settling_time: value,
                ..base
            },
        ] {
            assert!(matches!(
                HealthMonitor::new(config),
                Err(RonError::ConfigInvalid(_))
            ));
        }
    }
    for value in bad_non_negative {
        for config in [
            HealthConfig {
                divergence_threshold: value,
                ..base
            },
            HealthConfig {
                dead_band: value,
                ..base
            },
            HealthConfig {
                steady_state_threshold: value,
                ..base
            },
        ] {
            assert!(HealthMonitor::new(config).is_err());
        }
    }
    let window = u8::try_from(HEALTH_OSC_WINDOW).unwrap();
    assert!(HealthMonitor::new(HealthConfig {
        oscillation_count_threshold: window,
        ..base
    })
    .is_err());

    let mut monitor = HealthMonitor::new(quiet_config()).unwrap();
    for dt in [0.0, -0.01, RonFloat::NAN] {
        assert!(matches!(
            monitor.step(0.0, 0.0, 0.0, dt),
            Err(RonError::InvalidArgument(_))
        ));
    }
    assert!(monitor.step(RonFloat::NAN, 0.0, 0.0, DT).is_err());
    assert!(monitor.step(0.0, RonFloat::INFINITY, 0.0, DT).is_err());
    assert!(monitor.step(0.0, 0.0, RonFloat::NAN, DT).is_err());
}

static STUCK_CALLS: AtomicU32 = AtomicU32::new(0);
static STUCK_LAST: AtomicU8 = AtomicU8::new(0);

fn stuck_callback(condition: HealthStatus) {
    STUCK_CALLS.fetch_add(1, Ordering::SeqCst);
    STUCK_LAST.store(condition.bits(), Ordering::SeqCst);
}

/// RON-TC-HLTH-002 | RON-FR-901
#[test]
fn ron_tc_hlth_002() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        stuck_time: 0.5,
        dead_band: 1.0e-3,
        callback: Some(stuck_callback),
        ..quiet_config()
    })
    .unwrap();
    for k in 1..=60 {
        let measurement = if k % 2 == 0 { 0.0 } else { 0.01 };
        let _ = monitor.step(measurement, measurement, 5.0, DT).unwrap();
        assert_eq!(
            monitor.status().contains(HealthStatus::OUTPUT_STUCK),
            k >= 50,
            "step {k}"
        );
    }
    assert_eq!(monitor.status(), HealthStatus::OUTPUT_STUCK);
    assert_eq!(STUCK_CALLS.load(Ordering::SeqCst), 1);
    assert_eq!(
        STUCK_LAST.load(Ordering::SeqCst),
        HealthStatus::OUTPUT_STUCK.bits()
    );
}

/// RON-TC-HLTH-003 | RON-FR-901
#[test]
fn ron_tc_hlth_003() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        divergence_threshold: 1.0,
        ..quiet_config()
    })
    .unwrap();
    for k in 1..=20 {
        let _ = monitor
            .step(0.0, -0.1 * step_index(k), 0.5 * step_index(k), DT)
            .unwrap();
    }
    assert!(monitor.status().contains(HealthStatus::DIVERGING));
    let _ = monitor.step(0.0, -2.0, 1.0, DT).unwrap();
    let _ = monitor.step(0.0, -1.5, 1.0, DT).unwrap();
    assert_eq!(monitor.status(), HealthStatus::DIVERGING);
}

/// RON-TC-HLTH-004 | RON-FR-901
#[test]
fn ron_tc_hlth_004() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        oscillation_count_threshold: 5,
        ..quiet_config()
    })
    .unwrap();
    for k in 1..=10 {
        let measurement = if k % 2 == 0 { 2.0 } else { -2.0 };
        let _ = monitor
            .step(0.0, measurement, 0.3 * step_index(k), DT)
            .unwrap();
    }
    assert!(monitor.status().contains(HealthStatus::OSCILLATING));
    let _ = monitor.step(0.0, -2.0, 3.0, DT).unwrap();
    let _ = monitor.step(0.0, -2.0, 3.1, DT).unwrap();
    assert_eq!(monitor.status(), HealthStatus::OSCILLATING);
}

/// RON-TC-HLTH-005 | RON-FR-901
#[test]
fn ron_tc_hlth_005() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        dead_band: 0.01,
        dropout_time: 0.5,
        ..quiet_config()
    })
    .unwrap();
    for k in 1..=60 {
        let _ = monitor.step(2.0, 2.0, 0.3 * step_index(k), DT).unwrap();
        if k == 40 {
            assert!(!monitor.status().contains(HealthStatus::SENSOR_DROPOUT));
        }
    }
    assert_eq!(monitor.status(), HealthStatus::SENSOR_DROPOUT);
}

/// RON-TC-HLTH-006 | RON-FR-901
#[test]
fn ron_tc_hlth_006() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        steady_state_threshold: 0.5,
        settling_time: 0.5,
        ..quiet_config()
    })
    .unwrap();
    for k in 1..=60 {
        let measurement = if k % 2 == 0 { 0.0 } else { 1.0e-3 };
        let _ = monitor
            .step(1.0, measurement, 0.3 * step_index(k), DT)
            .unwrap();
        if k == 40 {
            assert!(!monitor
                .status()
                .contains(HealthStatus::SETPOINT_UNREACHABLE));
        }
    }
    assert_eq!(monitor.status(), HealthStatus::SETPOINT_UNREACHABLE);
}

/// RON-TC-HLTH-007 | RON-FR-902
#[test]
fn ron_tc_hlth_007() {
    let mut low = HealthMonitor::new(HealthConfig {
        divergence_threshold: 1.5,
        ..quiet_config()
    })
    .unwrap();
    let mut high = HealthMonitor::new(quiet_config()).unwrap();
    for k in 1..=30 {
        let (measurement, output) = (-0.1 * step_index(k), 0.5 * step_index(k));
        let _ = low.step(0.0, measurement, output, DT).unwrap();
        let _ = high.step(0.0, measurement, output, DT).unwrap();
    }
    assert_eq!(low.status(), HealthStatus::DIVERGING);
    assert!(high.status().is_ok());
}

/// RON-TC-HLTH-008 | RON-FR-903
#[test]
fn ron_tc_hlth_008() {
    let pid_config = PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(1.0, 0.0, 0.0)
    };
    let armed = HealthConfig {
        stuck_time: 0.05,
        divergence_threshold: 0.1,
        oscillation_count_threshold: 1,
        dead_band: 1.0,
        dropout_time: 0.05,
        steady_state_threshold: 0.01,
        settling_time: 0.05,
        callback: None,
    };
    let run = |mut monitor: Option<HealthMonitor>| {
        let mut pid = Pid::new(pid_config).unwrap();
        let mut measurement = 0.0;
        let mut outputs = [0.0; 200];
        for slot in &mut outputs {
            let (output, _) = pid.step(1.0, measurement, DT).unwrap();
            if let Some(monitor) = monitor.as_mut() {
                let _ = monitor.step(1.0, measurement, output, DT).unwrap();
            }
            *slot = output;
            measurement += (DT / 0.1) * (output - measurement);
        }
        outputs
    };
    let with_monitor = run(Some(HealthMonitor::new(armed).unwrap()));
    let without_monitor = run(None);
    for (with, without) in with_monitor.iter().zip(without_monitor.iter()) {
        assert_eq!(with.to_bits(), without.to_bits());
    }
}

static ORDER_CALLS: AtomicU32 = AtomicU32::new(0);
static ORDER_LAST: AtomicU8 = AtomicU8::new(0);

fn order_callback(condition: HealthStatus) {
    ORDER_CALLS.fetch_add(1, Ordering::SeqCst);
    ORDER_LAST.store(condition.bits(), Ordering::SeqCst);
}

/// RON-TC-HLTH-009 | RON-FR-904
#[test]
fn ron_tc_hlth_009() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        stuck_time: 0.3,
        dead_band: 0.01,
        dropout_time: 0.6,
        callback: Some(order_callback),
        ..quiet_config()
    })
    .unwrap();
    let mut activations = HealthStatus::OK;
    for k in 1..=80 {
        activations |= monitor.step(2.0, 2.0, 5.0, DT).unwrap();
        if k == 35 {
            assert_eq!(ORDER_CALLS.load(Ordering::SeqCst), 1);
            assert_eq!(
                ORDER_LAST.load(Ordering::SeqCst),
                HealthStatus::OUTPUT_STUCK.bits()
            );
        }
    }
    assert_eq!(ORDER_CALLS.load(Ordering::SeqCst), 2);
    assert_eq!(
        ORDER_LAST.load(Ordering::SeqCst),
        HealthStatus::SENSOR_DROPOUT.bits()
    );
    assert_eq!(
        activations,
        HealthStatus::OUTPUT_STUCK | HealthStatus::SENSOR_DROPOUT
    );

    let mut silent = HealthMonitor::new(HealthConfig {
        stuck_time: 0.1,
        ..quiet_config()
    })
    .unwrap();
    for _ in 0..40 {
        let _ = silent.step(0.0, 0.0, 5.0, DT).unwrap();
    }
    assert!(silent.status().contains(HealthStatus::OUTPUT_STUCK));
}

/// RON-TC-HLTH-010 | RON-FR-905
#[test]
fn ron_tc_hlth_010() {
    let mut monitor = HealthMonitor::new(HealthConfig {
        divergence_threshold: 1.0,
        ..quiet_config()
    })
    .unwrap();
    let diverge = |monitor: &mut HealthMonitor| {
        for k in 1..=20 {
            let _ = monitor.step(0.0, -0.1 * step_index(k), 1.0, DT).unwrap();
        }
    };
    diverge(&mut monitor);
    assert!(monitor.status().contains(HealthStatus::DIVERGING));
    for _ in 0..10 {
        let _ = monitor.step(0.0, 0.0, 1.0, DT).unwrap();
        assert!(monitor.status().contains(HealthStatus::DIVERGING));
    }
    assert!(monitor.step(0.0, -3.0, 1.0, DT).unwrap().is_ok());
    assert!(monitor.step(0.0, -6.0, 1.0, DT).unwrap().is_ok());
    monitor.clear();
    assert!(monitor.status().is_ok());
    diverge(&mut monitor);
    assert!(monitor.status().contains(HealthStatus::DIVERGING));
}
