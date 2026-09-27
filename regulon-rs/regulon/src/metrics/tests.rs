//! # `metrics::tests`
//!
//! Traceable tests for the performance-metrics accumulator.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-950-RON-FR-954
//! **Tests:** RON-TC-MET-001-RON-TC-MET-007
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Metrics, MetricsConfig, MetricsMode, MetricsResults};
use crate::{
    pid::{AntiWindupMode, Pid, PidConfig},
    RonError, RonFloat,
};

const DT: RonFloat = 0.01;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

fn config(step_threshold: RonFloat) -> MetricsConfig {
    MetricsConfig {
        mode: MetricsMode::Cumulative,
        band_fraction: 0.02,
        settle_confirm: 0.05,
        step_threshold,
    }
}

fn enabled(config: MetricsConfig) -> Metrics {
    let mut metrics = Metrics::new(config).unwrap();
    metrics.set_enabled(true);
    metrics
}

/// RON-TC-MET-001 | RON-FR-950, RON-FR-953
#[test]
fn ron_tc_met_001() {
    let base = config(0.5);
    let metrics = Metrics::new(base).unwrap();
    assert!(!metrics.is_enabled());

    assert!(Metrics::new(MetricsConfig {
        mode: MetricsMode::Windowed { window_steps: 0 },
        ..base
    })
    .is_err());
    assert!(Metrics::new(MetricsConfig {
        mode: MetricsMode::Windowed { window_steps: 8 },
        ..base
    })
    .is_ok());
    for band in [0.0, -0.1, RonFloat::NAN, RonFloat::INFINITY] {
        assert!(matches!(
            Metrics::new(MetricsConfig {
                band_fraction: band,
                ..base
            }),
            Err(RonError::ConfigInvalid(_))
        ));
    }
    for confirm in [-1.0, RonFloat::NEG_INFINITY] {
        assert!(Metrics::new(MetricsConfig {
            settle_confirm: confirm,
            ..base
        })
        .is_err());
    }
    for threshold in [0.0, RonFloat::NAN] {
        assert!(Metrics::new(config(threshold)).is_err());
    }

    let mut metrics = enabled(base);
    for dt in [0.0, -0.01, RonFloat::NAN] {
        assert!(matches!(
            metrics.step(0.0, 0.0, dt),
            Err(RonError::InvalidArgument(_))
        ));
    }
    assert!(metrics.step(RonFloat::NAN, 0.0, DT).is_err());
    assert!(metrics.step(0.0, RonFloat::INFINITY, DT).is_err());
    metrics.step(1.0, 0.0, DT).unwrap();
    metrics.reset();
    assert!(metrics.is_enabled());
    assert_eq!(metrics.results(), MetricsResults::default());
}

/// RON-TC-MET-002 | RON-FR-951
#[test]
fn ron_tc_met_002() {
    let mut metrics = enabled(config(10.0));
    for _ in 0..100 {
        metrics.step(0.5, 0.0, DT).unwrap();
    }
    let results = metrics.results();
    approx_eq(results.iae, 0.5, 5.0e-5);
    approx_eq(results.ise, 0.25, 2.5e-5);
    approx_eq(results.itae, 0.2525, 2.5e-5);
    assert_eq!(results.rise_time, None);
    assert_eq!(results.settling_time, None);
    approx_eq(results.peak_overshoot, 0.0, 1.0e-6);
}

/// RON-TC-MET-003 | RON-FR-951
#[test]
fn ron_tc_met_003() {
    let mut metrics = enabled(config(0.5));
    for measurement in [0.0, 0.4, 0.8, 1.0, 1.1, 1.2, 1.15, 1.1, 1.05, 1.0, 1.0, 1.0] {
        metrics.step(1.0, measurement, DT).unwrap();
    }
    approx_eq(metrics.results().peak_overshoot, 20.0, 0.5);
}

/// RON-TC-MET-004 | RON-FR-951
#[test]
fn ron_tc_met_004() {
    let mut metrics = enabled(config(0.5));
    metrics.step(1.0, 0.0, DT).unwrap();
    for k in 1_u8..=20 {
        metrics.step(1.0, 0.05 * RonFloat::from(k), DT).unwrap();
    }
    for _ in 22..=40 {
        metrics.step(1.0, 1.0, DT).unwrap();
    }
    let results = metrics.results();
    approx_eq(results.rise_time.unwrap(), 0.16, 0.02);
    approx_eq(results.settling_time.unwrap(), 0.25, 0.03);
}

/// RON-TC-MET-005 | RON-FR-952
#[test]
fn ron_tc_met_005() {
    let mut cumulative = enabled(config(10.0));
    let mut windowed = enabled(MetricsConfig {
        mode: MetricsMode::Windowed { window_steps: 10 },
        ..config(10.0)
    });
    for _ in 0..25 {
        cumulative.step(0.5, 0.0, DT).unwrap();
        windowed.step(0.5, 0.0, DT).unwrap();
    }
    approx_eq(cumulative.results().iae, 0.125, 1.0e-4);
    approx_eq(windowed.results().iae, 0.025, 1.0e-4);
}

/// RON-TC-MET-006 | RON-FR-953
#[test]
fn ron_tc_met_006() {
    let mut disabled = Metrics::new(config(0.5)).unwrap();
    let before = disabled;
    for _ in 0..1_000 {
        disabled.step(1.0, 0.0, DT).unwrap();
    }
    disabled.step(RonFloat::NAN, 0.0, -1.0).unwrap();
    assert_eq!(disabled, before);

    let pid_config = PidConfig {
        output_min: -1_000.0,
        output_max: 1_000.0,
        integral_min: -1_000.0,
        integral_max: 1_000.0,
        anti_windup_mode: AntiWindupMode::Disabled,
        ..PidConfig::new_parallel(1.0, 0.0, 0.0)
    };
    let run = |mut metrics: Metrics| {
        let mut pid = Pid::new(pid_config).unwrap();
        let mut measurement = 0.0;
        let mut outputs = [0.0; 200];
        for slot in &mut outputs {
            let (output, _) = pid.step(1.0, measurement, DT).unwrap();
            metrics.step(1.0, measurement, DT).unwrap();
            *slot = output;
            measurement += (DT / 0.1) * (output - measurement);
        }
        outputs
    };
    let with_metrics = run(enabled(config(0.5)));
    let without_metrics = run(Metrics::new(config(0.5)).unwrap());
    for (with, without) in with_metrics.iter().zip(without_metrics.iter()) {
        assert_eq!(with.to_bits(), without.to_bits());
    }
}

/// RON-TC-MET-007 | RON-FR-954
#[test]
fn ron_tc_met_007() {
    let mut metrics = enabled(config(0.5));
    for _ in 0..5 {
        metrics.step(0.0, 0.0, DT).unwrap();
    }
    let results = metrics.results();
    assert_eq!(results.rise_time, None);
    assert_eq!(results.settling_time, None);
    approx_eq(results.peak_overshoot, 0.0, 1.0e-6);

    for measurement in [0.0, 0.5, 1.0, 1.2, 1.1, 1.05, 1.0, 1.0] {
        metrics.step(1.0, measurement, DT).unwrap();
    }
    let frame = metrics.step_frame();
    approx_eq(frame.target, 1.0, 1.0e-6);
    approx_eq(frame.reference, 0.0, 1.0e-6);
    approx_eq(frame.size, 1.0, 1.0e-6);
    approx_eq(metrics.results().peak_overshoot, 20.0, 0.5);
}
