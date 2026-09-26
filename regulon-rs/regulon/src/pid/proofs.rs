//! # `pid::proofs`
//!
//! Kani proofs for the PID safety baseline.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-020, RON-PR-001, RON-SR-010, RON-SR-012
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{AntiWindupMode, Pid, PidConfig, PidFault};
use crate::RonError;

/// Bound on `|r|` and `|y|` from RON-TC-PID-015-FV.
const SIGNAL_BOUND: f32 = 1_000.0;
/// Largest sample period from RON-TC-PID-015-FV.
const DT_MAX: f32 = 1.0;

/// RON-TC-PID-015-FV | RON-FR-020
#[kani::proof]
fn ron_tc_pid_015_fv() {
    let config = PidConfig {
        kp: 10.0,
        ki: 1.0,
        kd: 0.0,
        output_min: -5.0,
        output_max: 5.0,
        anti_windup_mode: AntiWindupMode::BackCalculation,
        anti_windup_tracking_time: 0.1,
        ..PidConfig::default()
    };
    let mut pid = Pid::new(config).unwrap();
    let setpoint: f32 = kani::any();
    let measurement: f32 = kani::any();
    let dt: f32 = kani::any();
    // Harness bounds from RON-TC-PID-015-FV (RON-ASM-02 / RON-ASM-03), as in
    // the C CBMC harness pid_saturation_proof.c.
    kani::assume(setpoint.is_finite() && setpoint.abs() <= SIGNAL_BOUND);
    kani::assume(measurement.is_finite() && measurement.abs() <= SIGNAL_BOUND);
    kani::assume(dt.is_finite() && dt > 0.0 && dt <= DT_MAX);
    if let Ok((output, _status)) = pid.step(setpoint, measurement, dt) {
        assert!((-5.0..=5.0).contains(&output));
    }
}

/// RON-TC-SAFE-011-FV | RON-SR-020
#[kani::proof]
fn ron_tc_safe_011_fv() {
    let mut pid = Pid::new(PidConfig::default()).unwrap();
    let result = pid.step(f32::NAN, 0.0, 0.01);
    assert_eq!(result, Err(RonError::Fault(PidFault::INPUT_NOT_FINITE)));
}
