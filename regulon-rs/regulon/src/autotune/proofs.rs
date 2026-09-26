//! # `autotune::proofs`
//!
//! Kani proof that the relay output stays within `bias ± d`.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-806
//! **Tests:** RON-TC-AT-007-FV
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{AutotuneConfig, Autotuner, TuningRule};
use crate::pid::{Pid, PidConfig};

/// RON-TC-AT-007-FV | RON-FR-806
#[kani::proof]
fn ron_tc_at_007_fv() {
    let config = AutotuneConfig {
        relay_amplitude: kani::any(),
        hysteresis: kani::any(),
        bias: kani::any(),
        min_cycles: 1,
        timeout: 30.0,
        rule: TuningRule::ZieglerNichols,
    };
    kani::assume(config.relay_amplitude.is_finite() && config.relay_amplitude > 0.0);
    kani::assume(config.relay_amplitude < 1.0e6);
    kani::assume(config.hysteresis.is_finite() && config.hysteresis >= 0.0);
    kani::assume(config.bias.is_finite() && config.bias.abs() < 1.0e6);
    let Ok(mut tuner) = Autotuner::new(config) else {
        return;
    };
    let Ok(mut pid) = Pid::new(PidConfig::default()) else {
        return;
    };
    if tuner.start(&mut pid).is_err() {
        return;
    }
    let setpoint: f32 = kani::any();
    let measurement: f32 = kani::any();
    kani::assume(setpoint.is_finite() && measurement.is_finite());
    if let Ok(output) = tuner.step(setpoint, measurement, 0.001) {
        assert!(output >= config.bias - config.relay_amplitude);
        assert!(output <= config.bias + config.relay_amplitude);
    }
}
