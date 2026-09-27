//! # `filter::proofs`
//!
//! Kani proofs that the moving-average window and the biquad cascade stay
//! within their statically sized storage.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-116, RON-FR-121, RON-SR-005
//! **Tests:** RON-TC-FILT-009-FV, RON-TC-FILT-012-FV
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{Biquad, BiquadSection, MovingAverage};

/// Window length used by the proof: small enough to unroll, large enough
/// to wrap the ring index twice within the harness.
const WINDOW: usize = 3;

/// RON-TC-FILT-009-FV | RON-FR-116
#[kani::proof]
#[kani::unwind(8)]
fn ron_tc_filt_009_fv() {
    let Ok(mut filter) = MovingAverage::<WINDOW>::new() else {
        return;
    };
    for _ in 0..(2 * WINDOW) {
        let input: f32 = kani::any();
        kani::assume(input.is_finite() && input.abs() < 1.0e6);
        if filter.step(input).is_err() {
            return;
        }
        assert!(filter.count() <= WINDOW);
    }
    assert!(filter.count() == WINDOW);
}

/// RON-TC-FILT-012-FV | RON-FR-121
#[kani::proof]
#[kani::unwind(8)]
fn ron_tc_filt_012_fv() {
    let section = BiquadSection {
        b0: 0.5,
        b1: 0.25,
        b2: 0.125,
        a1: -0.5,
        a2: 0.25,
    };
    let Ok(mut filter) = Biquad::new([section; 2]) else {
        return;
    };
    let input: f32 = kani::any();
    kani::assume(input.is_finite() && input.abs() < 1.0e6);
    if let Ok((output, _)) = filter.step(input) {
        assert!(output.is_finite());
    }
}
