//! # `filter::tests`
//!
//! Traceable tests for the initial reusable filter slice.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-100-RON-FR-103, RON-FR-110-RON-FR-111,
//! RON-FR-130-RON-FR-131
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{FilterFault, FilterStatus, Lp1, Lp1Config, RateLimiter, RateLimiterConfig};
use crate::RonFloat;

fn approx_eq(lhs: RonFloat, rhs: RonFloat, tolerance: RonFloat) {
    assert!((lhs - rhs).abs() <= tolerance, "{lhs} != {rhs}");
}

/// RON-TC-FILT-005 | RON-FR-110
#[test]
fn ron_tc_filt_005() {
    let mut filter = Lp1::new(Lp1Config::Alpha {
        alpha: 0.1,
        initial_output: 0.0,
    })
    .unwrap();
    for _ in 0..10 {
        let _ = filter.step(0.0).unwrap();
    }
    let mut checkpoint = 0.0;
    for step in 0..200 {
        let (output, status) = filter.step(1.0).unwrap();
        assert_eq!(status, FilterStatus::OK);
        if step == 9 {
            checkpoint = output;
        }
    }
    approx_eq(checkpoint, 1.0 - 0.9_f32.powi(10), 0.001);
    assert!((filter.state().last_output - 1.0).abs() < 0.01);
}

/// RON-TC-FILT-006 | RON-FR-111
#[test]
fn ron_tc_filt_006() {
    let mut alpha_filter = Lp1::new(Lp1Config::Cutoff {
        cutoff_hz: 10.0,
        sample_period: 0.001,
        initial_output: 0.0,
    })
    .unwrap();
    let alpha = Lp1Config::Cutoff {
        cutoff_hz: 10.0,
        sample_period: 0.001,
        initial_output: 0.0,
    }
    .alpha()
    .unwrap();
    let mut direct_filter = Lp1::new(Lp1Config::Alpha {
        alpha,
        initial_output: 0.0,
    })
    .unwrap();
    for _ in 0..500 {
        let (alpha_output, _) = alpha_filter.step(1.0).unwrap();
        let (direct_output, _) = direct_filter.step(1.0).unwrap();
        approx_eq(alpha_output, direct_output, 0.001);
    }
}

/// RON-TC-FILT-016 | RON-FR-130
#[test]
fn ron_tc_filt_016() {
    let mut limiter = RateLimiter::new(RateLimiterConfig {
        rise_limit: 2.0,
        fall_limit: 2.0,
        initial_output: 0.0,
    })
    .unwrap();
    let (output, status) = limiter.step(10.0, 0.5).unwrap();
    approx_eq(output, 1.0, 16.0 * RonFloat::EPSILON);
    assert!(status.contains(FilterStatus::RATE_LIMITED));
}

/// RON-TC-FILT-017 | RON-FR-131
#[test]
fn ron_tc_filt_017() {
    let mut limiter = RateLimiter::new(RateLimiterConfig {
        rise_limit: 4.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    })
    .unwrap();
    let (rising, _) = limiter.step(10.0, 0.5).unwrap();
    approx_eq(rising, 2.0, 16.0 * RonFloat::EPSILON);
    let (falling, status) = limiter.step(-10.0, 0.5).unwrap();
    approx_eq(falling, 1.5, 16.0 * RonFloat::EPSILON);
    assert!(status.contains(FilterStatus::RATE_LIMITED));
}

/// RON-TC-FILT-001 | RON-FR-103
#[test]
fn ron_tc_filt_001() {
    let invalid = Lp1::new(Lp1Config::Alpha {
        alpha: 0.0,
        initial_output: 0.0,
    });
    assert_eq!(invalid, Err(FilterFault::CONFIG_INVALID));
}

/// RON-TC-FILT-002 | RON-FR-103
#[test]
fn ron_tc_filt_002() {
    let invalid = Lp1::new(Lp1Config::Cutoff {
        cutoff_hz: 0.0,
        sample_period: 0.001,
        initial_output: 0.0,
    });
    assert_eq!(invalid, Err(FilterFault::CONFIG_INVALID));
}

/// RON-TC-FILT-003 | RON-FR-101-RON-FR-102
#[test]
fn ron_tc_filt_003() {
    let mut filter = Lp1::new(Lp1Config::Alpha {
        alpha: 0.25,
        initial_output: 0.0,
    })
    .unwrap();
    assert!(filter.step(RonFloat::NAN).is_err());
    assert_eq!(filter.state().fault, FilterFault::INPUT_NOT_FINITE);
    assert_eq!(
        filter.state().fault.bits(),
        FilterFault::INPUT_NOT_FINITE.bits()
    );
    assert!(!filter.state().fault.is_none());
}

/// RON-TC-FILT-004 | RON-FR-102
#[test]
fn ron_tc_filt_004() {
    let mut filter = Lp1::new(Lp1Config::Alpha {
        alpha: 0.25,
        initial_output: 1.0,
    })
    .unwrap();
    let _ = filter.step(0.0).unwrap();
    filter.reset(2.0);
    assert_eq!(filter.state().last_output, 2.0);
    assert_eq!(filter.state().status, FilterStatus::OK);
}

/// RON-TC-FILT-008 | RON-FR-103
#[test]
fn ron_tc_filt_008() {
    let invalid = RateLimiter::new(RateLimiterConfig {
        rise_limit: 0.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    });
    assert_eq!(invalid, Err(FilterFault::CONFIG_INVALID));
}

/// RON-TC-FILT-009 | RON-FR-101-RON-FR-102
#[test]
fn ron_tc_filt_009() {
    let mut limiter = RateLimiter::new(RateLimiterConfig {
        rise_limit: 1.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    })
    .unwrap();
    assert!(limiter.step(1.0, RonFloat::NAN).is_err());
    assert_eq!(limiter.state().fault, FilterFault::INPUT_NOT_FINITE);
}

/// RON-TC-FILT-010 | RON-FR-102
#[test]
fn ron_tc_filt_010() {
    let mut limiter = RateLimiter::new(RateLimiterConfig {
        rise_limit: 1.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    })
    .unwrap();
    let _ = limiter.step(5.0, 1.0).unwrap();
    limiter.reset(-2.0);
    assert_eq!(limiter.state().last_output, -2.0);
    assert_eq!(limiter.state().status.bits(), FilterStatus::OK.bits());
}
