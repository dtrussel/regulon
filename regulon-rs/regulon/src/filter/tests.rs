//! # `filter::tests`
//!
//! Traceable tests for the initial reusable filter slice.
//!
//! **Document:** RON-TP-001
//! **Requirements:** RON-FR-100-RON-FR-103, RON-FR-110-RON-FR-111,
//! RON-FR-115-RON-FR-117, RON-FR-120-RON-FR-123, RON-FR-130-RON-FR-131
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{
    Biquad, BiquadSection, FilterFault, FilterStatus, Lp1, Lp1Config, MovingAverage, RateLimiter,
    RateLimiterConfig, BIQUAD_MAX_SECTIONS, MA_MAX_WINDOW,
};
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
    approx_eq(checkpoint, 1.0 - RonFloat::powi(0.9, 10), 0.001);
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

/// RON-TC-FILT-004 | RON-FR-103
#[test]
fn ron_tc_filt_004_rate_limiter_config_validation() {
    let invalid = RateLimiter::new(RateLimiterConfig {
        rise_limit: 0.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    });
    assert_eq!(invalid, Err(FilterFault::CONFIG_INVALID));
}

/// RON-TC-FILT-003 | RON-FR-101-RON-FR-102
#[test]
fn ron_tc_filt_003_rate_limiter_input_fault() {
    let mut limiter = RateLimiter::new(RateLimiterConfig {
        rise_limit: 1.0,
        fall_limit: 1.0,
        initial_output: 0.0,
    })
    .unwrap();
    assert!(limiter.step(1.0, RonFloat::NAN).is_err());
    assert_eq!(limiter.state().fault, FilterFault::INPUT_NOT_FINITE);
}

/// RON-TC-FILT-003 | RON-FR-102
#[test]
fn ron_tc_filt_003_rate_limiter_reset() {
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

const TIGHT: RonFloat = 1.0e-5;
const COEFF_TOL: RonFloat = 1.0e-6;
const DT: RonFloat = 0.001;
const BUTTERWORTH_Q: RonFloat = 0.707_106_78;

fn pass_through(gain: RonFloat) -> BiquadSection {
    BiquadSection {
        b0: gain,
        ..BiquadSection::default()
    }
}

/// RON-TC-FILT-008 | RON-FR-115
#[test]
fn ron_tc_filt_008() {
    let mut filter = MovingAverage::<4>::new().unwrap();
    for (input, expected) in [(1.0, 0.25), (2.0, 0.75), (3.0, 1.5), (4.0, 2.5)] {
        let (output, status) = filter.step(input).unwrap();
        approx_eq(output, expected, TIGHT);
        assert_eq!(status, FilterStatus::OK);
    }
    approx_eq(filter.step(5.0).unwrap().0, 3.5, TIGHT);
}

/// RON-TC-FILT-009 | RON-FR-116
#[test]
fn ron_tc_filt_009() {
    let filter = MovingAverage::<MA_MAX_WINDOW>::new().unwrap();
    assert_eq!(filter.count(), 0);
    assert_eq!(MovingAverage::<0>::new(), Err(FilterFault::CONFIG_INVALID));
    assert_eq!(
        MovingAverage::<{ MA_MAX_WINDOW + 1 }>::new(),
        Err(FilterFault::CONFIG_INVALID)
    );
}

/// RON-TC-FILT-010 | RON-FR-117
#[test]
fn ron_tc_filt_010() {
    let mut filter = MovingAverage::<3>::new().unwrap();
    for input in [3.0, 6.0] {
        let _ = filter.step(input).unwrap();
    }
    approx_eq(filter.step(9.0).unwrap().0, 6.0, TIGHT);
    approx_eq(filter.step(12.0).unwrap().0, 9.0, TIGHT);
    approx_eq(filter.sum(), 27.0, TIGHT);
    assert_eq!(filter.count(), 3);
}

/// RON-TC-FILT-003 | RON-FR-101-RON-FR-102
#[test]
fn ron_tc_filt_003_moving_average_and_biquad_faults() {
    let mut average = MovingAverage::<2>::new().unwrap();
    let _ = average.step(1.0).unwrap();
    let before = average;
    assert_eq!(
        average.step(RonFloat::NAN),
        Err(FilterFault::INPUT_NOT_FINITE)
    );
    assert_eq!(average.step(1.0), Err(FilterFault::INPUT_NOT_FINITE));
    assert_eq!(average.sum(), before.sum());
    average.reset();
    assert_eq!(average.state().fault, FilterFault::NONE);
    approx_eq(average.step(4.0).unwrap().0, 2.0, TIGHT);

    let mut overflow = MovingAverage::<2>::new().unwrap();
    let _ = overflow.step(RonFloat::MAX).unwrap();
    assert_eq!(
        overflow.step(RonFloat::MAX),
        Err(FilterFault::OUTPUT_NOT_FINITE)
    );
    approx_eq(overflow.sum(), RonFloat::MAX, RonFloat::MAX * 1.0e-6);

    let mut biquad = Biquad::new([pass_through(1.0)]).unwrap();
    assert_eq!(
        biquad.step(RonFloat::INFINITY),
        Err(FilterFault::INPUT_NOT_FINITE)
    );
    assert!(biquad.update_notch(0, 60.0, 4.0, DT).is_err());
    biquad.reset();
    approx_eq(biquad.step(2.0).unwrap().0, 2.0, TIGHT);
}

/// RON-TC-FILT-011 | RON-FR-120
#[test]
fn ron_tc_filt_011() {
    let mut filter = Biquad::new([BiquadSection {
        b0: 0.5,
        b1: 0.25,
        b2: 0.125,
        a1: 0.0,
        a2: 0.0,
    }])
    .unwrap();
    for (input, expected) in [(1.0, 0.5), (0.0, 0.25), (0.0, 0.125)] {
        approx_eq(filter.step(input).unwrap().0, expected, TIGHT);
    }
}

/// RON-TC-FILT-012 | RON-FR-121
#[test]
fn ron_tc_filt_012() {
    let mut filter = Biquad::new([pass_through(0.5), pass_through(0.5)]).unwrap();
    approx_eq(filter.step(4.0).unwrap().0, 1.0, TIGHT);
    assert!(Biquad::new([pass_through(1.0); BIQUAD_MAX_SECTIONS]).is_ok());
    assert!(Biquad::<0>::new([]).is_err());
    assert!(Biquad::new([pass_through(1.0); BIQUAD_MAX_SECTIONS + 1]).is_err());
    for bad in [
        pass_through(0.0),
        pass_through(RonFloat::NAN),
        BiquadSection {
            a2: 1.0,
            ..pass_through(1.0)
        },
    ] {
        assert_eq!(Biquad::new([bad]), Err(FilterFault::CONFIG_INVALID));
    }
}

/// RON-TC-FILT-013 | RON-FR-122
#[test]
fn ron_tc_filt_013() {
    for section in [
        BiquadSection::low_pass(100.0, BUTTERWORTH_Q, DT),
        BiquadSection::high_pass(100.0, BUTTERWORTH_Q, DT),
        BiquadSection::band_pass(100.0, BUTTERWORTH_Q, DT),
        BiquadSection::notch(60.0, BUTTERWORTH_Q, DT),
    ] {
        let section = section.unwrap();
        assert!(section.b0 > 0.0);
        assert!(section.validate().is_ok());
    }
    // Reference values computed with libm; the frequencies straddle the
    // series' range reduction up to just below Nyquist.
    let expected = [
        (
            100.0,
            0.067_455_274,
            0.134_910_548,
            -1.142_980_502,
            0.412_801_597,
        ),
        (250.0, 0.292_893_219, 0.585_786_437, 0.0, 0.171_572_874),
        (
            400.0,
            0.638_945_525,
            1.277_891_050,
            1.142_980_502,
            0.412_801_597,
        ),
        (
            499.0,
            0.995_566_972,
            1.991_133_944,
            1.991_114_292,
            0.991_153_596,
        ),
    ];
    for (frequency, b0, b1, a1, a2) in expected {
        let section = BiquadSection::low_pass(frequency, BUTTERWORTH_Q, DT).unwrap();
        approx_eq(section.b0, b0, COEFF_TOL);
        approx_eq(section.b1, b1, COEFF_TOL);
        approx_eq(section.a1, a1, COEFF_TOL);
        approx_eq(section.a2, a2, COEFF_TOL);
    }
}

/// RON-TC-FILT-014 | RON-FR-122
#[test]
fn ron_tc_filt_014() {
    let invalid = [
        BiquadSection::low_pass(0.0, BUTTERWORTH_Q, DT),
        BiquadSection::high_pass(100.0, 0.0, DT),
        BiquadSection::band_pass(100.0, BUTTERWORTH_Q, 0.0),
        BiquadSection::notch(500.0, BUTTERWORTH_Q, DT),
        BiquadSection::low_pass(RonFloat::NAN, BUTTERWORTH_Q, DT),
    ];
    for result in invalid {
        assert_eq!(result, Err(FilterFault::CONFIG_INVALID));
    }
}

/// RON-TC-FILT-015 | RON-FR-123
#[test]
fn ron_tc_filt_015() {
    let mut filter = Biquad::new([BiquadSection::notch(60.0, 4.0, DT).unwrap()]).unwrap();
    let amplitude = filter.step(1.0).unwrap().0.abs();
    let before = filter.section_state();
    filter.update_notch(0, 120.0, 4.0, DT).unwrap();
    assert_eq!(filter.section_state(), before);
    assert_eq!(
        filter.sections()[0],
        BiquadSection::notch(120.0, 4.0, DT).unwrap()
    );
    assert_eq!(
        filter.update_notch(1, 120.0, 4.0, DT),
        Err(FilterFault::CONFIG_INVALID)
    );
    let retuned = filter.sections()[0];
    assert!(filter.update_notch(0, 600.0, 4.0, DT).is_err());
    assert_eq!(filter.sections()[0], retuned);
    let output = filter.step(1.0).unwrap().0;
    assert!(output.abs() <= 3.0 * (amplitude + 1.0));
}
