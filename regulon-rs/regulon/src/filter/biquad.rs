//! # `filter::biquad`
//!
//! Cascaded second-order IIR sections in transposed direct form II, with
//! low-pass, high-pass, band-pass and notch design helpers and runtime notch
//! retuning.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-100-RON-FR-103, RON-FR-120-RON-FR-123
//! **Tests:** RON-TC-FILT-011-RON-TC-FILT-015
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{FilterFault, FilterSnapshot, FilterStatus, TWO_PI};
use crate::platform::{abs, is_finite, RonFloat};

/// Largest number of cascaded sections.
///
/// **Satisfies:** RON-FR-121
pub const BIQUAD_MAX_SECTIONS: usize = 8;

/// Normalised biquad coefficients (`a0 = 1`).
///
/// **Satisfies:** RON-FR-120
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BiquadSection {
    /// Feed-forward coefficient `b0`.
    pub b0: RonFloat,
    /// Feed-forward coefficient `b1`.
    pub b1: RonFloat,
    /// Feed-forward coefficient `b2`.
    pub b2: RonFloat,
    /// Feedback coefficient `a1`.
    pub a1: RonFloat,
    /// Feedback coefficient `a2`.
    pub a2: RonFloat,
}

#[derive(Clone, Copy)]
enum Response {
    LowPass,
    HighPass,
    BandPass,
    Notch,
}

impl BiquadSection {
    /// Checks that the coefficients are finite, pass some signal, and place
    /// both poles strictly inside the unit circle (Jury stability test).
    ///
    /// **Satisfies:** RON-FR-103, RON-FR-120
    ///
    /// # Errors
    ///
    /// Returns [`FilterFault::CONFIG_INVALID`] otherwise.
    pub fn validate(&self) -> Result<(), FilterFault> {
        let coefficients = [self.b0, self.b1, self.b2, self.a1, self.a2];
        let finite = coefficients.iter().all(|value| is_finite(*value));
        let feedthrough = [self.b0, self.b1, self.b2]
            .iter()
            .any(|value| abs(*value) > RonFloat::EPSILON);
        let stable = (1.0 + self.a1 + self.a2) > RonFloat::EPSILON
            && (1.0 - self.a1 + self.a2) > RonFloat::EPSILON
            && (1.0 - self.a2) > RonFloat::EPSILON;
        if finite && feedthrough && stable {
            Ok(())
        } else {
            Err(FilterFault::CONFIG_INVALID)
        }
    }

    /// Second-order low-pass section (Butterworth for `q = 1/sqrt(2)`).
    ///
    /// **Satisfies:** RON-FR-122
    ///
    /// # Errors
    ///
    /// Returns [`FilterFault::CONFIG_INVALID`] when a parameter is not positive
    /// and finite or `cutoff_hz` is at or above Nyquist.
    pub fn low_pass(cutoff_hz: RonFloat, q: RonFloat, dt: RonFloat) -> Result<Self, FilterFault> {
        design(Response::LowPass, cutoff_hz, q, dt)
    }

    /// Second-order high-pass section.
    ///
    /// **Satisfies:** RON-FR-122
    ///
    /// # Errors
    ///
    /// As [`BiquadSection::low_pass`].
    pub fn high_pass(cutoff_hz: RonFloat, q: RonFloat, dt: RonFloat) -> Result<Self, FilterFault> {
        design(Response::HighPass, cutoff_hz, q, dt)
    }

    /// Second-order band-pass section.
    ///
    /// **Satisfies:** RON-FR-122
    ///
    /// # Errors
    ///
    /// As [`BiquadSection::low_pass`].
    pub fn band_pass(center_hz: RonFloat, q: RonFloat, dt: RonFloat) -> Result<Self, FilterFault> {
        design(Response::BandPass, center_hz, q, dt)
    }

    /// Second-order notch (band-reject) section, `q = f0 / bandwidth`.
    ///
    /// **Satisfies:** RON-FR-122, RON-FR-123
    ///
    /// # Errors
    ///
    /// As [`BiquadSection::low_pass`].
    pub fn notch(center_hz: RonFloat, q: RonFloat, dt: RonFloat) -> Result<Self, FilterFault> {
        design(Response::Notch, center_hz, q, dt)
    }
}

/// Cascade of `S` biquad sections applied in series.
///
/// **Satisfies:** RON-FR-120, RON-FR-121
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Biquad<const S: usize> {
    sections: [BiquadSection; S],
    w1: [RonFloat; S],
    w2: [RonFloat; S],
    state: FilterSnapshot,
}

impl<const S: usize> Biquad<S> {
    /// Creates the cascade with zeroed state.
    ///
    /// **Satisfies:** RON-FR-103, RON-FR-120, RON-FR-121
    ///
    /// # Errors
    ///
    /// Returns [`FilterFault::CONFIG_INVALID`] when `S` is 0 or above
    /// [`BIQUAD_MAX_SECTIONS`] or a section fails
    /// [`BiquadSection::validate`].
    pub fn new(sections: [BiquadSection; S]) -> Result<Self, FilterFault> {
        if S == 0 || S > BIQUAD_MAX_SECTIONS {
            return Err(FilterFault::CONFIG_INVALID);
        }
        for section in &sections {
            section.validate()?;
        }
        Ok(Self {
            sections,
            w1: [0.0; S],
            w2: [0.0; S],
            state: fresh_snapshot(),
        })
    }

    /// Clears the section state and any latched fault.
    ///
    /// **Satisfies:** RON-FR-102
    pub fn reset(&mut self) {
        self.w1 = [0.0; S];
        self.w2 = [0.0; S];
        self.state = fresh_snapshot();
    }

    /// Applies one input sample through every section in turn.
    ///
    /// **Satisfies:** RON-FR-101, RON-FR-120, RON-FR-121
    ///
    /// # Errors
    ///
    /// Returns the latched fault, or latches and returns
    /// [`FilterFault::INPUT_NOT_FINITE`] / [`FilterFault::OUTPUT_NOT_FINITE`];
    /// the section state is unchanged on error.
    pub fn step(&mut self, input: RonFloat) -> Result<(RonFloat, FilterStatus), FilterFault> {
        if !self.state.fault.is_none() {
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        if !is_finite(input) {
            return Err(self.latch(FilterFault::INPUT_NOT_FINITE));
        }
        let mut w1 = self.w1;
        let mut w2 = self.w2;
        let mut signal = input;
        for ((section, s1), s2) in self.sections.iter().zip(w1.iter_mut()).zip(w2.iter_mut()) {
            let w0 = (signal - (section.a1 * *s1)) - (section.a2 * *s2);
            signal = ((section.b0 * w0) + (section.b1 * *s1)) + (section.b2 * *s2);
            *s2 = *s1;
            *s1 = w0;
        }
        if !is_finite(signal) {
            return Err(self.latch(FilterFault::OUTPUT_NOT_FINITE));
        }
        self.w1 = w1;
        self.w2 = w2;
        self.state.last_output = signal;
        self.state.status = FilterStatus::OK;
        Ok((signal, self.state.status))
    }

    /// Retunes one section as a notch without clearing its state, so the
    /// output stays continuous (with a short settling transient). A rejected
    /// design leaves the running filter untouched.
    ///
    /// **Satisfies:** RON-FR-123
    ///
    /// # Errors
    ///
    /// Returns the latched fault, or [`FilterFault::CONFIG_INVALID`] for an
    /// out-of-range section or an invalid design.
    pub fn update_notch(
        &mut self,
        section: usize,
        center_hz: RonFloat,
        q: RonFloat,
        dt: RonFloat,
    ) -> Result<(), FilterFault> {
        if !self.state.fault.is_none() {
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        let candidate = BiquadSection::notch(center_hz, q, dt)?;
        let slot = self
            .sections
            .get_mut(section)
            .ok_or(FilterFault::CONFIG_INVALID)?;
        *slot = candidate;
        Ok(())
    }

    /// Returns the section coefficients.
    #[must_use]
    pub const fn sections(&self) -> &[BiquadSection; S] {
        &self.sections
    }

    /// Returns the per-section state `(w1, w2)`.
    #[must_use]
    pub const fn section_state(&self) -> ([RonFloat; S], [RonFloat; S]) {
        (self.w1, self.w2)
    }

    /// Returns a state snapshot.
    #[must_use]
    pub const fn state(&self) -> FilterSnapshot {
        self.state
    }

    fn latch(&mut self, fault: FilterFault) -> FilterFault {
        self.state.fault = fault;
        self.state.status = FilterStatus::FAULT;
        fault
    }
}

const fn fresh_snapshot() -> FilterSnapshot {
    FilterSnapshot {
        last_output: 0.0,
        status: FilterStatus::OK,
        fault: FilterFault::NONE,
    }
}

/// Audio-EQ-cookbook design shared by the four responses.
///
/// **Satisfies:** RON-FR-122
fn design(
    response: Response,
    frequency: RonFloat,
    q: RonFloat,
    dt: RonFloat,
) -> Result<BiquadSection, FilterFault> {
    let positive = |value: RonFloat| is_finite(value) && value > 0.0;
    if !positive(frequency) || !positive(q) || !positive(dt) || frequency >= 1.0 / (2.0 * dt) {
        return Err(FilterFault::CONFIG_INVALID);
    }
    let (sin, cos) = sin_cos(f64::from(TWO_PI * frequency * dt));
    let (sn, c) = (narrow(sin), narrow(cos));
    let alpha = sn / (2.0 * q);
    let norm = 1.0 / (1.0 + alpha);
    let (b0, b1, b2) = match response {
        Response::LowPass => {
            let b0 = (1.0 - c) * 0.5 * norm;
            (b0, (1.0 - c) * norm, b0)
        }
        Response::HighPass => {
            let b0 = (1.0 + c) * 0.5 * norm;
            (b0, -(1.0 + c) * norm, b0)
        }
        Response::BandPass => (alpha * norm, 0.0, -alpha * norm),
        Response::Notch => (norm, -2.0 * c * norm, norm),
    };
    Ok(BiquadSection {
        b0,
        b1,
        b2,
        a1: -2.0 * c * norm,
        a2: (1.0 - alpha) * norm,
    })
}

/// Sine and cosine for `x` in `(0, pi)` by fixed-length Taylor series in
/// `f64`, so the design helpers need no math library and run in bounded time
/// (RON-DC-002). Arguments above `pi/2` fold via `sin(pi - x)` and
/// `-cos(pi - x)`; the worst-case error is ~6e-12, far below single
/// precision. Mirrors the C `filter_sincos`.
///
/// **Satisfies:** RON-FR-122, RON-DC-002
fn sin_cos(x: f64) -> (f64, f64) {
    use core::f64::consts::{FRAC_PI_2, PI};
    let (reduced, cos_sign) = if x > FRAC_PI_2 {
        (PI - x, -1.0)
    } else {
        (x, 1.0)
    };
    let x2 = reduced * reduced;
    // Horner form with reciprocal factorials 1/15! .. 1/3! and 1/16! .. 1/2!.
    let sin_terms = [
        -1.0 / 1_307_674_368_000.0,
        1.0 / 6_227_020_800.0,
        -1.0 / 39_916_800.0,
        1.0 / 362_880.0,
        -1.0 / 5_040.0,
        1.0 / 120.0,
        -1.0 / 6.0,
        1.0,
    ];
    let cos_terms = [
        1.0 / 20_922_789_888_000.0,
        -1.0 / 87_178_291_200.0,
        1.0 / 479_001_600.0,
        -1.0 / 3_628_800.0,
        1.0 / 40_320.0,
        -1.0 / 720.0,
        1.0 / 24.0,
        -1.0 / 2.0,
        1.0,
    ];
    let horner = |terms: &[f64]| terms.iter().fold(0.0, |acc, term| (acc * x2) + term);
    (reduced * horner(&sin_terms), cos_sign * horner(&cos_terms))
}

/// Narrows a design-time `f64` result to the library float type.
#[allow(clippy::cast_possible_truncation, clippy::unnecessary_cast)]
fn narrow(value: f64) -> RonFloat {
    value as RonFloat
}
