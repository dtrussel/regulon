//! # `filter::moving_average`
//!
//! Causal boxcar FIR filter `y(k) = (1/M) * sum x(k-i)` over a statically
//! sized window, updated by a constant-time sliding sum.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-100-RON-FR-103, RON-FR-115-RON-FR-117
//! **Tests:** RON-TC-FILT-008-RON-TC-FILT-010
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use super::{FilterFault, FilterSnapshot, FilterStatus};
use crate::platform::{is_finite, RonFloat};

/// Largest moving-average window.
///
/// **Satisfies:** RON-FR-116
pub const MA_MAX_WINDOW: usize = 64;

/// Moving-average filter with a window of `M` samples.
///
/// Until `M` samples have arrived the empty slots count as zero, so the
/// output ramps up from zero as in the C implementation.
///
/// **Satisfies:** RON-FR-115, RON-FR-116
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MovingAverage<const M: usize> {
    buffer: [RonFloat; M],
    sum: RonFloat,
    index: usize,
    count: usize,
    state: FilterSnapshot,
}

impl<const M: usize> MovingAverage<M> {
    /// Creates an empty filter.
    ///
    /// **Satisfies:** RON-FR-103, RON-FR-116
    ///
    /// # Errors
    ///
    /// Returns [`FilterFault::CONFIG_INVALID`] when `M` is 0 or above
    /// [`MA_MAX_WINDOW`].
    pub fn new() -> Result<Self, FilterFault> {
        if M == 0 || M > MA_MAX_WINDOW {
            return Err(FilterFault::CONFIG_INVALID);
        }
        Ok(Self::empty())
    }

    /// Clears the history and any latched fault.
    ///
    /// **Satisfies:** RON-FR-102
    pub fn reset(&mut self) {
        *self = Self::empty();
    }

    /// Applies one input sample in constant time: the oldest sample leaves
    /// the running sum and the new one enters it.
    ///
    /// **Satisfies:** RON-FR-101, RON-FR-115, RON-FR-117
    ///
    /// # Errors
    ///
    /// Returns the latched fault, or latches and returns
    /// [`FilterFault::INPUT_NOT_FINITE`] / [`FilterFault::OUTPUT_NOT_FINITE`];
    /// the window is unchanged on error.
    pub fn step(&mut self, input: RonFloat) -> Result<(RonFloat, FilterStatus), FilterFault> {
        if !self.state.fault.is_none() {
            self.state.status = FilterStatus::FAULT;
            return Err(self.state.fault);
        }
        if !is_finite(input) {
            return Err(self.latch(FilterFault::INPUT_NOT_FINITE));
        }
        let oldest = self.buffer.get(self.index).copied().unwrap_or(0.0);
        let sum = (self.sum + input) - oldest;
        let output = sum / window_length::<M>();
        if !is_finite(output) {
            return Err(self.latch(FilterFault::OUTPUT_NOT_FINITE));
        }
        if let Some(slot) = self.buffer.get_mut(self.index) {
            *slot = input;
        }
        self.sum = sum;
        self.index = (self.index + 1) % M;
        self.count = (self.count + 1).min(M);
        self.state.last_output = output;
        self.state.status = FilterStatus::OK;
        Ok((output, self.state.status))
    }

    /// Returns the running sum of the window.
    #[must_use]
    pub const fn sum(&self) -> RonFloat {
        self.sum
    }

    /// Returns how many samples the window holds (at most `M`).
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// Returns a state snapshot.
    #[must_use]
    pub const fn state(&self) -> FilterSnapshot {
        self.state
    }

    const fn empty() -> Self {
        Self {
            buffer: [0.0; M],
            sum: 0.0,
            index: 0,
            count: 0,
            state: FilterSnapshot {
                last_output: 0.0,
                status: FilterStatus::OK,
                fault: FilterFault::NONE,
            },
        }
    }

    fn latch(&mut self, fault: FilterFault) -> FilterFault {
        self.state.fault = fault;
        self.state.status = FilterStatus::FAULT;
        fault
    }
}

/// `M` as a float divisor; `M <= MA_MAX_WINDOW` so the conversion is exact.
fn window_length<const M: usize>() -> RonFloat {
    RonFloat::from(u8::try_from(M).unwrap_or(u8::MAX))
}
