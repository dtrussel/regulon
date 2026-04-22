//! # `pid::types`
//!
//! Public PID types and private runtime state.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-PR-021, RON-SR-010-RON-SR-013,
//! RON-QR-021
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use core::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, Not};

use crate::platform::RonFloat;

/// Anti-windup strategy.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AntiWindupMode {
    /// Back-calculation tracking.
    #[default]
    BackCalculation,
    /// Conditional integration clamping.
    ConditionalIntegration,
    /// Anti-windup disabled.
    Disabled,
}

/// Derivative mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DerivativeMode {
    /// Derivative on process variable to avoid setpoint kick.
    #[default]
    OnMeasurement,
    /// Derivative on error.
    OnError,
}

/// Integral update method.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntegrationMethod {
    /// Forward Euler integration.
    #[default]
    Euler,
    /// Trapezoidal integration.
    Trapezoidal,
}

/// Operating mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PidMode {
    /// Closed-loop PID control.
    #[default]
    Automatic,
    /// Manual output tracking.
    Manual,
}

/// Safe-state output selection.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SafePolicy {
    /// Keep the last valid output.
    #[default]
    HoldLast,
    /// Drive to zero.
    DriveZero,
    /// Drive to a configured safe value.
    DriveSafeValue,
}

/// Normalization domain.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NormalizationRange {
    /// Normalize into `[0, 1]`.
    #[default]
    ZeroToOne,
    /// Normalize into `[-1, 1]`.
    NegativeOneToOne,
}

/// Feed-forward mode selection.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FeedForwardMode {
    /// Feed-forward path disabled.
    #[default]
    Disabled,
    /// Static gain applied to the setpoint.
    StaticGain,
    /// Higher-order feed-forward modes planned for a later sprint.
    Reserved,
}

/// Feed-forward configuration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedForwardConfig {
    /// Selected feed-forward mode.
    pub mode: FeedForwardMode,
    /// Static gain applied when `mode` is `StaticGain`.
    pub static_gain: RonFloat,
}

/// Optional normalization configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalizationConfig {
    /// Minimum engineering-unit input value.
    pub input_min: RonFloat,
    /// Maximum engineering-unit input value.
    pub input_max: RonFloat,
    /// Minimum engineering-unit output value.
    pub output_min: RonFloat,
    /// Maximum engineering-unit output value.
    pub output_max: RonFloat,
    /// Target normalization range.
    pub range: NormalizationRange,
}

/// Fault register bitfield.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PidFault(u8);

impl PidFault {
    /// No fault bits set.
    pub const NONE: Self = Self(0x00);
    /// An input or sample period is NaN or infinite.
    pub const INPUT_NOT_FINITE: Self = Self(0x01);
    /// The computed control variable is NaN or infinite.
    pub const OUTPUT_NOT_FINITE: Self = Self(0x02);
    /// The integral accumulator exceeded the configured threshold.
    pub const INTEGRAL_OVERFLOW: Self = Self(0x04);
    /// The active configuration is invalid.
    pub const CONFIG_INVALID: Self = Self(0x08);

    /// Returns the raw bit representation.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// Returns `true` when no fault bits are set.
    #[must_use]
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }

    /// Returns `true` when all bits from `other` are present.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }
}

impl BitOr for PidFault {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for PidFault {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for PidFault {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitAndAssign for PidFault {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl Not for PidFault {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Status word bitfield.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PidStatus(u16);

impl PidStatus {
    /// No status bits set.
    pub const OK: Self = Self(0x0000);
    /// Output clipping occurred.
    pub const SATURATED: Self = Self(0x0001);
    /// Rate limiting occurred.
    pub const RATE_LIMITED: Self = Self(0x0002);
    /// Anti-windup correction or clamping was active.
    pub const ANTI_WINDUP_ACTIVE: Self = Self(0x0004);
    /// Controller is currently in manual mode.
    pub const MANUAL_MODE: Self = Self(0x0008);
    /// Fault is latched.
    pub const FAULT: Self = Self(0x0010);
    /// Input and output normalization are active.
    pub const NORMALIZED: Self = Self(0x0020);
    /// Feed-forward contributed to the output.
    pub const FEED_FORWARD_ACTIVE: Self = Self(0x0040);

    /// Returns the raw bit representation.
    #[must_use]
    pub const fn bits(self) -> u16 {
        self.0
    }

    /// Returns `true` when all bits from `other` are present.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }
}

impl BitOr for PidStatus {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Self(self.0 | rhs.0)
    }
}

impl BitOrAssign for PidStatus {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for PidStatus {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Self(self.0 & rhs.0)
    }
}

impl BitAndAssign for PidStatus {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl Not for PidStatus {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Public PID configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidConfig {
    /// Proportional gain.
    pub kp: RonFloat,
    /// Integral gain.
    pub ki: RonFloat,
    /// Derivative gain.
    pub kd: RonFloat,
    /// Derivative low-pass filter bandwidth multiplier.
    pub derivative_filter: RonFloat,
    /// Two-degree-of-freedom proportional weight.
    pub proportional_weight: RonFloat,
    /// Two-degree-of-freedom derivative weight.
    pub derivative_weight: RonFloat,
    /// Minimum output in engineering units.
    pub output_min: RonFloat,
    /// Maximum output in engineering units.
    pub output_max: RonFloat,
    /// Maximum change in output per second. Disabled when `<= 0`.
    pub rate_limit: RonFloat,
    /// Lower integral clamp.
    pub integral_min: RonFloat,
    /// Upper integral clamp.
    pub integral_max: RonFloat,
    /// Anti-windup strategy.
    pub anti_windup_mode: AntiWindupMode,
    /// Back-calculation tracking time constant.
    pub anti_windup_tracking_time: RonFloat,
    /// Integral update method.
    pub integration_method: IntegrationMethod,
    /// Derivative mode.
    pub derivative_mode: DerivativeMode,
    /// Optional setpoint pre-filter time constant.
    pub setpoint_filter_tau: RonFloat,
    /// Optional input/output normalization.
    pub normalization: Option<NormalizationConfig>,
    /// Feed-forward configuration.
    pub feed_forward: FeedForwardConfig,
    /// Safe-state policy.
    pub safe_policy: SafePolicy,
    /// Safe output value when `safe_policy` is `DriveSafeValue`.
    pub safe_value: RonFloat,
    /// Optional integral overflow threshold. Disabled when `<= 0`.
    pub integral_overflow_threshold: RonFloat,
    /// Optional setpoint-change threshold that clears the integral term.
    pub setpoint_reset_threshold: RonFloat,
    /// Initial mode after construction.
    pub initial_mode: PidMode,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            derivative_filter: 0.0,
            proportional_weight: 1.0,
            derivative_weight: 1.0,
            output_min: -1.0e6,
            output_max: 1.0e6,
            rate_limit: 0.0,
            integral_min: -1.0e6,
            integral_max: 1.0e6,
            anti_windup_mode: AntiWindupMode::BackCalculation,
            anti_windup_tracking_time: 0.1,
            integration_method: IntegrationMethod::Euler,
            derivative_mode: DerivativeMode::OnMeasurement,
            setpoint_filter_tau: 0.0,
            normalization: None,
            feed_forward: FeedForwardConfig::default(),
            safe_policy: SafePolicy::HoldLast,
            safe_value: 0.0,
            integral_overflow_threshold: 0.0,
            setpoint_reset_threshold: 0.0,
            initial_mode: PidMode::Automatic,
        }
    }
}

impl PidConfig {
    /// Creates a parallel-form PID configuration.
    #[must_use]
    pub fn new_parallel(kp: RonFloat, ki: RonFloat, kd: RonFloat) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Self::default()
        }
    }

    /// Creates an ideal-form PID configuration.
    #[must_use]
    pub fn new_ideal(kp: RonFloat, ti: RonFloat, td: RonFloat) -> Self {
        let mut config = Self::new_parallel(kp, 0.0, 0.0);
        if ti.is_finite() && ti > 0.0 {
            config.ki = kp / ti;
        }
        if td.is_finite() && td >= 0.0 {
            config.kd = kp * td;
        }
        config
    }
}

/// Public read-only state snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidSnapshot {
    /// Current integral accumulator.
    pub integral: RonFloat,
    /// Last filtered derivative term.
    pub last_derivative: RonFloat,
    /// Last saturated and rate-limited output.
    pub last_output: RonFloat,
    /// Last unsaturated output prior to limiting.
    pub last_unbounded_output: RonFloat,
    /// Last control error.
    pub last_error: RonFloat,
    /// Last filtered setpoint.
    pub last_setpoint: RonFloat,
    /// Last feed-forward contribution applied by the controller.
    pub last_feed_forward: RonFloat,
    /// Current operating mode.
    pub mode: PidMode,
    /// Last status word.
    pub status: PidStatus,
    /// Current fault register.
    pub fault: PidFault,
}

/// Private mutable PID runtime state.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct PidRuntime {
    pub(crate) integral: RonFloat,
    pub(crate) integral_compensation: RonFloat,
    pub(crate) y_prev: RonFloat,
    pub(crate) derivative_error_prev: RonFloat,
    pub(crate) derivative_filtered_prev: RonFloat,
    pub(crate) setpoint_filtered_prev: RonFloat,
    pub(crate) output_prev: RonFloat,
    pub(crate) output_unbounded_prev: RonFloat,
    pub(crate) error_prev: RonFloat,
    pub(crate) setpoint_prev: RonFloat,
    pub(crate) feed_forward_prev: RonFloat,
    pub(crate) mode: PidMode,
    pub(crate) status: PidStatus,
    pub(crate) fault: PidFault,
}

impl PidRuntime {
    #[must_use]
    pub(crate) fn snapshot(self) -> PidSnapshot {
        PidSnapshot {
            integral: self.integral,
            last_derivative: self.derivative_filtered_prev,
            last_output: self.output_prev,
            last_unbounded_output: self.output_unbounded_prev,
            last_error: self.error_prev,
            last_setpoint: self.setpoint_filtered_prev,
            last_feed_forward: self.feed_forward_prev,
            mode: self.mode,
            status: self.status,
            fault: self.fault,
        }
    }
}
