//! # `regulon`
//!
//! Rust-first implementation of the Regulon PID controller baseline.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-PR-001-RON-PR-022,
//! RON-SR-001-RON-SR-033, RON-QR-001-RON-QR-031
//! **SPDX-License-Identifier:** MIT

#![no_std]
#![deny(clippy::all, clippy::pedantic, missing_docs)]

#[cfg(test)]
extern crate std;

pub mod autotune;
pub mod cascade;
pub mod error;
pub mod estimator;
pub mod filter;
pub mod gain_sched;
pub mod health;
pub mod kalman;
pub mod matrix;
pub mod metrics;
pub mod observer;
pub mod pid;
pub mod platform;
pub mod statespace;
pub mod trajectory;

pub use autotune::{AutotuneConfig, AutotunePhase, AutotuneResults, Autotuner, TuningRule};
pub use cascade::{Cascade, CascadeStatus};
pub use error::RonError;
pub use estimator::{Estimator, EstimatorConfig, EstimatorSource};
pub use filter::{
    FilterFault, FilterSnapshot, FilterStatus, Lp1, Lp1Config, RateLimiter, RateLimiterConfig,
};
pub use gain_sched::{GainSchedule, ScheduleMode, GS_MAX_BREAKPOINTS};
pub use health::{HealthCallback, HealthConfig, HealthMonitor, HealthStatus};
pub use kalman::{Kalman, KalmanConfig};
pub use matrix::{Cholesky, Matrix, MATRIX_MAX_DIM};
pub use metrics::{Metrics, MetricsConfig, MetricsMode, MetricsResults, StepFrame};
pub use observer::{Observer, ObserverConfig};
pub use pid::{
    AntiWindupMode, DerivativeMode, FeedForwardConfig, FeedForwardMode, IntegrationMethod,
    NormalizationConfig, NormalizationRange, Pid, PidConfig, PidFault, PidMode, PidSnapshot,
    PidStatus, SafePolicy,
};
pub use platform::RonFloat;
pub use statespace::{IntegralAugmentation, OutputLimits, StateSpace, StateSpaceConfig};
pub use trajectory::{
    SCurve, SCurveConfig, Setpoint, TrajectoryFault, Trapezoidal, TrapezoidalConfig,
};
