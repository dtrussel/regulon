//! # `pid::core`
//!
//! Core PID computation helpers.
//!
//! **Document:** RON-IS-001
//! **Requirements:** RON-FR-001-RON-FR-071, RON-PR-001-RON-PR-004,
//! RON-SR-010-RON-SR-022
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use crate::{
    error::RonError,
    platform::{abs, clamp, is_finite, same_sign_nonzero, RonFloat},
};

use super::{
    AntiWindupMode, DerivativeMode, FeedForwardMode, IntegrationMethod, PidConfig, PidFault,
    PidMode, PidRuntime, PidStatus, SafePolicy,
};

#[derive(Clone, Copy)]
struct StepSignals {
    setpoint: RonFloat,
    measurement: RonFloat,
    filtered_setpoint: RonFloat,
    proportional_error: RonFloat,
    derivative_error: RonFloat,
    error: RonFloat,
}

#[derive(Clone, Copy)]
struct StepOutputs {
    controller_raw: RonFloat,
    output_raw: RonFloat,
    output_saturated: RonFloat,
    output_final: RonFloat,
    derivative: RonFloat,
    feed_forward: RonFloat,
    integral: RonFloat,
    integral_compensation: RonFloat,
    status: PidStatus,
}

pub(crate) fn step(
    config: PidConfig,
    state: &mut PidRuntime,
    setpoint: RonFloat,
    measurement: RonFloat,
    dt: RonFloat,
) -> Result<(RonFloat, PidStatus), RonError> {
    validate_inputs(dt, setpoint, measurement)?;
    if !state.fault.is_none() {
        state.output_prev = safe_state_output(config, state.output_prev);
        state.status = PidStatus::FAULT;
        return Err(RonError::Fault(state.fault));
    }
    if matches!(state.mode, PidMode::Manual) {
        state.output_prev = clamp(state.output_prev, config.output_min, config.output_max);
        state.status = PidStatus::MANUAL_MODE;
        return Ok((state.output_prev, state.status));
    }

    let signals = compute_signals(config, state, setpoint, measurement, dt);
    let provisional = compute_provisional_output(config, state, signals, dt);
    let outputs = apply_anti_windup(config, state, signals, provisional, dt);
    complete_step(config, state, setpoint, measurement, signals, outputs)
}

pub(crate) fn clear_fault(state: &mut PidRuntime) {
    state.fault = PidFault::NONE;
    state.status = PidStatus::OK;
}

fn validate_inputs(
    dt: RonFloat,
    setpoint: RonFloat,
    measurement: RonFloat,
) -> Result<(), RonError> {
    if !is_finite(setpoint) || !is_finite(measurement) || !is_finite(dt) || dt <= 0.0 {
        return Err(RonError::Fault(PidFault::INPUT_NOT_FINITE));
    }
    Ok(())
}

fn compute_signals(
    config: PidConfig,
    state: &PidRuntime,
    setpoint: RonFloat,
    measurement: RonFloat,
    dt: RonFloat,
) -> StepSignals {
    let mapped_setpoint = config.map_input(setpoint);
    let mapped_measurement = config.map_input(measurement);
    let filtered_setpoint = filter_setpoint(config, state, mapped_setpoint, dt);
    let error = filtered_setpoint - mapped_measurement;
    let proportional_error = (config.proportional_weight * filtered_setpoint) - mapped_measurement;
    let derivative_error = (config.derivative_weight * filtered_setpoint) - mapped_measurement;
    StepSignals {
        setpoint: mapped_setpoint,
        measurement: mapped_measurement,
        filtered_setpoint,
        proportional_error,
        derivative_error,
        error,
    }
}

fn filter_setpoint(
    config: PidConfig,
    state: &PidRuntime,
    setpoint: RonFloat,
    dt: RonFloat,
) -> RonFloat {
    if config.setpoint_filter_tau <= 0.0 {
        return setpoint;
    }
    let alpha = dt / (config.setpoint_filter_tau + dt);
    state.setpoint_filtered_prev + (alpha * (setpoint - state.setpoint_filtered_prev))
}

fn compute_provisional_output(
    config: PidConfig,
    state: &PidRuntime,
    signals: StepSignals,
    dt: RonFloat,
) -> StepOutputs {
    let proportional = config.kp * signals.proportional_error;
    let derivative = compute_derivative(config, state, signals, dt);
    let feed_forward = compute_feed_forward(config, signals);
    let (integral, integral_compensation) =
        compute_integral_candidate(config, state, signals.error, signals.setpoint, dt);
    let controller_raw = proportional + integral + derivative + feed_forward;
    let output_raw = config.map_output_from_controller(controller_raw);
    let (output_saturated, saturation_active) = saturate(config, output_raw);
    let (output_final, rate_limited) = rate_limit(config, state, output_saturated, dt);
    let mut status = PidStatus::OK;
    if saturation_active {
        status |= PidStatus::SATURATED;
    }
    if rate_limited {
        status |= PidStatus::RATE_LIMITED;
    }
    if config.normalization.is_some() {
        status |= PidStatus::NORMALIZED;
    }
    if differs(feed_forward, 0.0) {
        status |= PidStatus::FEED_FORWARD_ACTIVE;
    }
    StepOutputs {
        controller_raw,
        output_raw,
        output_saturated,
        output_final,
        derivative,
        feed_forward,
        integral,
        integral_compensation,
        status,
    }
}

fn compute_feed_forward(config: PidConfig, signals: StepSignals) -> RonFloat {
    match config.feed_forward.mode {
        FeedForwardMode::Disabled | FeedForwardMode::Reserved => 0.0,
        FeedForwardMode::StaticGain => config.feed_forward.static_gain * signals.filtered_setpoint,
    }
}

fn compute_derivative(
    config: PidConfig,
    state: &PidRuntime,
    signals: StepSignals,
    dt: RonFloat,
) -> RonFloat {
    let delta = match config.derivative_mode {
        DerivativeMode::OnMeasurement => -(signals.measurement - state.y_prev),
        DerivativeMode::OnError => signals.derivative_error - state.derivative_error_prev,
    };
    let raw_derivative = config.kd * (delta / dt);
    if config.derivative_filter <= 0.0 {
        return raw_derivative;
    }
    let tau = 1.0 / config.derivative_filter;
    let alpha = dt / (tau + dt);
    state.derivative_filtered_prev + (alpha * (raw_derivative - state.derivative_filtered_prev))
}

fn compute_integral_candidate(
    config: PidConfig,
    state: &PidRuntime,
    error: RonFloat,
    setpoint: RonFloat,
    dt: RonFloat,
) -> (RonFloat, RonFloat) {
    let reset_integral = config.setpoint_reset_threshold > 0.0
        && abs(setpoint - state.setpoint_prev) > config.setpoint_reset_threshold;
    let integral_base = if reset_integral { 0.0 } else { state.integral };
    let compensation_base = if reset_integral {
        0.0
    } else {
        state.integral_compensation
    };
    let increment = match config.integration_method {
        IntegrationMethod::Euler => config.ki * dt * error,
        IntegrationMethod::Trapezoidal => config.ki * dt * (error + state.error_prev) * 0.5,
    };
    let corrected_increment = increment - compensation_base;
    let summed = integral_base + corrected_increment;
    let compensation = (summed - integral_base) - corrected_increment;
    (
        clamp(summed, config.integral_min, config.integral_max),
        compensation,
    )
}

fn apply_anti_windup(
    config: PidConfig,
    state: &PidRuntime,
    signals: StepSignals,
    provisional: StepOutputs,
    dt: RonFloat,
) -> StepOutputs {
    let mut outputs = provisional;
    match config.anti_windup_mode {
        AntiWindupMode::Disabled => {}
        AntiWindupMode::BackCalculation => {
            let windup_blocked = (provisional.status.contains(PidStatus::SATURATED)
                || provisional.status.contains(PidStatus::RATE_LIMITED))
                && same_sign_nonzero(signals.error, provisional.output_final);
            let tracking_error = config.map_output_to_controller(provisional.output_final)
                - provisional.controller_raw;
            if differs(tracking_error, 0.0) {
                let integral_base = if windup_blocked {
                    state.integral
                } else {
                    provisional.integral
                };
                outputs.integral = clamp(
                    integral_base + ((dt / config.anti_windup_tracking_time) * tracking_error),
                    config.integral_min,
                    config.integral_max,
                );
                outputs.integral_compensation = 0.0;
                outputs.status |= PidStatus::ANTI_WINDUP_ACTIVE;
                outputs = recompute_outputs(config, state, signals, dt, outputs);
            }
        }
        AntiWindupMode::ConditionalIntegration => {
            let saturated = provisional.status.contains(PidStatus::SATURATED)
                || provisional.status.contains(PidStatus::RATE_LIMITED);
            if saturated && same_sign_nonzero(signals.error, provisional.integral) {
                outputs.integral = state.integral;
                outputs.integral_compensation = state.integral_compensation;
                outputs.status |= PidStatus::ANTI_WINDUP_ACTIVE;
                outputs = recompute_outputs(config, state, signals, dt, outputs);
            }
        }
    }
    outputs
}

fn recompute_outputs(
    config: PidConfig,
    state: &PidRuntime,
    signals: StepSignals,
    dt: RonFloat,
    mut outputs: StepOutputs,
) -> StepOutputs {
    let proportional = config.kp * signals.proportional_error;
    outputs.controller_raw =
        proportional + outputs.integral + outputs.derivative + outputs.feed_forward;
    outputs.output_raw = config.map_output_from_controller(outputs.controller_raw);
    let (saturated, saturation_active) = saturate(config, outputs.output_raw);
    let (final_output, rate_limited) = rate_limit(config, state, saturated, dt);
    outputs.output_saturated = saturated;
    outputs.output_final = final_output;
    outputs.status &= !PidStatus::SATURATED;
    outputs.status &= !PidStatus::RATE_LIMITED;
    if saturation_active {
        outputs.status |= PidStatus::SATURATED;
    }
    if rate_limited {
        outputs.status |= PidStatus::RATE_LIMITED;
    }
    outputs
}

fn saturate(config: PidConfig, output: RonFloat) -> (RonFloat, bool) {
    let clamped = clamp(output, config.output_min, config.output_max);
    (clamped, differs(clamped, output))
}

fn rate_limit(
    config: PidConfig,
    state: &PidRuntime,
    output: RonFloat,
    dt: RonFloat,
) -> (RonFloat, bool) {
    if config.rate_limit <= 0.0 {
        return (output, false);
    }
    let delta_max = config.rate_limit * dt;
    let delta = clamp(output - state.output_prev, -delta_max, delta_max);
    let limited = state.output_prev + delta;
    (limited, differs(limited, output))
}

fn complete_step(
    config: PidConfig,
    state: &mut PidRuntime,
    setpoint_raw: RonFloat,
    measurement_raw: RonFloat,
    signals: StepSignals,
    outputs: StepOutputs,
) -> Result<(RonFloat, PidStatus), RonError> {
    if !is_finite(outputs.output_raw) || !is_finite(outputs.output_final) {
        return latch_fault(config, state, PidFault::OUTPUT_NOT_FINITE);
    }
    if config.integral_overflow_threshold > 0.0
        && abs(outputs.integral) > config.integral_overflow_threshold
    {
        return latch_fault(config, state, PidFault::INTEGRAL_OVERFLOW);
    }

    state.integral = outputs.integral;
    state.integral_compensation = outputs.integral_compensation;
    state.y_prev = signals.measurement;
    state.derivative_error_prev = signals.derivative_error;
    state.derivative_filtered_prev = outputs.derivative;
    state.setpoint_filtered_prev = signals.filtered_setpoint;
    state.output_prev = outputs.output_final;
    state.output_unbounded_prev = outputs.output_raw;
    state.error_prev = signals.error;
    state.setpoint_prev = signals.setpoint;
    state.feed_forward_prev = outputs.feed_forward;
    state.status = outputs.status;
    if !is_finite(setpoint_raw) || !is_finite(measurement_raw) {
        return latch_fault(config, state, PidFault::INPUT_NOT_FINITE);
    }
    Ok((outputs.output_final, outputs.status))
}

fn latch_fault(
    config: PidConfig,
    state: &mut PidRuntime,
    fault: PidFault,
) -> Result<(RonFloat, PidStatus), RonError> {
    state.fault |= fault;
    state.status = PidStatus::FAULT;
    state.output_prev = safe_state_output(config, state.output_prev);
    Err(RonError::Fault(state.fault))
}

fn safe_state_output(config: PidConfig, last_output: RonFloat) -> RonFloat {
    match config.safe_policy {
        SafePolicy::HoldLast => clamp(last_output, config.output_min, config.output_max),
        SafePolicy::DriveZero => clamp(0.0, config.output_min, config.output_max),
        SafePolicy::DriveSafeValue => {
            clamp(config.safe_value, config.output_min, config.output_max)
        }
    }
}

fn differs(lhs: RonFloat, rhs: RonFloat) -> bool {
    abs(lhs - rhs) > (RonFloat::EPSILON * 4.0)
}
