/*
 * @file     ron_pid_core.c
 * @brief    Core computation helpers for the Regulon PID controller module.
 * @module   ron_pid_core
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-004, RON-FR-005, RON-FR-006, RON-FR-010,
 *           RON-FR-020, RON-FR-030, RON-FR-040, RON-FR-070, RON-SR-010
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_platform.h"

#include "ron_pid_internal.h"

/* Satisfies: RON-FR-010, RON-FR-011 | Test: RON-TC-PID-011, RON-TC-PID-012 */
static ron_float_t pid_normalise(ron_float_t x, ron_float_t x_min, ron_float_t x_max)
{
    return (x - x_min) / (x_max - x_min);
}

/* Satisfies: RON-FR-012 | Test: RON-TC-PID-013 */
static ron_float_t pid_denormalise(ron_float_t x_norm, ron_float_t out_min, ron_float_t out_max)
{
    return (x_norm * (out_max - out_min)) + out_min;
}

/* Satisfies: RON-FR-007, RON-FR-025 | Test: RON-TC-PID-010, RON-TC-PID-019 */
static ron_float_t pid_sp_filter(ron_float_t r, ron_float_t r_prev, ron_float_t tau_sp,
                                 ron_float_t dt)
{
    ron_float_t filtered;

    filtered = r;
    if (tau_sp > RON_FLOAT_C(0.0)) {
        ron_float_t alpha;

        alpha    = tau_sp / (tau_sp + dt);
        filtered = (alpha * r_prev) + ((RON_FLOAT_C(1.0) - alpha) * r);
    }

    return filtered;
}

/* Satisfies: RON-FR-005, RON-FR-006 | Test: RON-TC-PID-006 – RON-TC-PID-009 */
static ron_float_t pid_derivative(const ron_pid_config_t *cfg, const ron_pid_state_t *state,
                                  ron_float_t y_n, ron_float_t e_d, ron_float_t dt)
{
    ron_float_t delta;
    ron_float_t derivative;

    if (cfg->deriv_mode == RON_DERIV_ON_MEASUREMENT) {
        delta = -(y_n - state->y_prev);
    } else {
        delta = e_d - state->e_D_prev;
    }

    derivative = (cfg->Kd * delta) / dt;
    if (cfg->N > RON_FLOAT_C(0.0)) {
        ron_float_t coeff_n;

        coeff_n    = cfg->N * dt;
        derivative = ((coeff_n / (RON_FLOAT_C(1.0) + coeff_n)) * derivative) +
                     ((RON_FLOAT_C(1.0) / (RON_FLOAT_C(1.0) + coeff_n)) * state->D_filt_prev);
    }

    return derivative;
}

/* Satisfies: RON-FR-004, RON-FR-030 – RON-FR-035 | Test: RON-TC-PID-004, RON-TC-PID-005, RON-TC-PID-021 – RON-TC-PID-026 */
static ron_float_t pid_integral(const ron_pid_config_t *cfg, const ron_pid_state_t *state,
                                ron_float_t e, ron_float_t dt)
{
    ron_float_t i_update;
    ron_float_t i_new;

    if (cfg->integ_method == RON_INTEG_TRAPEZOIDAL) {
        i_update = cfg->Ki * dt * RON_FLOAT_C(0.5) * (e + state->e_prev);
    } else {
        i_update = cfg->Ki * dt * e;
    }

    if (cfg->aw_mode == RON_AW_BACK_CALC) {
        i_update += (dt / cfg->T_aw) * (state->u_sat_prev - state->u_prev);
    }

    i_new = state->integral + i_update;
    if (cfg->aw_mode == RON_AW_CLAMPING) {
        const bool saturated_low  = state->u_sat_prev <= (cfg->u_min + RON_FLOAT_EPSILON);
        const bool saturated_high = state->u_sat_prev >= (cfg->u_max - RON_FLOAT_EPSILON);
        const bool same_sign = ((e > RON_FLOAT_C(0.0)) && (state->integral > RON_FLOAT_C(0.0))) ||
                               ((e < RON_FLOAT_C(0.0)) && (state->integral < RON_FLOAT_C(0.0)));

        if ((saturated_low || saturated_high) && same_sign) {
            i_new = state->integral;
        }
    }

    return ron_clamp(i_new, cfg->I_min, cfg->I_max);
}

/* Satisfies: RON-FR-022 | Test: RON-TC-PID-017 */
static ron_float_t pid_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
                                  ron_float_t dt, bool *limited)
{
    ron_float_t limited_value;

    limited_value = u_sat;
    if (du_max <= RON_FLOAT_C(0.0)) {
        *limited = false;
    } else {
        ron_float_t delta_max;
        ron_float_t delta;

        delta_max = du_max * dt;
        delta     = u_sat - u_prev;
        if (delta > delta_max) {
            *limited      = true;
            limited_value = u_prev + delta_max;
        } else if (delta < (-delta_max)) {
            *limited      = true;
            limited_value = u_prev - delta_max;
        } else {
            *limited = false;
        }
    }

    return limited_value;
}

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007, RON-TC-SAFE-010 */
static ron_fault_t pid_fail_step(ron_pid_instance_t *inst, ron_fault_t code, ron_float_t *u_out,
                                 ron_status_t *status)
{
    ron_fault_t fault;

    ron_pid_fault_set(inst, code);
    *u_out  = ron_pid_fault_safe_output(inst);
    *status = inst->state.status;
    fault   = inst->state.fault_code;

    return fault;
}

/* Satisfies: RON-FR-040 | Test: RON-TC-PID-027 */
static bool pid_manual_step(ron_pid_state_t *state, ron_float_t *u_out, ron_status_t *status)
{
    bool handled;

    handled = (state->mode == RON_MODE_MANUAL);
    if (handled) {
        state->status = RON_STATUS_MANUAL_MODE;
        *u_out        = state->u_sat_prev;
        *status       = state->status;
    }

    return handled;
}

/* Satisfies: RON-FR-010 – RON-FR-013, RON-FR-054 | Test: RON-TC-PID-011 – RON-TC-PID-014, RON-TC-PID-034 */
static void pid_prepare_inputs(const ron_pid_config_t *cfg, ron_pid_state_t *state, ron_float_t r,
                               ron_float_t y, ron_float_t dt, ron_float_t *r_n, ron_float_t *y_n,
                               ron_float_t *r_f, ron_status_t *step_status)
{
    if (cfg->normalise) {
        *r_n         = pid_normalise(r, cfg->in_min, cfg->in_max);
        *y_n         = pid_normalise(y, cfg->in_min, cfg->in_max);
        *step_status = (ron_status_t) (*step_status | RON_STATUS_NORMALISED);
    } else {
        *r_n = r;
        *y_n = y;
    }

    *r_f = pid_sp_filter(*r_n, state->r_filt_prev, cfg->tau_sp, dt);
    if (cfg->tau_sp > RON_FLOAT_C(0.0)) {
        *step_status = (ron_status_t) (*step_status | RON_STATUS_SP_FILTER_ACTIVE);
    }
    if ((cfg->sp_reset_threshold > RON_FLOAT_C(0.0)) &&
        (ron_fabs(*r_f - state->r_filt_prev) > cfg->sp_reset_threshold)) {
        state->integral = RON_FLOAT_C(0.0);
    }
}

/* Satisfies: RON-FR-020 – RON-FR-035, RON-SR-010 | Test: RON-TC-PID-015 – RON-TC-PID-026, RON-TC-SAFE-010 */
static ron_fault_t pid_apply_output_limits(ron_pid_instance_t *inst, ron_float_t dt,
                                           ron_float_t i_term, ron_float_t u_raw,
                                           ron_float_t *u_final, ron_status_t *step_status,
                                           ron_float_t *u_out, ron_status_t *status)
{
    const ron_pid_config_t *cfg;
    ron_fault_t fault;

    cfg   = &inst->config;
    fault = RON_FAULT_NONE;
    if (!RON_ISFINITE(u_raw)) {
        fault = pid_fail_step(inst, RON_FAULT_OUTPUT_NAN, u_out, status);
    } else {
        ron_float_t u_sat;
        bool rate_limited;

        u_sat = ron_clamp(u_raw, cfg->u_min, cfg->u_max);
        if (u_sat != u_raw) {
            *step_status = (ron_status_t) (*step_status | RON_STATUS_SATURATED);
        }

        *u_final = pid_rate_limit(u_sat, inst->state.u_sat_prev, cfg->du_max, dt, &rate_limited);
        if (rate_limited) {
            *step_status = (ron_status_t) (*step_status | RON_STATUS_RATE_LIMITED);
        }
        if ((cfg->aw_mode != RON_AW_NONE) && ((*step_status & RON_STATUS_SATURATED) != 0U)) {
            *step_status = (ron_status_t) (*step_status | RON_STATUS_AW_ACTIVE);
        }
        if ((cfg->I_overflow_thresh > RON_FLOAT_C(0.0)) &&
            (ron_fabs(i_term) > cfg->I_overflow_thresh)) {
            fault = pid_fail_step(inst, RON_FAULT_INTEGRAL_OVERFLOW, u_out, status);
        }
    }

    return fault;
}

/* Satisfies: RON-FR-050, RON-FR-070 | Test: RON-TC-PID-030, RON-TC-PID-039 */
static void pid_store_step(ron_pid_state_t *state, ron_float_t y_n, ron_float_t e, ron_float_t e_d,
                           ron_float_t d_term, ron_float_t r_f, ron_float_t i_term,
                           ron_float_t u_raw, ron_float_t u_final, ron_status_t step_status)
{
    state->integral    = i_term;
    state->y_prev      = y_n;
    state->e_D_prev    = e_d;
    state->D_filt_prev = d_term;
    state->r_filt_prev = r_f;
    state->u_sat_prev  = u_final;
    state->u_prev      = u_raw;
    state->e_prev      = e;
    state->status      = step_status;
}

/* Satisfies: RON-FR-001 – RON-FR-071 | Test: RON-TC-PID-001 – RON-TC-PID-039 */
ron_fault_t ron_pid_core_step(ron_pid_instance_t *inst, ron_float_t r, ron_float_t y,
                              ron_float_t dt, ron_float_t *u_out, ron_status_t *status)
{
    const ron_pid_config_t *cfg;
    ron_pid_state_t *state;
    ron_status_t step_status;
    ron_float_t r_n;
    ron_float_t y_n;
    ron_float_t r_f;
    ron_fault_t fault;

    cfg         = &inst->config;
    state       = &inst->state;
    step_status = RON_STATUS_OK;

    if (!RON_ISFINITE(r) || !RON_ISFINITE(y)) {
        fault = pid_fail_step(inst, RON_FAULT_INPUT_NAN, u_out, status);
    } else if (pid_manual_step(state, u_out, status)) {
        fault = RON_FAULT_NONE;
    } else {
        ron_float_t e;
        ron_float_t e_p;
        ron_float_t e_d;
        ron_float_t p_term;
        ron_float_t d_term;
        ron_float_t i_term;
        ron_float_t u_raw;
        ron_float_t u_final;

        pid_prepare_inputs(cfg, state, r, y, dt, &r_n, &y_n, &r_f, &step_status);

        e      = r_f - y_n;
        e_p    = (cfg->b * r_f) - y_n;
        e_d    = (cfg->c * r_f) - y_n;
        p_term = cfg->Kp * e_p;
        d_term = pid_derivative(cfg, state, y_n, e_d, dt);
        i_term = pid_integral(cfg, state, e, dt);
        u_raw  = p_term + i_term + d_term;
        if (cfg->normalise) {
            u_raw = pid_denormalise(u_raw, cfg->out_min, cfg->out_max);
        }

        fault =
            pid_apply_output_limits(inst, dt, i_term, u_raw, &u_final, &step_status, u_out, status);
        if (fault == RON_FAULT_NONE) {
            pid_store_step(state, y_n, e, e_d, d_term, r_f, i_term, u_raw, u_final, step_status);
            *u_out  = u_final;
            *status = step_status;
        }
    }

    return fault;
}
