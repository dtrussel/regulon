/*
 * @file     ron_pid_api.c
 * @brief    Public API entry points for the Regulon PID controller module.
 * @module   ron_pid_api
 * @doc      RON-IS-001
 * @req      RON-FR-040, RON-FR-050, RON-FR-051, RON-FR-052, RON-FR-053,
 *           RON-FR-071, RON-SR-001, RON-SR-006, RON-SR-012
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_platform.h"

#include "ron_pid_internal.h"

/* Satisfies: RON-SR-006 | Test: RON-TC-SAFE-006 */
static ron_fault_t pid_check_inst(const ron_pid_instance_t *inst)
{
    if (inst == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!inst->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static bool pid_api_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* Satisfies: RON-FR-050, RON-FR-051 | Test: RON-TC-PID-030, RON-TC-PID-031 */
static void pid_reset_state(ron_pid_state_t *state)
{
    state->integral    = RON_FLOAT_C(0.0);
    state->y_prev      = RON_FLOAT_C(0.0);
    state->e_D_prev    = RON_FLOAT_C(0.0);
    state->D_filt_prev = RON_FLOAT_C(0.0);
    state->r_filt_prev = RON_FLOAT_C(0.0);
    state->u_sat_prev  = RON_FLOAT_C(0.0);
    state->u_prev      = RON_FLOAT_C(0.0);
    state->e_prev      = RON_FLOAT_C(0.0);
    state->ff_r_prev   = RON_FLOAT_C(0.0);
    state->ff_v_prev   = RON_FLOAT_C(0.0);
    state->ff_a_prev   = RON_FLOAT_C(0.0);
    state->u_ff_prev   = RON_FLOAT_C(0.0);
    state->fault_code  = RON_FAULT_NONE;
}

/* Satisfies: RON-FR-053 | Test: RON-TC-PID-033 */
static ron_fault_t pid_apply_candidate(ron_pid_instance_t *inst, const ron_pid_config_t *candidate)
{
    ron_fault_t fault;

    fault = ron_pid_config_validate(candidate);
    if (fault == RON_FAULT_NONE) {
        inst->config = *candidate;
    }

    return fault;
}

/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
ron_fault_t ron_pid_init(ron_pid_instance_t *inst, const ron_pid_config_t *cfg)
{
    ron_fault_t fault;

    if ((inst == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = ron_pid_config_validate(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    inst->config = *cfg;
    pid_reset_state(&inst->state);
    inst->state.status         = RON_STATUS_OK;
    inst->state.mode           = RON_MODE_AUTOMATIC;
    inst->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-051 | Test: RON-TC-PID-031 */
ron_fault_t ron_pid_reset(ron_pid_instance_t *inst)
{
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    pid_reset_state(&inst->state);
    inst->state.status =
        (inst->state.mode == RON_MODE_MANUAL) ? RON_STATUS_MANUAL_MODE : RON_STATUS_OK;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-001 – RON-FR-035 | Test: RON-TC-PID-001 – RON-TC-PID-026 */
ron_fault_t ron_pid_step(ron_pid_instance_t *inst, ron_float_t r, ron_float_t y, ron_float_t dt,
                         ron_float_t *u_out, ron_status_t *status)
{
    ron_fault_t fault;

    if ((inst == NULL) || (u_out == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!inst->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (inst->state.fault_code != RON_FAULT_NONE) {
        *u_out  = ron_pid_fault_safe_output(inst);
        *status = inst->state.status;
        return inst->state.fault_code;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !pid_api_isfinite(dt)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (inst->config.feedforward.mode == RON_FF_EXTERNAL) {
        return RON_FAULT_CONFIG_INVALID;
    }

    fault = ron_pid_core_step(inst, r, y, dt, RON_FLOAT_C(0.0), u_out, status);
    return fault;
}

/* Satisfies: RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_gains(ron_pid_instance_t *inst, ron_float_t Kp, ron_float_t Ki,
                              ron_float_t Kd)
{
    ron_pid_config_t candidate;
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    candidate    = inst->config;
    candidate.Kp = Kp;
    candidate.Ki = Ki;
    candidate.Kd = Kd;
    return pid_apply_candidate(inst, &candidate);
}

/* Satisfies: RON-FR-021, RON-FR-053 | Test: RON-TC-PID-016 */
ron_fault_t ron_pid_set_limits(ron_pid_instance_t *inst, ron_float_t u_min, ron_float_t u_max)
{
    ron_pid_config_t candidate;
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    candidate       = inst->config;
    candidate.u_min = u_min;
    candidate.u_max = u_max;
    return pid_apply_candidate(inst, &candidate);
}

/* Satisfies: RON-FR-006, RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_filter(ron_pid_instance_t *inst, ron_float_t N)
{
    ron_pid_config_t candidate;
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    candidate   = inst->config;
    candidate.N = N;
    return pid_apply_candidate(inst, &candidate);
}

/* Satisfies: RON-FR-033, RON-FR-053 | Test: RON-TC-PID-024 */
ron_fault_t ron_pid_set_antiwindup(ron_pid_instance_t *inst, ron_aw_mode_t mode, ron_float_t T_aw)
{
    ron_pid_config_t candidate;
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    candidate         = inst->config;
    candidate.aw_mode = mode;
    candidate.T_aw    = T_aw;
    return pid_apply_candidate(inst, &candidate);
}

/* Satisfies: RON-FR-040 – RON-FR-042 | Test: RON-TC-PID-027 – RON-TC-PID-029 */
ron_fault_t ron_pid_set_mode(ron_pid_instance_t *inst, ron_op_mode_t mode, ron_float_t manual_out)
{
    ron_fault_t fault;
    ron_float_t clamped_output;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!pid_api_isfinite(manual_out)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    clamped_output = ron_clamp(manual_out, inst->config.u_min, inst->config.u_max);
    if ((inst->state.mode == RON_MODE_MANUAL) && (mode == RON_MODE_AUTOMATIC)) {
        inst->state.integral   = ron_clamp(clamped_output, inst->config.I_min, inst->config.I_max);
        inst->state.u_sat_prev = clamped_output;
        inst->state.u_prev     = clamped_output;
    } else if (mode == RON_MODE_MANUAL) {
        inst->state.u_sat_prev = clamped_output;
        inst->state.u_prev     = clamped_output;
    }

    inst->state.mode   = mode;
    inst->state.status = (mode == RON_MODE_MANUAL) ? RON_STATUS_MANUAL_MODE : RON_STATUS_OK;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-052 | Test: RON-TC-PID-032 */
ron_fault_t ron_pid_set_integral(ron_pid_instance_t *inst, ron_float_t value)
{
    ron_fault_t fault;

    fault = pid_check_inst(inst);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!pid_api_isfinite(value)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    inst->state.integral = ron_clamp(value, inst->config.I_min, inst->config.I_max);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-071 | Test: RON-TC-PID-039 */
ron_fault_t ron_pid_get_state(const ron_pid_instance_t *inst, ron_float_t *integral,
                              ron_float_t *last_u, ron_float_t *last_D, ron_status_t *status,
                              ron_fault_t *fault)
{
    if (inst == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    if (integral != NULL) {
        *integral = inst->state.integral;
    }
    if (last_u != NULL) {
        *last_u = inst->state.u_sat_prev;
    }
    if (last_D != NULL) {
        *last_D = inst->state.D_filt_prev;
    }
    if (status != NULL) {
        *status = inst->state.status;
    }
    if (fault != NULL) {
        *fault = inst->state.fault_code;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-SR-012 | Test: RON-TC-SAFE-009 */
ron_fault_t ron_pid_fault_clear(ron_pid_instance_t *inst)
{
    if (inst == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    inst->state.fault_code = RON_FAULT_NONE;
    inst->state.status = (ron_status_t) (inst->state.status & (ron_status_t) (~RON_STATUS_FAULT));
    return RON_FAULT_NONE;
}
