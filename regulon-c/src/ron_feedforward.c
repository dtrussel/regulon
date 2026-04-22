/*
 * @file     ron_feedforward.c
 * @brief    PID feed-forward extension API.
 * @module   ron_feedforward
 * @doc      RON-IS-001
 * @req      RON-FR-200, RON-FR-201, RON-FR-202, RON-FR-203,
 *           RON-FR-204, RON-FR-205
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_feedforward.h"

#include "ron_pid_internal.h"

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static bool feedforward_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* Satisfies: RON-FR-201 | Test: RON-TC-FF-002 - RON-TC-FF-005 */
static bool feedforward_mode_valid(ron_feedforward_mode_t mode)
{
    return (mode == RON_FF_DISABLED) || (mode == RON_FF_STATIC_GAIN) || (mode == RON_FF_VELOCITY) ||
           (mode == RON_FF_ACCELERATION) || (mode == RON_FF_EXTERNAL);
}

/* Satisfies: RON-FR-202, RON-FR-204 | Test: RON-TC-FF-006, RON-TC-FF-008 */
static void feedforward_reset_state(ron_pid_state_t *state)
{
    state->ff_r_prev = RON_FLOAT_C(0.0);
    state->ff_v_prev = RON_FLOAT_C(0.0);
    state->ff_a_prev = RON_FLOAT_C(0.0);
    state->u_ff_prev = RON_FLOAT_C(0.0);
}

/* Satisfies: RON-SR-006 | Test: RON-TC-FF-009 */
static ron_fault_t feedforward_check_inst(const ron_pid_instance_t *inst)
{
    ron_fault_t fault;

    fault = RON_FAULT_NONE;
    if (inst == NULL) {
        fault = RON_FAULT_NULL_POINTER;
    } else if (!inst->state.is_initialised) {
        fault = RON_FAULT_CONFIG_INVALID;
    }

    return fault;
}

/* Satisfies: RON-FR-201, RON-FR-202 | Test: RON-TC-FF-002 - RON-TC-FF-006 */
ron_fault_t ron_feedforward_config_validate(const ron_feedforward_config_t *cfg)
{
    ron_fault_t fault;

    fault = RON_FAULT_NONE;
    if (cfg == NULL) {
        fault = RON_FAULT_NULL_POINTER;
    } else if (!feedforward_mode_valid(cfg->mode)) {
        fault = RON_FAULT_CONFIG_INVALID;
    } else if (!feedforward_isfinite(cfg->gain)) {
        fault = RON_FAULT_CONFIG_INVALID;
    } else if (!feedforward_isfinite(cfg->N_ff) || (cfg->N_ff < RON_FLOAT_C(0.0))) {
        fault = RON_FAULT_CONFIG_INVALID;
    }

    return fault;
}

/* Satisfies: RON-FR-201, RON-FR-202, RON-FR-204 | Test: RON-TC-FF-002 - RON-TC-FF-008 */
ron_fault_t ron_pid_set_feedforward(ron_pid_instance_t *inst, const ron_feedforward_config_t *cfg)
{
    ron_fault_t fault;

    fault = feedforward_check_inst(inst);
    if (fault == RON_FAULT_NONE) {
        fault = ron_feedforward_config_validate(cfg);
    }
    if (fault == RON_FAULT_NONE) {
        inst->config.feedforward = *cfg;
        feedforward_reset_state(&inst->state);
        inst->state.status =
            (ron_status_t) (inst->state.status & (ron_status_t) (~RON_STATUS_FF_ACTIVE));
    }

    return fault;
}

/* Satisfies: RON-FR-200 - RON-FR-205 | Test: RON-TC-FF-001 - RON-TC-FF-009 */
ron_fault_t ron_pid_step_feedforward(ron_pid_instance_t *inst, ron_float_t r, ron_float_t y,
                                     ron_float_t dt, ron_float_t external_ff, ron_float_t *u_out,
                                     ron_status_t *status)
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
    if (inst->config.feedforward.mode != RON_FF_EXTERNAL) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !feedforward_isfinite(dt) ||
        !feedforward_isfinite(external_ff)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    fault = ron_pid_core_step(inst, r, y, dt, external_ff, u_out, status);
    return fault;
}

/* Satisfies: RON-FR-205 | Test: RON-TC-FF-001, RON-TC-FF-009 */
ron_fault_t ron_pid_get_feedforward(const ron_pid_instance_t *inst, ron_float_t *u_ff)
{
    ron_fault_t fault;

    if ((inst == NULL) || (u_ff == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = feedforward_check_inst(inst);
    if (fault == RON_FAULT_NONE) {
        *u_ff = inst->state.u_ff_prev;
    }

    return fault;
}
