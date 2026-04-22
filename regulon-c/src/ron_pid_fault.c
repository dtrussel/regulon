/*
 * @file     ron_pid_fault.c
 * @brief    Fault management helpers for the Regulon PID controller module.
 * @module   ron_pid_fault
 * @doc      RON-IS-001
 * @req      RON-SR-010, RON-SR-011, RON-SR-012, RON-SR-013
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_platform.h"
#include "ron_pid_internal.h"

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
static void pid_fault_notify(const ron_pid_instance_t *inst,
                             ron_fault_t               new_bits)
{
    ron_fault_t remaining = new_bits;

    if ((inst->config.fault_cb == NULL) || (remaining == RON_FAULT_NONE))
    {
        return;
    }

    if ((remaining & RON_FAULT_INPUT_NAN) != RON_FAULT_NONE)
    {
        inst->config.fault_cb(RON_FAULT_INPUT_NAN);
        remaining = (ron_fault_t)(remaining & (ron_fault_t)(~RON_FAULT_INPUT_NAN));
    }
    if ((remaining & RON_FAULT_OUTPUT_NAN) != RON_FAULT_NONE)
    {
        inst->config.fault_cb(RON_FAULT_OUTPUT_NAN);
        remaining = (ron_fault_t)(remaining & (ron_fault_t)(~RON_FAULT_OUTPUT_NAN));
    }
    if ((remaining & RON_FAULT_INTEGRAL_OVERFLOW) != RON_FAULT_NONE)
    {
        inst->config.fault_cb(RON_FAULT_INTEGRAL_OVERFLOW);
        remaining = (ron_fault_t)(remaining & (ron_fault_t)(~RON_FAULT_INTEGRAL_OVERFLOW));
    }
    if ((remaining & RON_FAULT_CONFIG_INVALID) != RON_FAULT_NONE)
    {
        inst->config.fault_cb(RON_FAULT_CONFIG_INVALID);
        remaining = (ron_fault_t)(remaining & (ron_fault_t)(~RON_FAULT_CONFIG_INVALID));
    }
    if ((remaining & RON_FAULT_NULL_POINTER) != RON_FAULT_NONE)
    {
        inst->config.fault_cb(RON_FAULT_NULL_POINTER);
    }
}

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
void ron_pid_fault_set(ron_pid_instance_t *inst,
                       ron_fault_t         code)
{
    ron_fault_t new_bits;

    new_bits = (ron_fault_t)(code & (ron_fault_t)(~inst->state.fault_code));
    inst->state.fault_code = (ron_fault_t)(inst->state.fault_code | code);
    inst->state.status = (ron_status_t)(inst->state.status | RON_STATUS_FAULT);

    pid_fault_notify(inst, new_bits);
}

/* Satisfies: RON-SR-011 | Test: RON-TC-SAFE-008 */
ron_float_t ron_pid_fault_safe_output(const ron_pid_instance_t *inst)
{
    ron_float_t output;

    switch (inst->config.safe_policy)
    {
        case RON_SAFE_ZERO:
            output = RON_FLOAT_C(0.0);
            break;
        case RON_SAFE_CONSTANT:
            output = inst->config.safe_value;
            break;
        case RON_SAFE_HOLD_LAST:
        default:
            output = inst->state.u_sat_prev;
            break;
    }

    return ron_clamp(output, inst->config.u_min, inst->config.u_max);
}
