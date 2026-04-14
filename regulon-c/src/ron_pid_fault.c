/*
 * @file     ron_pid_fault.c
 * @brief    PID fault manager — bit-latch, safe-state, callback.
 * @module   ron_pid_fault
 * @doc      RON-IS-001
 * @req      RON-SR-010, RON-SR-011, RON-SR-012, RON-SR-013
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * Implements the two internal helpers declared in ron_pid_internal.h:
 *   - ron_pid_fault_set        : latch fault bit(s), set STATUS_FAULT,
 *                                invoke callback once per newly-set bit.
 *   - ron_pid_fault_safe_output: resolve the configured safe-state policy.
 *
 * Sprint status: Sprint 2 — full implementation.
 */

#include "ron/ron_pid.h"
#include "ron_pid_internal.h"

/* =========================================================================
 * ron_pid_fault_set
 *
 * ORs new bits into inst->state.fault_code, sets STATUS_FAULT, and invokes
 * the optional fault callback once for each bit that was newly set (bits
 * already present in the register do NOT re-trigger the callback).
 *
 * The callback is dispatched from within ron_pid_step(), and therefore
 * must be ISR-safe and MUST NOT re-enter any Regulon API function
 * (ron_pid_types.h, docstring for ron_fault_cb_t).
 * ========================================================================= */

/* Satisfies: RON-SR-010, RON-SR-013 | Test: RON-TC-SAFE-007 */
void ron_pid_fault_set(ron_pid_instance_t *inst, ron_fault_t code)
{
    if (inst == NULL)
    {
        return;
    }
    /* Bits that were NOT already latched — these are the newly-set bits
     * that warrant a callback invocation.                                */
    ron_fault_t before   = inst->state.fault_code;
    ron_fault_t new_bits = (ron_fault_t)(code & (ron_fault_t)~before);

    /* Latch the bits and assert the STATUS_FAULT flag. */
    inst->state.fault_code = (ron_fault_t)(before | code);
    inst->state.status     = (ron_status_t)(inst->state.status |
                                            (ron_status_t)RON_STATUS_FAULT);

    /* Invoke the callback once per newly-set bit (bounded loop over the
     * 8 possible bits — MISRA-C:2023 Rule 14.2 compatible).              */
    if ((new_bits != (ron_fault_t)0U) && (inst->config.fault_cb != NULL))
    {
        for (uint8_t i = 0U; i < 8U; ++i)
        {
            ron_fault_t mask = (ron_fault_t)((ron_fault_t)1U << i);
            if ((new_bits & mask) != (ron_fault_t)0U)
            {
                inst->config.fault_cb(mask);
            }
        }
    }
}

/* =========================================================================
 * ron_pid_fault_safe_output
 *
 * Resolves the configured safe-state policy into a concrete output value.
 * RON_SAFE_CONSTANT additionally clamps safe_value into [u_min, u_max] so
 * that callers need not worry about an out-of-range configuration here.
 * ========================================================================= */

/* Satisfies: RON-SR-011 | Test: RON-TC-SAFE-008 */
ron_float_t ron_pid_fault_safe_output(const ron_pid_instance_t *inst)
{
    ron_float_t out = RON_FLOAT_C(0.0);
    if (inst == NULL)
    {
        return out;
    }

    switch (inst->config.safe_policy)
    {
        case RON_SAFE_ZERO:
            out = RON_FLOAT_C(0.0);
            break;

        case RON_SAFE_CONSTANT:
            out = ron_clamp(inst->config.safe_value,
                            inst->config.u_min,
                            inst->config.u_max);
            break;

        case RON_SAFE_HOLD_LAST:
        default:
            out = inst->state.u_sat_prev;
            break;
    }
    return out;
}
