/*
 * @file     ron_pid_api.c
 * @brief    Public API dispatcher for the Regulon PID controller module.
 * @module   ron_pid_api
 * @doc      RON-IS-001
 * @req      RON-FR-050, RON-FR-051, RON-FR-052, RON-FR-053, RON-FR-070,
 *           RON-FR-071, RON-SR-001, RON-SR-002, RON-SR-006, RON-SR-012
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * Sprint status: NULL-pointer guards implemented (Sprint 1).
 *               Full computation delegated to ron_pid_core.c (Sprint 2/3).
 */

#include "ron/ron_pid.h"
#include "ron_pid_internal.h"

/* =========================================================================
 * Internal helper: null-pointer guard + init-check
 *
 * Error pattern per RON-IS-001 §3.4:
 *   null-check → init-check → fault-latch → input validation → computation
 * ========================================================================= */

/* Satisfies: RON-SR-006 | Test: RON-TC-SAFE-006 */
static ron_fault_t api_check_inst(const ron_pid_instance_t *inst)
{
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (!inst->state.is_initialised)
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/* Satisfies: RON-FR-050, RON-SR-001, RON-SR-002 | Test: RON-TC-PID-030 */
ron_fault_t ron_pid_init(ron_pid_instance_t     *inst,
                         const ron_pid_config_t *cfg)
{
    /* null-check (error pattern step 1) */
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (cfg == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }

    /* Configuration validation (delegated to ron_pid_config.c, Sprint 2). */
    ron_fault_t fault = ron_pid_config_validate(cfg);
    if (fault != RON_FAULT_NONE)
    {
        return fault;
    }

    /* Copy configuration and zero-initialise state. */
    inst->config = *cfg;

    /* Zero all state fields explicitly (no memset — avoids <string.h>). */
    inst->state.integral       = RON_FLOAT_C(0.0);
    inst->state.y_prev         = RON_FLOAT_C(0.0);
    inst->state.e_D_prev       = RON_FLOAT_C(0.0);
    inst->state.D_filt_prev    = RON_FLOAT_C(0.0);
    inst->state.r_filt_prev    = RON_FLOAT_C(0.0);
    inst->state.u_sat_prev     = RON_FLOAT_C(0.0);
    inst->state.u_prev         = RON_FLOAT_C(0.0);
    inst->state.e_prev         = RON_FLOAT_C(0.0);
    inst->state.mode           = RON_MODE_AUTOMATIC;
    inst->state.fault_code     = RON_FAULT_NONE;
    inst->state.status         = RON_STATUS_OK;
    inst->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-051 | Test: RON-TC-PID-031 */
ron_fault_t ron_pid_reset(ron_pid_instance_t *inst)
{
    ron_fault_t check = api_check_inst(inst);
    if (check != RON_FAULT_NONE)
    {
        return check;
    }

    /* Reset dynamic state, preserve config and mode. */
    inst->state.integral    = RON_FLOAT_C(0.0);
    inst->state.y_prev      = RON_FLOAT_C(0.0);
    inst->state.e_D_prev    = RON_FLOAT_C(0.0);
    inst->state.D_filt_prev = RON_FLOAT_C(0.0);
    inst->state.r_filt_prev = RON_FLOAT_C(0.0);
    inst->state.u_sat_prev  = RON_FLOAT_C(0.0);
    inst->state.u_prev      = RON_FLOAT_C(0.0);
    inst->state.e_prev      = RON_FLOAT_C(0.0);
    inst->state.fault_code  = RON_FAULT_NONE;
    inst->state.status      = RON_STATUS_OK;

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Runtime
 * ========================================================================= */

/* Satisfies: RON-FR-001 – RON-FR-035 | Test: RON-TC-PID-001 – RON-TC-PID-026 */
ron_fault_t ron_pid_step(ron_pid_instance_t *inst,
                         ron_float_t         r,
                         ron_float_t         y,
                         ron_float_t         dt,
                         ron_float_t        *u_out,
                         ron_status_t       *status)
{
    /* null-check (error pattern step 1) */
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (u_out == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (status == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }

    /* init-check (error pattern step 2) */
    if (!inst->state.is_initialised)
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* fault-latch check (error pattern step 3) —
     * if faulted, apply safe-state and return immediately. */
    if (inst->state.fault_code != RON_FAULT_NONE)
    {
        *status = (ron_status_t)(inst->state.status |
                                 (ron_status_t)RON_STATUS_FAULT);
        *u_out  = ron_pid_fault_safe_output(inst);
        return inst->state.fault_code;
    }

    /* dt sanity check (error pattern step 4) —
     * preconditions at ron_pid.h:131 require dt > 0 and finite. */
    if (!RON_ISFINITE(dt) || (dt <= RON_FLOAT_C(0.0)))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* Delegate to the core computation pipeline (Sprint 2). */
    return ron_pid_core_step(inst, r, y, dt, u_out, status);
}

/* =========================================================================
 * Configuration update (runtime)
 * ========================================================================= */

/* Satisfies: RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_gains(ron_pid_instance_t *inst,
                               ron_float_t         Kp,
                               ron_float_t         Ki,
                               ron_float_t         Kd)
{
    ron_fault_t check = api_check_inst(inst);
    if (check != RON_FAULT_NONE)
    {
        return check;
    }
    if (!RON_ISFINITE(Kp) || Kp < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(Ki) || Ki < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(Kd) || Kd < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    inst->config.Kp = Kp;
    inst->config.Ki = Ki;
    inst->config.Kd = Kd;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-021, RON-FR-053 | Test: RON-TC-PID-016 */
ron_fault_t ron_pid_set_limits(ron_pid_instance_t *inst,
                                ron_float_t         u_min,
                                ron_float_t         u_max)
{
    ron_fault_t check = api_check_inst(inst);
    if (check != RON_FAULT_NONE)
    {
        return check;
    }
    if (!RON_ISFINITE(u_min) || !RON_ISFINITE(u_max) || u_min >= u_max)
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    inst->config.u_min = u_min;
    inst->config.u_max = u_max;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-006, RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_filter(ron_pid_instance_t *inst,
                                ron_float_t         N)
{
    ron_fault_t check = api_check_inst(inst);
    if (check != RON_FAULT_NONE)
    {
        return check;
    }
    if (!RON_ISFINITE(N) || N < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    inst->config.N = N;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-033, RON-FR-053 | Test: RON-TC-PID-024 */
ron_fault_t ron_pid_set_antiwindup(ron_pid_instance_t *inst,
                                    ron_aw_mode_t       mode,
                                    ron_float_t         T_aw)
{
    ron_fault_t check = api_check_inst(inst);
    if (check != RON_FAULT_NONE)
    {
        return check;
    }
    if ((mode == RON_AW_BACK_CALC) &&
        (!RON_ISFINITE(T_aw) || T_aw <= RON_FLOAT_C(0.0)))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    inst->config.aw_mode = mode;
    inst->config.T_aw    = T_aw;
    return RON_FAULT_NONE;
}

/* =========================================================================
 * State mutation
 * ========================================================================= */

/* Satisfies: RON-FR-040 – RON-FR-042 | Test: RON-TC-PID-027 – RON-TC-PID-029 */
ron_fault_t ron_pid_set_mode(ron_pid_instance_t *inst,
                              ron_op_mode_t       mode,
                              ron_float_t         manual_out)
{
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (!inst->state.is_initialised)
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(manual_out))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    ron_float_t clamped = ron_clamp(manual_out,
                                    inst->config.u_min,
                                    inst->config.u_max);
    inst->state.mode = mode;

    if (mode == RON_MODE_MANUAL)
    {
        inst->state.u_sat_prev = clamped;
        /* Status: set MANUAL_MODE flag. */
        inst->state.status = (ron_status_t)(inst->state.status |
                                            (ron_status_t)RON_STATUS_MANUAL_MODE);
    }
    else
    {
        /* Bumpless transfer: pre-load integral so first auto output ≈ manual_out.
         * Full bumpless logic is implemented in Sprint 3 (ron_pid_core.c). */
        inst->state.status = (ron_status_t)(inst->state.status &
                                            ~((ron_status_t)RON_STATUS_MANUAL_MODE));
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-052 | Test: RON-TC-PID-032 */
ron_fault_t ron_pid_set_integral(ron_pid_instance_t *inst,
                                  ron_float_t         value)
{
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    if (!inst->state.is_initialised)
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(value))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    inst->state.integral = ron_clamp(value,
                                     inst->config.I_min,
                                     inst->config.I_max);
    return RON_FAULT_NONE;
}

/* =========================================================================
 * State inspection
 * ========================================================================= */

/* Satisfies: RON-FR-071 | Test: RON-TC-PID-039 */
ron_fault_t ron_pid_get_state(const ron_pid_instance_t *inst,
                               ron_float_t              *integral,
                               ron_float_t              *last_u,
                               ron_float_t              *last_D,
                               ron_status_t             *status,
                               ron_fault_t              *fault)
{
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    /* Any NULL output pointer is silently skipped per RON-IS-001 §8. */
    if (integral != NULL) { *integral = inst->state.integral;    }
    if (last_u   != NULL) { *last_u   = inst->state.u_sat_prev;  }
    if (last_D   != NULL) { *last_D   = inst->state.D_filt_prev; }
    if (status   != NULL) { *status   = inst->state.status;      }
    if (fault    != NULL) { *fault    = inst->state.fault_code;  }
    return RON_FAULT_NONE;
}

/* =========================================================================
 * Fault management
 * ========================================================================= */

/* Satisfies: RON-SR-012 | Test: RON-TC-SAFE-009 */
ron_fault_t ron_pid_fault_clear(ron_pid_instance_t *inst)
{
    if (inst == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }
    inst->state.fault_code = RON_FAULT_NONE;
    inst->state.status = (ron_status_t)(inst->state.status &
                                        ~((ron_status_t)RON_STATUS_FAULT));
    return RON_FAULT_NONE;
}
