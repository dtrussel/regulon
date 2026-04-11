/*
 * @file     ron_pid_config.c
 * @brief    PID controller configuration validation.
 * @module   ron_pid_config
 * @doc      RON-IS-001
 * @req      RON-SR-001, RON-SR-002, RON-FR-050
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * Sprint status: Full validation implemented (Sprint 1/2 baseline).
 */

#include "ron/ron_pid.h"

/* =========================================================================
 * ron_pid_config_validate — validates a configuration record.
 *
 * Error pattern: null-check → field validation.
 * Returns RON_FAULT_NONE when all fields are valid.
 * Returns RON_FAULT_CONFIG_INVALID on the first failing check.
 * ========================================================================= */

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
ron_fault_t ron_pid_config_validate(const ron_pid_config_t *cfg)
{
    /* null-check (error pattern step 1) */
    if (cfg == NULL)
    {
        return RON_FAULT_NULL_POINTER;
    }

    /* ── Gains must be non-negative and finite ───────────────────────── */
    if (!RON_ISFINITE(cfg->Kp) || cfg->Kp < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(cfg->Ki) || cfg->Ki < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(cfg->Kd) || cfg->Kd < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Derivative filter bandwidth multiplier ──────────────────────── */
    if (!RON_ISFINITE(cfg->N) || cfg->N < RON_FLOAT_C(0.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── 2DOF setpoint weights in [0, 1] ─────────────────────────────── */
    if (!RON_ISFINITE(cfg->b) ||
        cfg->b < RON_FLOAT_C(0.0) || cfg->b > RON_FLOAT_C(1.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!RON_ISFINITE(cfg->c) ||
        cfg->c < RON_FLOAT_C(0.0) || cfg->c > RON_FLOAT_C(1.0))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Output limits: u_min < u_max, both finite ───────────────────── */
    if (!RON_ISFINITE(cfg->u_min) || !RON_ISFINITE(cfg->u_max))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (cfg->u_min >= cfg->u_max)
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Integral clamp: I_min < I_max, both finite ──────────────────── */
    if (!RON_ISFINITE(cfg->I_min) || !RON_ISFINITE(cfg->I_max))
    {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (cfg->I_min >= cfg->I_max)
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Back-calculation AW: T_aw must be > 0 when mode=BACK_CALC ───── */
    if (cfg->aw_mode == RON_AW_BACK_CALC)
    {
        if (!RON_ISFINITE(cfg->T_aw) || cfg->T_aw <= RON_FLOAT_C(0.0))
        {
            return RON_FAULT_CONFIG_INVALID;
        }
    }

    /* ── Setpoint pre-filter: tau_sp must be finite ───────────────────── */
    if (!RON_ISFINITE(cfg->tau_sp))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Normalisation ranges (only validated when normalise=true) ────── */
    if (cfg->normalise)
    {
        if (!RON_ISFINITE(cfg->in_min) || !RON_ISFINITE(cfg->in_max))
        {
            return RON_FAULT_CONFIG_INVALID;
        }
        if (cfg->in_min >= cfg->in_max)
        {
            return RON_FAULT_CONFIG_INVALID;
        }
        if (!RON_ISFINITE(cfg->out_min) || !RON_ISFINITE(cfg->out_max))
        {
            return RON_FAULT_CONFIG_INVALID;
        }
        if (cfg->out_min >= cfg->out_max)
        {
            return RON_FAULT_CONFIG_INVALID;
        }
    }

    /* ── Safe value must be finite (used by RON_SAFE_CONSTANT policy) ─── */
    if (!RON_ISFINITE(cfg->safe_value))
    {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* ── Overflow threshold: 0 means disabled; if set, must be positive ─ */
    if (cfg->I_overflow_thresh != RON_FLOAT_C(0.0))
    {
        if (!RON_ISFINITE(cfg->I_overflow_thresh) ||
            cfg->I_overflow_thresh <= RON_FLOAT_C(0.0))
        {
            return RON_FAULT_CONFIG_INVALID;
        }
    }

    /* ── SP reset threshold: 0 means disabled; if set, must be positive ─ */
    if (cfg->sp_reset_threshold != RON_FLOAT_C(0.0))
    {
        if (!RON_ISFINITE(cfg->sp_reset_threshold) ||
            cfg->sp_reset_threshold <= RON_FLOAT_C(0.0))
        {
            return RON_FAULT_CONFIG_INVALID;
        }
    }

    return RON_FAULT_NONE;
}
