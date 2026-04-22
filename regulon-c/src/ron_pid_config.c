/*
 * @file     ron_pid_config.c
 * @brief    PID controller configuration validation.
 * @module   ron_pid_config
 * @doc      RON-IS-001
 * @req      RON-FR-010, RON-FR-021, RON-FR-033, RON-FR-054, RON-SR-001,
 *           RON-SR-002
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
static bool pid_cfg_nonnegative(ron_float_t value)
{
    return RON_ISFINITE(value) && (value >= RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-007 | Test: RON-TC-PID-010 */
static bool pid_cfg_unit_interval(ron_float_t value)
{
    return RON_ISFINITE(value)
        && (value >= RON_FLOAT_C(0.0))
        && (value <= RON_FLOAT_C(1.0));
}

/* Satisfies: RON-FR-021, RON-FR-035 | Test: RON-TC-PID-016, RON-TC-PID-026 */
static bool pid_cfg_strict_range(ron_float_t minimum,
                                 ron_float_t maximum)
{
    return RON_ISFINITE(minimum)
        && RON_ISFINITE(maximum)
        && (minimum < maximum);
}

/* Satisfies: RON-FR-001 – RON-FR-006 | Test: RON-TC-PID-001 – RON-TC-PID-009 */
static bool pid_cfg_valid_gains(const ron_pid_config_t *cfg)
{
    return pid_cfg_nonnegative(cfg->Kp)
        && pid_cfg_nonnegative(cfg->Ki)
        && pid_cfg_nonnegative(cfg->Kd)
        && pid_cfg_nonnegative(cfg->N);
}

/* Satisfies: RON-FR-007 | Test: RON-TC-PID-010 */
static bool pid_cfg_valid_weights(const ron_pid_config_t *cfg)
{
    return pid_cfg_unit_interval(cfg->b)
        && pid_cfg_unit_interval(cfg->c);
}

/* Satisfies: RON-FR-021, RON-FR-035 | Test: RON-TC-PID-016, RON-TC-PID-026 */
static bool pid_cfg_valid_limits(const ron_pid_config_t *cfg)
{
    return pid_cfg_strict_range(cfg->u_min, cfg->u_max)
        && pid_cfg_strict_range(cfg->I_min, cfg->I_max);
}

/* Satisfies: RON-FR-033, RON-SR-011 | Test: RON-TC-PID-024, RON-TC-SAFE-008 */
static bool pid_cfg_valid_enums(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = ((cfg->aw_mode == RON_AW_NONE)
          || (cfg->aw_mode == RON_AW_BACK_CALC)
          || (cfg->aw_mode == RON_AW_CLAMPING));
    valid = valid
         && ((cfg->integ_method == RON_INTEG_EULER)
          || (cfg->integ_method == RON_INTEG_TRAPEZOIDAL));
    valid = valid
         && ((cfg->deriv_mode == RON_DERIV_ON_ERROR)
          || (cfg->deriv_mode == RON_DERIV_ON_MEASUREMENT));
    valid = valid
         && ((cfg->safe_policy == RON_SAFE_HOLD_LAST)
          || (cfg->safe_policy == RON_SAFE_ZERO)
          || (cfg->safe_policy == RON_SAFE_CONSTANT));

    return valid;
}

/* Satisfies: RON-FR-006, RON-FR-021, RON-SR-011 | Test: RON-TC-PID-016, RON-TC-PID-024, RON-TC-SAFE-008 */
static bool pid_cfg_valid_runtime_scalars(const ron_pid_config_t *cfg)
{
    return RON_ISFINITE(cfg->tau_sp)
        && RON_ISFINITE(cfg->du_max)
        && RON_ISFINITE(cfg->safe_value);
}

/* Satisfies: RON-FR-033 | Test: RON-TC-PID-022, RON-TC-PID-024 */
static bool pid_cfg_valid_aw_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->aw_mode == RON_AW_BACK_CALC)
        && (!pid_cfg_nonnegative(cfg->T_aw) || (cfg->T_aw <= RON_FLOAT_C(0.0))))
    {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-SR-013 | Test: RON-TC-SAFE-010 */
static bool pid_cfg_valid_overflow_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->I_overflow_thresh != RON_FLOAT_C(0.0))
        && (!pid_cfg_nonnegative(cfg->I_overflow_thresh)
         || (cfg->I_overflow_thresh <= RON_FLOAT_C(0.0))))
    {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-FR-054 | Test: RON-TC-PID-034 */
static bool pid_cfg_valid_sp_reset_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->sp_reset_threshold != RON_FLOAT_C(0.0))
        && (!pid_cfg_nonnegative(cfg->sp_reset_threshold)
         || (cfg->sp_reset_threshold <= RON_FLOAT_C(0.0))))
    {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-FR-010, RON-FR-012 | Test: RON-TC-PID-011, RON-TC-PID-013 */
static bool pid_cfg_valid_normalisation(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if (cfg->normalise)
    {
        valid = RON_ISFINITE(cfg->in_min) && RON_ISFINITE(cfg->in_max);
        valid = valid && ((cfg->in_max - cfg->in_min) > RON_FLOAT_EPSILON);
        valid = valid && RON_ISFINITE(cfg->out_min) && RON_ISFINITE(cfg->out_max);
        valid = valid && ((cfg->out_max - cfg->out_min) > RON_FLOAT_EPSILON);
    }

    return valid;
}

/* Satisfies: RON-FR-033, RON-FR-054 | Test: RON-TC-PID-024, RON-TC-PID-034 */
static bool pid_cfg_valid_thresholds(const ron_pid_config_t *cfg)
{
    return pid_cfg_valid_runtime_scalars(cfg)
        && pid_cfg_valid_aw_threshold(cfg)
        && pid_cfg_valid_overflow_threshold(cfg)
        && pid_cfg_valid_sp_reset_threshold(cfg);
}

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
ron_fault_t ron_pid_config_validate(const ron_pid_config_t *cfg)
{
    ron_fault_t fault;

    fault = RON_FAULT_NONE;
    if (cfg == NULL)
    {
        fault = RON_FAULT_NULL_POINTER;
    }
    else if (!pid_cfg_valid_gains(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }
    else if (!pid_cfg_valid_weights(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }
    else if (!pid_cfg_valid_limits(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }
    else if (!pid_cfg_valid_enums(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }
    else if (!pid_cfg_valid_normalisation(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }
    else if (!pid_cfg_valid_thresholds(cfg))
    {
        fault = RON_FAULT_CONFIG_INVALID;
    }

    return fault;
}
