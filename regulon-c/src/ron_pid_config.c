/*
 * @file     ron_pid_config.c
 * @brief    PID controller configuration validation.
 * @module   ron_pid_config
 * @doc      RON-IS-001
 * @req      RON-FR-002, RON-FR-010, RON-FR-021, RON-FR-033, RON-FR-054, RON-SR-001,
 *           RON-SR-002
 * @version  1.0.0
 * @author   dtrussel
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_feedforward.h"

#include "ron_util_internal.h"

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
static bool pid_cfg_nonnegative(ron_float_t value)
{
    return ron_util_isfinite(value) && (value >= RON_FLOAT_C(0.0));
}

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
static bool pid_cfg_positive(ron_float_t value)
{
    return ron_util_isfinite(value) && (value > RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-007 | Test: RON-TC-PID-010 */
static bool pid_cfg_unit_interval(ron_float_t value)
{
    return ron_util_isfinite(value) && (value >= RON_FLOAT_C(0.0)) && (value <= RON_FLOAT_C(1.0));
}

/* Satisfies: RON-FR-021, RON-FR-035 | Test: RON-TC-PID-016, RON-TC-PID-026 */
static bool pid_cfg_strict_range(ron_float_t minimum, ron_float_t maximum)
{
    return ron_util_isfinite(minimum) && ron_util_isfinite(maximum) && (minimum < maximum);
}

/* Satisfies: RON-FR-001, RON-FR-003 – RON-FR-006, RON-SR-001 | Test: RON-TC-PID-001, RON-TC-SAFE-001 */
static bool pid_cfg_valid_gains(const ron_pid_config_t *cfg)
{
    return pid_cfg_nonnegative(cfg->Kp) && pid_cfg_nonnegative(cfg->Ki) &&
           pid_cfg_nonnegative(cfg->Kd) && pid_cfg_nonnegative(cfg->N);
}

/* Satisfies: RON-FR-007 | Test: RON-TC-PID-010 */
static bool pid_cfg_valid_weights(const ron_pid_config_t *cfg)
{
    return pid_cfg_unit_interval(cfg->b) && pid_cfg_unit_interval(cfg->c);
}

/* Satisfies: RON-FR-021, RON-FR-035 | Test: RON-TC-PID-016, RON-TC-PID-026 */
static bool pid_cfg_valid_limits(const ron_pid_config_t *cfg)
{
    return pid_cfg_strict_range(cfg->u_min, cfg->u_max) &&
           pid_cfg_strict_range(cfg->I_min, cfg->I_max);
}

/* Satisfies: RON-FR-033, RON-SR-011 | Test: RON-TC-PID-024, RON-TC-SAFE-008 */
static bool pid_cfg_valid_enums(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = ((cfg->aw_mode == RON_AW_NONE) || (cfg->aw_mode == RON_AW_BACK_CALC) ||
             (cfg->aw_mode == RON_AW_CLAMPING));
    valid = valid && ((cfg->integ_method == RON_INTEG_EULER) ||
                      (cfg->integ_method == RON_INTEG_TRAPEZOIDAL));
    valid = valid && ((cfg->deriv_mode == RON_DERIV_ON_ERROR) ||
                      (cfg->deriv_mode == RON_DERIV_ON_MEASUREMENT));
    valid =
        valid && ((cfg->safe_policy == RON_SAFE_HOLD_LAST) || (cfg->safe_policy == RON_SAFE_ZERO) ||
                  (cfg->safe_policy == RON_SAFE_CONSTANT));

    return valid;
}

/* Satisfies: RON-FR-006, RON-FR-021, RON-SR-011 | Test: RON-TC-PID-016, RON-TC-PID-024, RON-TC-SAFE-008 */
static bool pid_cfg_valid_runtime_scalars(const ron_pid_config_t *cfg)
{
    return ron_util_isfinite(cfg->tau_sp) && ron_util_isfinite(cfg->du_max) &&
           ron_util_isfinite(cfg->safe_value);
}

/* Satisfies: RON-FR-033 | Test: RON-TC-PID-022, RON-TC-PID-024 */
static bool pid_cfg_valid_aw_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->aw_mode == RON_AW_BACK_CALC) && !pid_cfg_positive(cfg->T_aw)) {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-SR-013 | Test: RON-TC-SAFE-010 */
static bool pid_cfg_valid_overflow_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->I_overflow_thresh != RON_FLOAT_C(0.0)) && !pid_cfg_positive(cfg->I_overflow_thresh)) {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-FR-054 | Test: RON-TC-PID-034 */
static bool pid_cfg_valid_sp_reset_threshold(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if ((cfg->sp_reset_threshold != RON_FLOAT_C(0.0)) &&
        !pid_cfg_positive(cfg->sp_reset_threshold)) {
        valid = false;
    }

    return valid;
}

/* Satisfies: RON-FR-010, RON-FR-012 | Test: RON-TC-PID-011, RON-TC-PID-013 */
static bool pid_cfg_valid_normalisation(const ron_pid_config_t *cfg)
{
    bool valid;

    valid = true;
    if (cfg->normalise) {
        valid = ron_util_isfinite(cfg->in_min) && ron_util_isfinite(cfg->in_max);
        valid = valid && ((cfg->in_max - cfg->in_min) > RON_FLOAT_EPSILON);
        valid = valid && ron_util_isfinite(cfg->out_min) && ron_util_isfinite(cfg->out_max);
        valid = valid && ((cfg->out_max - cfg->out_min) > RON_FLOAT_EPSILON);
    }

    return valid;
}

/* Satisfies: RON-FR-033, RON-FR-054 | Test: RON-TC-PID-024, RON-TC-PID-034 */
static bool pid_cfg_valid_thresholds(const ron_pid_config_t *cfg)
{
    return pid_cfg_valid_runtime_scalars(cfg) && pid_cfg_valid_aw_threshold(cfg) &&
           pid_cfg_valid_overflow_threshold(cfg) && pid_cfg_valid_sp_reset_threshold(cfg);
}

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
ron_fault_t ron_pid_config_validate(const ron_pid_config_t *cfg)
{
    if (cfg == NULL) {
        return RON_FAULT_NULL_POINTER;
    } else if (!pid_cfg_valid_gains(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    } else if (!pid_cfg_valid_weights(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    } else if (!pid_cfg_valid_limits(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    } else if (!pid_cfg_valid_enums(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    } else if (!pid_cfg_valid_normalisation(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    } else if (!pid_cfg_valid_thresholds(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_feedforward_config_validate(&cfg->feedforward);
}

/* Integral time +Inf means no integral action (Ki = 0); otherwise Ti must be
 * finite and positive. Both results are checked, since Kp / Ti overflows for
 * a large Kp and a tiny Ti. The record is written only when all is valid. */
/* Satisfies: RON-FR-002 | Test: RON-TC-PID-002 */
ron_fault_t ron_pid_config_from_isa(ron_pid_config_t *cfg, ron_float_t Kp, ron_float_t Ti,
                                    ron_float_t Td)
{
    ron_float_t Ki = RON_FLOAT_C(0.0);
    ron_float_t Kd;
    bool no_integral;

    if (cfg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    no_integral = (Ti > RON_FLOAT_MAX);
    if (!pid_cfg_nonnegative(Kp) || !pid_cfg_nonnegative(Td) ||
        (!no_integral && !pid_cfg_positive(Ti))) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!no_integral) {
        Ki = Kp / Ti;
    }
    Kd = Kp * Td;
    if (!ron_util_isfinite(Ki) || !ron_util_isfinite(Kd)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    cfg->Kp = Kp;
    cfg->Ki = Ki;
    cfg->Kd = Kd;
    return RON_FAULT_NONE;
}
