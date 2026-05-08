/*
 * @file     ron_gain_sched.c
 * @brief    Gain-scheduling support for bounded PID breakpoint tables.
 * @module   ron_gain_sched
 * @doc      RON-IS-001
 * @req      RON-FR-300, RON-FR-301, RON-FR-302, RON-FR-303,
 *           RON-FR-304, RON-FR-305, RON-FR-306
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_gain_sched.h"

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static bool gs_isfinite(ron_float_t value)
{
    return RON_ISFINITE(value);
}

/* Satisfies: RON-FR-302 | Test: RON-TC-GS-003, RON-TC-GS-004 */
static bool gs_mode_valid(ron_gs_mode_t mode)
{
    return (mode == RON_GS_HARD_SWITCH) || (mode == RON_GS_LINEAR_INTERP);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_gains_and_weights(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    return (lhs->Kp == rhs->Kp) && (lhs->Ki == rhs->Ki) && (lhs->Kd == rhs->Kd) &&
           (lhs->N == rhs->N) && (lhs->b == rhs->b) && (lhs->c == rhs->c);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_limits_and_integral(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    return (lhs->u_min == rhs->u_min) && (lhs->u_max == rhs->u_max) &&
           (lhs->du_max == rhs->du_max) && (lhs->I_min == rhs->I_min) &&
           (lhs->I_max == rhs->I_max) && (lhs->T_aw == rhs->T_aw) && (lhs->tau_sp == rhs->tau_sp);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_scaling_and_safety(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    return (lhs->in_min == rhs->in_min) && (lhs->in_max == rhs->in_max) &&
           (lhs->out_min == rhs->out_min) && (lhs->out_max == rhs->out_max) &&
           (lhs->safe_value == rhs->safe_value) &&
           (lhs->I_overflow_thresh == rhs->I_overflow_thresh) &&
           (lhs->sp_reset_threshold == rhs->sp_reset_threshold);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_modes_and_flags(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    return (lhs->aw_mode == rhs->aw_mode) && (lhs->integ_method == rhs->integ_method) &&
           (lhs->deriv_mode == rhs->deriv_mode) && (lhs->normalise == rhs->normalise) &&
           (lhs->safe_policy == rhs->safe_policy);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_feedforward_and_callback(const ron_pid_config_t *lhs,
                                             const ron_pid_config_t *rhs)
{
    return (lhs->feedforward.mode == rhs->feedforward.mode) &&
           (lhs->feedforward.gain == rhs->feedforward.gain) &&
           (lhs->feedforward.N_ff == rhs->feedforward.N_ff) && (lhs->fault_cb == rhs->fault_cb);
}

/* Satisfies: RON-FR-302, RON-FR-303 | Test: RON-TC-GS-004, RON-TC-GS-005 */
static bool gs_interp_compatible(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    ron_pid_config_t lhs_cmp = *lhs;
    ron_pid_config_t rhs_cmp = *rhs;

    lhs_cmp.Kp = RON_FLOAT_C(0.0);
    lhs_cmp.Ki = RON_FLOAT_C(0.0);
    lhs_cmp.Kd = RON_FLOAT_C(0.0);
    rhs_cmp.Kp = RON_FLOAT_C(0.0);
    rhs_cmp.Ki = RON_FLOAT_C(0.0);
    rhs_cmp.Kd = RON_FLOAT_C(0.0);

    return gs_same_gains_and_weights(&lhs_cmp, &rhs_cmp) &&
           gs_same_limits_and_integral(&lhs_cmp, &rhs_cmp) &&
           gs_same_scaling_and_safety(&lhs_cmp, &rhs_cmp) &&
           gs_same_modes_and_flags(&lhs_cmp, &rhs_cmp) &&
           gs_same_feedforward_and_callback(&lhs_cmp, &rhs_cmp);
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static bool gs_same_config(const ron_pid_config_t *lhs, const ron_pid_config_t *rhs)
{
    return gs_same_gains_and_weights(lhs, rhs) && gs_same_limits_and_integral(lhs, rhs) &&
           gs_same_scaling_and_safety(lhs, rhs) && gs_same_modes_and_flags(lhs, rhs) &&
           gs_same_feedforward_and_callback(lhs, rhs);
}

/* Satisfies: RON-FR-306 | Test: RON-TC-GS-008 */
static ron_fault_t gs_validate_first_entry(ron_float_t sigma, const ron_pid_config_t *cfg)
{
    if (!gs_isfinite(sigma)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (ron_pid_config_validate(cfg) != RON_FAULT_NONE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-306 | Test: RON-TC-GS-008 */
static ron_fault_t gs_validate_next_entry(ron_gs_mode_t mode, ron_float_t prev_sigma,
                                          const ron_pid_config_t *prev_cfg, ron_float_t sigma,
                                          const ron_pid_config_t *cfg)
{
    if (!gs_isfinite(sigma)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (ron_pid_config_validate(cfg) != RON_FAULT_NONE) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!(sigma > prev_sigma)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((mode == RON_GS_LINEAR_INTERP) && !gs_interp_compatible(prev_cfg, cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-306 | Test: RON-TC-GS-008 */
static ron_fault_t gs_validate_table(const ron_gs_table_t *tbl)
{
    uint8_t idx;

    if (tbl == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if ((tbl->n_points == 0U) || (tbl->n_points > (uint8_t) RON_GS_MAX_BREAKPOINTS)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!gs_mode_valid(tbl->mode)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    if (gs_validate_first_entry(tbl->sigma[0], &tbl->configs[0]) != RON_FAULT_NONE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    idx = 1U;
    while (idx < tbl->n_points) {
        if (gs_validate_next_entry(tbl->mode, tbl->sigma[idx - 1U], &tbl->configs[idx - 1U],
                                   tbl->sigma[idx], &tbl->configs[idx]) != RON_FAULT_NONE) {
            return RON_FAULT_CONFIG_INVALID;
        }
        idx++;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-300, RON-FR-302 | Test: RON-TC-GS-001, RON-TC-GS-003, RON-TC-GS-004 */
static uint8_t gs_find_lower_index(const ron_gs_table_t *tbl, ron_float_t sigma_value)
{
    uint8_t low;
    uint8_t high;

    if ((tbl->n_points == 1U) || (sigma_value <= tbl->sigma[0])) {
        return 0U;
    }

    high = (uint8_t) (tbl->n_points - 1U);
    if (sigma_value >= tbl->sigma[high]) {
        return high;
    }

    low = 0U;
    while ((uint8_t) (high - low) > 1U) {
        uint8_t mid = (uint8_t) (low + ((uint8_t) (high - low) / 2U));

        if (sigma_value < tbl->sigma[mid]) {
            high = mid;
        } else {
            low = mid;
        }
    }

    return low;
}

/* Satisfies: RON-FR-302 | Test: RON-TC-GS-004 */
static ron_float_t gs_lerp(ron_float_t lower, ron_float_t upper, ron_float_t t)
{
    return lower + ((upper - lower) * t);
}

/* Satisfies: RON-FR-302, RON-FR-303 | Test: RON-TC-GS-004, RON-TC-GS-005 */
static void gs_build_candidate(const ron_gs_table_t *tbl, uint8_t lower_idx,
                               ron_float_t sigma_value, ron_pid_config_t *candidate)
{
    uint8_t upper_idx;
    ron_float_t t;
    ron_float_t span;

    if ((tbl->mode == RON_GS_HARD_SWITCH) || (tbl->n_points == 1U) ||
        (sigma_value <= tbl->sigma[0])) {
        *candidate = tbl->configs[lower_idx];
        return;
    }

    upper_idx = (uint8_t) (tbl->n_points - 1U);
    if (lower_idx >= upper_idx) {
        *candidate = tbl->configs[upper_idx];
        return;
    }

    upper_idx  = (uint8_t) (lower_idx + 1U);
    *candidate = tbl->configs[lower_idx];
    span       = tbl->sigma[upper_idx] - tbl->sigma[lower_idx];
    t          = (sigma_value - tbl->sigma[lower_idx]) / span;

    candidate->Kp = gs_lerp(tbl->configs[lower_idx].Kp, tbl->configs[upper_idx].Kp, t);
    candidate->Ki = gs_lerp(tbl->configs[lower_idx].Ki, tbl->configs[upper_idx].Ki, t);
    candidate->Kd = gs_lerp(tbl->configs[lower_idx].Kd, tbl->configs[upper_idx].Kd, t);
}

/* Satisfies: RON-FR-300, RON-FR-301, RON-FR-306 | Test: RON-TC-GS-001, RON-TC-GS-002, RON-TC-GS-008 */
ron_fault_t ron_gs_init(const ron_gs_table_t *tbl)
{
    return gs_validate_table(tbl);
}

/* Satisfies: RON-FR-302, RON-FR-303, RON-FR-304, RON-FR-305 | Test: RON-TC-GS-003 - RON-TC-GS-007 */
ron_fault_t ron_gs_update(const ron_gs_table_t *tbl, ron_pid_instance_t *pid, ron_float_t sigma)
{
    ron_fault_t fault;
    ron_pid_config_t candidate;
    uint8_t lower_idx;
    bool switched;

    if ((tbl == NULL) || (pid == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!pid->state.is_initialised || !gs_isfinite(sigma)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    fault = gs_validate_table(tbl);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lower_idx = gs_find_lower_index(tbl, sigma);
    gs_build_candidate(tbl, lower_idx, sigma, &candidate);
    switched = !gs_same_config(&pid->config, &candidate);
    /* Table validation plus bounded interpolation guarantee a valid candidate. */
    (void) ron_pid_set_config(pid, &candidate);

    if ((tbl->mode == RON_GS_HARD_SWITCH) && tbl->reset_integral_on_switch && switched) {
        pid->state.integral = ron_clamp(RON_FLOAT_C(0.0), pid->config.I_min, pid->config.I_max);
    }

    return RON_FAULT_NONE;
}
