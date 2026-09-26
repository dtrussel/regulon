/*
 * @file     ron_statespace.c
 * @brief    Discrete-time state-feedback controller with integral augmentation.
 * @module   ron_statespace
 * @doc      RON-IS-001
 * @req      RON-FR-700, RON-FR-701, RON-FR-702, RON-FR-703, RON-FR-704
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_statespace.h"

#include "ron_matrix_internal.h"
#include "ron_util_internal.h"

/* Scalar finiteness via the shared (fully exercised) vector helper, so this
 * unit carries no inline RON_ISFINITE macro expansions. */
/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-702 | Test: RON-TC-SS-003, RON-TC-SS-009 */
static bool ss_integral_valid(const ron_ss_config_t *cfg)
{
    if (!ron_util_isfinite(cfg->Ki_aug) || !ron_util_isfinite(cfg->i_min) ||
        !ron_util_isfinite(cfg->i_max)) {
        return false;
    }
    if (cfg->i_min > cfg->i_max) {
        return false;
    }
    return ron_mat_vec_finite(&cfg->C_out[0], cfg->n);
}

/* Satisfies: RON-FR-700, RON-FR-703 | Test: RON-TC-SS-009 */
static bool ss_limits_valid(const ron_ss_config_t *cfg)
{
    if (!ron_util_isfinite(cfg->u_min) || !ron_util_isfinite(cfg->u_max) ||
        !ron_util_isfinite(cfg->du_max)) {
        return false;
    }
    if (cfg->u_min >= cfg->u_max) {
        return false;
    }
    if (cfg->use_integral && !ss_integral_valid(cfg)) {
        return false;
    }

    return true;
}

/* Satisfies: RON-FR-723 | Test: RON-TC-SS-009 */
static bool ss_dims_valid(const ron_ss_config_t *cfg)
{
    return (cfg->n >= 1U) && (cfg->n <= (uint8_t) RON_SS_MAX_STATES);
}

/* Satisfies: RON-FR-700, RON-FR-701, RON-FR-723 | Test: RON-TC-SS-002, RON-TC-SS-009 */
static ron_fault_t ss_validate_config(const ron_ss_config_t *cfg)
{
    if (!ss_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ron_mat_vec_finite(&cfg->K[0], cfg->n) || !ron_util_isfinite(cfg->Kr)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ss_limits_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (ron_estimator_config_validate(&cfg->est, cfg->n) != RON_FAULT_NONE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Control-law computation (RON-FR-700, RON-FR-702)
 * ========================================================================= */

/* Satisfies: RON-FR-700, RON-FR-702 | Test: RON-TC-SS-001, RON-TC-SS-003 */
static ron_float_t ss_dot(const ron_float_t *a, const ron_float_t *b, uint8_t n)
{
    ron_float_t sum = RON_FLOAT_C(0.0);
    uint8_t i;

    for (i = 0U; i < n; i++) {
        sum += a[i] * b[i];
    }

    return sum;
}

/* Satisfies: RON-FR-700, RON-FR-702 | Test: RON-TC-SS-001, RON-TC-SS-003 */
static ron_float_t ss_compute_raw(ron_ss_t *ss, ron_float_t r, ron_float_t dt,
                                  const ron_float_t *x_hat)
{
    const ron_ss_config_t *cfg = &ss->cfg;
    uint8_t n                  = cfg->n;
    ron_float_t u_fb           = -ss_dot(&cfg->K[0], x_hat, n);
    ron_float_t u_raw          = u_fb + (cfg->Kr * r);

    if (cfg->use_integral) {
        ron_float_t e_reg = r - ss_dot(&cfg->C_out[0], x_hat, n);

        ss->state.integral += cfg->Ki_aug * dt * e_reg;
        ss->state.integral = ron_clamp(ss->state.integral, cfg->i_min, cfg->i_max);
        u_raw += ss->state.integral;
    }

    return u_raw;
}

/* =========================================================================
 * Output limiting (RON-FR-703, PID-equivalent semantics)
 * ========================================================================= */

/* Satisfies: RON-FR-020, RON-FR-022, RON-FR-703 | Test: RON-TC-SS-004 */
static ron_float_t ss_apply_limits(const ron_ss_t *ss, ron_float_t u_raw, ron_float_t dt,
                                   ron_status_t *status)
{
    const ron_ss_config_t *cfg = &ss->cfg;
    ron_float_t u_sat          = ron_clamp(u_raw, cfg->u_min, cfg->u_max);
    bool rate_limited          = false;
    ron_float_t u_final;

    if (u_sat != u_raw) {
        *status = (ron_status_t) (*status | RON_STATUS_SATURATED);
    }

    u_final = ron_util_rate_limit(u_sat, ss->state.u_prev, cfg->du_max, dt, &rate_limited);
    if (rate_limited) {
        *status = (ron_status_t) (*status | RON_STATUS_RATE_LIMITED);
    }

    return u_final;
}

/* =========================================================================
 * Step (RON-FR-700, RON-FR-702, RON-FR-703)
 * ========================================================================= */

/* Satisfies: RON-FR-700, RON-FR-702, RON-FR-703 | Test: RON-TC-SS-001, RON-TC-SS-003, RON-TC-SS-004 */
ron_fault_t ron_ss_step(ron_ss_t *ss, ron_float_t r, ron_float_t dt, ron_float_t *u,
                        ron_status_t *status)
{
    ron_float_t x_hat[RON_SS_MAX_STATES];
    ron_float_t u_raw;
    ron_float_t u_final;
    ron_status_t step_status = RON_STATUS_OK;
    ron_fault_t fault;

    if ((ss == NULL) || (u == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ron_util_isfinite(r) || !ron_util_isfinite(dt) || (dt <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_INPUT_NAN;
    }

    fault = ron_estimator_get_state(&ss->est, x_hat, ss->cfg.n);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    u_raw = ss_compute_raw(ss, r, dt, x_hat);
    if (!ron_util_isfinite(u_raw)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    u_final = ss_apply_limits(ss, u_raw, dt, &step_status);

    ss->state.u_prev = u_final;
    *u               = u_final;
    *status          = step_status;

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Lifecycle, runtime gains, embedded estimators
 * ========================================================================= */

/* Satisfies: RON-FR-702, RON-FR-703 | Test: RON-TC-SS-003 */
static void ss_seed_state(ron_ss_t *ss)
{
    ss->state.integral = RON_FLOAT_C(0.0);
    ss->state.u_prev   = RON_FLOAT_C(0.0);
    ss->state.faults   = RON_FAULT_NONE;
}

/* Satisfies: RON-FR-700, RON-FR-701 | Test: RON-TC-SS-001, RON-TC-SS-002 */
ron_fault_t ron_ss_init(ron_ss_t *ss, const ron_ss_config_t *cfg)
{
    ron_fault_t fault;

    if ((ss == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = ss_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    ss->cfg = *cfg;

    fault = ron_estimator_init(&ss->est, &cfg->est, cfg->n);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    ss_seed_state(ss);
    ss->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-702 | Test: RON-TC-SS-003 */
ron_fault_t ron_ss_reset(ron_ss_t *ss)
{
    if (ss == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    ss_seed_state(ss);
    (void) ron_estimator_reset(&ss->est);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-704 | Test: RON-TC-SS-005 */
ron_fault_t ron_ss_set_gains(ron_ss_t *ss, const ron_float_t K[RON_SS_MAX_STATES], ron_float_t Kr)
{
    uint8_t i;

    if ((ss == NULL) || (K == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ron_mat_vec_finite(K, ss->cfg.n) || !ron_util_isfinite(Kr)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    for (i = 0U; i < ss->cfg.n; i++) {
        ss->cfg.K[i] = K[i];
    }
    ss->cfg.Kr = Kr;

    return RON_FAULT_NONE;
}
