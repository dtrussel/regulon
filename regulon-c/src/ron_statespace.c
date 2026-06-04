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

/* Scalar finiteness via the shared (fully exercised) vector helper, so this
 * unit carries no inline RON_ISFINITE macro expansions. */
/* Satisfies: RON-SR-020 | Test: RON-TC-SS-009 */
static bool ss_finite(ron_float_t v)
{
    return ron_mat_vec_finite(&v, 1U);
}

/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-702 | Test: RON-TC-SS-003, RON-TC-SS-009 */
static bool ss_integral_valid(const ron_ss_config_t *cfg)
{
    if (!ss_finite(cfg->Ki_aug) || !ss_finite(cfg->i_min) || !ss_finite(cfg->i_max)) {
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
    if (!ss_finite(cfg->u_min) || !ss_finite(cfg->u_max) || !ss_finite(cfg->du_max)) {
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

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002, RON-TC-SS-009 */
static bool ss_source_valid(ron_ss_source_t source)
{
    return (source == RON_SS_SOURCE_EXTERNAL) || (source == RON_SS_SOURCE_LUENBERGER) ||
           (source == RON_SS_SOURCE_KALMAN);
}

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-009 */
static bool ss_embedded_dims_ok(const ron_ss_config_t *cfg)
{
    if ((cfg->source == RON_SS_SOURCE_LUENBERGER) && (cfg->obs_cfg.n != cfg->n)) {
        return false;
    }
    if ((cfg->source == RON_SS_SOURCE_KALMAN) && (cfg->kf_cfg.n != cfg->n)) {
        return false;
    }

    return true;
}

/* Satisfies: RON-FR-700, RON-FR-701, RON-FR-723 | Test: RON-TC-SS-002, RON-TC-SS-009 */
static ron_fault_t ss_validate_config(const ron_ss_config_t *cfg)
{
    if (!ss_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ss_source_valid(cfg->source)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ron_mat_vec_finite(&cfg->K[0], cfg->n) || !ss_finite(cfg->Kr)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ss_limits_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ss_embedded_dims_ok(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * State-estimate acquisition (RON-FR-701)
 * ========================================================================= */

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
static ron_fault_t ss_fetch_state(const ron_ss_t *ss, ron_float_t *x_hat, uint8_t n)
{
    const ron_float_t *src;
    uint8_t i;

    /* A successful ron_ss_init guarantees the selected embedded estimator is
     * initialised, so no further is_initialised check is needed here. */
    switch (ss->cfg.source) {
    case RON_SS_SOURCE_LUENBERGER:
        src = &ss->observer.state.x_hat[0];
        break;
    case RON_SS_SOURCE_KALMAN:
        src = &ss->kalman.state.x_hat[0];
        break;
    default: /* RON_SS_SOURCE_EXTERNAL */
        if (ss->cfg.x_ext == NULL) {
            return RON_FAULT_NULL_POINTER;
        }
        src = ss->cfg.x_ext;
        break;
    }

    if (!ron_mat_vec_finite(src, n)) {
        return RON_FAULT_INPUT_NAN;
    }
    for (i = 0U; i < n; i++) {
        x_hat[i] = src[i];
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

/* Satisfies: RON-FR-022 | Test: RON-TC-SS-004 */
static ron_float_t ss_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
                                 ron_float_t dt, bool *limited)
{
    ron_float_t limited_value = u_sat;

    if (du_max <= RON_FLOAT_C(0.0)) {
        *limited = false;
    } else {
        ron_float_t delta_max = du_max * dt;
        ron_float_t delta     = u_sat - u_prev;

        if (delta > delta_max) {
            *limited      = true;
            limited_value = u_prev + delta_max;
        } else if (delta < (-delta_max)) {
            *limited      = true;
            limited_value = u_prev - delta_max;
        } else {
            *limited = false;
        }
    }

    return limited_value;
}

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

    u_final = ss_rate_limit(u_sat, ss->state.u_prev, cfg->du_max, dt, &rate_limited);
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
    if (!ss_finite(r) || !ss_finite(dt) || (dt <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_INPUT_NAN;
    }

    fault = ss_fetch_state(ss, x_hat, ss->cfg.n);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    u_raw = ss_compute_raw(ss, r, dt, x_hat);
    if (!ss_finite(u_raw)) {
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

    if (cfg->source == RON_SS_SOURCE_LUENBERGER) {
        fault = ron_obs_init(&ss->observer, &cfg->obs_cfg);
        if (fault != RON_FAULT_NONE) {
            return fault;
        }
    } else if (cfg->source == RON_SS_SOURCE_KALMAN) {
        fault = ron_kf_init(&ss->kalman, &cfg->kf_cfg);
        if (fault != RON_FAULT_NONE) {
            return fault;
        }
    } else {
        /* EXTERNAL: no embedded estimator to initialise. */
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

    if (ss->cfg.source == RON_SS_SOURCE_LUENBERGER) {
        (void) ron_obs_reset(&ss->observer);
    } else if (ss->cfg.source == RON_SS_SOURCE_KALMAN) {
        (void) ron_kf_reset(&ss->kalman);
    } else {
        /* EXTERNAL: nothing to reset. */
    }

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
    if (!ron_mat_vec_finite(K, ss->cfg.n) || !ss_finite(Kr)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    for (i = 0U; i < ss->cfg.n; i++) {
        ss->cfg.K[i] = K[i];
    }
    ss->cfg.Kr = Kr;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_observer_step(ron_ss_t *ss, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                 const ron_float_t u[RON_SS_MAX_INPUTS])
{
    if (ss == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised || (ss->cfg.source != RON_SS_SOURCE_LUENBERGER)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_obs_step(&ss->observer, y, u);
}

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_kalman_predict(ron_ss_t *ss, const ron_float_t u[RON_KF_MAX_INPUTS])
{
    if (ss == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised || (ss->cfg.source != RON_SS_SOURCE_KALMAN)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_kf_predict(&ss->kalman, u);
}

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_kalman_update(ron_ss_t *ss, const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                                 bool z_valid)
{
    if (ss == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ss->state.is_initialised || (ss->cfg.source != RON_SS_SOURCE_KALMAN)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_kf_update(&ss->kalman, z, z_valid);
}
