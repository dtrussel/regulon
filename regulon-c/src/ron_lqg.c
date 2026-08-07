/*
 * @file     ron_lqg.c
 * @brief    Discrete-time MIMO LQG: dual-DARE init and combined control law.
 * @module   ron_lqg
 * @doc      RON-IS-001
 * @req      RON-FR-750, RON-FR-751, RON-FR-752, RON-FR-753, RON-FR-754,
 *           RON-FR-755, RON-FR-756, RON-FR-757, RON-FR-758, RON-FR-759
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_lqg.h"

#include "ron_lqr_internal.h"
#include "ron_matrix_internal.h"

/* Satisfies: RON-SR-020 | Test: RON-TC-LQG-009 */
static bool lqg_finite(ron_float_t v)
{
    return ron_mat_vec_finite(&v, 1U);
}

/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-750, RON-FR-751 | Test: RON-TC-LQG-009 */
static bool lqg_dims_valid(const ron_lqg_config_t *cfg)
{
    if ((cfg->n < 1U) || (cfg->n > (uint8_t) RON_LQR_MAX_STATES)) {
        return false;
    }
    if ((cfg->m < 1U) || (cfg->m > (uint8_t) RON_LQR_MAX_INPUTS)) {
        return false;
    }
    return (cfg->p >= 1U) && (cfg->p <= (uint8_t) RON_KF_MAX_MEASUREMENTS);
}

/* Satisfies: RON-FR-756 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
static bool lqg_gain_mode_valid(ron_lqg_gain_mode_t mode)
{
    return (mode == RON_LQG_GAIN_PRECOMPUTED) || (mode == RON_LQG_GAIN_DARE);
}

/* A and B feed the DARE solver directly (RON-FR-756) and so must be
 * validated up front; H, the noise covariances, x0/P0, and K_f_inf are
 * consumed only by the embedded Kalman filter and are validated by its own
 * ron_kf_init() (delegation mirrors ron_statespace's embedded-estimator
 * pattern, avoiding redundant checks). */
/* Satisfies: RON-FR-751, RON-FR-756 | Test: RON-TC-LQG-001 */
static bool lqg_system_matrices_finite(const ron_lqg_config_t *cfg)
{
    return ron_mat_strided_finite(&cfg->A[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n) &&
           ron_mat_strided_finite(&cfg->B[0][0], (uint8_t) RON_LQR_MAX_INPUTS, cfg->n, cfg->m);
}

/* Satisfies: RON-FR-751, RON-FR-756 | Test: RON-TC-LQG-006 */
static bool lqg_dare_cost_valid(const ron_lqg_config_t *cfg)
{
    if (!ron_mat_strided_finite(&cfg->Q_cost[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n) ||
        !ron_mat_strided_finite(&cfg->R_cost[0][0], (uint8_t) RON_LQR_MAX_INPUTS, cfg->m, cfg->m)) {
        return false;
    }
    return lqg_finite(cfg->dare_tol) && (cfg->dare_tol > RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-756 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
static bool lqg_gain_valid(const ron_lqg_config_t *cfg)
{
    if (!ron_mat_vec_finite(&cfg->Kr[0], cfg->m)) {
        return false;
    }
    if (cfg->gain_mode == RON_LQG_GAIN_PRECOMPUTED) {
        return ron_mat_strided_finite(&cfg->K[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->m, cfg->n);
    }
    return lqg_dare_cost_valid(cfg);
}

/* Satisfies: RON-FR-757 | Test: RON-TC-LQG-008 */
static bool lqg_limits_valid(const ron_lqg_config_t *cfg)
{
    uint8_t j;

    if (!ron_mat_vec_finite(&cfg->u_min[0], cfg->m) ||
        !ron_mat_vec_finite(&cfg->u_max[0], cfg->m) ||
        !ron_mat_vec_finite(&cfg->du_max[0], cfg->m)) {
        return false;
    }
    for (j = 0U; j < cfg->m; j++) {
        if (cfg->u_min[j] >= cfg->u_max[j]) {
            return false;
        }
    }
    return true;
}

/* Satisfies: RON-FR-750, RON-FR-751, RON-FR-756, RON-FR-757 | Test: RON-TC-LQG-001, RON-TC-LQG-009 */
static ron_fault_t lqg_validate_config(const ron_lqg_config_t *cfg)
{
    if (!lqg_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqg_gain_mode_valid(cfg->gain_mode)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqg_system_matrices_finite(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqg_gain_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqg_limits_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Dual-DARE initialisation (RON-FR-752, RON-FR-756, separation principle)
 * ========================================================================= */

/* Satisfies: RON-FR-756 | Test: RON-TC-LQG-001 */
static void lqg_zero_matrix(ron_float_t *dst, uint8_t n)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < n; i++) {
        for (j = 0U; j < n; j++) {
            dst[((size_t) i * (size_t) RON_LQR_MAX_STATES) + (size_t) j] = RON_FLOAT_C(0.0);
        }
    }
}

/* Satisfies: RON-FR-756 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
static ron_fault_t lqg_resolve_lqr_gain(ron_lqg_t *lqg)
{
    const ron_lqg_config_t *cfg = &lqg->cfg;

    if (cfg->gain_mode == RON_LQG_GAIN_PRECOMPUTED) {
        ron_mat_t k_work;

        ron_mat_load(k_work, &cfg->K[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->m, cfg->n);
        ron_mat_store(&lqg->K_solved[0][0], (uint8_t) RON_LQR_MAX_STATES, k_work, cfg->m, cfg->n);
        lqg_zero_matrix(&lqg->P_lqr[0][0], cfg->n);

        return RON_FAULT_NONE;
    }

    return ron_lqr_dare_solve(&cfg->A[0][0], &cfg->B[0][0], &cfg->Q_cost[0][0], &cfg->R_cost[0][0],
                              cfg->n, cfg->m, cfg->dare_max_iter, cfg->dare_tol,
                              &lqg->K_solved[0][0], &lqg->P_lqr[0][0]);
}

/* Copies the LQG system/noise configuration into an embedded ron_kf_config_t
 * (RON-FR-752): the Kalman gain is solved independently by ron_kf_init from
 * (A, H, Q_noise, R_noise), never from the LQR cost matrices. */
/* Satisfies: RON-FR-751, RON-FR-752, RON-FR-753 | Test: RON-TC-LQG-001, RON-TC-LQG-007 */
static void lqg_populate_kf_config(const ron_lqg_config_t *cfg, ron_kf_config_t *kf_cfg)
{
    ron_mat_t work;
    uint8_t i;

    kf_cfg->n               = cfg->n;
    kf_cfg->m               = cfg->p;
    kf_cfg->p               = cfg->m;
    kf_cfg->use_joseph_form = cfg->use_joseph_form;
    kf_cfg->steady_state    = cfg->use_kf_steady_state;

    ron_mat_load(work, &cfg->A[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n);
    ron_mat_store(&kf_cfg->A[0][0], (uint8_t) RON_KF_MAX_STATES, work, cfg->n, cfg->n);

    ron_mat_load(work, &cfg->B[0][0], (uint8_t) RON_LQR_MAX_INPUTS, cfg->n, cfg->m);
    ron_mat_store(&kf_cfg->B[0][0], (uint8_t) RON_KF_MAX_INPUTS, work, cfg->n, cfg->m);

    ron_mat_load(work, &cfg->H[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->p, cfg->n);
    ron_mat_store(&kf_cfg->H[0][0], (uint8_t) RON_KF_MAX_STATES, work, cfg->p, cfg->n);

    ron_mat_load(work, &cfg->Q_noise[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n);
    ron_mat_store(&kf_cfg->Q[0][0], (uint8_t) RON_KF_MAX_STATES, work, cfg->n, cfg->n);

    ron_mat_load(work, &cfg->R_noise[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, cfg->p, cfg->p);
    ron_mat_store(&kf_cfg->R[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, work, cfg->p, cfg->p);

    ron_mat_load(work, &cfg->P0[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n);
    ron_mat_store(&kf_cfg->P0[0][0], (uint8_t) RON_KF_MAX_STATES, work, cfg->n, cfg->n);

    for (i = 0U; i < cfg->n; i++) {
        kf_cfg->x0[i] = cfg->x0[i];
    }

    if (cfg->use_kf_steady_state) {
        ron_mat_load(work, &cfg->K_f_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, cfg->n, cfg->p);
        ron_mat_store(&kf_cfg->K_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, work, cfg->n,
                      cfg->p);
    }
}

/* Satisfies: RON-FR-752, RON-FR-756 | Test: RON-TC-LQG-001 */
static ron_fault_t lqg_init_kalman(ron_lqg_t *lqg)
{
    ron_kf_config_t kf_cfg = {0};

    lqg_populate_kf_config(&lqg->cfg, &kf_cfg);

    return ron_kf_init(&lqg->kalman, &kf_cfg);
}

/* Satisfies: RON-FR-757 | Test: RON-TC-LQG-009 */
static void lqg_seed_state(ron_lqg_t *lqg)
{
    uint8_t j;

    for (j = 0U; j < lqg->cfg.m; j++) {
        lqg->u_prev[j] = RON_FLOAT_C(0.0);
    }
    lqg->faults = RON_FAULT_NONE;
}

/* Satisfies: RON-FR-750, RON-FR-752, RON-FR-756, RON-FR-759 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
ron_fault_t ron_lqg_init(ron_lqg_t *lqg, const ron_lqg_config_t *cfg)
{
    ron_fault_t fault;

    if ((lqg == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = lqg_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lqg->cfg = *cfg;

    fault = lqg_resolve_lqr_gain(lqg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    fault = lqg_init_kalman(lqg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lqg_seed_state(lqg);
    lqg->is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-757 | Test: RON-TC-LQG-009 */
ron_fault_t ron_lqg_reset(ron_lqg_t *lqg)
{
    if (lqg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqg->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    lqg_seed_state(lqg);

    return ron_kf_reset(&lqg->kalman);
}

/* =========================================================================
 * Predict / update delegation (RON-FR-753, RON-FR-754)
 * ========================================================================= */

/* Satisfies: RON-FR-753 | Test: RON-TC-LQG-002 */
ron_fault_t ron_lqg_predict(ron_lqg_t *lqg, const ron_float_t u[RON_LQR_MAX_INPUTS])
{
    if (lqg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    return ron_kf_predict(&lqg->kalman, u);
}

/* Satisfies: RON-FR-754 | Test: RON-TC-LQG-003, RON-TC-LQG-004 */
ron_fault_t ron_lqg_update(ron_lqg_t *lqg, const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                           bool z_valid)
{
    if (lqg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    return ron_kf_update(&lqg->kalman, z, z_valid);
}

/* =========================================================================
 * Control-law computation and output limiting (RON-FR-755, RON-FR-757)
 * ========================================================================= */

/* Satisfies: RON-FR-755 | Test: RON-TC-LQG-005 */
static void lqg_compute_raw(const ron_lqg_t *lqg, const ron_float_t *r, const ron_float_t *x_hat,
                            ron_float_t *u_raw)
{
    const ron_lqg_config_t *cfg = &lqg->cfg;
    uint8_t j;
    uint8_t i;

    for (j = 0U; j < cfg->m; j++) {
        ron_float_t sum = RON_FLOAT_C(0.0);

        for (i = 0U; i < cfg->n; i++) {
            sum += lqg->K_solved[j][i] * x_hat[i];
        }
        u_raw[j] = -sum + (cfg->Kr[j] * r[j]);
    }
}

/* Satisfies: RON-FR-022, RON-FR-757 | Test: RON-TC-LQG-008 */
static ron_float_t lqg_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
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

/* Satisfies: RON-FR-020, RON-FR-022, RON-FR-757 | Test: RON-TC-LQG-008 */
static void lqg_apply_limits(ron_lqg_t *lqg, const ron_float_t *u_raw, ron_float_t dt,
                             ron_float_t *u, ron_status_t *status)
{
    const ron_lqg_config_t *cfg = &lqg->cfg;
    uint8_t j;

    for (j = 0U; j < cfg->m; j++) {
        ron_float_t u_sat = ron_clamp(u_raw[j], cfg->u_min[j], cfg->u_max[j]);
        bool rate_limited = false;
        ron_float_t u_final;

        if (u_sat != u_raw[j]) {
            *status = (ron_status_t) (*status | RON_STATUS_SATURATED);
        }

        u_final = lqg_rate_limit(u_sat, lqg->u_prev[j], cfg->du_max[j], dt, &rate_limited);
        if (rate_limited) {
            *status = (ron_status_t) (*status | RON_STATUS_RATE_LIMITED);
        }

        lqg->u_prev[j] = u_final;
        u[j]           = u_final;
    }
}

/* Satisfies: RON-FR-755 | Test: RON-TC-LQG-005, RON-TC-LQG-009 */
static bool lqg_step_inputs_finite(const ron_lqg_t *lqg, const ron_float_t *r, ron_float_t dt)
{
    if (!ron_mat_vec_finite(r, lqg->cfg.m) || !lqg_finite(dt) || (dt <= RON_FLOAT_C(0.0))) {
        return false;
    }
    return ron_mat_vec_finite(&lqg->kalman.state.x_hat[0], lqg->cfg.n);
}

/* Satisfies: RON-FR-752, RON-FR-755, RON-FR-757 | Test: RON-TC-LQG-005, RON-TC-LQG-008, RON-TC-LQG-009 */
ron_fault_t ron_lqg_step(ron_lqg_t *lqg, const ron_float_t r[RON_LQR_MAX_INPUTS], ron_float_t dt,
                         ron_float_t u[RON_LQR_MAX_INPUTS], ron_status_t *status)
{
    ron_float_t u_raw[RON_LQR_MAX_INPUTS];
    ron_status_t step_status = RON_STATUS_OK;

    if ((lqg == NULL) || (r == NULL) || (u == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqg->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqg_step_inputs_finite(lqg, r, dt)) {
        return RON_FAULT_INPUT_NAN;
    }

    lqg_compute_raw(lqg, r, &lqg->kalman.state.x_hat[0], u_raw);
    if (!ron_mat_vec_finite(u_raw, lqg->cfg.m)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    lqg_apply_limits(lqg, u_raw, dt, u, &step_status);
    *status = step_status;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-758 | Test: RON-TC-LQG-005, RON-TC-LQG-007 */
ron_fault_t ron_lqg_get_state(const ron_lqg_t *lqg, ron_float_t x_hat[RON_LQR_MAX_STATES])
{
    if (lqg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    return ron_kf_get_state(&lqg->kalman, x_hat);
}
