/*
 * @file     ron_lqr.c
 * @brief    Discrete-time MIMO LQR: DARE solver and state-feedback control law.
 * @module   ron_lqr
 * @doc      RON-IS-001
 * @req      RON-FR-730, RON-FR-731, RON-FR-732, RON-FR-733, RON-FR-734,
 *           RON-FR-735, RON-FR-736, RON-FR-737, RON-FR-738, RON-FR-739
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_lqr.h"

#include "ron_lqr_internal.h"
#include "ron_matrix_internal.h"

#define RON_LQR_DARE_DEFAULT_MAX_ITER 200U

/* Satisfies: RON-SR-020 | Test: RON-TC-LQR-006 */
static bool lqr_finite(ron_float_t v)
{
    return ron_mat_vec_finite(&v, 1U);
}

/* =========================================================================
 * DARE solver (RON-FR-731, RON-FR-733, RON-FR-739) — SADS DD-19: iterative
 * value recursion, no Schur decomposition.  Shared with ron_lqg via
 * ron_lqr_internal.h.
 * ========================================================================= */

/* Satisfies: RON-FR-733 | Test: RON-TC-LQR-003 */
static void lqr_mat_copy(ron_mat_t dst, ron_mat_t src, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            dst[i][j] = src[i][j];
        }
    }
}

/* Satisfies: RON-FR-733 | Test: RON-TC-LQR-003 */
static void lqr_mat_sub(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            out[i][j] = lhs[i][j] - rhs[i][j];
        }
    }
}

/* Satisfies: RON-FR-733 | Test: RON-TC-LQR-003 */
static ron_float_t lqr_mat_max_abs_diff(ron_mat_t a, ron_mat_t b, uint8_t rows, uint8_t cols)
{
    ron_float_t max_diff = RON_FLOAT_C(0.0);
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            ron_float_t diff = ron_fabs(a[i][j] - b[i][j]);

            if (diff > max_diff) {
                max_diff = diff;
            }
        }
    }

    return max_diff;
}

/* K_i <- (R + B^T P B)^-1 (B^T P A) via Cholesky solve. Returns false if
 * R + B^T P B is not positive definite. */
/* Satisfies: RON-FR-733 | Test: RON-TC-LQR-003 */
static bool lqr_dare_solve_gain(ron_mat_t p, ron_mat_t a_work, ron_mat_t b_work, ron_mat_t r_work,
                                uint8_t n, uint8_t m, ron_mat_t k_i)
{
    ron_mat_t btp;
    ron_mat_t m_mat;
    ron_mat_t btpa;
    uint8_t i;
    uint8_t j;

    /* B^T is formed on the fly rather than materialised: at these dimensions
     * a scratch matrix costs more stack than the operation itself. */
    ron_mat_mul_ta(btp, b_work, p, m, n, n); /* B^T P    (m x n) */
    ron_mat_mul(m_mat, btp, b_work, m, n, m);
    ron_mat_add(m_mat, r_work, m_mat, m, m); /* M = R + B^T P B  */

    if (!ron_mat_cholesky(m_mat, m)) {
        return false;
    }

    ron_mat_mul(btpa, btp, a_work, m, n, n); /* B^T P A  (m x n) */
    for (i = 0U; i < n; i++) {
        ron_float_t rhs[RON_MAT_MAX_DIM];

        for (j = 0U; j < m; j++) {
            rhs[j] = btpa[j][i];
        }
        ron_mat_chol_solve(m_mat, rhs, m);
        for (j = 0U; j < m; j++) {
            k_i[j][i] = rhs[j];
        }
    }

    return true;
}

/* P_new <- Q + A^T P A - A^T P B K_i. */
/* Satisfies: RON-FR-733 | Test: RON-TC-LQR-003 */
static void lqr_dare_update_p(ron_mat_t p, ron_mat_t a_work, ron_mat_t b_work, ron_mat_t q_work,
                              ron_mat_t k_i, uint8_t n, uint8_t m, ron_mat_t p_new)
{
    /* Three scratch matrices carry six products.  A^T is formed on the fly,
     * and each buffer is reused as soon as its last reader has run - both to
     * keep this frame small, since it is the deepest one in the library. */
    ron_mat_t atp;  /* A^T P, then A^T P B K            */
    ron_mat_t atpa; /* A^T P A, then Q + A^T P A        */
    ron_mat_t atpb; /* A^T P B                          */

    ron_mat_mul_ta(atp, a_work, p, n, n, n); /* A^T P            */
    ron_mat_mul(atpa, atp, a_work, n, n, n); /* A^T P A          */
    ron_mat_mul(atpb, atp, b_work, n, n, m); /* A^T P B          */
    ron_mat_mul(atp, atpb, k_i, n, m, n);    /* A^T P B K        */
    ron_mat_add(atpa, q_work, atpa, n, n);   /* Q + A^T P A      */
    lqr_mat_sub(p_new, atpa, atp, n, n);
}

/* Satisfies: RON-FR-731, RON-FR-733, RON-FR-739, RON-FR-756 | Test: RON-TC-LQR-003, RON-TC-LQG-006 */
ron_fault_t ron_lqr_dare_solve(const ron_float_t *a, const ron_float_t *b, const ron_float_t *q,
                               const ron_float_t *r, uint8_t n, uint8_t m, uint16_t max_iter,
                               ron_float_t tol, ron_float_t *k_out, ron_float_t *p_out)
{
    ron_mat_t a_work;
    ron_mat_t b_work;
    ron_mat_t q_work;
    ron_mat_t r_work;
    ron_mat_t p;
    uint16_t effective_max_iter = (max_iter == 0U) ? RON_LQR_DARE_DEFAULT_MAX_ITER : max_iter;
    uint16_t iter;

    ron_mat_load(a_work, a, (uint8_t) RON_LQR_MAX_STATES, n, n);
    ron_mat_load(b_work, b, (uint8_t) RON_LQR_MAX_INPUTS, n, m);
    ron_mat_load(q_work, q, (uint8_t) RON_LQR_MAX_STATES, n, n);
    ron_mat_load(r_work, r, (uint8_t) RON_LQR_MAX_INPUTS, m, m);
    lqr_mat_copy(p, q_work, n, n); /* P <- Q */

    for (iter = 0U; iter < effective_max_iter; iter++) {
        ron_mat_t k_i;
        ron_mat_t p_new;

        if (!lqr_dare_solve_gain(p, a_work, b_work, r_work, n, m, k_i)) {
            return RON_FAULT_CONFIG_INVALID;
        }

        lqr_dare_update_p(p, a_work, b_work, q_work, k_i, n, m, p_new);

        if (lqr_mat_max_abs_diff(p_new, p, n, n) < tol) {
            ron_mat_store(k_out, (uint8_t) RON_LQR_MAX_STATES, k_i, m, n);
            ron_mat_store(p_out, (uint8_t) RON_LQR_MAX_STATES, p_new, n, n);
            return RON_FAULT_NONE;
        }

        lqr_mat_copy(p, p_new, n, n);
    }

    return RON_FAULT_CONFIG_INVALID; /* did not converge within max_iter */
}

/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-737 | Test: RON-TC-LQR-006 */
static bool lqr_dims_valid(const ron_lqr_config_t *cfg)
{
    if ((cfg->n < 1U) || (cfg->n > (uint8_t) RON_LQR_MAX_STATES)) {
        return false;
    }
    return (cfg->m >= 1U) && (cfg->m <= (uint8_t) RON_LQR_MAX_INPUTS);
}

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-002 */
static bool lqr_source_valid(ron_lqr_source_t source)
{
    return (source == RON_LQR_SOURCE_EXTERNAL) || (source == RON_LQR_SOURCE_LUENBERGER) ||
           (source == RON_LQR_SOURCE_KALMAN);
}

/* Satisfies: RON-FR-732 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
static bool lqr_gain_mode_valid(ron_lqr_gain_mode_t mode)
{
    return (mode == RON_LQR_GAIN_PRECOMPUTED) || (mode == RON_LQR_GAIN_DARE);
}

/* Satisfies: RON-FR-731 | Test: RON-TC-LQR-003 */
static bool lqr_dare_cost_valid(const ron_lqr_config_t *cfg)
{
    if (!ron_mat_strided_finite(&cfg->Q_cost[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n) ||
        !ron_mat_strided_finite(&cfg->R_cost[0][0], (uint8_t) RON_LQR_MAX_INPUTS, cfg->m, cfg->m)) {
        return false;
    }
    return lqr_finite(cfg->dare_tol) && (cfg->dare_tol > RON_FLOAT_C(0.0));
}

/* Kr is always consumed by the control law; K is only meaningful in
 * PRECOMPUTED mode (DARE mode overwrites it before first use). */
/* Satisfies: RON-FR-732, RON-FR-738 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
static bool lqr_gain_valid(const ron_lqr_config_t *cfg)
{
    if (!ron_mat_vec_finite(&cfg->Kr[0], cfg->m)) {
        return false;
    }
    if (cfg->gain_mode == RON_LQR_GAIN_PRECOMPUTED) {
        return ron_mat_strided_finite(&cfg->K[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->m, cfg->n);
    }
    return lqr_dare_cost_valid(cfg);
}

/* A/B are required in DARE mode or when an embedded estimator advances from
 * them; otherwise (EXTERNAL + PRECOMPUTED) they are unused. */
/* Satisfies: RON-FR-731, RON-FR-734 | Test: RON-TC-LQR-001, RON-TC-LQR-008, RON-TC-LQR-009 */
static bool lqr_system_matrices_ok(const ron_lqr_config_t *cfg)
{
    bool needed = (cfg->gain_mode == RON_LQR_GAIN_DARE) || (cfg->source != RON_LQR_SOURCE_EXTERNAL);

    if (!needed) {
        return true;
    }
    return ron_mat_strided_finite(&cfg->A[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->n, cfg->n) &&
           ron_mat_strided_finite(&cfg->B[0][0], (uint8_t) RON_LQR_MAX_INPUTS, cfg->n, cfg->m);
}

/* Satisfies: RON-FR-735 | Test: RON-TC-LQR-007 */
static bool lqr_integral_valid(const ron_lqr_config_t *cfg)
{
    uint8_t j;

    if (!cfg->use_integral) {
        return true;
    }
    if (!ron_mat_vec_finite(&cfg->Ki_aug[0], cfg->m) ||
        !ron_mat_vec_finite(&cfg->i_min[0], cfg->m) ||
        !ron_mat_vec_finite(&cfg->i_max[0], cfg->m)) {
        return false;
    }
    for (j = 0U; j < cfg->m; j++) {
        if (cfg->i_min[j] > cfg->i_max[j]) {
            return false;
        }
        if (!ron_mat_vec_finite(&cfg->C_out[j][0], cfg->n)) {
            return false;
        }
    }
    return true;
}

/* Satisfies: RON-FR-736 | Test: RON-TC-LQR-004 */
static bool lqr_limits_valid(const ron_lqr_config_t *cfg)
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

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-008, RON-TC-LQR-009 */
static bool lqr_embedded_dims_ok(const ron_lqr_config_t *cfg)
{
    if ((cfg->source == RON_LQR_SOURCE_LUENBERGER) && (cfg->obs_cfg.n != cfg->n)) {
        return false;
    }
    if ((cfg->source == RON_LQR_SOURCE_KALMAN) && (cfg->kf_cfg.n != cfg->n)) {
        return false;
    }
    return true;
}

/* Satisfies: RON-FR-730, RON-FR-732, RON-FR-734, RON-FR-736, RON-FR-737 | Test: RON-TC-LQR-001, RON-TC-LQR-006 */
static ron_fault_t lqr_validate_config(const ron_lqr_config_t *cfg)
{
    if (!lqr_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_source_valid(cfg->source)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_gain_mode_valid(cfg->gain_mode)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_gain_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_system_matrices_ok(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_integral_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_limits_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_embedded_dims_ok(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * State-estimate acquisition (RON-FR-734)
 * ========================================================================= */

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-002 */
static ron_fault_t lqr_fetch_state(const ron_lqr_t *lqr, ron_float_t *x_hat, uint8_t n)
{
    const ron_float_t *src;
    uint8_t i;

    switch (lqr->cfg.source) {
    case RON_LQR_SOURCE_LUENBERGER:
        src = &lqr->observer.state.x_hat[0];
        break;
    case RON_LQR_SOURCE_KALMAN:
        src = &lqr->kalman.state.x_hat[0];
        break;
    default: /* RON_LQR_SOURCE_EXTERNAL */
        if (lqr->cfg.x_ext == NULL) {
            return RON_FAULT_NULL_POINTER;
        }
        src = lqr->cfg.x_ext;
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
 * Control-law computation (RON-FR-730, RON-FR-735)
 * ========================================================================= */

/* Satisfies: RON-FR-730 | Test: RON-TC-LQR-001 */
static void lqr_compute_fb(const ron_lqr_t *lqr, const ron_float_t *x_hat, ron_float_t *u_fb)
{
    uint8_t n = lqr->cfg.n;
    uint8_t m = lqr->cfg.m;
    uint8_t j;
    uint8_t i;

    for (j = 0U; j < m; j++) {
        ron_float_t sum = RON_FLOAT_C(0.0);

        for (i = 0U; i < n; i++) {
            sum += lqr->state.K_solved[j][i] * x_hat[i];
        }
        u_fb[j] = -sum;
    }
}

/* Satisfies: RON-FR-735 | Test: RON-TC-LQR-007 */
static void lqr_apply_integral(ron_lqr_t *lqr, const ron_float_t *r, ron_float_t dt,
                               const ron_float_t *x_hat, ron_float_t *u_raw)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;
    uint8_t j;
    uint8_t i;

    for (j = 0U; j < cfg->m; j++) {
        ron_float_t reg = RON_FLOAT_C(0.0);
        ron_float_t e_reg;

        for (i = 0U; i < cfg->n; i++) {
            reg += cfg->C_out[j][i] * x_hat[i];
        }
        e_reg = r[j] - reg;

        lqr->state.integral[j] += cfg->Ki_aug[j] * dt * e_reg;
        lqr->state.integral[j] = ron_clamp(lqr->state.integral[j], cfg->i_min[j], cfg->i_max[j]);
        u_raw[j] += lqr->state.integral[j];
    }
}

/* Satisfies: RON-FR-730, RON-FR-735 | Test: RON-TC-LQR-001, RON-TC-LQR-007 */
static void lqr_compute_raw(ron_lqr_t *lqr, const ron_float_t *r, ron_float_t dt,
                            const ron_float_t *x_hat, ron_float_t *u_raw)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;
    uint8_t j;

    lqr_compute_fb(lqr, x_hat, u_raw);
    for (j = 0U; j < cfg->m; j++) {
        u_raw[j] += cfg->Kr[j] * r[j];
    }
    if (cfg->use_integral) {
        lqr_apply_integral(lqr, r, dt, x_hat, u_raw);
    }
}

/* =========================================================================
 * Output limiting (RON-FR-736, PID-equivalent semantics, per input)
 * ========================================================================= */

/* Satisfies: RON-FR-022, RON-FR-736 | Test: RON-TC-LQR-004 */
static ron_float_t lqr_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
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

/* Satisfies: RON-FR-020, RON-FR-022, RON-FR-736 | Test: RON-TC-LQR-004 */
static void lqr_apply_limits(ron_lqr_t *lqr, const ron_float_t *u_raw, ron_float_t dt,
                             ron_float_t *u, ron_status_t *status)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;
    uint8_t j;

    for (j = 0U; j < cfg->m; j++) {
        ron_float_t u_sat = ron_clamp(u_raw[j], cfg->u_min[j], cfg->u_max[j]);
        bool rate_limited = false;
        ron_float_t u_final;

        if (u_sat != u_raw[j]) {
            *status = (ron_status_t) (*status | RON_STATUS_SATURATED);
        }

        u_final = lqr_rate_limit(u_sat, lqr->state.u_prev[j], cfg->du_max[j], dt, &rate_limited);
        if (rate_limited) {
            *status = (ron_status_t) (*status | RON_STATUS_RATE_LIMITED);
        }

        lqr->state.u_prev[j] = u_final;
        u[j]                 = u_final;
    }
}

/* =========================================================================
 * Step (RON-FR-730, RON-FR-735, RON-FR-736)
 * ========================================================================= */

/* Satisfies: RON-FR-730, RON-FR-736 | Test: RON-TC-LQR-006 */
static bool lqr_step_args_valid(const ron_lqr_t *lqr, const ron_float_t *r, ron_float_t dt)
{
    if (!ron_mat_vec_finite(r, lqr->cfg.m) || !lqr_finite(dt)) {
        return false;
    }
    return dt > RON_FLOAT_C(0.0);
}

/* Satisfies: RON-FR-730, RON-FR-735, RON-FR-736 | Test: RON-TC-LQR-001, RON-TC-LQR-004, RON-TC-LQR-006, RON-TC-LQR-007 */
ron_fault_t ron_lqr_step(ron_lqr_t *lqr, const ron_float_t r[RON_LQR_MAX_INPUTS], ron_float_t dt,
                         ron_float_t u[RON_LQR_MAX_INPUTS], ron_status_t *status)
{
    ron_float_t x_hat[RON_LQR_MAX_STATES];
    ron_float_t u_raw[RON_LQR_MAX_INPUTS];
    ron_status_t step_status = RON_STATUS_OK;
    ron_fault_t fault;

    if ((lqr == NULL) || (r == NULL) || (u == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!lqr_step_args_valid(lqr, r, dt)) {
        return RON_FAULT_INPUT_NAN;
    }

    fault = lqr_fetch_state(lqr, x_hat, lqr->cfg.n);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lqr_compute_raw(lqr, r, dt, x_hat, u_raw);
    if (!ron_mat_vec_finite(u_raw, lqr->cfg.m)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    lqr_apply_limits(lqr, u_raw, dt, u, &step_status);
    *status = step_status;

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Lifecycle, runtime gains, embedded estimators
 * ========================================================================= */

/* Satisfies: RON-FR-739 | Test: RON-TC-LQR-001 */
static void lqr_zero_matrix(ron_float_t *dst, uint8_t n)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < n; i++) {
        for (j = 0U; j < n; j++) {
            dst[((size_t) i * (size_t) RON_LQR_MAX_STATES) + (size_t) j] = RON_FLOAT_C(0.0);
        }
    }
}

/* Satisfies: RON-FR-732, RON-FR-738, RON-FR-739 | Test: RON-TC-LQR-001 */
static ron_fault_t lqr_resolve_gain_precomputed(ron_lqr_t *lqr)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;
    ron_mat_t k_work;

    ron_mat_load(k_work, &cfg->K[0][0], (uint8_t) RON_LQR_MAX_STATES, cfg->m, cfg->n);
    ron_mat_store(&lqr->state.K_solved[0][0], (uint8_t) RON_LQR_MAX_STATES, k_work, cfg->m, cfg->n);
    lqr_zero_matrix(&lqr->state.P_solved[0][0], cfg->n);
    lqr->state.dare_converged = false;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-731, RON-FR-733, RON-FR-739 | Test: RON-TC-LQR-003 */
static ron_fault_t lqr_resolve_gain_dare(ron_lqr_t *lqr)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;
    ron_fault_t fault;

    fault = ron_lqr_dare_solve(&cfg->A[0][0], &cfg->B[0][0], &cfg->Q_cost[0][0], &cfg->R_cost[0][0],
                               cfg->n, cfg->m, cfg->dare_max_iter, cfg->dare_tol,
                               &lqr->state.K_solved[0][0], &lqr->state.P_solved[0][0]);
    lqr->state.dare_converged = (fault == RON_FAULT_NONE);

    return fault;
}

/* Satisfies: RON-FR-732 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
static ron_fault_t lqr_resolve_gain(ron_lqr_t *lqr)
{
    if (lqr->cfg.gain_mode == RON_LQR_GAIN_PRECOMPUTED) {
        return lqr_resolve_gain_precomputed(lqr);
    }
    return lqr_resolve_gain_dare(lqr);
}

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-008, RON-TC-LQR-009 */
static ron_fault_t lqr_init_estimator(ron_lqr_t *lqr)
{
    const ron_lqr_config_t *cfg = &lqr->cfg;

    if (cfg->source == RON_LQR_SOURCE_LUENBERGER) {
        return ron_obs_init(&lqr->observer, &cfg->obs_cfg);
    }
    if (cfg->source == RON_LQR_SOURCE_KALMAN) {
        return ron_kf_init(&lqr->kalman, &cfg->kf_cfg);
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-735, RON-FR-736 | Test: RON-TC-LQR-001 */
static void lqr_seed_state(ron_lqr_t *lqr)
{
    uint8_t j;

    for (j = 0U; j < lqr->cfg.m; j++) {
        lqr->state.integral[j] = RON_FLOAT_C(0.0);
        lqr->state.u_prev[j]   = RON_FLOAT_C(0.0);
    }
    lqr->state.faults = RON_FAULT_NONE;
}

/* Satisfies: RON-FR-730, RON-FR-732, RON-FR-734 | Test: RON-TC-LQR-001, RON-TC-LQR-002, RON-TC-LQR-003, RON-TC-LQR-006 */
ron_fault_t ron_lqr_init(ron_lqr_t *lqr, const ron_lqr_config_t *cfg)
{
    ron_fault_t fault;

    if ((lqr == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = lqr_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lqr->cfg = *cfg;

    fault = lqr_resolve_gain(lqr);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    fault = lqr_init_estimator(lqr);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    lqr_seed_state(lqr);
    lqr->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-735 | Test: RON-TC-LQR-006 */
ron_fault_t ron_lqr_reset(ron_lqr_t *lqr)
{
    if (lqr == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    lqr_seed_state(lqr);

    if (lqr->cfg.source == RON_LQR_SOURCE_LUENBERGER) {
        (void) ron_obs_reset(&lqr->observer);
    } else if (lqr->cfg.source == RON_LQR_SOURCE_KALMAN) {
        (void) ron_kf_reset(&lqr->kalman);
    } else {
        /* EXTERNAL: nothing to reset. */
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-738 | Test: RON-TC-LQR-005 */
ron_fault_t ron_lqr_set_gains(ron_lqr_t *lqr,
                              const ron_float_t K[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES],
                              const ron_float_t Kr[RON_LQR_MAX_INPUTS])
{
    ron_mat_t k_work;
    uint8_t j;

    if ((lqr == NULL) || (K == NULL) || (Kr == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!ron_mat_strided_finite(&K[0][0], (uint8_t) RON_LQR_MAX_STATES, lqr->cfg.m, lqr->cfg.n) ||
        !ron_mat_vec_finite(&Kr[0], lqr->cfg.m)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    ron_mat_load(k_work, &K[0][0], (uint8_t) RON_LQR_MAX_STATES, lqr->cfg.m, lqr->cfg.n);
    ron_mat_store(&lqr->state.K_solved[0][0], (uint8_t) RON_LQR_MAX_STATES, k_work, lqr->cfg.m,
                  lqr->cfg.n);

    for (j = 0U; j < lqr->cfg.m; j++) {
        lqr->cfg.Kr[j] = Kr[j];
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-739 | Test: RON-TC-LQR-003 */
ron_fault_t ron_lqr_get_dare_solution(const ron_lqr_t *lqr,
                                      ron_float_t P[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES])
{
    ron_mat_t p_work;

    if ((lqr == NULL) || (P == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    ron_mat_load(p_work, &lqr->state.P_solved[0][0], (uint8_t) RON_LQR_MAX_STATES, lqr->cfg.n,
                 lqr->cfg.n);
    ron_mat_store(&P[0][0], (uint8_t) RON_LQR_MAX_STATES, p_work, lqr->cfg.n, lqr->cfg.n);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-008 */
ron_fault_t ron_lqr_observer_step(ron_lqr_t *lqr, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                  const ron_float_t u[RON_SS_MAX_INPUTS])
{
    if (lqr == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised || (lqr->cfg.source != RON_LQR_SOURCE_LUENBERGER)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_obs_step(&lqr->observer, y, u);
}

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-009 */
ron_fault_t ron_lqr_kalman_predict(ron_lqr_t *lqr, const ron_float_t u[RON_KF_MAX_INPUTS])
{
    if (lqr == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised || (lqr->cfg.source != RON_LQR_SOURCE_KALMAN)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_kf_predict(&lqr->kalman, u);
}

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-009 */
ron_fault_t ron_lqr_kalman_update(ron_lqr_t *lqr, const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                                  bool z_valid)
{
    if (lqr == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!lqr->state.is_initialised || (lqr->cfg.source != RON_LQR_SOURCE_KALMAN)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return ron_kf_update(&lqr->kalman, z, z_valid);
}
