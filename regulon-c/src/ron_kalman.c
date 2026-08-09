/*
 * @file     ron_kalman.c
 * @brief    Discrete linear Kalman filter: predict/update with Cholesky solve.
 * @module   ron_kalman
 * @doc      RON-IS-001
 * @req      RON-FR-600, RON-FR-601, RON-FR-602, RON-FR-603,
 *           RON-FR-604, RON-FR-605, RON-FR-606, RON-FR-607
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_kalman.h"

#include "ron_matrix_internal.h"

#if (RON_KF_MAX_STATES > RON_MAT_MAX_DIM) || (RON_KF_MAX_MEASUREMENTS > RON_MAT_MAX_DIM) ||        \
    (RON_KF_MAX_INPUTS > RON_MAT_MAX_DIM)
#error "ron_kalman requires RON_MAT_MAX_DIM >= RON_KF_MAX_{STATES,MEASUREMENTS,INPUTS}"
#endif

/* =========================================================================
 * Kalman-specific finite check
 * ========================================================================= */

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-008 */
static bool kf_state_finite(const ron_kf_t *kf, uint8_t n)
{
    return ron_mat_vec_finite(&kf->state.x_hat[0], n) &&
           ron_mat_strided_finite(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
}

/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-601, RON-FR-607 | Test: RON-TC-KF-002 */
static bool kf_dims_valid(const ron_kf_config_t *cfg)
{
    if ((cfg->n < 1U) || (cfg->n > (uint8_t) RON_KF_MAX_STATES)) {
        return false;
    }
    if ((cfg->m < 1U) || (cfg->m > (uint8_t) RON_KF_MAX_MEASUREMENTS)) {
        return false;
    }
    if (cfg->p > (uint8_t) RON_KF_MAX_INPUTS) {
        return false;
    }

    return true;
}

/* Satisfies: RON-FR-601 | Test: RON-TC-KF-002 */
static ron_fault_t kf_validate_config(const ron_kf_config_t *cfg)
{
    uint8_t n;
    uint8_t m;
    uint8_t p;

    if (!kf_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    n = cfg->n;
    m = cfg->m;
    p = cfg->p;

    if (!ron_mat_strided_finite(&cfg->A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !ron_mat_strided_finite(&cfg->Q[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !ron_mat_strided_finite(&cfg->P0[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !ron_mat_strided_finite(&cfg->H[0][0], (uint8_t) RON_KF_MAX_STATES, m, n) ||
        !ron_mat_strided_finite(&cfg->R[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, m, m) ||
        !ron_mat_strided_finite(&cfg->K_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, n, m) ||
        !ron_mat_strided_finite(&cfg->B[0][0], (uint8_t) RON_KF_MAX_INPUTS, n, p) ||
        !ron_mat_vec_finite(&cfg->x0[0], n)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Predict step (RON-FR-600, RON-FR-602)
 * ========================================================================= */

/* Satisfies: RON-FR-600, RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
static void kf_predict_state(ron_kf_t *kf, const ron_float_t *u, uint8_t n, uint8_t p)
{
    ron_mat_t a_work;
    ron_vec_t ax;
    ron_vec_t bu;
    uint8_t i;

    ron_mat_load(a_work, &kf->cfg.A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    ron_mat_vec(ax, a_work, &kf->state.x_hat[0], n, n);

    for (i = 0U; i < n; i++) {
        bu[i] = RON_FLOAT_C(0.0);
    }
    if (p > 0U) {
        ron_mat_t b_work;

        ron_mat_load(b_work, &kf->cfg.B[0][0], (uint8_t) RON_KF_MAX_INPUTS, n, p);
        ron_mat_vec(bu, b_work, u, n, p);
    }

    for (i = 0U; i < n; i++) {
        kf->state.x_hat[i] = ax[i] + bu[i];
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003, RON-TC-KF-006 */
static void kf_predict_cov(ron_kf_t *kf, uint8_t n)
{
    ron_mat_t a_work;
    ron_mat_t p_work;
    ron_mat_t q_work;
    ron_mat_t ap;

    ron_mat_load(a_work, &kf->cfg.A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    ron_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    ron_mat_load(q_work, &kf->cfg.Q[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);

    /* p_work is dead as an input once A P exists, so A P A^T lands back in it
     * rather than in a fifth scratch matrix. */
    ron_mat_mul(ap, a_work, p_work, n, n, n);
    ron_mat_mul_t(p_work, ap, a_work, n, n, n);
    ron_mat_add(p_work, p_work, q_work, n, n);

    ron_mat_store(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, p_work, n, n);
}

/* Satisfies: RON-FR-600, RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003, RON-TC-KF-006 */
ron_fault_t ron_kf_predict(ron_kf_t *kf, const ron_float_t u[RON_KF_MAX_INPUTS])
{
    uint8_t n;
    uint8_t p;

    if (kf == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!kf->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    n = kf->cfg.n;
    p = kf->cfg.p;

    if (p > 0U) {
        if (u == NULL) {
            return RON_FAULT_NULL_POINTER;
        }
        if (!ron_mat_vec_finite(u, p)) {
            return RON_FAULT_INPUT_NAN;
        }
    }

    kf_predict_state(kf, u, n, p);
    kf_predict_cov(kf, n);

    if (!kf_state_finite(kf, n)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Update step (RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-605, RON-FR-606)
 * ========================================================================= */

/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
static ron_fault_t kf_compute_gain(ron_mat_t pht, ron_mat_t s, ron_mat_t gain, uint8_t n, uint8_t m)
{
    ron_mat_t lower;
    uint8_t i;
    uint8_t j;

    if (m == 1U) {
        if (s[0][0] <= RON_FLOAT_C(0.0)) {
            return RON_FAULT_CONFIG_INVALID;
        }
        for (i = 0U; i < n; i++) {
            gain[i][0] = pht[i][0] / s[0][0];
        }
        return RON_FAULT_NONE;
    }

    ron_mat_load(lower, &s[0][0], (uint8_t) RON_MAT_MAX_DIM, m, m);
    if (!ron_mat_cholesky(lower, m)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    for (i = 0U; i < n; i++) {
        ron_float_t rhs[RON_MAT_MAX_DIM];

        for (j = 0U; j < m; j++) {
            rhs[j] = pht[i][j];
        }
        ron_mat_chol_solve(lower, rhs, m);
        for (j = 0U; j < m; j++) {
            gain[i][j] = rhs[j];
        }
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-603, RON-FR-606 | Test: RON-TC-KF-004, RON-TC-KF-007 */
static ron_fault_t kf_resolve_gain(const ron_kf_t *kf, ron_mat_t h_work, ron_mat_t p_work,
                                   ron_mat_t r_work, ron_mat_t gain, uint8_t n, uint8_t m)
{
    ron_mat_t hp; /* H P, then P H^T once H P has been consumed */
    ron_mat_t s;  /* H P H^T, then the innovation covariance S  */

    if (kf->cfg.steady_state) {
        ron_mat_load(gain, &kf->cfg.K_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, n, m);
        return RON_FAULT_NONE;
    }

    ron_mat_mul(hp, h_work, p_work, m, n, n);
    ron_mat_mul_t(s, hp, h_work, m, n, m);
    ron_mat_add(s, s, r_work, m, m); /* S = H P H^T + R */
    ron_mat_mul_t(hp, p_work, h_work, n, n, m);

    return kf_compute_gain(hp, s, gain, n, m);
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
static void kf_apply_innovation(ron_kf_t *kf, ron_mat_t h_work, ron_mat_t gain,
                                const ron_float_t *z, uint8_t n, uint8_t m)
{
    ron_vec_t hx;
    ron_vec_t innov;
    ron_vec_t dx;
    uint8_t i;

    ron_mat_vec(hx, h_work, &kf->state.x_hat[0], m, n);
    for (i = 0U; i < m; i++) {
        innov[i] = z[i] - hx[i];
    }
    ron_mat_vec(dx, gain, innov, n, m);
    for (i = 0U; i < n; i++) {
        kf->state.x_hat[i] += dx[i];
    }
}

/* Satisfies: RON-FR-604 | Test: RON-TC-KF-004, RON-TC-KF-005 */
static void kf_update_cov(ron_kf_t *kf, ron_mat_t h_work, ron_mat_t r_work, ron_mat_t gain,
                          bool joseph, uint8_t n, uint8_t m)
{
    ron_mat_t p_work;
    ron_mat_t kh;
    ron_mat_t ikh;
    ron_mat_t ikhp;
    uint8_t i;
    uint8_t j;

    ron_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    ron_mat_mul(kh, gain, h_work, n, m, n);
    for (i = 0U; i < n; i++) {
        for (j = 0U; j < n; j++) {
            ron_float_t eye = (i == j) ? RON_FLOAT_C(1.0) : RON_FLOAT_C(0.0);

            ikh[i][j] = eye - kh[i][j];
        }
    }
    ron_mat_mul(ikhp, ikh, p_work, n, n, n);

    if (joseph) {
        /* Joseph form needs three more products but no more storage: kh and
         * p_work are both dead by here, and ikhp is dead the moment its
         * contribution has been folded into kh. */
        ron_mat_mul_t(kh, ikhp, ikh, n, n, n);      /* (I-KH) P (I-KH)^T */
        ron_mat_mul(p_work, gain, r_work, n, m, m); /* K R               */
        ron_mat_mul_t(ikhp, p_work, gain, n, m, n); /* K R K^T           */
        ron_mat_add(ikhp, kh, ikhp, n, n);
    }

    ron_mat_store(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, ikhp, n, n);
}

/* Satisfies: RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-606 | Test: RON-TC-KF-001, RON-TC-KF-004, RON-TC-KF-005, RON-TC-KF-007 */
static ron_fault_t kf_do_update(ron_kf_t *kf, const ron_float_t *z, uint8_t n, uint8_t m)
{
    ron_mat_t h_work;
    ron_mat_t r_work;
    ron_mat_t p_work;
    ron_mat_t gain;
    ron_fault_t fault;

    ron_mat_load(h_work, &kf->cfg.H[0][0], (uint8_t) RON_KF_MAX_STATES, m, n);
    ron_mat_load(r_work, &kf->cfg.R[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, m, m);
    ron_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);

    fault = kf_resolve_gain(kf, h_work, p_work, r_work, gain, n, m);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    kf_apply_innovation(kf, h_work, gain, z, n, m);
    kf_update_cov(kf, h_work, r_work, gain, kf->cfg.use_joseph_form, n, m);

    if (!kf_state_finite(kf, n)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-605, RON-FR-606 | Test: RON-TC-KF-001, RON-TC-KF-004, RON-TC-KF-005, RON-TC-KF-006, RON-TC-KF-007 */
ron_fault_t ron_kf_update(ron_kf_t *kf, const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid)
{
    uint8_t n;
    uint8_t m;

    if (kf == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!kf->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!z_valid) {
        return RON_FAULT_NONE; /* Measurement dropout: skip the update step. */
    }
    if (z == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    n = kf->cfg.n;
    m = kf->cfg.m;

    if (!ron_mat_vec_finite(z, m)) {
        return RON_FAULT_INPUT_NAN;
    }

    return kf_do_update(kf, z, n, m);
}

/* =========================================================================
 * Lifecycle and accessors
 * ========================================================================= */

/* Satisfies: RON-FR-601, RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
static void kf_seed_state(ron_kf_t *kf)
{
    uint8_t n = kf->cfg.n;
    uint8_t i;

    for (i = 0U; i < n; i++) {
        kf->state.x_hat[i] = kf->cfg.x0[i];
    }
    ron_mat_load(kf->state.P, &kf->cfg.P0[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
}

/* Satisfies: RON-FR-600, RON-FR-601, RON-FR-607 | Test: RON-TC-KF-001, RON-TC-KF-002, RON-TC-KF-008 */
ron_fault_t ron_kf_init(ron_kf_t *kf, const ron_kf_config_t *cfg)
{
    ron_fault_t fault;

    if ((kf == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = kf_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    kf->cfg = *cfg;
    kf_seed_state(kf);
    kf->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
ron_fault_t ron_kf_reset(ron_kf_t *kf)
{
    if (kf == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!kf->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    kf_seed_state(kf);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
ron_fault_t ron_kf_get_state(const ron_kf_t *kf, ron_float_t x_hat[RON_KF_MAX_STATES])
{
    uint8_t n;
    uint8_t i;

    if ((kf == NULL) || (x_hat == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!kf->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    n = kf->cfg.n;
    for (i = 0U; i < n; i++) {
        x_hat[i] = kf->state.x_hat[i];
    }

    return RON_FAULT_NONE;
}
