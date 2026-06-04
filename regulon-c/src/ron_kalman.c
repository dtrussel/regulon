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

#if (RON_KF_MAX_MEASUREMENTS > RON_KF_MAX_STATES) || (RON_KF_MAX_INPUTS > RON_KF_MAX_STATES)
#error "ron_kalman requires RON_KF_MAX_STATES >= RON_KF_MAX_MEASUREMENTS and RON_KF_MAX_INPUTS"
#endif

/* Uniform working stride for every internal scratch matrix (n, m, p all <= n_max). */
#define KF_DIM (RON_KF_MAX_STATES)

#define KF_SQRT_STEPS (30U)

typedef ron_float_t kf_mat_t[KF_DIM][KF_DIM];
typedef ron_float_t kf_vec_t[KF_DIM];

/* =========================================================================
 * Bounded scalar helpers (no <math.h>)
 * ========================================================================= */

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002 */
static bool kf_isfinite(ron_float_t value)
{
    return RON_ISFINITE(value);
}

/* Fixed-iteration Newton square root.  Precondition: value > 0. */
/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
static ron_float_t kf_sqrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    for (step = 0U; step < KF_SQRT_STEPS; step++) {
        x = RON_FLOAT_C(0.5) * (x + (value / x));
    }

    return x;
}

/* =========================================================================
 * Bounded matrix / vector helpers (fixed-size loops, no allocation)
 * ========================================================================= */

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008 */
static void kf_mat_load(kf_mat_t dst, const ron_float_t *src, uint8_t src_cols, uint8_t rows,
                        uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            dst[i][j] = src[((size_t) i * (size_t) src_cols) + (size_t) j];
        }
    }
}

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008 */
static void kf_mat_store(ron_float_t *dst, uint8_t dst_cols, kf_mat_t src, uint8_t rows,
                         uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            dst[((size_t) i * (size_t) dst_cols) + (size_t) j] = src[i][j];
        }
    }
}

/* out(lhs_rows x rhs_cols) = lhs(lhs_rows x inner) * rhs(inner x rhs_cols). */
/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
static void kf_mat_mul(kf_mat_t out, kf_mat_t lhs, kf_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                       uint8_t rhs_cols)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < lhs_rows; i++) {
        for (j = 0U; j < rhs_cols; j++) {
            ron_float_t sum = RON_FLOAT_C(0.0);

            for (k = 0U; k < inner; k++) {
                sum += lhs[i][k] * rhs[k][j];
            }
            out[i][j] = sum;
        }
    }
}

/* out(lhs_rows x rhs_rows) = lhs(lhs_rows x inner) * rhs(rhs_rows x inner)^T. */
/* Satisfies: RON-FR-602, RON-FR-604 | Test: RON-TC-KF-003, RON-TC-KF-005 */
static void kf_mat_mul_t(kf_mat_t out, kf_mat_t lhs, kf_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                         uint8_t rhs_rows)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < lhs_rows; i++) {
        for (j = 0U; j < rhs_rows; j++) {
            ron_float_t sum = RON_FLOAT_C(0.0);

            for (k = 0U; k < inner; k++) {
                sum += lhs[i][k] * rhs[j][k];
            }
            out[i][j] = sum;
        }
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
static void kf_mat_add(kf_mat_t out, kf_mat_t lhs, kf_mat_t rhs, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            out[i][j] = lhs[i][j] + rhs[i][j];
        }
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
static void kf_mat_vec(kf_vec_t out, kf_mat_t mat, const ron_float_t *vec, uint8_t rows,
                       uint8_t cols)
{
    uint8_t i;
    uint8_t k;

    for (i = 0U; i < rows; i++) {
        ron_float_t sum = RON_FLOAT_C(0.0);

        for (k = 0U; k < cols; k++) {
            sum += mat[i][k] * vec[k];
        }
        out[i] = sum;
    }
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-KF-003 */
static bool kf_vec_finite(const ron_float_t *vec, uint8_t count)
{
    uint8_t i;

    for (i = 0U; i < count; i++) {
        if (!kf_isfinite(vec[i])) {
            return false;
        }
    }

    return true;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-KF-003 */
static bool kf_strided_finite(const ron_float_t *src, uint8_t src_cols, uint8_t rows, uint8_t cols)
{
    uint8_t i;

    for (i = 0U; i < rows; i++) {
        if (!kf_vec_finite(&src[(size_t) i * (size_t) src_cols], cols)) {
            return false;
        }
    }

    return true;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-008 */
static bool kf_state_finite(const ron_kf_t *kf, uint8_t n)
{
    return kf_vec_finite(&kf->state.x_hat[0], n) &&
           kf_strided_finite(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
}

/*
 * In-place Cholesky factorisation: on success the lower triangle of `mat`
 * holds L with mat == L * L^T.  Returns false if `mat` is not numerically
 * positive definite.
 */
/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
static bool kf_cholesky(kf_mat_t mat, uint8_t dim)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < dim; i++) {
        for (j = 0U; j <= i; j++) {
            ron_float_t sum = mat[i][j];

            for (k = 0U; k < j; k++) {
                sum -= mat[i][k] * mat[j][k];
            }
            if (i == j) {
                if (sum <= RON_FLOAT_C(0.0)) {
                    return false;
                }
                mat[i][j] = kf_sqrt(sum);
            } else {
                mat[i][j] = sum / mat[j][j];
            }
        }
    }

    return true;
}

/* Solve L * L^T * x = rhs in place on `x` (`x` already holds rhs on entry). */
/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
static void kf_chol_solve(kf_mat_t lower, ron_float_t *x, uint8_t dim)
{
    uint8_t i;
    uint8_t k;

    /* Forward substitution: solve L y = rhs. */
    for (i = 0U; i < dim; i++) {
        ron_float_t sum = x[i];

        for (k = 0U; k < i; k++) {
            sum -= lower[i][k] * x[k];
        }
        x[i] = sum / lower[i][i];
    }

    /* Back substitution: solve L^T x = y. */
    for (i = 0U; i < dim; i++) {
        uint8_t row     = (uint8_t) ((dim - 1U) - i);
        ron_float_t sum = x[row];

        for (k = (uint8_t) (row + 1U); k < dim; k++) {
            sum -= lower[k][row] * x[k];
        }
        x[row] = sum / lower[row][row];
    }
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

    if (!kf_strided_finite(&cfg->A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !kf_strided_finite(&cfg->Q[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !kf_strided_finite(&cfg->P0[0][0], (uint8_t) RON_KF_MAX_STATES, n, n) ||
        !kf_strided_finite(&cfg->H[0][0], (uint8_t) RON_KF_MAX_STATES, m, n) ||
        !kf_strided_finite(&cfg->R[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, m, m) ||
        !kf_strided_finite(&cfg->K_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, n, m) ||
        !kf_strided_finite(&cfg->B[0][0], (uint8_t) RON_KF_MAX_INPUTS, n, p) ||
        !kf_vec_finite(&cfg->x0[0], n)) {
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
    kf_mat_t a_work;
    kf_vec_t ax;
    kf_vec_t bu;
    uint8_t i;

    kf_mat_load(a_work, &kf->cfg.A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    kf_mat_vec(ax, a_work, &kf->state.x_hat[0], n, n);

    for (i = 0U; i < n; i++) {
        bu[i] = RON_FLOAT_C(0.0);
    }
    if (p > 0U) {
        kf_mat_t b_work;

        kf_mat_load(b_work, &kf->cfg.B[0][0], (uint8_t) RON_KF_MAX_INPUTS, n, p);
        kf_mat_vec(bu, b_work, u, n, p);
    }

    for (i = 0U; i < n; i++) {
        kf->state.x_hat[i] = ax[i] + bu[i];
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003, RON-TC-KF-006 */
static void kf_predict_cov(ron_kf_t *kf, uint8_t n)
{
    kf_mat_t a_work;
    kf_mat_t p_work;
    kf_mat_t q_work;
    kf_mat_t ap;
    kf_mat_t apat;

    kf_mat_load(a_work, &kf->cfg.A[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    kf_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    kf_mat_load(q_work, &kf->cfg.Q[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);

    kf_mat_mul(ap, a_work, p_work, n, n, n);
    kf_mat_mul_t(apat, ap, a_work, n, n, n);
    kf_mat_add(p_work, apat, q_work, n, n);

    kf_mat_store(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, p_work, n, n);
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
        if (!kf_vec_finite(u, p)) {
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
static ron_fault_t kf_compute_gain(kf_mat_t pht, kf_mat_t s, kf_mat_t gain, uint8_t n, uint8_t m)
{
    kf_mat_t lower;
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

    kf_mat_load(lower, &s[0][0], (uint8_t) KF_DIM, m, m);
    if (!kf_cholesky(lower, m)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    for (i = 0U; i < n; i++) {
        ron_float_t rhs[KF_DIM];

        for (j = 0U; j < m; j++) {
            rhs[j] = pht[i][j];
        }
        kf_chol_solve(lower, rhs, m);
        for (j = 0U; j < m; j++) {
            gain[i][j] = rhs[j];
        }
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-603, RON-FR-606 | Test: RON-TC-KF-004, RON-TC-KF-007 */
static ron_fault_t kf_resolve_gain(const ron_kf_t *kf, kf_mat_t h_work, kf_mat_t p_work,
                                   kf_mat_t r_work, kf_mat_t gain, uint8_t n, uint8_t m)
{
    kf_mat_t hp;
    kf_mat_t s_tmp;
    kf_mat_t s;
    kf_mat_t pht;

    if (kf->cfg.steady_state) {
        kf_mat_load(gain, &kf->cfg.K_inf[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, n, m);
        return RON_FAULT_NONE;
    }

    kf_mat_mul(hp, h_work, p_work, m, n, n);
    kf_mat_mul_t(s_tmp, hp, h_work, m, n, m);
    kf_mat_add(s, s_tmp, r_work, m, m);
    kf_mat_mul_t(pht, p_work, h_work, n, n, m);

    return kf_compute_gain(pht, s, gain, n, m);
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
static void kf_apply_innovation(ron_kf_t *kf, kf_mat_t h_work, kf_mat_t gain, const ron_float_t *z,
                                uint8_t n, uint8_t m)
{
    kf_vec_t hx;
    kf_vec_t innov;
    kf_vec_t dx;
    uint8_t i;

    kf_mat_vec(hx, h_work, &kf->state.x_hat[0], m, n);
    for (i = 0U; i < m; i++) {
        innov[i] = z[i] - hx[i];
    }
    kf_mat_vec(dx, gain, innov, n, m);
    for (i = 0U; i < n; i++) {
        kf->state.x_hat[i] += dx[i];
    }
}

/* Satisfies: RON-FR-604 | Test: RON-TC-KF-004, RON-TC-KF-005 */
static void kf_update_cov(ron_kf_t *kf, kf_mat_t h_work, kf_mat_t r_work, kf_mat_t gain,
                          bool joseph, uint8_t n, uint8_t m)
{
    kf_mat_t p_work;
    kf_mat_t kh;
    kf_mat_t ikh;
    kf_mat_t ikhp;
    uint8_t i;
    uint8_t j;

    kf_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
    kf_mat_mul(kh, gain, h_work, n, m, n);
    for (i = 0U; i < n; i++) {
        for (j = 0U; j < n; j++) {
            ron_float_t eye = (i == j) ? RON_FLOAT_C(1.0) : RON_FLOAT_C(0.0);

            ikh[i][j] = eye - kh[i][j];
        }
    }
    kf_mat_mul(ikhp, ikh, p_work, n, n, n);

    if (joseph) {
        kf_mat_t t1;
        kf_mat_t kr;
        kf_mat_t t2;

        kf_mat_mul_t(t1, ikhp, ikh, n, n, n);
        kf_mat_mul(kr, gain, r_work, n, m, m);
        kf_mat_mul_t(t2, kr, gain, n, m, n);
        kf_mat_add(ikhp, t1, t2, n, n);
    }

    kf_mat_store(&kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, ikhp, n, n);
}

/* Satisfies: RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-606 | Test: RON-TC-KF-001, RON-TC-KF-004, RON-TC-KF-005, RON-TC-KF-007 */
static ron_fault_t kf_do_update(ron_kf_t *kf, const ron_float_t *z, uint8_t n, uint8_t m)
{
    kf_mat_t h_work;
    kf_mat_t r_work;
    kf_mat_t p_work;
    kf_mat_t gain;
    ron_fault_t fault;

    kf_mat_load(h_work, &kf->cfg.H[0][0], (uint8_t) RON_KF_MAX_STATES, m, n);
    kf_mat_load(r_work, &kf->cfg.R[0][0], (uint8_t) RON_KF_MAX_MEASUREMENTS, m, m);
    kf_mat_load(p_work, &kf->state.P[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);

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

    if (!kf_vec_finite(z, m)) {
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
    kf_mat_load(kf->state.P, &kf->cfg.P0[0][0], (uint8_t) RON_KF_MAX_STATES, n, n);
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
