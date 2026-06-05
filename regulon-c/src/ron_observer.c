/*
 * @file     ron_observer.c
 * @brief    Discrete-time Luenberger state observer predict/update recursion.
 * @module   ron_observer
 * @doc      RON-IS-001
 * @req      RON-FR-720, RON-FR-721, RON-FR-722, RON-FR-723
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_observer.h"

#include "ron_matrix_internal.h"

#if (RON_SS_MAX_STATES > RON_MAT_MAX_DIM) || (RON_SS_MAX_OUTPUTS > RON_MAT_MAX_DIM) ||             \
    (RON_SS_MAX_INPUTS > RON_MAT_MAX_DIM)
#error "ron_observer requires RON_MAT_MAX_DIM >= RON_SS_MAX_{STATES,OUTPUTS,INPUTS}"
#endif

/* =========================================================================
 * Configuration validation
 * ========================================================================= */

/* Satisfies: RON-FR-723 | Test: RON-TC-SS-009 */
static bool obs_dims_valid(const ron_obs_config_t *cfg)
{
    if ((cfg->n < 1U) || (cfg->n > (uint8_t) RON_SS_MAX_STATES)) {
        return false;
    }
    if ((cfg->m < 1U) || (cfg->m > (uint8_t) RON_SS_MAX_OUTPUTS)) {
        return false;
    }
    if (cfg->p > (uint8_t) RON_SS_MAX_INPUTS) {
        return false;
    }

    return true;
}

/* Satisfies: RON-FR-721 | Test: RON-TC-SS-007 */
static ron_fault_t obs_validate_config(const ron_obs_config_t *cfg)
{
    uint8_t n;
    uint8_t m;
    uint8_t p;

    if (!obs_dims_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    n = cfg->n;
    m = cfg->m;
    p = cfg->p;

    if (!ron_mat_strided_finite(&cfg->A[0][0], (uint8_t) RON_SS_MAX_STATES, n, n) ||
        !ron_mat_strided_finite(&cfg->C[0][0], (uint8_t) RON_SS_MAX_STATES, m, n) ||
        !ron_mat_strided_finite(&cfg->L[0][0], (uint8_t) RON_SS_MAX_OUTPUTS, n, m) ||
        !ron_mat_strided_finite(&cfg->B[0][0], (uint8_t) RON_SS_MAX_INPUTS, n, p) ||
        !ron_mat_vec_finite(&cfg->x0[0], n)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Observer step (RON-FR-720)
 *
 *   innovation = y - C x_hat
 *   x_hat      = A x_hat + B u + L innovation
 * ========================================================================= */

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006 */
static void obs_advance(ron_obs_t *obs, const ron_float_t *y, const ron_float_t *u, uint8_t n,
                        uint8_t m, uint8_t p)
{
    ron_mat_t work;
    ron_vec_t cx;
    ron_vec_t innov;
    ron_vec_t ax;
    ron_vec_t li;
    ron_vec_t bu;
    uint8_t i;

    /* innovation = y - C x_hat */
    ron_mat_load(work, &obs->cfg.C[0][0], (uint8_t) RON_SS_MAX_STATES, m, n);
    ron_mat_vec(cx, work, &obs->state.x_hat[0], m, n);
    for (i = 0U; i < m; i++) {
        innov[i] = y[i] - cx[i];
    }

    /* ax = A x_hat */
    ron_mat_load(work, &obs->cfg.A[0][0], (uint8_t) RON_SS_MAX_STATES, n, n);
    ron_mat_vec(ax, work, &obs->state.x_hat[0], n, n);

    /* li = L innovation */
    ron_mat_load(work, &obs->cfg.L[0][0], (uint8_t) RON_SS_MAX_OUTPUTS, n, m);
    ron_mat_vec(li, work, innov, n, m);

    /* bu = B u (zero when there is no control input) */
    for (i = 0U; i < n; i++) {
        bu[i] = RON_FLOAT_C(0.0);
    }
    if (p > 0U) {
        ron_mat_load(work, &obs->cfg.B[0][0], (uint8_t) RON_SS_MAX_INPUTS, n, p);
        ron_mat_vec(bu, work, u, n, p);
    }

    for (i = 0U; i < n; i++) {
        obs->state.x_hat[i] = ax[i] + bu[i] + li[i];
    }
}

/* Satisfies: RON-FR-720, RON-FR-721 | Test: RON-TC-SS-006, RON-TC-SS-007 */
ron_fault_t ron_obs_step(ron_obs_t *obs, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                         const ron_float_t u[RON_SS_MAX_INPUTS])
{
    uint8_t n;
    uint8_t m;
    uint8_t p;

    if (obs == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!obs->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (y == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    n = obs->cfg.n;
    m = obs->cfg.m;
    p = obs->cfg.p;

    if (!ron_mat_vec_finite(y, m)) {
        return RON_FAULT_INPUT_NAN;
    }
    if (p > 0U) {
        if (u == NULL) {
            return RON_FAULT_NULL_POINTER;
        }
        if (!ron_mat_vec_finite(u, p)) {
            return RON_FAULT_INPUT_NAN;
        }
    }

    obs_advance(obs, y, u, n, m, p);

    if (!ron_mat_vec_finite(&obs->state.x_hat[0], n)) {
        return RON_FAULT_OUTPUT_NAN;
    }

    return RON_FAULT_NONE;
}

/* =========================================================================
 * Lifecycle and accessors
 * ========================================================================= */

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006 */
static void obs_seed_state(ron_obs_t *obs)
{
    uint8_t n = obs->cfg.n;
    uint8_t i;

    for (i = 0U; i < n; i++) {
        obs->state.x_hat[i] = obs->cfg.x0[i];
    }
}

/* Satisfies: RON-FR-721, RON-FR-723 | Test: RON-TC-SS-007, RON-TC-SS-009 */
ron_fault_t ron_obs_init(ron_obs_t *obs, const ron_obs_config_t *cfg)
{
    ron_fault_t fault;

    if ((obs == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = obs_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    obs->cfg = *cfg;
    obs_seed_state(obs);
    obs->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006 */
ron_fault_t ron_obs_reset(ron_obs_t *obs)
{
    if (obs == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!obs->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    obs_seed_state(obs);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-722 | Test: RON-TC-SS-008 */
ron_fault_t ron_obs_get_state(const ron_obs_t *obs, ron_float_t x_hat[RON_SS_MAX_STATES])
{
    uint8_t n;
    uint8_t i;

    if ((obs == NULL) || (x_hat == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!obs->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    n = obs->cfg.n;
    for (i = 0U; i < n; i++) {
        x_hat[i] = obs->state.x_hat[i];
    }

    return RON_FAULT_NONE;
}
