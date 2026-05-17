/*
 * @file     test_ron_kalman.c
 * @brief    Discrete linear Kalman filter unit tests.
 * @module   test_ron_kalman
 * @doc      RON-TP-001
 * @req      RON-FR-600, RON-FR-601, RON-FR-602, RON-FR-603,
 *           RON-FR-604, RON-FR-605, RON-FR-606, RON-FR-607
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_kalman.h"

#include "unity.h"

#define KF_TOL RON_FLOAT_C(0.01)
#define KF_TIGHT_TOL RON_FLOAT_C(0.0001)

void setUp(void)
{
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-SR-021 | Test: RON-TC-KF-003 */
static ron_float_t kf_abs(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? (-value) : value;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002 */
static ron_float_t kf_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002 */
static ron_float_t kf_make_neg_inf(void)
{
    return -kf_make_inf();
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002 */
static ron_float_t kf_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* Deterministic pseudo-random noise in [-0.5, 0.5] (RON-TC-KF-001 seed). */
/* Satisfies: RON-FR-600 | Test: RON-TC-KF-001 */
static ron_float_t kf_noise(uint32_t *seed)
{
    *seed = (*seed * 1103515245U) + 12345U;
    return (RON_FLOAT_C(1.0) / RON_FLOAT_C(4294967296.0)) * (ron_float_t) (*seed) -
           RON_FLOAT_C(0.5);
}

/* Scalar random-walk system A=1, H=1, Q=0.01, R=1, x0=0, P0=10. */
/* Satisfies: RON-FR-600 | Test: RON-TC-KF-001, RON-TC-KF-006, RON-TC-KF-007 */
static void kf_scalar_config(ron_kf_config_t *cfg)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < (uint8_t) RON_KF_MAX_STATES; i++) {
        cfg->x0[i] = RON_FLOAT_C(0.0);
        for (j = 0U; j < (uint8_t) RON_KF_MAX_STATES; j++) {
            cfg->A[i][j]  = RON_FLOAT_C(0.0);
            cfg->Q[i][j]  = RON_FLOAT_C(0.0);
            cfg->P0[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_INPUTS; j++) {
            cfg->B[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_MEASUREMENTS; j++) {
            cfg->K_inf[i][j] = RON_FLOAT_C(0.0);
        }
    }
    for (i = 0U; i < (uint8_t) RON_KF_MAX_MEASUREMENTS; i++) {
        for (j = 0U; j < (uint8_t) RON_KF_MAX_STATES; j++) {
            cfg->H[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_MEASUREMENTS; j++) {
            cfg->R[i][j] = RON_FLOAT_C(0.0);
        }
    }

    cfg->n               = 1U;
    cfg->m               = 1U;
    cfg->p               = 0U;
    cfg->A[0][0]         = RON_FLOAT_C(1.0);
    cfg->H[0][0]         = RON_FLOAT_C(1.0);
    cfg->Q[0][0]         = RON_FLOAT_C(0.01);
    cfg->R[0][0]         = RON_FLOAT_C(1.0);
    cfg->x0[0]           = RON_FLOAT_C(0.0);
    cfg->P0[0][0]        = RON_FLOAT_C(10.0);
    cfg->use_joseph_form = false;
    cfg->steady_state    = false;
}

/* Zeroes every config array; caller fills the active leading block. */
/* Satisfies: RON-FR-601 | Test: RON-TC-KF-002 */
static void kf_zero_config(ron_kf_config_t *cfg)
{
    uint8_t i;
    uint8_t j;

    cfg->n               = 1U;
    cfg->m               = 1U;
    cfg->p               = 0U;
    cfg->use_joseph_form = false;
    cfg->steady_state    = false;
    for (i = 0U; i < (uint8_t) RON_KF_MAX_STATES; i++) {
        cfg->x0[i] = RON_FLOAT_C(0.0);
        for (j = 0U; j < (uint8_t) RON_KF_MAX_STATES; j++) {
            cfg->A[i][j]  = RON_FLOAT_C(0.0);
            cfg->Q[i][j]  = RON_FLOAT_C(0.0);
            cfg->P0[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_INPUTS; j++) {
            cfg->B[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_MEASUREMENTS; j++) {
            cfg->K_inf[i][j] = RON_FLOAT_C(0.0);
        }
    }
    for (i = 0U; i < (uint8_t) RON_KF_MAX_MEASUREMENTS; i++) {
        for (j = 0U; j < (uint8_t) RON_KF_MAX_STATES; j++) {
            cfg->H[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_KF_MAX_MEASUREMENTS; j++) {
            cfg->R[i][j] = RON_FLOAT_C(0.0);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-001 — Scalar state estimation convergence                     */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-600, RON-FR-602 | Test: RON-TC-KF-001 */
void test_ron_tc_kf_001(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    uint32_t seed = 1U;
    ron_float_t x_hat[RON_KF_MAX_STATES];
    ron_float_t p_prev;
    uint16_t cycle;

    kf_scalar_config(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_TRUE(kf.state.is_initialised);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(0.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(10.0)) <= KF_TIGHT_TOL);

    p_prev = kf.state.P[0][0];
    for (cycle = 0U; cycle < 100U; cycle++) {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS];

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
        z[0] = RON_FLOAT_C(5.0) + kf_noise(&seed);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
        if (cycle < 20U) {
            TEST_ASSERT_TRUE(kf.state.P[0][0] < p_prev);
        }
        p_prev = kf.state.P[0][0];
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_get_state(&kf, x_hat));
    TEST_ASSERT_TRUE(kf_abs(x_hat[0] - RON_FLOAT_C(5.0)) < RON_FLOAT_C(0.5));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-002 — Parameter matrices and configuration validation         */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-601 | Test: RON-TC-KF-002 */
void test_ron_tc_kf_002(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;

    kf_scalar_config(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_init(NULL, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_init(&kf, NULL));

    cfg.n = 0U;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    cfg.n = (uint8_t) (RON_KF_MAX_STATES + 1U);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.m = 0U;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    cfg.m = (uint8_t) (RON_KF_MAX_MEASUREMENTS + 1U);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.p = (uint8_t) (RON_KF_MAX_INPUTS + 1U);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));

    kf_scalar_config(&cfg);
    cfg.A[0][0] = kf_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.Q[0][0] = kf_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.P0[0][0] = kf_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.H[0][0] = kf_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.R[0][0] = kf_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.x0[0] = kf_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
    kf_scalar_config(&cfg);
    cfg.K_inf[0][0] = kf_make_neg_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));

    /* A two-state, two-measurement, one-input configuration initialises and
     * seeds state from x0 / P0. */
    kf_zero_config(&cfg);
    cfg.n        = 2U;
    cfg.m        = 2U;
    cfg.p        = 1U;
    cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.B[0][0]  = RON_FLOAT_C(0.5);
    cfg.B[1][0]  = RON_FLOAT_C(0.25);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.H[1][1]  = RON_FLOAT_C(1.0);
    cfg.Q[0][0]  = RON_FLOAT_C(0.01);
    cfg.Q[1][1]  = RON_FLOAT_C(0.01);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.R[1][1]  = RON_FLOAT_C(2.0);
    cfg.x0[0]    = RON_FLOAT_C(3.0);
    cfg.x0[1]    = RON_FLOAT_C(-1.0);
    cfg.P0[0][0] = RON_FLOAT_C(5.0);
    cfg.P0[1][1] = RON_FLOAT_C(7.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_TRUE(kf.state.is_initialised);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(3.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(-1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(5.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][1] - RON_FLOAT_C(7.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][1] - RON_FLOAT_C(0.0)) <= KF_TIGHT_TOL);

    /* Non-finite input matrix in the input-bearing config is also rejected. */
    cfg.B[0][0] = kf_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_init(&kf, &cfg));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-003 — Predict / update algorithm correctness                  */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
void test_ron_tc_kf_003(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    ron_float_t x_hat[RON_KF_MAX_STATES];
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];

    /* Scalar example: A=2, H=1, Q=0, R=1, x0=1, P0=1.  Hand-checked. */
    kf_zero_config(&cfg);
    cfg.n        = 1U;
    cfg.m        = 1U;
    cfg.p        = 0U;
    cfg.A[0][0]  = RON_FLOAT_C(2.0);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.x0[0]    = RON_FLOAT_C(1.0);
    cfg.P0[0][0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_get_state(&kf, x_hat));
    TEST_ASSERT_TRUE(kf_abs(x_hat[0] - RON_FLOAT_C(2.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(4.0)) <= KF_TIGHT_TOL);

    z[0] = RON_FLOAT_C(3.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_get_state(&kf, x_hat));
    TEST_ASSERT_TRUE(kf_abs(x_hat[0] - RON_FLOAT_C(2.8)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(0.8)) <= KF_TIGHT_TOL);

    /* ron_kf_reset re-seeds x_hat and P from the stored configuration. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_reset(&kf));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);

    /* Two-state constant-velocity example.  Hand-checked. */
    kf_zero_config(&cfg);
    cfg.n        = 2U;
    cfg.m        = 1U;
    cfg.p        = 0U;
    cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.A[0][1]  = RON_FLOAT_C(1.0);
    cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.x0[0]    = RON_FLOAT_C(0.0);
    cfg.x0[1]    = RON_FLOAT_C(1.0);
    cfg.P0[0][0] = RON_FLOAT_C(1.0);
    cfg.P0[1][1] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(2.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][1] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][0] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][1] - RON_FLOAT_C(1.0)) <= KF_TIGHT_TOL);

    z[0] = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - (RON_FLOAT_C(5.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - (RON_FLOAT_C(4.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - (RON_FLOAT_C(2.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][1] - (RON_FLOAT_C(1.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][0] - (RON_FLOAT_C(1.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][1] - (RON_FLOAT_C(2.0) / RON_FLOAT_C(3.0))) <= KF_TOL);

    /* Dropout update is a no-op even on a multi-state instance. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, false));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - (RON_FLOAT_C(5.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-004 — Cholesky-based innovation solve (m > 1)                  */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
static void kf_two_meas_config(ron_kf_config_t *cfg)
{
    kf_zero_config(cfg);
    cfg->n       = 2U;
    cfg->m       = 2U;
    cfg->p       = 0U;
    cfg->A[0][0] = RON_FLOAT_C(1.0);
    cfg->A[1][1] = RON_FLOAT_C(1.0);
    cfg->H[0][0] = RON_FLOAT_C(1.0);
    cfg->H[1][1] = RON_FLOAT_C(1.0);
}

/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
void test_ron_tc_kf_004(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];

    /* Diagonal innovation covariance: K = P (S^-1). */
    kf_two_meas_config(&cfg);
    cfg.R[0][0]  = RON_FLOAT_C(2.0);
    cfg.R[1][1]  = RON_FLOAT_C(3.0);
    cfg.P0[0][0] = RON_FLOAT_C(4.0);
    cfg.P0[1][1] = RON_FLOAT_C(9.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    z[0] = RON_FLOAT_C(1.0);
    z[1] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - (RON_FLOAT_C(2.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(0.75)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - (RON_FLOAT_C(4.0) / RON_FLOAT_C(3.0))) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][1] - RON_FLOAT_C(2.25)) <= KF_TOL);

    /* Non-diagonal innovation covariance exercises the full Cholesky solve. */
    kf_two_meas_config(&cfg);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.R[1][1]  = RON_FLOAT_C(1.0);
    cfg.P0[0][0] = RON_FLOAT_C(2.0);
    cfg.P0[0][1] = RON_FLOAT_C(1.0);
    cfg.P0[1][0] = RON_FLOAT_C(1.0);
    cfg.P0[1][1] = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    z[0] = RON_FLOAT_C(1.0);
    z[1] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(0.625)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(0.125)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - RON_FLOAT_C(0.625)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][1] - RON_FLOAT_C(0.125)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][0] - RON_FLOAT_C(0.125)) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.P[1][1] - RON_FLOAT_C(0.625)) <= KF_TOL);

    /* Non-positive-definite innovation covariance (m > 1) is rejected and the
     * state is left unchanged. */
    kf_two_meas_config(&cfg);
    cfg.R[0][0] = RON_FLOAT_C(1.0);
    cfg.R[0][1] = RON_FLOAT_C(2.0);
    cfg.R[1][0] = RON_FLOAT_C(2.0);
    cfg.R[1][1] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    z[0] = RON_FLOAT_C(1.0);
    z[1] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_update(&kf, z, true));
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(0.0)) <= KF_TIGHT_TOL);
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(0.0)) <= KF_TIGHT_TOL);

    /* Degenerate scalar innovation covariance (m == 1, S <= 0) is rejected. */
    kf_zero_config(&cfg);
    cfg.n       = 1U;
    cfg.m       = 1U;
    cfg.p       = 0U;
    cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.H[0][0] = RON_FLOAT_C(1.0);
    cfg.R[0][0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    z[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_update(&kf, z, true));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-005 — Joseph-form covariance update                           */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-604 | Test: RON-TC-KF-005 */
void test_ron_tc_kf_005(void)
{
    ron_kf_t std_kf;
    ron_kf_t jos_kf;
    ron_kf_config_t cfg;
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];
    uint32_t seed_a = 7U;
    uint32_t seed_b = 7U;
    uint16_t cycle;

    /* Same two-state model run with and without Joseph form: identical
     * estimate, identical covariance, symmetric covariance. */
    kf_zero_config(&cfg);
    cfg.n        = 2U;
    cfg.m        = 1U;
    cfg.p        = 0U;
    cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.A[0][1]  = RON_FLOAT_C(1.0);
    cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.Q[0][0]  = RON_FLOAT_C(0.01);
    cfg.Q[1][1]  = RON_FLOAT_C(0.01);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.x0[0]    = RON_FLOAT_C(0.0);
    cfg.x0[1]    = RON_FLOAT_C(1.0);
    cfg.P0[0][0] = RON_FLOAT_C(1.0);
    cfg.P0[1][1] = RON_FLOAT_C(1.0);

    cfg.use_joseph_form = false;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&std_kf, &cfg));
    cfg.use_joseph_form = true;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&jos_kf, &cfg));

    for (cycle = 0U; cycle < 30U; cycle++) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&std_kf, NULL));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&jos_kf, NULL));
        z[0] = RON_FLOAT_C(2.0) + kf_noise(&seed_a);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&std_kf, z, true));
        z[0] = RON_FLOAT_C(2.0) + kf_noise(&seed_b);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&jos_kf, z, true));
    }

    TEST_ASSERT_TRUE(kf_abs(std_kf.state.x_hat[0] - jos_kf.state.x_hat[0]) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(std_kf.state.x_hat[1] - jos_kf.state.x_hat[1]) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(std_kf.state.P[0][0] - jos_kf.state.P[0][0]) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(std_kf.state.P[0][1] - jos_kf.state.P[0][1]) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(std_kf.state.P[1][0] - jos_kf.state.P[1][0]) <= KF_TOL);
    TEST_ASSERT_TRUE(kf_abs(std_kf.state.P[1][1] - jos_kf.state.P[1][1]) <= KF_TOL);
    /* Joseph-form covariance stays symmetric. */
    TEST_ASSERT_TRUE(kf_abs(jos_kf.state.P[0][1] - jos_kf.state.P[1][0]) <= KF_TIGHT_TOL);

    /* Joseph form with multiple measurements also produces a sane update. */
    kf_two_meas_config(&cfg);
    cfg.use_joseph_form = true;
    cfg.R[0][0]         = RON_FLOAT_C(1.0);
    cfg.R[1][1]         = RON_FLOAT_C(1.0);
    cfg.P0[0][0]        = RON_FLOAT_C(2.0);
    cfg.P0[0][1]        = RON_FLOAT_C(1.0);
    cfg.P0[1][0]        = RON_FLOAT_C(1.0);
    cfg.P0[1][1]        = RON_FLOAT_C(2.0);
    cfg.x0[0]           = RON_FLOAT_C(0.0);
    cfg.x0[1]           = RON_FLOAT_C(0.0);
    {
        ron_kf_t kf;

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
        z[0] = RON_FLOAT_C(1.0);
        z[1] = RON_FLOAT_C(0.0);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
        TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(0.625)) <= KF_TOL);
        TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[1] - RON_FLOAT_C(0.125)) <= KF_TOL);
        TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][0] - kf.state.P[1][1]) <= KF_TIGHT_TOL);
        TEST_ASSERT_TRUE(kf_abs(kf.state.P[0][1] - kf.state.P[1][0]) <= KF_TIGHT_TOL);
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-006 — Measurement dropout                                     */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-605 | Test: RON-TC-KF-006 */
void test_ron_tc_kf_006(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    uint32_t seed = 3U;
    ron_float_t x_converged;
    ron_float_t p_prev;
    uint16_t cycle;

    kf_scalar_config(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    for (cycle = 0U; cycle < 50U; cycle++) {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS];

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
        z[0] = RON_FLOAT_C(5.0) + kf_noise(&seed);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    }

    x_converged = kf.state.x_hat[0];
    p_prev      = kf.state.P[0][0];
    for (cycle = 0U; cycle < 10U; cycle++) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
        /* Dropout: no measurement is consumed; the pointer is not read. */
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, NULL, false));
        TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - x_converged) <= KF_TIGHT_TOL);
        TEST_ASSERT_TRUE(kf.state.P[0][0] > p_prev);
        p_prev = kf.state.P[0][0];
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-007 — Steady-state (fixed-gain) mode                          */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-606 | Test: RON-TC-KF-007 */
void test_ron_tc_kf_007(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    const ron_float_t k_inf = RON_FLOAT_C(0.0951249);
    ron_float_t x_prev;
    uint16_t cycle;

    kf_scalar_config(&cfg);
    cfg.steady_state = true;
    cfg.K_inf[0][0]  = k_inf;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
    {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(5.0)};

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    }
    /* After one fixed-gain update the estimate moved by exactly K_inf * z. */
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - (k_inf * RON_FLOAT_C(5.0))) <= KF_TIGHT_TOL);

    x_prev = kf.state.x_hat[0];
    for (cycle = 0U; cycle < 100U; cycle++) {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(5.0)};

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, NULL));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
        TEST_ASSERT_TRUE(kf.state.x_hat[0] >= x_prev);
        x_prev = kf.state.x_hat[0];
    }
    TEST_ASSERT_TRUE(kf_abs(kf.state.x_hat[0] - RON_FLOAT_C(5.0)) < RON_FLOAT_C(0.5));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-KF-008 — Bounded fixed storage, defensive paths, finite checks   */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008 */
static void kf_max_dim_config(ron_kf_config_t *cfg)
{
    uint8_t i;

    kf_zero_config(cfg);
    cfg->n = (uint8_t) RON_KF_MAX_STATES;
    cfg->m = (uint8_t) RON_KF_MAX_MEASUREMENTS;
    cfg->p = (uint8_t) RON_KF_MAX_INPUTS;
    for (i = 0U; i < (uint8_t) RON_KF_MAX_STATES; i++) {
        cfg->A[i][i]  = RON_FLOAT_C(1.0);
        cfg->Q[i][i]  = RON_FLOAT_C(0.01);
        cfg->P0[i][i] = RON_FLOAT_C(1.0);
        cfg->x0[i]    = RON_FLOAT_C(0.0);
    }
    for (i = 0U; i < (uint8_t) RON_KF_MAX_INPUTS; i++) {
        cfg->B[i][i] = RON_FLOAT_C(0.1);
    }
    for (i = 0U; i < (uint8_t) RON_KF_MAX_MEASUREMENTS; i++) {
        cfg->H[i][i] = RON_FLOAT_C(1.0);
        cfg->R[i][i] = RON_FLOAT_C(1.0);
    }
}

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008 */
void test_ron_tc_kf_008(void)
{
    ron_kf_t kf;
    ron_kf_t fresh;
    ron_kf_config_t cfg;
    ron_float_t x_hat[RON_KF_MAX_STATES];
    ron_float_t u[RON_KF_MAX_INPUTS];
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];
    ron_fault_t fault;
    uint8_t i;
    uint16_t cycle;

    /* Maximum-dimension filter runs with caller-owned fixed storage only. */
    kf_max_dim_config(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    for (i = 0U; i < (uint8_t) RON_KF_MAX_INPUTS; i++) {
        u[i] = RON_FLOAT_C(1.0);
    }
    for (i = 0U; i < (uint8_t) RON_KF_MAX_MEASUREMENTS; i++) {
        z[i] = RON_FLOAT_C(0.5);
    }
    for (cycle = 0U; cycle < 5U; cycle++) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_predict(&kf, u));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, z, true));
    }
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_get_state(&kf, x_hat));
    for (i = 0U; i < (uint8_t) RON_KF_MAX_STATES; i++) {
        TEST_ASSERT_TRUE(RON_ISFINITE(x_hat[i]));
    }

    /* Null-pointer rejection on every entry point. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_predict(NULL, u));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_update(NULL, z, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_get_state(NULL, x_hat));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_get_state(&kf, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_update(&kf, NULL, true));
    /* p > 0 requires a non-NULL control-input vector. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_predict(&kf, NULL));

    /* Operations on an uninitialised filter are rejected. */
    fresh.state.is_initialised = false;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_predict(&fresh, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_update(&fresh, z, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_update(&fresh, z, false));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_reset(&fresh));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_kf_get_state(&fresh, x_hat));

    /* Non-finite inputs are rejected; dropout still ignores the pointer. */
    for (i = 0U; i < (uint8_t) RON_KF_MAX_INPUTS; i++) {
        u[i] = RON_FLOAT_C(1.0);
    }
    u[0] = kf_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_kf_predict(&kf, u));
    z[0] = kf_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_kf_update(&kf, z, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_update(&kf, NULL, false));

    /* Non-finite estimate / covariance is detected and reported. */
    kf_zero_config(&cfg);
    cfg.n        = 1U;
    cfg.m        = 1U;
    cfg.p        = 0U;
    cfg.A[0][0]  = RON_FLOAT_C(1.0e30);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.x0[0]    = RON_FLOAT_C(1.0);
    cfg.P0[0][0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_kf_init(&kf, &cfg));
    fault = RON_FAULT_NONE;
    for (cycle = 0U; (cycle < 40U) && (fault == RON_FAULT_NONE); cycle++) {
        fault = ron_kf_predict(&kf, NULL);
    }
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_OUTPUT_NAN, fault);
    z[0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_OUTPUT_NAN, ron_kf_update(&kf, z, true));
}

/* ----------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_kf_001);
    RUN_TEST(test_ron_tc_kf_002);
    RUN_TEST(test_ron_tc_kf_003);
    RUN_TEST(test_ron_tc_kf_004);
    RUN_TEST(test_ron_tc_kf_005);
    RUN_TEST(test_ron_tc_kf_006);
    RUN_TEST(test_ron_tc_kf_007);
    RUN_TEST(test_ron_tc_kf_008);
    return UNITY_END();
}
