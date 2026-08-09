/*
 * @file     test_ron_lqr.c
 * @brief    Discrete-time MIMO LQR controller unit tests.
 * @module   test_ron_lqr
 * @doc      RON-TP-001
 * @req      RON-FR-730, RON-FR-731, RON-FR-732, RON-FR-733, RON-FR-734,
 *           RON-FR-735, RON-FR-736, RON-FR-737, RON-FR-738, RON-FR-739
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_lqr.h"

#include "unity.h"

#define LQR_TOL RON_FLOAT_C(0.0001)

void setUp(void)
{
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-SR-020 | Test: RON-TC-LQR-006 */
static ron_float_t lqr_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-LQR-006 */
static ron_float_t lqr_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* 2-state, 1-input external-source config with pre-computed gain, wide
 * limits and rate limiting disabled. */
static ron_lqr_config_t make_ext_cfg(uint8_t n, uint8_t m)
{
    ron_lqr_config_t cfg = {0};
    /* m may deliberately exceed RON_LQR_MAX_INPUTS in dimension-validation
     * tests; never write past the fixed-size limit arrays in that case. */
    uint8_t fill_m = (m > (uint8_t) RON_LQR_MAX_INPUTS) ? (uint8_t) RON_LQR_MAX_INPUTS : m;
    uint8_t j;

    cfg.n         = n;
    cfg.m         = m;
    cfg.source    = RON_LQR_SOURCE_EXTERNAL;
    cfg.gain_mode = RON_LQR_GAIN_PRECOMPUTED;
    for (j = 0U; j < fill_m; j++) {
        cfg.u_min[j] = RON_FLOAT_C(-1000.0);
        cfg.u_max[j] = RON_FLOAT_C(1000.0);
    }

    return cfg;
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-001 — Init and Basic Control Law (Pre-computed Gain)         */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-001 | RON-FR-730, RON-FR-732 */
void test_ron_tc_lqr_001(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = make_ext_cfg(2U, 1U);
    ron_float_t x_ext[2]              = {RON_FLOAT_C(0.5), RON_FLOAT_C(0.2)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.x_ext   = x_ext;
    cfg.K[0][0] = RON_FLOAT_C(2.0);
    cfg.K[0][1] = RON_FLOAT_C(1.0);
    cfg.Kr[0]   = RON_FLOAT_C(1.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    /* u = -(K[0]*x[0] + K[1]*x[1]) + Kr[0]*r[0] = -(1.0+0.2) + 1.0 = -0.2. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(-0.2), u[0]);
    TEST_ASSERT_EQUAL(RON_STATUS_OK, status);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-002 — External State Source                                  */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-002 | RON-FR-730, RON-FR-734 */
void test_ron_tc_lqr_002(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = make_ext_cfg(2U, 1U);
    ron_float_t x_ext[2]              = {RON_FLOAT_C(1.0), RON_FLOAT_C(2.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.x_ext   = x_ext;
    cfg.K[0][0] = RON_FLOAT_C(2.0);
    cfg.K[0][1] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    /* u = -(2*1 + 1*2) = -4. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(-4.0), u[0]);

    /* Mutate the external state; the next step reflects the new value. */
    x_ext[0] = RON_FLOAT_C(3.0);
    x_ext[1] = RON_FLOAT_C(4.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(-10.0), u[0]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-003 — DARE Solver Convergence                                */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-003 | RON-FR-731, RON-FR-733, RON-FR-739 */
void test_ron_tc_lqr_003(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg = {0};
    ron_float_t x_ext[2] = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
    ron_float_t p[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];
    uint8_t i;
    uint8_t j;

    cfg.n             = 2U;
    cfg.m             = 1U;
    cfg.source        = RON_LQR_SOURCE_EXTERNAL;
    cfg.x_ext         = x_ext;
    cfg.gain_mode     = RON_LQR_GAIN_DARE;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.A[0][1]       = RON_FLOAT_C(1.0);
    cfg.A[1][1]       = RON_FLOAT_C(1.0);
    cfg.B[1][0]       = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1]  = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.dare_max_iter = 200U;
    cfg.dare_tol      = RON_FLOAT_C(1e-4);
    cfg.u_min[0]      = RON_FLOAT_C(-1000.0);
    cfg.u_max[0]      = RON_FLOAT_C(1000.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    TEST_ASSERT_TRUE(lqr.state.dare_converged);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_get_dare_solution(&lqr, p));

    /* P must be symmetric and positive semi-definite (diagonal >= 0 as a
     * necessary condition; full symmetry checked directly). */
    for (i = 0U; i < 2U; i++) {
        TEST_ASSERT_TRUE(p[i][i] >= RON_FLOAT_C(0.0));
        for (j = 0U; j < 2U; j++) {
            TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.001), p[i][j], p[j][i]);
        }
    }

    /* Riccati fixed point: K must satisfy P = Q + A'PA - A'PB*K (checked
     * indirectly via the closed-loop being stable: repeated stepping from a
     * nonzero state converges toward zero). */
    {
        ron_float_t x0[2]                 = {RON_FLOAT_C(5.0), RON_FLOAT_C(0.0)};
        ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;
        uint16_t step;

        cfg.x_ext = x0;
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

        for (step = 0U; step < 50U; step++) {
            ron_float_t x_next0;
            ron_float_t x_next1;

            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status));
            x_next0 = x0[0] + x0[1];
            x_next1 = x0[1] + u[0];
            x0[0]   = x_next0;
            x0[1]   = x_next1;
        }
        TEST_ASSERT_TRUE(ron_fabs(x0[0]) < RON_FLOAT_C(0.5));
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-004 — Per-Input Output Saturation                            */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-004 | RON-FR-736 */
void test_ron_tc_lqr_004(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = make_ext_cfg(2U, 1U);
    ron_float_t x_ext[2]              = {RON_FLOAT_C(-5.0), RON_FLOAT_C(0.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.x_ext    = x_ext;
    cfg.K[0][0]  = RON_FLOAT_C(1.0); /* u_raw = 5.0 -> saturate to 1.0 */
    cfg.u_min[0] = RON_FLOAT_C(-1.0);
    cfg.u_max[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(1.0), u[0]);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-005 — Runtime Gain Update                                    */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-005 | RON-FR-738 */
void test_ron_tc_lqr_005(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg = make_ext_cfg(2U, 1U);
    ron_float_t x_ext[2] = {RON_FLOAT_C(3.0), RON_FLOAT_C(4.0)};
    /* K/Kr are 2-D array parameters; the local arguments must already be
     * const-qualified (ISO C forbids implicitly adding a const qualifier at
     * a nested array level before C23). */
    const ron_float_t new_k[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES] = {
        {RON_FLOAT_C(0.0), RON_FLOAT_C(2.0)}};
    const ron_float_t new_kr[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS]            = {RON_FLOAT_C(1.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.x_ext   = x_ext;
    cfg.K[0][0] = RON_FLOAT_C(1.0);
    cfg.K[0][1] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    r[0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(-3.0), u[0]);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_set_gains(&lqr, new_k, new_kr));

    r[0] = RON_FLOAT_C(1.0);
    /* u = -(0*3 + 2*4) + 1*1 = -7. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(-7.0), u[0]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-006 — Fault Detection: Null Pointer and Uninitialised        */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-006 | RON-FR-736, RON-SR-001 */
void test_ron_tc_lqr_006(void)
{
    ron_lqr_t lqr;
    ron_lqr_t fresh;
    ron_lqr_config_t cfg              = make_ext_cfg(1U, 1U);
    ron_float_t x_ext[1]              = {RON_FLOAT_C(0.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    fresh.state.is_initialised = false;
    cfg.x_ext                  = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    /* (a) init with lqr == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_init(NULL, &cfg));
    /* (b) step with lqr == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_step(NULL, r, RON_FLOAT_C(0.01), u, &status));
    /* (c) step on uninitialised instance. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID,
                      ron_lqr_step(&fresh, r, RON_FLOAT_C(0.01), u, &status));
    /* (d) step with u == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), NULL, &status));

    /* Additional defensive paths exercised for coverage. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_init(&lqr, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_lqr_step(&lqr, NULL, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_lqr_step(&lqr, r, lqr_make_nan(), u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.0), u, &status));
    {
        ron_float_t bad_r[RON_LQR_MAX_INPUTS] = {lqr_make_inf()};

        TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                          ron_lqr_step(&lqr, bad_r, RON_FLOAT_C(0.01), u, &status));
    }
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_reset(NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_reset(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_get_dare_solution(NULL, NULL));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-007 — Integral Augmentation: Steady-State Tracking           */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-007 | RON-FR-735 */
void test_ron_tc_lqr_007(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = {0};
    ron_float_t x[1]                  = {RON_FLOAT_C(0.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(2.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;
    uint16_t step;

    /* First-order integrator plant x(k+1) = x(k) + u(k)*dt with PI-style
     * feedback: proportional K provides damping (integral action alone on a
     * bare integrator plant is only marginally stable/undamped) while the
     * integral term drives the steady-state error to zero through
     * C_out = 1. */
    cfg.n            = 1U;
    cfg.m            = 1U;
    cfg.source       = RON_LQR_SOURCE_EXTERNAL;
    cfg.x_ext        = x;
    cfg.gain_mode    = RON_LQR_GAIN_PRECOMPUTED;
    cfg.K[0][0]      = RON_FLOAT_C(2.0); /* u_fb = -K*x_hat: negative feedback for damping */
    cfg.use_integral = true;
    cfg.Ki_aug[0]    = RON_FLOAT_C(0.8);
    cfg.C_out[0][0]  = RON_FLOAT_C(1.0);
    cfg.i_min[0]     = RON_FLOAT_C(-100.0);
    cfg.i_max[0]     = RON_FLOAT_C(100.0);
    cfg.u_min[0]     = RON_FLOAT_C(-100.0);
    cfg.u_max[0]     = RON_FLOAT_C(100.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    for (step = 0U; step < 500U; step++) {
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.05), u, &status));
        x[0] += u[0] * RON_FLOAT_C(0.05);
        TEST_ASSERT_TRUE(lqr.state.integral[0] >= cfg.i_min[0]);
        TEST_ASSERT_TRUE(lqr.state.integral[0] <= cfg.i_max[0]);
    }

    TEST_ASSERT_TRUE(ron_fabs(r[0] - x[0]) < RON_FLOAT_C(0.01));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-008 — Luenberger Observer Source (Integration)               */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-008 | RON-FR-734 */
void test_ron_tc_lqr_008(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = {0};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    /* The Kalman entry points declare their measurement vector as
     * [RON_KF_MAX_MEASUREMENTS], which need not match the observer's output
     * bound, so give them their own correctly sized buffer. */
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(0.0)};
    ron_status_t status;
    uint16_t step;

    cfg.n               = 2U;
    cfg.m               = 1U;
    cfg.source          = RON_LQR_SOURCE_LUENBERGER;
    cfg.gain_mode       = RON_LQR_GAIN_PRECOMPUTED;
    cfg.K[0][0]         = RON_FLOAT_C(0.5);
    cfg.K[0][1]         = RON_FLOAT_C(0.5);
    cfg.A[0][0]         = RON_FLOAT_C(1.0);
    cfg.A[0][1]         = RON_FLOAT_C(1.0);
    cfg.A[1][1]         = RON_FLOAT_C(1.0);
    cfg.B[1][0]         = RON_FLOAT_C(1.0);
    cfg.obs_cfg.n       = 2U;
    cfg.obs_cfg.m       = 1U;
    cfg.obs_cfg.p       = 1U;
    cfg.obs_cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.A[0][1] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.A[1][1] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.B[1][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.u_min[0]        = RON_FLOAT_C(-1000.0);
    cfg.u_max[0]        = RON_FLOAT_C(1000.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    for (step = 0U; step < 200U; step++) {
        ron_float_t prev_u[RON_SS_MAX_INPUTS] = {u[0]};

        y[0] = RON_FLOAT_C(1.0);
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_observer_step(&lqr, y, prev_u));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
        TEST_ASSERT_TRUE(u[0] >= cfg.u_min[0]);
        TEST_ASSERT_TRUE(u[0] <= cfg.u_max[0]);
    }

    /* Cross-source estimator calls are rejected. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_kalman_predict(&lqr, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_kalman_update(&lqr, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_observer_step(NULL, y, NULL));

    /* Reset on a LUENBERGER-source instance also resets the observer. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_reset(&lqr));

    /* Matching dims but an internally invalid embedded observer config is
     * rejected by ron_obs_init and propagated through ron_lqr_init. */
    {
        ron_lqr_t bad;
        ron_lqr_config_t bad_cfg = cfg;

        bad_cfg.obs_cfg.A[0][0] = lqr_make_inf();
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&bad, &bad_cfg));
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQR-009 — Kalman Filter Source (Integration)                     */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQR-009 | RON-FR-734 */
void test_ron_tc_lqr_009(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = {0};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];
    ron_status_t status;
    uint16_t step;

    cfg.n               = 2U;
    cfg.m               = 1U;
    cfg.source          = RON_LQR_SOURCE_KALMAN;
    cfg.gain_mode       = RON_LQR_GAIN_PRECOMPUTED;
    cfg.K[0][0]         = RON_FLOAT_C(0.5);
    cfg.K[0][1]         = RON_FLOAT_C(0.5);
    cfg.A[0][0]         = RON_FLOAT_C(1.0);
    cfg.A[0][1]         = RON_FLOAT_C(1.0);
    cfg.A[1][1]         = RON_FLOAT_C(1.0);
    cfg.B[1][0]         = RON_FLOAT_C(1.0);
    cfg.kf_cfg.n        = 2U;
    cfg.kf_cfg.m        = 1U;
    cfg.kf_cfg.p        = 0U;
    cfg.kf_cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.A[0][1]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.Q[0][0]  = RON_FLOAT_C(0.01);
    cfg.kf_cfg.Q[1][1]  = RON_FLOAT_C(0.01);
    cfg.kf_cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.P0[0][0] = RON_FLOAT_C(1.0);
    cfg.kf_cfg.P0[1][1] = RON_FLOAT_C(1.0);
    cfg.u_min[0]        = RON_FLOAT_C(-1000.0);
    cfg.u_max[0]        = RON_FLOAT_C(1000.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

    for (step = 0U; step < 200U; step++) {
        z[0] = RON_FLOAT_C(1.0);
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_kalman_predict(&lqr, NULL));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_kalman_update(&lqr, z, true));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
        TEST_ASSERT_TRUE(u[0] >= cfg.u_min[0]);
        TEST_ASSERT_TRUE(u[0] <= cfg.u_max[0]);
    }

    TEST_ASSERT_TRUE(ron_fabs(lqr.kalman.state.x_hat[0] - RON_FLOAT_C(1.0)) < RON_FLOAT_C(0.5));

    /* Cross-source estimator calls are rejected. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_observer_step(&lqr, z, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_kalman_predict(NULL, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_kalman_update(NULL, z, true));

    /* Reset on a KALMAN-source instance also resets the embedded filter. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_reset(&lqr));

    /* Matching dims but an internally invalid embedded Kalman config is
     * rejected by ron_kf_init and propagated through ron_lqr_init. */
    {
        ron_lqr_t bad;
        ron_lqr_config_t bad_cfg = cfg;

        bad_cfg.kf_cfg.R[0][0] = lqr_make_inf();
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&bad, &bad_cfg));
    }
}

/* ----------------------------------------------------------------------- */
/* Bounds, validation and defensive paths (RON-FR-737, dims/validation)    */
/* ----------------------------------------------------------------------- */

/* RON-FR-737 | Test: RON-TC-LQR-006 */
void test_ron_tc_lqr_validation(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg;
    ron_float_t x_ext[1] = {RON_FLOAT_C(0.0)};

    /* Dimension bounds. */
    cfg       = make_ext_cfg(0U, 1U);
    cfg.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg = make_ext_cfg((uint8_t) (RON_LQR_MAX_STATES + 1U), 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg = make_ext_cfg(1U, 0U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg = make_ext_cfg(1U, (uint8_t) (RON_LQR_MAX_INPUTS + 1U));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Invalid source / gain mode enums. */
    cfg        = make_ext_cfg(1U, 1U);
    cfg.x_ext  = x_ext;
    cfg.source = (ron_lqr_source_t) 99;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg           = make_ext_cfg(1U, 1U);
    cfg.x_ext     = x_ext;
    cfg.gain_mode = (ron_lqr_gain_mode_t) 99;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Non-finite Kr always rejected; non-finite K rejected only when
     * PRECOMPUTED. */
    cfg       = make_ext_cfg(1U, 1U);
    cfg.x_ext = x_ext;
    cfg.Kr[0] = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg         = make_ext_cfg(1U, 1U);
    cfg.x_ext   = x_ext;
    cfg.K[0][0] = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* DARE mode requires finite, positive Q_cost/R_cost/dare_tol. */
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.gain_mode    = RON_LQR_GAIN_DARE;
    cfg.A[0][0]      = RON_FLOAT_C(1.0);
    cfg.B[0][0]      = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.dare_tol     = RON_FLOAT_C(0.0); /* invalid: must be > 0 */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg.dare_tol = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg.dare_tol     = RON_FLOAT_C(1e-6);
    cfg.Q_cost[0][0] = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* DARE mode needs finite A/B (source == EXTERNAL still requires them). */
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.gain_mode    = RON_LQR_GAIN_DARE;
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.dare_tol     = RON_FLOAT_C(1e-6);
    cfg.A[0][0]      = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.B[0][0] = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Output limits: non-finite / inverted. */
    cfg          = make_ext_cfg(1U, 1U);
    cfg.x_ext    = x_ext;
    cfg.u_min[0] = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg          = make_ext_cfg(1U, 1U);
    cfg.x_ext    = x_ext;
    cfg.u_max[0] = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg           = make_ext_cfg(1U, 1U);
    cfg.x_ext     = x_ext;
    cfg.du_max[0] = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg          = make_ext_cfg(1U, 1U);
    cfg.x_ext    = x_ext;
    cfg.u_min[0] = RON_FLOAT_C(5.0);
    cfg.u_max[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Integral-path validation. */
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.use_integral = true;
    cfg.Ki_aug[0]    = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.use_integral = true;
    cfg.i_min[0]     = lqr_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.use_integral = true;
    cfg.i_max[0]     = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.use_integral = true;
    cfg.i_min[0]     = RON_FLOAT_C(2.0);
    cfg.i_max[0]     = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.use_integral = true;
    cfg.C_out[0][0]  = lqr_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Embedded-estimator dimension mismatch. */
    cfg           = make_ext_cfg(2U, 1U);
    cfg.source    = RON_LQR_SOURCE_LUENBERGER;
    cfg.obs_cfg.n = 1U; /* != cfg.n */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
    cfg          = make_ext_cfg(2U, 1U);
    cfg.source   = RON_LQR_SOURCE_KALMAN;
    cfg.kf_cfg.n = 1U; /* != cfg.n */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* DARE non-convergence (degenerate: A huge, few iterations). */
    cfg              = make_ext_cfg(1U, 1U);
    cfg.x_ext        = x_ext;
    cfg.gain_mode    = RON_LQR_GAIN_DARE;
    cfg.A[0][0]      = RON_FLOAT_C(1.0);
    cfg.B[0][0]      = RON_FLOAT_C(0.0); /* uncontrollable: B == 0 */
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = RON_FLOAT_C(0.0); /* R + B'PB == 0: not PD */
    cfg.dare_tol     = RON_FLOAT_C(1e-8);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));

    /* Maximum-dimension run (RON_LQR_MAX_STATES / RON_LQR_MAX_INPUTS). */
    {
        ron_lqr_config_t big = {0};
        ron_float_t x_big[RON_LQR_MAX_STATES];
        ron_float_t r[RON_LQR_MAX_INPUTS];
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;
        uint8_t i;
        uint8_t j;

        big.n         = (uint8_t) RON_LQR_MAX_STATES;
        big.m         = (uint8_t) RON_LQR_MAX_INPUTS;
        big.source    = RON_LQR_SOURCE_EXTERNAL;
        big.gain_mode = RON_LQR_GAIN_PRECOMPUTED;
        for (i = 0U; i < (uint8_t) RON_LQR_MAX_STATES; i++) {
            x_big[i] = RON_FLOAT_C(1.0);
        }
        big.x_ext = x_big;
        for (j = 0U; j < (uint8_t) RON_LQR_MAX_INPUTS; j++) {
            big.u_min[j] = RON_FLOAT_C(-1000.0);
            big.u_max[j] = RON_FLOAT_C(1000.0);
            r[j]         = RON_FLOAT_C(0.0);
            for (i = 0U; i < (uint8_t) RON_LQR_MAX_STATES; i++) {
                big.K[j][i] = RON_FLOAT_C(0.1);
            }
        }
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &big));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    }

    /* set_gains defensive paths. */
    {
        ron_lqr_t fresh;
        const ron_float_t good_k[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES] = {0};
        const ron_float_t good_kr[RON_LQR_MAX_INPUTS]                    = {0};
        const ron_float_t bad_k[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]  = {{lqr_make_inf()}};
        const ron_float_t bad_kr[RON_LQR_MAX_INPUTS]                     = {lqr_make_nan()};

        fresh.state.is_initialised = false;
        cfg                        = make_ext_cfg(1U, 1U);
        cfg.x_ext                  = x_ext;
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));

        TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_set_gains(NULL, good_k, good_kr));
        TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_set_gains(&lqr, NULL, good_kr));
        TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_set_gains(&lqr, good_k, NULL));
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_set_gains(&fresh, good_k, good_kr));
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_set_gains(&lqr, bad_k, good_kr));
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_set_gains(&lqr, good_k, bad_kr));

        /* get_dare_solution: P == NULL on a valid instance. */
        TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqr_get_dare_solution(&lqr, NULL));

        /* observer_step / kalman_predict / kalman_update on a valid but
         * uninitialised (non-NULL) instance short-circuit before the
         * source check. */
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_observer_step(&fresh, NULL, NULL));
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_kalman_predict(&fresh, NULL));
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_kalman_update(&fresh, NULL, true));
    }

    /* External source, NULL/non-finite state pointer at step time. */
    cfg         = make_ext_cfg(1U, 1U);
    cfg.K[0][0] = RON_FLOAT_C(1.0);
    cfg.x_ext   = NULL;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    {
        ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                          ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    }

    x_ext[0]  = lqr_make_inf();
    cfg.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    {
        ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                          ron_lqr_step(&lqr, r, RON_FLOAT_C(0.01), u, &status));
    }

    /* Overflow in the control law yields RON_FAULT_OUTPUT_NAN. */
    cfg          = make_ext_cfg(1U, 1U);
    cfg.K[0][0]  = RON_FLOAT_MAX;
    cfg.u_min[0] = -RON_FLOAT_MAX;
    cfg.u_max[0] = RON_FLOAT_MAX;
    x_ext[0]     = RON_FLOAT_MAX;
    cfg.x_ext    = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    {
        ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        TEST_ASSERT_EQUAL(RON_FAULT_OUTPUT_NAN,
                          ron_lqr_step(&lqr, r, RON_FLOAT_C(0.1), u, &status));
    }

    /* Rate limiting exercise (negative branch). */
    cfg           = make_ext_cfg(1U, 1U);
    cfg.K[0][0]   = RON_FLOAT_C(1.0);
    cfg.du_max[0] = RON_FLOAT_C(1.0);
    x_ext[0]      = RON_FLOAT_C(-100.0);
    cfg.x_ext     = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    {
        ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status));
        TEST_ASSERT_FLOAT_WITHIN(LQR_TOL, RON_FLOAT_C(1.0), u[0]);
        TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);

        x_ext[0] = RON_FLOAT_C(100.0);
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status));
        TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);

        /* Not-limited branch: delta within [-delta_max, delta_max]. */
        x_ext[0] = RON_FLOAT_C(0.0);
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status));
        TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) == 0U);
    }

    /* Reset on an EXTERNAL-source instance (no embedded estimator). */
    cfg       = make_ext_cfg(1U, 1U);
    cfg.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_reset(&lqr));

    /* get_dare_solution on a valid-but-uninitialised instance. */
    {
        ron_lqr_t fresh2;
        ron_float_t p[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];

        fresh2.state.is_initialised = false;
        TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_get_dare_solution(&fresh2, p));
    }

    /* DARE fails to converge within max_iter (well-posed system, tolerance
     * unreachable within the iteration budget). */
    cfg               = make_ext_cfg(1U, 1U);
    cfg.x_ext         = x_ext;
    x_ext[0]          = RON_FLOAT_C(0.0);
    cfg.gain_mode     = RON_LQR_GAIN_DARE;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.B[0][0]       = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.dare_max_iter = 2U;
    cfg.dare_tol      = RON_FLOAT_C(1e-30);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqr_init(&lqr, &cfg));
}

/* ----------------------------------------------------------------------- */
/* Test runner                                                             */
/* ----------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_lqr_001);
    RUN_TEST(test_ron_tc_lqr_002);
    RUN_TEST(test_ron_tc_lqr_003);
    RUN_TEST(test_ron_tc_lqr_004);
    RUN_TEST(test_ron_tc_lqr_005);
    RUN_TEST(test_ron_tc_lqr_006);
    RUN_TEST(test_ron_tc_lqr_007);
    RUN_TEST(test_ron_tc_lqr_008);
    RUN_TEST(test_ron_tc_lqr_009);
    RUN_TEST(test_ron_tc_lqr_validation);
    return UNITY_END();
}
