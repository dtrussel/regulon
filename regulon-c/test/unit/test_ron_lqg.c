/*
 * @file     test_ron_lqg.c
 * @brief    Discrete-time MIMO LQG controller unit tests.
 * @module   test_ron_lqg
 * @doc      RON-TP-001
 * @req      RON-FR-750, RON-FR-751, RON-FR-752, RON-FR-753, RON-FR-754,
 *           RON-FR-755, RON-FR-756, RON-FR-757, RON-FR-758, RON-FR-759
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_lqg.h"

#include "unity.h"

#define LQG_TOL RON_FLOAT_C(0.0001)

void setUp(void)
{
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-SR-020 | Test: RON-TC-LQG-009 */
static ron_float_t lqg_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-LQG-009 */
static ron_float_t lqg_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* 2-state, 1-input, 1-measurement double-integrator with pre-computed gain,
 * wide limits, and no rate limiting. */
static ron_lqg_config_t make_base_cfg(void)
{
    ron_lqg_config_t cfg = {0};

    cfg.n             = 2U;
    cfg.m             = 1U;
    cfg.p             = 1U;
    cfg.gain_mode     = RON_LQG_GAIN_PRECOMPUTED;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.A[0][1]       = RON_FLOAT_C(1.0);
    cfg.A[1][1]       = RON_FLOAT_C(1.0);
    cfg.B[1][0]       = RON_FLOAT_C(1.0);
    cfg.H[0][0]       = RON_FLOAT_C(1.0);
    cfg.Q_noise[0][0] = RON_FLOAT_C(0.01);
    cfg.Q_noise[1][1] = RON_FLOAT_C(0.01);
    cfg.R_noise[0][0] = RON_FLOAT_C(1.0);
    cfg.P0[0][0]      = RON_FLOAT_C(10.0);
    cfg.P0[1][1]      = RON_FLOAT_C(10.0);
    cfg.K[0][0]       = RON_FLOAT_C(1.0);
    cfg.K[0][1]       = RON_FLOAT_C(1.0);
    cfg.u_min[0]      = RON_FLOAT_C(-1000.0);
    cfg.u_max[0]      = RON_FLOAT_C(1000.0);

    return cfg;
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-001 — Init with Pre-computed K and Kalman                    */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-001 | RON-FR-750, RON-FR-756 */
void test_ron_tc_lqg_001(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg = make_base_cfg();

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_TRUE(lqg.is_initialised);
    TEST_ASSERT_TRUE(lqg.kalman.state.is_initialised);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-002 — Predict Step Advances Kalman State                     */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-002 | RON-FR-753 */
void test_ron_tc_lqg_002(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg              = make_base_cfg();
    ron_float_t u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    ron_float_t x_hat[RON_LQR_MAX_STATES];

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, x_hat));

    /* x_new = A*x + B*u with x0 = {0,0}: x_hat = {0, 1}. */
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, RON_FLOAT_C(0.0), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, RON_FLOAT_C(1.0), x_hat[1]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-003 — Update Corrects State Estimate                         */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-003 | RON-FR-754 */
void test_ron_tc_lqg_003(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg                   = make_base_cfg();
    ron_float_t u[RON_LQR_MAX_INPUTS]      = {RON_FLOAT_C(0.0)};
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(0.5)};
    ron_float_t before[RON_LQR_MAX_STATES];
    ron_float_t after[RON_LQR_MAX_STATES];

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, before));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, after));

    /* Innovation z - H*x_hat = 0.5 - 0 = 0.5 > 0, so the estimate moves in
     * the positive direction. */
    TEST_ASSERT_TRUE(after[0] > before[0]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-004 — Measurement Dropout is Silent                          */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-004 | RON-FR-754 */
void test_ron_tc_lqg_004(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg              = make_base_cfg();
    ron_float_t u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t before[RON_LQR_MAX_STATES];
    ron_float_t after[RON_LQR_MAX_STATES];

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, before));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, NULL, false));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, after));

    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, before[0], after[0]);
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, before[1], after[1]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-005 — Control Step Uses Kalman Estimate                      */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-005 | RON-FR-755 */
void test_ron_tc_lqg_005(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg                      = make_base_cfg();
    ron_float_t predict_u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(2.0)};
    ron_float_t z[RON_KF_MAX_MEASUREMENTS]    = {RON_FLOAT_C(1.5)};
    ron_float_t r[RON_LQR_MAX_INPUTS]         = {RON_FLOAT_C(1.0)};
    ron_float_t x_hat[RON_LQR_MAX_STATES];
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.K[0][0] = RON_FLOAT_C(0.5);
    cfg.K[0][1] = RON_FLOAT_C(0.25);
    cfg.Kr[0]   = RON_FLOAT_C(2.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, predict_u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_get_state(&lqg, x_hat));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(0.01), u, &status));

    {
        ron_float_t expected =
            -((cfg.K[0][0] * x_hat[0]) + (cfg.K[0][1] * x_hat[1])) + (cfg.Kr[0] * r[0]);

        TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, expected, u[0]);
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-006 — DARE at Init Time (Both Gains)                         */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-006 | RON-FR-756 */
void test_ron_tc_lqg_006(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg = make_base_cfg();

    cfg.gain_mode           = RON_LQG_GAIN_DARE;
    cfg.Q_cost[0][0]        = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1]        = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]        = RON_FLOAT_C(1.0);
    cfg.dare_max_iter       = 200U;
    cfg.dare_tol            = RON_FLOAT_C(1e-4);
    cfg.use_kf_steady_state = true;
    cfg.K_f_inf[0][0]       = RON_FLOAT_C(0.5);
    cfg.K_f_inf[1][0]       = RON_FLOAT_C(0.2);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_TRUE(lqg.kalman.state.is_initialised);
    TEST_ASSERT_TRUE(lqg.kalman.cfg.steady_state);
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, RON_FLOAT_C(0.5), lqg.kalman.cfg.K_inf[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, RON_FLOAT_C(0.2), lqg.kalman.cfg.K_inf[1][0]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-007 — Separation Principle                                   */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-007 | RON-FR-752 */
void test_ron_tc_lqg_007(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg = make_base_cfg();

    cfg.gain_mode           = RON_LQG_GAIN_DARE;
    cfg.Q_cost[0][0]        = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1]        = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]        = RON_FLOAT_C(1.0);
    cfg.dare_max_iter       = 200U;
    cfg.dare_tol            = RON_FLOAT_C(1e-4);
    cfg.use_kf_steady_state = false;

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));

    /* Standalone LQR solving the identical DARE problem must match the LQG
     * gain (separation principle). */
    {
        ron_lqr_t lqr;
        ron_lqr_config_t lqr_cfg = {0};
        ron_float_t x_ext[2]     = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
        uint8_t i;

        lqr_cfg.n             = 2U;
        lqr_cfg.m             = 1U;
        lqr_cfg.source        = RON_LQR_SOURCE_EXTERNAL;
        lqr_cfg.x_ext         = x_ext;
        lqr_cfg.gain_mode     = RON_LQR_GAIN_DARE;
        lqr_cfg.A[0][0]       = RON_FLOAT_C(1.0);
        lqr_cfg.A[0][1]       = RON_FLOAT_C(1.0);
        lqr_cfg.A[1][1]       = RON_FLOAT_C(1.0);
        lqr_cfg.B[1][0]       = RON_FLOAT_C(1.0);
        lqr_cfg.Q_cost[0][0]  = RON_FLOAT_C(1.0);
        lqr_cfg.Q_cost[1][1]  = RON_FLOAT_C(1.0);
        lqr_cfg.R_cost[0][0]  = RON_FLOAT_C(1.0);
        lqr_cfg.dare_max_iter = 200U;
        lqr_cfg.dare_tol      = RON_FLOAT_C(1e-4);
        lqr_cfg.u_min[0]      = RON_FLOAT_C(-1000.0);
        lqr_cfg.u_max[0]      = RON_FLOAT_C(1000.0);

        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqr_init(&lqr, &lqr_cfg));

        for (i = 0U; i < 2U; i++) {
            TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-4), lqr.state.K_solved[0][i],
                                     lqg.K_solved[0][i]);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-008 — Output Saturation                                      */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-008 | RON-FR-757 */
void test_ron_tc_lqg_008(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg                      = make_base_cfg();
    ron_float_t predict_u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t z[RON_KF_MAX_MEASUREMENTS]    = {RON_FLOAT_C(-100.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS]         = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    cfg.K[0][0]  = RON_FLOAT_C(5.0);
    cfg.K[0][1]  = RON_FLOAT_C(5.0);
    cfg.u_min[0] = RON_FLOAT_C(-1.0);
    cfg.u_max[0] = RON_FLOAT_C(1.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, predict_u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z, true));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(1.0), u, &status));
    TEST_ASSERT_FLOAT_WITHIN(LQG_TOL, RON_FLOAT_C(1.0), u[0]);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-LQG-009 — Fault Detection: Null and Uninitialised                */
/* ----------------------------------------------------------------------- */

/* RON-TC-LQG-009 | RON-FR-757, RON-SR-001 */
void test_ron_tc_lqg_009(void)
{
    ron_lqg_t lqg;
    ron_lqg_t fresh;
    ron_lqg_config_t cfg              = make_base_cfg();
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;

    fresh.is_initialised = false;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));

    /* (a) init with lqg == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_init(NULL, &cfg));
    /* (b) step with lqg == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_step(NULL, r, RON_FLOAT_C(0.01), u, &status));
    /* (c) step on uninitialised instance. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID,
                      ron_lqg_step(&fresh, r, RON_FLOAT_C(0.01), u, &status));
    /* (d) step with u == NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_lqg_step(&lqg, r, RON_FLOAT_C(0.01), NULL, &status));

    /* Additional defensive paths exercised for coverage. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_init(&lqg, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_lqg_step(&lqg, NULL, RON_FLOAT_C(0.01), u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_step(&lqg, r, RON_FLOAT_C(0.01), u, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_lqg_step(&lqg, r, lqg_make_nan(), u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_lqg_step(&lqg, r, RON_FLOAT_C(0.0), u, &status));
    {
        ron_float_t bad_r[RON_LQR_MAX_INPUTS] = {lqg_make_inf()};

        TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                          ron_lqg_step(&lqg, bad_r, RON_FLOAT_C(0.01), u, &status));
    }
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_reset(NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_reset(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_predict(NULL, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_update(NULL, NULL, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_lqg_get_state(NULL, NULL));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_reset(&lqg));
}

/* ----------------------------------------------------------------------- */
/* Configuration validation                                                */
/* ----------------------------------------------------------------------- */

/* RON-FR-750, RON-FR-751 | Test: RON-TC-LQG-009 */
void test_ron_tc_lqg_validation(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg;

    /* Dimension bounds. */
    cfg   = make_base_cfg();
    cfg.n = 0U;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg   = make_base_cfg();
    cfg.n = (uint8_t) (RON_LQR_MAX_STATES + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg   = make_base_cfg();
    cfg.m = 0U;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg   = make_base_cfg();
    cfg.m = (uint8_t) (RON_LQR_MAX_INPUTS + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg   = make_base_cfg();
    cfg.p = 0U;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg   = make_base_cfg();
    cfg.p = (uint8_t) (RON_KF_MAX_MEASUREMENTS + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Invalid gain mode enum. */
    cfg           = make_base_cfg();
    cfg.gain_mode = (ron_lqg_gain_mode_t) 99;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Non-finite system matrices. */
    cfg         = make_base_cfg();
    cfg.A[0][0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg         = make_base_cfg();
    cfg.B[1][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg         = make_base_cfg();
    cfg.H[0][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Non-finite noise / initial covariance. */
    cfg               = make_base_cfg();
    cfg.Q_noise[0][0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg               = make_base_cfg();
    cfg.R_noise[0][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg       = make_base_cfg();
    cfg.x0[0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg          = make_base_cfg();
    cfg.P0[0][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg                     = make_base_cfg();
    cfg.use_kf_steady_state = true;
    cfg.K_f_inf[0][0]       = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Non-finite Kr always rejected; non-finite K rejected only when
     * PRECOMPUTED. */
    cfg       = make_base_cfg();
    cfg.Kr[0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg         = make_base_cfg();
    cfg.K[0][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* DARE mode requires finite, positive Q_cost/R_cost/dare_tol. */
    cfg              = make_base_cfg();
    cfg.gain_mode    = RON_LQG_GAIN_DARE;
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.dare_tol     = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg.dare_tol     = RON_FLOAT_C(1e-6);
    cfg.R_cost[0][0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg.R_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.dare_tol     = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* DARE gain resolution failure (uncontrollable: B == 0, R + B'PB == 0
     * is not positive definite) propagates through ron_lqg_init. */
    cfg              = make_base_cfg();
    cfg.gain_mode    = RON_LQG_GAIN_DARE;
    cfg.B[1][0]      = RON_FLOAT_C(0.0);
    cfg.Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1] = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0] = RON_FLOAT_C(0.0);
    cfg.dare_tol     = RON_FLOAT_C(1e-4);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Embedded Kalman init failure (non-finite noise covariance, now caught
     * one layer down inside ron_kf_init rather than duplicated here). */
    cfg               = make_base_cfg();
    cfg.Q_noise[0][0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Output limits: non-finite / inverted. */
    cfg          = make_base_cfg();
    cfg.u_min[0] = lqg_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg          = make_base_cfg();
    cfg.u_max[0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg           = make_base_cfg();
    cfg.du_max[0] = lqg_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));
    cfg          = make_base_cfg();
    cfg.u_min[0] = RON_FLOAT_C(5.0);
    cfg.u_max[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_lqg_init(&lqg, &cfg));

    /* Maximum-dimension run. */
    {
        ron_lqg_config_t big = {0};
        ron_float_t r[RON_LQR_MAX_INPUTS];
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;
        uint8_t i;
        uint8_t j;

        big.n         = (uint8_t) RON_LQR_MAX_STATES;
        big.m         = (uint8_t) RON_LQR_MAX_INPUTS;
        big.p         = (uint8_t) RON_KF_MAX_MEASUREMENTS;
        big.gain_mode = RON_LQG_GAIN_PRECOMPUTED;
        for (i = 0U; i < (uint8_t) RON_LQR_MAX_STATES; i++) {
            big.A[i][i]  = RON_FLOAT_C(1.0);
            big.P0[i][i] = RON_FLOAT_C(1.0);
        }
        for (i = 0U; i < (uint8_t) RON_KF_MAX_MEASUREMENTS; i++) {
            big.H[i][i]       = RON_FLOAT_C(1.0);
            big.R_noise[i][i] = RON_FLOAT_C(1.0);
        }
        for (j = 0U; j < (uint8_t) RON_LQR_MAX_INPUTS; j++) {
            big.u_min[j] = RON_FLOAT_C(-1000.0);
            big.u_max[j] = RON_FLOAT_C(1000.0);
            r[j]         = RON_FLOAT_C(0.0);
        }
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &big));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(0.01), u, &status));
    }

    /* Overflow in the control law yields RON_FAULT_OUTPUT_NAN. */
    cfg          = make_base_cfg();
    cfg.K[0][0]  = RON_FLOAT_MAX;
    cfg.u_min[0] = -RON_FLOAT_MAX;
    cfg.u_max[0] = RON_FLOAT_MAX;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    {
        ron_float_t predict_u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t z[RON_KF_MAX_MEASUREMENTS]    = {RON_FLOAT_MAX};
        ron_float_t r[RON_LQR_MAX_INPUTS]         = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, predict_u));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z, true));
        TEST_ASSERT_EQUAL(RON_FAULT_OUTPUT_NAN,
                          ron_lqg_step(&lqg, r, RON_FLOAT_C(0.1), u, &status));
    }

    /* Rate limiting exercise (all three branches). */
    cfg           = make_base_cfg();
    cfg.K[0][0]   = RON_FLOAT_C(1.0);
    cfg.K[0][1]   = RON_FLOAT_C(0.0);
    cfg.du_max[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_init(&lqg, &cfg));
    {
        ron_float_t predict_u[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(0.0)};
        ron_float_t r[RON_LQR_MAX_INPUTS]         = {RON_FLOAT_C(0.0)};
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        /* Not-limited branch: the fresh instance's x_hat and u_prev both
         * start at zero, so the very first step has zero delta. */
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(1.0), u, &status));
        TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) == 0U);

        {
            ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(-100.0)};

            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, predict_u));
            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z, true));
            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(1.0), u, &status));
            TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
        }

        /* Negative-direction rate limiting: a large positive measurement
         * swings the estimate (and therefore u_raw) far enough negative
         * relative to u_prev to hit the other rate-limit branch. */
        {
            ron_float_t z_pos[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(1000.0)};

            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_predict(&lqg, predict_u));
            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_update(&lqg, z_pos, true));
            TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_lqg_step(&lqg, r, RON_FLOAT_C(1.0), u, &status));
            TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Test runner                                                             */
/* ----------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_lqg_001);
    RUN_TEST(test_ron_tc_lqg_002);
    RUN_TEST(test_ron_tc_lqg_003);
    RUN_TEST(test_ron_tc_lqg_004);
    RUN_TEST(test_ron_tc_lqg_005);
    RUN_TEST(test_ron_tc_lqg_006);
    RUN_TEST(test_ron_tc_lqg_007);
    RUN_TEST(test_ron_tc_lqg_008);
    RUN_TEST(test_ron_tc_lqg_009);
    RUN_TEST(test_ron_tc_lqg_validation);
    return UNITY_END();
}
