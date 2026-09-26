/*
 * @file     test_ron_statespace.c
 * @brief    State-feedback (state-space) controller unit tests.
 * @module   test_ron_statespace
 * @doc      RON-TP-001
 * @req      RON-FR-700, RON-FR-701, RON-FR-702, RON-FR-703, RON-FR-704
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_statespace.h"

#include "unity.h"

#define SS_TOL RON_FLOAT_C(0.0001)

void setUp(void)
{
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-SR-020 | Test: RON-TC-SS-009 */
static ron_float_t ss_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SS-009 */
static ron_float_t ss_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* External-source config with wide limits and rate limiting disabled. */
/* Satisfies: RON-FR-700 | Test: RON-TC-SS-001 */
static ron_ss_config_t make_ext_cfg(uint8_t n)
{
    ron_ss_config_t cfg = {0};

    cfg.n          = n;
    cfg.est.source = RON_ESTIMATOR_EXTERNAL;
    cfg.Kr         = RON_FLOAT_C(0.0);
    cfg.u_min      = RON_FLOAT_C(-1000.0);
    cfg.u_max      = RON_FLOAT_C(1000.0);
    cfg.du_max     = RON_FLOAT_C(0.0);

    return cfg;
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-001 — State-Feedback Output Correctness                       */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-001 | RON-FR-700 */
void test_ron_tc_ss_001(void)
{
    ron_ss_t ss;
    ron_ss_config_t cfg  = make_ext_cfg(2U);
    ron_float_t x_ext[2] = {RON_FLOAT_C(3.0), RON_FLOAT_C(4.0)};
    ron_float_t u;
    ron_status_t status;

    cfg.K[0]      = RON_FLOAT_C(2.0);
    cfg.K[1]      = RON_FLOAT_C(1.0);
    cfg.Kr        = RON_FLOAT_C(1.0);
    cfg.est.x_ext = x_ext;

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));

    /* u = -K·x + Kr·r = -(2*3 + 1*4) + 1*5 = -10 + 5 = -5. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(5.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-5.0), u);
    TEST_ASSERT_EQUAL(RON_STATUS_OK, status);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-002 — State-Estimate Source Selection                         */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-002 | RON-FR-701 */
void test_ron_tc_ss_002(void)
{
    ron_ss_t ext;
    ron_ss_t lue;
    ron_ss_t kal;
    ron_float_t x_ext[2] = {RON_FLOAT_C(1.0), RON_FLOAT_C(2.0)};
    ron_float_t u;
    ron_status_t status;
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];

    /* All three sources hold x_hat = [1, 2]; with K = [2,1], Kr = 0, r = 0
     * the feedback output is -(2*1 + 1*2) = -4 in every case. */

    /* EXTERNAL. */
    ron_ss_config_t ce = make_ext_cfg(2U);
    ce.K[0]            = RON_FLOAT_C(2.0);
    ce.K[1]            = RON_FLOAT_C(1.0);
    ce.est.x_ext       = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ext, &ce));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ext, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-4.0), u);

    /* LUENBERGER: embedded observer seeded to x0 = [1, 2]. */
    ron_ss_config_t cl     = make_ext_cfg(2U);
    cl.est.source          = RON_ESTIMATOR_LUENBERGER;
    cl.K[0]                = RON_FLOAT_C(2.0);
    cl.K[1]                = RON_FLOAT_C(1.0);
    cl.est.obs_cfg.n       = 2U;
    cl.est.obs_cfg.m       = 1U;
    cl.est.obs_cfg.p       = 0U;
    cl.est.obs_cfg.A[0][0] = RON_FLOAT_C(1.0);
    cl.est.obs_cfg.A[1][1] = RON_FLOAT_C(1.0);
    cl.est.obs_cfg.C[0][0] = RON_FLOAT_C(1.0);
    cl.est.obs_cfg.x0[0]   = RON_FLOAT_C(1.0);
    cl.est.obs_cfg.x0[1]   = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&lue, &cl));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&lue, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-4.0), u);

    /* Advancing the embedded observer (A = I, L = 0) keeps x_hat = [1, 2]. */
    y[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_observer_step(&lue.est, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&lue, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-4.0), u);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_reset(&lue));

    /* KALMAN: embedded filter seeded to x0 = [1, 2]. */
    ron_ss_config_t ck     = make_ext_cfg(2U);
    ck.est.source          = RON_ESTIMATOR_KALMAN;
    ck.K[0]                = RON_FLOAT_C(2.0);
    ck.K[1]                = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.n        = 2U;
    ck.est.kf_cfg.m        = 1U;
    ck.est.kf_cfg.p        = 0U;
    ck.est.kf_cfg.A[0][0]  = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.A[1][1]  = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.H[0][0]  = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.R[0][0]  = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.P0[0][0] = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.P0[1][1] = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.x0[0]    = RON_FLOAT_C(1.0);
    ck.est.kf_cfg.x0[1]    = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&kal, &ck));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&kal, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-4.0), u);

    /* The embedded Kalman lifecycle is reachable through the controller. */
    z[0] = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_kalman_predict(&kal.est, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_kalman_update(&kal.est, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_reset(&kal));

    /* Cross-source estimator calls are rejected. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_observer_step(&kal.est, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_predict(&lue.est, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_update(&lue.est, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_observer_step(&ext.est, y, NULL));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-003 — Integral Augmentation                                   */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-003 | RON-FR-702 */
void test_ron_tc_ss_003(void)
{
    ron_ss_t ss;
    ron_ss_config_t cfg  = make_ext_cfg(1U);
    ron_float_t x_ext[1] = {RON_FLOAT_C(0.0)};
    ron_float_t u;
    ron_status_t status;

    cfg.K[0]         = RON_FLOAT_C(0.0); /* no proportional feedback */
    cfg.Kr           = RON_FLOAT_C(0.0);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.Ki_aug       = RON_FLOAT_C(1.0);
    cfg.C_out[0]     = RON_FLOAT_C(1.0);
    cfg.i_min        = RON_FLOAT_C(-100.0);
    cfg.i_max        = RON_FLOAT_C(100.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));

    /* e_reg = r - C_out·x = 2 - 0 = 2; integral += 1*0.5*2 = 1; u = 1. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.0), u);

    /* Second step accumulates to 2. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(2.0), u);

    /* Reset clears the accumulator. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_reset(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.0), u);

    /* Integral clamp: tighten i_max to 1.5 and re-run. */
    cfg.i_max = RON_FLOAT_C(1.5);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.0), u);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.5), u); /* clamped */
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-004 — Saturation and Rate Limiting (PID-equivalent)           */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-004 | RON-FR-703 */
void test_ron_tc_ss_004(void)
{
    ron_ss_t sat;
    ron_ss_t rl;
    ron_float_t x_ext[1];
    ron_float_t u;
    ron_status_t status;

    /* --- Saturation: u_min = -2, u_max = 2, rate limiting disabled. --- */
    ron_ss_config_t cs = make_ext_cfg(1U);
    cs.K[0]            = RON_FLOAT_C(1.0);
    cs.u_min           = RON_FLOAT_C(-2.0);
    cs.u_max           = RON_FLOAT_C(2.0);
    cs.est.x_ext       = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&sat, &cs));

    x_ext[0] = RON_FLOAT_C(10.0); /* u_raw = -10 -> clamp to -2 */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&sat, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-2.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);

    x_ext[0] = RON_FLOAT_C(-10.0); /* u_raw = 10 -> clamp to 2 */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&sat, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(2.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);

    x_ext[0] = RON_FLOAT_C(0.5); /* u_raw = -0.5, within limits */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&sat, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-0.5), u);
    TEST_ASSERT_EQUAL(RON_STATUS_OK, status);

    /* --- Rate limiting: du_max = 1, dt = 1 -> |Δu| <= 1 per step. --- */
    ron_ss_config_t cr = make_ext_cfg(1U);
    cr.K[0]            = RON_FLOAT_C(1.0);
    cr.u_min           = RON_FLOAT_C(-100.0);
    cr.u_max           = RON_FLOAT_C(100.0);
    cr.du_max          = RON_FLOAT_C(1.0);
    cr.est.x_ext       = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&rl, &cr));

    x_ext[0] = RON_FLOAT_C(-5.0); /* u_raw = 5, from u_prev 0 -> +1 */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&rl, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, /* u_raw = 5, from 1 -> 2 */
                      ron_ss_step(&rl, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(2.0), u);

    x_ext[0] = RON_FLOAT_C(-2.5); /* u_raw = 2.5, delta 0.5 -> not limited */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&rl, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(2.5), u);
    TEST_ASSERT_EQUAL(RON_STATUS_OK, status);

    x_ext[0] = RON_FLOAT_C(10.0); /* u_raw = -10, from 2.5 -> 1.5 (negative branch) */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&rl, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(1.5), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-005 — Runtime Gain Update                                     */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-005 | RON-FR-704 */
void test_ron_tc_ss_005(void)
{
    ron_ss_t ss;
    ron_ss_t fresh;
    ron_ss_config_t cfg                  = make_ext_cfg(2U);
    ron_float_t x_ext[2]                 = {RON_FLOAT_C(3.0), RON_FLOAT_C(4.0)};
    ron_float_t new_k[RON_SS_MAX_STATES] = {0};
    ron_float_t bad_k[RON_SS_MAX_STATES] = {0};
    ron_float_t u;
    ron_status_t status;

    fresh.state.is_initialised = false;

    cfg.K[0]      = RON_FLOAT_C(1.0);
    cfg.K[1]      = RON_FLOAT_C(0.0);
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));

    /* Initial: u = -(1*3 + 0*4) = -3. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-3.0), u);

    /* Update K = [0, 2], Kr = 1: u = -(0*3 + 2*4) + 1*1 = -8 + 1 = -7. */
    new_k[0] = RON_FLOAT_C(0.0);
    new_k[1] = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_set_gains(&ss, new_k, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(1.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-7.0), u);

    /* Defensive paths. */
    bad_k[0] = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_set_gains(NULL, new_k, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_set_gains(&ss, NULL, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_set_gains(&fresh, new_k, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_set_gains(&ss, bad_k, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_set_gains(&ss, new_k, ss_make_inf()));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-009 — Bounds, Validation, and Defensive Paths                 */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-009 | RON-FR-723 — configuration validation branches. */
void test_ron_tc_ss_009_validation(void)
{
    ron_ss_t ss;
    ron_float_t x_ext[1] = {RON_FLOAT_C(0.0)};
    ron_ss_config_t cfg;

    /* Dimension bounds. */
    cfg           = make_ext_cfg(0U);
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg.n = (uint8_t) (RON_SS_MAX_STATES + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Invalid source enum. */
    cfg            = make_ext_cfg(1U);
    cfg.est.x_ext  = x_ext;
    cfg.est.source = (ron_estimator_source_t) 99;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Non-finite gains. */
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.K[0]      = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.Kr        = ss_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Non-finite / inconsistent output limits. */
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.u_min     = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.u_max     = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.du_max    = ss_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg           = make_ext_cfg(1U);
    cfg.est.x_ext = x_ext;
    cfg.u_min     = RON_FLOAT_C(5.0);
    cfg.u_max     = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Integral-path validation. */
    cfg              = make_ext_cfg(1U);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.Ki_aug       = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg              = make_ext_cfg(1U);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.i_min        = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg              = make_ext_cfg(1U);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.i_max        = ss_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg              = make_ext_cfg(1U);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.i_min        = RON_FLOAT_C(2.0);
    cfg.i_max        = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg              = make_ext_cfg(1U);
    cfg.est.x_ext    = x_ext;
    cfg.use_integral = true;
    cfg.C_out[0]     = ss_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Embedded-estimator dimension mismatch and init failure. */
    cfg               = make_ext_cfg(2U);
    cfg.est.source    = RON_ESTIMATOR_LUENBERGER;
    cfg.est.obs_cfg.n = 1U; /* != cfg.n */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg              = make_ext_cfg(2U);
    cfg.est.source   = RON_ESTIMATOR_KALMAN;
    cfg.est.kf_cfg.n = 1U; /* != cfg.n */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Valid dims but the embedded observer config is itself invalid. */
    cfg                     = make_ext_cfg(2U);
    cfg.est.source          = RON_ESTIMATOR_LUENBERGER;
    cfg.est.obs_cfg.n       = 2U;
    cfg.est.obs_cfg.m       = 1U;
    cfg.est.obs_cfg.A[0][0] = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Valid dims but the embedded Kalman config is itself invalid. */
    cfg                    = make_ext_cfg(2U);
    cfg.est.source         = RON_ESTIMATOR_KALMAN;
    cfg.est.kf_cfg.n       = 2U;
    cfg.est.kf_cfg.m       = 1U;
    cfg.est.kf_cfg.R[0][0] = ss_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
}

/* RON-TC-SS-009 | RON-FR-723 — max-dimension run, step defensive paths. */
void test_ron_tc_ss_009_runtime(void)
{
    ron_ss_t ss;
    ron_ss_t fresh;
    ron_ss_config_t cfg;
    ron_float_t x_ext[RON_SS_MAX_STATES];
    ron_float_t u;
    ron_status_t status;
    uint8_t i;

    fresh.state.is_initialised = false;

    /* Maximum-dimension external-source controller initialises and runs. */
    cfg = make_ext_cfg((uint8_t) RON_SS_MAX_STATES);
    for (i = 0U; i < (uint8_t) RON_SS_MAX_STATES; i++) {
        x_ext[i] = RON_FLOAT_C(1.0);
        cfg.K[i] = RON_FLOAT_C(0.5);
    }
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    /* Null-pointer rejection. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_init(NULL, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_init(&ss, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_reset(NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_ss_step(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), NULL, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, NULL));

    /* Uninitialised-instance rejection. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_reset(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID,
                      ron_ss_step(&fresh, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID,
                      ron_estimator_observer_step(&fresh.est, x_ext, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_predict(&fresh.est, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID,
                      ron_estimator_kalman_update(&fresh.est, x_ext, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_observer_step(NULL, x_ext, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_kalman_predict(NULL, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_kalman_update(NULL, x_ext, true));

    /* Non-finite r / non-positive dt rejection (each latches; cleared so
     * the next case reaches its own check). */
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, ss_make_inf(), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_fault_clear(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), ss_make_nan(), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_fault_clear(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &u, &status));

    /* External source with a NULL state pointer is rejected at step time. */
    cfg           = make_ext_cfg(1U);
    cfg.K[0]      = RON_FLOAT_C(1.0);
    cfg.est.x_ext = NULL;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    /* External source with a non-finite state entry. */
    x_ext[0]      = ss_make_inf();
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    /* Overflow in the control law yields RON_FAULT_OUTPUT_NAN. */
    cfg           = make_ext_cfg(1U);
    cfg.K[0]      = RON_FLOAT_MAX;
    cfg.u_min     = -RON_FLOAT_MAX;
    cfg.u_max     = RON_FLOAT_MAX;
    x_ext[0]      = RON_FLOAT_MAX;
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_OUTPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));

    /* The rejected step does not wind the integrator: the integral stays at
     * zero, and only a later finite step advances it (by Ki * dt * r). */
    cfg.use_integral = true;
    cfg.Ki_aug       = RON_FLOAT_C(1.0);
    cfg.C_out[0]     = RON_FLOAT_C(0.0);
    cfg.i_min        = RON_FLOAT_C(-100.0);
    cfg.i_max        = RON_FLOAT_C(100.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_OUTPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(1.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), ss.state.integral);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), ss.state.u_prev);
    x_ext[0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_fault_clear(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(1.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.1), ss.state.integral);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-010 — Fault Latch and Explicit Clear                          */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-010 | RON-FR-703, RON-SR-010, RON-SR-012, RON-SR-013 */
void test_ron_tc_ss_010(void)
{
    ron_ss_t ss;
    ron_ss_t fresh       = {0};
    ron_ss_config_t cfg  = make_ext_cfg(1U);
    ron_float_t x_ext[1] = {RON_FLOAT_C(1.0)};
    ron_float_t u        = RON_FLOAT_C(0.0);
    ron_status_t status  = RON_STATUS_OK;

    cfg.K[0]      = RON_FLOAT_C(1.0);
    cfg.est.x_ext = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-1.0), u);

    /* The fault latches: the last output is held and FAULT reported. */
    x_ext[0] = RON_FLOAT_C(2.0);
    u        = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, ss_make_nan(), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ss.state.faults);
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-1.0), u);
    TEST_ASSERT_EQUAL(RON_STATUS_FAULT, status);
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-1.0), ss.state.u_prev);

    /* Finite inputs do not clear it. */
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-1.0), u);
    TEST_ASSERT_EQUAL(RON_STATUS_FAULT, status);

    /* An explicit clear resumes normal stepping. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_fault_clear(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-2.0), u);
    TEST_ASSERT_EQUAL(0, status & RON_STATUS_FAULT);

    /* Reset clears a latched fault too. */
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), ss_make_nan(), &u, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_reset(&ss));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ss.state.faults);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    /* Defensive paths; argument faults are not latched. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_ss_fault_clear(NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_fault_clear(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), NULL, &status));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ss.state.faults);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-011 — Safe-State Output Policy                                */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-011 | RON-FR-703, RON-SR-011 */
void test_ron_tc_ss_011(void)
{
    const ron_safe_policy_t policies[3] = {RON_SAFE_HOLD_LAST, RON_SAFE_ZERO, RON_SAFE_CONSTANT};
    /* du_max = 1/s at dt = 0.01 s: the output moves 0.01 per step. */
    const ron_float_t expected[3] = {RON_FLOAT_C(-0.01), RON_FLOAT_C(0.0), RON_FLOAT_C(5.0)};
    ron_float_t x_ext[1]          = {RON_FLOAT_C(1.0)};
    ron_ss_t ss;
    ron_ss_config_t cfg;
    ron_float_t u;
    ron_status_t status;
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        cfg             = make_ext_cfg(1U);
        cfg.K[0]        = RON_FLOAT_C(1.0);
        cfg.u_min       = RON_FLOAT_C(-5.0);
        cfg.u_max       = RON_FLOAT_C(5.0);
        cfg.du_max      = RON_FLOAT_C(1.0);
        cfg.est.x_ext   = x_ext;
        cfg.safe_policy = policies[i];
        cfg.safe_value  = RON_FLOAT_C(9.0); /* beyond u_max: clamped */
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                          ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
        TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-0.01), u);

        /* Latched: the policy output, twice, with the history untouched. */
        TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                          ron_ss_step(&ss, ss_make_nan(), RON_FLOAT_C(0.01), &u, &status));
        TEST_ASSERT_FLOAT_WITHIN(SS_TOL, expected[i], u);
        TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                          ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
        TEST_ASSERT_FLOAT_WITHIN(SS_TOL, expected[i], u);
        TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-0.01), ss.state.u_prev);

        /* After the clear, rate limiting continues from the last output. */
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_fault_clear(&ss));
        TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                          ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
        TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-0.02), u);
    }

    /* Invalid safe-state configuration. */
    cfg            = make_ext_cfg(1U);
    cfg.est.x_ext  = x_ext;
    cfg.safe_value = ss_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));
    cfg.safe_value  = RON_FLOAT_C(0.0);
    cfg.safe_policy = (ron_safe_policy_t) 7;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_ss_init(&ss, &cfg));

    /* Defensive default: a policy corrupted after init falls back to
     * holding the last output (mirrors RON-TC-SAFE-008 for the PID). */
    cfg.safe_policy = RON_SAFE_ZERO;
    cfg.K[0]        = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE,
                      ron_ss_step(&ss, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    ss.cfg.safe_policy = (ron_safe_policy_t) 7;
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN,
                      ron_ss_step(&ss, ss_make_nan(), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(SS_TOL, RON_FLOAT_C(-1.0), u);
}

/* ----------------------------------------------------------------------- */
/* Test runner                                                             */
/* ----------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_ss_001);
    RUN_TEST(test_ron_tc_ss_002);
    RUN_TEST(test_ron_tc_ss_003);
    RUN_TEST(test_ron_tc_ss_004);
    RUN_TEST(test_ron_tc_ss_005);
    RUN_TEST(test_ron_tc_ss_009_validation);
    RUN_TEST(test_ron_tc_ss_009_runtime);
    RUN_TEST(test_ron_tc_ss_010);
    RUN_TEST(test_ron_tc_ss_011);
    return UNITY_END();
}
