/*
 * @file     test_ron_estimator.c
 * @brief    State-estimator component unit tests.
 * @module   test_ron_estimator
 * @doc      RON-TP-001
 * @req      RON-FR-701, RON-FR-734
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_estimator.h"

#include "unity.h"

#define EST_TOL RON_FLOAT_C(0.0001)
#define EST_N (2U)

void setUp(void)
{
}

void tearDown(void)
{
}

/* ----------------------------------------------------------------------- */
/* Helpers                                                                 */
/* ----------------------------------------------------------------------- */

/* Satisfies: RON-SR-020 | Test: RON-TC-EST-003 */
static ron_float_t est_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* Identity-dynamics observer seeded to x0 = [1, 2]. */
/* Satisfies: RON-FR-701 | Test: RON-TC-EST-001 */
static ron_estimator_config_t est_luenberger_cfg(void)
{
    ron_estimator_config_t cfg = {0};

    cfg.source          = RON_ESTIMATOR_LUENBERGER;
    cfg.obs_cfg.n       = EST_N;
    cfg.obs_cfg.m       = 1U;
    cfg.obs_cfg.p       = 0U;
    cfg.obs_cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.A[1][1] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.L[0][0] = RON_FLOAT_C(0.5);
    cfg.obs_cfg.x0[0]   = RON_FLOAT_C(1.0);
    cfg.obs_cfg.x0[1]   = RON_FLOAT_C(2.0);

    return cfg;
}

/* Identity-dynamics Kalman filter seeded to x0 = [1, 2]. */
/* Satisfies: RON-FR-701 | Test: RON-TC-EST-001 */
static ron_estimator_config_t est_kalman_cfg(void)
{
    ron_estimator_config_t cfg = {0};

    cfg.source          = RON_ESTIMATOR_KALMAN;
    cfg.kf_cfg.n        = EST_N;
    cfg.kf_cfg.m        = 1U;
    cfg.kf_cfg.p        = 0U;
    cfg.kf_cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.R[0][0]  = RON_FLOAT_C(1.0);
    cfg.kf_cfg.P0[0][0] = RON_FLOAT_C(1.0);
    cfg.kf_cfg.P0[1][1] = RON_FLOAT_C(1.0);
    cfg.kf_cfg.x0[0]    = RON_FLOAT_C(1.0);
    cfg.kf_cfg.x0[1]    = RON_FLOAT_C(2.0);

    return cfg;
}

/* ----------------------------------------------------------------------- */
/* RON-TC-EST-001 — Source Selection and Initialisation                    */
/* ----------------------------------------------------------------------- */

/* RON-TC-EST-001 | RON-FR-701, RON-FR-734 */
void test_ron_tc_est_001(void)
{
    ron_float_t x_ext[EST_N]               = {RON_FLOAT_C(3.0), RON_FLOAT_C(4.0)};
    ron_float_t y[RON_SS_MAX_OUTPUTS]      = {RON_FLOAT_C(5.0)};
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(5.0)};
    ron_float_t x_hat[EST_N]               = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
    ron_estimator_config_t ext_cfg         = {0};
    ron_estimator_config_t lue_cfg         = est_luenberger_cfg();
    ron_estimator_config_t kal_cfg         = est_kalman_cfg();
    ron_estimator_config_t bad             = {0};
    ron_estimator_t ext;
    ron_estimator_t lue;
    ron_estimator_t kal;
    ron_estimator_t fresh = {0};

    /* Each source validates and initialises. */
    ext_cfg.source = RON_ESTIMATOR_EXTERNAL;
    ext_cfg.x_ext  = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_config_validate(&ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&ext, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&lue, &lue_cfg, EST_N));
    TEST_ASSERT_TRUE(lue.observer.state.is_initialised);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&kal, &kal_cfg, EST_N));
    TEST_ASSERT_TRUE(kal.kalman.state.is_initialised);

    /* Out-of-range source and dimension mismatches. */
    bad.source = (ron_estimator_source_t) 99;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_config_validate(&bad, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_init(&fresh, &bad, EST_N));
    TEST_ASSERT_FALSE(fresh.is_initialised);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_config_validate(&lue_cfg, 1U));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_config_validate(&kal_cfg, 1U));

    /* An invalid embedded configuration returns that component's fault. */
    bad                 = est_luenberger_cfg();
    bad.obs_cfg.A[0][0] = est_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_init(&fresh, &bad, EST_N));
    TEST_ASSERT_FALSE(fresh.is_initialised);
    bad                = est_kalman_cfg();
    bad.kf_cfg.R[0][0] = est_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_init(&fresh, &bad, EST_N));

    /* NULL arguments. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_config_validate(NULL, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_init(NULL, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_init(&fresh, NULL, EST_N));

    /* Reset restores the embedded component's initial estimate. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_observer_step(&lue, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_reset(&lue));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_get_state(&lue, x_hat, EST_N));
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, RON_FLOAT_C(1.0), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, RON_FLOAT_C(2.0), x_hat[1]);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_kalman_update(&kal, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_reset(&kal));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_get_state(&kal, x_hat, EST_N));
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, RON_FLOAT_C(1.0), x_hat[0]);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_reset(&ext));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_reset(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_reset(NULL));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-EST-002 — Estimator Update Guards                                */
/* ----------------------------------------------------------------------- */

/* RON-TC-EST-002 | RON-FR-701, RON-FR-734 */
void test_ron_tc_est_002(void)
{
    ron_float_t y[RON_SS_MAX_OUTPUTS]      = {RON_FLOAT_C(5.0)};
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(5.0)};
    ron_estimator_config_t ext_cfg         = {0};
    ron_estimator_config_t lue_cfg         = est_luenberger_cfg();
    ron_estimator_config_t kal_cfg         = est_kalman_cfg();
    ron_estimator_t ext;
    ron_estimator_t lue;
    ron_estimator_t kal;
    ron_estimator_t fresh = {0};

    ext_cfg.source = RON_ESTIMATOR_EXTERNAL;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&ext, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&lue, &lue_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&kal, &kal_cfg, EST_N));

    /* The matching source advances the embedded component. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_observer_step(&lue, y, NULL));
    TEST_ASSERT_TRUE(lue.observer.state.x_hat[0] > RON_FLOAT_C(1.0));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_kalman_predict(&kal, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_kalman_update(&kal, z, true));
    TEST_ASSERT_TRUE(kal.kalman.state.x_hat[0] > RON_FLOAT_C(1.0));

    /* The embedded component's own fault is passed through. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_observer_step(&lue, NULL, NULL));

    /* Any other source is rejected. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_observer_step(&kal, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_observer_step(&ext, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_predict(&lue, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_predict(&ext, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_update(&lue, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_update(&ext, z, true));

    /* Uninitialised and NULL. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_observer_step(&fresh, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_predict(&fresh, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_kalman_update(&fresh, z, true));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_observer_step(NULL, y, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_kalman_predict(NULL, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_kalman_update(NULL, z, true));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-EST-003 — State-Estimate Read-Back                               */
/* ----------------------------------------------------------------------- */

/* RON-TC-EST-003 | RON-FR-701, RON-FR-734 */
void test_ron_tc_est_003(void)
{
    ron_float_t x_ext[EST_N]       = {RON_FLOAT_C(3.0), RON_FLOAT_C(4.0)};
    ron_float_t x_bad[EST_N]       = {RON_FLOAT_C(3.0), RON_FLOAT_C(0.0)};
    ron_float_t x_hat[EST_N]       = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
    ron_estimator_config_t ext_cfg = {0};
    ron_estimator_config_t lue_cfg = est_luenberger_cfg();
    ron_estimator_config_t kal_cfg = est_kalman_cfg();
    ron_estimator_t ext;
    ron_estimator_t lue;
    ron_estimator_t kal;
    ron_estimator_t fresh = {0};

    ext_cfg.source = RON_ESTIMATOR_EXTERNAL;
    ext_cfg.x_ext  = x_ext;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&ext, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&lue, &lue_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&kal, &kal_cfg, EST_N));

    /* Each source's estimate is copied. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_get_state(&ext, x_hat, EST_N));
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, RON_FLOAT_C(3.0), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, RON_FLOAT_C(4.0), x_hat[1]);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_get_state(&lue, x_hat, EST_N));
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, lue.observer.state.x_hat[0], x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, lue.observer.state.x_hat[1], x_hat[1]);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_get_state(&kal, x_hat, EST_N));
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, kal.kalman.state.x_hat[0], x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(EST_TOL, kal.kalman.state.x_hat[1], x_hat[1]);

    /* A non-finite estimate is rejected. */
    x_bad[1]      = est_make_nan();
    ext_cfg.x_ext = x_bad;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&ext, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_estimator_get_state(&ext, x_hat, EST_N));

    /* A missing external vector, NULL arguments, uninitialised. */
    ext_cfg.x_ext = NULL;
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_estimator_init(&ext, &ext_cfg, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_get_state(&ext, x_hat, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_get_state(NULL, x_hat, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_estimator_get_state(&lue, NULL, EST_N));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_estimator_get_state(&fresh, x_hat, EST_N));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_est_001);
    RUN_TEST(test_ron_tc_est_002);
    RUN_TEST(test_ron_tc_est_003);
    return UNITY_END();
}
