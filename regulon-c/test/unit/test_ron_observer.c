/*
 * @file     test_ron_observer.c
 * @brief    Luenberger observer unit tests.
 * @module   test_ron_observer
 * @doc      RON-TP-001
 * @req      RON-FR-720, RON-FR-721, RON-FR-722, RON-FR-723
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_observer.h"

#include "unity.h"

#define OBS_TOL RON_FLOAT_C(0.0001)

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
static ron_float_t obs_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SS-009 */
static ron_float_t obs_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* A valid 2-state, 1-output, 1-input observer:
 *   A = [[1,1],[0,1]], B = [[0],[1]], C = [[1,0]], L = [[0.5],[0.2]]. */
/* Satisfies: RON-FR-721 | Test: RON-TC-SS-006 */
static ron_obs_config_t make_obs_cfg(void)
{
    ron_obs_config_t cfg;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < (uint8_t) RON_SS_MAX_STATES; i++) {
        cfg.x0[i] = RON_FLOAT_C(0.0);
        for (j = 0U; j < (uint8_t) RON_SS_MAX_STATES; j++) {
            cfg.A[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_SS_MAX_INPUTS; j++) {
            cfg.B[i][j] = RON_FLOAT_C(0.0);
        }
        for (j = 0U; j < (uint8_t) RON_SS_MAX_OUTPUTS; j++) {
            cfg.L[i][j] = RON_FLOAT_C(0.0);
        }
    }
    for (i = 0U; i < (uint8_t) RON_SS_MAX_OUTPUTS; i++) {
        for (j = 0U; j < (uint8_t) RON_SS_MAX_STATES; j++) {
            cfg.C[i][j] = RON_FLOAT_C(0.0);
        }
    }

    cfg.n = 2U;
    cfg.m = 1U;
    cfg.p = 1U;

    cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.A[0][1] = RON_FLOAT_C(1.0);
    cfg.A[1][1] = RON_FLOAT_C(1.0);
    cfg.B[1][0] = RON_FLOAT_C(1.0);
    cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.L[0][0] = RON_FLOAT_C(0.5);
    cfg.L[1][0] = RON_FLOAT_C(0.2);

    return cfg;
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-006 — Luenberger Observer Step Correctness                    */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-006 | RON-FR-720 */
void test_ron_tc_ss_006(void)
{
    ron_obs_t obs;
    ron_obs_config_t cfg = make_obs_cfg();
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    ron_float_t u[RON_SS_MAX_INPUTS];
    ron_float_t x_hat[RON_SS_MAX_STATES];

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));

    y[0] = RON_FLOAT_C(1.0);
    u[0] = RON_FLOAT_C(0.0);

    /* Step 1: innovation = 1 - 0 = 1; x_hat = [0.5, 0.2]. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_step(&obs, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.5), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.2), x_hat[1]);

    /* Step 2: C x = 0.5; innov = 0.5; ax = [0.7,0.2]; li = [0.25,0.1].
     *         x_hat = [0.95, 0.3]. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_step(&obs, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.95), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.3), x_hat[1]);

    /* Reset returns to the seeded x0. */
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_reset(&obs));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.0), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.0), x_hat[1]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-007 — Parameterisation and Convergence                        */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-007 | RON-FR-721 */
void test_ron_tc_ss_007_convergence(void)
{
    /* Scalar autonomous plant: A = 1, C = 1, L = 0.5, p = 0, true state 5. */
    ron_obs_t obs;
    ron_obs_config_t cfg = make_obs_cfg();
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    ron_float_t x_hat[RON_SS_MAX_STATES];
    uint8_t k;

    cfg.n       = 1U;
    cfg.m       = 1U;
    cfg.p       = 0U;
    cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.L[0][0] = RON_FLOAT_C(0.5);
    cfg.x0[0]   = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));

    y[0] = RON_FLOAT_C(5.0);
    for (k = 0U; k < 40U; k++) {
        /* p == 0: control input is ignored, u may be NULL. */
        TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_step(&obs, y, NULL));
    }

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.001), RON_FLOAT_C(5.0), x_hat[0]);
}

/* RON-TC-SS-007 | RON-FR-721 — config validation isolates each matrix. */
void test_ron_tc_ss_007_validation(void)
{
    ron_obs_t obs;
    ron_obs_config_t cfg;
    const ron_float_t inf = obs_make_inf();
    const ron_float_t nan = obs_make_nan();

    /* All-finite config initialises. */
    cfg = make_obs_cfg();
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));

    /* Non-finite A. */
    cfg         = make_obs_cfg();
    cfg.A[0][0] = inf;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));

    /* Non-finite C. */
    cfg         = make_obs_cfg();
    cfg.C[0][0] = nan;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));

    /* Non-finite L. */
    cfg         = make_obs_cfg();
    cfg.L[0][0] = inf;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));

    /* Non-finite B. */
    cfg         = make_obs_cfg();
    cfg.B[1][0] = nan;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));

    /* Non-finite x0. */
    cfg       = make_obs_cfg();
    cfg.x0[0] = inf;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-008 — State Getter and Defensive Paths                        */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-008 | RON-FR-722 */
void test_ron_tc_ss_008(void)
{
    ron_obs_t obs;
    ron_obs_t fresh;
    ron_obs_config_t cfg = make_obs_cfg();
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    ron_float_t u[RON_SS_MAX_INPUTS];
    ron_float_t x_hat[RON_SS_MAX_STATES];

    fresh.state.is_initialised = false;

    /* Null-pointer rejection across the API. */
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_init(NULL, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_init(&obs, NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_reset(NULL));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_step(NULL, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_get_state(NULL, x_hat));

    /* Uninitialised-instance rejection. */
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_reset(&fresh));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_step(&fresh, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_get_state(&fresh, x_hat));

    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_get_state(&obs, NULL));

    /* Null y, and null u with p > 0. */
    y[0] = RON_FLOAT_C(1.0);
    u[0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_step(&obs, NULL, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NULL_POINTER, ron_obs_step(&obs, y, NULL));

    /* Non-finite measurement / input rejection. */
    y[0] = obs_make_inf();
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_obs_step(&obs, y, u));
    y[0] = RON_FLOAT_C(1.0);
    u[0] = obs_make_nan();
    TEST_ASSERT_EQUAL(RON_FAULT_INPUT_NAN, ron_obs_step(&obs, y, u));

    /* Getter returns the full estimate after one valid step. */
    u[0] = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_step(&obs, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.5), x_hat[0]);
    TEST_ASSERT_FLOAT_WITHIN(OBS_TOL, RON_FLOAT_C(0.2), x_hat[1]);
}

/* ----------------------------------------------------------------------- */
/* RON-TC-SS-009 — Compile-Time Dimension Bounds and Storage               */
/* ----------------------------------------------------------------------- */

/* RON-TC-SS-009 | RON-FR-723 */
void test_ron_tc_ss_009(void)
{
    ron_obs_t obs;
    ron_obs_config_t cfg;
    ron_float_t y[RON_SS_MAX_OUTPUTS];
    ron_float_t u[RON_SS_MAX_INPUTS];
    ron_float_t x_hat[RON_SS_MAX_STATES];
    uint8_t i;

    /* Maximum-dimension identity-style observer initialises and runs. */
    cfg = make_obs_cfg();
    for (i = 0U; i < (uint8_t) RON_SS_MAX_OUTPUTS; i++) {
        y[i] = RON_FLOAT_C(1.0);
    }
    for (i = 0U; i < (uint8_t) RON_SS_MAX_INPUTS; i++) {
        u[i] = RON_FLOAT_C(0.0);
    }
    cfg.n = (uint8_t) RON_SS_MAX_STATES;
    cfg.m = (uint8_t) RON_SS_MAX_OUTPUTS;
    cfg.p = (uint8_t) RON_SS_MAX_INPUTS;
    for (i = 0U; i < (uint8_t) RON_SS_MAX_STATES; i++) {
        cfg.A[i][i] = RON_FLOAT_C(1.0);
    }
    for (i = 0U; i < (uint8_t) RON_SS_MAX_OUTPUTS; i++) {
        cfg.C[i][i] = RON_FLOAT_C(1.0);
        cfg.L[i][i] = RON_FLOAT_C(0.1);
    }
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_step(&obs, y, u));
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_get_state(&obs, x_hat));

    /* Invalid dimensions rejected. */
    cfg   = make_obs_cfg();
    cfg.n = 0U;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));
    cfg.n = (uint8_t) (RON_SS_MAX_STATES + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));
    cfg   = make_obs_cfg();
    cfg.m = 0U;
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));
    cfg.m = (uint8_t) (RON_SS_MAX_OUTPUTS + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));
    cfg   = make_obs_cfg();
    cfg.p = (uint8_t) (RON_SS_MAX_INPUTS + 1U);
    TEST_ASSERT_EQUAL(RON_FAULT_CONFIG_INVALID, ron_obs_init(&obs, &cfg));

    /* Numeric overflow during the step yields RON_FAULT_OUTPUT_NAN. */
    cfg         = make_obs_cfg();
    cfg.n       = 1U;
    cfg.m       = 1U;
    cfg.p       = 0U;
    cfg.A[0][0] = RON_FLOAT_C(1.0e30);
    cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.L[0][0] = RON_FLOAT_C(0.0);
    cfg.x0[0]   = RON_FLOAT_C(1.0e30);
    TEST_ASSERT_EQUAL(RON_FAULT_NONE, ron_obs_init(&obs, &cfg));
    y[0] = RON_FLOAT_C(0.0);
    {
        ron_fault_t fault = RON_FAULT_NONE;
        uint8_t k;

        for (k = 0U; (k < 20U) && (fault == RON_FAULT_NONE); k++) {
            fault = ron_obs_step(&obs, y, NULL);
        }
        TEST_ASSERT_EQUAL(RON_FAULT_OUTPUT_NAN, fault);
    }
}

/* ----------------------------------------------------------------------- */
/* Test runner                                                             */
/* ----------------------------------------------------------------------- */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_ss_006);
    RUN_TEST(test_ron_tc_ss_007_convergence);
    RUN_TEST(test_ron_tc_ss_007_validation);
    RUN_TEST(test_ron_tc_ss_008);
    RUN_TEST(test_ron_tc_ss_009);
    return UNITY_END();
}
