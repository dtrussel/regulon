/*
 * @file     test_ron_metrics.c
 * @brief    Runtime performance metrics unit tests.
 * @module   test_ron_metrics
 * @doc      RON-TP-001
 * @req      RON-FR-950, RON-FR-951, RON-FR-952, RON-FR-953, RON-FR-954
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <float.h>

#include "ron/ron_metrics.h"

#include "test_ron_pid_common.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* =========================================================================
 * Test helpers
 * ========================================================================= */

static ron_float_t test_met_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

static ron_float_t test_met_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

static ron_float_t test_met_make_neg_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return -(big * big);
}

/* A valid cumulative baseline configuration. */
static ron_metrics_config_t test_met_default_cfg(void)
{
    ron_metrics_config_t cfg;

    cfg.mode           = RON_METRICS_CUMULATIVE;
    cfg.window_steps   = 0U;
    cfg.band_pct       = RON_FLOAT_C(0.02);
    cfg.settle_confirm = RON_FLOAT_C(0.05);
    cfg.step_thresh    = RON_FLOAT_C(0.5);
    return cfg;
}

/* =========================================================================
 * RON-TC-MET-001 | RON-FR-950 / FR-953 — Accumulator lifecycle & defensive paths
 * ========================================================================= */
void test_ron_tc_met_001(void)
{
    ron_metrics_t m;
    ron_metrics_t uninit;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t out;

    uninit.is_initialised = false;

    /* Null-pointer rejection on init. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_init(NULL, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_init(&m, NULL));

    /* Valid init succeeds, is initialised, and is DISABLED by default. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_TRUE(m.is_initialised);
    TEST_ASSERT_FALSE(m.enabled);

    /* Per-field configuration validation. */
    cfg      = test_met_default_cfg();
    cfg.mode = (ron_metrics_mode_t) 7;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));

    cfg              = test_met_default_cfg();
    cfg.mode         = RON_METRICS_WINDOWED;
    cfg.window_steps = 0U; /* windowed requires a positive window. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.window_steps = 8U; /* now valid. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));

    cfg          = test_met_default_cfg();
    cfg.band_pct = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.band_pct = RON_FLOAT_C(-0.1);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.band_pct = test_met_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.band_pct = test_met_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));

    cfg                = test_met_default_cfg();
    cfg.settle_confirm = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.settle_confirm = test_met_make_neg_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));

    cfg             = test_met_default_cfg();
    cfg.step_thresh = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));
    cfg.step_thresh = test_met_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_init(&m, &cfg));

    /* Defensive paths on the lifecycle / accessor API. */
    cfg = test_met_default_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_reset(&uninit));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_reset(&m));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_enable(NULL, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_enable(&uninit, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));
    TEST_ASSERT_TRUE(m.enabled);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_get(NULL, &out));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_get(&m, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_metrics_get(&uninit, &out));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));

    /* Defensive paths on step: null, uninitialised, bad dt, non-finite r/y. */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_metrics_step(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&uninit, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&m, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&m, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-0.01)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&m, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), test_met_make_nan()));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&m, test_met_make_nan(), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_metrics_step(&m, RON_FLOAT_C(0.0), test_met_make_inf(), RON_FLOAT_C(0.01)));
}

/* =========================================================================
 * RON-TC-MET-002 | RON-FR-951 — IAE / ISE / ITAE closed-form computation
 * ========================================================================= */
void test_ron_tc_met_002(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t out;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    unsigned k;

    /* Large step threshold so the constant setpoint never re-triggers. */
    cfg.step_thresh = RON_FLOAT_C(10.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));

    /* 100 steps with a constant error e = r - y = 0.5. */
    for (k = 0U; k < 100U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), dt));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));

    /* IAE  = 0.5  * 100 * 0.01            = 0.5
     * ISE  = 0.25 * 100 * 0.01            = 0.25
     * ITAE = 0.5 * 0.01 * 0.01 * Σk(1..100) = 0.5 * 0.0001 * 5050 = 0.2525 */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(5.0e-5), RON_FLOAT_C(0.5), out.IAE);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(2.5e-5), RON_FLOAT_C(0.25), out.ISE);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(2.5e-5), RON_FLOAT_C(0.2525), out.ITAE);

    /* No step was registered, so the transient metrics stay unset. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(-1.0), out.rise_time);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(-1.0), out.settling_time);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), out.peak_overshoot);
}

/* =========================================================================
 * RON-TC-MET-003 | RON-FR-951 — Peak-overshoot tracking
 * ========================================================================= */
void test_ron_tc_met_003(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t out;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    /* Step from 0 -> 1; PV rises, overshoots to 1.2 (20 %), then recovers. */
    static const ron_float_t y_seq[] = {
        RON_FLOAT_C(0.0),  RON_FLOAT_C(0.4), RON_FLOAT_C(0.8),  RON_FLOAT_C(1.0),
        RON_FLOAT_C(1.1),  RON_FLOAT_C(1.2), RON_FLOAT_C(1.15), RON_FLOAT_C(1.1),
        RON_FLOAT_C(1.05), RON_FLOAT_C(1.0), RON_FLOAT_C(1.0),  RON_FLOAT_C(1.0),
    };
    const unsigned n = (unsigned) (sizeof(y_seq) / sizeof(y_seq[0]));
    unsigned k;

    cfg.step_thresh = RON_FLOAT_C(0.5);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));

    for (k = 0U; k < n; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(1.0), y_seq[k], dt));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));
    /* Peak overshoot = (1.2 - 1.0) / 1.0 * 100 = 20 %. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.5), RON_FLOAT_C(20.0), out.peak_overshoot);
}

/* =========================================================================
 * RON-TC-MET-004 | RON-FR-951 — Rise-time and settling-time tracking
 * ========================================================================= */
void test_ron_tc_met_004(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t out;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.band_pct       = RON_FLOAT_C(0.02); /* band = 1.0 * 0.02 = 0.02. */
    cfg.settle_confirm = RON_FLOAT_C(0.05); /* dwell five samples within band. */
    cfg.step_thresh    = RON_FLOAT_C(0.5);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));

    /* Sample 1 establishes the step (ref = 0, target = 1, size = 1). */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_metrics_step(&m, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), dt));

    /* Linear ramp 0.05 -> 1.0: crosses 10 % at call 3, 90 % at call 19. */
    for (k = 2U; k <= 21U; ++k) {
        ron_float_t y = RON_FLOAT_C(0.05) * (ron_float_t) (k - 1U);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_step(&m, RON_FLOAT_C(1.0), y, dt));
    }
    /* Hold at the target so the loop settles. */
    for (k = 22U; k <= 40U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), dt));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));
    /* Rise: t(90 %) - t(10 %) = 0.19 - 0.03 = 0.16 s. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.02), RON_FLOAT_C(0.16), out.rise_time);
    /* Settling: PV is within band from call 21; confirmed five samples later. */
    TEST_ASSERT_TRUE(out.settling_time > RON_FLOAT_C(0.0));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.03), RON_FLOAT_C(0.25), out.settling_time);
}

/* =========================================================================
 * RON-TC-MET-005 | RON-FR-952 — Windowed vs cumulative accumulation
 * ========================================================================= */
void test_ron_tc_met_005(void)
{
    ron_metrics_t cum;
    ron_metrics_t win;
    ron_metrics_config_t ccfg = test_met_default_cfg();
    ron_metrics_config_t wcfg = test_met_default_cfg();
    ron_metrics_result_t cout;
    ron_metrics_result_t wout;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    unsigned k;

    ccfg.step_thresh  = RON_FLOAT_C(10.0);
    wcfg.step_thresh  = RON_FLOAT_C(10.0);
    wcfg.mode         = RON_METRICS_WINDOWED;
    wcfg.window_steps = 10U;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&cum, &ccfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&win, &wcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&cum, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&win, true));

    /* 25 steps of constant error 0.5: the window rolls every 10 samples. */
    for (k = 0U; k < 25U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&cum, RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), dt));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&win, RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), dt));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&cum, &cout));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&win, &wout));

    /* Cumulative: 0.5 * 0.01 * 25 = 0.125.  Windowed holds the last 5 samples:
     * 0.5 * 0.01 * 5 = 0.025. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), RON_FLOAT_C(0.125), cout.IAE);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), RON_FLOAT_C(0.025), wout.IAE);
    TEST_ASSERT_TRUE(wout.IAE < cout.IAE);
}

/* =========================================================================
 * RON-TC-MET-006 | RON-FR-953 — Zero overhead / no state change when disabled
 * ========================================================================= */
void test_ron_tc_met_006(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t before;
    ron_metrics_result_t after;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    const ron_float_t dt  = RON_FLOAT_C(0.01);
    const ron_float_t tau = RON_FLOAT_C(0.1);
    ron_float_t u_with[200];
    ron_float_t u_without[200];
    ron_status_t status;
    ron_float_t y;
    unsigned k;

    /* A disabled accumulator must not change state across many steps. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &before));
    for (k = 0U; k < 1000U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), dt));
    }
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &after));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.0), before.IAE, after.IAE);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.0), before.ISE, after.ISE);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.0), before.ITAE, after.ITAE);
    TEST_ASSERT_EQUAL_UINT(0U, m.window_counter);
    TEST_ASSERT_FALSE(m.prev_valid);

    /* The accumulator is passive: PID outputs are identical whether metrics
     * collection is enabled or disabled. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));
    y = RON_FLOAT_C(0.0);
    for (k = 0U; k < 200U; ++k) {
        ron_float_t u = RON_FLOAT_C(0.0);
        (void) ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &u, &status);
        (void) ron_metrics_step(&m, RON_FLOAT_C(1.0), y, dt);
        u_with[k] = u;
        y         = y + (dt / tau) * (u - y);
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg)); /* disabled. */
    y = RON_FLOAT_C(0.0);
    for (k = 0U; k < 200U; ++k) {
        ron_float_t u = RON_FLOAT_C(0.0);
        (void) ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &u, &status);
        (void) ron_metrics_step(&m, RON_FLOAT_C(1.0), y, dt);
        u_without[k] = u;
        y            = y + (dt / tau) * (u - y);
    }

    for (k = 0U; k < 200U; ++k) {
        TEST_ASSERT_TRUE(u_with[k] == u_without[k]);
    }
}

/* =========================================================================
 * RON-TC-MET-007 | RON-FR-954 — Setpoint-step detection restarts transients
 * ========================================================================= */
void test_ron_tc_met_007(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg = test_met_default_cfg();
    ron_metrics_result_t out;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    /* After the step (r: 0 -> 1) the PV overshoots to 1.2 then recovers. */
    static const ron_float_t y_seq[] = {
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.5),  RON_FLOAT_C(1.0), RON_FLOAT_C(1.2),
        RON_FLOAT_C(1.1), RON_FLOAT_C(1.05), RON_FLOAT_C(1.0), RON_FLOAT_C(1.0),
    };
    const unsigned n = (unsigned) (sizeof(y_seq) / sizeof(y_seq[0]));
    unsigned k;

    cfg.step_thresh = RON_FLOAT_C(0.5);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&m, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&m, true));

    /* Phase A: r == y == 0 → step_size 0, so transient metrics stay unset. */
    for (k = 0U; k < 5U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), dt));
    }
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(-1.0), out.rise_time);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(-1.0), out.settling_time);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), out.peak_overshoot);

    /* Phase B: a setpoint step (|Δr| = 1.0 ≥ 0.5) restarts the step frame. */
    for (k = 0U; k < n; ++k) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_metrics_step(&m, RON_FLOAT_C(1.0), y_seq[k], dt));
    }

    /* The reference frame snapped to the new step (white-box). */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(1.0), m.step_target);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), m.step_ref);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(1.0), m.step_size);

    /* Transient metrics are now measured against the post-step target. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&m, &out));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.5), RON_FLOAT_C(20.0), out.peak_overshoot);
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_met_001);
    RUN_TEST(test_ron_tc_met_002);
    RUN_TEST(test_ron_tc_met_003);
    RUN_TEST(test_ron_tc_met_004);
    RUN_TEST(test_ron_tc_met_005);
    RUN_TEST(test_ron_tc_met_006);
    RUN_TEST(test_ron_tc_met_007);
    return UNITY_END();
}
