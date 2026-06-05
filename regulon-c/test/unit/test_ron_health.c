/*
 * @file     test_ron_health.c
 * @brief    Control-loop health monitor unit tests.
 * @module   test_ron_health
 * @doc      RON-TP-001
 * @req      RON-FR-900, RON-FR-901, RON-FR-902,
 *           RON-FR-903, RON-FR-904, RON-FR-905
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <float.h>

#include "ron/ron_health.h"

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

static ron_float_t test_health_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

static ron_float_t test_health_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

static ron_float_t test_health_make_neg_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return -(big * big);
}

/* A valid baseline configuration with every threshold set high (non-tripping). */
static ron_health_config_t test_health_default_cfg(void)
{
    ron_health_config_t cfg;

    cfg.t_sat_max          = RON_FLOAT_C(100.0);
    cfg.err_diverge_thresh = RON_FLOAT_C(100.0);
    cfg.osc_count_thresh   = (uint8_t) 31U;
    cfg.dead_band          = RON_FLOAT_C(1.0e-4);
    cfg.dropout_time       = RON_FLOAT_C(100.0);
    cfg.ss_err_thresh      = RON_FLOAT_C(100.0);
    cfg.settling_time      = RON_FLOAT_C(100.0);
    cfg.cb                 = NULL;
    return cfg;
}

/* Module-scope callback bookkeeping (reset at the start of each cb test). */
static unsigned g_cb_count;
static ron_health_status_t g_cb_last;

static void test_health_cb(ron_health_status_t condition)
{
    g_cb_count += 1U;
    g_cb_last = condition;
}

static bool test_health_bit(const ron_health_t *h, ron_health_status_t bit)
{
    return (bool) ((h->state.status & bit) != 0U);
}

/* =========================================================================
 * RON-TC-HLTH-001 | RON-FR-900 — Attach / init lifecycle and defensive paths
 * ========================================================================= */
void test_ron_tc_hlth_001(void)
{
    ron_health_t h;
    ron_health_t uninit;
    ron_health_config_t cfg = test_health_default_cfg();
    ron_health_status_t status;

    /* Null-pointer rejection on init. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_init(NULL, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_init(&h, NULL));

    /* Valid init succeeds and leaves a healthy, initialised instance. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));
    TEST_ASSERT_TRUE(h.state.is_initialised);
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OK, h.state.status);

    /* Per-field configuration validation (RON-FR-902). */
    cfg           = test_health_default_cfg();
    cfg.t_sat_max = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.t_sat_max = test_health_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.t_sat_max = test_health_make_neg_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.t_sat_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg                    = test_health_default_cfg();
    cfg.err_diverge_thresh = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.err_diverge_thresh = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg                  = test_health_default_cfg();
    cfg.osc_count_thresh = (uint8_t) RON_HEALTH_OSC_WINDOW;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg           = test_health_default_cfg();
    cfg.dead_band = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.dead_band = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg              = test_health_default_cfg();
    cfg.dropout_time = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.dropout_time = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg               = test_health_default_cfg();
    cfg.ss_err_thresh = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.ss_err_thresh = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    cfg               = test_health_default_cfg();
    cfg.settling_time = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));
    cfg.settling_time = test_health_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_init(&h, &cfg));

    /* Defensive paths on the accessor / mutator API. */
    uninit.state.is_initialised = false;
    cfg                         = test_health_default_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_get(NULL, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_get(&h, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_get(&uninit, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_get(&h, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OK, status);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_clear(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_health_clear(&uninit));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_clear(&h));

    /* Defensive paths on step (null, uninitialised, bad dt, non-finite I/O). */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_health_step(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&uninit, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(-0.01)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), test_health_make_nan()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, test_health_make_nan(), RON_FLOAT_C(0.0),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, RON_FLOAT_C(0.0), test_health_make_inf(),
                                            RON_FLOAT_C(0.0), RON_FLOAT_C(0.01)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                            test_health_make_nan(), RON_FLOAT_C(0.01)));
}

/* =========================================================================
 * RON-TC-HLTH-002 | RON-FR-901(a) — Output-stuck detection at the threshold
 * ========================================================================= */
void test_ron_tc_hlth_002(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.t_sat_max = RON_FLOAT_C(0.5); /* 0.5 / 0.01 = 50-step threshold. */
    cfg.dead_band = RON_FLOAT_C(1.0e-3);
    cfg.cb        = test_health_cb;
    g_cb_count    = 0U;
    g_cb_last     = RON_HEALTH_OK;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Output held constant (stuck); y wiggles so dropout never trips; r == y so
     * the error is zero and no other condition can activate. */
    for (k = 1U; k <= 60U; ++k) {
        ron_float_t y = ((k % 2U) == 0U) ? RON_FLOAT_C(0.0) : RON_FLOAT_C(0.01);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, y, y, RON_FLOAT_C(5.0), dt));

        if (k < 50U) {
            TEST_ASSERT_FALSE(test_health_bit(&h, RON_HEALTH_OUTPUT_STUCK));
        } else {
            TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_OUTPUT_STUCK));
        }
    }

    /* Exactly one condition (output-stuck) ever activated. */
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OUTPUT_STUCK, h.state.status);
    TEST_ASSERT_EQUAL_UINT(1U, g_cb_count);
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OUTPUT_STUCK, g_cb_last);
}

/* =========================================================================
 * RON-TC-HLTH-003 | RON-FR-901(b) — Divergence detection (large and growing)
 * ========================================================================= */
void test_ron_tc_hlth_003(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.err_diverge_thresh = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Error grows linearly: e = 0.1*k, same sign, magnitude increasing. */
    for (k = 1U; k <= 20U; ++k) {
        ron_float_t y = -(RON_FLOAT_C(0.1) * (ron_float_t) k);
        ron_float_t u = RON_FLOAT_C(0.5) * (ron_float_t) k;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), y, u, dt));
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_DIVERGING));

    /* Large but shrinking error exercises the "not growing" branch. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-2.0),
                                                            RON_FLOAT_C(1.0), dt));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-1.5),
                                                            RON_FLOAT_C(1.0), dt));

    /* No other condition should have activated. */
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_DIVERGING, h.state.status);
}

/* =========================================================================
 * RON-TC-HLTH-004 | RON-FR-901(c) — Oscillation detection over the window
 * ========================================================================= */
void test_ron_tc_hlth_004(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.osc_count_thresh = (uint8_t) 5U;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Alternating error sign: one sign change per step. */
    for (k = 1U; k <= 10U; ++k) {
        ron_float_t y = ((k % 2U) == 0U) ? RON_FLOAT_C(2.0) : RON_FLOAT_C(-2.0);
        ron_float_t u = RON_FLOAT_C(0.3) * (ron_float_t) k;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), y, u, dt));
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_OSCILLATING));

    /* Two consecutive same-sign samples exercise the "no change" window branch. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-2.0),
                                                            RON_FLOAT_C(3.0), dt));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-2.0),
                                                            RON_FLOAT_C(3.1), dt));

    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OSCILLATING, h.state.status);
}

/* =========================================================================
 * RON-TC-HLTH-005 | RON-FR-901(d) — Sensor dropout detection
 * ========================================================================= */
void test_ron_tc_hlth_005(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.dead_band    = RON_FLOAT_C(0.01);
    cfg.dropout_time = RON_FLOAT_C(0.5); /* ~50-step threshold. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Measurement frozen at 2.0; setpoint matches so the error stays zero. */
    for (k = 1U; k <= 60U; ++k) {
        ron_float_t u = RON_FLOAT_C(0.3) * (ron_float_t) k;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_health_step(&h, RON_FLOAT_C(2.0), RON_FLOAT_C(2.0), u, dt));

        if (k == 40U) {
            TEST_ASSERT_FALSE(test_health_bit(&h, RON_HEALTH_SENSOR_DROPOUT));
        }
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_SENSOR_DROPOUT));
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_SENSOR_DROPOUT, h.state.status);
}

/* =========================================================================
 * RON-TC-HLTH-006 | RON-FR-901(e) — Setpoint-unreachable detection
 * ========================================================================= */
void test_ron_tc_hlth_006(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.dead_band     = RON_FLOAT_C(1.0e-4);
    cfg.ss_err_thresh = RON_FLOAT_C(0.5);
    cfg.settling_time = RON_FLOAT_C(0.5); /* ~50-step threshold. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Setpoint = 1.0, measurement stuck near zero: a persistent ~1.0 error.
     * y wiggles slightly so dropout cannot trip. */
    for (k = 1U; k <= 60U; ++k) {
        ron_float_t y = ((k % 2U) == 0U) ? RON_FLOAT_C(0.0) : RON_FLOAT_C(1.0e-3);
        ron_float_t u = RON_FLOAT_C(0.3) * (ron_float_t) k;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(1.0), y, u, dt));

        if (k == 40U) {
            TEST_ASSERT_FALSE(test_health_bit(&h, RON_HEALTH_SP_UNREACHABLE));
        }
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_SP_UNREACHABLE));
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_SP_UNREACHABLE, h.state.status);
}

/* =========================================================================
 * RON-TC-HLTH-007 | RON-FR-902 — Independently configurable thresholds
 * ========================================================================= */
void test_ron_tc_hlth_007(void)
{
    ron_health_t low;
    ron_health_t high;
    ron_health_config_t cfg_low  = test_health_default_cfg();
    ron_health_config_t cfg_high = test_health_default_cfg();
    const ron_float_t dt         = RON_FLOAT_C(0.01);
    unsigned k;

    cfg_low.err_diverge_thresh  = RON_FLOAT_C(1.5);
    cfg_high.err_diverge_thresh = RON_FLOAT_C(100.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&low, &cfg_low));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&high, &cfg_high));

    /* The identical growing-error stimulus drives both monitors. */
    for (k = 1U; k <= 30U; ++k) {
        ron_float_t y = -(RON_FLOAT_C(0.1) * (ron_float_t) k);
        ron_float_t u = RON_FLOAT_C(0.5) * (ron_float_t) k;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&low, RON_FLOAT_C(0.0), y, u, dt));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&high, RON_FLOAT_C(0.0), y, u, dt));
    }

    /* Only the low-threshold monitor reports divergence; nothing else trips. */
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_DIVERGING, low.state.status);
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OK, high.state.status);
}

/* =========================================================================
 * RON-TC-HLTH-008 | RON-FR-903 — The monitor does not modify the controller
 * ========================================================================= */
void test_ron_tc_hlth_008(void)
{
    ron_health_t mon;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg   = test_ron_make_pid_cfg();
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    const ron_float_t tau   = RON_FLOAT_C(0.1);
    ron_float_t u_with[200];
    ron_float_t u_without[200];
    ron_float_t y;
    ron_status_t status;
    unsigned k;

    /* Arm every condition so the monitor is doing maximal work. */
    cfg.t_sat_max          = RON_FLOAT_C(0.05);
    cfg.err_diverge_thresh = RON_FLOAT_C(0.1);
    cfg.osc_count_thresh   = (uint8_t) 1U;
    cfg.dead_band          = RON_FLOAT_C(1.0);
    cfg.dropout_time       = RON_FLOAT_C(0.05);
    cfg.ss_err_thresh      = RON_FLOAT_C(0.01);
    cfg.settling_time      = RON_FLOAT_C(0.05);

    /* Run 1: closed loop WITH an attached health monitor. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&mon, &cfg));
    y = RON_FLOAT_C(0.0);
    for (k = 0U; k < 200U; ++k) {
        ron_float_t u = RON_FLOAT_C(0.0);
        (void) ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &u, &status);
        (void) ron_health_step(&mon, RON_FLOAT_C(1.0), y, u, dt);
        u_with[k] = u;
        y         = y + (dt / tau) * (u - y);
    }

    /* Run 2: identical closed loop WITHOUT the monitor. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    y = RON_FLOAT_C(0.0);
    for (k = 0U; k < 200U; ++k) {
        ron_float_t u = RON_FLOAT_C(0.0);
        (void) ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &u, &status);
        u_without[k] = u;
        y            = y + (dt / tau) * (u - y);
    }

    /* The controller output sequence is bit-identical either way. */
    for (k = 0U; k < 200U; ++k) {
        TEST_ASSERT_TRUE(u_with[k] == u_without[k]);
    }
}

/* =========================================================================
 * RON-TC-HLTH-009 | RON-FR-904 — Callback fires once per first activation
 * ========================================================================= */
void test_ron_tc_hlth_009(void)
{
    ron_health_t h;
    ron_health_t silent;
    ron_health_config_t cfg = test_health_default_cfg();
    ron_health_config_t scfg;
    const ron_float_t dt = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.t_sat_max    = RON_FLOAT_C(0.3); /* stuck trips first (~step 30).  */
    cfg.dead_band    = RON_FLOAT_C(0.01);
    cfg.dropout_time = RON_FLOAT_C(0.6); /* dropout trips later (~step 60). */
    cfg.cb           = test_health_cb;
    g_cb_count       = 0U;
    g_cb_last        = RON_HEALTH_OK;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Output and measurement both frozen: stuck, then dropout. */
    for (k = 1U; k <= 80U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE,
            ron_health_step(&h, RON_FLOAT_C(2.0), RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), dt));
        if (k == 35U) {
            TEST_ASSERT_EQUAL_UINT(1U, g_cb_count);
            TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OUTPUT_STUCK, g_cb_last);
        }
    }

    /* Two distinct conditions activated → exactly two callbacks, no re-fire. */
    TEST_ASSERT_EQUAL_UINT(2U, g_cb_count);
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_SENSOR_DROPOUT, g_cb_last);

    /* A NULL callback is tolerated (no dispatch, condition still latches). */
    scfg           = test_health_default_cfg();
    scfg.t_sat_max = RON_FLOAT_C(0.1);
    scfg.cb        = NULL;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&silent, &scfg));
    for (k = 1U; k <= 40U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE,
            ron_health_step(&silent, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(5.0), dt));
    }
    TEST_ASSERT_TRUE(test_health_bit(&silent, RON_HEALTH_OUTPUT_STUCK));
}

/* =========================================================================
 * RON-TC-HLTH-010 | RON-FR-905 — Status latches until explicitly cleared
 * ========================================================================= */
void test_ron_tc_hlth_010(void)
{
    ron_health_t h;
    ron_health_config_t cfg = test_health_default_cfg();
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    ron_health_status_t status;
    unsigned k;

    cfg.err_diverge_thresh = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&h, &cfg));

    /* Trip divergence. */
    for (k = 1U; k <= 20U; ++k) {
        ron_float_t y = -(RON_FLOAT_C(0.1) * (ron_float_t) k);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_health_step(&h, RON_FLOAT_C(0.0), y, RON_FLOAT_C(1.0), dt));
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_DIVERGING));

    /* Drive the error to zero: the condition is no longer active, yet it must
     * remain latched (RON-FR-905). */
    for (k = 0U; k < 10U; ++k) {
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE,
            ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), dt));
        TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_DIVERGING));
    }

    /* Re-asserting an already-latched condition does not toggle anything. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-3.0),
                                                            RON_FLOAT_C(1.0), dt));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_step(&h, RON_FLOAT_C(0.0), RON_FLOAT_C(-6.0),
                                                            RON_FLOAT_C(1.0), dt));
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_DIVERGING));

    /* Explicit clear resets the status and the detector state. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_clear(&h));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_get(&h, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_HEALTH_OK, status);

    /* The cleared monitor can detect the condition afresh. */
    for (k = 1U; k <= 20U; ++k) {
        ron_float_t y = -(RON_FLOAT_C(0.1) * (ron_float_t) k);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_health_step(&h, RON_FLOAT_C(0.0), y, RON_FLOAT_C(1.0), dt));
    }
    TEST_ASSERT_TRUE(test_health_bit(&h, RON_HEALTH_DIVERGING));
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_hlth_001);
    RUN_TEST(test_ron_tc_hlth_002);
    RUN_TEST(test_ron_tc_hlth_003);
    RUN_TEST(test_ron_tc_hlth_004);
    RUN_TEST(test_ron_tc_hlth_005);
    RUN_TEST(test_ron_tc_hlth_006);
    RUN_TEST(test_ron_tc_hlth_007);
    RUN_TEST(test_ron_tc_hlth_008);
    RUN_TEST(test_ron_tc_hlth_009);
    RUN_TEST(test_ron_tc_hlth_010);
    return UNITY_END();
}
