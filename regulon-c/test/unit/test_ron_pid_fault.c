/*
 * @file     test_ron_pid_fault.c
 * @brief    Sprint 2 unit tests — fault manager, safe-state, NaN guards.
 * @module   test_ron_pid_fault
 * @doc      RON-IS-001
 * @req      RON-SR-010, RON-SR-011, RON-SR-012, RON-SR-020, RON-FR-035
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Test cases implemented here:
 *   RON-TC-SAFE-007  Fault bit latch + callback single-shot       | RON-SR-010
 *   RON-TC-SAFE-008  Safe-state policies (HOLD_LAST/ZERO/CONSTANT)| RON-SR-011
 *   RON-TC-SAFE-009  ron_pid_fault_clear clears the latch         | RON-SR-012
 *   RON-TC-SAFE-011  NaN/Inf guard on inputs and output           | RON-SR-020
 *   RON-TC-SAFE-013  Integral overflow threshold trips fault      | RON-FR-035
 */

#include "unity.h"
#include "ron/ron_pid.h"

/* =========================================================================
 * Helpers
 * ========================================================================= */

static unsigned int s_callback_count = 0U;
static ron_fault_t  s_last_fault     = RON_FAULT_NONE;

static void reset_callback_tracker(void)
{
    s_callback_count = 0U;
    s_last_fault     = RON_FAULT_NONE;
}

static void fault_cb(ron_fault_t f)
{
    ++s_callback_count;
    s_last_fault = f;
}

static ron_pid_config_t make_default_cfg(void)
{
    ron_pid_config_t cfg = {
        .Kp = RON_FLOAT_C(1.0),
        .Ki = RON_FLOAT_C(0.0),
        .Kd = RON_FLOAT_C(0.0),
        .N  = RON_FLOAT_C(0.0),
        .b  = RON_FLOAT_C(1.0),
        .c  = RON_FLOAT_C(1.0),
        .u_min  = RON_FLOAT_C(-100.0),
        .u_max  = RON_FLOAT_C( 100.0),
        .du_max = RON_FLOAT_C(0.0),
        .I_min  = RON_FLOAT_C(-100.0),
        .I_max  = RON_FLOAT_C( 100.0),
        .aw_mode      = RON_AW_NONE,
        .T_aw         = RON_FLOAT_C(0.1),
        .integ_method = RON_INTEG_EULER,
        .deriv_mode   = RON_DERIV_ON_MEASUREMENT,
        .tau_sp       = RON_FLOAT_C(0.0),
        .normalise    = false,
        .in_min  = RON_FLOAT_C(0.0),
        .in_max  = RON_FLOAT_C(1.0),
        .out_min = RON_FLOAT_C(0.0),
        .out_max = RON_FLOAT_C(1.0),
        .safe_policy = RON_SAFE_HOLD_LAST,
        .safe_value  = RON_FLOAT_C(0.0),
        .I_overflow_thresh  = RON_FLOAT_C(0.0),
        .sp_reset_threshold = RON_FLOAT_C(0.0),
        .fault_cb = NULL
    };
    return cfg;
}

void setUp(void)    { reset_callback_tracker(); }
void tearDown(void) { }

/* =========================================================================
 * RON-TC-SAFE-007 — Fault latch + callback single-shot.
 *
 * After an input-NaN fault is detected, subsequent calls with the same
 * fault condition must NOT re-invoke the callback (the bit is already
 * latched).  Only a fresh fault bit should call back.
 * ========================================================================= */

/* RON-TC-SAFE-007 | RON-SR-010 */
void test_ron_tc_safe_007_callback_single_shot(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.fault_cb = fault_cb;
    (void)ron_pid_init(&inst, &cfg);

    /* Build NaN via 0/0 — no <math.h>. */
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t nan_val = zero / zero;

    ron_float_t  u;
    ron_status_t status;
    ron_fault_t  f1 = ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                                   RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, f1);
    TEST_ASSERT_EQUAL_UINT(1U, s_callback_count);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, s_last_fault);
    TEST_ASSERT_TRUE((status & (ron_status_t)RON_STATUS_FAULT) != 0U);

    /* Second step — fault is latched; callback must NOT fire again. */
    (void)ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT(1U, s_callback_count);
}

/* =========================================================================
 * RON-TC-SAFE-008 — Safe-state policies (HOLD_LAST / ZERO / CONSTANT).
 * ========================================================================= */

/* RON-TC-SAFE-008 | RON-SR-011 */
void test_ron_tc_safe_008_safe_state_policies(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t nan_val = zero / zero;

    /* --- HOLD_LAST: the last good saturated output (42.0) must be held. --- */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        cfg.safe_policy = RON_SAFE_HOLD_LAST;
        (void)ron_pid_init(&inst, &cfg);
        /* Seed a known last output by running one clean step. */
        ron_float_t  u;
        ron_status_t status;
        (void)ron_pid_step(&inst, RON_FLOAT_C(42.0), RON_FLOAT_C(0.0),
                           RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4),
                                 RON_FLOAT_C(42.0), u);

        /* Now induce a fault: output must freeze at the last clean value. */
        (void)ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                           RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4),
                                 RON_FLOAT_C(42.0), u);
    }

    /* --- ZERO policy. --- */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        cfg.safe_policy = RON_SAFE_ZERO;
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        (void)ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                           RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6),
                                 RON_FLOAT_C(0.0), u);
    }

    /* --- CONSTANT policy drives u to safe_value, clamped to [u_min,u_max]. */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        cfg.safe_policy = RON_SAFE_CONSTANT;
        cfg.safe_value  = RON_FLOAT_C(-7.5);
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        (void)ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                           RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4),
                                 RON_FLOAT_C(-7.5), u);
    }
}

/* =========================================================================
 * RON-TC-SAFE-009 — ron_pid_fault_clear() clears the latched fault.
 * ========================================================================= */

/* RON-TC-SAFE-009 | RON-SR-012 */
void test_ron_tc_safe_009_fault_clear(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    (void)ron_pid_init(&inst, &cfg);

    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t nan_val = zero / zero;

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);

    ron_fault_t latched = RON_FAULT_NONE;
    (void)ron_pid_get_state(&inst, NULL, NULL, NULL, NULL, &latched);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, latched);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_fault_clear(&inst));

    (void)ron_pid_get_state(&inst, NULL, NULL, NULL, NULL, &latched);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, latched);

    /* After clearing, a clean step must succeed again. */
    ron_fault_t f = ron_pid_step(&inst, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                 RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, f);
}

/* =========================================================================
 * RON-TC-SAFE-011 — NaN/Inf guard on setpoint, measurement, and output.
 * ========================================================================= */

/* RON-TC-SAFE-011 | RON-SR-020 */
void test_ron_tc_safe_011_nan_guards(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    volatile ron_float_t big  = RON_FLOAT_MAX;
    ron_float_t nan_val = zero / zero;
    ron_float_t inf_val = big * big;

    /* NaN on setpoint. */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        ron_fault_t  f = ron_pid_step(&inst, nan_val, RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, f);
    }

    /* Inf on measurement. */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        ron_fault_t  f = ron_pid_step(&inst, RON_FLOAT_C(0.0), inf_val,
                                      RON_FLOAT_C(0.01), &u, &status);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, f);
    }

    /* Invalid dt (NaN). */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        ron_fault_t  f = ron_pid_step(&inst, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                      nan_val, &u, &status);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, f);
    }

    /* dt <= 0. */
    {
        ron_pid_instance_t inst;
        ron_pid_config_t   cfg = make_default_cfg();
        (void)ron_pid_init(&inst, &cfg);
        ron_float_t  u;
        ron_status_t status;
        ron_fault_t  f = ron_pid_step(&inst, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.0), &u, &status);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, f);
    }
}

/* =========================================================================
 * RON-TC-SAFE-013 — Integral overflow threshold trips a fault.
 *
 * With Ki=1, I_overflow_thresh=0.5, dt=1, e=1, after one step the
 * integral reaches 1.0 which exceeds the threshold; FAULT_INTEGRAL_OVERFLOW
 * is latched.
 * ========================================================================= */

/* RON-TC-SAFE-013 | RON-FR-035 */
void test_ron_tc_safe_013_integral_overflow(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.integ_method = RON_INTEG_EULER;
    cfg.I_overflow_thresh = RON_FLOAT_C(0.5);
    cfg.safe_policy = RON_SAFE_ZERO;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    ron_fault_t  f = ron_pid_step(&inst, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(1.0), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INTEGRAL_OVERFLOW, f);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-6), RON_FLOAT_C(0.0), u);

    ron_fault_t latched = RON_FAULT_NONE;
    (void)ron_pid_get_state(&inst, NULL, NULL, NULL, NULL, &latched);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INTEGRAL_OVERFLOW, latched);
}

/* =========================================================================
 * Unity test runner
 * ========================================================================= */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_safe_007_callback_single_shot);
    RUN_TEST(test_ron_tc_safe_008_safe_state_policies);
    RUN_TEST(test_ron_tc_safe_009_fault_clear);
    RUN_TEST(test_ron_tc_safe_011_nan_guards);
    RUN_TEST(test_ron_tc_safe_013_integral_overflow);
    return UNITY_END();
}
