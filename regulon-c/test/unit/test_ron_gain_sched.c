/*
 * @file     test_ron_gain_sched.c
 * @brief    Gain-scheduling unit tests.
 * @module   test_ron_gain_sched
 * @doc      RON-TP-001
 * @req      RON-FR-300, RON-FR-301, RON-FR-302, RON-FR-303,
 *           RON-FR-304, RON-FR-305, RON-FR-306
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_gain_sched.h"

#include "test_ron_pid_common.h"
#include "unity.h"

#define TEST_GS_TOL RON_FLOAT_C(1e-6)

void setUp(void)
{
}

void tearDown(void)
{
}

/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
static ron_pid_instance_t test_ron_gs_init_pid(const ron_pid_config_t *cfg)
{
    ron_pid_instance_t inst;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, cfg));
    return inst;
}

/* Satisfies: RON-FR-300 | Test: RON-TC-GS-001 */
static ron_gs_table_t test_ron_gs_make_table(ron_gs_mode_t mode, bool reset_integral)
{
    ron_gs_table_t table;

    table.n_points                 = 0U;
    table.mode                     = mode;
    table.reset_integral_on_switch = reset_integral;

    return table;
}

/* Satisfies: RON-FR-300 | Test: RON-TC-GS-001 */
static void test_ron_gs_set_point(ron_gs_table_t *table, uint8_t index, ron_float_t sigma,
                                  const ron_pid_config_t *cfg)
{
    table->sigma[index]   = sigma;
    table->configs[index] = *cfg;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_gs_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_gs_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_gs_make_neg_inf(void)
{
    return -test_ron_gs_make_inf();
}

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
static void test_ron_gs_fault_cb(ron_fault_t fault)
{
    (void) fault;
}

/* Satisfies: RON-FR-303 | Test: RON-TC-GS-005 */
static void test_ron_gs_expect_hard_switch_update(const ron_pid_config_t *base_cfg,
                                                  const ron_pid_config_t *next_cfg)
{
    ron_gs_table_t table = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_pid_instance_t pid;

    table.n_points = 2U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), base_cfg);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), next_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));
    pid = test_ron_gs_init_pid(base_cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(1.0)));
}

/* Satisfies: RON-FR-306 | Test: RON-TC-GS-008 */
static void test_ron_gs_expect_interp_invalid(const ron_pid_config_t *base_cfg,
                                              const ron_pid_config_t *next_cfg)
{
    ron_gs_table_t table = test_ron_gs_make_table(RON_GS_LINEAR_INTERP, false);

    table.n_points = 2U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), base_cfg);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), next_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&table));
}

/* RON-TC-GS-001 | RON-FR-300 */
void test_ron_tc_gs_001(void)
{
    ron_pid_config_t cfg_lo = test_ron_make_pid_cfg();
    ron_pid_config_t cfg_hi = test_ron_make_pid_cfg();
    ron_gs_table_t table    = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg_lo.Kp    = RON_FLOAT_C(1.0);
    cfg_lo.u_min = RON_FLOAT_C(-2.0);
    cfg_lo.u_max = RON_FLOAT_C(2.0);
    cfg_hi       = cfg_lo;
    cfg_hi.Kp    = RON_FLOAT_C(4.0);
    cfg_hi.u_min = RON_FLOAT_C(-4.0);
    cfg_hi.u_max = RON_FLOAT_C(4.0);

    table.n_points = 2U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), &cfg_lo);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), &cfg_hi);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));
    pid = test_ron_gs_init_pid(&cfg_lo);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(1.5)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(4.0), pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(-4.0), pid.config.u_min);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(4.0), pid.config.u_max);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(4.0), u);
}

/* RON-TC-GS-002 | RON-FR-301 */
void test_ron_tc_gs_002(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_gs_table_t valid = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t invalid_count;
    uint8_t index;

    valid.n_points = (uint8_t) RON_GS_MAX_BREAKPOINTS;
    index          = 0U;
    while (index < valid.n_points) {
        cfg.Kp = RON_FLOAT_C(1.0) + (ron_float_t) index;
        test_ron_gs_set_point(&valid, index, (ron_float_t) index, &cfg);
        index++;
    }

    invalid_count          = valid;
    invalid_count.n_points = (uint8_t) (RON_GS_MAX_BREAKPOINTS + 1U);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&valid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&invalid_count));
}

/* RON-TC-GS-003 | RON-FR-302 */
void test_ron_tc_gs_003(void)
{
    ron_pid_config_t cfg0 = test_ron_make_pid_cfg();
    ron_pid_config_t cfg1 = test_ron_make_pid_cfg();
    ron_pid_config_t cfg2 = test_ron_make_pid_cfg();
    ron_gs_table_t table  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg0.Kp = RON_FLOAT_C(1.0);
    cfg1.Kp = RON_FLOAT_C(2.0);
    cfg2.Kp = RON_FLOAT_C(3.0);

    table.n_points = 3U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), &cfg0);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), &cfg1);
    test_ron_gs_set_point(&table, 2U, RON_FLOAT_C(2.0), &cfg2);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));

    pid = test_ron_gs_init_pid(&cfg0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(0.5)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(1.0), pid.config.Kp);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(1.0), u);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(1.5)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(2.0), pid.config.Kp);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(2.0), u);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(2.5)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(3.0), pid.config.Kp);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(3.0), u);
}

/* RON-TC-GS-004 | RON-FR-302 */
void test_ron_tc_gs_004(void)
{
    ron_pid_config_t cfg0 = test_ron_make_pid_cfg();
    ron_pid_config_t cfg1 = test_ron_make_pid_cfg();
    ron_gs_table_t table  = test_ron_gs_make_table(RON_GS_LINEAR_INTERP, false);
    ron_pid_instance_t pid;

    cfg0.Kp = RON_FLOAT_C(1.0);
    cfg1.Kp = RON_FLOAT_C(3.0);

    table.n_points = 2U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), &cfg0);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), &cfg1);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));

    pid = test_ron_gs_init_pid(&cfg0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(-0.25)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-5), RON_FLOAT_C(1.0), pid.config.Kp);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(0.25)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-5), RON_FLOAT_C(1.5), pid.config.Kp);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(0.75)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-5), RON_FLOAT_C(2.5), pid.config.Kp);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(1.25)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-5), RON_FLOAT_C(3.0), pid.config.Kp);
}

/* RON-TC-GS-004 | RON-FR-302 */
void test_ron_tc_gs_004_single_point_interp_table(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_gs_table_t table = test_ron_gs_make_table(RON_GS_LINEAR_INTERP, false);
    ron_pid_instance_t pid;

    cfg.Kp         = RON_FLOAT_C(1.25);
    table.n_points = 1U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.5), &cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));
    pid = test_ron_gs_init_pid(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(0.75)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(1.25), pid.config.Kp);
}

/* RON-TC-GS-005 | RON-FR-303 */
void test_ron_tc_gs_005(void)
{
    ron_pid_config_t base_cfg    = test_ron_make_pid_cfg();
    ron_pid_config_t low_cfg     = test_ron_make_pid_cfg();
    ron_pid_config_t high_cfg    = test_ron_make_pid_cfg();
    ron_pid_config_t invalid_cfg = test_ron_make_pid_cfg();
    ron_gs_table_t atomic_table  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t invalid_table = test_ron_gs_make_table(RON_GS_LINEAR_INTERP, false);
    ron_pid_instance_t pid;

    base_cfg.Kp  = RON_FLOAT_C(0.5);
    low_cfg      = base_cfg;
    high_cfg     = base_cfg;
    high_cfg.Kp  = RON_FLOAT_C(3.0);
    high_cfg.u_min = RON_FLOAT_C(-2.0);
    high_cfg.u_max = RON_FLOAT_C(2.0);
    invalid_cfg  = high_cfg;
    invalid_cfg.aw_mode = RON_AW_BACK_CALC;

    atomic_table.n_points = 2U;
    test_ron_gs_set_point(&atomic_table, 0U, RON_FLOAT_C(0.0), &low_cfg);
    test_ron_gs_set_point(&atomic_table, 1U, RON_FLOAT_C(1.0), &high_cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&atomic_table));

    pid = test_ron_gs_init_pid(&base_cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&atomic_table, &pid, RON_FLOAT_C(1.0)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(3.0), pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(-2.0), pid.config.u_min);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(2.0), pid.config.u_max);

    invalid_table.n_points = 2U;
    test_ron_gs_set_point(&invalid_table, 0U, RON_FLOAT_C(0.0), &high_cfg);
    test_ron_gs_set_point(&invalid_table, 1U, RON_FLOAT_C(1.0), &invalid_cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_gs_update(&invalid_table, &pid, RON_FLOAT_C(0.5)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(3.0), pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(-2.0), pid.config.u_min);
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(2.0), pid.config.u_max);
}

/* RON-TC-GS-005 | RON-FR-303 */
void test_ron_tc_gs_005_config_compare_paths(void)
{
    ron_pid_config_t base_cfg = test_ron_make_pid_cfg();
    ron_pid_config_t next_cfg;

    base_cfg.Kp                = RON_FLOAT_C(1.0);
    base_cfg.Ki                = RON_FLOAT_C(0.5);
    base_cfg.Kd                = RON_FLOAT_C(0.25);
    base_cfg.N                 = RON_FLOAT_C(8.0);
    base_cfg.b                 = RON_FLOAT_C(0.8);
    base_cfg.c                 = RON_FLOAT_C(0.2);
    base_cfg.u_min             = RON_FLOAT_C(-10.0);
    base_cfg.u_max             = RON_FLOAT_C(10.0);
    base_cfg.du_max            = RON_FLOAT_C(20.0);
    base_cfg.I_min             = RON_FLOAT_C(-5.0);
    base_cfg.I_max             = RON_FLOAT_C(5.0);
    base_cfg.aw_mode           = RON_AW_BACK_CALC;
    base_cfg.T_aw              = RON_FLOAT_C(0.2);
    base_cfg.integ_method      = RON_INTEG_TRAPEZOIDAL;
    base_cfg.deriv_mode        = RON_DERIV_ON_MEASUREMENT;
    base_cfg.normalise         = false;
    base_cfg.safe_policy       = RON_SAFE_HOLD_LAST;
    base_cfg.tau_sp            = RON_FLOAT_C(0.1);
    base_cfg.in_min            = RON_FLOAT_C(-1.0);
    base_cfg.in_max            = RON_FLOAT_C(1.0);
    base_cfg.out_min           = RON_FLOAT_C(-10.0);
    base_cfg.out_max           = RON_FLOAT_C(10.0);
    base_cfg.safe_value        = RON_FLOAT_C(0.0);
    base_cfg.I_overflow_thresh = RON_FLOAT_C(100.0);
    base_cfg.sp_reset_threshold = RON_FLOAT_C(50.0);
    base_cfg.feedforward.mode  = RON_FF_STATIC_GAIN;
    base_cfg.feedforward.gain  = RON_FLOAT_C(0.5);
    base_cfg.feedforward.N_ff  = RON_FLOAT_C(4.0);
    base_cfg.fault_cb          = NULL;

    next_cfg = base_cfg;
    next_cfg.Ki = RON_FLOAT_C(0.6);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.Kd = RON_FLOAT_C(0.35);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.N = RON_FLOAT_C(9.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.b = RON_FLOAT_C(0.7);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.c = RON_FLOAT_C(0.3);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.u_min = RON_FLOAT_C(-9.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.u_max = RON_FLOAT_C(9.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.du_max = RON_FLOAT_C(19.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.I_min = RON_FLOAT_C(-4.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.I_max = RON_FLOAT_C(4.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.T_aw = RON_FLOAT_C(0.3);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.tau_sp = RON_FLOAT_C(0.2);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.in_min = RON_FLOAT_C(-2.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.in_max = RON_FLOAT_C(2.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.out_min = RON_FLOAT_C(-9.5);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.out_max = RON_FLOAT_C(9.5);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.safe_value = RON_FLOAT_C(0.25);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.I_overflow_thresh = RON_FLOAT_C(101.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.sp_reset_threshold = RON_FLOAT_C(49.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.integ_method = RON_INTEG_EULER;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.deriv_mode = RON_DERIV_ON_ERROR;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.normalise = true;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.safe_policy = RON_SAFE_ZERO;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.feedforward.mode = RON_FF_VELOCITY;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.feedforward.gain = RON_FLOAT_C(0.6);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.feedforward.N_ff = RON_FLOAT_C(5.0);
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.fault_cb = test_ron_gs_fault_cb;
    test_ron_gs_expect_hard_switch_update(&base_cfg, &next_cfg);
}

/* RON-TC-GS-006 | RON-FR-304 */
void test_ron_tc_gs_006(void)
{
    ron_pid_config_t cfg0 = test_ron_make_pid_cfg();
    ron_pid_config_t cfg1 = test_ron_make_pid_cfg();
    ron_gs_table_t table  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_pid_instance_t pid;
    ron_float_t u_before  = RON_FLOAT_C(0.0);
    ron_float_t u_after   = RON_FLOAT_C(0.0);
    ron_status_t status   = RON_STATUS_OK;

    cfg0.Kp = RON_FLOAT_C(1.0);
    cfg1.Kp = RON_FLOAT_C(5.0);

    table.n_points = 2U;
    test_ron_gs_set_point(&table, 0U, RON_FLOAT_C(0.0), &cfg0);
    test_ron_gs_set_point(&table, 1U, RON_FLOAT_C(1.0), &cfg1);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&table));

    pid = test_ron_gs_init_pid(&cfg0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u_before, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u_before, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(1.0), u_before);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&table, &pid, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.01), &u_after, &status));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(5.0), u_after);
}

/* RON-TC-GS-007 | RON-FR-305 */
void test_ron_tc_gs_007(void)
{
    ron_pid_config_t cfg0      = test_ron_make_pid_cfg();
    ron_pid_config_t cfg1      = test_ron_make_pid_cfg();
    ron_gs_table_t reset_table = test_ron_gs_make_table(RON_GS_HARD_SWITCH, true);
    ron_gs_table_t keep_table  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_pid_instance_t reset_pid;
    ron_pid_instance_t keep_pid;
    ron_float_t integral = RON_FLOAT_C(0.0);

    cfg0.Kp    = RON_FLOAT_C(0.0);
    cfg0.Ki    = RON_FLOAT_C(1.0);
    cfg0.I_min = RON_FLOAT_C(-10.0);
    cfg0.I_max = RON_FLOAT_C(10.0);
    cfg1       = cfg0;
    cfg1.Kp    = RON_FLOAT_C(2.0);

    reset_table.n_points = 2U;
    keep_table.n_points  = 2U;
    test_ron_gs_set_point(&reset_table, 0U, RON_FLOAT_C(0.0), &cfg0);
    test_ron_gs_set_point(&reset_table, 1U, RON_FLOAT_C(1.0), &cfg1);
    test_ron_gs_set_point(&keep_table, 0U, RON_FLOAT_C(0.0), &cfg0);
    test_ron_gs_set_point(&keep_table, 1U, RON_FLOAT_C(1.0), &cfg1);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&reset_table));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_init(&keep_table));

    reset_pid = test_ron_gs_init_pid(&cfg0);
    keep_pid  = test_ron_gs_init_pid(&cfg0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_set_integral(&reset_pid, RON_FLOAT_C(0.75)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_set_integral(&keep_pid, RON_FLOAT_C(0.75)));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_gs_update(&reset_table, &reset_pid, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_get_state(&reset_pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(0.75), integral);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&reset_table, &reset_pid, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_get_state(&reset_pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(0.0), integral);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_gs_update(&keep_table, &keep_pid, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_get_state(&keep_pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(TEST_GS_TOL, RON_FLOAT_C(0.75), integral);
}

/* RON-TC-GS-008 | RON-FR-306 */
void test_ron_tc_gs_008(void)
{
    ron_pid_config_t cfg        = test_ron_make_pid_cfg();
    ron_pid_config_t first_bad  = test_ron_make_pid_cfg();
    ron_pid_config_t second_bad = test_ron_make_pid_cfg();
    ron_gs_table_t unsorted     = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t zero_points  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t duplicate    = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t invalid_cfg  = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t invalid_mode = test_ron_gs_make_table((ron_gs_mode_t) 99, false);
    ron_gs_table_t first_sigma_nan = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t first_cfg_bad = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t second_sigma_nan = test_ron_gs_make_table(RON_GS_HARD_SWITCH, false);
    ron_gs_table_t interp_mixed = test_ron_gs_make_table(RON_GS_LINEAR_INTERP, false);
    ron_pid_instance_t pid      = test_ron_gs_init_pid(&cfg);
    ron_pid_instance_t uninit_pid = {0};
    ron_float_t nonfinite       = test_ron_gs_make_inf();

    unsorted.n_points = 2U;
    test_ron_gs_set_point(&unsorted, 0U, RON_FLOAT_C(1.0), &cfg);
    test_ron_gs_set_point(&unsorted, 1U, RON_FLOAT_C(0.0), &cfg);

    duplicate.n_points = 2U;
    test_ron_gs_set_point(&duplicate, 0U, RON_FLOAT_C(0.0), &cfg);
    test_ron_gs_set_point(&duplicate, 1U, RON_FLOAT_C(0.0), &cfg);

    invalid_cfg.n_points = 2U;
    cfg.Kp = RON_FLOAT_C(1.0);
    test_ron_gs_set_point(&invalid_cfg, 0U, RON_FLOAT_C(0.0), &cfg);
    cfg.Kp = RON_FLOAT_C(-1.0);
    test_ron_gs_set_point(&invalid_cfg, 1U, RON_FLOAT_C(1.0), &cfg);

    invalid_mode.n_points = 1U;
    cfg = test_ron_make_pid_cfg();
    test_ron_gs_set_point(&invalid_mode, 0U, RON_FLOAT_C(0.0), &cfg);

    first_sigma_nan.n_points = 1U;
    test_ron_gs_set_point(&first_sigma_nan, 0U, nonfinite, &cfg);

    first_bad.Kp = RON_FLOAT_C(-1.0);
    first_cfg_bad.n_points = 1U;
    test_ron_gs_set_point(&first_cfg_bad, 0U, RON_FLOAT_C(0.0), &first_bad);

    second_bad.Kp = RON_FLOAT_C(2.0);
    second_sigma_nan.n_points = 2U;
    test_ron_gs_set_point(&second_sigma_nan, 0U, RON_FLOAT_C(0.0), &cfg);
    test_ron_gs_set_point(&second_sigma_nan, 1U, nonfinite, &second_bad);

    interp_mixed.n_points = 2U;
    cfg                   = test_ron_make_pid_cfg();
    cfg.Kp                = RON_FLOAT_C(1.0);
    test_ron_gs_set_point(&interp_mixed, 0U, RON_FLOAT_C(0.0), &cfg);
    cfg.Kp                = RON_FLOAT_C(3.0);
    cfg.aw_mode           = RON_AW_BACK_CALC;
    test_ron_gs_set_point(&interp_mixed, 1U, RON_FLOAT_C(1.0), &cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_gs_init(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&zero_points));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&unsorted));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&duplicate));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&invalid_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&invalid_mode));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&first_sigma_nan));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&first_cfg_bad));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&second_sigma_nan));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_init(&interp_mixed));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_gs_update(&unsorted, NULL, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_gs_update(NULL, &pid, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_gs_update(&duplicate, &uninit_pid, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_gs_update(&unsorted, &pid, test_ron_gs_make_nan()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_gs_update(&unsorted, &pid, test_ron_gs_make_neg_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_gs_update(&unsorted, &pid, nonfinite));
}

/* RON-TC-GS-008 | RON-FR-306 */
void test_ron_tc_gs_008_interp_validation_groups(void)
{
    ron_pid_config_t base_cfg = test_ron_make_pid_cfg();
    ron_pid_config_t next_cfg;

    base_cfg.N                 = RON_FLOAT_C(8.0);
    base_cfg.u_min             = RON_FLOAT_C(-10.0);
    base_cfg.u_max             = RON_FLOAT_C(10.0);
    base_cfg.du_max            = RON_FLOAT_C(20.0);
    base_cfg.I_min             = RON_FLOAT_C(-5.0);
    base_cfg.I_max             = RON_FLOAT_C(5.0);
    base_cfg.aw_mode           = RON_AW_BACK_CALC;
    base_cfg.T_aw              = RON_FLOAT_C(0.2);
    base_cfg.tau_sp            = RON_FLOAT_C(0.1);
    base_cfg.in_min            = RON_FLOAT_C(-1.0);
    base_cfg.in_max            = RON_FLOAT_C(1.0);
    base_cfg.out_min           = RON_FLOAT_C(-10.0);
    base_cfg.out_max           = RON_FLOAT_C(10.0);
    base_cfg.I_overflow_thresh = RON_FLOAT_C(100.0);
    base_cfg.sp_reset_threshold = RON_FLOAT_C(50.0);
    base_cfg.feedforward.mode  = RON_FF_STATIC_GAIN;
    base_cfg.feedforward.gain  = RON_FLOAT_C(0.5);
    base_cfg.feedforward.N_ff  = RON_FLOAT_C(4.0);

    next_cfg = base_cfg;
    next_cfg.N = RON_FLOAT_C(9.0);
    test_ron_gs_expect_interp_invalid(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.u_min = RON_FLOAT_C(-9.0);
    test_ron_gs_expect_interp_invalid(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.in_min = RON_FLOAT_C(-2.0);
    test_ron_gs_expect_interp_invalid(&base_cfg, &next_cfg);

    next_cfg = base_cfg;
    next_cfg.feedforward.mode = RON_FF_VELOCITY;
    test_ron_gs_expect_interp_invalid(&base_cfg, &next_cfg);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_gs_001);
    RUN_TEST(test_ron_tc_gs_002);
    RUN_TEST(test_ron_tc_gs_003);
    RUN_TEST(test_ron_tc_gs_004);
    RUN_TEST(test_ron_tc_gs_004_single_point_interp_table);
    RUN_TEST(test_ron_tc_gs_005);
    RUN_TEST(test_ron_tc_gs_005_config_compare_paths);
    RUN_TEST(test_ron_tc_gs_006);
    RUN_TEST(test_ron_tc_gs_007);
    RUN_TEST(test_ron_tc_gs_008);
    RUN_TEST(test_ron_tc_gs_008_interp_validation_groups);
    return UNITY_END();
}
