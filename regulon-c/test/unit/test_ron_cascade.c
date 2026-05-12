/*
 * @file     test_ron_cascade.c
 * @brief    Cascade PID controller unit tests.
 * @module   test_ron_cascade
 * @doc      RON-TP-001
 * @req      RON-FR-400, RON-FR-401, RON-FR-402, RON-FR-403,
 *           RON-FR-404, RON-FR-405, RON-FR-406
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <float.h>

#include "ron/ron_cascade.h"

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

static ron_float_t test_ron_casc_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

static ron_float_t test_ron_casc_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

static ron_float_t test_ron_casc_make_neg_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return -(big * big);
}

/*
 * Return a simple proportional-only config with ±100 limits, suitable for
 * composing without special requirements.
 */
static ron_pid_config_t test_ron_casc_default_cfg(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();

    cfg.Kp    = RON_FLOAT_C(1.0);
    cfg.Ki    = RON_FLOAT_C(0.0);
    cfg.Kd    = RON_FLOAT_C(0.0);
    cfg.u_min = RON_FLOAT_C(-100.0);
    cfg.u_max = RON_FLOAT_C(100.0);
    cfg.I_min = RON_FLOAT_C(-100.0);
    cfg.I_max = RON_FLOAT_C(100.0);
    return cfg;
}

static ron_cascade_instance_t test_ron_casc_init(const ron_pid_config_t *outer_cfg,
                                                 const ron_pid_config_t *inner_cfg)
{
    ron_cascade_instance_t casc;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&casc, outer_cfg, inner_cfg));
    return casc;
}

/* =========================================================================
 * RON-TC-CASC-001 | RON-FR-400 — Init: valid configs succeed
 * ========================================================================= */
void test_ron_tc_casc_001(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;

    outer_cfg.Kp = RON_FLOAT_C(2.0);
    inner_cfg.Kp = RON_FLOAT_C(3.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&casc, &outer_cfg, &inner_cfg));
    TEST_ASSERT_TRUE(casc.outer.state.is_initialised);
    TEST_ASSERT_TRUE(casc.inner.state.is_initialised);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(2.0), casc.outer.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(3.0), casc.inner.config.Kp);
}

/* =========================================================================
 * RON-TC-CASC-002 | RON-FR-400 — Init: NULL and invalid config rejection
 * ========================================================================= */
void test_ron_tc_casc_002(void)
{
    ron_pid_config_t outer_cfg  = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg  = test_ron_casc_default_cfg();
    ron_pid_config_t bad_cfg    = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc = {0};

    /* NULL pointer checks */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_init(NULL, &outer_cfg, &inner_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_init(&casc, NULL, &inner_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_init(&casc, &outer_cfg, NULL));

    /* Invalid outer config: u_min >= u_max */
    bad_cfg.u_min = RON_FLOAT_C(10.0);
    bad_cfg.u_max = RON_FLOAT_C(10.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_init(&casc, &bad_cfg, &inner_cfg));

    /* Invalid inner config */
    bad_cfg       = test_ron_casc_default_cfg();
    bad_cfg.u_min = RON_FLOAT_C(5.0);
    bad_cfg.u_max = RON_FLOAT_C(5.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_init(&casc, &outer_cfg, &bad_cfg));
    TEST_ASSERT_FALSE(casc.inner.state.is_initialised);
}

/* =========================================================================
 * RON-TC-CASC-003 | RON-FR-401 — Nominal step: outer output feeds inner
 * ========================================================================= */
void test_ron_tc_casc_003(void)
{
    /*
     * Outer: Kp=1, limits ±50.  Error=10 → u_outer=10.
     * Inner: Kp=2, limits ±100. Error=10 (r_inner=10, y_in=0) → u_inner=20.
     */
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;

    outer_cfg.Kp    = RON_FLOAT_C(1.0);
    outer_cfg.u_min = RON_FLOAT_C(-50.0);
    outer_cfg.u_max = RON_FLOAT_C(50.0);
    inner_cfg.Kp    = RON_FLOAT_C(2.0);

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(20.0), u);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, RON_CASCADE_STATUS_OUTER(status));
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, RON_CASCADE_STATUS_INNER(status));
}

/* =========================================================================
 * RON-TC-CASC-004 | RON-FR-402 — Outer output clamped before inner
 * ========================================================================= */
void test_ron_tc_casc_004(void)
{
    /*
     * Outer: Kp=100, limits ±5.  Raw output=1000 → clamped to 5.
     * Inner: Kp=1,   limits ±100.  r_inner=5 → u_inner=5.
     */
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;

    outer_cfg.Kp    = RON_FLOAT_C(100.0);
    outer_cfg.u_min = RON_FLOAT_C(-5.0);
    outer_cfg.u_max = RON_FLOAT_C(5.0);
    inner_cfg.Kp    = RON_FLOAT_C(1.0);

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(5.0), u);
    TEST_ASSERT_TRUE((RON_CASCADE_STATUS_OUTER(status) & RON_STATUS_SATURATED) != 0U);
}

/* =========================================================================
 * RON-TC-CASC-005 | RON-FR-406 — Status bits are distinct per loop
 * ========================================================================= */
void test_ron_tc_casc_005(void)
{
    /*
     * Outer: Kp=1, limits ±1000 (will not saturate).
     * Inner: Kp=50, limits ±2 (will saturate on r_inner=1.0).
     */
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;

    outer_cfg.Kp    = RON_FLOAT_C(1.0);
    outer_cfg.u_min = RON_FLOAT_C(-1000.0);
    outer_cfg.u_max = RON_FLOAT_C(1000.0);
    inner_cfg.Kp    = RON_FLOAT_C(50.0);
    inner_cfg.u_min = RON_FLOAT_C(-2.0);
    inner_cfg.u_max = RON_FLOAT_C(2.0);

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    /* Inner saturated, outer was not */
    TEST_ASSERT_TRUE((RON_CASCADE_STATUS_INNER(status) & RON_STATUS_SATURATED) != 0U);
    TEST_ASSERT_TRUE((RON_CASCADE_STATUS_OUTER(status) & RON_STATUS_SATURATED) == 0U);
}

/* =========================================================================
 * RON-TC-CASC-006 | RON-FR-403 — Cross-loop back-calc AW propagation
 * ========================================================================= */
void test_ron_tc_casc_006(void)
{
    /*
     * Outer: Ki=10, back-calc AW with T_aw=0.1.  Without cascade AW the
     * outer integral winds up freely.  With cascade AW (inner saturated) the
     * correction restrains the outer integral.
     */
    ron_pid_config_t outer_aw   = test_ron_casc_default_cfg();
    ron_pid_config_t outer_noaw = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg  = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc_aw;
    ron_cascade_instance_t casc_noaw;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;
    ron_float_t I_aw;
    ron_float_t I_noaw;
    unsigned i;

    outer_aw.Kp      = RON_FLOAT_C(1.0);
    outer_aw.Ki      = RON_FLOAT_C(10.0);
    outer_aw.u_min   = RON_FLOAT_C(-100.0);
    outer_aw.u_max   = RON_FLOAT_C(100.0);
    outer_aw.I_min   = RON_FLOAT_C(-500.0);
    outer_aw.I_max   = RON_FLOAT_C(500.0);
    outer_aw.aw_mode = RON_AW_BACK_CALC;
    outer_aw.T_aw    = RON_FLOAT_C(0.1);

    outer_noaw         = outer_aw;
    outer_noaw.aw_mode = RON_AW_NONE;

    inner_cfg.Kp    = RON_FLOAT_C(1.0);
    inner_cfg.Ki    = RON_FLOAT_C(0.0);
    inner_cfg.u_min = RON_FLOAT_C(-0.5);
    inner_cfg.u_max = RON_FLOAT_C(0.5);

    casc_aw   = test_ron_casc_init(&outer_aw, &inner_cfg);
    casc_noaw = test_ron_casc_init(&outer_noaw, &inner_cfg);

    for (i = 0U; i < 20U; ++i) {
        (void) ron_cascade_step(&casc_aw, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                RON_FLOAT_C(0.01), &u, &status);
        (void) ron_cascade_step(&casc_noaw, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                RON_FLOAT_C(0.01), &u, &status);
    }

    (void) ron_pid_get_state(&casc_aw.outer, &I_aw, NULL, NULL, NULL, NULL);
    (void) ron_pid_get_state(&casc_noaw.outer, &I_noaw, NULL, NULL, NULL, NULL);

    /* AW version must have smaller magnitude outer integral */
    TEST_ASSERT_TRUE(test_ron_abs(I_aw) < test_ron_abs(I_noaw));
}

/* =========================================================================
 * RON-TC-CASC-007 | RON-FR-404 — AUTO→MANUAL: both loops enter manual
 * ========================================================================= */
void test_ron_tc_casc_007(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;
    unsigned i;

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    /* Run a few auto steps */
    for (i = 0U; i < 5U; ++i) {
        (void) ron_cascade_step(&casc, RON_FLOAT_C(5.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                RON_FLOAT_C(0.01), &u, &status);
    }

    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE,
        ron_cascade_set_mode(&casc, RON_MODE_MANUAL, RON_FLOAT_C(3.0), RON_FLOAT_C(7.0)));

    /* Both loops in manual */
    TEST_ASSERT_EQUAL_INT(RON_MODE_MANUAL, (int) casc.outer.state.mode);
    TEST_ASSERT_EQUAL_INT(RON_MODE_MANUAL, (int) casc.inner.state.mode);

    /* Step returns the manual_inner value as output */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(5.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));

    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(3.0), u);
    TEST_ASSERT_TRUE((RON_CASCADE_STATUS_INNER(status) & RON_STATUS_MANUAL_MODE) != 0U);
    TEST_ASSERT_TRUE((RON_CASCADE_STATUS_OUTER(status) & RON_STATUS_MANUAL_MODE) != 0U);
}

/* =========================================================================
 * RON-TC-CASC-008 | RON-FR-404 — MANUAL→AUTO: bumpless transfer
 * ========================================================================= */
void test_ron_tc_casc_008(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    /* Switch to manual */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE,
        ron_cascade_set_mode(&casc, RON_MODE_MANUAL, RON_FLOAT_C(4.0), RON_FLOAT_C(8.0)));

    /* Confirm manual output */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(5.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(4.0), u);

    /* Switch back to auto — bumpless */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE,
        ron_cascade_set_mode(&casc, RON_MODE_AUTOMATIC, RON_FLOAT_C(4.0), RON_FLOAT_C(8.0)));

    TEST_ASSERT_EQUAL_INT(RON_MODE_AUTOMATIC, (int) casc.outer.state.mode);
    TEST_ASSERT_EQUAL_INT(RON_MODE_AUTOMATIC, (int) casc.inner.state.mode);

    /*
     * First auto step at zero-error operating point (r_out=outer_manual, y_out=outer_manual,
     * y_in=inner_manual): P term is zero so output = integral = manual handoff value.
     * This verifies bumpless transfer pre-loaded the integrals correctly.
     */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_step(&casc, RON_FLOAT_C(8.0), RON_FLOAT_C(8.0),
                                             RON_FLOAT_C(8.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(4.0), u);
}

/* =========================================================================
 * RON-TC-CASC-009 | RON-FR-401, FR-405 — Step null/uninit/dt guards
 * ========================================================================= */
void test_ron_tc_casc_009(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_cascade_instance_t uninit;
    ron_float_t u               = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    /* Manually craft an uninitialised instance */
    uninit.outer.state.is_initialised = false;
    uninit.inner.state.is_initialised = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_cascade_step(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), NULL, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_step(&uninit, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), test_ron_casc_make_nan(), &u,
                                             &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(-0.001), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_step(&casc, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), test_ron_casc_make_inf(), &u,
                                             &status));

    /* Outer-only-initialized instance: covers left=F,right=T branch of || in step uninit check */
    {
        ron_cascade_instance_t partial;
        partial.outer.state.is_initialised = true;
        partial.inner.state.is_initialised = false;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                                ron_cascade_step(&partial, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                 RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    }

    /* set_mode null guard */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_cascade_set_mode(NULL, RON_MODE_MANUAL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));

    /* set_mode uninit guards: covers cascade_check_inst uninit path */
    {
        ron_cascade_instance_t sm_uninit = {0};

        /* Both uninit: outer=T in || → short-circuit (covers line 31 return) */
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_CONFIG_INVALID,
            ron_cascade_set_mode(&sm_uninit, RON_MODE_MANUAL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));

        /* Outer only init: outer=F, inner=T in || */
        sm_uninit.outer.state.is_initialised = true;
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_CONFIG_INVALID,
            ron_cascade_set_mode(&sm_uninit, RON_MODE_MANUAL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));
    }

    /* set_mode non-finite manual guards */
    /* NaN manual_inner: left=T in || → short-circuit */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_cascade_set_mode(&casc, RON_MODE_MANUAL, test_ron_casc_make_nan(), RON_FLOAT_C(0.0)));
    /* NaN manual_outer (valid inner): left=F, right=T in || */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_cascade_set_mode(&casc, RON_MODE_MANUAL, RON_FLOAT_C(0.0), test_ron_casc_make_nan()));
    /* -Inf manual_inner: covers value >= RON_FLOAT_MIN FALSE branch in cascade_isfinite */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_cascade_set_mode(&casc, RON_MODE_MANUAL,
                                                 test_ron_casc_make_neg_inf(), RON_FLOAT_C(0.0)));
}

/* =========================================================================
 * RON-TC-CASC-010 | RON-FR-406 — ron_cascade_get_state returns correct data
 * ========================================================================= */
void test_ron_tc_casc_010(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t u                    = RON_FLOAT_C(0.0);
    ron_cascade_status_t step_status = (ron_cascade_status_t) 0U;
    ron_cascade_status_t get_status  = (ron_cascade_status_t) 0U;
    ron_fault_t outer_fault          = RON_FAULT_NONE;
    ron_fault_t inner_fault          = RON_FAULT_NONE;
    ron_status_t outer_step;

    /* Tight inner limits so inner saturates */
    outer_cfg.Kp    = RON_FLOAT_C(1.0);
    outer_cfg.u_min = RON_FLOAT_C(-1000.0);
    outer_cfg.u_max = RON_FLOAT_C(1000.0);
    inner_cfg.Kp    = RON_FLOAT_C(50.0);
    inner_cfg.u_min = RON_FLOAT_C(-2.0);
    inner_cfg.u_max = RON_FLOAT_C(2.0);

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_step(&casc, RON_FLOAT_C(1.0),
                                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                             RON_FLOAT_C(0.01), &u, &step_status));

    /* get_state returns same status as step */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_cascade_get_state(&casc, &get_status, &outer_fault, &inner_fault));
    outer_step = RON_CASCADE_STATUS_OUTER(step_status);
    TEST_ASSERT_EQUAL_UINT16(outer_step, RON_CASCADE_STATUS_OUTER(get_status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, outer_fault);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, inner_fault);

    /* NULL output pointers silently skipped */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_get_state(&casc, NULL, NULL, NULL));

    /* NULL casc pointer */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_cascade_get_state(NULL, &get_status, NULL, NULL));

    /* Force a fault on the outer loop via NaN input */
    (void) ron_pid_step(&casc.outer, test_ron_casc_make_nan(), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01),
                        &u, &outer_step);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_get_state(&casc, NULL, &outer_fault, NULL));
    TEST_ASSERT_TRUE(outer_fault != RON_FAULT_NONE);
}

/* =========================================================================
 * RON-TC-CASC-011 | RON-FR-405 — ron_cascade_fault_clear clears both loops
 * ========================================================================= */
void test_ron_tc_casc_011(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_fault_t outer_fault = RON_FAULT_NONE;
    ron_fault_t inner_fault = RON_FAULT_NONE;
    ron_float_t dummy_u;
    ron_status_t dummy_s;

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    /* Force faults on both loops via NaN inputs */
    (void) ron_pid_step(&casc.outer, test_ron_casc_make_nan(), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01),
                        &dummy_u, &dummy_s);
    (void) ron_pid_step(&casc.inner, test_ron_casc_make_nan(), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01),
                        &dummy_u, &dummy_s);

    (void) ron_cascade_get_state(&casc, NULL, &outer_fault, &inner_fault);
    TEST_ASSERT_TRUE(outer_fault != RON_FAULT_NONE);
    TEST_ASSERT_TRUE(inner_fault != RON_FAULT_NONE);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_fault_clear(&casc));

    (void) ron_cascade_get_state(&casc, NULL, &outer_fault, &inner_fault);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, outer_fault);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, inner_fault);

    /* NULL pointer rejection */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_fault_clear(NULL));
}

/* =========================================================================
 * RON-TC-CASC-012 | RON-FR-405 — ron_cascade_reset zeros state, keeps config
 * ========================================================================= */
void test_ron_tc_casc_012(void)
{
    ron_pid_config_t outer_cfg = test_ron_casc_default_cfg();
    ron_pid_config_t inner_cfg = test_ron_casc_default_cfg();
    ron_cascade_instance_t casc;
    ron_float_t I_outer         = RON_FLOAT_C(0.0);
    ron_float_t I_inner         = RON_FLOAT_C(0.0);
    ron_float_t last_u          = RON_FLOAT_C(0.0);
    ron_float_t last_D          = RON_FLOAT_C(0.0);
    ron_float_t dummy_u         = RON_FLOAT_C(0.0);
    ron_cascade_status_t status = (ron_cascade_status_t) 0U;
    unsigned i;

    outer_cfg.Kp = RON_FLOAT_C(1.0);
    outer_cfg.Ki = RON_FLOAT_C(5.0);
    inner_cfg.Kp = RON_FLOAT_C(1.0);
    inner_cfg.Ki = RON_FLOAT_C(5.0);

    casc = test_ron_casc_init(&outer_cfg, &inner_cfg);

    /* Build up integral state in both loops */
    for (i = 0U; i < 10U; ++i) {
        (void) ron_cascade_step(&casc, RON_FLOAT_C(5.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                RON_FLOAT_C(0.01), &dummy_u, &status);
    }

    (void) ron_pid_get_state(&casc.outer, &I_outer, NULL, NULL, NULL, NULL);
    (void) ron_pid_get_state(&casc.inner, &I_inner, NULL, NULL, NULL, NULL);
    TEST_ASSERT_TRUE(test_ron_abs(I_outer) > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(test_ron_abs(I_inner) > RON_FLOAT_C(0.0));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_reset(&casc));

    (void) ron_pid_get_state(&casc.outer, &I_outer, &last_u, &last_D, NULL, NULL);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), I_outer);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), last_u);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), last_D);

    (void) ron_pid_get_state(&casc.inner, &I_inner, &last_u, &last_D, NULL, NULL);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), I_inner);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), last_u);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), last_D);

    /* Config preserved */
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(1.0), casc.outer.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(5.0), casc.outer.config.Ki);

    /* NULL pointer rejection */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_reset(NULL));
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_casc_001);
    RUN_TEST(test_ron_tc_casc_002);
    RUN_TEST(test_ron_tc_casc_003);
    RUN_TEST(test_ron_tc_casc_004);
    RUN_TEST(test_ron_tc_casc_005);
    RUN_TEST(test_ron_tc_casc_006);
    RUN_TEST(test_ron_tc_casc_007);
    RUN_TEST(test_ron_tc_casc_008);
    RUN_TEST(test_ron_tc_casc_009);
    RUN_TEST(test_ron_tc_casc_010);
    RUN_TEST(test_ron_tc_casc_011);
    RUN_TEST(test_ron_tc_casc_012);
    return UNITY_END();
}
