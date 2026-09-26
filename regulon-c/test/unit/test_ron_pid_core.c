/*
 * @file     test_ron_pid_core.c
 * @brief    PID core and normalisation unit tests.
 * @module   test_ron_pid_core
 * @doc      RON-TP-001
 * @req      RON-FR-001, RON-FR-002, RON-FR-004, RON-FR-005, RON-FR-006, RON-FR-007,
 *           RON-FR-010, RON-FR-011, RON-FR-012, RON-FR-013
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "unity.h"
#include "test_ron_pid_common.h"

#include <float.h>

void setUp(void)
{
}

void tearDown(void)
{
}

/* Satisfies: RON-FR-001 | Test: RON-TC-PID-001 */
static ron_pid_t test_ron_init_pid(const ron_pid_config_t *cfg)
{
    ron_pid_t inst;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, cfg));
    return inst;
}

/* RON-TC-PID-001 | RON-FR-001 */
void test_ron_tc_pid_001(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(2.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(2.0), u);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, status);
}

static ron_float_t test_ron_isa_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    return zero / zero;
}

static ron_float_t test_ron_isa_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;
    return big * RON_FLOAT_C(2.0);
}

/* Expects cfg_isa gains to be rejected and left at their sentinel values. */
static void test_ron_isa_rejected(ron_float_t Kp, ron_float_t Ti, ron_float_t Td)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();

    cfg.Kp = RON_FLOAT_C(7.0);
    cfg.Ki = RON_FLOAT_C(8.0);
    cfg.Kd = RON_FLOAT_C(9.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_pid_config_from_isa(&cfg, Kp, Ti, Td));
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(7.0), cfg.Kp);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(8.0), cfg.Ki);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(9.0), cfg.Kd);
}

/* RON-TC-PID-002 | RON-FR-002 */
void test_ron_tc_pid_002(void)
{
    ron_pid_config_t   cfg_parallel = test_ron_make_pid_cfg();
    ron_pid_config_t   cfg_isa = test_ron_make_pid_cfg();
    ron_pid_t pid_parallel;
    ron_pid_t pid_isa;
    ron_float_t        u_parallel = RON_FLOAT_C(0.0);
    ron_float_t        u_isa = RON_FLOAT_C(0.0);
    ron_status_t       status_parallel = RON_STATUS_OK;
    ron_status_t       status_isa = RON_STATUS_OK;
    uint8_t            index;

    cfg_parallel.Kp = RON_FLOAT_C(2.0);
    cfg_parallel.Ki = RON_FLOAT_C(0.4);
    cfg_parallel.Kd = RON_FLOAT_C(0.2);
    cfg_parallel.deriv_mode = RON_DERIV_ON_ERROR;

    /* (a) conversion writes only the gains */
    cfg_isa.deriv_mode = RON_DERIV_ON_ERROR;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_config_from_isa(&cfg_isa, RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), RON_FLOAT_C(0.1)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(2.0), cfg_isa.Kp);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(0.4), cfg_isa.Ki);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(0.2), cfg_isa.Kd);
    TEST_ASSERT_EQUAL_INT((int) RON_DERIV_ON_ERROR, (int) cfg_isa.deriv_mode);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, cfg_parallel.u_max, cfg_isa.u_max);

    pid_parallel = test_ron_init_pid(&cfg_parallel);
    pid_isa = test_ron_init_pid(&cfg_isa);

    for (index = 0U; index < 50U; ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&pid_parallel, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01),
                         &u_parallel, &status_parallel));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&pid_isa, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01),
                         &u_isa, &status_isa));
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, u_parallel, u_isa);
    }

    /* (b) Ti = +Inf: no integral action */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_config_from_isa(&cfg_isa, RON_FLOAT_C(2.0), test_ron_isa_inf(), RON_FLOAT_C(0.0)));
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), cfg_isa.Ki);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.0), cfg_isa.Kd);

    /* (c) rejected inputs leave the record unchanged */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_config_from_isa(NULL, RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), RON_FLOAT_C(0.1)));
    test_ron_isa_rejected(RON_FLOAT_C(-1.0), RON_FLOAT_C(5.0), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(test_ron_isa_nan(), RON_FLOAT_C(5.0), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), test_ron_isa_nan(), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), -test_ron_isa_inf(), RON_FLOAT_C(0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), RON_FLOAT_C(-0.1));
    test_ron_isa_rejected(RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), test_ron_isa_nan());
    test_ron_isa_rejected(RON_FLOAT_C(2.0), RON_FLOAT_C(5.0), test_ron_isa_inf());
    /* Kp / Ti overflows */
    test_ron_isa_rejected(RON_FLOAT_MAX, RON_FLOAT_C(0.5), RON_FLOAT_C(0.0));
    /* Kp * Td overflows while Ki stays finite */
    test_ron_isa_rejected(RON_FLOAT_MAX, test_ron_isa_inf(), RON_FLOAT_C(2.0));
}

/* RON-TC-PID-003 | RON-FR-003 */
void test_ron_tc_pid_003(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(3.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(0.4), RON_FLOAT_C(0.2), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.6), u);
}

/* RON-TC-PID-004 | RON-FR-004 */
void test_ron_tc_pid_004(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.integ_method = RON_INTEG_EULER;
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.1), u);
}

/* RON-TC-PID-005 | RON-FR-004 */
void test_ron_tc_pid_005(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.integ_method = RON_INTEG_TRAPEZOIDAL;
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.05), u);
}

/* RON-TC-PID-006 | RON-FR-005 */
void test_ron_tc_pid_006(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kd = RON_FLOAT_C(2.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(0.5), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE(u > RON_FLOAT_C(50.0));
}

/* RON-TC-PID-007 | RON-FR-005 */
void test_ron_tc_pid_007(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(1.0);
    cfg.Kd = RON_FLOAT_C(2.0);
    cfg.deriv_mode = RON_DERIV_ON_MEASUREMENT;
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(0.5), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.5), u);
}

/* RON-TC-PID-008 | RON-FR-006 */
void test_ron_tc_pid_008(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kd = RON_FLOAT_C(2.0);
    cfg.N = RON_FLOAT_C(1.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    pid = test_ron_init_pid(&cfg);

    (void)ron_pid_step(&pid, RON_FLOAT_C(0.5), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-3), RON_FLOAT_C(1.490099), u);
}

/* RON-TC-PID-009 | RON-FR-006 */
void test_ron_tc_pid_009(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kd = RON_FLOAT_C(2.0);
    cfg.N = RON_FLOAT_C(0.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    pid = test_ron_init_pid(&cfg);

    (void)ron_pid_step(&pid, RON_FLOAT_C(0.5), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE(u > RON_FLOAT_C(99.0));
}

/* RON-TC-PID-010 | RON-FR-007 */
void test_ron_tc_pid_010(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.b = RON_FLOAT_C(0.5);
    cfg.c = RON_FLOAT_C(0.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
}

/* RON-TC-PID-011 | RON-FR-010 */
void test_ron_tc_pid_011(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.normalise = true;
    cfg.in_min = RON_FLOAT_C(0.0);
    cfg.in_max = RON_FLOAT_C(10.0);
    cfg.out_min = RON_FLOAT_C(0.0);
    cfg.out_max = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_NORMALISED) != 0U);
}

/* RON-TC-PID-012 | RON-FR-011 */
void test_ron_tc_pid_012(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.normalise = true;
    cfg.in_min = RON_FLOAT_C(0.0);
    cfg.in_max = RON_FLOAT_C(10.0);
    cfg.out_min = RON_FLOAT_C(0.0);
    cfg.out_max = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(7.5), RON_FLOAT_C(2.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.5), u);
}

/* RON-TC-PID-013 | RON-FR-012 */
void test_ron_tc_pid_013(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.normalise = true;
    cfg.in_min = RON_FLOAT_C(0.0);
    cfg.in_max = RON_FLOAT_C(10.0);
    cfg.out_min = RON_FLOAT_C(-20.0);
    cfg.out_max = RON_FLOAT_C(20.0);
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(5.0), RON_FLOAT_C(2.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(-10.0), u);
}

/* RON-TC-PID-014 | RON-FR-013 */
void test_ron_tc_pid_014(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(1.0);
    cfg.normalise = false;
    pid = test_ron_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(7.5), RON_FLOAT_C(2.5), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(5.0), u);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, status);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_pid_001);
    RUN_TEST(test_ron_tc_pid_002);
    RUN_TEST(test_ron_tc_pid_003);
    RUN_TEST(test_ron_tc_pid_004);
    RUN_TEST(test_ron_tc_pid_005);
    RUN_TEST(test_ron_tc_pid_006);
    RUN_TEST(test_ron_tc_pid_007);
    RUN_TEST(test_ron_tc_pid_008);
    RUN_TEST(test_ron_tc_pid_009);
    RUN_TEST(test_ron_tc_pid_010);
    RUN_TEST(test_ron_tc_pid_011);
    RUN_TEST(test_ron_tc_pid_012);
    RUN_TEST(test_ron_tc_pid_013);
    RUN_TEST(test_ron_tc_pid_014);
    return UNITY_END();
}
