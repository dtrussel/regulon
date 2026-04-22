/*
 * @file     test_ron_pid_api.c
 * @brief    PID API, safety, and diagnostics unit tests.
 * @module   test_ron_pid_api
 * @doc      RON-TP-001
 * @req      RON-FR-020, RON-FR-021, RON-FR-022, RON-FR-025, RON-FR-026,
 *           RON-FR-027, RON-FR-030, RON-FR-031, RON-FR-032, RON-FR-033,
 *           RON-FR-034, RON-FR-035, RON-FR-040, RON-FR-041, RON-FR-042,
 *           RON-FR-050, RON-FR-051, RON-FR-052, RON-FR-053, RON-FR-054,
 *           RON-FR-060, RON-FR-061, RON-FR-062, RON-FR-070, RON-FR-071,
 *           RON-SR-001, RON-SR-010, RON-SR-011, RON-SR-012, RON-SR-020
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "unity.h"
#include "test_ron_pid_common.h"

static unsigned test_ron_fault_cb_count = 0U;
static ron_fault_t test_ron_last_fault = RON_FAULT_NONE;

void setUp(void)
{
    test_ron_fault_cb_count = 0U;
    test_ron_last_fault = RON_FAULT_NONE;
}

void tearDown(void)
{
}

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
static void test_ron_fault_cb(ron_fault_t fault)
{
    ++test_ron_fault_cb_count;
    test_ron_last_fault = fault;
}

/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
static ron_pid_instance_t test_ron_init_pid_api(const ron_pid_config_t *cfg)
{
    ron_pid_instance_t inst;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, cfg));
    return inst;
}

/* Satisfies: RON-FR-031 | Test: RON-TC-PID-022 */
static void test_ron_drive_positive_saturation(ron_pid_instance_t *pid,
                                               unsigned            steps)
{
    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    unsigned     step;
    const ron_float_t dt = RON_FLOAT_C(0.01);

    for (step = 0U; step < steps; ++step)
    {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(pid, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), dt, &u, &status));
    }
}

/* Satisfies: RON-QR-031 | Test: RON-TC-QUAL-017 */
static uint32_t test_ron_hash_bytes(uint32_t             hash,
                                    const unsigned char *bytes,
                                    unsigned             length)
{
    unsigned index;

    for (index = 0U; index < length; ++index)
    {
        hash ^= (uint32_t)bytes[index];
        hash *= 16777619UL;
    }

    return hash;
}

/* Satisfies: RON-QR-031 | Test: RON-TC-QUAL-017 */
static uint32_t test_ron_hash_float(uint32_t    hash,
                                    ron_float_t value)
{
    union
    {
        ron_float_t    value;
        unsigned char  bytes[sizeof(ron_float_t)];
    } raw;

    raw.value = value;
    return test_ron_hash_bytes(hash, raw.bytes, (unsigned)sizeof(raw.bytes));
}

/* Satisfies: RON-QR-031 | Test: RON-TC-QUAL-017 */
static uint32_t test_ron_hash_status(uint32_t     hash,
                                     ron_status_t status)
{
    union
    {
        ron_status_t   value;
        unsigned char  bytes[sizeof(ron_status_t)];
    } raw;

    raw.value = status;
    return test_ron_hash_bytes(hash, raw.bytes, (unsigned)sizeof(raw.bytes));
}

/* Satisfies: RON-QR-031 | Test: RON-TC-QUAL-017 */
static uint32_t test_ron_hash_fault(uint32_t    hash,
                                    ron_fault_t fault)
{
    union
    {
        ron_fault_t    value;
        unsigned char  bytes[sizeof(ron_fault_t)];
    } raw;

    raw.value = fault;
    return test_ron_hash_bytes(hash, raw.bytes, (unsigned)sizeof(raw.bytes));
}

/* Satisfies: RON-QR-031 | Test: RON-TC-QUAL-017 */
static uint32_t test_ron_hash_trace(const ron_pid_config_t *cfg)
{
    static const ron_float_t setpoints[8] = {
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(0.5),
        RON_FLOAT_C(-0.5), RON_FLOAT_C(-1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.25)
    };
    static const ron_float_t measurements[8] = {
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), RON_FLOAT_C(0.4), RON_FLOAT_C(0.6),
        RON_FLOAT_C(0.2), RON_FLOAT_C(-0.2), RON_FLOAT_C(-0.1), RON_FLOAT_C(0.05)
    };
    static const ron_float_t sample_periods[8] = {
        RON_FLOAT_C(0.01), RON_FLOAT_C(0.01), RON_FLOAT_C(0.02), RON_FLOAT_C(0.01),
        RON_FLOAT_C(0.01), RON_FLOAT_C(0.02), RON_FLOAT_C(0.01), RON_FLOAT_C(0.01)
    };
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    uint32_t           hash = 2166136261UL;
    unsigned           index;

    pid = test_ron_init_pid_api(cfg);
    for (index = 0U; index < 8U; ++index)
    {
        ron_fault_t fault;

        fault = ron_pid_step(&pid,
                             setpoints[index],
                             measurements[index],
                             sample_periods[index],
                             &u,
                             &status);
        hash = test_ron_hash_float(hash, u);
        hash = test_ron_hash_status(hash, status);
        hash = test_ron_hash_fault(hash, fault);
    }

    return hash;
}

/* RON-TC-PID-015 | RON-FR-020 */
void test_ron_tc_pid_015(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg.Kp = RON_FLOAT_C(1000.0);
    cfg.u_min = RON_FLOAT_C(-10.0);
    cfg.u_max = RON_FLOAT_C(10.0);
    pid = test_ron_init_pid_api(&cfg);

    for (index = 0U; index < 20U; ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&pid, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(10.0), u);
        TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
    }
}

/* RON-TC-PID-016 | RON-FR-021 */
void test_ron_tc_pid_016(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(10.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_limits(&pid, RON_FLOAT_C(-3.0), RON_FLOAT_C(3.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(3.0), u);
}

/* RON-TC-PID-017 | RON-FR-022 */
void test_ron_tc_pid_017(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(100.0);
    cfg.u_min = RON_FLOAT_C(-2.0);
    cfg.u_max = RON_FLOAT_C(2.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
}

/* RON-TC-PID-018 | RON-FR-025 */
void test_ron_tc_pid_018(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(100.0);
    cfg.du_max = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.1), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
}

/* RON-TC-PID-019 | RON-FR-026 */
void test_ron_tc_pid_019(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(100.0);
    cfg.du_max = RON_FLOAT_C(0.0);
    cfg.u_max = RON_FLOAT_C(10.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(10.0), u);
    TEST_ASSERT_FALSE((status & RON_STATUS_RATE_LIMITED) != 0U);
}

/* RON-TC-PID-020 | RON-FR-027 */
void test_ron_tc_pid_020(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(1000.0);
    cfg.u_max = RON_FLOAT_C(5.0);
    cfg.du_max = RON_FLOAT_C(10.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
}

/* RON-TC-PID-021 | RON-FR-030 */
void test_ron_tc_pid_021(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_float_t        integral = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg.Kp = RON_FLOAT_C(1.0);
    cfg.Ki = RON_FLOAT_C(10.0);
    cfg.u_min = RON_FLOAT_C(-5.0);
    cfg.u_max = RON_FLOAT_C(5.0);
    cfg.aw_mode = RON_AW_BACK_CALC;
    cfg.T_aw = RON_FLOAT_C(0.1);
    pid = test_ron_init_pid_api(&cfg);

    for (index = 0U; index < 50U; ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&pid, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_TRUE(test_ron_abs(integral) < RON_FLOAT_C(100.0));
}

/* RON-TC-PID-022 | RON-FR-031 */
void test_ron_tc_pid_022(void)
{
    ron_pid_config_t   cfg_aw = test_ron_make_pid_cfg();
    ron_pid_config_t   cfg_none = test_ron_make_pid_cfg();
    ron_pid_instance_t pid_aw;
    ron_pid_instance_t pid_none;
    ron_float_t        u_aw = RON_FLOAT_C(0.0);
    ron_float_t        u_none = RON_FLOAT_C(0.0);
    ron_float_t        i_aw = RON_FLOAT_C(0.0);
    ron_float_t        i_none = RON_FLOAT_C(0.0);
    ron_status_t       status_aw = RON_STATUS_OK;
    ron_status_t       status_none = RON_STATUS_OK;

    cfg_aw.Kp = RON_FLOAT_C(1.0);
    cfg_aw.Ki = RON_FLOAT_C(10.0);
    cfg_aw.u_min = RON_FLOAT_C(-5.0);
    cfg_aw.u_max = RON_FLOAT_C(5.0);
    cfg_aw.aw_mode = RON_AW_BACK_CALC;
    cfg_aw.T_aw = RON_FLOAT_C(0.1);

    cfg_none = cfg_aw;
    cfg_none.aw_mode = RON_AW_NONE;

    pid_aw = test_ron_init_pid_api(&cfg_aw);
    pid_none = test_ron_init_pid_api(&cfg_none);

    test_ron_drive_positive_saturation(&pid_aw, 100U);
    test_ron_drive_positive_saturation(&pid_none, 100U);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid_aw, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u_aw, &status_aw));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid_none, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u_none, &status_none));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid_aw, &i_aw, NULL, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid_none, &i_none, NULL, NULL, NULL, NULL));

    TEST_ASSERT_TRUE(u_aw < RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(u_none > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(test_ron_abs(i_aw) < test_ron_abs(i_none));
}

/* RON-TC-PID-023 | RON-FR-032 */
void test_ron_tc_pid_023(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        before = RON_FLOAT_C(0.0);
    ron_float_t        after = RON_FLOAT_C(0.0);
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(50.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.u_max = RON_FLOAT_C(1.0);
    cfg.aw_mode = RON_AW_CLAMPING;
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &before, NULL, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &after, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), before, after);
}

/* RON-TC-PID-024 | RON-FR-033 */
void test_ron_tc_pid_024(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(50.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.u_max = RON_FLOAT_C(1.0);
    cfg.aw_mode = RON_AW_NONE;
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_antiwindup(&pid, RON_AW_BACK_CALC, RON_FLOAT_C(0.1)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_TRUE((status & RON_STATUS_AW_ACTIVE) != 0U);
}

/* RON-TC-PID-025 | RON-FR-034 */
void test_ron_tc_pid_025(void)
{
    ron_pid_config_t   cfg_none = test_ron_make_pid_cfg();
    ron_pid_config_t   cfg_aw = test_ron_make_pid_cfg();
    ron_pid_instance_t pid_none;
    ron_pid_instance_t pid_aw;
    ron_float_t        i_none = RON_FLOAT_C(0.0);
    ron_float_t        i_aw = RON_FLOAT_C(0.0);
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg_none.Kp = RON_FLOAT_C(1.0);
    cfg_none.Ki = RON_FLOAT_C(10.0);
    cfg_none.u_max = RON_FLOAT_C(5.0);
    cfg_none.aw_mode = RON_AW_NONE;

    cfg_aw = cfg_none;
    cfg_aw.aw_mode = RON_AW_BACK_CALC;
    cfg_aw.T_aw = RON_FLOAT_C(0.1);

    pid_none = test_ron_init_pid_api(&cfg_none);
    pid_aw = test_ron_init_pid_api(&cfg_aw);

    for (index = 0U; index < 20U; ++index)
    {
        (void)ron_pid_step(&pid_none, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status);
        (void)ron_pid_step(&pid_aw, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status);
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_state(&pid_none, &i_none, NULL, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_state(&pid_aw, &i_aw, NULL, NULL, NULL, NULL));
    TEST_ASSERT_TRUE(i_none > i_aw);
}

/* RON-TC-PID-026 | RON-FR-035 */
void test_ron_tc_pid_026(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        integral = RON_FLOAT_C(0.0);
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(10.0);
    cfg.I_min = RON_FLOAT_C(-0.5);
    cfg.I_max = RON_FLOAT_C(0.5);
    pid = test_ron_init_pid_api(&cfg);

    for (index = 0U; index < 10U; ++index)
    {
        (void)ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status);
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_state(&pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.5), integral);
}

/* RON-TC-PID-027 | RON-FR-040 */
void test_ron_tc_pid_027(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_init_pid_api(&cfg);
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_mode(&pid, RON_MODE_MANUAL, RON_FLOAT_C(3.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(10.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(3.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_MANUAL_MODE) != 0U);
}

/* RON-TC-PID-028 | RON-FR-041 */
void test_ron_tc_pid_028(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_mode(&pid, RON_MODE_MANUAL, RON_FLOAT_C(3.5)));
    for (index = 0U; index < 10U; ++index)
    {
        (void)ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status);
    }
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_mode(&pid, RON_MODE_AUTOMATIC, RON_FLOAT_C(3.5)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE((test_ron_abs(u - RON_FLOAT_C(3.5)) / RON_FLOAT_C(3.5)) < RON_FLOAT_C(0.05));
}

/* RON-TC-PID-029 | RON-FR-042 */
void test_ron_tc_pid_029(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.u_max = RON_FLOAT_C(5.0);
    pid = test_ron_init_pid_api(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_mode(&pid, RON_MODE_MANUAL, RON_FLOAT_C(100.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(5.0), u);
}

/* RON-TC-PID-030 | RON-FR-050 */
void test_ron_tc_pid_030(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_init_pid_api(&cfg);
    ron_float_t        integral = RON_FLOAT_C(1.0);
    ron_float_t        last_u = RON_FLOAT_C(1.0);
    ron_float_t        last_d = RON_FLOAT_C(1.0);
    ron_status_t       status = RON_STATUS_FAULT;
    ron_fault_t        fault = RON_FAULT_INPUT_NAN;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &integral, &last_u, &last_d, &status, &fault));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), integral);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), last_u);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), last_d);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, fault);
}

/* RON-TC-PID-031 | RON-FR-051 */
void test_ron_tc_pid_031(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        integral = RON_FLOAT_C(0.0);
    ron_float_t        last_u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Ki = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid_api(&cfg);
    (void)ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1),
                       &last_u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_reset(&pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &integral, &last_u, NULL, &status, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), integral);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), last_u);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, status);
}

/* RON-TC-PID-032 | RON-FR-052 */
void test_ron_tc_pid_032(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_init_pid_api(&cfg);
    ron_float_t        integral = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_integral(&pid, RON_FLOAT_C(0.75)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &integral, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.75), integral);
}

/* RON-TC-PID-033 | RON-FR-053 */
void test_ron_tc_pid_033(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_set_gains(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(-1.0), RON_FLOAT_C(3.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_set_filter(&pid, RON_FLOAT_C(-1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_set_limits(&pid, RON_FLOAT_C(5.0), RON_FLOAT_C(5.0)));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
}

/* RON-TC-PID-034 | RON-FR-054 */
void test_ron_tc_pid_034(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.sp_reset_threshold = RON_FLOAT_C(0.2);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(2.0), u);
}

/* RON-TC-PID-035 | RON-FR-060 */
void test_ron_tc_pid_035(void)
{
    static const ron_float_t setpoints[10] = {
        RON_FLOAT_C(0.1), RON_FLOAT_C(0.5), RON_FLOAT_C(-0.2), RON_FLOAT_C(1.0), RON_FLOAT_C(0.3),
        RON_FLOAT_C(-0.1), RON_FLOAT_C(0.9), RON_FLOAT_C(0.0), RON_FLOAT_C(0.2), RON_FLOAT_C(-0.4)
    };
    static const ron_float_t measurements[10] = {
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.2), RON_FLOAT_C(-0.1), RON_FLOAT_C(0.4), RON_FLOAT_C(0.1),
        RON_FLOAT_C(-0.2), RON_FLOAT_C(0.6), RON_FLOAT_C(-0.1), RON_FLOAT_C(0.0), RON_FLOAT_C(-0.3)
    };
    ron_pid_config_t   cfg_a = test_ron_make_pid_cfg();
    ron_pid_config_t   cfg_b = test_ron_make_pid_cfg();
    ron_pid_instance_t a_interleaved;
    ron_pid_instance_t b_interleaved;
    ron_pid_instance_t a_alone;
    ron_pid_instance_t b_alone;
    ron_float_t        u_a_i = RON_FLOAT_C(0.0);
    ron_float_t        u_b_i = RON_FLOAT_C(0.0);
    ron_float_t        u_a_a = RON_FLOAT_C(0.0);
    ron_float_t        u_b_a = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    cfg_a.Kp = RON_FLOAT_C(1.0);
    cfg_a.Ki = RON_FLOAT_C(0.1);
    cfg_b.Kp = RON_FLOAT_C(3.0);
    cfg_b.Ki = RON_FLOAT_C(0.5);
    cfg_b.Kd = RON_FLOAT_C(0.2);
    cfg_b.deriv_mode = RON_DERIV_ON_ERROR;

    a_interleaved = test_ron_init_pid_api(&cfg_a);
    b_interleaved = test_ron_init_pid_api(&cfg_b);
    a_alone = test_ron_init_pid_api(&cfg_a);
    b_alone = test_ron_init_pid_api(&cfg_b);

    for (index = 0U; index < 10U; ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&a_interleaved, setpoints[index], measurements[index], RON_FLOAT_C(0.01),
                         &u_a_i, &status));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&b_interleaved, measurements[index], setpoints[index], RON_FLOAT_C(0.01),
                         &u_b_i, &status));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&a_alone, setpoints[index], measurements[index], RON_FLOAT_C(0.01),
                         &u_a_a, &status));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
            ron_pid_step(&b_alone, measurements[index], setpoints[index], RON_FLOAT_C(0.01),
                         &u_b_a, &status));
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), u_a_a, u_a_i);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), u_b_a, u_b_i);
    }
}

/* RON-TC-PID-036 | RON-FR-061 */
void test_ron_tc_pid_036(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t used;
    ron_pid_instance_t fresh;
    ron_float_t        u_used = RON_FLOAT_C(0.0);
    ron_float_t        u_fresh = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Ki = RON_FLOAT_C(1.0);
    used = test_ron_init_pid_api(&cfg);
    fresh = test_ron_init_pid_api(&cfg);

    (void)ron_pid_step(&used, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u_used, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_init(&fresh, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&fresh, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u_fresh, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.1), u_used);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.1), u_fresh);
}

/* RON-TC-PID-037 | RON-FR-062 */
void test_ron_tc_pid_037(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t a;
    ron_pid_instance_t b;
    ron_float_t        u_a = RON_FLOAT_C(0.0);
    ron_float_t        u_b = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(10.0);
    a = test_ron_init_pid_api(&cfg);
    b = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_set_limits(&a, RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&a, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u_a, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&b, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u_b, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u_a);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(100.0), u_b);
}

/* RON-TC-PID-038 | RON-FR-070 */
void test_ron_tc_pid_038(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    cfg.Kp = RON_FLOAT_C(10.0);
    cfg.normalise = true;
    cfg.in_min = RON_FLOAT_C(0.0);
    cfg.in_max = RON_FLOAT_C(1.0);
    cfg.out_min = RON_FLOAT_C(0.0);
    cfg.out_max = RON_FLOAT_C(1.0);
    cfg.tau_sp = RON_FLOAT_C(0.1);
    cfg.u_max = RON_FLOAT_C(0.4);
    cfg.du_max = RON_FLOAT_C(1.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_TRUE((status & RON_STATUS_NORMALISED) != 0U);
    TEST_ASSERT_TRUE((status & RON_STATUS_SP_FILTER_ACTIVE) != 0U);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
    TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
}

/* RON-TC-PID-039 | RON-FR-071 */
void test_ron_tc_pid_039(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_float_t        integral = RON_FLOAT_C(0.0);
    ron_float_t        last_u = RON_FLOAT_C(0.0);
    ron_float_t        last_d = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    ron_fault_t        fault = RON_FAULT_NONE;

    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.Kd = RON_FLOAT_C(1.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    pid = test_ron_init_pid_api(&cfg);

    (void)ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, &integral, &last_u, &last_d, &status, &fault));
    TEST_ASSERT_TRUE(integral > RON_FLOAT_C(0.0));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), u, last_u);
    TEST_ASSERT_TRUE(last_d > RON_FLOAT_C(0.0));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, fault);
}

/* RON-TC-SAFE-001 | RON-SR-001 */
void test_ron_tc_safe_001(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_init_pid_api(&cfg);
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &u, &status));
    cfg.Kp = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_config_validate(&cfg));
}

/* RON-TC-SAFE-007 | RON-SR-010 */
void test_ron_tc_safe_007(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t        nan_value = zero / zero;

    cfg.safe_policy = RON_SAFE_ZERO;
    cfg.fault_cb = test_ron_fault_cb;
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE((status & RON_STATUS_FAULT) != 0U);
    TEST_ASSERT_EQUAL_UINT(1U, test_ron_fault_cb_count);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, test_ron_last_fault);
}

/* RON-TC-SAFE-008 | RON-SR-011 */
void test_ron_tc_safe_008(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t        nan_value = zero / zero;

    cfg.safe_policy = RON_SAFE_CONSTANT;
    cfg.safe_value = RON_FLOAT_C(20.0);
    cfg.u_max = RON_FLOAT_C(5.0);
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(5.0), u);
}

/* RON-TC-SAFE-009 | RON-SR-012 */
void test_ron_tc_safe_009(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    ron_float_t        nan_value = zero / zero;

    cfg.safe_policy = RON_SAFE_ZERO;
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_fault_clear(&pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
}

/* RON-TC-SAFE-010 | RON-SR-013 */
void test_ron_tc_safe_010(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    ron_fault_t        fault = RON_FAULT_NONE;

    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(100.0);
    cfg.I_overflow_thresh = RON_FLOAT_C(1.0);
    cfg.safe_policy = RON_SAFE_ZERO;
    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INTEGRAL_OVERFLOW,
        ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
        ron_pid_get_state(&pid, NULL, NULL, NULL, &status, &fault));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INTEGRAL_OVERFLOW, fault);
    TEST_ASSERT_TRUE((status & RON_STATUS_FAULT) != 0U);
}

/* RON-TC-SAFE-011 | RON-SR-020 */
void test_ron_tc_safe_011(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    volatile ron_float_t big = RON_FLOAT_MAX;
    ron_float_t        nan_value = zero / zero;
    ron_float_t        inf_value = big * big;

    pid = test_ron_init_pid_api(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_fault_clear(&pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
        ron_pid_step(&pid, RON_FLOAT_C(0.0), inf_value, RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_fault_clear(&pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
        ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), inf_value, &u, &status));
}

/* RON-TC-SAFE-006 | RON-SR-001, RON-SR-006 */
void test_ron_tc_safe_006(void)
{
    ron_pid_config_t   cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_init_pid_api(&cfg);
    ron_float_t        u = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_gains(NULL, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_limits(NULL, RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_filter(NULL, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_antiwindup(NULL, RON_AW_BACK_CALC, RON_FLOAT_C(0.1)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_mode(NULL, RON_MODE_MANUAL, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
        ron_pid_set_integral(NULL, RON_FLOAT_C(0.0)));
}

/* RON-TC-QUAL-017 | RON-QR-031 */
void test_ron_tc_qual_017(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    uint32_t         hash_first;
    uint32_t         hash_second;

    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.Ki = RON_FLOAT_C(0.5);
    cfg.Kd = RON_FLOAT_C(0.2);
    cfg.N = RON_FLOAT_C(5.0);
    cfg.du_max = RON_FLOAT_C(20.0);
    cfg.u_min = RON_FLOAT_C(-10.0);
    cfg.u_max = RON_FLOAT_C(10.0);
    cfg.aw_mode = RON_AW_BACK_CALC;
    cfg.T_aw = RON_FLOAT_C(0.2);

    hash_first = test_ron_hash_trace(&cfg);
    hash_second = test_ron_hash_trace(&cfg);
    TEST_ASSERT_TRUE(hash_first == hash_second);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_pid_015);
    RUN_TEST(test_ron_tc_pid_016);
    RUN_TEST(test_ron_tc_pid_017);
    RUN_TEST(test_ron_tc_pid_018);
    RUN_TEST(test_ron_tc_pid_019);
    RUN_TEST(test_ron_tc_pid_020);
    RUN_TEST(test_ron_tc_pid_021);
    RUN_TEST(test_ron_tc_pid_022);
    RUN_TEST(test_ron_tc_pid_023);
    RUN_TEST(test_ron_tc_pid_024);
    RUN_TEST(test_ron_tc_pid_025);
    RUN_TEST(test_ron_tc_pid_026);
    RUN_TEST(test_ron_tc_pid_027);
    RUN_TEST(test_ron_tc_pid_028);
    RUN_TEST(test_ron_tc_pid_029);
    RUN_TEST(test_ron_tc_pid_030);
    RUN_TEST(test_ron_tc_pid_031);
    RUN_TEST(test_ron_tc_pid_032);
    RUN_TEST(test_ron_tc_pid_033);
    RUN_TEST(test_ron_tc_pid_034);
    RUN_TEST(test_ron_tc_pid_035);
    RUN_TEST(test_ron_tc_pid_036);
    RUN_TEST(test_ron_tc_pid_037);
    RUN_TEST(test_ron_tc_pid_038);
    RUN_TEST(test_ron_tc_pid_039);
    RUN_TEST(test_ron_tc_safe_001);
    RUN_TEST(test_ron_tc_safe_006);
    RUN_TEST(test_ron_tc_safe_007);
    RUN_TEST(test_ron_tc_safe_008);
    RUN_TEST(test_ron_tc_safe_009);
    RUN_TEST(test_ron_tc_safe_010);
    RUN_TEST(test_ron_tc_safe_011);
    RUN_TEST(test_ron_tc_qual_017);
    return UNITY_END();
}
