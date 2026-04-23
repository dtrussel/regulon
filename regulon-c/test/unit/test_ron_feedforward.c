/*
 * @file     test_ron_feedforward.c
 * @brief    Feed-forward PID extension unit tests.
 * @module   test_ron_feedforward
 * @doc      RON-TP-001
 * @req      RON-FR-200, RON-FR-201, RON-FR-202, RON-FR-203,
 *           RON-FR-204, RON-FR-205
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <float.h>

#include "ron/ron_feedforward.h"

#include "test_ron_pid_common.h"
#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_ff_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_ff_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_ron_ff_make_neg_inf(void)
{
    return -test_ron_ff_make_inf();
}

/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
static ron_pid_instance_t test_ron_ff_init_pid(const ron_pid_config_t *cfg)
{
    ron_pid_instance_t inst;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, cfg));
    return inst;
}

/* Satisfies: RON-FR-201 | Test: RON-TC-FF-002 */
static ron_feedforward_config_t test_ron_ff_cfg(ron_feedforward_mode_t mode, ron_float_t gain)
{
    ron_feedforward_config_t cfg;

    cfg.mode = mode;
    cfg.gain = gain;
    cfg.N_ff = RON_FLOAT_C(0.0);
    return cfg;
}

/* RON-TC-FF-001 | RON-FR-200 */
void test_ron_tc_ff_001(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_float_t u_ff    = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(2.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(0.5));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(1.0),
                                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(3.0), u);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_feedforward(&pid, &u_ff));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(1.0), u_ff);
}

/* RON-TC-FF-002 | RON-FR-201 */
void test_ron_tc_ff_002(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(1.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(0.5));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(3.0), u);
}

/* RON-TC-FF-003 | RON-FR-201 */
void test_ron_tc_ff_003(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(0.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_VELOCITY, RON_FLOAT_C(0.25));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(1.2), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.5), u);
}

/* RON-TC-FF-004 | RON-FR-201 */
void test_ron_tc_ff_004(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(0.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_ACCELERATION, RON_FLOAT_C(0.1));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(0.1), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(0.3), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
}

/* RON-TC-FF-005 | RON-FR-201 */
void test_ron_tc_ff_005(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(1.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_EXTERNAL, RON_FLOAT_C(0.0));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(1.0), RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step_feedforward(
                                                &pid, RON_FLOAT_C(2.0), RON_FLOAT_C(1.0),
                                                RON_FLOAT_C(0.01), RON_FLOAT_C(0.75), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.75), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_FF_ACTIVE) != 0U);
}

/* RON-TC-FF-006 | RON-FR-202 */
void test_ron_tc_ff_006(void)
{
    ron_pid_config_t cfg_raw      = test_ron_make_pid_cfg();
    ron_pid_config_t cfg_filtered = test_ron_make_pid_cfg();
    ron_pid_instance_t raw;
    ron_pid_instance_t filtered;
    ron_float_t u_raw      = RON_FLOAT_C(0.0);
    ron_float_t u_filtered = RON_FLOAT_C(0.0);
    ron_float_t d_raw      = RON_FLOAT_C(1.0);
    ron_float_t d_filtered = RON_FLOAT_C(1.0);
    ron_status_t status    = RON_STATUS_OK;

    cfg_raw.Kp                    = RON_FLOAT_C(0.0);
    cfg_raw.Kd                    = RON_FLOAT_C(0.0);
    cfg_raw.feedforward           = test_ron_ff_cfg(RON_FF_VELOCITY, RON_FLOAT_C(1.0));
    cfg_filtered                  = cfg_raw;
    cfg_filtered.feedforward.N_ff = RON_FLOAT_C(1.0);
    raw                           = test_ron_ff_init_pid(&cfg_raw);
    filtered                      = test_ron_ff_init_pid(&cfg_filtered);

    (void) ron_pid_step(&raw, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1), &u_raw,
                        &status);
    (void) ron_pid_step(&filtered, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.1),
                        &u_filtered, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&raw, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u_raw, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_step(&filtered, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                         RON_FLOAT_C(0.1), &u_filtered, &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_get_state(&raw, NULL, NULL, &d_raw, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_get_state(&filtered, NULL, NULL, &d_filtered, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), d_raw);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), d_filtered);
    TEST_ASSERT_TRUE(u_filtered > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(u_filtered < u_raw);
}

/* RON-TC-FF-007 | RON-FR-203 */
void test_ron_tc_ff_007(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(0.0);
    cfg.u_max       = RON_FLOAT_C(5.0);
    cfg.du_max      = RON_FLOAT_C(10.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(100.0));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                                         RON_FLOAT_C(0.1), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_SATURATED) != 0U);
    TEST_ASSERT_TRUE((status & RON_STATUS_RATE_LIMITED) != 0U);
}

/* RON-TC-FF-008 | RON-FR-204 */
void test_ron_tc_ff_008(void)
{
    ron_pid_config_t cfg_plain    = test_ron_make_pid_cfg();
    ron_pid_config_t cfg_disabled = test_ron_make_pid_cfg();
    ron_pid_instance_t plain;
    ron_pid_instance_t disabled;
    ron_float_t u_plain          = RON_FLOAT_C(0.0);
    ron_float_t u_disabled       = RON_FLOAT_C(0.0);
    ron_status_t status_plain    = RON_STATUS_OK;
    ron_status_t status_disabled = RON_STATUS_OK;
    unsigned index;

    cfg_plain.Kp             = RON_FLOAT_C(1.25);
    cfg_plain.Ki             = RON_FLOAT_C(0.5);
    cfg_disabled             = cfg_plain;
    cfg_disabled.feedforward = test_ron_ff_cfg(RON_FF_DISABLED, RON_FLOAT_C(0.0));
    plain                    = test_ron_ff_init_pid(&cfg_plain);
    disabled                 = test_ron_ff_init_pid(&cfg_disabled);

    for (index = 0U; index < 1000U; ++index) {
        ron_float_t r = (ron_float_t) (index % 17U) * RON_FLOAT_C(0.1);
        ron_float_t y = (ron_float_t) (index % 11U) * RON_FLOAT_C(0.05);

        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE, ron_pid_step(&plain, r, y, RON_FLOAT_C(0.01), &u_plain, &status_plain));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&disabled, r, y, RON_FLOAT_C(0.01),
                                                             &u_disabled, &status_disabled));
        TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, u_plain, u_disabled);
        TEST_ASSERT_EQUAL_UINT16(status_plain, status_disabled);
    }
}

/* RON-TC-FF-009 | RON-FR-205 */
void test_ron_tc_ff_009(void)
{
    ron_pid_config_t cfg            = test_ron_make_pid_cfg();
    ron_feedforward_config_t ff_cfg = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(0.25));
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_float_t u_ff    = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    pid = test_ron_ff_init_pid(&cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_set_feedforward(&pid, &ff_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(4.0), RON_FLOAT_C(1.0),
                                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_TRUE((status & RON_STATUS_FF_ACTIVE) != 0U);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_feedforward(&pid, &u_ff));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(4.0) * FLT_EPSILON, RON_FLOAT_C(1.0), u_ff);
}

/* RON-TC-FF-005 | RON-FR-201 */
void test_ron_tc_ff_005_api_rejects_invalid_external_calls(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_pid_instance_t uninit;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.feedforward             = test_ron_ff_cfg(RON_FF_EXTERNAL, RON_FLOAT_C(0.0));
    pid                         = test_ron_ff_init_pid(&cfg);
    uninit.state.is_initialised = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_pid_step_feedforward(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), NULL,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&uninit, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     test_ron_ff_make_inf(), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), test_ron_ff_make_nan(), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), test_ron_ff_make_neg_inf(),
                                                     &u, &status));

    pid.config.feedforward.mode = RON_FF_DISABLED;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u,
                                                     &status));
}

/* RON-TC-FF-005 | RON-FR-201 */
void test_ron_tc_ff_005_latched_fault_path(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(1.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.feedforward = test_ron_ff_cfg(RON_FF_EXTERNAL, RON_FLOAT_C(0.0));
    cfg.safe_policy = RON_SAFE_ZERO;
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
                            ron_pid_step_feedforward(&pid, test_ron_ff_make_nan(), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
                            ron_pid_step_feedforward(&pid, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                                     RON_FLOAT_C(0.1), RON_FLOAT_C(0.0), &u,
                                                     &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), u);
}

/* RON-TC-FF-006 | RON-FR-202 */
void test_ron_tc_ff_006_config_validation(void)
{
    ron_feedforward_config_t cfg = test_ron_ff_cfg(RON_FF_DISABLED, RON_FLOAT_C(0.0));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_feedforward_config_validate(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_feedforward_config_validate(&cfg));

    cfg.mode = RON_FF_STATIC_GAIN;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_feedforward_config_validate(&cfg));
    cfg.mode = RON_FF_VELOCITY;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_feedforward_config_validate(&cfg));
    cfg.mode = RON_FF_ACCELERATION;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_feedforward_config_validate(&cfg));
    cfg.mode = RON_FF_EXTERNAL;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_feedforward_config_validate(&cfg));

    cfg.mode = (ron_feedforward_mode_t) 99;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
    cfg = test_ron_ff_cfg(RON_FF_STATIC_GAIN, test_ron_ff_make_nan());
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
    cfg = test_ron_ff_cfg(RON_FF_STATIC_GAIN, test_ron_ff_make_inf());
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
    cfg = test_ron_ff_cfg(RON_FF_STATIC_GAIN, test_ron_ff_make_neg_inf());
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
    cfg      = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(1.0));
    cfg.N_ff = RON_FLOAT_C(-1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
    cfg.N_ff = test_ron_ff_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_feedforward_config_validate(&cfg));
}

/* RON-TC-FF-009 | RON-FR-205 */
void test_ron_tc_ff_009_accessors_reject_invalid_inputs(void)
{
    ron_pid_config_t cfg   = test_ron_make_pid_cfg();
    ron_pid_instance_t pid = test_ron_ff_init_pid(&cfg);
    ron_pid_instance_t uninit;
    ron_feedforward_config_t ff_cfg = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(0.5));
    ron_float_t u_ff                = RON_FLOAT_C(0.0);

    uninit.state.is_initialised = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_set_feedforward(NULL, &ff_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_set_feedforward(&pid, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_pid_set_feedforward(&uninit, &ff_cfg));
    ff_cfg.mode = (ron_feedforward_mode_t) 99;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_pid_set_feedforward(&pid, &ff_cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_get_feedforward(NULL, &u_ff));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_get_feedforward(&pid, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_pid_get_feedforward(&uninit, &u_ff));
}

/* RON-TC-FF-008 | RON-FR-204 */
void test_ron_tc_ff_008_zero_gain_static_is_inactive(void)
{
    ron_pid_config_t cfg = test_ron_make_pid_cfg();
    ron_pid_instance_t pid;
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_float_t u_ff    = RON_FLOAT_C(1.0);
    ron_status_t status = RON_STATUS_OK;

    cfg.Kp          = RON_FLOAT_C(1.0);
    cfg.feedforward = test_ron_ff_cfg(RON_FF_STATIC_GAIN, RON_FLOAT_C(0.0));
    pid             = test_ron_ff_init_pid(&cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_step(&pid, RON_FLOAT_C(2.0), RON_FLOAT_C(1.0),
                                                         RON_FLOAT_C(0.01), &u, &status));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(1.0), u);
    TEST_ASSERT_TRUE((status & RON_STATUS_FF_ACTIVE) == 0U);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_get_feedforward(&pid, &u_ff));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6), RON_FLOAT_C(0.0), u_ff);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_ff_001);
    RUN_TEST(test_ron_tc_ff_002);
    RUN_TEST(test_ron_tc_ff_003);
    RUN_TEST(test_ron_tc_ff_004);
    RUN_TEST(test_ron_tc_ff_005);
    RUN_TEST(test_ron_tc_ff_006);
    RUN_TEST(test_ron_tc_ff_007);
    RUN_TEST(test_ron_tc_ff_008);
    RUN_TEST(test_ron_tc_ff_009);
    RUN_TEST(test_ron_tc_ff_005_api_rejects_invalid_external_calls);
    RUN_TEST(test_ron_tc_ff_005_latched_fault_path);
    RUN_TEST(test_ron_tc_ff_006_config_validation);
    RUN_TEST(test_ron_tc_ff_009_accessors_reject_invalid_inputs);
    RUN_TEST(test_ron_tc_ff_008_zero_gain_static_is_inactive);
    return UNITY_END();
}
