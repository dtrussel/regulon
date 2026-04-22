/*
 * @file     test_ron_pid_types.c
 * @brief    Type, platform, and NULL-safety unit tests for the PID slice.
 * @module   test_ron_pid_types
 * @doc      RON-IS-001
 * @req      RON-PR-010, RON-PR-011, RON-PR-021, RON-QR-001, RON-QR-003,
 *           RON-DC-001, RON-SR-006
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Test cases implemented here:
 *   RON-TC-PERF-005  precision-mode sanity and single-precision config budget
 *   RON-TC-PERF-006  single-precision state budget
 *   RON-TC-QUAL-005  portability and platform helper sanity
 *   RON-TC-QUAL-007  exact-width public types and named constants
 *   RON-TC-SAFE-006  NULL pointer -> RON_FAULT_NULL_POINTER
 */

#include "unity.h"
#include "ron/ron_pid.h"

#include <stddef.h>

void setUp(void)
{
}

void tearDown(void)
{
}

/* RON-TC-PERF-005 | RON-PR-010, RON-PR-011, RON-PR-021 */
void test_ron_tc_perf_005(void)
{
#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
    TEST_ASSERT_EQUAL_UINT(8U, (unsigned)sizeof(ron_float_t));
#else
    size_t sz = sizeof(ron_pid_config_t);

    TEST_ASSERT_EQUAL_UINT(4U, (unsigned)sizeof(ron_float_t));
    TEST_ASSERT_MESSAGE(sz <= 128U,
        "ron_pid_config_t exceeds 128-byte RAM budget (RON-PR-021)");
#endif
}

/* RON-TC-PERF-006 | RON-PR-021 */
void test_ron_tc_perf_006(void)
{
#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
    TEST_ASSERT_TRUE(sizeof(ron_pid_state_t) > 0U);
#else
    size_t sz = sizeof(ron_pid_state_t);

    TEST_ASSERT_MESSAGE(sz <= 128U,
        "ron_pid_state_t exceeds 128-byte RAM budget (RON-PR-021)");
#endif
}

/* RON-TC-QUAL-005 | RON-QR-001 */
void test_ron_tc_qual_005(void)
{
#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
    TEST_ASSERT_EQUAL_UINT(8U, (unsigned)sizeof(ron_float_t));
#else
    TEST_ASSERT_EQUAL_UINT(4U, (unsigned)sizeof(ron_float_t));
#endif
}

/* RON-TC-QUAL-007 | RON-QR-003 */
void test_ron_tc_qual_007(void)
{
    TEST_ASSERT_EQUAL_UINT(1U, (unsigned)sizeof(ron_fault_t));
    TEST_ASSERT_EQUAL_UINT(2U, (unsigned)sizeof(ron_status_t));
}

/* RON-TC-QUAL-007 | RON-QR-013 */
void test_ron_tc_qual_007_enum_values(void)
{
    TEST_ASSERT_EQUAL_INT(0, (int)RON_AW_NONE);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_AW_BACK_CALC);
    TEST_ASSERT_EQUAL_INT(2, (int)RON_AW_CLAMPING);
    TEST_ASSERT_EQUAL_INT(0, (int)RON_DERIV_ON_ERROR);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_DERIV_ON_MEASUREMENT);
    TEST_ASSERT_EQUAL_INT(0, (int)RON_MODE_AUTOMATIC);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_MODE_MANUAL);
    TEST_ASSERT_EQUAL_INT(0, (int)RON_SAFE_HOLD_LAST);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_SAFE_ZERO);
    TEST_ASSERT_EQUAL_INT(2, (int)RON_SAFE_CONSTANT);
    TEST_ASSERT_EQUAL_INT(0, (int)RON_INTEG_EULER);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_INTEG_TRAPEZOIDAL);
}

/* RON-TC-QUAL-007 | RON-QR-013 */
void test_ron_tc_qual_007_bitmasks(void)
{
    TEST_ASSERT_EQUAL_UINT8(0x00U, RON_FAULT_NONE);
    TEST_ASSERT_EQUAL_UINT8(0x01U, RON_FAULT_INPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0x02U, RON_FAULT_OUTPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0x04U, RON_FAULT_INTEGRAL_OVERFLOW);
    TEST_ASSERT_EQUAL_UINT8(0x08U, RON_FAULT_CONFIG_INVALID);
    TEST_ASSERT_EQUAL_UINT8(0x10U, RON_FAULT_NULL_POINTER);

    TEST_ASSERT_EQUAL_UINT16(0x0000U, RON_STATUS_OK);
    TEST_ASSERT_EQUAL_UINT16(0x0001U, RON_STATUS_SATURATED);
    TEST_ASSERT_EQUAL_UINT16(0x0002U, RON_STATUS_RATE_LIMITED);
    TEST_ASSERT_EQUAL_UINT16(0x0004U, RON_STATUS_AW_ACTIVE);
    TEST_ASSERT_EQUAL_UINT16(0x0008U, RON_STATUS_MANUAL_MODE);
    TEST_ASSERT_EQUAL_UINT16(0x0010U, RON_STATUS_FAULT);
    TEST_ASSERT_EQUAL_UINT16(0x0020U, RON_STATUS_SP_FILTER_ACTIVE);
    TEST_ASSERT_EQUAL_UINT16(0x0040U, RON_STATUS_NORMALISED);

    TEST_ASSERT_EQUAL_UINT8(0U, RON_FAULT_INPUT_NAN & RON_FAULT_OUTPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0U, RON_FAULT_OUTPUT_NAN & RON_FAULT_INTEGRAL_OVERFLOW);
    TEST_ASSERT_EQUAL_UINT8(0U, RON_FAULT_CONFIG_INVALID & RON_FAULT_NULL_POINTER);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_init_null_inst(void)
{
    ron_pid_config_t cfg = { .Kp = RON_FLOAT_C(0.0) };
    ron_fault_t      result = ron_pid_init(NULL, &cfg);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_init_null_cfg(void)
{
    ron_pid_instance_t inst;
    ron_fault_t        result = ron_pid_init(&inst, NULL);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_step_null_inst(void)
{
    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t  result = ron_pid_step(NULL,
                                       RON_FLOAT_C(0.0),
                                       RON_FLOAT_C(0.0),
                                       RON_FLOAT_C(0.001),
                                       &u,
                                       &status);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_step_null_output(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = {
        .Kp = RON_FLOAT_C(1.0),
        .Ki = RON_FLOAT_C(0.0),
        .Kd = RON_FLOAT_C(0.0),
        .N = RON_FLOAT_C(0.0),
        .b = RON_FLOAT_C(1.0),
        .c = RON_FLOAT_C(1.0),
        .u_min = RON_FLOAT_C(-1.0),
        .u_max = RON_FLOAT_C(1.0),
        .du_max = RON_FLOAT_C(0.0),
        .I_min = RON_FLOAT_C(-1.0),
        .I_max = RON_FLOAT_C(1.0),
        .aw_mode = RON_AW_NONE,
        .T_aw = RON_FLOAT_C(0.1),
        .integ_method = RON_INTEG_EULER,
        .deriv_mode = RON_DERIV_ON_MEASUREMENT,
        .tau_sp = RON_FLOAT_C(0.0),
        .normalise = false,
        .in_min = RON_FLOAT_C(0.0),
        .in_max = RON_FLOAT_C(1.0),
        .out_min = RON_FLOAT_C(0.0),
        .out_max = RON_FLOAT_C(1.0),
        .safe_policy = RON_SAFE_HOLD_LAST,
        .safe_value = RON_FLOAT_C(0.0),
        .I_overflow_thresh = RON_FLOAT_C(0.0),
        .sp_reset_threshold = RON_FLOAT_C(0.0),
        .fault_cb = NULL
    };
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t  result;

    (void)ron_pid_init(&inst, &cfg);
    result = ron_pid_step(&inst,
                          RON_FLOAT_C(1.0),
                          RON_FLOAT_C(0.0),
                          RON_FLOAT_C(0.001),
                          NULL,
                          &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_get_state_null(void)
{
    ron_fault_t result = ron_pid_get_state(NULL, NULL, NULL, NULL, NULL, NULL);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_config_validate_null(void)
{
    ron_fault_t result = ron_pid_config_validate(NULL);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_fault_clear_null(void)
{
    ron_fault_t result = ron_pid_fault_clear(NULL);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-QUAL-005 | RON-PR-022 */
void test_ron_tc_qual_005_dimension_constants(void)
{
    TEST_ASSERT_TRUE(RON_MA_MAX_WINDOW >= 2U);
    TEST_ASSERT_TRUE(RON_BIQUAD_MAX_SECTIONS >= 1U);
    TEST_ASSERT_TRUE(RON_GS_MAX_BREAKPOINTS >= 2U);
    TEST_ASSERT_TRUE(RON_KF_MAX_STATES >= 1U);
    TEST_ASSERT_TRUE(RON_KF_MAX_MEASUREMENTS >= 1U);
    TEST_ASSERT_TRUE(RON_KF_MAX_INPUTS >= 1U);
    TEST_ASSERT_TRUE(RON_SS_MAX_STATES >= 1U);
    TEST_ASSERT_TRUE(RON_SS_MAX_OUTPUTS >= 1U);
    TEST_ASSERT_TRUE(RON_SS_MAX_INPUTS >= 1U);
    TEST_ASSERT_TRUE(RON_AT_MIN_CYCLES >= 1U);
    TEST_ASSERT_TRUE(RON_HEALTH_OSC_WINDOW >= 4U);
}

/* RON-TC-QUAL-005 | RON-FR-020 */
void test_ron_tc_qual_005_clamp(void)
{
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(0.5),
                             ron_clamp(RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(0.0),
                             ron_clamp(RON_FLOAT_C(-5.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(1.0),
                             ron_clamp(RON_FLOAT_C(9.9), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(-10.0),
                             ron_clamp(RON_FLOAT_C(-100.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(10.0)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(-1.0),
                             ron_clamp(RON_FLOAT_C(-1.0), RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0)));
}

/* RON-TC-QUAL-005 | RON-SR-021 */
void test_ron_tc_qual_005_fabs(void)
{
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(3.14),
                             ron_fabs(RON_FLOAT_C(3.14)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
                             RON_FLOAT_C(3.14),
                             ron_fabs(RON_FLOAT_C(-3.14)));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-9),
                             RON_FLOAT_C(0.0),
                             ron_fabs(RON_FLOAT_C(0.0)));
}

/* RON-TC-QUAL-005 | RON-SR-020 */
void test_ron_tc_qual_005_float_classification(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    volatile ron_float_t big = RON_FLOAT_MAX;
    ron_float_t          nan_val = zero / zero;
    ron_float_t          inf_val = big * big;

    TEST_ASSERT_TRUE(RON_ISNAN(nan_val));
    TEST_ASSERT_FALSE(RON_ISNAN(RON_FLOAT_C(1.0)));
    TEST_ASSERT_FALSE(RON_ISNAN(inf_val));
    TEST_ASSERT_TRUE(RON_ISINF(inf_val));
    TEST_ASSERT_FALSE(RON_ISINF(RON_FLOAT_C(1.0)));
    TEST_ASSERT_FALSE(RON_ISINF(nan_val));
    TEST_ASSERT_TRUE(RON_ISFINITE(RON_FLOAT_C(1.0)));
    TEST_ASSERT_TRUE(RON_ISFINITE(RON_FLOAT_C(0.0)));
    TEST_ASSERT_FALSE(RON_ISFINITE(inf_val));
    TEST_ASSERT_FALSE(RON_ISFINITE(nan_val));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_perf_005);
    RUN_TEST(test_ron_tc_perf_006);
    RUN_TEST(test_ron_tc_qual_005);
    RUN_TEST(test_ron_tc_qual_005_dimension_constants);
    RUN_TEST(test_ron_tc_qual_005_clamp);
    RUN_TEST(test_ron_tc_qual_005_fabs);
    RUN_TEST(test_ron_tc_qual_005_float_classification);
    RUN_TEST(test_ron_tc_qual_007);
    RUN_TEST(test_ron_tc_qual_007_enum_values);
    RUN_TEST(test_ron_tc_qual_007_bitmasks);
    RUN_TEST(test_ron_tc_safe_006_init_null_inst);
    RUN_TEST(test_ron_tc_safe_006_init_null_cfg);
    RUN_TEST(test_ron_tc_safe_006_step_null_inst);
    RUN_TEST(test_ron_tc_safe_006_step_null_output);
    RUN_TEST(test_ron_tc_safe_006_get_state_null);
    RUN_TEST(test_ron_tc_safe_006_config_validate_null);
    RUN_TEST(test_ron_tc_safe_006_fault_clear_null);
    return UNITY_END();
}
