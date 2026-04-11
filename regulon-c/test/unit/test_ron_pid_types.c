/*
 * @file     test_ron_pid_types.c
 * @brief    Sprint 1 unit tests — type sizes, enum values, and NULL-safety stubs.
 * @module   test_ron_pid_types
 * @doc      RON-IS-001
 * @req      RON-PR-021, RON-QR-001, RON-QR-003, RON-DC-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Test cases implemented here:
 *   RON-TC-PERF-005  sizeof(ron_pid_config_t) <= 128
 *   RON-TC-PERF-006  sizeof(ron_pid_state_t)  <= 128
 *   RON-TC-QUAL-005  Portability — compile without warnings on host
 *   RON-TC-QUAL-007  Exact-width integer types — no plain int/long in types
 *   RON-TC-SAFE-006  NULL pointer → RON_FAULT_NULL_POINTER
 */

#include "unity.h"
#include "ron/ron_pid.h"

#include <stddef.h> /* size_t */

/* =========================================================================
 * Unity setUp / tearDown (called before / after every test function)
 * ========================================================================= */

void setUp(void)
{
    /* Nothing to set up for pure type-system tests. */
}

void tearDown(void)
{
    /* Nothing to tear down. */
}

/* =========================================================================
 * RON-TC-PERF-005 — sizeof(ron_pid_config_t) <= 128 bytes
 *
 * Requirement: RON-PR-021 — RAM footprint of a single controller instance
 * shall not exceed 128 bytes for the configuration record.
 * ========================================================================= */

/* RON-TC-PERF-005 | RON-PR-021 */
void test_ron_tc_perf_005(void)
{
    size_t sz = sizeof(ron_pid_config_t);
    /* Report actual size in the failure message for easy diagnosis. */
    TEST_ASSERT_MESSAGE(sz <= 128U,
        "ron_pid_config_t exceeds 128-byte RAM budget (RON-PR-021)");
}

/* =========================================================================
 * RON-TC-PERF-006 — sizeof(ron_pid_state_t) <= 128 bytes
 *
 * Requirement: RON-PR-021 — RAM footprint of the dynamic state record shall
 * not exceed 128 bytes.
 * ========================================================================= */

/* RON-TC-PERF-006 | RON-PR-021 */
void test_ron_tc_perf_006(void)
{
    size_t sz = sizeof(ron_pid_state_t);
    TEST_ASSERT_MESSAGE(sz <= 128U,
        "ron_pid_state_t exceeds 128-byte RAM budget (RON-PR-021)");
}

/* =========================================================================
 * RON-TC-QUAL-005 — Portability
 *
 * Verifying that the header compiles and that ron_float_t has the expected
 * size for the configured precision (32-bit float by default).
 * ========================================================================= */

/* RON-TC-QUAL-005 | RON-QR-001 */
void test_ron_tc_qual_005(void)
{
#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
    /* Double-precision build: ron_float_t must be 64-bit. */
    TEST_ASSERT_EQUAL_UINT(8U, (unsigned)sizeof(ron_float_t));
#else
    /* Default single-precision build: ron_float_t must be 32-bit. */
    TEST_ASSERT_EQUAL_UINT(4U, (unsigned)sizeof(ron_float_t));
#endif
}

/* =========================================================================
 * RON-TC-QUAL-007 — Exact-width integer types
 *
 * Verifies that ron_fault_t is exactly 1 byte (uint8_t) and ron_status_t
 * is exactly 2 bytes (uint16_t).  This ensures no implicit widening or
 * platform-dependent sizes crept into the type definitions.
 * ========================================================================= */

/* RON-TC-QUAL-007 | RON-QR-003 */
void test_ron_tc_qual_007(void)
{
    /* ron_fault_t must be uint8_t (1 byte). */
    TEST_ASSERT_EQUAL_UINT(1U, (unsigned)sizeof(ron_fault_t));

    /* ron_status_t must be uint16_t (2 bytes). */
    TEST_ASSERT_EQUAL_UINT(2U, (unsigned)sizeof(ron_status_t));
}

/* =========================================================================
 * RON-TC-QUAL-007b — Enum values are named constants (no magic numbers)
 *
 * Spot-checks that every expected enum constant exists and has the
 * documented value.  These compile-time constants are exercised by the
 * test; if any were undefined the compiler would error out.
 * ========================================================================= */

/* RON-TC-QUAL-007 (enum constants) | RON-QR-013 */
void test_ron_tc_qual_007_enum_values(void)
{
    /* ron_aw_mode_t */
    TEST_ASSERT_EQUAL_INT(0, (int)RON_AW_NONE);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_AW_BACK_CALC);
    TEST_ASSERT_EQUAL_INT(2, (int)RON_AW_CLAMPING);

    /* ron_deriv_mode_t */
    TEST_ASSERT_EQUAL_INT(0, (int)RON_DERIV_ON_ERROR);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_DERIV_ON_MEASUREMENT);

    /* ron_op_mode_t */
    TEST_ASSERT_EQUAL_INT(0, (int)RON_MODE_AUTOMATIC);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_MODE_MANUAL);

    /* ron_safe_policy_t */
    TEST_ASSERT_EQUAL_INT(0, (int)RON_SAFE_HOLD_LAST);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_SAFE_ZERO);
    TEST_ASSERT_EQUAL_INT(2, (int)RON_SAFE_CONSTANT);

    /* ron_integ_method_t */
    TEST_ASSERT_EQUAL_INT(0, (int)RON_INTEG_EULER);
    TEST_ASSERT_EQUAL_INT(1, (int)RON_INTEG_TRAPEZOIDAL);
}

/* =========================================================================
 * RON-TC-QUAL-007c — Fault and status bitmask constants
 * ========================================================================= */

/* RON-TC-QUAL-007 (bitmask constants) | RON-QR-013 */
void test_ron_tc_qual_007_bitmasks(void)
{
    /* Fault constants are single-bit masks. */
    TEST_ASSERT_EQUAL_UINT8(0x00U, RON_FAULT_NONE);
    TEST_ASSERT_EQUAL_UINT8(0x01U, RON_FAULT_INPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0x02U, RON_FAULT_OUTPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0x04U, RON_FAULT_INTEGRAL_OVERFLOW);
    TEST_ASSERT_EQUAL_UINT8(0x08U, RON_FAULT_CONFIG_INVALID);
    TEST_ASSERT_EQUAL_UINT8(0x10U, RON_FAULT_NULL_POINTER);

    /* Status constants are single-bit masks. */
    TEST_ASSERT_EQUAL_UINT16(0x0000U, RON_STATUS_OK);
    TEST_ASSERT_EQUAL_UINT16(0x0001U, RON_STATUS_SATURATED);
    TEST_ASSERT_EQUAL_UINT16(0x0002U, RON_STATUS_RATE_LIMITED);
    TEST_ASSERT_EQUAL_UINT16(0x0004U, RON_STATUS_AW_ACTIVE);
    TEST_ASSERT_EQUAL_UINT16(0x0008U, RON_STATUS_MANUAL_MODE);
    TEST_ASSERT_EQUAL_UINT16(0x0010U, RON_STATUS_FAULT);
    TEST_ASSERT_EQUAL_UINT16(0x0020U, RON_STATUS_SP_FILTER_ACTIVE);
    TEST_ASSERT_EQUAL_UINT16(0x0040U, RON_STATUS_NORMALISED);

    /* No two fault bits overlap. */
    TEST_ASSERT_EQUAL_UINT8(0U,
        RON_FAULT_INPUT_NAN & RON_FAULT_OUTPUT_NAN);
    TEST_ASSERT_EQUAL_UINT8(0U,
        RON_FAULT_OUTPUT_NAN & RON_FAULT_INTEGRAL_OVERFLOW);
    TEST_ASSERT_EQUAL_UINT8(0U,
        RON_FAULT_CONFIG_INVALID & RON_FAULT_NULL_POINTER);
}

/* =========================================================================
 * RON-TC-SAFE-006 — NULL pointer → RON_FAULT_NULL_POINTER
 *
 * Every public API function that accepts a pointer parameter must return
 * RON_FAULT_NULL_POINTER when that pointer is NULL.
 * ========================================================================= */

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_init_null_inst(void)
{
    /* We only need a pointer to cfg; the contents don't matter because the
     * null inst check fires before cfg is read. */
    ron_pid_config_t cfg = { .Kp = RON_FLOAT_C(0.0) }; /* suppress -Wmaybe-uninitialized */
    ron_fault_t result = ron_pid_init(NULL, &cfg);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_init_null_cfg(void)
{
    ron_pid_instance_t inst;
    ron_fault_t result = ron_pid_init(&inst, NULL);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_step_null_inst(void)
{
    ron_float_t  u      = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t result = ron_pid_step(NULL,
                                      RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.001),
                                      &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, result);
}

/* RON-TC-SAFE-006 | RON-SR-006 */
void test_ron_tc_safe_006_step_null_output(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg = {
        .Kp = RON_FLOAT_C(1.0),
        .Ki = RON_FLOAT_C(0.0),
        .Kd = RON_FLOAT_C(0.0),
        .N  = RON_FLOAT_C(0.0),
        .b  = RON_FLOAT_C(1.0),
        .c  = RON_FLOAT_C(1.0),
        .u_min = RON_FLOAT_C(-1.0),
        .u_max = RON_FLOAT_C(1.0),
        .du_max = RON_FLOAT_C(0.0),
        .I_min  = RON_FLOAT_C(-1.0),
        .I_max  = RON_FLOAT_C(1.0),
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
    (void)ron_pid_init(&inst, &cfg);

    ron_status_t status = RON_STATUS_OK;
    /* u_out is NULL — must return NULL_POINTER without crashing */
    ron_fault_t result = ron_pid_step(&inst,
                                      RON_FLOAT_C(1.0),
                                      RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.001),
                                      NULL, &status);
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

/* =========================================================================
 * RON-TC-QUAL-005b — Platform dimension constants are positive
 * ========================================================================= */

/* RON-TC-QUAL-005 (dimension constants) | RON-PR-022 */
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

/* =========================================================================
 * RON-TC-QUAL-005c — ron_clamp and ron_fabs inline utilities
 * ========================================================================= */

/* RON-TC-QUAL-005 (inline utilities) | RON-FR-020 */
void test_ron_tc_qual_005_clamp(void)
{
    /* Within range — unchanged. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
        RON_FLOAT_C(0.5),
        ron_clamp(RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));

    /* Below lo — clamped to lo. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
        RON_FLOAT_C(0.0),
        ron_clamp(RON_FLOAT_C(-5.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));

    /* Above hi — clamped to hi. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
        RON_FLOAT_C(1.0),
        ron_clamp(RON_FLOAT_C(9.9), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)));

    /* Negative range. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
        RON_FLOAT_C(-10.0),
        ron_clamp(RON_FLOAT_C(-100.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(10.0)));

    /* Exact boundary — lo. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1e-6),
        RON_FLOAT_C(-1.0),
        ron_clamp(RON_FLOAT_C(-1.0), RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0)));
}

/* RON-TC-QUAL-005 (inline utilities) | RON-SR-021 */
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

/* =========================================================================
 * RON-TC-QUAL-005d — RON_ISNAN / RON_ISINF / RON_ISFINITE macros
 * ========================================================================= */

/* RON-TC-QUAL-005 (float classification) | RON-SR-020 */
void test_ron_tc_qual_005_float_classification(void)
{
    /* Build NaN and Inf from arithmetic (avoids <math.h>). */
    volatile ron_float_t zero = RON_FLOAT_C(0.0);
    volatile ron_float_t big  = RON_FLOAT_MAX;

    ron_float_t nan_val = zero / zero;   /* 0/0 → NaN on IEEE 754 platforms */
    ron_float_t inf_val = big * big;     /* overflow → +Inf                 */

    /* NaN detection */
    TEST_ASSERT_TRUE(RON_ISNAN(nan_val));
    TEST_ASSERT_FALSE(RON_ISNAN(RON_FLOAT_C(1.0)));
    TEST_ASSERT_FALSE(RON_ISNAN(inf_val));

    /* Inf detection */
    TEST_ASSERT_TRUE(RON_ISINF(inf_val));
    TEST_ASSERT_FALSE(RON_ISINF(RON_FLOAT_C(1.0)));
    TEST_ASSERT_FALSE(RON_ISINF(nan_val));

    /* Finite detection */
    TEST_ASSERT_TRUE(RON_ISFINITE(RON_FLOAT_C(1.0)));
    TEST_ASSERT_TRUE(RON_ISFINITE(RON_FLOAT_C(0.0)));
    TEST_ASSERT_FALSE(RON_ISFINITE(inf_val));
    TEST_ASSERT_FALSE(RON_ISFINITE(nan_val));
}

/* =========================================================================
 * Unity test runner — main()
 * ========================================================================= */

int main(void)
{
    UNITY_BEGIN();

    /* RON-TC-PERF-005 */
    RUN_TEST(test_ron_tc_perf_005);

    /* RON-TC-PERF-006 */
    RUN_TEST(test_ron_tc_perf_006);

    /* RON-TC-QUAL-005 */
    RUN_TEST(test_ron_tc_qual_005);
    RUN_TEST(test_ron_tc_qual_005_dimension_constants);
    RUN_TEST(test_ron_tc_qual_005_clamp);
    RUN_TEST(test_ron_tc_qual_005_fabs);
    RUN_TEST(test_ron_tc_qual_005_float_classification);

    /* RON-TC-QUAL-007 */
    RUN_TEST(test_ron_tc_qual_007);
    RUN_TEST(test_ron_tc_qual_007_enum_values);
    RUN_TEST(test_ron_tc_qual_007_bitmasks);

    /* RON-TC-SAFE-006 */
    RUN_TEST(test_ron_tc_safe_006_init_null_inst);
    RUN_TEST(test_ron_tc_safe_006_init_null_cfg);
    RUN_TEST(test_ron_tc_safe_006_step_null_inst);
    RUN_TEST(test_ron_tc_safe_006_step_null_output);
    RUN_TEST(test_ron_tc_safe_006_get_state_null);
    RUN_TEST(test_ron_tc_safe_006_config_validate_null);
    RUN_TEST(test_ron_tc_safe_006_fault_clear_null);

    return UNITY_END();
}
