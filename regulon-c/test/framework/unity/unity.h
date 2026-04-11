/*
 * unity.h — Unity Test Framework v2.6.0 public API.
 *
 * Unity is copyright (c) 2007–2021 Mike Karlesky, Mark VanderVoord, Greg Williams.
 * Licensed under the MIT License.
 *
 * See: https://github.com/ThrowTheSwitch/Unity
 * SPDX-License-Identifier: MIT
 */

#ifndef UNITY_FRAMEWORK_H
#define UNITY_FRAMEWORK_H

#include "unity_internals.h"

/* =========================================================================
 * Test runner macros
 * ========================================================================= */

/**
 * @brief Begin a Unity test suite.  Call once before all RUN_TEST() calls.
 * @return Always 0 (use with UNITY_END() return value).
 */
int UnityBegin(const char *filename);

/**
 * @brief End a Unity test suite and print summary.
 * @return Number of test failures (0 = all passed).
 */
int UnityEnd(void);

/** Begin the test suite. */
#define UNITY_BEGIN()  UnityBegin(__FILE__)

/** End the test suite and return failure count to main(). */
#define UNITY_END()    UnityEnd()

/**
 * @brief Run one test function and record pass/fail.
 * @param func  Pointer to the void test function.
 */
#define RUN_TEST(func)  UnityDefaultTestRun((func), #func, __LINE__)

/* =========================================================================
 * Basic assertions
 * ========================================================================= */

/** Unconditionally fail the current test with a message. */
#define TEST_FAIL_MESSAGE(message) \
    UnityFail_((message), __LINE__)

/** Unconditionally fail the current test (no message). */
#define TEST_FAIL() \
    TEST_FAIL_MESSAGE(NULL)

/** Pass only if condition is true. */
#define TEST_ASSERT(condition) \
    UNITY_TEST_ASSERT((condition), __LINE__, "Expression Evaluated To FALSE")

/** Pass only if condition is true, with a custom failure message. */
#define TEST_ASSERT_MESSAGE(condition, message) \
    UNITY_TEST_ASSERT((condition), __LINE__, (message))

/** Pass only if condition is true. */
#define TEST_ASSERT_TRUE(condition) \
    UNITY_TEST_ASSERT((condition), __LINE__, "Expected TRUE Was FALSE")

/** Pass only if condition is true, with a custom failure message. */
#define TEST_ASSERT_TRUE_MESSAGE(condition, message) \
    UNITY_TEST_ASSERT((condition), __LINE__, (message))

/** Pass only if condition is false. */
#define TEST_ASSERT_FALSE(condition) \
    UNITY_TEST_ASSERT(!(condition), __LINE__, "Expected FALSE Was TRUE")

/** Pass only if condition is false, with a custom failure message. */
#define TEST_ASSERT_FALSE_MESSAGE(condition, message) \
    UNITY_TEST_ASSERT(!(condition), __LINE__, (message))

/** Pass only if pointer is NULL. */
#define TEST_ASSERT_NULL(pointer) \
    UNITY_TEST_ASSERT_NULL((pointer), __LINE__, "Expected NULL")

/** Pass only if pointer is NULL, with a custom failure message. */
#define TEST_ASSERT_NULL_MESSAGE(pointer, message) \
    UNITY_TEST_ASSERT_NULL((pointer), __LINE__, (message))

/** Pass only if pointer is NOT NULL. */
#define TEST_ASSERT_NOT_NULL(pointer) \
    UNITY_TEST_ASSERT_NOT_NULL((pointer), __LINE__, "Expected Non-NULL")

/** Pass only if pointer is NOT NULL, with a custom failure message. */
#define TEST_ASSERT_NOT_NULL_MESSAGE(pointer, message) \
    UNITY_TEST_ASSERT_NOT_NULL((pointer), __LINE__, (message))

/* =========================================================================
 * Integer equality assertions
 * ========================================================================= */

/** Pass only if expected == actual (compared as int32_t). */
#define TEST_ASSERT_EQUAL(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, NULL)

/** Pass only if expected == actual (compared as int32_t), with message. */
#define TEST_ASSERT_EQUAL_MESSAGE(expected, actual, message) \
    UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, (message))

/** Pass only if expected == actual (compared as int32_t). */
#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_INT((expected), (actual), __LINE__, NULL)

/** Pass only if expected == actual (compared as uint32_t). */
#define TEST_ASSERT_EQUAL_UINT(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_UINT((expected), (actual), __LINE__, NULL)

/** Pass only if expected == actual (compared as uint8_t). */
#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_UINT((uint32_t)(expected), (uint32_t)(actual), __LINE__, NULL)

/** Pass only if expected == actual (compared as uint16_t). */
#define TEST_ASSERT_EQUAL_UINT16(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_UINT((uint32_t)(expected), (uint32_t)(actual), __LINE__, NULL)

/* =========================================================================
 * Floating-point assertions
 * ========================================================================= */

/**
 * @brief Pass only if |expected - actual| <= delta.
 * @param delta    Maximum allowed absolute difference.
 * @param expected Expected float value.
 * @param actual   Actual float value.
 */
#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    UNITY_TEST_ASSERT_FLOAT_WITHIN((delta), (expected), (actual), __LINE__, NULL)

/**
 * @brief Pass only if |expected - actual| <= delta, with message.
 */
#define TEST_ASSERT_FLOAT_WITHIN_MESSAGE(delta, expected, actual, message) \
    UNITY_TEST_ASSERT_FLOAT_WITHIN((delta), (expected), (actual), __LINE__, (message))

/* =========================================================================
 * String assertions
 * ========================================================================= */

/** Pass only if the two strings are equal (strcmp == 0). */
#define TEST_ASSERT_EQUAL_STRING(expected, actual) \
    UNITY_TEST_ASSERT_EQUAL_STRING((expected), (actual), __LINE__, NULL)

/* =========================================================================
 * Size assertions (useful for RON-PR-021)
 * ========================================================================= */

/** Pass only if actual size is less-or-equal to the expected maximum. */
#define TEST_ASSERT_LESS_OR_EQUAL_size_t(threshold, actual) \
    UNITY_TEST_ASSERT(((size_t)(actual) <= (size_t)(threshold)), __LINE__, \
                      "Size exceeds threshold")

#endif /* UNITY_FRAMEWORK_H */
