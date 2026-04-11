/*
 * unity_internals.h — Unity Test Framework (v2.6.0) internal definitions.
 *
 * Unity is copyright (c) 2007–2021 Mike Karlesky, Mark VanderVoord, Greg Williams.
 * Licensed under the MIT License.
 *
 * This is a self-contained, header-only-friendly vendor copy.
 * SPDX-License-Identifier: MIT
 */

#ifndef UNITY_INTERNALS_H
#define UNITY_INTERNALS_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>   /* printf, putchar */
#include <string.h>  /* strlen          */
#include <math.h>    /* fabsf, fabs     */

/* -------------------------------------------------------------------------
 * Unity configuration defaults
 * ------------------------------------------------------------------------- */
#ifndef UNITY_OUTPUT_CHAR
#  define UNITY_OUTPUT_CHAR(a)   (void)putchar(a)
#endif

#ifndef UNITY_OUTPUT_FLUSH
#  define UNITY_OUTPUT_FLUSH()   (void)fflush(stdout)
#endif

#ifndef UNITY_OUTPUT_START
#  define UNITY_OUTPUT_START()
#endif

#ifndef UNITY_OUTPUT_COMPLETE
#  define UNITY_OUTPUT_COMPLETE()
#endif

#ifndef UNITY_PRINT_EOL
#  define UNITY_PRINT_EOL()      UNITY_OUTPUT_CHAR('\n')
#endif

/* -------------------------------------------------------------------------
 * Internal state structure
 * ------------------------------------------------------------------------- */
typedef struct UNITY_STORAGE_T
{
    const char  *TestFile;
    const char  *CurrentTestName;
    uint32_t     CurrentTestLineNumber;
    uint32_t     NumberOfTests;
    uint32_t     TestFailures;
    uint32_t     TestIgnores;
    uint8_t      CurrentTestFailed;
    uint8_t      CurrentTestIgnored;
} UNITY_STORAGE_T;

extern UNITY_STORAGE_T Unity;

/* -------------------------------------------------------------------------
 * Internal helper prototypes (implemented in unity.c)
 * ------------------------------------------------------------------------- */
typedef void (*UnityTestFunction)(void);

void UnityPrint(const char *string);
void UnityPrintLen(const char *string, uint32_t length);
void UnityPrintNumberInt(int32_t number);
void UnityPrintNumberUInt(uint32_t number);
void UnityPrintFloat(double number);
void UnityTestResultsBegin(const char *file, uint32_t line);
void UnityTestResultsFailBegin(uint32_t line);
void UnityAddMsgIfSpecified(const char *msg);
void UnityPrintExpectedAndActualStrings(const char *expected, const char *actual);
void UnityDefaultTestRun(UnityTestFunction Func, const char *FuncName, int FuncLineNum);

void UnityFail_(const char *msg, uint32_t line);
void UnityIgnore_(const char *msg, uint32_t line);

/* -------------------------------------------------------------------------
 * Assertion helper macros (used by the public assertion macros)
 * ------------------------------------------------------------------------- */
#define UNITY_TEST_ASSERT(condition, line, message) \
    do { \
        if (!(condition)) { UnityFail_((message), (uint32_t)(line)); } \
    } while (0)

#define UNITY_TEST_ASSERT_NULL(pointer, line, message) \
    UNITY_TEST_ASSERT(((pointer) == NULL), (line), (message))

#define UNITY_TEST_ASSERT_NOT_NULL(pointer, line, message) \
    UNITY_TEST_ASSERT(((pointer) != NULL), (line), (message))

#define UNITY_TEST_ASSERT_EQUAL_INT(expected, actual, line, message) \
    do { \
        if ((int32_t)(expected) != (int32_t)(actual)) { \
            printf("%s:%u:FAIL: Expected %d Was %d", \
                   Unity.TestFile, (unsigned)(line), \
                   (int)(expected), (int)(actual)); \
            if ((const char *)(message) != NULL) { printf(" %s", (const char *)(message)); } \
            UNITY_PRINT_EOL(); \
            Unity.CurrentTestFailed = 1U; \
        } \
    } while (0)

#define UNITY_TEST_ASSERT_EQUAL_UINT(expected, actual, line, message) \
    do { \
        if ((uint32_t)(expected) != (uint32_t)(actual)) { \
            printf("%s:%u:FAIL: Expected %u Was %u", \
                   Unity.TestFile, (unsigned)(line), \
                   (unsigned)(expected), (unsigned)(actual)); \
            if ((const char *)(message) != NULL) { printf(" %s", (const char *)(message)); } \
            UNITY_PRINT_EOL(); \
            Unity.CurrentTestFailed = 1U; \
        } \
    } while (0)

#define UNITY_TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual, line, message) \
    do { \
        double _d = (double)(delta); \
        double _e = (double)(expected); \
        double _a = (double)(actual); \
        if (fabs(_e - _a) > fabs(_d)) { \
            printf("%s:%u:FAIL: Expected %f within %f delta but was %f", \
                   Unity.TestFile, (unsigned)(line), _e, _d, _a); \
            if ((const char *)(message) != NULL) { printf(" %s", (const char *)(message)); } \
            UNITY_PRINT_EOL(); \
            Unity.CurrentTestFailed = 1U; \
        } \
    } while (0)

#define UNITY_TEST_ASSERT_EQUAL_STRING(expected, actual, line, message) \
    do { \
        if (strcmp((expected), (actual)) != 0) { \
            printf("%s:%u:FAIL: Expected \"%s\" Was \"%s\"", \
                   Unity.TestFile, (unsigned)(line), \
                   (expected), (actual)); \
            if ((const char *)(message) != NULL) { printf(" %s", (const char *)(message)); } \
            UNITY_PRINT_EOL(); \
            Unity.CurrentTestFailed = 1U; \
        } \
    } while (0)

#endif /* UNITY_INTERNALS_H */
