/*
 * unity.c — Unity Test Framework v2.6.0 implementation.
 *
 * Unity is copyright (c) 2007–2021 Mike Karlesky, Mark VanderVoord, Greg Williams.
 * Licensed under the MIT License.
 *
 * See: https://github.com/ThrowTheSwitch/Unity
 * SPDX-License-Identifier: MIT
 */

#include "unity.h"
#include <stdio.h>
#include <string.h>
#include <setjmp.h>

/* -------------------------------------------------------------------------
 * Global Unity state
 * ------------------------------------------------------------------------- */
UNITY_STORAGE_T Unity;

/* Used for non-local exit from a failing test function */
static jmp_buf s_unity_abort_frame;

/* -------------------------------------------------------------------------
 * Output helpers
 * ------------------------------------------------------------------------- */
void UnityPrint(const char *string)
{
    if (string != NULL)
    {
        while (*string != '\0')
        {
            UNITY_OUTPUT_CHAR((int)*string);
            string++;
        }
    }
}

void UnityPrintLen(const char *string, uint32_t length)
{
    if (string != NULL)
    {
        uint32_t i;
        for (i = 0U; i < length; i++)
        {
            if (string[i] == '\0') { break; }
            UNITY_OUTPUT_CHAR((int)string[i]);
        }
    }
}

void UnityPrintNumberInt(int32_t number)
{
    printf("%d", (int)number);
}

void UnityPrintNumberUInt(uint32_t number)
{
    printf("%u", (unsigned)number);
}

void UnityPrintFloat(double number)
{
    printf("%f", number);
}

/* -------------------------------------------------------------------------
 * Failure recording
 * ------------------------------------------------------------------------- */
void UnityFail_(const char *msg, uint32_t line)
{
    printf("%s:%u:FAIL", Unity.TestFile ? Unity.TestFile : "?", (unsigned)line);
    if (msg != NULL)
    {
        printf(": %s", msg);
    }
    UNITY_PRINT_EOL();
    Unity.CurrentTestFailed = 1U;
    /* Abort the current test function via long jump */
    longjmp(s_unity_abort_frame, 1);
}

void UnityIgnore_(const char *msg, uint32_t line)
{
    printf("%s:%u:IGNORE", Unity.TestFile ? Unity.TestFile : "?", (unsigned)line);
    if (msg != NULL)
    {
        printf(": %s", msg);
    }
    UNITY_PRINT_EOL();
    Unity.CurrentTestIgnored = 1U;
    longjmp(s_unity_abort_frame, 1);
}

/* -------------------------------------------------------------------------
 * Test runner
 * ------------------------------------------------------------------------- */
void UnityDefaultTestRun(UnityTestFunction Func, const char *FuncName, int FuncLineNum)
{
    Unity.NumberOfTests++;
    Unity.CurrentTestFailed  = 0U;
    Unity.CurrentTestIgnored = 0U;
    Unity.CurrentTestName    = FuncName;
    Unity.CurrentTestLineNumber = (uint32_t)FuncLineNum;

    UNITY_OUTPUT_FLUSH();

    if (setjmp(s_unity_abort_frame) == 0)
    {
        /* Run the test — longjmp returns here on TEST_FAIL */
        Func();
    }

    if (Unity.CurrentTestFailed != 0U)
    {
        Unity.TestFailures++;
    }
    else if (Unity.CurrentTestIgnored != 0U)
    {
        Unity.TestIgnores++;
        printf("%s:%d:%s:IGNORE\n",
               Unity.TestFile ? Unity.TestFile : "?",
               FuncLineNum,
               FuncName ? FuncName : "?");
    }
    else
    {
        printf("%s:%d:%s:PASS\n",
               Unity.TestFile ? Unity.TestFile : "?",
               FuncLineNum,
               FuncName ? FuncName : "?");
    }
}

/* -------------------------------------------------------------------------
 * Suite begin / end
 * ------------------------------------------------------------------------- */
int UnityBegin(const char *filename)
{
    memset(&Unity, 0, sizeof(Unity));
    Unity.TestFile = filename;
    return 0;
}

int UnityEnd(void)
{
    UNITY_OUTPUT_FLUSH();
    printf("\n-----------------------\n");
    printf("%u Tests  %u Failures  %u Ignored\n",
           (unsigned)Unity.NumberOfTests,
           (unsigned)Unity.TestFailures,
           (unsigned)Unity.TestIgnores);
    if (Unity.TestFailures == 0U)
    {
        printf("OK\n");
    }
    else
    {
        printf("FAIL\n");
    }
    UNITY_OUTPUT_FLUSH();
    return (int)Unity.TestFailures;
}
