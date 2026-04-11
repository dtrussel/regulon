/*
 * @file     ron_platform.h
 * @brief    Platform portability layer for the Regulon library.
 * @module   ron_platform
 * @doc      RON-IS-001
 * @req      RON-PR-010, RON-PR-011, RON-DC-001, RON-DC-002, RON-QR-001
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * This header is the single point of configuration for all platform-dependent
 * behaviour in the C implementation.  It SHALL be the first header included
 * in every .c file.
 *
 * Permitted standard headers: <stdint.h>, <stdbool.h>, <float.h> only.
 * No <math.h> or <stdio.h> here — bare-metal safe.
 */

#ifndef RON_PLATFORM_H
#define RON_PLATFORM_H

#include <stdint.h>  /* uint8_t, uint16_t, uint32_t, int32_t … */
#include <stddef.h>  /* NULL, size_t, ptrdiff_t                  */
#include <stdbool.h> /* bool, true, false                        */
#include <float.h>   /* FLT_MAX, FLT_EPSILON, DBL_MAX …         */

/* =========================================================================
 * Floating-point precision selection
 *
 * Default: 32-bit single-precision float  (RON-PR-010).
 * Override: define RON_USE_DOUBLE=1 at compile time to select 64-bit
 *           double precision              (RON-PR-011).
 * ========================================================================= */

#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)

typedef double ron_float_t;

#define RON_FLOAT_MAX      DBL_MAX
#define RON_FLOAT_MIN      (-DBL_MAX)
#define RON_FLOAT_EPSILON  DBL_EPSILON

/* Literal suffix: no suffix needed for double (plain floating literal is double). */
#define RON_FLOAT_C(x)     (x)

#else /* default: single precision */

typedef float ron_float_t;

#define RON_FLOAT_MAX      FLT_MAX
#define RON_FLOAT_MIN      (-FLT_MAX)
#define RON_FLOAT_EPSILON  FLT_EPSILON

/* Literal suffix: append 'F' for float literals (MISRA C:2023 Rule 7.3). */
#define RON_FLOAT_C(x)     (x##F)

#endif /* RON_USE_DOUBLE */

/* =========================================================================
 * Compile-time assertion
 *
 * C11: use _Static_assert (ISO/IEC 9899:2011 §6.7.10).
 * C99 fallback: typedef trick — triggers a compiler error when condition
 * is false by declaring an array of size -1.
 * ========================================================================= */

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
    #define RON_STATIC_ASSERT(cond, msg) \
        _Static_assert((cond), msg)
#else
    /* C99 fallback */
    #define RON_STATIC_ASSERT(cond, msg) \
        typedef char ron_static_assert_##__LINE__[(cond) ? 1 : -1]
#endif

/* =========================================================================
 * Runtime assertion
 *
 * Default: no-op (production builds — DO NOT add overhead to ISR paths).
 * Override: define RON_ASSERT before including this header, or pass
 *   -DRON_ASSERT(cond)="do{if(!(cond)){__builtin_trap();}}while(0)"
 *   via CMake's RON_ENABLE_ASSERT option.
 *
 * Satisfies: RON-SR-001 defensive pattern.
 * ========================================================================= */

#ifndef RON_ASSERT
    #define RON_ASSERT(cond)  ((void)(cond))
#endif

/* =========================================================================
 * Floating-point classification helpers
 *
 * These macros do NOT use <math.h> — they rely on IEEE 754 arithmetic
 * properties that are mandatory for any conforming C11 implementation
 * targeting hardware with IEEE 754 floating-point.
 *
 * RON_ISNAN:    NaN is the only value where x != x is true.
 * RON_ISINF:    Only ±Inf exceed ±RON_FLOAT_MAX (for the selected type).
 * RON_ISFINITE: A value is finite iff it is neither NaN nor infinite.
 *
 * Satisfies: RON-SR-020, RON-PR-012.
 * ========================================================================= */

/** @brief Non-zero iff x is a NaN (Not-a-Number). */
/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
#define RON_ISNAN(x)     ((x) != (x))

/** @brief Non-zero iff x is positive or negative infinity. */
/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
#define RON_ISINF(x)     (((x) > RON_FLOAT_MAX) || ((x) < RON_FLOAT_MIN))

/** @brief Non-zero iff x is a normal, subnormal, or zero value (not NaN/Inf). */
/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
#define RON_ISFINITE(x)  (!RON_ISNAN(x) && !RON_ISINF(x))

/* =========================================================================
 * Inline utility functions
 *
 * Declared static inline so they appear in every translation unit that
 * includes this header without link-time symbol duplication.
 * ========================================================================= */

/**
 * @brief Saturating clamp — clamps x to the closed interval [lo, hi].
 *
 * @pre  lo <= hi (guaranteed by the configuration validation layer).
 *
 * Satisfies: RON-FR-020 | Test: RON-TC-PID-015
 */
/* Satisfies: RON-FR-020 | Test: RON-TC-PID-015 */
static inline ron_float_t ron_clamp(ron_float_t x,
                                    ron_float_t lo,
                                    ron_float_t hi)
{
    ron_float_t result = x;
    if (result < lo)
    {
        result = lo;
    }
    if (result > hi)
    {
        result = hi;
    }
    return result;
}

/**
 * @brief Absolute value — returns |x| without depending on <math.h>.
 *
 * Satisfies: RON-SR-021 | Test: RON-TC-SAFE-012
 */
/* Satisfies: RON-SR-021 | Test: RON-TC-SAFE-012 */
static inline ron_float_t ron_fabs(ron_float_t x)
{
    return (x < RON_FLOAT_C(0.0)) ? (-x) : x;
}

/* =========================================================================
 * Compiler hint macros
 * ========================================================================= */

/**
 * @brief Marks a function as non-returning (for fault handlers).
 *
 * Used on the platform fault handler to suppress "function does not return"
 * warnings in callers.
 */
#if defined(__GNUC__) || defined(__clang__)
    #define RON_NORETURN  __attribute__((noreturn))
#elif defined(_MSC_VER)
    #define RON_NORETURN  __declspec(noreturn)
#else
    #define RON_NORETURN  /* unsupported — omit attribute */
#endif

/** @brief Inline suggestion for performance-critical utilities. */
#define RON_INLINE  static inline

/* =========================================================================
 * Version information
 * ========================================================================= */

#define RON_VERSION_MAJOR  1U
#define RON_VERSION_MINOR  0U
#define RON_VERSION_PATCH  0U

/* =========================================================================
 * Compile-time array dimension constants
 *
 * These bound all fixed-size arrays in the library.  Define them before
 * including this header (or via a user ron_config.h) to override defaults.
 *
 * Satisfies: RON-SR-003 (no dynamic allocation), RON-PR-022.
 * ========================================================================= */

/** Maximum moving-average window length (RON-FR-116). */
#ifndef RON_MA_MAX_WINDOW
#  define RON_MA_MAX_WINDOW          64U
#endif

/** Maximum number of cascaded biquad sections (RON-FR-121). */
#ifndef RON_BIQUAD_MAX_SECTIONS
#  define RON_BIQUAD_MAX_SECTIONS     8U
#endif

/** Maximum number of gain-scheduling breakpoints (RON-FR-301). */
#ifndef RON_GS_MAX_BREAKPOINTS
#  define RON_GS_MAX_BREAKPOINTS     16U
#endif

/** Maximum Kalman state dimension n (RON-FR-601). */
#ifndef RON_KF_MAX_STATES
#  define RON_KF_MAX_STATES           8U
#endif

/** Maximum Kalman measurement dimension m (RON-FR-601). */
#ifndef RON_KF_MAX_MEASUREMENTS
#  define RON_KF_MAX_MEASUREMENTS     4U
#endif

/** Maximum Kalman input dimension p (RON-FR-601). */
#ifndef RON_KF_MAX_INPUTS
#  define RON_KF_MAX_INPUTS           4U
#endif

/** Maximum state-space state dimension (RON-FR-700). */
#ifndef RON_SS_MAX_STATES
#  define RON_SS_MAX_STATES           8U
#endif

/** Maximum state-space output dimension (RON-FR-700). */
#ifndef RON_SS_MAX_OUTPUTS
#  define RON_SS_MAX_OUTPUTS          4U
#endif

/** Maximum state-space input dimension (RON-FR-700). */
#ifndef RON_SS_MAX_INPUTS
#  define RON_SS_MAX_INPUTS           4U
#endif

/** Minimum relay oscillation cycles before Ku/Tu estimate is valid (RON-FR-802). */
#ifndef RON_AT_MIN_CYCLES
#  define RON_AT_MIN_CYCLES           3U
#endif

/** Oscillation detection window length (RON-FR-901). */
#ifndef RON_HEALTH_OSC_WINDOW
#  define RON_HEALTH_OSC_WINDOW      32U
#endif

/* =========================================================================
 * Compile-time sanity checks on dimension constants
 * ========================================================================= */

RON_STATIC_ASSERT(RON_MA_MAX_WINDOW >= 2U,
    "RON_MA_MAX_WINDOW must be at least 2");
RON_STATIC_ASSERT(RON_BIQUAD_MAX_SECTIONS >= 1U,
    "RON_BIQUAD_MAX_SECTIONS must be at least 1");
RON_STATIC_ASSERT(RON_GS_MAX_BREAKPOINTS >= 2U,
    "RON_GS_MAX_BREAKPOINTS must be at least 2");
RON_STATIC_ASSERT(RON_KF_MAX_STATES >= 1U,
    "RON_KF_MAX_STATES must be at least 1");
RON_STATIC_ASSERT(RON_AT_MIN_CYCLES >= 1U,
    "RON_AT_MIN_CYCLES must be at least 1");
RON_STATIC_ASSERT(RON_HEALTH_OSC_WINDOW >= 4U,
    "RON_HEALTH_OSC_WINDOW must be at least 4");

#endif /* RON_PLATFORM_H */
