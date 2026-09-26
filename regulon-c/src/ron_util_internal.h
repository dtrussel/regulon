/*
 * @file     ron_util_internal.h
 * @brief    Internal scalar helpers shared by every module (no allocation).
 * @module   ron_util
 * @doc      RON-IS-001
 * @req      RON-SR-020, RON-FR-022, RON-FR-026, RON-FR-500, RON-FR-603
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * One implementation of the scalar operations the modules previously each
 * carried a private copy of: the finite check, the output rate limiter, a
 * libm-free square root and a sign helper. Part of the mandatory baseline, so
 * every module can rely on it in any RON_ENABLE_* subset.
 *
 * This is a PRIVATE header. It lives under src/ and is never installed as
 * part of the public API.
 */

#ifndef RON_UTIL_INTERNAL_H
#define RON_UTIL_INTERNAL_H

#include "ron/ron_platform.h"

/** Pi in the configured precision (no <math.h> dependency). */
#define RON_UTIL_PI RON_FLOAT_C(3.14159265358979323846)

/** Two pi in the configured precision. */
#define RON_UTIL_TWO_PI RON_FLOAT_C(6.28318530717958647692)

/**
 * @brief True iff value is neither NaN nor +/-Inf.
 *
 * Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011
 */
/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
bool ron_util_isfinite(ron_float_t value);

/**
 * @brief Limit the change of an output to du_max per second.
 *
 * Returns u_sat moved at most du_max * dt away from u_prev. A du_max <= 0
 * disables the limit (RON-FR-026). *limited reports whether it was active.
 *
 * Satisfies: RON-FR-022, RON-FR-026 | Test: RON-TC-PID-017, RON-TC-PID-019
 */
/* Satisfies: RON-FR-022, RON-FR-026 | Test: RON-TC-PID-017, RON-TC-PID-019 */
ron_float_t ron_util_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
                                ron_float_t dt, bool *limited);

/**
 * @brief Square root by a fixed number of Newton iterations (bounded WCET).
 *
 * Precondition: value >= 0 and finite.
 *
 * Satisfies: RON-FR-500, RON-FR-603 | Test: RON-TC-TRAJ-001, RON-TC-KF-004
 */
/* Satisfies: RON-FR-500, RON-FR-603 | Test: RON-TC-TRAJ-001, RON-TC-KF-004 */
ron_float_t ron_util_sqrt(ron_float_t value);

/**
 * @brief -1 for negative values, +1 otherwise (never 0).
 *
 * Satisfies: RON-FR-503 | Test: RON-TC-TRAJ-004
 */
/* Satisfies: RON-FR-503 | Test: RON-TC-TRAJ-004 */
ron_float_t ron_util_sign_nonzero(ron_float_t value);

#endif /* RON_UTIL_INTERNAL_H */
