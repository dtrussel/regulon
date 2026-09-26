/*
 * @file     ron_util.c
 * @brief    Internal scalar helpers shared by every module (no allocation).
 * @module   ron_util
 * @doc      RON-IS-001
 * @req      RON-SR-020, RON-FR-022, RON-FR-026, RON-FR-500, RON-FR-603
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron_util_internal.h"

/* Newton iterations for ron_util_sqrt: enough to converge from the initial
 * guess x0 = max(value, 1) for any finite single- or double-precision input
 * the modules pass (covariance diagonals, a*d products, v/j ratios). */
#define RON_UTIL_SQRT_STEPS (30U)

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
bool ron_util_isfinite(ron_float_t value)
{
    return RON_ISFINITE(value);
}

/* Satisfies: RON-FR-022, RON-FR-026 | Test: RON-TC-PID-017, RON-TC-PID-019 */
ron_float_t ron_util_rate_limit(ron_float_t u_sat, ron_float_t u_prev, ron_float_t du_max,
                                ron_float_t dt, bool *limited)
{
    ron_float_t limited_value = u_sat;

    if (du_max <= RON_FLOAT_C(0.0)) {
        *limited = false;
    } else {
        ron_float_t delta_max = du_max * dt;
        ron_float_t delta     = u_sat - u_prev;

        if (delta > delta_max) {
            *limited      = true;
            limited_value = u_prev + delta_max;
        } else if (delta < (-delta_max)) {
            *limited      = true;
            limited_value = u_prev - delta_max;
        } else {
            *limited = false;
        }
    }

    return limited_value;
}

/* Satisfies: RON-FR-500, RON-FR-603 | Test: RON-TC-TRAJ-001, RON-TC-KF-004 */
ron_float_t ron_util_sqrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    for (step = 0U; step < RON_UTIL_SQRT_STEPS; step++) {
        x = RON_FLOAT_C(0.5) * (x + (value / x));
    }

    return x;
}

/* Satisfies: RON-FR-503 | Test: RON-TC-TRAJ-004 */
ron_float_t ron_util_sign_nonzero(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? RON_FLOAT_C(-1.0) : RON_FLOAT_C(1.0);
}
