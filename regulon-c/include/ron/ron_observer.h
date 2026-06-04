/*
 * @file     ron_observer.h
 * @brief    Public API for the Regulon discrete-time Luenberger observer.
 * @module   ron_observer
 * @doc      RON-IS-001
 * @req      RON-FR-720, RON-FR-721, RON-FR-722, RON-FR-723
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Discrete-time Luenberger state observer for the plant
 *
 *     x(k+1) = A x(k) + B u(k)
 *     y(k)   = C x(k)
 *
 * estimating the full state via the recursion
 *
 *     x_hat(k+1) = A x_hat(k) + B u(k) + L (y(k) - C x_hat(k)).
 *
 * All matrices and vectors use caller-owned storage bounded by the
 * compile-time constants RON_SS_MAX_STATES (n), RON_SS_MAX_OUTPUTS (m), and
 * RON_SS_MAX_INPUTS (p).  No dynamic allocation, recursion, or VLAs are used.
 *
 * Typical usage:
 *
 *   static ron_obs_t obs;
 *
 *   void init(void) {
 *       ron_obs_config_t cfg = { .n = 2U, .m = 1U, .p = 1U,
 *                                .A = {{1.0F, 1.0F}, {0.0F, 1.0F}},
 *                                .B = {{0.0F}, {1.0F}},
 *                                .C = {{1.0F, 0.0F}},
 *                                .L = {{0.5F}, {0.1F}},
 *                                .x0 = {0.0F, 0.0F} };
 *       (void)ron_obs_init(&obs, &cfg);
 *   }
 *
 *   void isr(ron_float_t y_meas, ron_float_t u_applied) {
 *       ron_float_t y[RON_SS_MAX_OUTPUTS] = { y_meas };
 *       ron_float_t u[RON_SS_MAX_INPUTS]  = { u_applied };
 *       (void)ron_obs_step(&obs, y, u);
 *   }
 */

#ifndef RON_OBSERVER_H
#define RON_OBSERVER_H

#include "ron/ron_pid_types.h" /* ron_float_t, ron_fault_t, shared fault codes */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Observer configuration (RON-FR-721, RON-FR-723)
 *
 * Only the leading n/m/p rows and columns of each array are used; the active
 * dimensions n, m, p are validated against the RON_SS_MAX_* bounds by
 * ron_obs_init().
 * ========================================================================= */

/* Satisfies: RON-FR-721, RON-FR-723 | Test: RON-TC-SS-007, RON-TC-SS-009 */
typedef struct {
    uint8_t n; /**< State dimension       (1..RON_SS_MAX_STATES).  */
    uint8_t m; /**< Output dimension      (1..RON_SS_MAX_OUTPUTS). */
    uint8_t p; /**< Input dimension       (0..RON_SS_MAX_INPUTS).  */
    ron_float_t A[RON_SS_MAX_STATES][RON_SS_MAX_STATES];  /**< State transition.    */
    ron_float_t B[RON_SS_MAX_STATES][RON_SS_MAX_INPUTS];  /**< Input matrix.        */
    ron_float_t C[RON_SS_MAX_OUTPUTS][RON_SS_MAX_STATES]; /**< Output matrix.       */
    ron_float_t L[RON_SS_MAX_STATES][RON_SS_MAX_OUTPUTS]; /**< Observer gain.       */
    ron_float_t x0[RON_SS_MAX_STATES];                    /**< Initial estimate.    */
} ron_obs_config_t;

/* =========================================================================
 * Observer state (RON-FR-720, RON-FR-722)
 * ========================================================================= */

/* Satisfies: RON-FR-720, RON-FR-722 | Test: RON-TC-SS-006, RON-TC-SS-008 */
typedef struct {
    ron_float_t x_hat[RON_SS_MAX_STATES]; /**< Current state estimate. */
    bool is_initialised;                  /**< Set by ron_obs_init.    */
} ron_obs_state_t;

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006 */
typedef struct {
    ron_obs_config_t cfg;
    ron_obs_state_t state;
} ron_obs_t;

/* =========================================================================
 * API
 * ========================================================================= */

/* Satisfies: RON-FR-721, RON-FR-723 | Test: RON-TC-SS-007, RON-TC-SS-009 */
ron_fault_t ron_obs_init(ron_obs_t *obs, const ron_obs_config_t *cfg);

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006 */
ron_fault_t ron_obs_reset(ron_obs_t *obs);

/* Satisfies: RON-FR-720 | Test: RON-TC-SS-006, RON-TC-SS-007 */
ron_fault_t ron_obs_step(ron_obs_t *obs, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                         const ron_float_t u[RON_SS_MAX_INPUTS]);

/* Satisfies: RON-FR-722 | Test: RON-TC-SS-008 */
ron_fault_t ron_obs_get_state(const ron_obs_t *obs, ron_float_t x_hat[RON_SS_MAX_STATES]);

#ifdef __cplusplus
}
#endif

#endif /* RON_OBSERVER_H */
