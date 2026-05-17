/*
 * @file     ron_kalman.h
 * @brief    Public API for the Regulon discrete linear Kalman filter.
 * @module   ron_kalman
 * @doc      RON-IS-001
 * @req      RON-FR-600, RON-FR-601, RON-FR-602, RON-FR-603,
 *           RON-FR-604, RON-FR-605, RON-FR-606, RON-FR-607
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Discrete-time linear Kalman filter for the system
 *
 *     x(k+1) = A x(k) + B u(k) + w(k)
 *     z(k)   = H x(k) + v(k)
 *
 * with process-noise covariance Q and measurement-noise covariance R.  All
 * matrices and vectors use caller-owned storage bounded by the compile-time
 * constants RON_KF_MAX_STATES (n), RON_KF_MAX_MEASUREMENTS (m), and
 * RON_KF_MAX_INPUTS (p).  No dynamic allocation, recursion, or VLAs are used.
 *
 * Typical usage:
 *
 *   static ron_kf_t kf;
 *
 *   void init(void) {
 *       ron_kf_config_t cfg = { .n = 2U, .m = 1U, .p = 0U,
 *                               .A = {{1.0F, 1.0F}, {0.0F, 1.0F}},
 *                               .H = {{1.0F, 0.0F}},
 *                               .Q = {{0.01F, 0.0F}, {0.0F, 0.01F}},
 *                               .R = {{1.0F}},
 *                               .P0 = {{10.0F, 0.0F}, {0.0F, 10.0F}} };
 *       (void)ron_kf_init(&kf, &cfg);
 *   }
 *
 *   void isr(ron_float_t z_meas, bool z_ok) {
 *       (void)ron_kf_predict(&kf, NULL);     // p == 0: no control input
 *       ron_float_t z[RON_KF_MAX_MEASUREMENTS] = { z_meas };
 *       (void)ron_kf_update(&kf, z, z_ok);
 *   }
 */

#ifndef RON_KALMAN_H
#define RON_KALMAN_H

#include "ron/ron_pid_types.h" /* ron_float_t, ron_fault_t, shared fault codes */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Filter configuration (RON-FR-600, RON-FR-601, RON-FR-604, RON-FR-606,
 * RON-FR-607)
 *
 * Only the leading n/m/p rows and columns of each array are used; the active
 * dimensions n, m, p are validated against the RON_KF_MAX_* bounds by
 * ron_kf_init().
 * ========================================================================= */

/* Satisfies: RON-FR-601, RON-FR-604, RON-FR-606, RON-FR-607 | Test: RON-TC-KF-002, RON-TC-KF-005, RON-TC-KF-007, RON-TC-KF-008 */
typedef struct {
    uint8_t n; /**< State dimension      (1..RON_KF_MAX_STATES).       */
    uint8_t m; /**< Measurement dimension(1..RON_KF_MAX_MEASUREMENTS). */
    uint8_t p; /**< Input dimension      (0..RON_KF_MAX_INPUTS).       */
    ron_float_t A[RON_KF_MAX_STATES][RON_KF_MAX_STATES];             /**< State transition.   */
    ron_float_t B[RON_KF_MAX_STATES][RON_KF_MAX_INPUTS];             /**< Input matrix.       */
    ron_float_t H[RON_KF_MAX_MEASUREMENTS][RON_KF_MAX_STATES];       /**< Measurement matrix. */
    ron_float_t Q[RON_KF_MAX_STATES][RON_KF_MAX_STATES];             /**< Process-noise cov.  */
    ron_float_t R[RON_KF_MAX_MEASUREMENTS][RON_KF_MAX_MEASUREMENTS]; /**< Meas.-noise cov.    */
    ron_float_t x0[RON_KF_MAX_STATES];                               /**< Initial estimate.   */
    ron_float_t P0[RON_KF_MAX_STATES][RON_KF_MAX_STATES];            /**< Initial covariance. */
    bool use_joseph_form;                                            /**< Joseph-form update. */
    bool steady_state;                                               /**< Use fixed K_inf.    */
    ron_float_t K_inf[RON_KF_MAX_STATES][RON_KF_MAX_MEASUREMENTS];   /**< Steady-state gain.  */
} ron_kf_config_t;

/* =========================================================================
 * Filter state (RON-FR-602, RON-FR-607)
 * ========================================================================= */

/* Satisfies: RON-FR-602, RON-FR-607 | Test: RON-TC-KF-001, RON-TC-KF-003 */
typedef struct {
    ron_float_t x_hat[RON_KF_MAX_STATES];                /**< Current estimate.   */
    ron_float_t P[RON_KF_MAX_STATES][RON_KF_MAX_STATES]; /**< Current covariance. */
    bool is_initialised;                                 /**< Set by ron_kf_init. */
} ron_kf_state_t;

/* Satisfies: RON-FR-600 | Test: RON-TC-KF-001 */
typedef struct {
    ron_kf_config_t cfg;
    ron_kf_state_t state;
} ron_kf_t;

/* =========================================================================
 * API
 * ========================================================================= */

/* Satisfies: RON-FR-600, RON-FR-601, RON-FR-607 | Test: RON-TC-KF-001, RON-TC-KF-002, RON-TC-KF-008 */
ron_fault_t ron_kf_init(ron_kf_t *kf, const ron_kf_config_t *cfg);

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
ron_fault_t ron_kf_reset(ron_kf_t *kf);

/* Satisfies: RON-FR-600, RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003, RON-TC-KF-006 */
ron_fault_t ron_kf_predict(ron_kf_t *kf, const ron_float_t u[RON_KF_MAX_INPUTS]);

/* Satisfies: RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-605, RON-FR-606 | Test: RON-TC-KF-001, RON-TC-KF-004, RON-TC-KF-005, RON-TC-KF-006, RON-TC-KF-007 */
ron_fault_t ron_kf_update(ron_kf_t *kf, const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid);

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
ron_fault_t ron_kf_get_state(const ron_kf_t *kf, ron_float_t x_hat[RON_KF_MAX_STATES]);

#ifdef __cplusplus
}
#endif

#endif /* RON_KALMAN_H */
