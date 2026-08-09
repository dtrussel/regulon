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

/**
 * @brief Initialise a discrete-time linear Kalman filter.
 *
 * Copies the configuration into the instance and seeds the estimate and
 * covariance from @c x0 and @c P0. The configuration need not outlive this
 * call.
 *
 * @param[out] kf   Filter instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration. Dimensions @c n, @c m and @c p must be
 *                  within their ::RON_KF_MAX_STATES,
 *                  ::RON_KF_MAX_MEASUREMENTS and ::RON_KF_MAX_INPUTS bounds,
 *                  every active matrix entry must be finite, and @c R must be
 *                  positive-definite. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter ready to use.
 * @retval RON_FAULT_NULL_POINTER   @p kf or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A dimension was out of range or a matrix
 *                                  entry was not finite.
 */
/* Satisfies: RON-FR-600, RON-FR-601, RON-FR-607 | Test: RON-TC-KF-001, RON-TC-KF-002, RON-TC-KF-008 */
ron_fault_t ron_kf_init(ron_kf_t *kf, const ron_kf_config_t *cfg);

/**
 * @brief Return the filter to its post-initialisation state.
 *
 * Restores the estimate and covariance to the configured @c x0 and @c P0 and
 * clears any latched fault, keeping the model matrices.
 *
 * @param[in,out] kf  Initialised filter instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter reset.
 * @retval RON_FAULT_NULL_POINTER   @p kf was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
ron_fault_t ron_kf_reset(ron_kf_t *kf);

/**
 * @brief Propagate the estimate and covariance forward by one step.
 *
 * Applies the time update
 * @c x = A*x + B*u and @c P = A*P*A' + Q.
 *
 * Call this once per sample period, before ron_kf_update(). A predict
 * without a matching update is legitimate and is how the filter coasts
 * through a sample with no measurement.
 *
 * @param[in,out] kf  Initialised filter instance. Must not be NULL.
 * @param[in]     u   Control input vector, @c p entries. May be NULL when the
 *                    filter was configured with @c p == 0, in which case the
 *                    input term is skipped. All entries must be finite.
 *
 * @retval RON_FAULT_NONE           Estimate propagated.
 * @retval RON_FAULT_NULL_POINTER   @p kf was NULL, or @p u was NULL while
 *                                  @c p > 0.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 * @retval RON_FAULT_INPUT_NAN      An entry of @p u was not finite.
 * @retval RON_FAULT_OUTPUT_NAN     The propagated estimate or covariance was
 *                                  not finite; the fault latches.
 */
/* Satisfies: RON-FR-600, RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003, RON-TC-KF-006 */
ron_fault_t ron_kf_predict(ron_kf_t *kf, const ron_float_t u[RON_KF_MAX_INPUTS]);

/**
 * @brief Correct the estimate with a measurement.
 *
 * Applies the measurement update, forming the innovation @c z - H*x and the
 * Kalman gain from the innovation covariance. The gain is obtained by
 * Cholesky solve rather than explicit inversion, which is better behaved when
 * the innovation covariance is poorly conditioned.
 *
 * Passing @p z_valid as @c false advances the filter without correcting it,
 * which is the intended way to handle a dropped or rejected sample: the
 * covariance keeps growing rather than the estimate being corrupted.
 *
 * @param[in,out] kf       Initialised filter instance. Must not be NULL.
 * @param[in]     z        Measurement vector, @c m entries. All must be
 *                         finite. Ignored when @p z_valid is @c false.
 * @param[in]     z_valid  Whether @p z holds a usable measurement.
 *
 * @retval RON_FAULT_NONE           Estimate corrected, or skipped because
 *                                  @p z_valid was @c false.
 * @retval RON_FAULT_NULL_POINTER   @p kf, or @p z while @p z_valid was set,
 *                                  was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 * @retval RON_FAULT_INPUT_NAN      An entry of @p z was not finite.
 * @retval RON_FAULT_OUTPUT_NAN     The innovation covariance was not
 *                                  positive-definite or the corrected
 *                                  estimate was not finite; the fault
 *                                  latches.
 */
/* Satisfies: RON-FR-602, RON-FR-603, RON-FR-604, RON-FR-605, RON-FR-606 | Test: RON-TC-KF-001, RON-TC-KF-004, RON-TC-KF-005, RON-TC-KF-006, RON-TC-KF-007 */
ron_fault_t ron_kf_update(ron_kf_t *kf, const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid);

/**
 * @brief Copy out the current state estimate.
 *
 * @param[in]  kf     Initialised filter instance. Must not be NULL.
 * @param[out] x_hat  Receives the estimate; the leading @c n entries are
 *                    written. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Estimate copied.
 * @retval RON_FAULT_NULL_POINTER   @p kf or @p x_hat was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-602 | Test: RON-TC-KF-001, RON-TC-KF-003 */
ron_fault_t ron_kf_get_state(const ron_kf_t *kf, ron_float_t x_hat[RON_KF_MAX_STATES]);

#ifdef __cplusplus
}
#endif

#endif /* RON_KALMAN_H */
