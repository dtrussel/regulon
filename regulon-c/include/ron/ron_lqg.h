/*
 * @file     ron_lqg.h
 * @brief    Public API for the Regulon discrete-time MIMO LQG controller.
 * @module   ron_lqg
 * @doc      RON-IS-001
 * @req      RON-FR-750, RON-FR-751, RON-FR-752, RON-FR-753, RON-FR-754,
 *           RON-FR-755, RON-FR-756, RON-FR-757, RON-FR-758, RON-FR-759
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Discrete-time MIMO Linear Quadratic Gaussian controller
 *
 *     x(k+1) = A x(k) + B u(k) + w(k)     (state equation)
 *     z(k)   = H x(k) + v(k)               (observation equation)
 *     u(k)   = -K x_hat(k) + Kr r(k)       (LQR control law)
 *
 * where w ~ N(0, Q_noise), v ~ N(0, R_noise), and the LQR gain K minimises
 * sum_k (x_k^T Q_cost x_k + u_k^T R_cost u_k).
 *
 * The LQG combines an optimal Kalman filter state estimator with an optimal
 * LQR state-feedback controller.  By the separation principle (RON-FR-752),
 * the two components are designed independently:
 *   - The Kalman gain is determined by Q_noise, R_noise, A, H.
 *   - The LQR gain K is determined by Q_cost, R_cost, A, B.
 *
 * Both gains are solved via DARE at init time (RON-FR-756) and are not
 * recomputed per step.  Pre-computed gains may be supplied directly to
 * bypass the DARE solver (RON_LQG_GAIN_PRECOMPUTED).
 *
 * All storage resides in the caller-owned ron_lqg_t instance; no dynamic
 * allocation, recursion, or VLAs are used (RON-FR-759).
 *
 * Typical per-step call sequence:
 *
 *   static ron_lqg_t lqg;
 *
 *   void init(void) {
 *       ron_lqg_config_t cfg = {
 *           .n = 2U, .m = 1U, .p = 1U,
 *           .gain_mode = RON_LQG_GAIN_DARE,
 *           .A = {{1,1},{0,1}}, .B = {{0},{1}}, .H = {{1,0}},
 *           .Q_noise = {{0.01F,0},{0,0.01F}}, .R_noise = {{1.0F}},
 *           .Q_cost  = {{1.0F, 0},{0, 1.0F}}, .R_cost  = {{1.0F}},
 *           .u_min = {-10.0F}, .u_max = {10.0F},
 *       };
 *       (void)ron_lqg_init(&lqg, &cfg);
 *   }
 *
 *   void isr(ron_float_t z_meas, bool z_ok) {
 *       ron_float_t u[RON_LQR_MAX_INPUTS];
 *       ron_status_t status;
 *       ron_float_t r[RON_LQR_MAX_INPUTS]  = { setpoint };
 *       ron_float_t z[RON_KF_MAX_MEASUREMENTS] = { z_meas };
 *
 *       (void)ron_lqg_predict(&lqg, u);    // Kalman prediction step
 *       (void)ron_lqg_update(&lqg, z, z_ok); // Kalman correction
 *       (void)ron_lqg_step(&lqg, r, 0.01F, u, &status);
 *       actuator_set(u[0]);
 *   }
 */

#ifndef RON_LQG_H
#define RON_LQG_H

#include "ron/ron_lqr.h" /* for ron_lqr_* types and RON_LQR_MAX_* constants */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Gain computation mode (RON-FR-756)
 * ========================================================================= */

/* Satisfies: RON-FR-756 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
typedef enum {
    RON_LQG_GAIN_PRECOMPUTED = 0, /**< K and K_f_inf supplied; DARE skipped. */
    RON_LQG_GAIN_DARE        = 1  /**< Both gains computed via DARE at init. */
} ron_lqg_gain_mode_t;

/* =========================================================================
 * Controller configuration (RON-FR-750 .. RON-FR-759)
 *
 * Only the leading n/m/p rows and columns of each 2-D array are used.
 * ========================================================================= */

/* Satisfies: RON-FR-750..RON-FR-759 | Test: RON-TC-LQG-001..RON-TC-LQG-010 */
typedef struct {
    uint8_t            n;          /**< State dim  (1..RON_LQR_MAX_STATES).       */
    uint8_t            m;          /**< Input dim  (1..RON_LQR_MAX_INPUTS).       */
    uint8_t            p;          /**< Meas  dim  (1..RON_KF_MAX_MEASUREMENTS).  */
    ron_lqg_gain_mode_t gain_mode; /**< Pre-computed or DARE.                     */

    /* System matrices — shared by Kalman predictor and LQR law (RON-FR-751). */
    ron_float_t A[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];               /**< State transition.   */
    ron_float_t B[RON_LQR_MAX_STATES][RON_LQR_MAX_INPUTS];                /**< Input matrix.       */
    ron_float_t H[RON_KF_MAX_MEASUREMENTS][RON_LQR_MAX_STATES];           /**< Observation matrix. */

    /* Kalman noise covariances (RON-FR-751). */
    ron_float_t Q_noise[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];              /**< Process noise cov.  */
    ron_float_t R_noise[RON_KF_MAX_MEASUREMENTS][RON_KF_MAX_MEASUREMENTS];    /**< Meas noise cov.     */
    ron_float_t x0[RON_LQR_MAX_STATES];                                        /**< Initial estimate.   */
    ron_float_t P0[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];                  /**< Initial covariance. */
    bool        use_joseph_form;     /**< Joseph-form covariance update.         */
    bool        use_kf_steady_state; /**< Use pre-computed K_f_inf (no adapt).   */
    ron_float_t K_f_inf[RON_LQR_MAX_STATES][RON_KF_MAX_MEASUREMENTS]; /**< SS KF gain.  */

    /* LQR cost matrices (RON-FR-751). */
    ron_float_t Q_cost[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES]; /**< State cost (PSD). */
    ron_float_t R_cost[RON_LQR_MAX_INPUTS][RON_LQR_MAX_INPUTS];  /**< Input cost (PD).  */
    uint16_t    dare_max_iter; /**< DARE iteration limit (0 → default 200).    */
    ron_float_t dare_tol;      /**< DARE convergence tolerance.               */

    /* Pre-computed LQR gain override (PRECOMPUTED mode). */
    ron_float_t K[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]; /**< Pre-comp feedback gain. */
    ron_float_t Kr[RON_LQR_MAX_INPUTS];                      /**< Reference pre-gain.     */

    /* Per-input output constraints (RON-FR-757). */
    ron_float_t u_min[RON_LQR_MAX_INPUTS];  /**< Per-input sat lower bound.  */
    ron_float_t u_max[RON_LQR_MAX_INPUTS];  /**< Per-input sat upper bound.  */
    ron_float_t du_max[RON_LQR_MAX_INPUTS]; /**< Per-input rate limit (≤ 0 disables). */
} ron_lqg_config_t;

/* =========================================================================
 * Controller instance (RON-FR-759)
 * ========================================================================= */

/* Satisfies: RON-FR-759 | Test: RON-TC-LQG-001, RON-TC-LQG-010 */
typedef struct {
    ron_lqg_config_t cfg;
    ron_kf_t         kalman;   /**< Embedded Kalman filter (optimal estimator). */
    ron_float_t      K_solved[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]; /**< LQR gain in use.  */
    ron_float_t      P_lqr[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES];   /**< LQR DARE solution.*/
    ron_float_t      u_prev[RON_LQR_MAX_INPUTS]; /**< Previous output (rate limiting).        */
    ron_fault_t      faults;                      /**< Latched fault register.                 */
    bool             is_initialised;              /**< Set by ron_lqg_init.                    */
} ron_lqg_t;

/* =========================================================================
 * API
 * ========================================================================= */

/* Satisfies: RON-FR-750, RON-FR-756 | Test: RON-TC-LQG-001, RON-TC-LQG-006 */
ron_fault_t ron_lqg_init(ron_lqg_t *lqg, const ron_lqg_config_t *cfg);

/* Satisfies: RON-FR-757 | Test: RON-TC-LQG-009 */
ron_fault_t ron_lqg_reset(ron_lqg_t *lqg);

/* Advance the embedded Kalman filter prediction step (RON-FR-753). */
/* Satisfies: RON-FR-753 | Test: RON-TC-LQG-002 */
ron_fault_t ron_lqg_predict(ron_lqg_t *lqg,
                             const ron_float_t u[RON_LQR_MAX_INPUTS]);

/* Apply Kalman measurement correction; silently skips if z_valid == false (RON-FR-754). */
/* Satisfies: RON-FR-754 | Test: RON-TC-LQG-003, RON-TC-LQG-004 */
ron_fault_t ron_lqg_update(ron_lqg_t *lqg,
                            const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                            bool z_valid);

/* Compute MIMO control output u = -K x_hat + Kr r using Kalman estimate. */
/* Satisfies: RON-FR-755, RON-FR-757 | Test: RON-TC-LQG-005, RON-TC-LQG-008 */
ron_fault_t ron_lqg_step(ron_lqg_t *lqg,
                          const ron_float_t r[RON_LQR_MAX_INPUTS],
                          ron_float_t dt,
                          ron_float_t u[RON_LQR_MAX_INPUTS],
                          ron_status_t *status);

/* Read the current Kalman state estimate (RON-FR-758). */
/* Satisfies: RON-FR-758 | Test: RON-TC-LQG-005, RON-TC-LQG-007 */
ron_fault_t ron_lqg_get_state(const ron_lqg_t *lqg,
                               ron_float_t x_hat[RON_LQR_MAX_STATES]);

#ifdef __cplusplus
}
#endif

#endif /* RON_LQG_H */
