/*
 * @file     ron_lqr.h
 * @brief    Public API for the Regulon discrete-time MIMO LQR controller.
 * @module   ron_lqr
 * @doc      RON-IS-001
 * @req      RON-FR-730, RON-FR-731, RON-FR-732, RON-FR-733, RON-FR-734,
 *           RON-FR-735, RON-FR-736, RON-FR-737, RON-FR-738, RON-FR-739
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Discrete-time MIMO Linear Quadratic Regulator
 *
 *     u(k) = -K x_hat(k) + Kr r(k)
 *
 * where K is an (m x n) optimal state-feedback gain matrix computed by
 * solving the Discrete Algebraic Riccati Equation (DARE) at init time, or
 * supplied pre-computed by the caller.  r and u are m-vectors; x_hat is
 * the n-dimensional state estimate (RON-FR-730).
 *
 * Two gain modes are supported (RON-FR-732):
 *   RON_LQR_GAIN_DARE        — K computed from Q_cost and R_cost via the
 *                              iterative value recursion (RON-FR-733).
 *   RON_LQR_GAIN_PRECOMPUTED — K supplied directly by the caller; DARE
 *                              is skipped.
 *
 * Three state-estimate sources are supported (RON-FR-734):
 *   RON_LQR_SOURCE_EXTERNAL   — x_hat from a caller-owned vector.
 *   RON_LQR_SOURCE_LUENBERGER — x_hat from an embedded ron_obs_t.
 *   RON_LQR_SOURCE_KALMAN     — x_hat from an embedded ron_kf_t.
 *
 * Optional per-input integral augmentation is available for steady-state
 * output regulation (RON-FR-735).  Per-input output saturation, rate
 * limiting, and fault detection apply as for the PID module (RON-FR-736).
 * All matrix dimensions are bounded by compile-time constants
 * RON_LQR_MAX_STATES and RON_LQR_MAX_INPUTS (RON-FR-737).
 *
 * Typical usage (pre-computed gain, external state source):
 *
 *   static ron_lqr_t lqr;
 *
 *   void init(void) {
 *       ron_lqr_config_t cfg = {
 *           .n = 2U, .m = 1U,
 *           .source    = RON_LQR_SOURCE_EXTERNAL,
 *           .gain_mode = RON_LQR_GAIN_PRECOMPUTED,
 *           .x_ext     = state_vector,
 *           .K  = {{2.0F, 1.0F}},
 *           .Kr = {1.0F},
 *           .u_min = {-10.0F}, .u_max = {10.0F},
 *           .du_max = {50.0F},
 *       };
 *       (void)ron_lqr_init(&lqr, &cfg);
 *   }
 *
 *   void isr(void) {
 *       ron_float_t r[RON_LQR_MAX_INPUTS] = { setpoint };
 *       ron_float_t u[RON_LQR_MAX_INPUTS];
 *       ron_status_t status;
 *       (void)ron_lqr_step(&lqr, r, 0.01F, u, &status);
 *       actuator_set(u[0]);
 *   }
 */

#ifndef RON_LQR_H
#define RON_LQR_H

#include "ron/ron_kalman.h"   /* embedded Kalman source (RON-FR-734)     */
#include "ron/ron_observer.h" /* embedded Luenberger source (RON-FR-734) */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * State-estimate source selection (RON-FR-734)
 * ========================================================================= */

/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-002, RON-TC-LQR-008, RON-TC-LQR-009 */
typedef enum {
    RON_LQR_SOURCE_EXTERNAL   = 0, /**< x_hat from cfg.x_ext.             */
    RON_LQR_SOURCE_LUENBERGER = 1, /**< x_hat from the embedded observer. */
    RON_LQR_SOURCE_KALMAN     = 2  /**< x_hat from the embedded Kalman.   */
} ron_lqr_source_t;

/* =========================================================================
 * Gain computation mode (RON-FR-732)
 * ========================================================================= */

/* Satisfies: RON-FR-732 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
typedef enum {
    RON_LQR_GAIN_PRECOMPUTED = 0, /**< K supplied by caller; DARE skipped.  */
    RON_LQR_GAIN_DARE        = 1  /**< K computed from Q_cost/R_cost (DARE).*/
} ron_lqr_gain_mode_t;

/* =========================================================================
 * Controller configuration (RON-FR-730 .. RON-FR-737)
 *
 * Only the leading n rows/columns and m rows of each 2-D array are used.
 * obs_cfg is consumed only when source == RON_LQR_SOURCE_LUENBERGER;
 * kf_cfg  is consumed only when source == RON_LQR_SOURCE_KALMAN.
 * A and B are required in DARE mode or when an embedded estimator is used.
 * ========================================================================= */

/* Satisfies: RON-FR-730..RON-FR-737 | Test: RON-TC-LQR-001..RON-TC-LQR-009 */
typedef struct {
    uint8_t n;                     /**< State dim (1..RON_LQR_MAX_STATES). */
    uint8_t m;                     /**< Input dim (1..RON_LQR_MAX_INPUTS). */
    ron_lqr_source_t source;       /**< State estimate source.             */
    ron_lqr_gain_mode_t gain_mode; /**< Pre-computed gain or DARE.         */
    const ron_float_t *x_ext;      /**< External state (EXTERNAL source).  */

    /* System matrices — used by DARE solver and embedded estimators. */
    ron_float_t A[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES]; /**< State transition. */
    ron_float_t B[RON_LQR_MAX_STATES][RON_LQR_MAX_INPUTS]; /**< Input matrix.     */

    /* DARE cost matrices (RON-FR-731) — consumed only in DARE mode. */
    ron_float_t Q_cost[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES]; /**< State cost (PSD). */
    ron_float_t R_cost[RON_LQR_MAX_INPUTS][RON_LQR_MAX_INPUTS]; /**< Input cost (PD).  */
    uint16_t dare_max_iter; /**< Iteration limit (0 → default 200).    */
    ron_float_t dare_tol;   /**< Convergence tolerance.                */

    /* Feedback and reference gains — initial value or pre-computed K. */
    ron_float_t K[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]; /**< Feedback gain.    */
    ron_float_t Kr[RON_LQR_MAX_INPUTS];                    /**< Reference pre-gain.*/

    /* Integral augmentation (RON-FR-735). */
    bool use_integral;
    ron_float_t Ki_aug[RON_LQR_MAX_INPUTS];                    /**< Integral gains.  */
    ron_float_t C_out[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]; /**< Output rows.     */
    ron_float_t i_min[RON_LQR_MAX_INPUTS]; /**< Per-input integral lower clamp. */
    ron_float_t i_max[RON_LQR_MAX_INPUTS]; /**< Per-input integral upper clamp. */

    /* Output constraints (RON-FR-736). */
    ron_float_t u_min[RON_LQR_MAX_INPUTS];  /**< Per-input sat lower bound.  */
    ron_float_t u_max[RON_LQR_MAX_INPUTS];  /**< Per-input sat upper bound.  */
    ron_float_t du_max[RON_LQR_MAX_INPUTS]; /**< Per-input rate limit (≤ 0 disables). */

    /* Embedded estimator configs. */
    ron_obs_config_t obs_cfg; /**< Observer config (LUENBERGER source). */
    ron_kf_config_t kf_cfg;   /**< Kalman config  (KALMAN source).       */
} ron_lqr_config_t;

/* =========================================================================
 * Controller state (RON-FR-737, RON-FR-739)
 * ========================================================================= */

/* Satisfies: RON-FR-737, RON-FR-739 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
typedef struct {
    ron_float_t K_solved[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES]; /**< Active gain.    */
    ron_float_t P_solved[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES]; /**< DARE solution P.*/
    ron_float_t integral[RON_LQR_MAX_INPUTS];                     /**< Integral accumulator.      */
    ron_float_t u_prev[RON_LQR_MAX_INPUTS];                       /**< Previous output (rate lim).*/
    ron_fault_t faults;                                           /**< Latched fault register.    */
    bool dare_converged;                                          /**< Set when DARE converged.   */
    bool is_initialised;                                          /**< Set by ron_lqr_init.       */
} ron_lqr_state_t;

/* Satisfies: RON-FR-730, RON-FR-734 | Test: RON-TC-LQR-001 */
typedef struct {
    ron_lqr_config_t cfg;
    ron_lqr_state_t state;
    ron_obs_t observer; /**< Embedded Luenberger observer. */
    ron_kf_t kalman;    /**< Embedded Kalman filter.       */
} ron_lqr_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
 * @brief Initialise a MIMO linear quadratic regulator.
 *
 * Implements @c u = -K*x_hat + Kr*r for an @c m -input, @c n -state plant.
 *
 * With @c gain_mode == ::RON_LQR_GAIN_PRECOMPUTED the supplied @c K and @c Kr
 * are used directly. With ::RON_LQR_GAIN_DARE the gain is solved here from
 * @c A, @c B, @c Q_cost and @c R_cost by iterating the Riccati recursion
 * until successive solutions differ by less than @c dare_tol, or until
 * @c dare_max_iter iterations have run.
 *
 * The DARE solve happens once, at init, and is the expensive call in this
 * module; steady-state stepping is cheap. Note that @c dare_tol is compared
 * against values of the same magnitude as the solution matrix, so in a
 * single-precision build a tolerance near float epsilon may never be
 * reached - prefer a value the arithmetic can actually resolve.
 *
 * @param[out] lqr  Controller instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration. Dimensions @c n and @c m must be within
 *                  ::RON_LQR_MAX_STATES and ::RON_LQR_MAX_INPUTS, all active
 *                  matrix entries and limits must be finite, and the selected
 *                  estimate source must be configured consistently. Must not
 *                  be NULL.
 *
 * @retval RON_FAULT_NONE           Controller ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p lqr or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A dimension, matrix, cost or limit was
 *                                  invalid, or an embedded estimator's
 *                                  configuration was rejected.
 * @retval RON_FAULT_OUTPUT_NAN     The DARE iteration failed to converge
 *                                  within @c dare_max_iter, or produced a
 *                                  non-finite result.
 */
/* Satisfies: RON-FR-730, RON-FR-733 | Test: RON-TC-LQR-001, RON-TC-LQR-003 */
ron_fault_t ron_lqr_init(ron_lqr_t *lqr, const ron_lqr_config_t *cfg);

/**
 * @brief Return the controller to its post-initialisation state.
 *
 * Clears the integral accumulator, output history and any latched fault, and
 * resets the embedded estimator if one is in use. The solved gain is kept, so
 * this does not repeat the DARE solve.
 *
 * @param[in,out] lqr  Initialised controller instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Controller reset.
 * @retval RON_FAULT_NULL_POINTER   @p lqr was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised.
 */
/* Satisfies: RON-FR-736 | Test: RON-TC-LQR-006 */
ron_fault_t ron_lqr_reset(ron_lqr_t *lqr);

/**
 * @brief Compute one control vector from the current state estimate.
 *
 * Fetches the state estimate from the configured source, forms
 * @c u = -K*x_hat + Kr*r, adds the optional per-input integral term, then
 * applies per-input saturation and rate limiting.
 *
 * The estimate is not advanced here; whichever estimator the instance was
 * configured with must be driven separately each cycle.
 *
 * @param[in,out] lqr     Initialised controller instance. Must not be NULL.
 * @param[in]     r       Reference vector, @c m entries, all finite. Must not
 *                        be NULL.
 * @param[in]     dt      Sample period in seconds. Must be positive and
 *                        finite; it scales both the integral term and the
 *                        rate limit.
 * @param[out]    u       Receives the control vector, @c m entries. Must not
 *                        be NULL.
 * @param[out]    status  Receives the status word. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p lqr, @p r, @p u or @p status was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  @p dt was not positive.
 * @retval RON_FAULT_INPUT_NAN      An entry of @p r, or the state estimate,
 *                                  was not finite; the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     A computed output was not finite; the
 *                                  fault latches.
 */
/* Satisfies: RON-FR-730, RON-FR-735, RON-FR-736 | Test: RON-TC-LQR-001..007 */
ron_fault_t ron_lqr_step(ron_lqr_t *lqr, const ron_float_t r[RON_LQR_MAX_INPUTS], ron_float_t dt,
                         ron_float_t u[RON_LQR_MAX_INPUTS], ron_status_t *status);

/**
 * @brief Replace the feedback and reference gains at runtime.
 *
 * Bypasses the DARE solver entirely, which makes this the mechanism for
 * gain-scheduling an LQR across operating points using gains computed
 * offline. Integral state and output history are preserved.
 *
 * @param[in,out] lqr  Initialised controller instance. Must not be NULL.
 * @param[in]     K    New gain matrix, @c m by @c n, all active entries
 *                     finite. Must not be NULL.
 * @param[in]     Kr   New reference gain vector, @c m entries, all finite.
 *                     Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Gains replaced.
 * @retval RON_FAULT_NULL_POINTER   @p lqr, @p K or @p Kr was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or a
 *                                  supplied gain was not finite.
 */
/* Satisfies: RON-FR-738 | Test: RON-TC-LQR-005 */
ron_fault_t ron_lqr_set_gains(ron_lqr_t *lqr,
                              const ron_float_t K[RON_LQR_MAX_INPUTS][RON_LQR_MAX_STATES],
                              const ron_float_t Kr[RON_LQR_MAX_INPUTS]);

/**
 * @brief Copy out the Riccati solution matrix @c P.
 *
 * Only meaningful after a ::RON_LQR_GAIN_DARE initialisation. @c P is the
 * cost-to-go matrix, useful for verifying the design offline or as a
 * Lyapunov function for stability analysis.
 *
 * @param[in]  lqr  Initialised controller instance. Must not be NULL.
 * @param[out] P    Receives the solution; the leading @c n by @c n block is
 *                  written. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Solution copied.
 * @retval RON_FAULT_NULL_POINTER   @p lqr or @p P was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  no DARE solution exists because the gain
 *                                  was supplied pre-computed.
 */
/* Satisfies: RON-FR-739 | Test: RON-TC-LQR-003 */
ron_fault_t ron_lqr_get_dare_solution(const ron_lqr_t *lqr,
                                      ron_float_t P[RON_LQR_MAX_STATES][RON_LQR_MAX_STATES]);

/* Advance the embedded Luenberger observer (LUENBERGER source). */
/**
 * @brief Advance the embedded Luenberger observer by one sample.
 *
 * Only meaningful when the instance was configured with
 * ::RON_LQR_SOURCE_LUENBERGER. Call it once per cycle before ron_lqr_step().
 *
 * @param[in,out] lqr  Initialised controller instance. Must not be NULL.
 * @param[in]     y    Measured output vector, all entries finite. Must not be
 *                     NULL.
 * @param[in]     u    Previously applied input vector, all entries finite.
 *                     May be NULL when the observer's input dimension is
 *                     zero.
 *
 * @retval RON_FAULT_NONE           Observer advanced.
 * @retval RON_FAULT_NULL_POINTER   @p lqr or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  observer.
 * @retval other                    Any fault reported by the observer.
 */
/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-008 */
ron_fault_t ron_lqr_observer_step(ron_lqr_t *lqr, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                  const ron_float_t u[RON_SS_MAX_INPUTS]);

/* Advance the embedded Kalman filter prediction (KALMAN source). */
/**
 * @brief Run the embedded Kalman filter's time update.
 *
 * Only meaningful when the instance was configured with
 * ::RON_LQR_SOURCE_KALMAN.
 *
 * @param[in,out] lqr  Initialised controller instance. Must not be NULL.
 * @param[in]     u    Control input vector, all entries finite. May be NULL
 *                     when the filter's input dimension is zero.
 *
 * @retval RON_FAULT_NONE           Estimate propagated.
 * @retval RON_FAULT_NULL_POINTER   @p lqr was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  Kalman filter.
 * @retval other                    Any fault reported by the filter.
 */
/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-009 */
ron_fault_t ron_lqr_kalman_predict(ron_lqr_t *lqr, const ron_float_t u[RON_KF_MAX_INPUTS]);

/* Correct the embedded Kalman filter with a measurement (KALMAN source). */
/**
 * @brief Run the embedded Kalman filter's measurement update.
 *
 * Only meaningful when the instance was configured with
 * ::RON_LQR_SOURCE_KALMAN. Pass @p z_valid as @c false to skip the correction
 * for a sample with no usable measurement.
 *
 * @param[in,out] lqr      Initialised controller instance. Must not be NULL.
 * @param[in]     z        Measurement vector, all entries finite. Ignored
 *                         when @p z_valid is @c false.
 * @param[in]     z_valid  Whether @p z holds a usable measurement.
 *
 * @retval RON_FAULT_NONE           Estimate corrected, or correction skipped.
 * @retval RON_FAULT_NULL_POINTER   @p lqr was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  Kalman filter.
 * @retval other                    Any fault reported by the filter.
 */
/* Satisfies: RON-FR-734 | Test: RON-TC-LQR-009 */
ron_fault_t ron_lqr_kalman_update(ron_lqr_t *lqr, const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                                  bool z_valid);

#ifdef __cplusplus
}
#endif

#endif /* RON_LQR_H */
