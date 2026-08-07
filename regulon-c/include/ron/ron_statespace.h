/*
 * @file     ron_statespace.h
 * @brief    Public API for the Regulon discrete-time state-feedback controller.
 * @module   ron_statespace
 * @doc      RON-IS-001
 * @req      RON-FR-700, RON-FR-701, RON-FR-702, RON-FR-703, RON-FR-704
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Discrete-time state-feedback controller
 *
 *     u(k) = -K x_hat(k) + K_r r(k)
 *
 * with optional integral augmentation for output regulation and PID-equivalent
 * output saturation and rate limiting.  The state estimate x_hat may be taken
 * from a caller-supplied external vector, an embedded Luenberger observer, or
 * an embedded Kalman filter (RON-FR-701).
 *
 * The embedded observer / Kalman instances are advanced by the caller through
 * the ron_ss_observer_step() / ron_ss_kalman_predict() / ron_ss_kalman_update()
 * helpers; ron_ss_step() then consumes the most recent estimate.  All storage
 * is caller-owned inside a single ron_ss_t: no dynamic allocation, recursion,
 * or VLAs are used.
 *
 * Typical usage (Luenberger source):
 *
 *   static ron_ss_t ss;
 *
 *   void init(void) {
 *       ron_ss_config_t cfg = { .n = 2U, .source = RON_SS_SOURCE_LUENBERGER,
 *                               .K = {1.0F, 0.5F}, .Kr = 1.0F,
 *                               .u_min = -10.0F, .u_max = 10.0F,
 *                               .obs_cfg = { ... } };
 *       (void)ron_ss_init(&ss, &cfg);
 *   }
 *
 *   void isr(ron_float_t r, ron_float_t y_meas, ron_float_t u_applied) {
 *       ron_float_t y[RON_SS_MAX_OUTPUTS] = { y_meas };
 *       ron_float_t u_in[RON_SS_MAX_INPUTS] = { u_applied };
 *       (void)ron_ss_observer_step(&ss, y, u_in);
 *       ron_float_t u; ron_status_t status;
 *       (void)ron_ss_step(&ss, r, 0.001F, &u, &status);
 *       actuator_set(u);
 *   }
 */

#ifndef RON_STATESPACE_H
#define RON_STATESPACE_H

#include "ron/ron_kalman.h"   /* embedded Kalman source (RON-FR-701)   */
#include "ron/ron_observer.h" /* embedded Luenberger source (RON-FR-701) */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * State-estimate source selection (RON-FR-701)
 * ========================================================================= */

/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
typedef enum {
    RON_SS_SOURCE_EXTERNAL   = 0, /**< x_hat taken from cfg.x_ext.            */
    RON_SS_SOURCE_LUENBERGER = 1, /**< x_hat taken from the embedded observer. */
    RON_SS_SOURCE_KALMAN     = 2  /**< x_hat taken from the embedded Kalman.   */
} ron_ss_source_t;

/* =========================================================================
 * Controller configuration (RON-FR-700 .. RON-FR-704)
 *
 * Only the leading n entries of K / C_out and the external vector are used.
 * obs_cfg is consumed only when source == RON_SS_SOURCE_LUENBERGER; kf_cfg is
 * consumed only when source == RON_SS_SOURCE_KALMAN.
 * ========================================================================= */

/* Satisfies: RON-FR-700, RON-FR-701, RON-FR-702, RON-FR-703 | Test: RON-TC-SS-001, RON-TC-SS-002, RON-TC-SS-003, RON-TC-SS-004 */
typedef struct {
    uint8_t n;                /**< State dimension (1..RON_SS_MAX_STATES).      */
    ron_ss_source_t source;   /**< State-estimate source.                       */
    const ron_float_t *x_ext; /**< External state vector (EXTERNAL source).    */

    ron_float_t K[RON_SS_MAX_STATES]; /**< State-feedback row gain (1 x n).    */
    ron_float_t Kr;                   /**< Reference pre-gain.                 */

    bool use_integral;                    /**< Enable integral augmentation.   */
    ron_float_t Ki_aug;                   /**< Integral gain.                  */
    ron_float_t C_out[RON_SS_MAX_STATES]; /**< Regulated-output row (1 x n).   */
    ron_float_t i_min;                    /**< Integral lower clamp.           */
    ron_float_t i_max;                    /**< Integral upper clamp.           */

    ron_float_t u_min;  /**< Minimum control output. Must be < u_max.          */
    ron_float_t u_max;  /**< Maximum control output.                           */
    ron_float_t du_max; /**< Max |Δu| per second. <= 0 disables rate limiting. */

    ron_obs_config_t obs_cfg; /**< Embedded observer config (LUENBERGER).      */
    ron_kf_config_t kf_cfg;   /**< Embedded Kalman config (KALMAN).            */
} ron_ss_config_t;

/* =========================================================================
 * Controller state (RON-FR-702, RON-FR-703)
 * ========================================================================= */

/* Satisfies: RON-FR-702, RON-FR-703 | Test: RON-TC-SS-003, RON-TC-SS-004 */
typedef struct {
    ron_float_t integral; /**< Augmented-integral accumulator. */
    ron_float_t u_prev;   /**< Previous output (rate limiting). */
    ron_fault_t faults;   /**< Latched fault register.          */
    bool is_initialised;  /**< Set by ron_ss_init.              */
} ron_ss_state_t;

/* Satisfies: RON-FR-700, RON-FR-701 | Test: RON-TC-SS-001, RON-TC-SS-002 */
typedef struct {
    ron_ss_config_t cfg;
    ron_ss_state_t state;
    ron_obs_t observer; /**< Embedded Luenberger observer. */
    ron_kf_t kalman;    /**< Embedded Kalman filter.       */
} ron_ss_t;

/* =========================================================================
 * API
 * ========================================================================= */

/**
 * @brief Initialise a state-feedback controller.
 *
 * Implements @c u = -K*x_hat + Kr*r. The state estimate comes from one of
 * three sources selected by @c cfg.source: a caller-owned vector, an
 * embedded Luenberger observer, or an embedded Kalman filter. For the two
 * embedded sources the corresponding nested configuration is validated and
 * initialised here as well.
 *
 * @param[out] ss   Controller instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration. The state dimension @c n must be within
 *                  ::RON_SS_MAX_STATES, gains and limits must be finite, and
 *                  the selected estimate source must be configured
 *                  consistently. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Controller ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p ss or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A dimension, gain, limit or estimator
 *                                  configuration was invalid.
 */
/* Satisfies: RON-FR-700, RON-FR-701 | Test: RON-TC-SS-001, RON-TC-SS-002, RON-TC-SS-009 */
ron_fault_t ron_ss_init(ron_ss_t *ss, const ron_ss_config_t *cfg);

/**
 * @brief Return the controller to its post-initialisation state.
 *
 * Clears the integral accumulator, output history and any latched fault, and
 * resets the embedded estimator if one is in use.
 *
 * @param[in,out] ss  Initialised controller instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Controller reset.
 * @retval RON_FAULT_NULL_POINTER   @p ss was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised.
 */
/* Satisfies: RON-FR-702 | Test: RON-TC-SS-003 */
ron_fault_t ron_ss_reset(ron_ss_t *ss);

/**
 * @brief Compute one control output from the current state estimate.
 *
 * Fetches the state estimate from the configured source, forms
 * @c u = -K*x_hat + Kr*r, then applies saturation and rate limiting.
 *
 * The estimate is not advanced by ron_ss_step(); whichever estimator the
 * instance was configured with must be driven separately each cycle.
 *
 * @param[in,out] ss      Initialised controller instance. Must not be NULL.
 * @param[in]     r       Reference input. Must be finite.
 * @param[in]     dt      Sample period in seconds. Must be positive and
 *                        finite; it scales the rate limit.
 * @param[out]    u       Receives the control output. Must not be NULL.
 * @param[out]    status  Receives the status word. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p ss, @p u or @p status was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  @p dt was not positive.
 * @retval RON_FAULT_INPUT_NAN      @p r, @p dt or the state estimate was not
 *                                  finite; the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     The computed output was not finite; the
 *                                  fault latches.
 */
/* Satisfies: RON-FR-700, RON-FR-702, RON-FR-703 | Test: RON-TC-SS-001, RON-TC-SS-003, RON-TC-SS-004 */
ron_fault_t ron_ss_step(ron_ss_t *ss, ron_float_t r, ron_float_t dt, ron_float_t *u,
                        ron_status_t *status);

/**
 * @brief Replace the feedback and reference gains at runtime.
 *
 * Changes only the gains; the estimator, integral state and output history
 * are left alone, so control continues without a discontinuity beyond the one
 * the new gains imply.
 *
 * @param[in,out] ss  Initialised controller instance. Must not be NULL.
 * @param[in]     K   New state-feedback gain row, @c n entries, all finite.
 *                    Must not be NULL.
 * @param[in]     Kr  New reference gain. Must be finite.
 *
 * @retval RON_FAULT_NONE           Gains replaced.
 * @retval RON_FAULT_NULL_POINTER   @p ss or @p K was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or a
 *                                  supplied gain was not finite.
 */
/* Satisfies: RON-FR-704 | Test: RON-TC-SS-005 */
ron_fault_t ron_ss_set_gains(ron_ss_t *ss, const ron_float_t K[RON_SS_MAX_STATES], ron_float_t Kr);

/* Advance the embedded Luenberger observer (LUENBERGER source). */
/**
 * @brief Advance the embedded Luenberger observer by one sample.
 *
 * Only meaningful when the instance was configured with
 * ::RON_SS_SOURCE_LUENBERGER. Call it once per cycle before ron_ss_step().
 *
 * @param[in,out] ss  Initialised controller instance. Must not be NULL.
 * @param[in]     y   Measured output vector, all entries finite. Must not be
 *                    NULL.
 * @param[in]     u   Previously applied input vector, all entries finite. May
 *                    be NULL when the observer's input dimension is zero.
 *
 * @retval RON_FAULT_NONE           Observer advanced.
 * @retval RON_FAULT_NULL_POINTER   @p ss or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  observer.
 * @retval other                    Any fault reported by the observer.
 */
/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_observer_step(ron_ss_t *ss, const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                 const ron_float_t u[RON_SS_MAX_INPUTS]);

/* Advance the embedded Kalman filter prediction (KALMAN source). */
/**
 * @brief Run the embedded Kalman filter's time update.
 *
 * Only meaningful when the instance was configured with
 * ::RON_SS_SOURCE_KALMAN.
 *
 * @param[in,out] ss  Initialised controller instance. Must not be NULL.
 * @param[in]     u   Control input vector, all entries finite. May be NULL
 *                    when the filter's input dimension is zero.
 *
 * @retval RON_FAULT_NONE           Estimate propagated.
 * @retval RON_FAULT_NULL_POINTER   @p ss was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  Kalman filter.
 * @retval other                    Any fault reported by the filter.
 */
/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_kalman_predict(ron_ss_t *ss, const ron_float_t u[RON_KF_MAX_INPUTS]);

/* Correct the embedded Kalman filter with a measurement (KALMAN source). */
/**
 * @brief Run the embedded Kalman filter's measurement update.
 *
 * Only meaningful when the instance was configured with
 * ::RON_SS_SOURCE_KALMAN. Pass @p z_valid as @c false to skip the correction
 * for a sample with no usable measurement.
 *
 * @param[in,out] ss       Initialised controller instance. Must not be NULL.
 * @param[in]     z        Measurement vector, all entries finite. Ignored
 *                         when @p z_valid is @c false.
 * @param[in]     z_valid  Whether @p z holds a usable measurement.
 *
 * @retval RON_FAULT_NONE           Estimate corrected, or correction skipped.
 * @retval RON_FAULT_NULL_POINTER   @p ss was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The controller was never initialised, or
 *                                  its estimate source is not the embedded
 *                                  Kalman filter.
 * @retval other                    Any fault reported by the filter.
 */
/* Satisfies: RON-FR-701 | Test: RON-TC-SS-002 */
ron_fault_t ron_ss_kalman_update(ron_ss_t *ss, const ron_float_t z[RON_KF_MAX_MEASUREMENTS],
                                 bool z_valid);

#ifdef __cplusplus
}
#endif

#endif /* RON_STATESPACE_H */
