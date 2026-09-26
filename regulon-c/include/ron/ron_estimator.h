/*
 * @file     ron_estimator.h
 * @brief    Public API for the state-estimate source shared by controllers.
 * @module   ron_estimator
 * @doc      RON-IS-001
 * @req      RON-FR-701, RON-FR-734
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * The state-space controller (RON-FR-701) and the LQR (RON-FR-734) take their
 * state estimate x_hat from one of three sources, selected at configuration
 * time: a caller-owned external vector, an embedded Luenberger observer, or
 * an embedded Kalman filter. This component owns that choice. A controller
 * embeds the configuration as cfg.est and the instance as est, and the caller
 * advances the embedded estimator through the functions below:
 *
 *       ron_ss_t ss;                       // cfg.est.source = RON_ESTIMATOR_KALMAN
 *       (void)ron_ss_init(&ss, &cfg);
 *       (void)ron_estimator_kalman_predict(&ss.est, u_prev);
 *       (void)ron_estimator_kalman_update(&ss.est, z, true);
 *       (void)ron_ss_step(&ss, r, dt, &u, &status);
 */

#ifndef RON_ESTIMATOR_H
#define RON_ESTIMATOR_H

#include "ron/ron_kalman.h"
#include "ron/ron_observer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
typedef enum {
    RON_ESTIMATOR_EXTERNAL   = 0, /**< x_hat taken from cfg.x_ext.             */
    RON_ESTIMATOR_LUENBERGER = 1, /**< x_hat taken from the embedded observer. */
    RON_ESTIMATOR_KALMAN     = 2  /**< x_hat taken from the embedded Kalman.   */
} ron_estimator_source_t;

/**
 * @brief Estimator configuration. obs_cfg is used only by the LUENBERGER
 *        source, kf_cfg only by KALMAN, x_ext only by EXTERNAL.
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
typedef struct {
    ron_estimator_source_t source; /**< State-estimate source.                  */
    const ron_float_t *x_ext;      /**< External state vector (EXTERNAL).       */
    ron_obs_config_t obs_cfg;      /**< Embedded observer config (LUENBERGER).  */
    ron_kf_config_t kf_cfg;        /**< Embedded Kalman config (KALMAN).        */
} ron_estimator_config_t;

/** @brief Estimator instance. Access it only through this API. */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
typedef struct {
    ron_estimator_source_t source; /**< Selected source.                    */
    const ron_float_t *x_ext;      /**< External state vector (EXTERNAL).   */
    ron_obs_t observer;            /**< Embedded Luenberger observer.       */
    ron_kf_t kalman;               /**< Embedded Kalman filter.             */
    bool is_initialised;           /**< Set by ron_estimator_init.          */
} ron_estimator_t;

/**
 * @brief Validate an estimator configuration for an n-state controller.
 *
 * @param[in] cfg  Configuration to check. Must not be NULL.
 * @param[in] n    State dimension of the owning controller.
 *
 * @retval RON_FAULT_NONE           Valid.
 * @retval RON_FAULT_NULL_POINTER   @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID Unknown source, or the selected embedded
 *                                  estimator's dimension differs from @p n.
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_config_validate(const ron_estimator_config_t *cfg, uint8_t n);

/**
 * @brief Validate the configuration and initialise the selected embedded
 *        estimator (none for EXTERNAL).
 *
 * @param[out] est  Instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration. Must not be NULL.
 * @param[in]  n    State dimension of the owning controller.
 *
 * @retval RON_FAULT_NONE           Initialised.
 * @retval RON_FAULT_NULL_POINTER   @p est or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID Rejected by ron_estimator_config_validate().
 * @return Any fault from ron_obs_init() / ron_kf_init(), unchanged.
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_init(ron_estimator_t *est, const ron_estimator_config_t *cfg, uint8_t n);

/**
 * @brief Reset the selected embedded estimator to its initial estimate.
 *
 * @param[in,out] est  Initialised estimator. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Reset (nothing to do for EXTERNAL).
 * @retval RON_FAULT_NULL_POINTER   @p est was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @p est was never initialised.
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_reset(ron_estimator_t *est);

/**
 * @brief Advance the embedded Luenberger observer by one sample
 *        (LUENBERGER source). Call it once per cycle before the controller's
 *        step.
 *
 * @param[in,out] est  Initialised estimator. Must not be NULL.
 * @param[in]     y    Measured output vector, all entries finite. Must not be
 *                     NULL.
 * @param[in]     u    Previously applied input vector, all entries finite.
 *                     May be NULL when the observer's input dimension is zero.
 *
 * @retval RON_FAULT_NULL_POINTER   @p est was NULL.
 * @retval RON_FAULT_CONFIG_INVALID Not initialised, or another source.
 * @return Otherwise the result of ron_obs_step().
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_observer_step(ron_estimator_t *est,
                                        const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                        const ron_float_t u[RON_SS_MAX_INPUTS]);

/**
 * @brief Run the embedded Kalman filter's time update (KALMAN source).
 *
 * @param[in,out] est  Initialised estimator. Must not be NULL.
 * @param[in]     u    Control input vector, all entries finite. May be NULL
 *                     when the filter's input dimension is zero.
 *
 * @retval RON_FAULT_NULL_POINTER   @p est was NULL.
 * @retval RON_FAULT_CONFIG_INVALID Not initialised, or another source.
 * @return Otherwise the result of ron_kf_predict().
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_kalman_predict(ron_estimator_t *est,
                                         const ron_float_t u[RON_KF_MAX_INPUTS]);

/**
 * @brief Run the embedded Kalman filter's measurement update (KALMAN source).
 *
 * @param[in,out] est      Initialised estimator. Must not be NULL.
 * @param[in]     z        Measurement vector, all entries finite. Ignored when
 *                         @p z_valid is @c false.
 * @param[in]     z_valid  Whether @p z holds a usable measurement; @c false
 *                         skips the correction for this sample.
 *
 * @retval RON_FAULT_NULL_POINTER   @p est was NULL.
 * @retval RON_FAULT_CONFIG_INVALID Not initialised, or another source.
 * @return Otherwise the result of ron_kf_update().
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_kalman_update(ron_estimator_t *est,
                                        const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid);

/**
 * @brief Copy the leading n entries of the current state estimate.
 *
 * @param[in]  est    Initialised instance. Must not be NULL.
 * @param[out] x_hat  Receives n values. Must not be NULL.
 * @param[in]  n      Number of states to copy (the owning controller's n).
 *
 * @retval RON_FAULT_NONE           Copied.
 * @retval RON_FAULT_NULL_POINTER   @p est, @p x_hat, or the external vector
 *                                  (EXTERNAL source) was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @p est was never initialised.
 * @retval RON_FAULT_INPUT_NAN      The estimate contains NaN or Inf.
 */
/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-003 */
ron_fault_t ron_estimator_get_state(const ron_estimator_t *est, ron_float_t *x_hat, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* RON_ESTIMATOR_H */
