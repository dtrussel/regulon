/*
 * @file     ron_estimator.c
 * @brief    State-estimate source shared by the state-space controller and LQR.
 * @module   ron_estimator
 * @doc      RON-IS-001
 * @req      RON-FR-701, RON-FR-734
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_estimator.h"

#include "ron_matrix_internal.h"

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
static bool estimator_source_valid(ron_estimator_source_t source)
{
    return (source == RON_ESTIMATOR_EXTERNAL) || (source == RON_ESTIMATOR_LUENBERGER) ||
           (source == RON_ESTIMATOR_KALMAN);
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
static ron_fault_t estimator_check(const ron_estimator_t *est, ron_estimator_source_t source)
{
    if (est == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!est->is_initialised || (est->source != source)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_config_validate(const ron_estimator_config_t *cfg, uint8_t n)
{
    if (cfg == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!estimator_source_valid(cfg->source)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((cfg->source == RON_ESTIMATOR_LUENBERGER) && (cfg->obs_cfg.n != n)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((cfg->source == RON_ESTIMATOR_KALMAN) && (cfg->kf_cfg.n != n)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_init(ron_estimator_t *est, const ron_estimator_config_t *cfg, uint8_t n)
{
    ron_fault_t fault;

    if (est == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    fault = ron_estimator_config_validate(cfg, n);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    est->is_initialised = false;
    if (cfg->source == RON_ESTIMATOR_LUENBERGER) {
        fault = ron_obs_init(&est->observer, &cfg->obs_cfg);
    } else if (cfg->source == RON_ESTIMATOR_KALMAN) {
        fault = ron_kf_init(&est->kalman, &cfg->kf_cfg);
    } else {
        /* EXTERNAL: no embedded estimator to initialise. */
    }
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    est->source         = cfg->source;
    est->x_ext          = cfg->x_ext;
    est->is_initialised = true;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-001 */
ron_fault_t ron_estimator_reset(ron_estimator_t *est)
{
    if (est == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!est->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    if (est->source == RON_ESTIMATOR_LUENBERGER) {
        (void) ron_obs_reset(&est->observer);
    } else if (est->source == RON_ESTIMATOR_KALMAN) {
        (void) ron_kf_reset(&est->kalman);
    } else {
        /* EXTERNAL: nothing to reset. */
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_observer_step(ron_estimator_t *est,
                                        const ron_float_t y[RON_SS_MAX_OUTPUTS],
                                        const ron_float_t u[RON_SS_MAX_INPUTS])
{
    ron_fault_t fault = estimator_check(est, RON_ESTIMATOR_LUENBERGER);

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    return ron_obs_step(&est->observer, y, u);
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_kalman_predict(ron_estimator_t *est,
                                         const ron_float_t u[RON_KF_MAX_INPUTS])
{
    ron_fault_t fault = estimator_check(est, RON_ESTIMATOR_KALMAN);

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    return ron_kf_predict(&est->kalman, u);
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-002 */
ron_fault_t ron_estimator_kalman_update(ron_estimator_t *est,
                                        const ron_float_t z[RON_KF_MAX_MEASUREMENTS], bool z_valid)
{
    ron_fault_t fault = estimator_check(est, RON_ESTIMATOR_KALMAN);

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    return ron_kf_update(&est->kalman, z, z_valid);
}

/* Satisfies: RON-FR-701, RON-FR-734 | Test: RON-TC-EST-003 */
ron_fault_t ron_estimator_get_state(const ron_estimator_t *est, ron_float_t *x_hat, uint8_t n)
{
    const ron_float_t *src;
    uint8_t i;

    if ((est == NULL) || (x_hat == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!est->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    if (est->source == RON_ESTIMATOR_LUENBERGER) {
        src = &est->observer.state.x_hat[0];
    } else if (est->source == RON_ESTIMATOR_KALMAN) {
        src = &est->kalman.state.x_hat[0];
    } else {
        src = est->x_ext;
    }
    if (src == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!ron_mat_vec_finite(src, n)) {
        return RON_FAULT_INPUT_NAN;
    }
    for (i = 0U; i < n; i++) {
        x_hat[i] = src[i];
    }
    return RON_FAULT_NONE;
}
