/*
 * @file     ron_feedforward.h
 * @brief    Public API for the Regulon PID feed-forward extension.
 * @module   ron_feedforward
 * @doc      RON-IS-001
 * @req      RON-FR-200, RON-FR-201, RON-FR-202, RON-FR-203,
 *           RON-FR-204, RON-FR-205
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_FEEDFORWARD_H
#define RON_FEEDFORWARD_H

#include "ron/ron_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check a feed-forward configuration without applying it.
 *
 * Useful for validating a configuration built at runtime before committing it
 * to a live controller with ron_pid_set_feedforward().
 *
 * @param[in] cfg  Configuration to check. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Configuration is valid.
 * @retval RON_FAULT_NULL_POINTER   @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The mode was unrecognised, or @c gain or
 *                                  @c N_ff was not finite.
 */
/* Satisfies: RON-FR-201, RON-FR-202 | Test: RON-TC-FF-002 - RON-TC-FF-006 */
ron_fault_t ron_feedforward_config_validate(const ron_feedforward_config_t *cfg);

/**
 * @brief Attach or replace a controller's feed-forward configuration.
 *
 * The configuration is validated before being applied, so a rejected
 * configuration leaves the controller's existing behaviour untouched.
 *
 * @param[in,out] inst  Initialised PID instance. Must not be NULL.
 * @param[in]     cfg   Feed-forward configuration. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Configuration applied.
 * @retval RON_FAULT_NULL_POINTER   @p inst or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The instance was never initialised, or the
 *                                  configuration failed validation.
 */
/* Satisfies: RON-FR-201, RON-FR-202, RON-FR-204 | Test: RON-TC-FF-002 - RON-TC-FF-008 */
ron_fault_t ron_pid_set_feedforward(ron_pid_instance_t *inst, const ron_feedforward_config_t *cfg);

/**
 * @brief Step a PID controller with an added feed-forward term.
 *
 * Behaves exactly as ron_pid_step(), except that the configured feed-forward
 * contribution and @p external_ff are added to the control output before
 * saturation and rate limiting.
 *
 * Because feed-forward is added ahead of the limiter, it participates in
 * anti-windup: a feed-forward term large enough to saturate the output will
 * correctly stop the integrator winding.
 *
 * @param[in,out] inst         Initialised PID instance. Must not be NULL.
 * @param[in]     r            Setpoint. Must be finite.
 * @param[in]     y            Process variable. Must be finite.
 * @param[in]     dt           Sample period in seconds. Must be positive and
 *                             finite.
 * @param[in]     external_ff  Additional caller-computed feed-forward term,
 *                             added as-is. Pass 0 when unused. Must be
 *                             finite.
 * @param[out]    u_out        Receives the control output. Must not be NULL.
 * @param[out]    status       Receives the status word. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p inst, @p u_out or @p status was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The instance was never initialised, or
 *                                  @p dt was not positive.
 * @retval other                    Any fault ron_pid_step() would report,
 *                                  including latched faults.
 */
/* Satisfies: RON-FR-200 - RON-FR-205 | Test: RON-TC-FF-001 - RON-TC-FF-009 */
ron_fault_t ron_pid_step_feedforward(ron_pid_instance_t *inst, ron_float_t r, ron_float_t y,
                                     ron_float_t dt, ron_float_t external_ff, ron_float_t *u_out,
                                     ron_status_t *status);

/**
 * @brief Read back the feed-forward contribution from the last step.
 *
 * Reports the term the controller added, which is what separates the
 * feed-forward and feedback halves of the output when tuning or logging.
 *
 * @param[in]  inst  Initialised PID instance. Must not be NULL.
 * @param[out] u_ff  Receives the feed-forward contribution. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Value copied.
 * @retval RON_FAULT_NULL_POINTER   @p inst or @p u_ff was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The instance was never initialised.
 */
/* Satisfies: RON-FR-205 | Test: RON-TC-FF-001, RON-TC-FF-009 */
ron_fault_t ron_pid_get_feedforward(const ron_pid_instance_t *inst, ron_float_t *u_ff);

#ifdef __cplusplus
}
#endif

#endif /* RON_FEEDFORWARD_H */
