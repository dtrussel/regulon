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

/* Satisfies: RON-FR-201, RON-FR-202 | Test: RON-TC-FF-002 - RON-TC-FF-006 */
ron_fault_t ron_feedforward_config_validate(const ron_feedforward_config_t *cfg);

/* Satisfies: RON-FR-201, RON-FR-202, RON-FR-204 | Test: RON-TC-FF-002 - RON-TC-FF-008 */
ron_fault_t ron_pid_set_feedforward(ron_pid_instance_t *inst, const ron_feedforward_config_t *cfg);

/* Satisfies: RON-FR-200 - RON-FR-205 | Test: RON-TC-FF-001 - RON-TC-FF-009 */
ron_fault_t ron_pid_step_feedforward(ron_pid_instance_t *inst, ron_float_t r, ron_float_t y,
                                     ron_float_t dt, ron_float_t external_ff, ron_float_t *u_out,
                                     ron_status_t *status);

/* Satisfies: RON-FR-205 | Test: RON-TC-FF-001, RON-TC-FF-009 */
ron_fault_t ron_pid_get_feedforward(const ron_pid_instance_t *inst, ron_float_t *u_ff);

#ifdef __cplusplus
}
#endif

#endif /* RON_FEEDFORWARD_H */
