/*
 * @file     ron_pid_internal.h
 * @brief    Internal helpers for the Regulon PID controller module.
 * @module   ron_pid_internal
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-050, RON-SR-010, RON-SR-012
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_PID_INTERNAL_H
#define RON_PID_INTERNAL_H

#include "ron/ron_pid.h"

/* Satisfies: RON-FR-001 – RON-FR-071 | Test: RON-TC-PID-001 – RON-TC-PID-039 */
ron_fault_t ron_pid_core_step(ron_pid_instance_t *inst,
                              ron_float_t         r,
                              ron_float_t         y,
                              ron_float_t         dt,
                              ron_float_t        *u_out,
                              ron_status_t       *status);

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
void ron_pid_fault_set(ron_pid_instance_t *inst,
                       ron_fault_t         code);

/* Satisfies: RON-SR-011 | Test: RON-TC-SAFE-008 */
ron_float_t ron_pid_fault_safe_output(const ron_pid_instance_t *inst);

#endif /* RON_PID_INTERNAL_H */
