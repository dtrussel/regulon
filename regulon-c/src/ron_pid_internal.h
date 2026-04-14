/*
 * @file     ron_pid_internal.h
 * @brief    Private (intra-library) prototypes for the PID controller.
 * @module   ron_pid_internal
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-030, RON-SR-010, RON-SR-011, RON-SR-012
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * This header is NOT installed and MUST NOT be included by library
 * consumers.  It exposes the internal PID core and fault-manager entry
 * points used by ron_pid_api.c.
 */

#ifndef RON_PID_INTERNAL_H
#define RON_PID_INTERNAL_H

#include "ron/ron_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Fault manager (ron_pid_fault.c)
 * ========================================================================= */

/**
 * @brief OR the supplied fault bits into the instance's sticky fault
 *        register, set the STATUS_FAULT status bit, and invoke the
 *        optional fault callback once for each bit that was NEWLY set.
 *
 * @note  Previously-latched bits do NOT re-trigger the callback
 *        (ron_pid_types.h docstring for ron_fault_cb_t).
 *
 * Satisfies: RON-SR-010, RON-SR-013.
 */
void ron_pid_fault_set(ron_pid_instance_t *inst, ron_fault_t code);

/**
 * @brief Compute the safe-state output for the configured policy.
 *
 * Return value is the u_out that callers of ron_pid_step() should emit
 * while the instance is latched in a fault state.
 *
 * Satisfies: RON-SR-011.
 */
ron_float_t ron_pid_fault_safe_output(const ron_pid_instance_t *inst);

/* =========================================================================
 * Core computation (ron_pid_core.c)
 * ========================================================================= */

/**
 * @brief Execute the full 17-stage PID computation pipeline.
 *
 * Invoked by ron_pid_step() after all boundary checks pass (inst/u_out/
 * status non-NULL, instance initialised, no latched fault, dt > 0 and
 * finite).  Updates inst->state in place and writes *u_out and *status.
 *
 * @return RON_FAULT_NONE on success, or the fault bit that was newly
 *         latched (FAULT_INPUT_NAN, FAULT_OUTPUT_NAN,
 *         FAULT_INTEGRAL_OVERFLOW).  On fault the safe-state policy
 *         determines *u_out.
 *
 * Satisfies: RON-FR-001 – RON-FR-035, RON-FR-054, RON-FR-070.
 */
ron_fault_t ron_pid_core_step(ron_pid_instance_t *inst,
                              ron_float_t         r,
                              ron_float_t         y,
                              ron_float_t         dt,
                              ron_float_t        *u_out,
                              ron_status_t       *status);

#ifdef __cplusplus
}
#endif

#endif /* RON_PID_INTERNAL_H */
