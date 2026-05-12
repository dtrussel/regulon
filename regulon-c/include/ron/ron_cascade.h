/*
 * @file     ron_cascade.h
 * @brief    Public API for the Regulon cascade (master/slave) PID controller.
 * @module   ron_cascade
 * @doc      RON-IS-001
 * @req      RON-FR-400, RON-FR-401, RON-FR-402, RON-FR-403,
 *           RON-FR-404, RON-FR-405, RON-FR-406
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * A cascade controller composes two independent ron_pid_instance_t loops:
 * the outer (master) loop computes a setpoint that drives the inner (slave)
 * loop.  The cascade layer adds coordinated anti-windup propagation, mode
 * transitions, and a unified status word.
 *
 * Typical usage:
 *
 *   static ron_cascade_instance_t casc;
 *
 *   void init(void) {
 *       ron_pid_config_t outer = { .Kp = 1.0F, .Ki = 0.5F,
 *                                  .u_min = -10.0F, .u_max = 10.0F, ... };
 *       ron_pid_config_t inner = { .Kp = 5.0F, .Ki = 2.0F,
 *                                  .u_min = -100.0F, .u_max = 100.0F, ... };
 *       (void)ron_cascade_init(&casc, &outer, &inner);
 *   }
 *
 *   void isr_1kHz(void) {
 *       ron_float_t u;
 *       ron_cascade_status_t status;
 *       (void)ron_cascade_step(&casc, sp_outer, pv_outer, pv_inner,
 *                              0.001F, &u, &status);
 *       actuator_set(u);
 *   }
 */

#ifndef RON_CASCADE_H
#define RON_CASCADE_H

#include "ron/ron_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Unified cascade status type (RON-FR-406)
 * ========================================================================= */

/**
 * @brief Unified cascade status word.
 *
 * Bits 0–15  carry the outer loop's ron_status_t verbatim.
 * Bits 16–31 carry the inner loop's ron_status_t shifted left by 16.
 *
 * Use RON_CASCADE_STATUS_OUTER() and RON_CASCADE_STATUS_INNER() to extract
 * per-loop status words for individual inspection.
 *
 * Satisfies: RON-FR-406.
 */
/* Satisfies: RON-FR-406 | Test: RON-TC-CASC-005, RON-TC-CASC-010 */
typedef uint32_t ron_cascade_status_t;

#define RON_CASCADE_STATUS_OUTER_SHIFT ((uint32_t) 0U)
#define RON_CASCADE_STATUS_INNER_SHIFT ((uint32_t) 16U)
#define RON_CASCADE_STATUS_OUTER_MASK ((ron_cascade_status_t) 0x0000FFFFU)
#define RON_CASCADE_STATUS_INNER_MASK ((ron_cascade_status_t) 0xFFFF0000U)

/** @brief Extract the outer loop's ron_status_t from a cascade status word. */
#define RON_CASCADE_STATUS_OUTER(cs) ((ron_status_t) ((cs) & RON_CASCADE_STATUS_OUTER_MASK))

/** @brief Extract the inner loop's ron_status_t from a cascade status word. */
#define RON_CASCADE_STATUS_INNER(cs)                                                               \
    ((ron_status_t) (((cs) & RON_CASCADE_STATUS_INNER_MASK) >> RON_CASCADE_STATUS_INNER_SHIFT))

/* =========================================================================
 * Cascade instance structure (RON-FR-400)
 * ========================================================================= */

/**
 * @brief Complete cascade controller instance.
 *
 * The caller allocates one of these (typically as a file-scope static
 * variable or as a field in a task-control block).  The library NEVER
 * allocates memory.
 *
 * The outer loop's output is automatically used as the inner loop's setpoint
 * on each call to ron_cascade_step().  The outer output is already clamped to
 * [outer.u_min, outer.u_max] by ron_pid_step() before reaching the inner loop,
 * satisfying RON-FR-402.
 *
 * @example
 * @code
 *   static ron_cascade_instance_t cascade;
 *   ron_fault_t f = ron_cascade_init(&cascade, &outer_cfg, &inner_cfg);
 * @endcode
 *
 * Satisfies: RON-FR-400.
 */
/* Satisfies: RON-FR-400 | Test: RON-TC-CASC-001 */
typedef struct {
    ron_pid_instance_t outer; /**< Outer (master) loop.  Output becomes inner setpoint. */
    ron_pid_instance_t inner; /**< Inner (slave) loop.   Receives outer output as r.    */
} ron_cascade_instance_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise a cascade controller instance.
 *
 * Validates both configurations independently then calls ron_pid_init() for
 * each loop.  The outer loop's [u_min, u_max] defines the legal range for the
 * inner setpoint (RON-FR-402); the caller is responsible for configuring these
 * to match the inner loop's meaningful operating range.
 *
 * @param[in,out] casc       Pointer to caller-allocated instance.  Must not be NULL.
 * @param[in]     outer_cfg  Configuration for the outer (master) loop.  Must not be NULL.
 * @param[in]     inner_cfg  Configuration for the inner (slave) loop.   Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if any pointer is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if either configuration is invalid.
 *
 * @post  casc->outer.state.is_initialised == true  iff return == RON_FAULT_NONE.
 * @post  casc->inner.state.is_initialised == true  iff return == RON_FAULT_NONE.
 *
 * Satisfies: RON-FR-400, RON-FR-402, RON-FR-405.
 */
/* Satisfies: RON-FR-400, RON-FR-402, RON-FR-405 | Test: RON-TC-CASC-001, RON-TC-CASC-002 */
ron_fault_t ron_cascade_init(ron_cascade_instance_t *casc, const ron_pid_config_t *outer_cfg,
                             const ron_pid_config_t *inner_cfg);

/* =========================================================================
 * Runtime
 * ========================================================================= */

/**
 * @brief Execute one cascade computation step.
 *
 * Sequence:
 *   1. Run outer PID: r_out → y_out → u_outer (clamped to outer [u_min, u_max]).
 *   2. Pass u_outer directly as the inner setpoint r_inner (RON-FR-401).
 *   3. Run inner PID: r_inner → y_in → u_inner.
 *   4. If inner saturated and outer configured for RON_AW_BACK_CALC, apply
 *      cross-loop back-calculation correction to the outer integral (RON-FR-403).
 *   5. Write u_inner to *u_out and assemble the unified status word (RON-FR-406).
 *
 * If either loop is in a latched fault state, that loop returns its safe-state
 * output.  Both fault codes are OR-ed into the return value.
 *
 * @param[in,out] casc    Pointer to an initialised instance.
 * @param[in]     r_out   Outer loop setpoint (engineering units).
 * @param[in]     y_out   Outer loop process variable.
 * @param[in]     y_in    Inner loop process variable.
 * @param[in]     dt      Sample period in seconds.  Must be > 0 and finite.
 * @param[out]    u_out   Receives the inner loop (final) control output.  Must not be NULL.
 * @param[out]    status  Receives the unified cascade status word.           Must not be NULL.
 *
 * @return  RON_FAULT_NONE  if both steps completed without new fault.
 * @return  Non-zero OR of both loops' fault codes if any fault was detected.
 *
 * @post  *u_out is the inner loop's saturated output.
 *
 * Satisfies: RON-FR-401, RON-FR-402, RON-FR-403, RON-FR-405, RON-FR-406.
 */
/* Satisfies: RON-FR-401 – RON-FR-403, RON-FR-406 | Test: RON-TC-CASC-003 – RON-TC-CASC-009 */
ron_fault_t ron_cascade_step(ron_cascade_instance_t *casc, ron_float_t r_out, ron_float_t y_out,
                             ron_float_t y_in, ron_float_t dt, ron_float_t *u_out,
                             ron_cascade_status_t *status);

/* =========================================================================
 * Mode transition
 * ========================================================================= */

/**
 * @brief Coordinated mode transition across both loops.
 *
 * Transition order for bumpless transfer (RON-FR-404):
 *   AUTO → MANUAL : outer loop first, inner loop second.
 *   MANUAL → AUTO : inner loop first (bumpless integral pre-load),
 *                   outer loop second (bumpless integral pre-load).
 *
 * ron_pid_set_mode() with RON_MODE_AUTOMATIC pre-loads each loop's integral
 * accumulator so its first automatic output closely matches the supplied
 * manual handoff value.
 *
 * @param[in,out] casc          Pointer to an initialised instance.
 * @param[in]     mode          Target operating mode for both loops.
 * @param[in]     manual_inner  Manual output / bumpless handoff for the inner loop.
 *                              Must be finite and will be clamped to inner [u_min, u_max].
 * @param[in]     manual_outer  Manual output / bumpless handoff for the outer loop.
 *                              Must be finite and will be clamped to outer [u_min, u_max].
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if casc is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if either manual value is NaN or Inf, or
 *                                   if the instance is not yet initialised.
 *
 * Satisfies: RON-FR-404.
 */
/* Satisfies: RON-FR-404 | Test: RON-TC-CASC-007, RON-TC-CASC-008 */
ron_fault_t ron_cascade_set_mode(ron_cascade_instance_t *casc, ron_op_mode_t mode,
                                 ron_float_t manual_inner, ron_float_t manual_outer);

/* =========================================================================
 * State inspection
 * ========================================================================= */

/**
 * @brief Read the unified status word and per-loop fault codes.
 *
 * Any output pointer that is NULL is silently skipped (no error), allowing
 * the caller to request only the fields of interest.
 *
 * @param[in]  casc         Pointer to an initialised instance.
 * @param[out] status       Receives the combined cascade status word.  May be NULL.
 * @param[out] outer_fault  Receives the outer loop fault code.          May be NULL.
 * @param[out] inner_fault  Receives the inner loop fault code.          May be NULL.
 *
 * @return  RON_FAULT_NONE         on success.
 * @return  RON_FAULT_NULL_POINTER if casc is NULL.
 *
 * Satisfies: RON-FR-406.
 */
/* Satisfies: RON-FR-406 | Test: RON-TC-CASC-010 */
ron_fault_t ron_cascade_get_state(const ron_cascade_instance_t *casc, ron_cascade_status_t *status,
                                  ron_fault_t *outer_fault, ron_fault_t *inner_fault);

/* =========================================================================
 * Fault management
 * ========================================================================= */

/**
 * @brief Clear the fault registers on both loops.
 *
 * @note  Clears fault bits but does NOT reset dynamic state.  If a fault was
 *        caused by numerical instability, call ron_cascade_reset() too before
 *        resuming automatic mode.
 *
 * @param[in,out] casc  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if casc is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if the instance is not yet initialised.
 *
 * Satisfies: RON-FR-405.
 */
/* Satisfies: RON-FR-405 | Test: RON-TC-CASC-011 */
ron_fault_t ron_cascade_fault_clear(ron_cascade_instance_t *casc);

/**
 * @brief Reset the dynamic state on both loops.
 *
 * Zeroes the integral accumulators, derivative histories, filter states, and
 * output histories in both loops.  Configuration parameters and operating
 * modes are preserved.  Fault registers are also cleared.
 *
 * @param[in,out] casc  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if casc is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if the instance is not yet initialised.
 *
 * Satisfies: RON-FR-405.
 */
/* Satisfies: RON-FR-405 | Test: RON-TC-CASC-012 */
ron_fault_t ron_cascade_reset(ron_cascade_instance_t *casc);

#ifdef __cplusplus
}
#endif

#endif /* RON_CASCADE_H */
