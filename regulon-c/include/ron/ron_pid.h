/*
 * @file     ron_pid.h
 * @brief    Public API for the Regulon PID controller module.
 * @module   ron_pid_api
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-021, RON-FR-033, RON-FR-040, RON-FR-050,
 *           RON-FR-051, RON-FR-052, RON-FR-053, RON-FR-070, RON-FR-071,
 *           RON-SR-001, RON-SR-002, RON-SR-012
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * This is the ONLY header that library consumers shall include for PID
 * controller functionality.  It transitively includes ron_pid_types.h and
 * ron_platform.h.
 *
 * Typical usage (bare-metal, 1 kHz sample rate):
 *
 *   static ron_pid_instance_t speed_pid;
 *
 *   void control_init(void) {
 *       ron_pid_config_t cfg = {
 *           .Kp = 2.0F, .Ki = 0.5F, .Kd = 0.1F,
 *           .N  = 10.0F,
 *           .b  = 1.0F, .c = 0.0F,
 *           .u_min = -100.0F, .u_max = 100.0F,
 *           .du_max = 500.0F,
 *           .I_min = -50.0F, .I_max = 50.0F,
 *           .aw_mode = RON_AW_BACK_CALC, .T_aw = 0.1F,
 *           .integ_method = RON_INTEG_EULER,
 *           .deriv_mode   = RON_DERIV_ON_MEASUREMENT,
 *           .normalise    = false,
 *           .safe_policy  = RON_SAFE_HOLD_LAST,
 *           .I_overflow_thresh = 200.0F,
 *       };
 *       (void)ron_pid_init(&speed_pid, &cfg);
 *   }
 *
 *   void control_1kHz_isr(void) {
 *       ron_float_t  u;
 *       ron_status_t status;
 *       ron_fault_t  fault = ron_pid_step(&speed_pid,
 *                                          setpoint, measurement,
 *                                          0.001F, &u, &status);
 *       if (fault != RON_FAULT_NONE) { handle_fault(fault); }
 *       actuator_set(u);
 *   }
 */

#ifndef RON_PID_H
#define RON_PID_H

#include "ron/ron_pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise a PID controller instance.
 *
 * Validates the supplied configuration, copies it into the instance by
 * value, and zero-initialises all dynamic state.  The caller does not need
 * to keep cfg alive after this call returns.
 *
 * @param[in,out] inst  Pointer to caller-allocated instance.  Must not be NULL.
 * @param[in]     cfg   Pointer to configuration record.  Must not be NULL.
 *
 * @return  RON_FAULT_NONE             on success.
 * @return  RON_FAULT_NULL_POINTER     if inst or cfg is NULL.
 * @return  RON_FAULT_CONFIG_INVALID   if any configuration field is out of
 *                                     range or logically inconsistent.
 *
 * @pre   inst points to writable storage of sizeof(ron_pid_instance_t) bytes.
 * @post  inst->state.is_initialised == true  iff return == RON_FAULT_NONE.
 *
 * Satisfies: RON-FR-050, RON-SR-001, RON-SR-002.
 */
/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
ron_fault_t ron_pid_init(ron_pid_instance_t     *inst,
                         const ron_pid_config_t *cfg);

/**
 * @brief Reset the dynamic state of an initialised controller.
 *
 * Zeroes the integral accumulator, derivative history, filter states, and
 * output history.  Configuration parameters and operating mode are
 * preserved.  The fault register is also cleared.
 *
 * @param[in,out] inst  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE             on success.
 * @return  RON_FAULT_NULL_POINTER     if inst is NULL.
 * @return  RON_FAULT_CONFIG_INVALID   if inst is not yet initialised.
 *
 * Satisfies: RON-FR-051.
 */
/* Satisfies: RON-FR-051 | Test: RON-TC-PID-031 */
ron_fault_t ron_pid_reset(ron_pid_instance_t *inst);

/* =========================================================================
 * Runtime
 * ========================================================================= */

/**
 * @brief Execute one PID computation step.
 *
 * Primary runtime function.  Call once per sample period with the current
 * setpoint and process variable.  The computed, saturated, rate-limited
 * control variable is written to *u_out.
 *
 * If the instance is in a latched fault state (fault_code != RON_FAULT_NONE),
 * no computation is performed and the safe-state output is returned
 * immediately.  The caller must call ron_pid_fault_clear() to resume.
 *
 * @param[in,out] inst    Pointer to an initialised instance.
 * @param[in]     r       Setpoint (engineering units or normalised).
 * @param[in]     y       Process variable (engineering units).
 * @param[in]     dt      Sample period in seconds.  Must be > 0 and finite.
 * @param[out]    u_out   Receives the control output.  Must not be NULL.
 * @param[out]    status  Receives the status word.    Must not be NULL.
 *
 * @return  RON_FAULT_NONE  if the step completed without fault.
 * @return  Non-zero ron_fault_t bitmask if a fault was detected or latched.
 *
 * @pre   inst was successfully initialised by ron_pid_init().
 * @pre   dt > 0.0 and RON_ISFINITE(dt).
 * @post  *u_out ∈ [inst->config.u_min, inst->config.u_max]
 *        OR a fault is active and the safe-state policy governs *u_out.
 *
 * Satisfies: RON-FR-001 – RON-FR-007, RON-FR-020 – RON-FR-035,
 *            RON-FR-070, RON-SR-010 – RON-SR-013, RON-PR-001 – RON-PR-002.
 */
/* Satisfies: RON-FR-001 – RON-FR-035 | Test: RON-TC-PID-001 – RON-TC-PID-026 */
ron_fault_t ron_pid_step(ron_pid_instance_t *inst,
                         ron_float_t         r,
                         ron_float_t         y,
                         ron_float_t         dt,
                         ron_float_t        *u_out,
                         ron_status_t       *status);

/* =========================================================================
 * Configuration update (runtime, atomic)
 * ========================================================================= */

/**
 * @brief Atomically update all three PID gain parameters.
 *
 * Validates the new gains first; only applies them if all are valid.
 * No other configuration fields are modified.
 *
 * @param[in,out] inst  Pointer to an initialised instance.
 * @param[in]     Kp    New proportional gain.  Must be >= 0 and finite.
 * @param[in]     Ki    New integral gain.      Must be >= 0 and finite.
 * @param[in]     Kd    New derivative gain.    Must be >= 0 and finite.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_CONFIG_INVALID if any gain is negative, NaN, or Inf.
 *
 * Satisfies: RON-FR-053.
 */
/* Satisfies: RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_gains(ron_pid_instance_t *inst,
                               ron_float_t         Kp,
                               ron_float_t         Ki,
                               ron_float_t         Kd);

/**
 * @brief Atomically update the output saturation limits.
 *
 * @param[in,out] inst   Pointer to an initialised instance.
 * @param[in]     u_min  New minimum output.  Must be < u_max and finite.
 * @param[in]     u_max  New maximum output.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_CONFIG_INVALID if u_min >= u_max or either is non-finite.
 *
 * Satisfies: RON-FR-021, RON-FR-053.
 */
/* Satisfies: RON-FR-021, RON-FR-053 | Test: RON-TC-PID-016 */
ron_fault_t ron_pid_set_limits(ron_pid_instance_t *inst,
                                ron_float_t         u_min,
                                ron_float_t         u_max);

/**
 * @brief Update the derivative filter coefficient N.
 *
 * @param[in,out] inst  Pointer to an initialised instance.
 * @param[in]     N     New filter bandwidth multiplier.  0 disables.
 *                      Must be >= 0 and finite.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_CONFIG_INVALID if N < 0 or non-finite.
 *
 * Satisfies: RON-FR-006, RON-FR-053.
 */
/* Satisfies: RON-FR-006, RON-FR-053 | Test: RON-TC-PID-033 */
ron_fault_t ron_pid_set_filter(ron_pid_instance_t *inst,
                                ron_float_t         N);

/**
 * @brief Update the anti-windup scheme and back-calculation time constant.
 *
 * @param[in,out] inst   Pointer to an initialised instance.
 * @param[in]     mode   New anti-windup mode.
 * @param[in]     T_aw   Back-calculation time constant.  Must be > 0 when
 *                       mode == RON_AW_BACK_CALC.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_CONFIG_INVALID if mode==BACK_CALC and T_aw <= 0.
 *
 * Satisfies: RON-FR-033, RON-FR-053.
 */
/* Satisfies: RON-FR-033, RON-FR-053 | Test: RON-TC-PID-024 */
ron_fault_t ron_pid_set_antiwindup(ron_pid_instance_t *inst,
                                    ron_aw_mode_t       mode,
                                    ron_float_t         T_aw);

/* =========================================================================
 * State mutation
 * ========================================================================= */

/**
 * @brief Switch between automatic and manual operating modes.
 *
 * Performs bumpless transfer when switching from manual to automatic: the
 * integral accumulator is pre-loaded so the first automatic output matches
 * the last manual output.
 *
 * @param[in,out] inst        Pointer to an initialised instance.
 * @param[in]     mode        New operating mode.
 * @param[in]     manual_out  Desired output during manual mode, or the
 *                            handoff value when switching to automatic.
 *                            Must be finite.  Clamped to [u_min, u_max].
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if inst is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if manual_out is NaN or Inf.
 *
 * Satisfies: RON-FR-040 – RON-FR-042.
 */
/* Satisfies: RON-FR-040 – RON-FR-042 | Test: RON-TC-PID-027 – RON-TC-PID-029 */
ron_fault_t ron_pid_set_mode(ron_pid_instance_t *inst,
                              ron_op_mode_t       mode,
                              ron_float_t         manual_out);

/**
 * @brief Pre-load the integral accumulator (warm start).
 *
 * The supplied value is clamped to [I_min, I_max] before being stored.
 *
 * @param[in,out] inst   Pointer to an initialised instance.
 * @param[in]     value  Desired integral accumulator value.  Must be finite.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if inst is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if value is NaN or Inf.
 *
 * Satisfies: RON-FR-052.
 */
/* Satisfies: RON-FR-052 | Test: RON-TC-PID-032 */
ron_fault_t ron_pid_set_integral(ron_pid_instance_t *inst,
                                  ron_float_t         value);

/* =========================================================================
 * State inspection
 * ========================================================================= */

/**
 * @brief Read all observable internal state in a single call.
 *
 * Any output pointer that is NULL is silently skipped (no error).  This
 * allows the caller to request only the fields of interest.
 *
 * @param[in]  inst      Pointer to an initialised instance.
 * @param[out] integral  Receives the current integral accumulator.
 * @param[out] last_u    Receives the last saturated output.
 * @param[out] last_D    Receives the last filtered derivative value.
 * @param[out] status    Receives the status word from the last step.
 * @param[out] fault     Receives the current accumulated fault code.
 *
 * @return  RON_FAULT_NONE         on success.
 * @return  RON_FAULT_NULL_POINTER if inst is NULL.
 *
 * Satisfies: RON-FR-071, RON-QR-021.
 */
/* Satisfies: RON-FR-071 | Test: RON-TC-PID-039 */
ron_fault_t ron_pid_get_state(const ron_pid_instance_t *inst,
                               ron_float_t              *integral,
                               ron_float_t              *last_u,
                               ron_float_t              *last_D,
                               ron_status_t             *status,
                               ron_fault_t              *fault);

/* =========================================================================
 * Fault management
 * ========================================================================= */

/**
 * @brief Clear the fault register, allowing normal operation to resume.
 *
 * @note  This clears fault bits but does NOT reset dynamic state.  If the
 *        fault was caused by numerical instability, call ron_pid_reset()
 *        too before resuming automatic mode.
 *
 * @param[in,out] inst  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE         on success.
 * @return  RON_FAULT_NULL_POINTER if inst is NULL.
 *
 * Satisfies: RON-SR-012.
 */
/* Satisfies: RON-SR-012 | Test: RON-TC-SAFE-009 */
ron_fault_t ron_pid_fault_clear(ron_pid_instance_t *inst);

/* =========================================================================
 * Configuration validation (standalone utility)
 * ========================================================================= */

/**
 * @brief Validate a configuration record without applying it.
 *
 * Useful for pre-checking configuration before ron_pid_init(), or for
 * validating externally loaded parameters (e.g., from NVM) before use.
 *
 * @param[in] cfg  Pointer to the configuration to validate.  Must not be NULL.
 *
 * @return  RON_FAULT_NONE           if the configuration is valid.
 * @return  RON_FAULT_NULL_POINTER   if cfg is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if any field is out of range or
 *                                   logically inconsistent.
 *
 * Satisfies: RON-SR-001, RON-SR-002.
 */
/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001 */
ron_fault_t ron_pid_config_validate(const ron_pid_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* RON_PID_H */
