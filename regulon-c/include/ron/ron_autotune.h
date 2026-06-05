/*
 * @file     ron_autotune.h
 * @brief    Public API for the Regulon relay-feedback PID auto-tuner.
 * @module   ron_autotune
 * @doc      RON-IS-001
 * @req      RON-FR-800, RON-FR-801, RON-FR-802, RON-FR-803,
 *           RON-FR-804, RON-FR-805, RON-FR-806, RON-FR-807
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * The auto-tuner drives a plant with a hysteresis relay signal in place of the
 * normal PID output (RON-FR-800), provoking a sustained limit-cycle
 * oscillation.  Zero-crossing counting (no FFT — see SADS DD-15) yields the
 * ultimate period Tu, and the peak-to-peak excursion yields the ultimate gain
 * Ku = 4d / (pi * A) (RON-FR-802).  Standard tuning rules then derive parallel
 * PID gains (RON-FR-803).  The computed gains are applied to the target PID
 * ONLY on an explicit ron_autotune_apply() call (RON-FR-804); a fault or an
 * explicit ron_autotune_abort() restores the controller untouched
 * (RON-FR-807).
 *
 * The library NEVER allocates memory: the caller owns the ron_at_t instance
 * (typically a file-scope static) and runs the relay loop from its own control
 * task.  ron_autotune_step() is standalone and does not touch the PID; the PID
 * is referenced only at start/apply/abort to snapshot, apply, or restore gains.
 *
 * Typical usage:
 *
 *   static ron_pid_instance_t pid;
 *   static ron_at_t           at;
 *
 *   void tune_begin(void) {
 *       ron_at_config_t cfg = {
 *           .relay_amplitude = 0.5F, .hysteresis = 0.05F, .u_bias = 0.0F,
 *           .min_cycles = 5U, .timeout_s = 30.0F, .tuning_rule = RON_AT_RULE_ZN,
 *       };
 *       (void)ron_autotune_init(&at, &cfg);
 *       (void)ron_autotune_start(&at, &pid);
 *   }
 *
 *   void control_isr(void) {
 *       ron_float_t u;
 *       (void)ron_autotune_step(&at, setpoint, measurement, 0.001F, &u);
 *       actuator_set(u);
 *       if (at.state.done) { (void)ron_autotune_apply(&at, &pid); }
 *   }
 */

#ifndef RON_AUTOTUNE_H
#define RON_AUTOTUNE_H

#include "ron/ron_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Tuning-rule selection (RON-FR-803)
 * ========================================================================= */

/**
 * @brief Selectable tuning rule applied to the measured Ku / Tu.
 *
 * The numeric ordering is part of the public contract: it indexes the internal
 * rule-factor tables and MUST remain stable.
 *
 * Satisfies: RON-FR-803.
 */
/* Satisfies: RON-FR-803 | Test: RON-TC-AT-004 */
typedef enum {
    RON_AT_RULE_ZN      = 0, /**< Ziegler-Nichols (classic).                 */
    RON_AT_RULE_TL      = 1, /**< Tyreus-Luyben (robust, slow).              */
    RON_AT_RULE_SOME_OS = 2, /**< Some-overshoot.                            */
    RON_AT_RULE_NO_OS   = 3  /**< No-overshoot (conservative).               */
} ron_at_rule_t;

/* =========================================================================
 * Auto-tune lifecycle phase (SADS state machine)
 * ========================================================================= */

/**
 * @brief Auto-tune lifecycle phase, stored in ron_at_state_t.phase.
 *
 * Satisfies: RON-FR-800.
 */
/* Satisfies: RON-FR-800 | Test: RON-TC-AT-001 */
typedef enum {
    RON_AT_IDLE       = 0, /**< Initialised, not yet started.               */
    RON_AT_SETTLING   = 1, /**< Relay driving; awaiting first crossing.     */
    RON_AT_RELAY      = 2, /**< Oscillating; counting cycles.               */
    RON_AT_ESTIMATING = 3, /**< Computing Ku / Tu and gains.                */
    RON_AT_DONE       = 4, /**< Estimation complete; results valid.         */
    RON_AT_ABORTED    = 5  /**< Aborted (fault, timeout, or caller abort).  */
} ron_at_phase_t;

/* =========================================================================
 * Configuration structure (RON-FR-801)
 * ========================================================================= */

/**
 * @brief Relay auto-tuner configuration.
 *
 * Satisfies: RON-FR-801.
 */
/* Satisfies: RON-FR-801 | Test: RON-TC-AT-002 */
typedef struct {
    ron_float_t relay_amplitude; /**< Relay half-amplitude d. Must be > 0, finite.   */
    ron_float_t hysteresis;      /**< Switching band epsilon. Must be >= 0, finite.  */
    ron_float_t u_bias;          /**< Output bias the relay swings about. Finite.    */
    uint8_t min_cycles;          /**< Full cycles required before estimating. >= 1.  */
    ron_float_t timeout_s;       /**< Abort if not done within this time. > 0.       */
    ron_at_rule_t tuning_rule;   /**< Rule applied to Ku / Tu.                       */
} ron_at_config_t;

/* =========================================================================
 * State structure (dynamic, mutable)
 * ========================================================================= */

/**
 * @brief Dynamic auto-tuner state.
 *
 * Zero-initialised by ron_autotune_init().  The leading result fields and the
 * phase / done / aborted / is_initialised flags are the observable surface; the
 * remaining fields are internal oscillation-tracking bookkeeping and SHALL be
 * treated as opaque by callers.
 *
 * Satisfies: RON-FR-800, RON-FR-802, RON-FR-805.
 */
/* Satisfies: RON-FR-800, RON-FR-802 | Test: RON-TC-AT-001, RON-TC-AT-003 */
typedef struct {
    /* ── Observable results (valid once done == true) ─────────────────── */
    ron_float_t Ku;        /**< Estimated ultimate gain.                    */
    ron_float_t Tu;        /**< Estimated ultimate period (s).              */
    ron_float_t Kp_result; /**< Computed proportional gain.                 */
    ron_float_t Ki_result; /**< Computed integral gain.                     */
    ron_float_t Kd_result; /**< Computed derivative gain.                   */

    /* ── Observable status ────────────────────────────────────────────── */
    uint8_t phase;       /**< Current ron_at_phase_t value.                 */
    bool done;           /**< Estimation completed; results valid.          */
    bool aborted;        /**< Tuning was aborted.                           */
    bool is_initialised; /**< Guard: set by ron_autotune_init() only.       */

    /* ── Internal oscillation tracking (opaque) ───────────────────────── */
    ron_float_t u_relay_prev;     /**< Last relay output (hysteresis hold).  */
    ron_float_t elapsed_s;        /**< Time since start (timeout clock).     */
    ron_float_t time_since_cross; /**< Time since last zero crossing.        */
    ron_float_t half_period_sum;  /**< Sum of measured half-period durations.*/
    ron_float_t pv_min;           /**< Min PV seen while oscillating.        */
    ron_float_t pv_max;           /**< Max PV seen while oscillating.        */
    uint16_t half_period_count;   /**< Number of zero crossings timed.       */
    int8_t last_sign;             /**< Sign of last error (0 = unknown).     */

    /* ── Saved PID context for restore (opaque) ───────────────────────── */
    ron_float_t saved_Kp;     /**< PID Kp captured at start.                */
    ron_float_t saved_Ki;     /**< PID Ki captured at start.                */
    ron_float_t saved_Kd;     /**< PID Kd captured at start.                */
    ron_op_mode_t saved_mode; /**< PID operating mode captured at start.    */
} ron_at_state_t;

/* =========================================================================
 * Instance structure (the handle)
 * ========================================================================= */

/**
 * @brief Complete auto-tuner instance.
 *
 * The caller allocates one of these (typically as a file-scope static).  The
 * library NEVER allocates memory.
 *
 * Satisfies: RON-FR-800.
 */
/* Satisfies: RON-FR-800 | Test: RON-TC-AT-001 */
typedef struct {
    ron_at_config_t cfg;  /**< Configuration (constant during a run).      */
    ron_at_state_t state; /**< Dynamic computation state.                  */
} ron_at_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise an auto-tuner instance.
 *
 * Validates the configuration, copies it into the instance, and zeroes all
 * dynamic state (phase becomes RON_AT_IDLE).
 *
 * @param[in,out] at   Pointer to caller-allocated instance.  Must not be NULL.
 * @param[in]     cfg  Pointer to configuration record.       Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at or cfg is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if any configuration field is invalid.
 *
 * @post  at->state.is_initialised == true iff return == RON_FAULT_NONE.
 *
 * Satisfies: RON-FR-800, RON-FR-801.
 */
/* Satisfies: RON-FR-800, RON-FR-801 | Test: RON-TC-AT-001, RON-TC-AT-002 */
ron_fault_t ron_autotune_init(ron_at_t *at, const ron_at_config_t *cfg);

/**
 * @brief Begin a relay-feedback tuning run against a PID instance.
 *
 * Snapshots the PID's current gains and operating mode for later restore, then
 * switches the PID to manual mode so it does not fight the relay.  The PID
 * gains are NOT modified (RON-FR-804).
 *
 * @param[in,out] at   Pointer to an initialised instance.
 * @param[in,out] pid  Target PID instance.  Must be initialised.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at or pid is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if at or pid is not initialised.
 *
 * Satisfies: RON-FR-800, RON-FR-804.
 */
/* Satisfies: RON-FR-800, RON-FR-804 | Test: RON-TC-AT-001, RON-TC-AT-005 */
ron_fault_t ron_autotune_start(ron_at_t *at, ron_pid_instance_t *pid);

/* =========================================================================
 * Runtime
 * ========================================================================= */

/**
 * @brief Execute one relay-feedback step.
 *
 * Computes the relay output for the supplied setpoint r and measurement y,
 * advances oscillation detection, and (once min_cycles full cycles have been
 * observed) estimates Ku / Tu and the tuned gains.  The returned output always
 * lies in [u_bias - d, u_bias + d] (RON-FR-806).  Exceeding timeout_s, or an
 * oscillation too small to measure, transitions the run to RON_AT_ABORTED.
 *
 * Once the run has reached RON_AT_DONE or RON_AT_ABORTED, further calls return
 * the bias output and leave the results unchanged.
 *
 * @param[in,out] at     Pointer to a started instance.
 * @param[in]     r      Setpoint / reference (engineering units).
 * @param[in]     y      Process variable (engineering units).
 * @param[in]     dt     Sample period in seconds.  Must be > 0 and finite.
 * @param[out]    u_out  Receives the relay control output.  Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at or u_out is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if not started, dt invalid, or r/y non-finite.
 *
 * Satisfies: RON-FR-800, RON-FR-802, RON-FR-806.
 */
/* Satisfies: RON-FR-800, RON-FR-802, RON-FR-806 | Test: RON-TC-AT-003, RON-TC-AT-007 */
ron_fault_t ron_autotune_step(ron_at_t *at, ron_float_t r, ron_float_t y, ron_float_t dt,
                              ron_float_t *u_out);

/* =========================================================================
 * Staged apply / abort
 * ========================================================================= */

/**
 * @brief Apply the computed gains to the target PID.
 *
 * Permitted only after the run has reached RON_AT_DONE.  Writes
 * (Kp_result, Ki_result, Kd_result) via ron_pid_set_gains() and restores the
 * PID operating mode captured at start.  This is the ONLY path that modifies
 * the PID gains (RON-FR-804).
 *
 * @param[in,out] at   Pointer to a completed instance.
 * @param[in,out] pid  Target PID instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at or pid is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if the run is not in the done state.
 *
 * Satisfies: RON-FR-804.
 */
/* Satisfies: RON-FR-804 | Test: RON-TC-AT-005 */
ron_fault_t ron_autotune_apply(ron_at_t *at, ron_pid_instance_t *pid);

/**
 * @brief Abort the tuning run and restore the PID untouched.
 *
 * Restores the gains and operating mode captured at start and marks the run as
 * aborted.  Safe to call in any phase after start.
 *
 * @param[in,out] at   Pointer to a started instance.
 * @param[in,out] pid  Target PID instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at or pid is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if at is not initialised.
 *
 * Satisfies: RON-FR-807.
 */
/* Satisfies: RON-FR-807 | Test: RON-TC-AT-008 */
ron_fault_t ron_autotune_abort(ron_at_t *at, ron_pid_instance_t *pid);

/* =========================================================================
 * Results
 * ========================================================================= */

/**
 * @brief Read the raw Ku / Tu and computed gains.
 *
 * Exposes the ultimate gain and period so callers may apply custom tuning
 * rules (RON-FR-805).  Any output pointer that is NULL is silently skipped.
 *
 * @param[in]  at  Pointer to a completed instance.
 * @param[out] Ku  Receives the ultimate gain.   May be NULL.
 * @param[out] Tu  Receives the ultimate period. May be NULL.
 * @param[out] Kp  Receives the computed Kp.     May be NULL.
 * @param[out] Ki  Receives the computed Ki.     May be NULL.
 * @param[out] Kd  Receives the computed Kd.     May be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if at is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if the run is not in the done state.
 *
 * Satisfies: RON-FR-805.
 */
/* Satisfies: RON-FR-805 | Test: RON-TC-AT-006 */
ron_fault_t ron_autotune_results(const ron_at_t *at, ron_float_t *Ku, ron_float_t *Tu,
                                 ron_float_t *Kp, ron_float_t *Ki, ron_float_t *Kd);

#ifdef __cplusplus
}
#endif

#endif /* RON_AUTOTUNE_H */
