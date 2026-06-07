/*
 * @file     ron_metrics.h
 * @brief    Public API for the Regulon runtime performance metrics accumulator.
 * @module   ron_metrics
 * @doc      RON-IS-001
 * @req      RON-FR-950, RON-FR-951, RON-FR-952, RON-FR-953, RON-FR-954
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * The metrics accumulator is a passive observer that attaches to any controller
 * and quantifies closed-loop quality once per control step from the setpoint r
 * and the process variable y (RON-FR-950).  It computes (RON-FR-951):
 *
 *   IAE             — integral of |e| dt.
 *   ISE             — integral of e^2 dt.
 *   ITAE            — integral of t |e| dt (time-weighted).
 *   peak_overshoot  — peak overshoot beyond the target, in percent of the step.
 *   rise_time       — time to traverse 10 % -> 90 % of the step (s).
 *   settling_time   — time to enter and stay within the settling band (s).
 *
 * The accumulator supports cumulative and windowed (rolling) modes
 * (RON-FR-952), is enable/disable at runtime with zero overhead when disabled
 * (RON-FR-953), and automatically restarts the transient metrics (overshoot,
 * rise, settling) whenever it detects a setpoint step (RON-FR-954).  The module
 * NEVER modifies the controller and NEVER allocates memory: it only reads
 * (r, y, dt) and updates its own caller-owned ron_metrics_t instance.
 *
 * Typical usage:
 *
 *   static ron_metrics_t met;
 *
 *   void setup(void) {
 *       ron_metrics_config_t cfg = {
 *           .mode = RON_METRICS_CUMULATIVE, .window_steps = 0U,
 *           .band_pct = 0.02F, .settle_confirm = 0.10F, .step_thresh = 0.05F,
 *       };
 *       (void)ron_metrics_init(&met, &cfg);
 *       (void)ron_metrics_enable(&met, true);   // off by default
 *   }
 *
 *   void control_isr(void) {
 *       (void)ron_metrics_step(&met, sp, pv, 0.001F);
 *   }
 */

#ifndef RON_METRICS_H
#define RON_METRICS_H

#include "ron/ron_pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Accumulation mode (RON-FR-952)
 * ========================================================================= */

/**
 * @brief Accumulator mode selection.
 *
 * Satisfies: RON-FR-952.
 */
/* Satisfies: RON-FR-952 | Test: RON-TC-MET-005 */
typedef enum {
    RON_METRICS_CUMULATIVE = 0, /**< Integrals accumulate for the whole run.   */
    RON_METRICS_WINDOWED   = 1  /**< Integrals reset every window_steps samples.*/
} ron_metrics_mode_t;

/* =========================================================================
 * Configuration structure
 * ========================================================================= */

/**
 * @brief Metrics-accumulator configuration.
 *
 * Satisfies: RON-FR-950, RON-FR-951, RON-FR-952, RON-FR-954.
 */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
typedef struct {
    ron_metrics_mode_t mode;    /**< Cumulative or windowed accumulation.            */
    uint32_t window_steps;      /**< Window length in samples (used if WINDOWED). >0.*/
    ron_float_t band_pct;       /**< Settling band as a fraction, e.g. 0.02 = 2 %. >0.*/
    ron_float_t settle_confirm; /**< Dwell within band to confirm settling (s). >=0.*/
    ron_float_t step_thresh;    /**< |Δr| that marks a new setpoint step. > 0.     */
} ron_metrics_config_t;

/* =========================================================================
 * Result structure (read-only snapshot)
 * ========================================================================= */

/**
 * @brief Snapshot of the computed metrics.
 *
 * rise_time and settling_time are RON_FLOAT_C(-1.0) until the corresponding
 * event has been observed.
 *
 * Satisfies: RON-FR-951.
 */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-002, RON-TC-MET-003, RON-TC-MET-004 */
typedef struct {
    ron_float_t IAE;            /**< Integral of |e| dt.                          */
    ron_float_t ISE;            /**< Integral of e^2 dt.                          */
    ron_float_t ITAE;           /**< Integral of t |e| dt.                        */
    ron_float_t peak_overshoot; /**< Peak overshoot (% of step size).            */
    ron_float_t rise_time;      /**< 10 %->90 % rise time (s), -1 if not reached. */
    ron_float_t settling_time;  /**< Settling time (s), -1 if not settled.        */
} ron_metrics_result_t;

/* =========================================================================
 * Instance structure (the handle)
 * ========================================================================= */

/**
 * @brief Complete metrics-accumulator instance.
 *
 * The caller allocates one of these (typically as a file-scope static).  The
 * library NEVER allocates memory.  All fields except cfg are internal
 * bookkeeping and SHALL be treated as opaque by callers; read the published
 * metrics only through ron_metrics_get().
 *
 * Satisfies: RON-FR-950, RON-FR-953.
 */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
typedef struct {
    ron_metrics_config_t cfg; /**< Configuration (constant during a run).         */

    /* ── Error integrals (RON-FR-951) ─────────────────────────────────── */
    ron_float_t iae;  /**< Running integral of |e| dt.                          */
    ron_float_t ise;  /**< Running integral of e^2 dt.                          */
    ron_float_t itae; /**< Running integral of t |e| dt.                        */

    /* ── Transient metrics (RON-FR-951, RON-FR-954) ───────────────────── */
    ron_float_t peak_overshoot; /**< Peak overshoot (% of step).                */
    ron_float_t rise_time;      /**< Latched rise time (s), -1 until reached.    */
    ron_float_t settling_time;  /**< Latched settling time (s), -1 until reached.*/

    /* ── Step-response reference frame (RON-FR-954) ───────────────────── */
    ron_float_t step_ref;    /**< Process variable captured at the step.          */
    ron_float_t step_target; /**< Setpoint captured at the step.                  */
    ron_float_t step_size;   /**< Signed step magnitude (step_target - step_ref). */

    /* ── Timing / detector bookkeeping (opaque) ───────────────────────── */
    ron_float_t t_elapsed;    /**< Time since the last step / window restart (s). */
    ron_float_t t_rise_start; /**< Elapsed time when the 10 % level was crossed.  */
    ron_float_t in_band_time; /**< Contiguous dwell time within the band (s).     */
    ron_float_t r_prev;       /**< Previous setpoint (for step detection).        */
    uint32_t window_counter;  /**< Samples accumulated in the current window.     */
    bool rise_10pct_crossed;  /**< True once the 10 % level has been crossed.     */
    bool prev_valid;          /**< False until the first step has been observed.  */

    /* ── Lifecycle flags ──────────────────────────────────────────────── */
    bool enabled;        /**< Runtime collection switch (off by default).         */
    bool is_initialised; /**< Guard: set by ron_metrics_init() only.              */
} ron_metrics_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise a metrics-accumulator instance.
 *
 * Validates the configuration, copies it into the instance, zeroes all running
 * state, and leaves the accumulator DISABLED (RON-FR-953).
 *
 * @param[in,out] m    Pointer to caller-allocated instance.  Must not be NULL.
 * @param[in]     cfg  Pointer to configuration record.       Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if m or cfg is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if any configuration field is invalid.
 *
 * @post  m->is_initialised == true and m->enabled == false on success.
 *
 * Satisfies: RON-FR-950, RON-FR-953.
 */
/* Satisfies: RON-FR-950, RON-FR-953 | Test: RON-TC-MET-001 */
ron_fault_t ron_metrics_init(ron_metrics_t *m, const ron_metrics_config_t *cfg);

/**
 * @brief Reset all running metrics, preserving configuration and enable state.
 *
 * @param[in,out] m  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if m is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if m is not initialised.
 *
 * Satisfies: RON-FR-950.
 */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
ron_fault_t ron_metrics_reset(ron_metrics_t *m);

/**
 * @brief Enable or disable runtime metrics collection.
 *
 * While disabled, ron_metrics_step() performs no work and changes no state
 * (RON-FR-953).
 *
 * @param[in,out] m       Pointer to an initialised instance.
 * @param[in]     enable  true to collect metrics, false to suspend.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if m is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if m is not initialised.
 *
 * Satisfies: RON-FR-953.
 */
/* Satisfies: RON-FR-953 | Test: RON-TC-MET-006 */
ron_fault_t ron_metrics_enable(ron_metrics_t *m, bool enable);

/* =========================================================================
 * Runtime
 * ========================================================================= */

/**
 * @brief Accumulate the metrics for one control step.
 *
 * Reads the setpoint r and process variable y, advances the error integrals,
 * and updates the transient metrics.  A setpoint step (|Δr| >= step_thresh)
 * restarts the transient metrics from that event (RON-FR-954).  The call is
 * passive: it never modifies the controller.  When the accumulator is disabled
 * the call returns immediately without touching any state (RON-FR-953).
 *
 * @param[in,out] m   Pointer to an initialised instance.
 * @param[in]     r   Setpoint / reference (engineering units).  Finite.
 * @param[in]     y   Process variable (engineering units).      Finite.
 * @param[in]     dt  Sample period in seconds.  Must be > 0 and finite.
 *
 * @return  RON_FAULT_NONE           on success (including the disabled no-op).
 * @return  RON_FAULT_NULL_POINTER   if m is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if not initialised, dt invalid, or r/y non-finite.
 *
 * Satisfies: RON-FR-951, RON-FR-952, RON-FR-953, RON-FR-954.
 */
/* Satisfies: RON-FR-951, RON-FR-954 | Test: RON-TC-MET-002 – RON-TC-MET-007 */
ron_fault_t ron_metrics_step(ron_metrics_t *m, ron_float_t r, ron_float_t y, ron_float_t dt);

/**
 * @brief Read the current metrics snapshot.
 *
 * @param[in]  m    Pointer to an initialised instance.
 * @param[out] out  Receives the computed metrics.  Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if m or out is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if m is not initialised.
 *
 * Satisfies: RON-FR-951.
 */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-002 */
ron_fault_t ron_metrics_get(const ron_metrics_t *m, ron_metrics_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RON_METRICS_H */
