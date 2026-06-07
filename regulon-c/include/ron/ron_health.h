/*
 * @file     ron_health.h
 * @brief    Public API for the Regulon control-loop health monitor.
 * @module   ron_health
 * @doc      RON-IS-001
 * @req      RON-FR-900, RON-FR-901, RON-FR-902,
 *           RON-FR-903, RON-FR-904, RON-FR-905
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * The health monitor is a passive observer that attaches to any controller and
 * evaluates loop health once per control step from the setpoint r, the process
 * variable y, and the commanded output u (RON-FR-900).  It reports five
 * independent conditions through a latched bitmask (RON-FR-901):
 *
 *   OUTPUT_STUCK    — the output has not moved for longer than t_sat_max.
 *   DIVERGING       — the error is large and still growing in magnitude.
 *   OSCILLATING     — error sign changes exceed osc_count_thresh in the window.
 *   SENSOR_DROPOUT  — the measurement has not moved beyond dead_band.
 *   SP_UNREACHABLE  — a steady-state error persists past settling_time.
 *
 * Each condition has its own thresholds and time constants (RON-FR-902).  The
 * monitor NEVER modifies the controller output, state, or configuration: it
 * only reads (r, y, u, dt) and updates its own instance (RON-FR-903, SADS
 * DD-16).  An optional callback fires the first time each condition becomes
 * active (RON-FR-904), and every condition latches until ron_health_clear()
 * (RON-FR-905).  The library NEVER allocates memory; the caller owns the
 * ron_health_t instance (typically a file-scope static).
 *
 * Typical usage:
 *
 *   static ron_health_t mon;
 *
 *   void setup(void) {
 *       ron_health_config_t cfg = {
 *           .t_sat_max = 0.5F, .err_diverge_thresh = 10.0F,
 *           .osc_count_thresh = 6U, .dead_band = 1.0e-3F,
 *           .dropout_time = 1.0F, .ss_err_thresh = 0.5F,
 *           .settling_time = 5.0F, .cb = on_health_event,
 *       };
 *       (void)ron_health_init(&mon, &cfg);
 *   }
 *
 *   void control_isr(void) {
 *       ron_float_t u = pid_output();        // computed elsewhere
 *       (void)ron_health_step(&mon, sp, pv, u, 0.001F);
 *   }
 */

#ifndef RON_HEALTH_H
#define RON_HEALTH_H

#include "ron/ron_pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Health status bitmask (RON-FR-901)
 * ========================================================================= */

/**
 * @brief Loop-health condition bitmask.
 *
 * Each detected condition sets one bit; the OK value (no bits) means healthy.
 *
 * Satisfies: RON-FR-901.
 */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-002 – RON-TC-HLTH-006 */
typedef uint8_t ron_health_status_t;
#define RON_HEALTH_OK ((ron_health_status_t) 0x00U)             /**< Healthy.                  */
#define RON_HEALTH_OUTPUT_STUCK ((ron_health_status_t) 0x01U)   /**< Output not moving.        */
#define RON_HEALTH_DIVERGING ((ron_health_status_t) 0x02U)      /**< Error large and growing.  */
#define RON_HEALTH_OSCILLATING ((ron_health_status_t) 0x04U)    /**< Error sign-changing.      */
#define RON_HEALTH_SENSOR_DROPOUT ((ron_health_status_t) 0x08U) /**< Measurement not moving.   */
#define RON_HEALTH_SP_UNREACHABLE ((ron_health_status_t) 0x10U) /**< Steady-state error.       */

/**
 * @brief Callback invoked the first time a condition becomes active.
 *
 * @param[in] condition  The single status bit that just became active.
 *
 * Satisfies: RON-FR-904.
 */
/* Satisfies: RON-FR-904 | Test: RON-TC-HLTH-009 */
typedef void (*ron_health_cb_t)(ron_health_status_t condition);

/* =========================================================================
 * Configuration structure (RON-FR-902)
 * ========================================================================= */

/**
 * @brief Health-monitor configuration.
 *
 * Every condition has its own threshold and time constant so the conditions are
 * independently tunable (RON-FR-902).  cb may be NULL to disable callbacks.
 *
 * Satisfies: RON-FR-902.
 */
/* Satisfies: RON-FR-902 | Test: RON-TC-HLTH-007 */
typedef struct {
    ron_float_t t_sat_max;          /**< Stuck duration threshold (s). > 0, finite.        */
    ron_float_t err_diverge_thresh; /**< Divergence error magnitude. >= 0, finite.         */
    uint8_t osc_count_thresh;       /**< Sign changes in window to trip. < window length.  */
    ron_float_t dead_band;          /**< Measurement dead-band for dropout. >= 0, finite.  */
    ron_float_t dropout_time;       /**< Dropout duration threshold (s). > 0, finite.      */
    ron_float_t ss_err_thresh;      /**< Steady-state error magnitude. >= 0, finite.       */
    ron_float_t settling_time;      /**< Settling time budget (s). > 0, finite.            */
    ron_health_cb_t cb;             /**< First-activation callback. May be NULL.           */
} ron_health_config_t;

/* =========================================================================
 * State structure (dynamic, mutable)
 * ========================================================================= */

/**
 * @brief Dynamic health-monitor state.
 *
 * Zero-initialised by ron_health_init().  status is the observable surface; the
 * remaining fields are internal sliding-window / counter bookkeeping and SHALL
 * be treated as opaque by callers.  u_prev and prev_valid extend the
 * IS-enumerated set with the opaque previous-output reference and first-step
 * guard required by the output-stuck comparator.
 *
 * Satisfies: RON-FR-901, RON-FR-905.
 */
/* Satisfies: RON-FR-901, RON-FR-905 | Test: RON-TC-HLTH-010 */
typedef struct {
    /* ── Observable status (latched bitmask) ──────────────────────────── */
    ron_health_status_t status; /**< Latched condition bitmask.            */

    /* ── Internal condition bookkeeping (opaque) ──────────────────────── */
    ron_float_t t_saturated;                   /**< Time the output has been stuck (s).     */
    ron_float_t t_dropout;                     /**< Time the measurement has been still (s).*/
    ron_float_t t_since_step;                  /**< Time since the last setpoint step (s).  */
    uint8_t osc_window[RON_HEALTH_OSC_WINDOW]; /**< Ring of error signs.   */
    uint8_t osc_idx;                           /**< Next write index into osc_window.       */
    ron_float_t e_prev;                        /**< Previous error (r - y).                 */
    ron_float_t y_prev;                        /**< Previous measurement.                   */
    bool is_initialised;                       /**< Guard: set by ron_health_init() only.   */

    /* ── Opaque extensions beyond the IS field set ────────────────────── */
    ron_float_t u_prev; /**< Previous commanded output (stuck reference).  */
    bool prev_valid;    /**< False until the first step has been observed. */
} ron_health_state_t;

/* =========================================================================
 * Instance structure (the handle)
 * ========================================================================= */

/**
 * @brief Complete health-monitor instance.
 *
 * The caller allocates one of these (typically as a file-scope static).  The
 * library NEVER allocates memory.
 *
 * Satisfies: RON-FR-900.
 */
/* Satisfies: RON-FR-900 | Test: RON-TC-HLTH-001 */
typedef struct {
    ron_health_config_t cfg;  /**< Configuration (constant during a run).   */
    ron_health_state_t state; /**< Dynamic detection state.                */
} ron_health_t;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

/**
 * @brief Initialise a health-monitor instance.
 *
 * Validates the configuration, copies it into the instance, and zeroes all
 * dynamic state (status becomes RON_HEALTH_OK).
 *
 * @param[in,out] h    Pointer to caller-allocated instance.  Must not be NULL.
 * @param[in]     cfg  Pointer to configuration record.       Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if h or cfg is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if any configuration field is invalid.
 *
 * @post  h->state.is_initialised == true iff return == RON_FAULT_NONE.
 *
 * Satisfies: RON-FR-900, RON-FR-902.
 */
/* Satisfies: RON-FR-900, RON-FR-902 | Test: RON-TC-HLTH-001, RON-TC-HLTH-007 */
ron_fault_t ron_health_init(ron_health_t *h, const ron_health_config_t *cfg);

/* =========================================================================
 * Runtime
 * ========================================================================= */

/**
 * @brief Evaluate loop health for one control step.
 *
 * Reads the setpoint r, process variable y, and commanded output u and advances
 * every condition detector.  Any condition that becomes active latches its bit
 * and, on its first activation, invokes the configured callback (RON-FR-904).
 * The call is passive: it never modifies the controller (RON-FR-903).
 *
 * @param[in,out] h   Pointer to an initialised instance.
 * @param[in]     r   Setpoint / reference (engineering units).  Finite.
 * @param[in]     y   Process variable (engineering units).      Finite.
 * @param[in]     u   Commanded controller output.               Finite.
 * @param[in]     dt  Sample period in seconds.  Must be > 0 and finite.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if h is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if not initialised, dt invalid, or r/y/u non-finite.
 *
 * Satisfies: RON-FR-900, RON-FR-901, RON-FR-903, RON-FR-904, RON-FR-905.
 */
/* Satisfies: RON-FR-900, RON-FR-901, RON-FR-903 | Test: RON-TC-HLTH-002 – RON-TC-HLTH-006 */
ron_fault_t ron_health_step(ron_health_t *h, ron_float_t r, ron_float_t y, ron_float_t u,
                            ron_float_t dt);

/* =========================================================================
 * Status access
 * ========================================================================= */

/**
 * @brief Clear all latched conditions and reset the detectors.
 *
 * Resets status to RON_HEALTH_OK and zeroes every counter and the oscillation
 * window.  The configuration and is_initialised guard are preserved.
 *
 * @param[in,out] h  Pointer to an initialised instance.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if h is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if h is not initialised.
 *
 * Satisfies: RON-FR-905.
 */
/* Satisfies: RON-FR-905 | Test: RON-TC-HLTH-010 */
ron_fault_t ron_health_clear(ron_health_t *h);

/**
 * @brief Read the current latched health status.
 *
 * @param[in]  h       Pointer to an initialised instance.
 * @param[out] status  Receives the latched bitmask.  Must not be NULL.
 *
 * @return  RON_FAULT_NONE           on success.
 * @return  RON_FAULT_NULL_POINTER   if h or status is NULL.
 * @return  RON_FAULT_CONFIG_INVALID if h is not initialised.
 *
 * Satisfies: RON-FR-901, RON-FR-905.
 */
/* Satisfies: RON-FR-901, RON-FR-905 | Test: RON-TC-HLTH-010 */
ron_fault_t ron_health_get(const ron_health_t *h, ron_health_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* RON_HEALTH_H */
