/*
 * @file     ron_pid_types.h
 * @brief    Public type definitions for the Regulon PID controller module.
 * @module   ron_pid_types
 * @doc      RON-IS-001
 * @req      RON-FR-001, RON-FR-004, RON-FR-005, RON-FR-030, RON-FR-040,
 *           RON-FR-050, RON-FR-060, RON-FR-070,
 *           RON-FR-200, RON-FR-201, RON-FR-202, RON-FR-205,
 *           RON-SR-010, RON-SR-011, RON-SR-012, RON-SR-013,
 *           RON-PR-021
 * @version  1.0.0
 * @author   TBD
 * SPDX-License-Identifier: MIT
 *
 * All public enumerations, bitmask types, and data structures used by the
 * PID controller module.  This header has no dependencies other than
 * ron_platform.h.
 *
 * Consumers SHOULD include ron/ron_pid.h rather than this header directly.
 */

#ifndef RON_PID_TYPES_H
#define RON_PID_TYPES_H

#include "ron/ron_platform.h"

/* =========================================================================
 * Enumerations
 * ========================================================================= */

/**
 * @brief Anti-windup scheme selection.
 *
 * Satisfies: RON-FR-030, RON-FR-031, RON-FR-032, RON-FR-033, RON-FR-034.
 */
/* Satisfies: RON-FR-030 – RON-FR-034 | Test: RON-TC-PID-021 – RON-TC-PID-025 */
typedef enum {
    RON_AW_NONE      = 0, /**< Anti-windup disabled.                           */
    RON_AW_BACK_CALC = 1, /**< Back-calculation (tracking) method.             */
    RON_AW_CLAMPING  = 2  /**< Conditional integration clamping.               */
} ron_aw_mode_t;

/**
 * @brief Derivative term computation mode.
 *
 * Satisfies: RON-FR-005.
 */
/* Satisfies: RON-FR-005 | Test: RON-TC-PID-006, RON-TC-PID-007 */
typedef enum {
    RON_DERIV_ON_ERROR       = 0, /**< D term uses error difference.           */
    RON_DERIV_ON_MEASUREMENT = 1  /**< D term uses measurement difference
                                        (default — avoids derivative kick).     */
} ron_deriv_mode_t;

/**
 * @brief Controller operating mode.
 *
 * Satisfies: RON-FR-040, RON-FR-041, RON-FR-042.
 */
/* Satisfies: RON-FR-040 – RON-FR-042 | Test: RON-TC-PID-027 – RON-TC-PID-029 */
typedef enum {
    RON_MODE_AUTOMATIC = 0, /**< Closed-loop PID active.                       */
    RON_MODE_MANUAL    = 1  /**< Caller supplies output; integral frozen.      */
} ron_op_mode_t;

/**
 * @brief Safe-state output policy applied when a fault is detected.
 *
 * Satisfies: RON-SR-011.
 */
/* Satisfies: RON-SR-011 | Test: RON-TC-SAFE-008 */
typedef enum {
    RON_SAFE_HOLD_LAST = 0, /**< Freeze output at the last valid value.        */
    RON_SAFE_ZERO      = 1, /**< Drive output to 0.0.                          */
    RON_SAFE_CONSTANT  = 2  /**< Drive output to ron_pid_config_t.safe_value.  */
} ron_safe_policy_t;

/**
 * @brief Numerical integration method for the integral term.
 *
 * Satisfies: RON-FR-004.
 */
/* Satisfies: RON-FR-004 | Test: RON-TC-PID-004, RON-TC-PID-005 */
typedef enum {
    RON_INTEG_EULER       = 0, /**< Rectangular (Euler forward) — default.    */
    RON_INTEG_TRAPEZOIDAL = 1  /**< Trapezoidal (Tustin) method.              */
} ron_integ_method_t;

/**
 * @brief Feed-forward contribution source.
 *
 * Satisfies: RON-FR-201.
 */
/* Satisfies: RON-FR-201 | Test: RON-TC-FF-002 - RON-TC-FF-005 */
typedef enum {
    RON_FF_DISABLED     = 0, /**< Feed-forward path disabled.                 */
    RON_FF_STATIC_GAIN  = 1, /**< u_ff = gain * setpoint.                    */
    RON_FF_VELOCITY     = 2, /**< u_ff = gain * filtered setpoint velocity.  */
    RON_FF_ACCELERATION = 3, /**< u_ff = gain * filtered setpoint accel.     */
    RON_FF_EXTERNAL     = 4  /**< Caller supplies u_ff directly.             */
} ron_feedforward_mode_t;

/* =========================================================================
 * Fault register type and bitmasks
 * ========================================================================= */

/**
 * @brief Fault code register type.
 *
 * Multiple fault bits may be OR-ed together.  Once set, bits persist until
 * explicitly cleared by ron_pid_fault_clear().  The API uses this type both
 * as a function return value (new fault detected) and as the accumulated
 * fault register inside ron_pid_state_t.
 *
 * Satisfies: RON-SR-010, RON-SR-013.
 */
/* Satisfies: RON-SR-010, RON-SR-013 | Test: RON-TC-SAFE-007, RON-TC-SAFE-010 */
typedef uint8_t ron_fault_t;

#define RON_FAULT_NONE ((ron_fault_t) 0x00U)              /**< No fault.        */
#define RON_FAULT_INPUT_NAN ((ron_fault_t) 0x01U)         /**< r or y is NaN/Inf.         */
#define RON_FAULT_OUTPUT_NAN ((ron_fault_t) 0x02U)        /**< Computed u is NaN/Inf.     */
#define RON_FAULT_INTEGRAL_OVERFLOW ((ron_fault_t) 0x04U) /**< |I| exceeded overflow thresh. */
#define RON_FAULT_CONFIG_INVALID ((ron_fault_t) 0x08U)    /**< Configuration inconsistency. */
#define RON_FAULT_NULL_POINTER ((ron_fault_t) 0x10U)      /**< NULL pointer in API call.  */

/**
 * @brief Status word returned after each computation step.
 *
 * Individual bits may be tested using the RON_STATUS_* constants.
 *
 * Satisfies: RON-FR-070.
 */
/* Satisfies: RON-FR-070 | Test: RON-TC-PID-038 */
typedef uint16_t ron_status_t;

#define RON_STATUS_OK ((ron_status_t) 0x0000U)               /**< Normal.       */
#define RON_STATUS_SATURATED ((ron_status_t) 0x0001U)        /**< Output clamped. */
#define RON_STATUS_RATE_LIMITED ((ron_status_t) 0x0002U)     /**< Rate clamped. */
#define RON_STATUS_AW_ACTIVE ((ron_status_t) 0x0004U)        /**< AW correction active. */
#define RON_STATUS_MANUAL_MODE ((ron_status_t) 0x0008U)      /**< Manual mode.  */
#define RON_STATUS_FAULT ((ron_status_t) 0x0010U)            /**< Fault latched.*/
#define RON_STATUS_SP_FILTER_ACTIVE ((ron_status_t) 0x0020U) /**< SP filter on. */
#define RON_STATUS_NORMALISED ((ron_status_t) 0x0040U)       /**< Normalisation enabled. */
#define RON_STATUS_FF_ACTIVE ((ron_status_t) 0x0080U)        /**< FF contribution nonzero. */

/* =========================================================================
 * Fault handler callback type
 * ========================================================================= */

/**
 * @brief Fault notification callback type.
 *
 * Registered in ron_pid_config_t.fault_cb.  Called once each time a NEW
 * fault bit is set (not re-called for already-latched bits).  The callback
 * is invoked from within ron_pid_step(), so it MUST be ISR-safe if the
 * controller runs in an interrupt context.  The callback MUST NOT call any
 * Regulon API function (no re-entry).
 *
 * @param[in] fault  The single newly-detected fault bit.
 *
 * Satisfies: RON-SR-010.
 */
/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007 */
typedef void (*ron_fault_cb_t)(ron_fault_t fault);

/**
 * @brief Feed-forward configuration.
 *
 * The gain is interpreted by the selected mode and may be signed. N_ff is the
 * independent low-pass filter coefficient for velocity and acceleration
 * estimates; 0 disables feed-forward derivative filtering.
 *
 * Satisfies: RON-FR-201, RON-FR-202.
 */
/* Satisfies: RON-FR-201, RON-FR-202 | Test: RON-TC-FF-002 - RON-TC-FF-006 */
typedef struct {
    ron_feedforward_mode_t mode; /**< Selected feed-forward source.      */
    ron_float_t gain;            /**< Signed gain interpreted by mode.   */
    ron_float_t N_ff;            /**< Independent FF derivative filter.  */
} ron_feedforward_config_t;

/* =========================================================================
 * Configuration structure
 * ========================================================================= */

/**
 * @brief Complete configuration record for one PID controller instance.
 *
 * This structure is copied by value into ron_pid_instance_t at
 * initialisation.  The caller does not need to keep it alive afterwards.
 *
 * All fields are validated by ron_pid_config_validate() before use.
 *
 * Satisfies: RON-FR-001 – RON-FR-007, RON-FR-010 – RON-FR-013,
 *            RON-FR-020 – RON-FR-027, RON-FR-030 – RON-FR-035,
 *            RON-SR-010 – RON-SR-013.
 */
/* Satisfies: RON-FR-001 – RON-FR-035 | Test: RON-TC-PID-001 – RON-TC-PID-026 */
typedef struct {
    /* ── Gain parameters (parallel form) ─────────────────────────────── */
    ron_float_t Kp; /**< Proportional gain. Must be >= 0 and finite.        */
    ron_float_t Ki; /**< Integral gain.     Must be >= 0 and finite.        */
    ron_float_t Kd; /**< Derivative gain.   Must be >= 0 and finite.        */

    /* ── Derivative filter ───────────────────────────────────────────── */
    ron_float_t N; /**< Derivative LP filter bandwidth multiplier.
                            0 disables the filter.  Must be >= 0 and finite.  */

    /* ── 2DOF setpoint weights ───────────────────────────────────────── */
    ron_float_t b; /**< Proportional SP weight in [0, 1]. Default: 1.0.   */
    ron_float_t c; /**< Derivative SP weight in [0, 1].   Default: 1.0.   */

    /* ── Output limits ───────────────────────────────────────────────── */
    ron_float_t u_min; /**< Minimum control output. Must be < u_max.        */
    ron_float_t u_max; /**< Maximum control output.                          */

    /* ── Output rate limit ───────────────────────────────────────────── */
    ron_float_t du_max; /**< Max |Δu| per second. <= 0 disables.            */

    /* ── Integral accumulator clamp ──────────────────────────────────── */
    ron_float_t I_min; /**< Integral lower clamp. Must be < I_max.          */
    ron_float_t I_max; /**< Integral upper clamp.                            */

    /* ── Anti-windup ─────────────────────────────────────────────────── */
    ron_aw_mode_t aw_mode; /**< Anti-windup scheme.                          */
    ron_float_t T_aw;      /**< Back-calc tracking time constant.
                                  Required > 0 when aw_mode=RON_AW_BACK_CALC. */

    /* ── Integration method ──────────────────────────────────────────── */
    ron_integ_method_t integ_method; /**< Euler (default) or Trapezoidal.   */

    /* ── Derivative mode ─────────────────────────────────────────────── */
    ron_deriv_mode_t deriv_mode; /**< Default: RON_DERIV_ON_MEASUREMENT.    */

    /* ── Setpoint pre-filter ─────────────────────────────────────────── */
    ron_float_t tau_sp; /**< SP 1st-order filter time constant.
                               <= 0 disables the filter.                       */

    /* ── Input/output normalisation ──────────────────────────────────── */
    bool normalise;      /**< Enable input/output normalisation.            */
    ron_float_t in_min;  /**< Input engineering minimum (r and y).         */
    ron_float_t in_max;  /**< Input engineering maximum.                    */
    ron_float_t out_min; /**< Output engineering minimum.                  */
    ron_float_t out_max; /**< Output engineering maximum.                  */

    /* ── Safety / fault ──────────────────────────────────────────────── */
    ron_safe_policy_t safe_policy; /**< Safe-state output policy.    */
    ron_float_t safe_value;        /**< Used when policy=CONSTANT.   */
    ron_float_t I_overflow_thresh; /**< |I| > this → FAULT_INTEGRAL_OVERFLOW.
                                                  <= 0 disables.              */

    /* ── Setpoint-change integral reset ──────────────────────────────── */
    ron_float_t sp_reset_threshold; /**< |Δr| > this resets the integral.
                                           <= 0 disables.                      */

    /* Feed-forward extension */
    ron_feedforward_config_t feedforward; /**< Optional additive FF path.      */

    /* ── Fault handler callback ──────────────────────────────────────── */
    ron_fault_cb_t fault_cb; /**< Optional fault notification callback.
                                    NULL = not used.                           */
} ron_pid_config_t;

/* =========================================================================
 * State structure (dynamic, mutable per-step)
 * ========================================================================= */

/**
 * @brief Dynamic (mutable) state for one PID controller instance.
 *
 * Must be zero-initialised before first use, which is performed by
 * ron_pid_init().  The caller shall treat this as opaque and ONLY read it
 * through ron_pid_get_state().
 *
 * Satisfies: RON-FR-050 – RON-FR-054, RON-FR-060 – RON-FR-062.
 */
/* Satisfies: RON-FR-050 – RON-FR-062 | Test: RON-TC-PID-030 – RON-TC-PID-037 */
typedef struct {
    ron_float_t integral;    /**< Accumulated integral term (I).          */
    ron_float_t y_prev;      /**< Previous normalised PV (DoM).           */
    ron_float_t e_D_prev;    /**< Previous derivative error (DoE).        */
    ron_float_t D_filt_prev; /**< Previous filtered derivative output.    */
    ron_float_t r_filt_prev; /**< Previous filtered setpoint.             */
    ron_float_t u_sat_prev;  /**< Previous saturated output (for AW).     */
    ron_float_t u_prev;      /**< Previous pre-saturation output.         */
    ron_float_t e_prev;      /**< Previous error (for trapezoidal).       */
    ron_float_t ff_r_prev;   /**< Previous FF setpoint input.             */
    ron_float_t ff_v_prev;   /**< Previous filtered FF velocity.          */
    ron_float_t ff_a_prev;   /**< Previous filtered FF acceleration.      */
    ron_float_t u_ff_prev;   /**< Previous feed-forward contribution.     */
    ron_op_mode_t mode;      /**< Current operating mode.                 */
    ron_fault_t fault_code;  /**< Accumulated fault register (sticky).    */
    ron_status_t status;     /**< Status word from the last step.         */
    bool is_initialised;     /**< Guard: set by ron_pid_init() only.     */
} ron_pid_state_t;

/* =========================================================================
 * Instance structure (the controller handle)
 * ========================================================================= */

/**
 * @brief Complete PID controller instance.
 *
 * The caller allocates one of these (typically as a file-scope static
 * variable or as a field in a task-control block).  The library NEVER
 * allocates memory.
 *
 * @example
 * @code
 *   static ron_pid_instance_t speed_pid;
 *   ron_pid_config_t cfg = { .Kp = 1.5F, .Ki = 0.3F, ... };
 *   (void)ron_pid_init(&speed_pid, &cfg);
 * @endcode
 *
 * Satisfies: RON-FR-060 – RON-FR-062, RON-SR-003, RON-PR-021.
 */
/* Satisfies: RON-FR-060 – RON-FR-062 | Test: RON-TC-PID-035 – RON-TC-PID-037 */
typedef struct {
    ron_pid_config_t config; /**< Configuration (copied at init, constant during step). */
    ron_pid_state_t state;   /**< Dynamic computation state.                             */
} ron_pid_instance_t;

/* =========================================================================
 * Compile-time size assertions (RON-PR-021, RON-SR-022)
 *
 * The 128-byte RAM budget is specified for the single-precision
 * configuration. The double-precision verification build remains supported
 * but is not constrained by that same RAM target.
 * ========================================================================= */

/* Satisfies: RON-PR-021 | Test: RON-TC-PERF-006 */
#if !defined(RON_USE_DOUBLE) || (RON_USE_DOUBLE != 1)
RON_STATIC_ASSERT(sizeof(ron_pid_state_t) <= 128U,
                  "ron_pid_state_t exceeds 128-byte RAM budget (RON-PR-021)");

/* Satisfies: RON-PR-021 | Test: RON-TC-PERF-005 */
RON_STATIC_ASSERT(sizeof(ron_pid_config_t) <= 128U,
                  "ron_pid_config_t exceeds 128-byte RAM budget (RON-PR-021)");
#endif

#endif /* RON_PID_TYPES_H */
