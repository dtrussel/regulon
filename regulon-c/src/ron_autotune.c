/*
 * @file     ron_autotune.c
 * @brief    Relay-feedback PID auto-tuner implementation.
 * @module   ron_autotune
 * @doc      RON-IS-001
 * @req      RON-FR-800, RON-FR-801, RON-FR-802, RON-FR-803,
 *           RON-FR-804, RON-FR-805, RON-FR-806, RON-FR-807
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_autotune.h"

/* Number of half-period crossings per full oscillation cycle. */
#define RON_AT_HALF_PER_CYCLE ((uint16_t) 2U)

/* Pi (no <math.h> dependency — bare-metal safe per ron_platform.h policy). */
#define RON_AT_PI RON_FLOAT_C(3.14159265358979323846)

/* Smallest peak-to-peak half-amplitude treated as a real oscillation. */
#define RON_AT_MIN_AMPLITUDE RON_FLOAT_C(1.0e-6)

/* =========================================================================
 * Tuning-rule factor tables (RON-FR-803), indexed by ron_at_rule_t.
 *
 *   Kp = kp_factor * Ku
 *   Ti = ti_factor * Tu   ->  Ki = Kp / Ti
 *   Td = td_factor * Tu   ->  Kd = Kp * Td
 * ========================================================================= */

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Satisfies: RON-SR-020 */
static bool at_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* Satisfies: RON-FR-801 | Test: RON-TC-AT-002 */
static bool at_config_valid(const ron_at_config_t *cfg)
{
    if (!at_isfinite(cfg->relay_amplitude) || (cfg->relay_amplitude <= RON_FLOAT_C(0.0))) {
        return false;
    }
    if (!at_isfinite(cfg->hysteresis) || (cfg->hysteresis < RON_FLOAT_C(0.0))) {
        return false;
    }
    if (!at_isfinite(cfg->u_bias)) {
        return false;
    }
    if (cfg->min_cycles == 0U) {
        return false;
    }
    if (!at_isfinite(cfg->timeout_s) || (cfg->timeout_s <= RON_FLOAT_C(0.0))) {
        return false;
    }
    if ((unsigned) cfg->tuning_rule > (unsigned) RON_AT_RULE_NO_OS) {
        return false;
    }
    return true;
}

/*
 * Relay output law with hysteresis hold (SADS AT_RELAY).  The result always
 * lies in [u_bias - d, u_bias + d], which the FV harness proves (RON-FR-806).
 */
/* Satisfies: RON-FR-800, RON-FR-806 | Test: RON-TC-AT-007, RON-TC-AT-007-FV */
static ron_float_t at_relay_output(ron_at_t *at, ron_float_t e)
{
    ron_float_t d   = at->cfg.relay_amplitude;
    ron_float_t eps = at->cfg.hysteresis;
    ron_float_t u;

    if (e > eps) {
        u = at->cfg.u_bias + d;
    } else if (e < -eps) {
        u = at->cfg.u_bias - d;
    } else {
        /* Within the hysteresis band: hold the previous relay output. */
        u = at->state.u_relay_prev;
    }

    at->state.u_relay_prev = u;
    return u;
}

/*
 * Update the running sign of the error and report whether a zero crossing just
 * occurred.  The first observed sample only seeds last_sign.
 */
/* Satisfies: RON-FR-802 | Test: RON-TC-AT-003 */
static bool at_detect_crossing(ron_at_t *at, ron_float_t e)
{
    int8_t sign  = (e >= RON_FLOAT_C(0.0)) ? (int8_t) 1 : (int8_t) -1;
    bool crossed = false;

    if (at->state.last_sign == (int8_t) 0) {
        at->state.last_sign = sign;
    } else if (sign != at->state.last_sign) {
        at->state.last_sign = sign;
        crossed             = true;
    } else {
        /* Same sign — no crossing. */
        crossed = false;
    }
    return crossed;
}

/* Satisfies: RON-FR-803 | Test: RON-TC-AT-004 */
static void at_compute_rule(ron_at_t *at)
{
    /* Block scope keeps the tables next to their only reader (MISRA C:2023
     * Rule 8.9); static const still places them in read-only storage. */
    static const ron_float_t at_rule_kp[4] = {RON_FLOAT_C(0.60), RON_FLOAT_C(0.45),
                                              RON_FLOAT_C(0.33), RON_FLOAT_C(0.20)};
    static const ron_float_t at_rule_ti[4] = {RON_FLOAT_C(0.50), RON_FLOAT_C(2.20),
                                              RON_FLOAT_C(0.50), RON_FLOAT_C(0.50)};
    static const ron_float_t at_rule_td[4] = {RON_FLOAT_C(0.125), RON_FLOAT_C(0.158),
                                              RON_FLOAT_C(0.333), RON_FLOAT_C(0.333)};

    unsigned idx   = (unsigned) at->cfg.tuning_rule;
    ron_float_t kp = at_rule_kp[idx] * at->state.Ku;
    ron_float_t ti = at_rule_ti[idx] * at->state.Tu;
    ron_float_t td = at_rule_td[idx] * at->state.Tu;

    at->state.Kp_result = kp;
    at->state.Ki_result = kp / ti;
    at->state.Kd_result = kp * td;
}

/*
 * Estimate Ku / Tu from the accumulated oscillation and derive tuned gains.
 * An oscillation too small to measure aborts the run (insufficient excitation).
 */
/* Satisfies: RON-FR-802, RON-FR-803 | Test: RON-TC-AT-003, RON-TC-AT-004 */
static void at_estimate(ron_at_t *at)
{
    ron_float_t half_avg;
    ron_float_t amplitude;

    at->state.phase = (uint8_t) RON_AT_ESTIMATING;

    half_avg  = at->state.half_period_sum / (ron_float_t) at->state.half_period_count;
    amplitude = (at->state.pv_max - at->state.pv_min) * RON_FLOAT_C(0.5);

    if (amplitude < RON_AT_MIN_AMPLITUDE) {
        at->state.aborted = true;
        at->state.phase   = (uint8_t) RON_AT_ABORTED;
        return;
    }

    at->state.Tu = RON_FLOAT_C(2.0) * half_avg;
    at->state.Ku = (RON_FLOAT_C(4.0) * at->cfg.relay_amplitude) / (RON_AT_PI * amplitude);
    at_compute_rule(at);

    at->state.done  = true;
    at->state.phase = (uint8_t) RON_AT_DONE;
}

/* Advance the settling phase: wait for the first crossing, then start timing. */
/* Satisfies: RON-FR-800 | Test: RON-TC-AT-001 */
static void at_step_settling(ron_at_t *at, ron_float_t y, bool crossed)
{
    if (crossed) {
        at->state.phase            = (uint8_t) RON_AT_RELAY;
        at->state.time_since_cross = RON_FLOAT_C(0.0);
        at->state.pv_min           = y;
        at->state.pv_max           = y;
    }
}

/* Advance the relay phase: track peaks, time half-periods, estimate when ready. */
/* Satisfies: RON-FR-802 | Test: RON-TC-AT-003 */
static void at_step_relay(ron_at_t *at, ron_float_t y, ron_float_t dt, bool crossed)
{
    uint16_t needed = (uint16_t) (RON_AT_HALF_PER_CYCLE * (uint16_t) at->cfg.min_cycles);

    at->state.time_since_cross += dt;

    if (y < at->state.pv_min) {
        at->state.pv_min = y;
    }
    if (y > at->state.pv_max) {
        at->state.pv_max = y;
    }

    if (crossed) {
        at->state.half_period_sum += at->state.time_since_cross;
        at->state.half_period_count = (uint16_t) (at->state.half_period_count + 1U);
        at->state.time_since_cross  = RON_FLOAT_C(0.0);
    }

    if (at->state.half_period_count >= needed) {
        at_estimate(at);
    }
}

/* Restore the PID gains and operating mode captured at start. */
/* Satisfies: RON-FR-807 | Test: RON-TC-AT-008 */
static void at_restore_pid(const ron_at_t *at, ron_pid_instance_t *pid)
{
    (void) ron_pid_set_gains(pid, at->state.saved_Kp, at->state.saved_Ki, at->state.saved_Kd);
    (void) ron_pid_set_mode(pid, at->state.saved_mode, at->cfg.u_bias);
}

/* Validate the runtime arguments of one step. */
/* Satisfies: RON-SR-020 | Test: RON-TC-AT-007 */
static ron_fault_t at_step_args_valid(const ron_at_t *at, ron_float_t r, ron_float_t y,
                                      ron_float_t dt, const ron_float_t *u_out)
{
    if ((at == NULL) || (u_out == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!at->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !at_isfinite(dt)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!at_isfinite(r) || !at_isfinite(y)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* Abort the run if it is still active past the configured time budget. */
/* Satisfies: RON-FR-807 | Test: RON-TC-AT-008 */
static void at_check_timeout(ron_at_t *at)
{
    uint8_t phase = at->state.phase;

    if ((phase == (uint8_t) RON_AT_SETTLING) || (phase == (uint8_t) RON_AT_RELAY)) {
        if (at->state.elapsed_s > at->cfg.timeout_s) {
            at->state.aborted = true;
            at->state.phase   = (uint8_t) RON_AT_ABORTED;
        }
    }
}

/* Zero all dynamic state (phase becomes RON_AT_IDLE). */
/* Satisfies: RON-FR-800 | Test: RON-TC-AT-001 */
static void at_seed_state(ron_at_t *at)
{
    at->state.Ku        = RON_FLOAT_C(0.0);
    at->state.Tu        = RON_FLOAT_C(0.0);
    at->state.Kp_result = RON_FLOAT_C(0.0);
    at->state.Ki_result = RON_FLOAT_C(0.0);
    at->state.Kd_result = RON_FLOAT_C(0.0);

    at->state.phase   = (uint8_t) RON_AT_IDLE;
    at->state.done    = false;
    at->state.aborted = false;

    at->state.u_relay_prev      = RON_FLOAT_C(0.0);
    at->state.elapsed_s         = RON_FLOAT_C(0.0);
    at->state.time_since_cross  = RON_FLOAT_C(0.0);
    at->state.half_period_sum   = RON_FLOAT_C(0.0);
    at->state.pv_min            = RON_FLOAT_C(0.0);
    at->state.pv_max            = RON_FLOAT_C(0.0);
    at->state.half_period_count = 0U;
    at->state.last_sign         = (int8_t) 0;

    at->state.saved_Kp   = RON_FLOAT_C(0.0);
    at->state.saved_Ki   = RON_FLOAT_C(0.0);
    at->state.saved_Kd   = RON_FLOAT_C(0.0);
    at->state.saved_mode = RON_MODE_AUTOMATIC;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/* Satisfies: RON-FR-800, RON-FR-801 | Test: RON-TC-AT-001, RON-TC-AT-002 */
ron_fault_t ron_autotune_init(ron_at_t *at, const ron_at_config_t *cfg)
{
    if ((at == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!at_config_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    at->cfg = *cfg;
    at_seed_state(at);
    at->state.is_initialised = true;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-800, RON-FR-804 | Test: RON-TC-AT-001, RON-TC-AT-005 */
ron_fault_t ron_autotune_start(ron_at_t *at, ron_pid_instance_t *pid)
{
    if ((at == NULL) || (pid == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!at->state.is_initialised || !pid->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    /* Snapshot PID context for later restore (gains are NOT modified). */
    at->state.saved_Kp   = pid->config.Kp;
    at->state.saved_Ki   = pid->config.Ki;
    at->state.saved_Kd   = pid->config.Kd;
    at->state.saved_mode = pid->state.mode;

    /* Prime the relay with a definite initial drive. */
    at->state.u_relay_prev = at->cfg.u_bias + at->cfg.relay_amplitude;
    at->state.phase        = (uint8_t) RON_AT_SETTLING;

    /* Park the PID in manual so it does not fight the relay. */
    (void) ron_pid_set_mode(pid, RON_MODE_MANUAL, at->cfg.u_bias);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-800, RON-FR-802, RON-FR-806 | Test: RON-TC-AT-003, RON-TC-AT-007 */
ron_fault_t ron_autotune_step(ron_at_t *at, ron_float_t r, ron_float_t y, ron_float_t dt,
                              ron_float_t *u_out)
{
    ron_fault_t fault;
    ron_float_t e;
    bool crossed;
    uint8_t phase;

    fault = at_step_args_valid(at, r, y, dt, u_out);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    phase = at->state.phase;

    /* Terminal phases: hold the bias output, results unchanged. */
    if ((phase == (uint8_t) RON_AT_DONE) || (phase == (uint8_t) RON_AT_ABORTED)) {
        *u_out = at->cfg.u_bias;
        return RON_FAULT_NONE;
    }
    /* Not yet started. */
    if (phase == (uint8_t) RON_AT_IDLE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    e       = r - y;
    *u_out  = at_relay_output(at, e);
    crossed = at_detect_crossing(at, e);
    at->state.elapsed_s += dt;

    if (phase == (uint8_t) RON_AT_SETTLING) {
        at_step_settling(at, y, crossed);
    } else {
        at_step_relay(at, y, dt, crossed);
    }

    at_check_timeout(at);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-804 | Test: RON-TC-AT-005 */
ron_fault_t ron_autotune_apply(const ron_at_t *at, ron_pid_instance_t *pid)
{
    ron_fault_t fault;

    if ((at == NULL) || (pid == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (at->state.phase != (uint8_t) RON_AT_DONE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    fault = ron_pid_set_gains(pid, at->state.Kp_result, at->state.Ki_result, at->state.Kd_result);
    (void) ron_pid_set_mode(pid, at->state.saved_mode, at->cfg.u_bias);
    return fault;
}

/* Satisfies: RON-FR-807 | Test: RON-TC-AT-008 */
ron_fault_t ron_autotune_abort(ron_at_t *at, ron_pid_instance_t *pid)
{
    if ((at == NULL) || (pid == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!at->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    at_restore_pid(at, pid);
    at->state.aborted = true;
    at->state.phase   = (uint8_t) RON_AT_ABORTED;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-805 | Test: RON-TC-AT-006 */
ron_fault_t ron_autotune_results(const ron_at_t *at, ron_float_t *Ku, ron_float_t *Tu,
                                 ron_float_t *Kp, ron_float_t *Ki, ron_float_t *Kd)
{
    if (at == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (at->state.phase != (uint8_t) RON_AT_DONE) {
        return RON_FAULT_CONFIG_INVALID;
    }

    if (Ku != NULL) {
        *Ku = at->state.Ku;
    }
    if (Tu != NULL) {
        *Tu = at->state.Tu;
    }
    if (Kp != NULL) {
        *Kp = at->state.Kp_result;
    }
    if (Ki != NULL) {
        *Ki = at->state.Ki_result;
    }
    if (Kd != NULL) {
        *Kd = at->state.Kd_result;
    }
    return RON_FAULT_NONE;
}
