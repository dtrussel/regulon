/*
 * @file     ron_metrics.c
 * @brief    Runtime performance metrics accumulator implementation.
 * @module   ron_metrics
 * @doc      RON-IS-001
 * @req      RON-FR-950, RON-FR-951, RON-FR-952, RON-FR-953, RON-FR-954
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_metrics.h"

/* Smallest |step_size| (engineering units) for which the transient metrics
 * (rise, overshoot, settling) are evaluated.  A step below this is treated as
 * "no step", so the transient metrics stay at their not-reached sentinels and
 * the divide by step_size is never taken. */
#define RON_METRICS_MIN_STEP RON_FLOAT_C(1.0e-6)

/* Fraction of the step that marks the 10 % and 90 % rise-time levels. */
#define RON_METRICS_RISE_LO RON_FLOAT_C(0.10)
#define RON_METRICS_RISE_HI RON_FLOAT_C(0.90)

/* Percent scaling for the overshoot metric. */
#define RON_METRICS_PERCENT RON_FLOAT_C(100.0)

/* Sentinel for a transient metric that has not yet been observed. */
#define RON_METRICS_UNSET RON_FLOAT_C(-1.0)

/* =========================================================================
 * Internal helpers — validation
 * ========================================================================= */

/* Satisfies: RON-SR-020 */
static bool metrics_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* A finite, strictly positive quantity. */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
static bool metrics_pos_valid(ron_float_t value)
{
    return metrics_isfinite(value) && (value > RON_FLOAT_C(0.0));
}

/* A finite, non-negative quantity. */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
static bool metrics_nonneg_valid(ron_float_t value)
{
    return metrics_isfinite(value) && (value >= RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-950, RON-FR-952 | Test: RON-TC-MET-001 */
static bool metrics_config_valid(const ron_metrics_config_t *cfg)
{
    if ((cfg->mode != RON_METRICS_CUMULATIVE) && (cfg->mode != RON_METRICS_WINDOWED)) {
        return false;
    }
    if ((cfg->mode == RON_METRICS_WINDOWED) && (cfg->window_steps == 0U)) {
        return false;
    }
    if (!metrics_pos_valid(cfg->band_pct)) {
        return false;
    }
    if (!metrics_nonneg_valid(cfg->settle_confirm)) {
        return false;
    }
    if (!metrics_pos_valid(cfg->step_thresh)) {
        return false;
    }
    return true;
}

/* Validate the instance handle for a mutating call. */
/* Satisfies: RON-SR-020 | Test: RON-TC-MET-001 */
static ron_fault_t metrics_check_handle(const ron_metrics_t *m)
{
    if (m == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!m->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* Validate the per-step signal arguments. */
/* Satisfies: RON-SR-020 | Test: RON-TC-MET-001 */
static ron_fault_t metrics_check_inputs(ron_float_t r, ron_float_t y, ron_float_t dt)
{
    if (!metrics_isfinite(dt) || (dt <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!metrics_isfinite(r) || !metrics_isfinite(y)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/*
 * Round an accumulated duration to the nearest sample and test it against a
 * threshold.  The half-sample bias makes the boundary land on the expected step
 * regardless of single-precision accumulation error.
 */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-004 */
static bool metrics_dur_reached(ron_float_t accum, ron_float_t dt, ron_float_t thresh)
{
    return (accum + (RON_FLOAT_C(0.5) * dt)) >= thresh;
}

/* =========================================================================
 * Internal helpers — state restart
 * ========================================================================= */

/* Clear the transient outputs and their timers (keeps the step reference). */
/* Satisfies: RON-FR-951, RON-FR-954 | Test: RON-TC-MET-007 */
static void metrics_clear_transient(ron_metrics_t *m)
{
    m->peak_overshoot     = RON_FLOAT_C(0.0);
    m->rise_time          = RON_METRICS_UNSET;
    m->settling_time      = RON_METRICS_UNSET;
    m->t_elapsed          = RON_FLOAT_C(0.0);
    m->t_rise_start       = RON_FLOAT_C(0.0);
    m->in_band_time       = RON_FLOAT_C(0.0);
    m->rise_10pct_crossed = false;
}

/* Restart the transient metrics around a fresh setpoint step. */
/* Satisfies: RON-FR-954 | Test: RON-TC-MET-007 */
static void metrics_restart_transient(ron_metrics_t *m, ron_float_t r, ron_float_t y)
{
    m->step_target = r;
    m->step_ref    = y;
    m->step_size   = r - y;
    metrics_clear_transient(m);
}

/* Start a fresh accumulation window (error integrals and timers). */
/* Satisfies: RON-FR-952 | Test: RON-TC-MET-005 */
static void metrics_window_restart(ron_metrics_t *m)
{
    m->iae            = RON_FLOAT_C(0.0);
    m->ise            = RON_FLOAT_C(0.0);
    m->itae           = RON_FLOAT_C(0.0);
    m->window_counter = 0U;
    metrics_clear_transient(m);
}

/* Zero every running field (configuration and enable state are preserved). */
/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
static void metrics_seed_state(ron_metrics_t *m)
{
    m->iae            = RON_FLOAT_C(0.0);
    m->ise            = RON_FLOAT_C(0.0);
    m->itae           = RON_FLOAT_C(0.0);
    m->step_ref       = RON_FLOAT_C(0.0);
    m->step_target    = RON_FLOAT_C(0.0);
    m->step_size      = RON_FLOAT_C(0.0);
    m->r_prev         = RON_FLOAT_C(0.0);
    m->window_counter = 0U;
    m->prev_valid     = false;
    metrics_clear_transient(m);
}

/* =========================================================================
 * Internal helpers — per-step detectors
 * ========================================================================= */

/* True when |Δr| since the previous step reaches the configured threshold. */
/* Satisfies: RON-FR-954 | Test: RON-TC-MET-007 */
static bool metrics_step_detected(const ron_metrics_t *m, ron_float_t r)
{
    return ron_fabs(r - m->r_prev) >= m->cfg.step_thresh;
}

/* Roll over to a new window at the window boundary (windowed mode only). */
/* Satisfies: RON-FR-952 | Test: RON-TC-MET-005 */
static void metrics_maybe_window_restart(ron_metrics_t *m)
{
    if ((m->cfg.mode == RON_METRICS_WINDOWED) && (m->window_counter >= m->cfg.window_steps)) {
        metrics_window_restart(m);
    }
}

/* Establish the step reference on the first sample or on a detected step. */
/* Satisfies: RON-FR-954 | Test: RON-TC-MET-007 */
static void metrics_maybe_restart_transient(ron_metrics_t *m, ron_float_t r, ron_float_t y)
{
    if (!m->prev_valid) {
        metrics_restart_transient(m, r, y);
        m->prev_valid = true;
    } else if (metrics_step_detected(m, r)) {
        metrics_restart_transient(m, r, y);
    } else {
        /* No step: keep the current reference frame. */
    }
}

/* Advance the error integrals (IAE, ISE, ITAE). */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-002 */
static void metrics_accumulate(ron_metrics_t *m, ron_float_t e, ron_float_t e_abs, ron_float_t dt)
{
    m->iae += e_abs * dt;
    m->ise += (e * e) * dt;
    m->itae += (m->t_elapsed * e_abs) * dt;
}

/* Track the 10 %->90 % rise time of the step response. */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-004 */
static void metrics_update_rise(ron_metrics_t *m, ron_float_t y)
{
    ron_float_t frac = (y - m->step_ref) / m->step_size;

    if ((!m->rise_10pct_crossed) && (frac >= RON_METRICS_RISE_LO)) {
        m->t_rise_start       = m->t_elapsed;
        m->rise_10pct_crossed = true;
    }
    if (m->rise_10pct_crossed && (frac >= RON_METRICS_RISE_HI) &&
        (m->rise_time < RON_FLOAT_C(0.0))) {
        m->rise_time = m->t_elapsed - m->t_rise_start;
    }
}

/* Track the peak overshoot beyond the target, in percent of the step. */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-003 */
static void metrics_update_overshoot(ron_metrics_t *m, ron_float_t y)
{
    ron_float_t os_frac = (y - m->step_target) / m->step_size;

    if (os_frac > RON_FLOAT_C(0.0)) {
        ron_float_t pct = os_frac * RON_METRICS_PERCENT;
        if (pct > m->peak_overshoot) {
            m->peak_overshoot = pct;
        }
    }
}

/* Track the settling time: dwell within the band for settle_confirm seconds. */
/* Satisfies: RON-FR-951 | Test: RON-TC-MET-004 */
static void metrics_update_settling(ron_metrics_t *m, ron_float_t e_abs, ron_float_t dt)
{
    ron_float_t band = ron_fabs(m->step_size) * m->cfg.band_pct;

    if (e_abs <= band) {
        m->in_band_time += dt;
        if (metrics_dur_reached(m->in_band_time, dt, m->cfg.settle_confirm) &&
            (m->settling_time < RON_FLOAT_C(0.0))) {
            m->settling_time = m->t_elapsed;
        }
    } else {
        m->in_band_time = RON_FLOAT_C(0.0);
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/* Satisfies: RON-FR-950, RON-FR-953 | Test: RON-TC-MET-001 */
ron_fault_t ron_metrics_init(ron_metrics_t *m, const ron_metrics_config_t *cfg)
{
    if ((m == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!metrics_config_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    m->cfg = *cfg;
    metrics_seed_state(m);
    m->enabled        = false;
    m->is_initialised = true;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-950 | Test: RON-TC-MET-001 */
ron_fault_t ron_metrics_reset(ron_metrics_t *m)
{
    ron_fault_t fault = metrics_check_handle(m);

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    metrics_seed_state(m);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-953 | Test: RON-TC-MET-006 */
ron_fault_t ron_metrics_enable(ron_metrics_t *m, bool enable)
{
    ron_fault_t fault = metrics_check_handle(m);

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    m->enabled = enable;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-951, RON-FR-952, RON-FR-953, RON-FR-954
 * Test: RON-TC-MET-002 – RON-TC-MET-007 */
ron_fault_t ron_metrics_step(ron_metrics_t *m, ron_float_t r, ron_float_t y, ron_float_t dt)
{
    ron_fault_t fault = metrics_check_handle(m);
    ron_float_t e;
    ron_float_t e_abs;

    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!m->enabled) {
        return RON_FAULT_NONE; /* Zero-overhead no-op when disabled (RON-FR-953). */
    }
    fault = metrics_check_inputs(r, y, dt);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    metrics_maybe_window_restart(m);
    metrics_maybe_restart_transient(m, r, y);
    m->r_prev = r;

    e     = r - y;
    e_abs = ron_fabs(e);
    m->t_elapsed += dt;
    m->window_counter += 1U;
    metrics_accumulate(m, e, e_abs, dt);

    if (ron_fabs(m->step_size) > RON_METRICS_MIN_STEP) {
        metrics_update_rise(m, y);
        metrics_update_overshoot(m, y);
        metrics_update_settling(m, e_abs, dt);
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-951 | Test: RON-TC-MET-002 */
ron_fault_t ron_metrics_get(const ron_metrics_t *m, ron_metrics_result_t *out)
{
    if ((m == NULL) || (out == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!m->is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    out->IAE            = m->iae;
    out->ISE            = m->ise;
    out->ITAE           = m->itae;
    out->peak_overshoot = m->peak_overshoot;
    out->rise_time      = m->rise_time;
    out->settling_time  = m->settling_time;
    return RON_FAULT_NONE;
}
