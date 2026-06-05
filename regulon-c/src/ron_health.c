/*
 * @file     ron_health.c
 * @brief    Control-loop health monitor implementation.
 * @module   ron_health
 * @doc      RON-IS-001
 * @req      RON-FR-900, RON-FR-901, RON-FR-902,
 *           RON-FR-903, RON-FR-904, RON-FR-905
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_health.h"

/* Largest output delta (in absolute units) still treated as "not moving". */
#define RON_HEALTH_STUCK_EPS RON_FLOAT_C(1.0e-6)

/* Smallest setpoint change treated as a new setpoint step. */
#define RON_HEALTH_STEP_EPS RON_FLOAT_C(1.0e-6)

/* Error-sign codes stored in the oscillation ring (0 = empty slot). */
#define RON_HEALTH_SIGN_POS ((uint8_t) 1U)
#define RON_HEALTH_SIGN_NEG ((uint8_t) 2U)

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Satisfies: RON-SR-020 */
static bool health_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* A finite, strictly positive time constant. */
/* Satisfies: RON-FR-902 | Test: RON-TC-HLTH-001 */
static bool health_pos_valid(ron_float_t value)
{
    return health_isfinite(value) && (value > RON_FLOAT_C(0.0));
}

/* A finite, non-negative threshold magnitude. */
/* Satisfies: RON-FR-902 | Test: RON-TC-HLTH-001 */
static bool health_nonneg_valid(ron_float_t value)
{
    return health_isfinite(value) && (value >= RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-902 | Test: RON-TC-HLTH-007 */
static bool health_config_valid(const ron_health_config_t *cfg)
{
    if (!health_pos_valid(cfg->t_sat_max)) {
        return false;
    }
    if (!health_nonneg_valid(cfg->err_diverge_thresh)) {
        return false;
    }
    if ((unsigned) cfg->osc_count_thresh >= (unsigned) RON_HEALTH_OSC_WINDOW) {
        return false;
    }
    if (!health_nonneg_valid(cfg->dead_band)) {
        return false;
    }
    if (!health_pos_valid(cfg->dropout_time)) {
        return false;
    }
    if (!health_nonneg_valid(cfg->ss_err_thresh)) {
        return false;
    }
    if (!health_pos_valid(cfg->settling_time)) {
        return false;
    }
    return true;
}

/*
 * Round an accumulated duration to the nearest sample and test it against a
 * threshold.  The half-sample bias makes the boundary land on the expected step
 * regardless of single-precision accumulation error.
 */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-002 */
static bool health_dur_reached(ron_float_t accum, ron_float_t dt, ron_float_t thresh)
{
    return (accum + (RON_FLOAT_C(0.5) * dt)) >= thresh;
}

/* Zero every dynamic field (status becomes RON_HEALTH_OK). */
/* Satisfies: RON-FR-905 | Test: RON-TC-HLTH-010 */
static void health_seed_state(ron_health_t *h)
{
    uint8_t i;

    h->state.status       = RON_HEALTH_OK;
    h->state.t_saturated  = RON_FLOAT_C(0.0);
    h->state.t_dropout    = RON_FLOAT_C(0.0);
    h->state.t_since_step = RON_FLOAT_C(0.0);

    for (i = 0U; i < (uint8_t) RON_HEALTH_OSC_WINDOW; ++i) {
        h->state.osc_window[i] = 0U;
    }
    h->state.osc_idx = 0U;

    h->state.e_prev     = RON_FLOAT_C(0.0);
    h->state.y_prev     = RON_FLOAT_C(0.0);
    h->state.u_prev     = RON_FLOAT_C(0.0);
    h->state.prev_valid = false;
}

/* OUTPUT_STUCK: the commanded output has not moved for longer than t_sat_max. */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-002 */
static bool health_stuck(ron_health_t *h, ron_float_t u, ron_float_t dt)
{
    bool moved = h->state.prev_valid && (ron_fabs(u - h->state.u_prev) > RON_HEALTH_STUCK_EPS);

    if (moved) {
        h->state.t_saturated = RON_FLOAT_C(0.0);
    } else {
        h->state.t_saturated += dt;
    }
    return health_dur_reached(h->state.t_saturated, dt, h->cfg.t_sat_max);
}

/* DIVERGING: the error is large in magnitude and still growing. */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-003 */
static bool health_diverging(const ron_health_t *h, ron_float_t e)
{
    ron_float_t de = e - h->state.e_prev;
    bool large     = ron_fabs(e) > h->cfg.err_diverge_thresh;
    bool growing   = (e * de) > RON_FLOAT_C(0.0);

    return large && growing;
}

/* Count error-sign changes across the sliding window (oldest -> newest). */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-004 */
static uint8_t health_osc_count(const ron_health_t *h)
{
    uint8_t changes = 0U;
    uint8_t i;

    for (i = 0U; i < (uint8_t) (RON_HEALTH_OSC_WINDOW - 1U); ++i) {
        uint8_t ia = (uint8_t) ((h->state.osc_idx + i) % (uint8_t) RON_HEALTH_OSC_WINDOW);
        uint8_t ib = (uint8_t) ((h->state.osc_idx + i + 1U) % (uint8_t) RON_HEALTH_OSC_WINDOW);
        uint8_t a  = h->state.osc_window[ia];
        uint8_t b  = h->state.osc_window[ib];

        /* Empty slots form a contiguous prefix of the oldest->newest walk, so a
         * non-empty a is always followed by a non-empty b; testing a suffices. */
        if ((a != 0U) && (a != b)) {
            changes = (uint8_t) (changes + 1U);
        }
    }
    return changes;
}

/* OSCILLATING: push the current error sign and test the window change count. */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-004 */
static bool health_oscillating(ron_health_t *h, ron_float_t e)
{
    uint8_t sign = (e >= RON_FLOAT_C(0.0)) ? RON_HEALTH_SIGN_POS : RON_HEALTH_SIGN_NEG;

    h->state.osc_window[h->state.osc_idx] = sign;
    h->state.osc_idx = (uint8_t) ((h->state.osc_idx + 1U) % (uint8_t) RON_HEALTH_OSC_WINDOW);

    return health_osc_count(h) > h->cfg.osc_count_thresh;
}

/* SENSOR_DROPOUT: the measurement has stayed within dead_band for too long. */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-005 */
static bool health_dropout(ron_health_t *h, ron_float_t y, ron_float_t dt)
{
    bool moved = h->state.prev_valid && (ron_fabs(y - h->state.y_prev) >= h->cfg.dead_band);

    if (moved) {
        h->state.t_dropout = RON_FLOAT_C(0.0);
    } else {
        h->state.t_dropout += dt;
    }
    return health_dur_reached(h->state.t_dropout, dt, h->cfg.dropout_time);
}

/* SP_UNREACHABLE: a steady-state error persists past the settling budget. */
/* Satisfies: RON-FR-901 | Test: RON-TC-HLTH-006 */
static bool health_unreachable(ron_health_t *h, ron_float_t r, ron_float_t e, ron_float_t dt)
{
    /* Previous setpoint recovered from the stored error and measurement. */
    ron_float_t r_prev = h->state.e_prev + h->state.y_prev;
    bool stepped       = h->state.prev_valid && (ron_fabs(r - r_prev) > RON_HEALTH_STEP_EPS);
    bool persists;
    bool beyond;

    if (stepped) {
        h->state.t_since_step = RON_FLOAT_C(0.0);
    } else {
        h->state.t_since_step += dt;
    }

    persists = ron_fabs(e) > h->cfg.ss_err_thresh;
    beyond   = health_dur_reached(h->state.t_since_step, dt, h->cfg.settling_time);
    return persists && beyond;
}

/* Latch a condition bit and fire the callback on its first activation. */
/* Satisfies: RON-FR-904, RON-FR-905 | Test: RON-TC-HLTH-009, RON-TC-HLTH-010 */
static void health_latch(ron_health_t *h, ron_health_status_t bit, bool active)
{
    if (active && ((h->state.status & bit) == 0U)) {
        h->state.status = (ron_health_status_t) (h->state.status | bit);
        if (h->cfg.cb != NULL) {
            h->cfg.cb(bit);
        }
    }
}

/* Validate the runtime arguments of one step. */
/* Satisfies: RON-SR-020 | Test: RON-TC-HLTH-002 */
static ron_fault_t health_step_args(const ron_health_t *h, ron_float_t r, ron_float_t y,
                                    ron_float_t u, ron_float_t dt)
{
    if (h == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!h->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!health_isfinite(dt) || (dt <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (!health_isfinite(r) || !health_isfinite(y) || !health_isfinite(u)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/* Satisfies: RON-FR-900, RON-FR-902 | Test: RON-TC-HLTH-001, RON-TC-HLTH-007 */
ron_fault_t ron_health_init(ron_health_t *h, const ron_health_config_t *cfg)
{
    if ((h == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!health_config_valid(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    h->cfg = *cfg;
    health_seed_state(h);
    h->state.is_initialised = true;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-900, RON-FR-901, RON-FR-903 | Test: RON-TC-HLTH-002 – RON-TC-HLTH-006 */
ron_fault_t ron_health_step(ron_health_t *h, ron_float_t r, ron_float_t y, ron_float_t u,
                            ron_float_t dt)
{
    ron_fault_t fault;
    ron_float_t e;

    fault = health_step_args(h, r, y, u, dt);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    e = r - y;

    health_latch(h, RON_HEALTH_OUTPUT_STUCK, health_stuck(h, u, dt));
    health_latch(h, RON_HEALTH_DIVERGING, health_diverging(h, e));
    health_latch(h, RON_HEALTH_OSCILLATING, health_oscillating(h, e));
    health_latch(h, RON_HEALTH_SENSOR_DROPOUT, health_dropout(h, y, dt));
    health_latch(h, RON_HEALTH_SP_UNREACHABLE, health_unreachable(h, r, e, dt));

    h->state.e_prev     = e;
    h->state.y_prev     = y;
    h->state.u_prev     = u;
    h->state.prev_valid = true;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-905 | Test: RON-TC-HLTH-010 */
ron_fault_t ron_health_clear(ron_health_t *h)
{
    if (h == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!h->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    health_seed_state(h);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-901, RON-FR-905 | Test: RON-TC-HLTH-010 */
ron_fault_t ron_health_get(const ron_health_t *h, ron_health_status_t *status)
{
    if ((h == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!h->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    *status = h->state.status;
    return RON_FAULT_NONE;
}
