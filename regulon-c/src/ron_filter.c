/*
 * @file     ron_filter.c
 * @brief    Signal conditioning filter implementations.
 * @module   ron_filter
 * @doc      RON-IS-001
 * @req      RON-FR-100, RON-FR-101, RON-FR-102, RON-FR-103,
 *           RON-FR-110, RON-FR-111, RON-FR-115, RON-FR-116,
 *           RON-FR-117, RON-FR-120, RON-FR-121, RON-FR-122,
 *           RON-FR-123, RON-FR-130, RON-FR-131
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_filter.h"

#include <math.h>

#define RON_FILTER_TWO_PI RON_FLOAT_C(6.28318530717958647692)
#define RON_FILTER_HALF RON_FLOAT_C(0.5)
#define RON_FILTER_TWO RON_FLOAT_C(2.0)
#define RON_FILTER_FOUR RON_FLOAT_C(4.0)

typedef enum {
    FILTER_KIND_LP    = 0,
    FILTER_KIND_HP    = 1,
    FILTER_KIND_BP    = 2,
    FILTER_KIND_NOTCH = 3
} filter_coeff_kind_t;

/* Satisfies: RON-SR-020 | Test: RON-TC-FILT-004 */
static bool filter_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* Satisfies: RON-FR-103 | Test: RON-TC-FILT-004 */
static bool filter_positive(ron_float_t value)
{
    return filter_isfinite(value) && (value > RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-103 | Test: RON-TC-FILT-004 */
static bool filter_valid_alpha(ron_float_t alpha)
{
    return filter_positive(alpha) && (alpha <= RON_FLOAT_C(1.0));
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
static ron_fault_t filter_check_initialised(bool is_initialised)
{
    return is_initialised ? RON_FAULT_NONE : RON_FAULT_CONFIG_INVALID;
}

/* Satisfies: RON-FR-101 | Test: RON-TC-FILT-002 */
static ron_fault_t filter_step_fault(ron_fault_t fault, ron_status_t *status)
{
    *status = RON_STATUS_FAULT;
    return fault;
}

/* Satisfies: RON-FR-103, RON-FR-120 | Test: RON-TC-FILT-004, RON-TC-FILT-011 */
static bool filter_section_coefficients_finite(const ron_biquad_section_t *s)
{
    return filter_isfinite(s->b0) && filter_isfinite(s->b1) && filter_isfinite(s->b2) &&
           filter_isfinite(s->a1) && filter_isfinite(s->a2);
}

/* Satisfies: RON-FR-103, RON-FR-120 | Test: RON-TC-FILT-004, RON-TC-FILT-011 */
static bool filter_section_has_feedthrough(const ron_biquad_section_t *s)
{
    return (ron_fabs(s->b0) > RON_FLOAT_EPSILON) || (ron_fabs(s->b1) > RON_FLOAT_EPSILON) ||
           (ron_fabs(s->b2) > RON_FLOAT_EPSILON);
}

/* Satisfies: RON-FR-103, RON-FR-120 | Test: RON-TC-FILT-004, RON-TC-FILT-011 */
static bool filter_section_denominator_stable(const ron_biquad_section_t *s)
{
    bool valid;

    valid = ((RON_FLOAT_C(1.0) + s->a1 + s->a2) > RON_FLOAT_EPSILON);
    valid = valid && ((RON_FLOAT_C(1.0) - s->a1 + s->a2) > RON_FLOAT_EPSILON);
    valid = valid && ((RON_FLOAT_C(1.0) - s->a2) > RON_FLOAT_EPSILON);

    return valid;
}

/* Satisfies: RON-FR-103, RON-FR-120 | Test: RON-TC-FILT-004, RON-TC-FILT-011 */
static bool filter_valid_section(const ron_biquad_section_t *s)
{
    return filter_section_coefficients_finite(s) && filter_section_has_feedthrough(s) &&
           filter_section_denominator_stable(s);
}

/* Satisfies: RON-FR-103, RON-FR-121 | Test: RON-TC-FILT-004, RON-TC-FILT-012 */
static bool filter_valid_biquad_config(const ron_biquad_config_t *cfg)
{
    bool valid;
    uint8_t idx;

    valid = (cfg->n_sections > 0U) && (cfg->n_sections <= (uint8_t) RON_BIQUAD_MAX_SECTIONS);
    idx   = 0U;
    while (valid && (idx < cfg->n_sections)) {
        valid = filter_valid_section(&cfg->sections[idx]);
        idx++;
    }

    return valid;
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
static ron_fault_t filter_validate_coeff_inputs(const ron_biquad_section_t *s,
                                                ron_float_t frequency, ron_float_t Q,
                                                ron_float_t dt)
{
    ron_fault_t fault;

    fault = RON_FAULT_NONE;
    if (s == NULL) {
        fault = RON_FAULT_NULL_POINTER;
    } else if (!filter_positive(frequency) || !filter_positive(Q) || !filter_positive(dt)) {
        fault = RON_FAULT_CONFIG_INVALID;
    } else {
        ron_float_t nyquist_limit = RON_FLOAT_C(1.0) / (RON_FILTER_TWO * dt);
        if (frequency >= nyquist_limit) {
            fault = RON_FAULT_CONFIG_INVALID;
        }
    }

    return fault;
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
static ron_fault_t filter_coeff_common(ron_biquad_section_t *s, ron_float_t frequency,
                                       ron_float_t Q, ron_float_t dt, filter_coeff_kind_t kind)
{
    ron_fault_t fault;

    fault = filter_validate_coeff_inputs(s, frequency, Q, dt);
    if (fault == RON_FAULT_NONE) {
        ron_float_t omega = RON_FILTER_TWO_PI * frequency * dt;
        ron_float_t c     = (ron_float_t) cos((double) omega);
        ron_float_t sn    = (ron_float_t) sin((double) omega);
        ron_float_t alpha = sn / (RON_FILTER_TWO * Q);
        ron_float_t norm  = RON_FLOAT_C(1.0) / (RON_FLOAT_C(1.0) + alpha);

        if (kind == FILTER_KIND_LP) {
            s->b0 = ((RON_FLOAT_C(1.0) - c) * RON_FILTER_HALF) * norm;
            s->b1 = (RON_FLOAT_C(1.0) - c) * norm;
            s->b2 = s->b0;
        } else if (kind == FILTER_KIND_HP) {
            s->b0 = ((RON_FLOAT_C(1.0) + c) * RON_FILTER_HALF) * norm;
            s->b1 = (-(RON_FLOAT_C(1.0) + c)) * norm;
            s->b2 = s->b0;
        } else if (kind == FILTER_KIND_BP) {
            s->b0 = alpha * norm;
            s->b1 = RON_FLOAT_C(0.0);
            s->b2 = -alpha * norm;
        } else {
            s->b0 = norm;
            s->b1 = (-RON_FILTER_TWO * c) * norm;
            s->b2 = norm;
        }
        s->a1 = (-RON_FILTER_TWO * c) * norm;
        s->a2 = (RON_FLOAT_C(1.0) - alpha) * norm;
    }

    return fault;
}

/* Satisfies: RON-FR-110 | Test: RON-TC-FILT-005 */
static ron_float_t filter_lp1_apply(ron_lp1_t *f, ron_float_t x)
{
    return (f->cfg.alpha * x) + ((RON_FLOAT_C(1.0) - f->cfg.alpha) * f->state.y_prev);
}

/* Satisfies: RON-FR-115, RON-FR-117 | Test: RON-TC-FILT-008, RON-TC-FILT-010 */
static ron_float_t filter_ma_apply(ron_ma_t *f, ron_float_t x)
{
    ron_float_t oldest;
    ron_float_t denominator;

    oldest                     = f->state.buf[f->state.idx];
    f->state.sum               = (f->state.sum + x) - oldest;
    f->state.buf[f->state.idx] = x;
    f->state.idx++;
    if (f->state.idx >= f->cfg.M) {
        f->state.idx = 0U;
    }
    if (f->state.count < f->cfg.M) {
        f->state.count++;
    }
    denominator = (ron_float_t) f->cfg.M;

    return f->state.sum / denominator;
}

/* Satisfies: RON-FR-120 | Test: RON-TC-FILT-011 */
static ron_float_t filter_biquad_section_step(ron_biquad_t *f, uint8_t idx, ron_float_t input)
{
    const ron_biquad_section_t *section;
    ron_float_t w0;
    ron_float_t output;

    section = &f->cfg.sections[idx];
    w0      = (input - (section->a1 * f->state.w1[idx])) - (section->a2 * f->state.w2[idx]);
    output =
        ((section->b0 * w0) + (section->b1 * f->state.w1[idx])) + (section->b2 * f->state.w2[idx]);
    f->state.w2[idx] = f->state.w1[idx];
    f->state.w1[idx] = w0;

    return output;
}

/* Satisfies: RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-011, RON-TC-FILT-012 */
static ron_float_t filter_biquad_apply(ron_biquad_t *f, ron_float_t x)
{
    ron_float_t output;
    uint8_t idx;

    output = x;
    idx    = 0U;
    while (idx < f->cfg.n_sections) {
        output = filter_biquad_section_step(f, idx, output);
        idx++;
    }

    return output;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
static void filter_reset_ma_state(ron_ma_state_t *state)
{
    uint8_t idx;

    idx = 0U;
    while (idx < (uint8_t) RON_MA_MAX_WINDOW) {
        state->buf[idx] = RON_FLOAT_C(0.0);
        idx++;
    }
    state->sum            = RON_FLOAT_C(0.0);
    state->y_prev         = RON_FLOAT_C(0.0);
    state->idx            = 0U;
    state->count          = 0U;
    state->fault_code     = RON_FAULT_NONE;
    state->status         = RON_STATUS_OK;
    state->is_initialised = true;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
static void filter_reset_biquad_state(ron_biquad_state_t *state)
{
    uint8_t idx;

    idx = 0U;
    while (idx < (uint8_t) RON_BIQUAD_MAX_SECTIONS) {
        state->w1[idx] = RON_FLOAT_C(0.0);
        state->w2[idx] = RON_FLOAT_C(0.0);
        idx++;
    }
    state->y_prev         = RON_FLOAT_C(0.0);
    state->fault_code     = RON_FAULT_NONE;
    state->status         = RON_STATUS_OK;
    state->is_initialised = true;
}

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-110 | Test: RON-TC-FILT-001 - RON-TC-FILT-005 */
ron_fault_t ron_lp1_init(ron_lp1_t *f, const ron_lp1_config_t *cfg)
{
    if ((f == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!filter_valid_alpha(cfg->alpha)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->cfg                  = *cfg;
    f->state.y_prev         = RON_FLOAT_C(0.0);
    f->state.fault_code     = RON_FAULT_NONE;
    f->state.status         = RON_STATUS_OK;
    f->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-111 | Test: RON-TC-FILT-006, RON-TC-FILT-007 */
ron_fault_t ron_lp1_init_fc(ron_lp1_t *f, ron_float_t fc, ron_float_t dt)
{
    ron_lp1_config_t cfg;
    ron_float_t omega;

    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!filter_positive(fc) || !filter_positive(dt)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    omega     = RON_FILTER_TWO_PI * fc * dt;
    cfg.alpha = omega / (RON_FLOAT_C(1.0) + omega);
    return ron_lp1_init(f, &cfg);
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_lp1_reset(ron_lp1_t *f)
{
    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->state.y_prev     = RON_FLOAT_C(0.0);
    f->state.fault_code = RON_FAULT_NONE;
    f->state.status     = RON_STATUS_OK;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102, RON-FR-110 | Test: RON-TC-FILT-003, RON-TC-FILT-005 */
ron_fault_t ron_lp1_step(ron_lp1_t *f, ron_float_t x, ron_float_t *y)
{
    ron_float_t y_new;

    if ((f == NULL) || (y == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (f->state.fault_code != RON_FAULT_NONE) {
        *y = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    if (!filter_isfinite(x)) {
        f->state.fault_code = RON_FAULT_INPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }

    y_new = filter_lp1_apply(f, x);
    if (!filter_isfinite(y_new)) {
        f->state.fault_code = RON_FAULT_OUTPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    f->state.y_prev = y_new;
    f->state.status = RON_STATUS_OK;
    *y              = y_new;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_lp1_get_state(const ron_lp1_t *f, ron_lp1_state_t *state)
{
    if ((f == NULL) || (state == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    *state = f->state;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-115, RON-FR-116 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-008, RON-TC-FILT-009 */
ron_fault_t ron_ma_init(ron_ma_t *f, const ron_ma_config_t *cfg)
{
    if ((f == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if ((cfg->M == 0U) || (cfg->M > (uint8_t) RON_MA_MAX_WINDOW)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->cfg = *cfg;
    filter_reset_ma_state(&f->state);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ma_reset(ron_ma_t *f)
{
    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    filter_reset_ma_state(&f->state);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-115, RON-FR-117 | Test: RON-TC-FILT-008, RON-TC-FILT-010 */
ron_fault_t ron_ma_step(ron_ma_t *f, ron_float_t x, ron_float_t *y)
{
    ron_float_t y_new;

    if ((f == NULL) || (y == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (filter_check_initialised(f->state.is_initialised) != RON_FAULT_NONE) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (f->state.fault_code != RON_FAULT_NONE) {
        *y = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    if (!filter_isfinite(x)) {
        f->state.fault_code = RON_FAULT_INPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }

    y_new = filter_ma_apply(f, x);
    if (!filter_isfinite(y_new)) {
        f->state.fault_code = RON_FAULT_OUTPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    f->state.y_prev = y_new;
    f->state.status = RON_STATUS_OK;
    *y              = y_new;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ma_get_state(const ron_ma_t *f, ron_ma_state_t *state)
{
    if ((f == NULL) || (state == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    *state = f->state;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_init(ron_biquad_t *f, const ron_biquad_config_t *cfg)
{
    if ((f == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!filter_valid_biquad_config(cfg)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->cfg = *cfg;
    filter_reset_biquad_state(&f->state);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_reset(ron_biquad_t *f)
{
    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    filter_reset_biquad_state(&f->state);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_step(ron_biquad_t *f, ron_float_t x, ron_float_t *y)
{
    ron_float_t y_new;

    if ((f == NULL) || (y == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (f->state.fault_code != RON_FAULT_NONE) {
        *y = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    if (!filter_isfinite(x)) {
        f->state.fault_code = RON_FAULT_INPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }

    y_new = filter_biquad_apply(f, x);
    if (!filter_isfinite(y_new)) {
        f->state.fault_code = RON_FAULT_OUTPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    f->state.y_prev = y_new;
    f->state.status = RON_STATUS_OK;
    *y              = y_new;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_get_state(const ron_biquad_t *f, ron_biquad_state_t *state)
{
    if ((f == NULL) || (state == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    *state = f->state;
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_lp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt)
{
    return filter_coeff_common(s, fc, Q, dt, FILTER_KIND_LP);
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_hp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt)
{
    return filter_coeff_common(s, fc, Q, dt, FILTER_KIND_HP);
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_bp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt)
{
    return filter_coeff_common(s, fc, Q, dt, FILTER_KIND_BP);
}

/* Satisfies: RON-FR-122, RON-FR-123 | Test: RON-TC-FILT-013 - RON-TC-FILT-015 */
ron_fault_t ron_biquad_coeff_notch(ron_biquad_section_t *s, ron_float_t f0, ron_float_t Q,
                                   ron_float_t dt)
{
    return filter_coeff_common(s, f0, Q, dt, FILTER_KIND_NOTCH);
}

/* Satisfies: RON-FR-123 | Test: RON-TC-FILT-015 */
ron_fault_t ron_biquad_update_notch(ron_biquad_t *f, uint8_t section_idx, ron_float_t f0,
                                    ron_float_t Q, ron_float_t dt)
{
    ron_biquad_section_t candidate;
    ron_fault_t fault;

    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (f->state.fault_code != RON_FAULT_NONE) {
        f->state.status = RON_STATUS_FAULT;
        return f->state.fault_code;
    }
    if (section_idx >= f->cfg.n_sections) {
        return RON_FAULT_CONFIG_INVALID;
    }

    fault = ron_biquad_coeff_notch(&candidate, f0, Q, dt);
    if (fault == RON_FAULT_NONE) {
        f->cfg.sections[section_idx] = candidate;
    }

    return fault;
}

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_init(ron_ratelim_t *f, const ron_ratelim_config_t *cfg)
{
    if ((f == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!filter_positive(cfg->rate_rise) || !filter_positive(cfg->rate_fall)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->cfg                  = *cfg;
    f->state.y_prev         = RON_FLOAT_C(0.0);
    f->state.fault_code     = RON_FAULT_NONE;
    f->state.status         = RON_STATUS_OK;
    f->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_reset(ron_ratelim_t *f)
{
    if (f == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    f->state.y_prev     = RON_FLOAT_C(0.0);
    f->state.fault_code = RON_FAULT_NONE;
    f->state.status     = RON_STATUS_OK;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_step(ron_ratelim_t *f, ron_float_t x, ron_float_t dt, ron_float_t *y)
{
    ron_float_t lower;
    ron_float_t upper;
    ron_float_t y_new;

    if ((f == NULL) || (y == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (f->state.fault_code != RON_FAULT_NONE) {
        *y = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    if (!filter_isfinite(x) || !filter_isfinite(dt)) {
        f->state.fault_code = RON_FAULT_INPUT_NAN;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }
    if (dt <= RON_FLOAT_C(0.0)) {
        f->state.fault_code = RON_FAULT_CONFIG_INVALID;
        *y                  = f->state.y_prev;
        return filter_step_fault(f->state.fault_code, &f->state.status);
    }

    upper           = f->state.y_prev + (f->cfg.rate_rise * dt);
    lower           = f->state.y_prev - (f->cfg.rate_fall * dt);
    y_new           = ron_clamp(x, lower, upper);
    f->state.status = (ron_fabs(y_new - x) > (RON_FLOAT_EPSILON * RON_FILTER_FOUR))
                          ? RON_STATUS_RATE_LIMITED
                          : RON_STATUS_OK;
    f->state.y_prev = y_new;
    *y              = y_new;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_get_state(const ron_ratelim_t *f, ron_ratelim_state_t *state)
{
    if ((f == NULL) || (state == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!f->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    *state = f->state;
    return RON_FAULT_NONE;
}
