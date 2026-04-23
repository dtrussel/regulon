/*
 * @file     ron_filter.h
 * @brief    Public API for Regulon signal conditioning filters.
 * @module   ron_filter
 * @doc      RON-IS-001
 * @req      RON-FR-100, RON-FR-101, RON-FR-102, RON-FR-103,
 *           RON-FR-110, RON-FR-111, RON-FR-115, RON-FR-116,
 *           RON-FR-117, RON-FR-120, RON-FR-121, RON-FR-122,
 *           RON-FR-123, RON-FR-130, RON-FR-131
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_FILTER_H
#define RON_FILTER_H

#include "ron/ron_pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * First-order IIR low-pass filter
 * ========================================================================= */

typedef struct {
    ron_float_t alpha; /**< Exponential smoothing factor in (0, 1]. */
} ron_lp1_config_t;

typedef struct {
    ron_float_t y_prev;
    ron_fault_t fault_code;
    ron_status_t status;
    bool is_initialised;
} ron_lp1_state_t;

typedef struct {
    ron_lp1_config_t cfg;
    ron_lp1_state_t state;
} ron_lp1_t;

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-110 | Test: RON-TC-FILT-001 - RON-TC-FILT-005 */
ron_fault_t ron_lp1_init(ron_lp1_t *f, const ron_lp1_config_t *cfg);

/* Satisfies: RON-FR-111 | Test: RON-TC-FILT-006, RON-TC-FILT-007 */
ron_fault_t ron_lp1_init_fc(ron_lp1_t *f, ron_float_t fc, ron_float_t dt);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_lp1_reset(ron_lp1_t *f);

/* Satisfies: RON-FR-102, RON-FR-110 | Test: RON-TC-FILT-003, RON-TC-FILT-005 */
ron_fault_t ron_lp1_step(ron_lp1_t *f, ron_float_t x, ron_float_t *y);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_lp1_get_state(const ron_lp1_t *f, ron_lp1_state_t *state);

/* =========================================================================
 * Moving-average FIR filter
 * ========================================================================= */

typedef struct {
    uint8_t M; /**< Active window length, bounded by RON_MA_MAX_WINDOW. */
} ron_ma_config_t;

typedef struct {
    ron_float_t buf[RON_MA_MAX_WINDOW];
    ron_float_t sum;
    ron_float_t y_prev;
    uint8_t idx;
    uint8_t count;
    ron_fault_t fault_code;
    ron_status_t status;
    bool is_initialised;
} ron_ma_state_t;

typedef struct {
    ron_ma_config_t cfg;
    ron_ma_state_t state;
} ron_ma_t;

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-115, RON-FR-116 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-008, RON-TC-FILT-009 */
ron_fault_t ron_ma_init(ron_ma_t *f, const ron_ma_config_t *cfg);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ma_reset(ron_ma_t *f);

/* Satisfies: RON-FR-115, RON-FR-117 | Test: RON-TC-FILT-008, RON-TC-FILT-010 */
ron_fault_t ron_ma_step(ron_ma_t *f, ron_float_t x, ron_float_t *y);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ma_get_state(const ron_ma_t *f, ron_ma_state_t *state);

/* =========================================================================
 * Cascaded biquad IIR filter
 * ========================================================================= */

typedef struct {
    ron_float_t b0;
    ron_float_t b1;
    ron_float_t b2;
    ron_float_t a1;
    ron_float_t a2;
} ron_biquad_section_t;

typedef struct {
    ron_biquad_section_t sections[RON_BIQUAD_MAX_SECTIONS];
    uint8_t n_sections;
} ron_biquad_config_t;

typedef struct {
    ron_float_t w1[RON_BIQUAD_MAX_SECTIONS];
    ron_float_t w2[RON_BIQUAD_MAX_SECTIONS];
    ron_float_t y_prev;
    ron_fault_t fault_code;
    ron_status_t status;
    bool is_initialised;
} ron_biquad_state_t;

typedef struct {
    ron_biquad_config_t cfg;
    ron_biquad_state_t state;
} ron_biquad_t;

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_init(ron_biquad_t *f, const ron_biquad_config_t *cfg);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_reset(ron_biquad_t *f);

/* Satisfies: RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_step(ron_biquad_t *f, ron_float_t x, ron_float_t *y);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_get_state(const ron_biquad_t *f, ron_biquad_state_t *state);

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_lp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_hp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_bp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/* Satisfies: RON-FR-122, RON-FR-123 | Test: RON-TC-FILT-013 - RON-TC-FILT-015 */
ron_fault_t ron_biquad_coeff_notch(ron_biquad_section_t *s, ron_float_t f0, ron_float_t Q,
                                   ron_float_t dt);

/* Satisfies: RON-FR-123 | Test: RON-TC-FILT-015 */
ron_fault_t ron_biquad_update_notch(ron_biquad_t *f, uint8_t section_idx, ron_float_t f0,
                                    ron_float_t Q, ron_float_t dt);

/* =========================================================================
 * Standalone asymmetric rate limiter
 * ========================================================================= */

typedef struct {
    ron_float_t rate_rise; /**< Maximum positive delta per second. */
    ron_float_t rate_fall; /**< Maximum negative delta per second, as a positive value. */
} ron_ratelim_config_t;

typedef struct {
    ron_float_t y_prev;
    ron_fault_t fault_code;
    ron_status_t status;
    bool is_initialised;
} ron_ratelim_state_t;

typedef struct {
    ron_ratelim_config_t cfg;
    ron_ratelim_state_t state;
} ron_ratelim_t;

/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_init(ron_ratelim_t *f, const ron_ratelim_config_t *cfg);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_reset(ron_ratelim_t *f);

/* Satisfies: RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_step(ron_ratelim_t *f, ron_float_t x, ron_float_t dt, ron_float_t *y);

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_get_state(const ron_ratelim_t *f, ron_ratelim_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* RON_FILTER_H */
