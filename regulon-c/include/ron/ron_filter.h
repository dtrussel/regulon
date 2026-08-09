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

/**
 * @brief Initialise a first-order low-pass filter from a smoothing factor.
 *
 * Implements @c y[n] = y[n-1] + alpha * (x[n] - y[n-1]). Use
 * ron_lp1_init_fc() instead to specify a cutoff frequency directly.
 *
 * @param[out] f    Filter instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration; @c alpha must be finite and in (0, 1].
 *                  Smaller values smooth harder. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @c alpha was outside (0, 1] or not finite.
 */
/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-110 | Test: RON-TC-FILT-001 - RON-TC-FILT-005 */
ron_fault_t ron_lp1_init(ron_lp1_t *f, const ron_lp1_config_t *cfg);

/**
 * @brief Initialise a first-order low-pass filter from a cutoff frequency.
 *
 * Convenience wrapper over ron_lp1_init() that derives the smoothing factor
 * as @c alpha = w / (1 + w) with @c w = 2*pi*fc*dt. The sample period is
 * baked into the coefficient, so a filter initialised this way assumes the
 * loop actually runs at @p dt.
 *
 * @param[out] f   Filter instance to initialise. Must not be NULL.
 * @param[in]  fc  Cutoff frequency in Hz. Must be positive and finite.
 * @param[in]  dt  Sample period in seconds. Must be positive and finite.
 *
 * @retval RON_FAULT_NONE           Filter ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @p fc or @p dt was non-positive or
 *                                  non-finite, or the derived @c alpha was
 *                                  out of range.
 */
/* Satisfies: RON-FR-111 | Test: RON-TC-FILT-006, RON-TC-FILT-007 */
ron_fault_t ron_lp1_init_fc(ron_lp1_t *f, ron_float_t fc, ron_float_t dt);

/**
 * @brief Return the filter to its post-initialisation state.
 *
 * Clears the internal history and any latched fault, keeping the
 * configuration. This is how a latched fault is cleared.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter reset.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_lp1_reset(ron_lp1_t *f);

/**
 * @brief Advance the low-pass filter by one sample.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 * @param[in]     x  Input sample. Must be finite.
 * @param[out]    y  Receives the filtered output. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 * @retval RON_FAULT_INPUT_NAN      @p x was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     The result was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 */
/* Satisfies: RON-FR-102, RON-FR-110 | Test: RON-TC-FILT-003, RON-TC-FILT-005 */
ron_fault_t ron_lp1_step(ron_lp1_t *f, ron_float_t x, ron_float_t *y);

/**
 * @brief Copy out the filter's internal state for inspection or logging.
 *
 * @param[in]  f      Initialised filter instance. Must not be NULL.
 * @param[out] state  Receives a snapshot of the state. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           State copied.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p state was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
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

/**
 * @brief Initialise a moving-average (boxcar FIR) filter.
 *
 * Averages the most recent @c M samples. Until @c M samples have been seen
 * the filter averages only what it has, so the output is usable immediately
 * rather than being held at zero.
 *
 * @param[out] f    Filter instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration; @c M must be in [1, ::RON_MA_MAX_WINDOW].
 *                  Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @c M was zero or exceeded
 *                                  ::RON_MA_MAX_WINDOW.
 */
/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-115, RON-FR-116 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-008, RON-TC-FILT-009 */
ron_fault_t ron_ma_init(ron_ma_t *f, const ron_ma_config_t *cfg);

/**
 * @brief Return the filter to its post-initialisation state.
 *
 * Clears the internal history and any latched fault, keeping the
 * configuration. This is how a latched fault is cleared.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter reset.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ma_reset(ron_ma_t *f);

/**
 * @brief Advance the moving-average filter by one sample.
 *
 * Runs in constant time: the oldest sample is subtracted from a running sum
 * and the new one added, rather than the window being re-summed.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 * @param[in]     x  Input sample. Must be finite.
 * @param[out]    y  Receives the filtered output. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 * @retval RON_FAULT_INPUT_NAN      @p x was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     The result was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 */
/* Satisfies: RON-FR-115, RON-FR-117 | Test: RON-TC-FILT-008, RON-TC-FILT-010 */
ron_fault_t ron_ma_step(ron_ma_t *f, ron_float_t x, ron_float_t *y);

/**
 * @brief Copy out the filter's internal state for inspection or logging.
 *
 * @param[in]  f      Initialised filter instance. Must not be NULL.
 * @param[out] state  Receives a snapshot of the state. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           State copied.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p state was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
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

/**
 * @brief Initialise a cascade of biquad sections.
 *
 * Sections run in series and are evaluated in transposed direct form II,
 * which needs two state words per section and is well behaved numerically in
 * single precision. Design the coefficients with the @c ron_biquad_coeff_*
 * helpers, or supply them directly.
 *
 * @param[out] f    Filter instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration; @c n_sections must be in
 *                  [1, ::RON_BIQUAD_MAX_SECTIONS] and every active section's
 *                  coefficients must be finite. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID @c n_sections was out of range or a
 *                                  coefficient was not finite.
 */
/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_init(ron_biquad_t *f, const ron_biquad_config_t *cfg);

/**
 * @brief Return the filter to its post-initialisation state.
 *
 * Clears the internal history and any latched fault, keeping the
 * configuration. This is how a latched fault is cleared.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter reset.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_reset(ron_biquad_t *f);

/**
 * @brief Advance the biquad cascade by one sample.
 *
 * Each section's output feeds the next; the value returned is the output of
 * the final section.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 * @param[in]     x  Input sample. Must be finite.
 * @param[out]    y  Receives the filtered output. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 * @retval RON_FAULT_INPUT_NAN      @p x was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     The result was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 */
/* Satisfies: RON-FR-120, RON-FR-121 | Test: RON-TC-FILT-011, RON-TC-FILT-012 */
ron_fault_t ron_biquad_step(ron_biquad_t *f, ron_float_t x, ron_float_t *y);

/**
 * @brief Copy out the filter's internal state for inspection or logging.
 *
 * @param[in]  f      Initialised filter instance. Must not be NULL.
 * @param[out] state  Receives a snapshot of the state. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           State copied.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p state was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_biquad_get_state(const ron_biquad_t *f, ron_biquad_state_t *state);

/**
 * @brief Compute second-order low-pass biquad coefficients for one section.
 *
 * Designs the section in place; it does not touch any filter instance, so the
 * result can be inspected or staged before being installed into a
 * ::ron_biquad_config_t.
 *
 * @param[out] s   Section to receive the coefficients. Must not be NULL.
 * @param[in]  fc  Cutoff frequency in Hz. Must be positive, finite, and
 *                 below the Nyquist rate implied by @p dt.
 * @param[in]  Q   Quality factor. Must be positive and finite.
 * @param[in]  dt  Sample period in seconds. Must be positive and finite.
 *
 * @retval RON_FAULT_NONE           Coefficients written to @p s.
 * @retval RON_FAULT_NULL_POINTER   @p s was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A parameter was non-finite, non-positive,
 *                                  or the cutoff was at or above Nyquist.
 */
/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_lp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/**
 * @brief Compute second-order high-pass biquad coefficients for one section.
 *
 * Designs the section in place; it does not touch any filter instance, so the
 * result can be inspected or staged before being installed into a
 * ::ron_biquad_config_t.
 *
 * @param[out] s   Section to receive the coefficients. Must not be NULL.
 * @param[in]  fc  Cutoff frequency in Hz. Must be positive, finite, and
 *                 below the Nyquist rate implied by @p dt.
 * @param[in]  Q   Quality factor. Must be positive and finite.
 * @param[in]  dt  Sample period in seconds. Must be positive and finite.
 *
 * @retval RON_FAULT_NONE           Coefficients written to @p s.
 * @retval RON_FAULT_NULL_POINTER   @p s was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A parameter was non-finite, non-positive,
 *                                  or the cutoff was at or above Nyquist.
 */
/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_hp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/**
 * @brief Compute second-order band-pass biquad coefficients for one section.
 *
 * Designs the section in place; it does not touch any filter instance, so the
 * result can be inspected or staged before being installed into a
 * ::ron_biquad_config_t.
 *
 * @param[out] s   Section to receive the coefficients. Must not be NULL.
 * @param[in]  fc  Centre frequency in Hz. Must be positive, finite, and
 *                 below the Nyquist rate implied by @p dt.
 * @param[in]  Q   Quality factor. Must be positive and finite.
 * @param[in]  dt  Sample period in seconds. Must be positive and finite.
 *
 * @retval RON_FAULT_NONE           Coefficients written to @p s.
 * @retval RON_FAULT_NULL_POINTER   @p s was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A parameter was non-finite, non-positive,
 *                                  or the cutoff was at or above Nyquist.
 */
/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013, RON-TC-FILT-014 */
ron_fault_t ron_biquad_coeff_bp(ron_biquad_section_t *s, ron_float_t fc, ron_float_t Q,
                                ron_float_t dt);

/**
 * @brief Compute second-order notch biquad coefficients for one section.
 *
 * Designs the section in place; it does not touch any filter instance, so the
 * result can be inspected or staged before being installed into a
 * ::ron_biquad_config_t.
 *
 * @param[out] s   Section to receive the coefficients. Must not be NULL.
 * @param[in]  f0  Notch centre frequency in Hz. Must be positive, finite, and
 *                 below the Nyquist rate implied by @p dt.
 * @param[in]  Q   Quality factor. Must be positive and finite.
 * @param[in]  dt  Sample period in seconds. Must be positive and finite.
 *
 * @retval RON_FAULT_NONE           Coefficients written to @p s.
 * @retval RON_FAULT_NULL_POINTER   @p s was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A parameter was non-finite, non-positive,
 *                                  or the cutoff was at or above Nyquist.
 */
/* Satisfies: RON-FR-122, RON-FR-123 | Test: RON-TC-FILT-013 - RON-TC-FILT-015 */
ron_fault_t ron_biquad_coeff_notch(ron_biquad_section_t *s, ron_float_t f0, ron_float_t Q,
                                   ron_float_t dt);

/**
 * @brief Retune one notch section of an initialised cascade in place.
 *
 * Intended for tracking a disturbance whose frequency moves at runtime, such
 * as a notch following shaft speed. The new coefficients are computed into a
 * scratch section first and only installed once they validate, so a rejected
 * update leaves the running filter untouched.
 *
 * The section's state is deliberately **not** cleared, which keeps the output
 * continuous across a retune; expect a short transient while the state
 * settles into the new response.
 *
 * @param[in,out] f            Initialised filter instance. Must not be NULL.
 * @param[in]     section_idx  Section to retune. Must be less than the
 *                             configured @c n_sections.
 * @param[in]     f0           New notch centre frequency in Hz. Must be
 *                             positive, finite, and below Nyquist.
 * @param[in]     Q            New quality factor. Must be positive and finite.
 * @param[in]     dt           Sample period in seconds. Must be positive and
 *                             finite.
 *
 * @retval RON_FAULT_NONE           Section retuned.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised,
 *                                  @p section_idx was out of range, or the
 *                                  requested design was invalid.
 * @retval other                    The filter's latched fault, if one is
 *                                  already set; the update is not applied.
 */
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

/**
 * @brief Initialise a standalone asymmetric rate limiter.
 *
 * Bounds how fast a signal may change, with independent rise and fall rates.
 * Both are given as positive magnitudes in units per second, so a limiter
 * that may rise slowly but fall quickly sets @c rate_fall larger than
 * @c rate_rise.
 *
 * @param[out] f    Limiter instance to initialise. Must not be NULL.
 * @param[in]  cfg  Configuration; @c rate_rise and @c rate_fall must both be
 *                  positive and finite. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Limiter ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A rate was non-positive or non-finite.
 */
/* Satisfies: RON-FR-100 - RON-FR-103, RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-001 - RON-TC-FILT-004, RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_init(ron_ratelim_t *f, const ron_ratelim_config_t *cfg);

/**
 * @brief Return the filter to its post-initialisation state.
 *
 * Clears the internal history and any latched fault, keeping the
 * configuration. This is how a latched fault is cleared.
 *
 * @param[in,out] f  Initialised filter instance. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Filter reset.
 * @retval RON_FAULT_NULL_POINTER   @p f was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_reset(ron_ratelim_t *f);

/**
 * @brief Advance the rate limiter by one sample.
 *
 * The permitted change is @c rate * dt, so the limiter behaves correctly when
 * the loop period varies.
 *
 * @param[in,out] f   Initialised limiter instance. Must not be NULL.
 * @param[in]     x   Requested value. Must be finite.
 * @param[in]     dt  Elapsed time since the previous call, in seconds. Must
 *                    be positive and finite.
 * @param[out]    y   Receives the rate-limited value. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           Output computed normally.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p y was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The limiter was never initialised, or
 *                                  @p dt was not positive.
 * @retval RON_FAULT_INPUT_NAN      @p x or @p dt was not finite; @p *y holds
 *                                  the previous output and the fault latches.
 * @retval RON_FAULT_OUTPUT_NAN     The result was not finite; @p *y holds the
 *                                  previous output and the fault latches.
 */
/* Satisfies: RON-FR-130, RON-FR-131 | Test: RON-TC-FILT-016, RON-TC-FILT-017 */
ron_fault_t ron_ratelim_step(ron_ratelim_t *f, ron_float_t x, ron_float_t dt, ron_float_t *y);

/**
 * @brief Copy out the filter's internal state for inspection or logging.
 *
 * @param[in]  f      Initialised filter instance. Must not be NULL.
 * @param[out] state  Receives a snapshot of the state. Must not be NULL.
 *
 * @retval RON_FAULT_NONE           State copied.
 * @retval RON_FAULT_NULL_POINTER   @p f or @p state was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The filter was never initialised.
 */
/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
ron_fault_t ron_ratelim_get_state(const ron_ratelim_t *f, ron_ratelim_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* RON_FILTER_H */
