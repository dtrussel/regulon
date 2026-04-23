/*
 * @file     test_ron_filter.c
 * @brief    Unit tests for Regulon signal conditioning filters.
 * @module   test_ron_filter
 * @doc      RON-TP-001
 * @req      RON-FR-100, RON-FR-101, RON-FR-102, RON-FR-103,
 *           RON-FR-110, RON-FR-111, RON-FR-115, RON-FR-116,
 *           RON-FR-117, RON-FR-120, RON-FR-121, RON-FR-122,
 *           RON-FR-123, RON-FR-130, RON-FR-131
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <math.h>

#include "ron/ron_filter.h"

#include "unity.h"

#define TEST_FILTER_TOL RON_FLOAT_C(0.001)
#define TEST_FILTER_TIGHT_TOL RON_FLOAT_C(0.00001)
#define TEST_FILTER_DT RON_FLOAT_C(0.001)
#define TEST_FILTER_Q RON_FLOAT_C(0.70710678)
#define TEST_FILTER_SPIKE_LIMIT RON_FLOAT_C(3.0)

void setUp(void)
{
}

void tearDown(void)
{
}

/* Satisfies: RON-FR-100, RON-FR-101 | Test: RON-TC-FILT-001 */
void test_ron_tc_filt_001_filters_are_independent_components(void)
{
    ron_lp1_t lp1;
    ron_ma_t ma;
    ron_ratelim_t rate;
    ron_lp1_config_t lp1_cfg      = {RON_FLOAT_C(0.5)};
    ron_ma_config_t ma_cfg        = {4U};
    ron_ratelim_config_t rate_cfg = {RON_FLOAT_C(1.0), RON_FLOAT_C(1.0)};
    ron_float_t lp1_y             = RON_FLOAT_C(0.0);
    ron_float_t ma_y              = RON_FLOAT_C(0.0);
    ron_float_t rate_y            = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&lp1, &lp1_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&ma, &ma_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&rate, &rate_cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&lp1, RON_FLOAT_C(2.0), &lp1_y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&ma, RON_FLOAT_C(2.0), &ma_y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&rate, RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), &rate_y));

    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.0), lp1_y);
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.5), ma_y);
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.5), rate_y);
}

/* Satisfies: RON-FR-101 | Test: RON-TC-FILT-002 */
void test_ron_tc_filt_002_instance_config_state_model(void)
{
    ron_lp1_t lp1;
    ron_lp1_state_t state;
    ron_lp1_config_t cfg = {RON_FLOAT_C(0.25)};
    ron_float_t y        = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&lp1, &cfg));
    cfg.alpha = RON_FLOAT_C(1.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&lp1, RON_FLOAT_C(4.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.0), y);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_get_state(&lp1, &state));
    TEST_ASSERT_TRUE(state.is_initialised);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, state.fault_code);
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, state.status);
}

/* Satisfies: RON-FR-102 | Test: RON-TC-FILT-003 */
void test_ron_tc_filt_003_lifecycle_reset_and_get_state(void)
{
    ron_lp1_t lp1       = {0};
    ron_ma_t ma         = {0};
    ron_biquad_t biquad = {0};
    ron_ratelim_t rate  = {0};
    ron_lp1_state_t lp_state;
    ron_ma_state_t ma_state;
    ron_biquad_state_t biquad_state;
    ron_ratelim_state_t rate_state;
    ron_lp1_config_t lp_cfg        = {RON_FLOAT_C(0.5)};
    ron_ma_config_t ma_cfg         = {3U};
    ron_biquad_config_t biquad_cfg = {{{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                      1U};
    ron_ratelim_config_t rate_cfg  = {RON_FLOAT_C(1.0), RON_FLOAT_C(1.0)};
    ron_float_t y                  = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&lp1, &lp_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&ma, &ma_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &biquad_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&rate, &rate_cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&lp1, RON_FLOAT_C(2.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&ma, RON_FLOAT_C(3.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&biquad, RON_FLOAT_C(4.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&rate, RON_FLOAT_C(5.0), RON_FLOAT_C(1.0), &y));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_reset(&lp1));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_reset(&ma));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_reset(&biquad));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_reset(&rate));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_get_state(&lp1, &lp_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_get_state(&ma, &ma_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_get_state(&biquad, &biquad_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_get_state(&rate, &rate_state));

    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.0), lp_state.y_prev);
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.0), ma_state.sum);
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.0), biquad_state.w1[0]);
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.0), rate_state.y_prev);
}

/* Satisfies: RON-FR-103 | Test: RON-TC-FILT-004 */
void test_ron_tc_filt_004_rejects_invalid_coefficients_and_inputs(void)
{
    ron_lp1_t lp1                        = {0};
    ron_ma_t ma                          = {0};
    ron_biquad_t biquad                  = {0};
    ron_ratelim_t rate                   = {0};
    ron_lp1_config_t bad_lp_low          = {RON_FLOAT_C(0.0)};
    ron_lp1_config_t bad_lp_high         = {RON_FLOAT_C(1.5)};
    ron_ma_config_t bad_ma_zero          = {0U};
    ron_ma_config_t bad_ma_large         = {(uint8_t) (RON_MA_MAX_WINDOW + 1U)};
    ron_biquad_config_t bad_biquad_count = {{{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                              RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                            0U};
    ron_biquad_config_t bad_biquad_unstable = {
        {{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(2.0),
          RON_FLOAT_C(1.0)}},
        1U};
    ron_ratelim_config_t bad_rate = {RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)};
    ron_float_t y                 = RON_FLOAT_C(0.0);
    ron_float_t nonfinite         = RON_FLOAT_MAX;

    nonfinite *= RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_init(NULL, &bad_lp_low));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_init(&lp1, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_lp1_init(&lp1, &bad_lp_low));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_lp1_init(&lp1, &bad_lp_high));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_lp1_init_fc(&lp1, RON_FLOAT_C(0.0), TEST_FILTER_DT));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ma_init(&ma, &bad_ma_zero));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ma_init(&ma, &bad_ma_large));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &bad_biquad_count));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_biquad_init(&biquad, &bad_biquad_unstable));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ratelim_init(&rate, &bad_rate));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_lp1_reset(&lp1));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ma_reset(&ma));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_reset(&biquad));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ratelim_reset(&rate));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_step(NULL, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_step(NULL, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_step(NULL, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_ratelim_step(NULL, RON_FLOAT_C(1.0), TEST_FILTER_DT, &y));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_lp1_init(&lp1, &(ron_lp1_config_t){RON_FLOAT_C(0.5)}));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_lp1_step(&lp1, nonfinite, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_lp1_step(&lp1, RON_FLOAT_C(1.0), &y));
}

/* Satisfies: RON-FR-103 | Test: RON-TC-FILT-004 */
void test_ron_tc_filt_004_defensive_api_branches(void)
{
    ron_lp1_t lp1       = {0};
    ron_ma_t ma         = {0};
    ron_biquad_t biquad = {0};
    ron_ratelim_t rate  = {0};
    ron_lp1_state_t lp_state;
    ron_ma_state_t ma_state;
    ron_biquad_state_t biquad_state;
    ron_ratelim_state_t rate_state;
    ron_float_t y                         = RON_FLOAT_C(0.0);
    ron_float_t nan_value                 = (ron_float_t) NAN;
    ron_float_t neg_nonfinite             = (ron_float_t) (-INFINITY);
    ron_lp1_config_t lp_cfg               = {RON_FLOAT_C(0.5)};
    ron_ma_config_t ma_cfg                = {2U};
    ron_ratelim_config_t rate_cfg         = {RON_FLOAT_C(1.0), RON_FLOAT_C(1.0)};
    ron_ratelim_config_t bad_fall_cfg     = {RON_FLOAT_C(1.0), RON_FLOAT_C(0.0)};
    ron_ratelim_config_t bad_rise_nan_cfg = {nan_value, RON_FLOAT_C(1.0)};
    ron_biquad_config_t max_count_cfg     = {{{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                               RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                             (uint8_t) (RON_BIQUAD_MAX_SECTIONS + 1U)};
    ron_biquad_config_t nan_coeff_cfg     = {
        {{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), nan_value, RON_FLOAT_C(0.0)}}, 1U};
    ron_biquad_config_t nan_b0_cfg = {
        {{nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}}, 1U};
    ron_biquad_config_t nan_b1_cfg = {
        {{RON_FLOAT_C(1.0), nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}}, 1U};
    ron_biquad_config_t nan_b2_cfg = {
        {{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), nan_value, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}}, 1U};
    ron_biquad_config_t nan_a2_cfg = {
        {{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), nan_value}}, 1U};
    ron_biquad_config_t b1_only_cfg     = {{{RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                           1U};
    ron_biquad_config_t b2_only_cfg     = {{{RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                           1U};
    ron_biquad_config_t zero_b_cfg      = {{{RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                                           1U};
    ron_biquad_config_t a2_unstable_cfg = {{{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                             RON_FLOAT_C(0.0), RON_FLOAT_C(1.0)}},
                                           1U};
    ron_biquad_config_t first_jury_unstable_cfg = {
        {{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-2.0),
          RON_FLOAT_C(1.0)}},
        1U};
    ron_biquad_config_t high_gain_cfg = {
        {{RON_FLOAT_MAX, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
        1U};

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_init(NULL, &ma_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_init(&ma, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_init(NULL, &b1_only_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_init(&biquad, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ratelim_init(NULL, &rate_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ratelim_init(&rate, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ratelim_init(&rate, &bad_fall_cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ratelim_reset(NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_get_state(NULL, &lp_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_get_state(NULL, &ma_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_get_state(NULL, &biquad_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ratelim_get_state(NULL, &rate_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_get_state(&lp1, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_get_state(&ma, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_biquad_get_state(&biquad, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ratelim_get_state(&rate, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_lp1_get_state(&lp1, &lp_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ma_get_state(&ma, &ma_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_get_state(&biquad, &biquad_state));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ratelim_get_state(&rate, &rate_state));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ma_step(&ma, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_biquad_step(&biquad, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_ratelim_step(&rate, RON_FLOAT_C(1.0), TEST_FILTER_DT, &y));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &max_count_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &nan_coeff_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &nan_b0_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &nan_b1_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &nan_b2_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &nan_a2_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &zero_b_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_biquad_init(&biquad, &a2_unstable_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_biquad_init(&biquad, &first_jury_unstable_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &b1_only_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &b2_only_cfg));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_lp1_step(&lp1, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&lp1, &lp_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_step(&lp1, RON_FLOAT_C(1.0), NULL));
    lp1.cfg.alpha = RON_FLOAT_MAX;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_OUTPUT_NAN, ron_lp1_step(&lp1, RON_FLOAT_MAX, &y));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&ma, &ma_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ma_step(&ma, RON_FLOAT_C(1.0), NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_ma_step(&ma, neg_nonfinite, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_ma_step(&ma, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_reset(&ma));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&ma, RON_FLOAT_MAX, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_OUTPUT_NAN, ron_ma_step(&ma, RON_FLOAT_MAX, &y));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &high_gain_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_biquad_step(&biquad, RON_FLOAT_C(1.0), NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_biquad_step(&biquad, nan_value, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN, ron_biquad_step(&biquad, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &high_gain_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_OUTPUT_NAN, ron_biquad_step(&biquad, RON_FLOAT_C(2.0), &y));

    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_biquad_update_notch(NULL, 0U, RON_FLOAT_C(100.0), TEST_FILTER_Q, TEST_FILTER_DT));
    biquad.state.is_initialised = false;
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_update_notch(&biquad, 0U, RON_FLOAT_C(100.0), TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &b1_only_cfg));
    biquad.state.fault_code = RON_FAULT_INPUT_NAN;
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_INPUT_NAN,
        ron_biquad_update_notch(&biquad, 0U, RON_FLOAT_C(100.0), TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&biquad, &b1_only_cfg));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_update_notch(&biquad, 0U, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), TEST_FILTER_DT));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&rate, &rate_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_ratelim_step(&rate, RON_FLOAT_C(1.0), TEST_FILTER_DT, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
                            ron_ratelim_step(&rate, nan_value, TEST_FILTER_DT, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
                            ron_ratelim_step(&rate, RON_FLOAT_C(0.0), TEST_FILTER_DT, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&rate, &rate_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_INPUT_NAN,
                            ron_ratelim_step(&rate, RON_FLOAT_C(0.0), nan_value, &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_ratelim_init(&rate, &bad_rise_nan_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&rate, &rate_cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&rate, RON_FLOAT_C(0.25), RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_get_state(&rate, &rate_state));
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_OK, rate_state.status);
}

/* Satisfies: RON-FR-110 | Test: RON-TC-FILT-005 */
void test_ron_tc_filt_005_lp1_step_response(void)
{
    ron_lp1_t filter;
    ron_lp1_config_t cfg = {RON_FLOAT_C(0.1)};
    ron_float_t y        = RON_FLOAT_C(0.0);
    uint16_t idx;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&filter, &cfg));
    idx = 0U;
    while (idx < 10U) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&filter, RON_FLOAT_C(0.0), &y));
        idx++;
    }
    idx = 0U;
    while (idx < 200U) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&filter, RON_FLOAT_C(1.0), &y));
        if (idx == 9U) {
            TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TOL, RON_FLOAT_C(0.651322), y);
        }
        idx++;
    }
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.01), RON_FLOAT_C(1.0), y);
}

/* Satisfies: RON-FR-111 | Test: RON-TC-FILT-006 */
void test_ron_tc_filt_006_lp1_frequency_configuration_matches_alpha(void)
{
    ron_lp1_t fc_filter;
    ron_lp1_t alpha_filter;
    ron_lp1_config_t alpha_cfg;
    ron_float_t omega   = RON_FLOAT_C(6.28318530717958647692) * RON_FLOAT_C(10.0) * TEST_FILTER_DT;
    ron_float_t y_fc    = RON_FLOAT_C(0.0);
    ron_float_t y_alpha = RON_FLOAT_C(0.0);
    uint16_t idx;

    alpha_cfg.alpha = omega / (RON_FLOAT_C(1.0) + omega);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_lp1_init_fc(&fc_filter, RON_FLOAT_C(10.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&alpha_filter, &alpha_cfg));

    idx = 0U;
    while (idx < 500U) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_step(&fc_filter, RON_FLOAT_C(1.0), &y_fc));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_lp1_step(&alpha_filter, RON_FLOAT_C(1.0), &y_alpha));
        TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TOL, y_alpha, y_fc);
        idx++;
    }
}

/* Satisfies: RON-FR-111 | Test: RON-TC-FILT-007 */
void test_ron_tc_filt_007_lp1_cutoff_configuration_rejects_invalid_sample_time(void)
{
    ron_lp1_t filter;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_lp1_init_fc(NULL, RON_FLOAT_C(10.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_lp1_init_fc(&filter, RON_FLOAT_C(10.0), RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_lp1_init_fc(&filter, RON_FLOAT_C(-1.0), TEST_FILTER_DT));
}

/* Satisfies: RON-FR-115 | Test: RON-TC-FILT-008 */
void test_ron_tc_filt_008_moving_average_outputs_boxcar_mean(void)
{
    ron_ma_t filter;
    ron_ma_config_t cfg = {4U};
    ron_float_t y       = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.25), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(2.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.75), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(3.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.5), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(4.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(2.5), y);
}

/* Satisfies: RON-FR-116 | Test: RON-TC-FILT-009 */
void test_ron_tc_filt_009_moving_average_storage_is_bounded(void)
{
    ron_ma_t filter;
    ron_ma_state_t state;
    ron_ma_config_t cfg = {(uint8_t) RON_MA_MAX_WINDOW};

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_get_state(&filter, &state));
    TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_MA_MAX_WINDOW, filter.cfg.M);
    TEST_ASSERT_EQUAL_UINT8(0U, state.idx);
    TEST_ASSERT_EQUAL_UINT8(0U, state.count);
}

/* Satisfies: RON-FR-117 | Test: RON-TC-FILT-010 */
void test_ron_tc_filt_010_moving_average_uses_sliding_sum(void)
{
    ron_ma_t filter;
    ron_ma_state_t state;
    ron_ma_config_t cfg = {3U};
    ron_float_t y       = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(3.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(6.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(9.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(6.0), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_step(&filter, RON_FLOAT_C(12.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(9.0), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ma_get_state(&filter, &state));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(27.0), state.sum);
}

/* Satisfies: RON-FR-120 | Test: RON-TC-FILT-011 */
void test_ron_tc_filt_011_biquad_direct_form_known_transfer(void)
{
    ron_biquad_t filter;
    ron_biquad_config_t cfg = {{{RON_FLOAT_C(0.5), RON_FLOAT_C(0.25), RON_FLOAT_C(0.125),
                                 RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                               1U};
    ron_float_t y           = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.5), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(0.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.25), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(0.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(0.125), y);
}

/* Satisfies: RON-FR-121 | Test: RON-TC-FILT-012 */
void test_ron_tc_filt_012_biquad_cascaded_sections_apply_in_series(void)
{
    ron_biquad_t filter;
    ron_biquad_config_t cfg = {
        {{RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)},
         {RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
          RON_FLOAT_C(0.0)}},
        2U};
    ron_float_t y = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(4.0), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.0), y);
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-013 */
void test_ron_tc_filt_013_biquad_coefficient_helpers_generate_valid_sections(void)
{
    ron_biquad_section_t lp;
    ron_biquad_section_t hp;
    ron_biquad_section_t bp;
    ron_biquad_section_t notch;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_coeff_lp(&lp, RON_FLOAT_C(100.0),
                                                                TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_coeff_hp(&hp, RON_FLOAT_C(100.0),
                                                                TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_coeff_bp(&bp, RON_FLOAT_C(100.0),
                                                                TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_coeff_notch(&notch, RON_FLOAT_C(60.0),
                                                                   TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_TRUE(lp.b0 > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(hp.b0 > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(bp.b0 > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(notch.b0 > RON_FLOAT_C(0.0));
}

/* Satisfies: RON-FR-122 | Test: RON-TC-FILT-014 */
void test_ron_tc_filt_014_biquad_coefficient_helpers_reject_invalid_inputs(void)
{
    ron_biquad_section_t section;

    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_biquad_coeff_lp(NULL, RON_FLOAT_C(100.0), TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_coeff_lp(&section, RON_FLOAT_C(0.0), TEST_FILTER_Q, TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_coeff_hp(&section, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_coeff_bp(&section, RON_FLOAT_C(100.0), TEST_FILTER_Q, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_coeff_notch(&section, RON_FLOAT_C(500.0), TEST_FILTER_Q, TEST_FILTER_DT));
}

/* Satisfies: RON-FR-123 | Test: RON-TC-FILT-015 */
void test_ron_tc_filt_015_notch_runtime_update_preserves_state(void)
{
    ron_biquad_t filter;
    ron_biquad_config_t cfg;
    ron_biquad_state_t before;
    ron_biquad_state_t after;
    ron_float_t y = RON_FLOAT_C(0.0);
    ron_float_t amplitude;

    cfg.n_sections = 1U;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_biquad_coeff_notch(&cfg.sections[0], RON_FLOAT_C(60.0),
                                                   RON_FLOAT_C(4.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_init(&filter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_get_state(&filter, &before));
    amplitude = ron_fabs(y);

    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE,
        ron_biquad_update_notch(&filter, 0U, RON_FLOAT_C(120.0), RON_FLOAT_C(4.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_get_state(&filter, &after));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, before.w1[0], after.w1[0]);
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_biquad_update_notch(&filter, 1U, RON_FLOAT_C(120.0), RON_FLOAT_C(4.0), TEST_FILTER_DT));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_biquad_step(&filter, RON_FLOAT_C(1.0), &y));
    TEST_ASSERT_TRUE(ron_fabs(y) <= (TEST_FILTER_SPIKE_LIMIT * (amplitude + RON_FLOAT_C(1.0))));
}

/* Satisfies: RON-FR-130 | Test: RON-TC-FILT-016 */
void test_ron_tc_filt_016_rate_limiter_clamps_change_per_sample(void)
{
    ron_ratelim_t limiter;
    ron_ratelim_config_t cfg = {RON_FLOAT_C(2.0), RON_FLOAT_C(2.0)};
    ron_ratelim_state_t state;
    ron_float_t y = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&limiter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&limiter, RON_FLOAT_C(10.0), RON_FLOAT_C(0.5), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.0), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_get_state(&limiter, &state));
    TEST_ASSERT_EQUAL_UINT16(RON_STATUS_RATE_LIMITED, state.status);
}

/* Satisfies: RON-FR-131 | Test: RON-TC-FILT-017 */
void test_ron_tc_filt_017_rate_limiter_supports_asymmetric_limits(void)
{
    ron_ratelim_t limiter;
    ron_ratelim_config_t cfg = {RON_FLOAT_C(4.0), RON_FLOAT_C(1.0)};
    ron_float_t y            = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ratelim_init(&limiter, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&limiter, RON_FLOAT_C(10.0), RON_FLOAT_C(0.5), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(2.0), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_ratelim_step(&limiter, RON_FLOAT_C(-10.0), RON_FLOAT_C(0.5), &y));
    TEST_ASSERT_FLOAT_WITHIN(TEST_FILTER_TIGHT_TOL, RON_FLOAT_C(1.5), y);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_ratelim_step(&limiter, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &y));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_filt_001_filters_are_independent_components);
    RUN_TEST(test_ron_tc_filt_002_instance_config_state_model);
    RUN_TEST(test_ron_tc_filt_003_lifecycle_reset_and_get_state);
    RUN_TEST(test_ron_tc_filt_004_rejects_invalid_coefficients_and_inputs);
    RUN_TEST(test_ron_tc_filt_004_defensive_api_branches);
    RUN_TEST(test_ron_tc_filt_005_lp1_step_response);
    RUN_TEST(test_ron_tc_filt_006_lp1_frequency_configuration_matches_alpha);
    RUN_TEST(test_ron_tc_filt_007_lp1_cutoff_configuration_rejects_invalid_sample_time);
    RUN_TEST(test_ron_tc_filt_008_moving_average_outputs_boxcar_mean);
    RUN_TEST(test_ron_tc_filt_009_moving_average_storage_is_bounded);
    RUN_TEST(test_ron_tc_filt_010_moving_average_uses_sliding_sum);
    RUN_TEST(test_ron_tc_filt_011_biquad_direct_form_known_transfer);
    RUN_TEST(test_ron_tc_filt_012_biquad_cascaded_sections_apply_in_series);
    RUN_TEST(test_ron_tc_filt_013_biquad_coefficient_helpers_generate_valid_sections);
    RUN_TEST(test_ron_tc_filt_014_biquad_coefficient_helpers_reject_invalid_inputs);
    RUN_TEST(test_ron_tc_filt_015_notch_runtime_update_preserves_state);
    RUN_TEST(test_ron_tc_filt_016_rate_limiter_clamps_change_per_sample);
    RUN_TEST(test_ron_tc_filt_017_rate_limiter_supports_asymmetric_limits);
    return UNITY_END();
}
