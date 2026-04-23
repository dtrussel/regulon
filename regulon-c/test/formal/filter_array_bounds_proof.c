/*
 * @file     filter_array_bounds_proof.c
 * @brief    CBMC harness for filter bounded array access.
 * @module   filter_array_bounds_proof
 * @doc      RON-TP-001
 * @req      RON-FR-116, RON-FR-121, RON-SR-005
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "filter_proof_common.h"

/* Satisfies: RON-FR-116, RON-FR-121, RON-SR-005 | Test: RON-TC-FILT-009-FV, RON-TC-FILT-012-FV */
void filter_array_bounds_proof(void)
{
    ron_ma_t ma;
    ron_biquad_t biquad;
    ron_ma_config_t ma_cfg         = filter_proof_ma_config();
    ron_biquad_config_t biquad_cfg = filter_proof_biquad_config();
    ron_float_t y                  = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_ma_init(&ma, &ma_cfg) == RON_FAULT_NONE,
                     "valid moving-average config initialises");
    __CPROVER_assert(ron_ma_step(&ma, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "moving-average step has bounded buffer access");
    __CPROVER_assert(ron_biquad_init(&biquad, &biquad_cfg) == RON_FAULT_NONE,
                     "valid biquad config initialises");
    __CPROVER_assert(ron_biquad_step(&biquad, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "biquad step has bounded section access");
}
