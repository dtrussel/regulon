/*
 * @file     filter_bounded_execution_proof.c
 * @brief    CBMC harness for bounded filter execution.
 * @module   filter_bounded_execution_proof
 * @doc      RON-TP-001
 * @req      RON-FR-101, RON-FR-117, RON-FR-121
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "filter_proof_common.h"

/* Satisfies: RON-FR-101, RON-FR-117, RON-FR-121 | Test: RON-TC-FILT-010-FV, RON-TC-FILT-012-FV */
void filter_bounded_execution_proof(void)
{
    ron_ma_t ma;
    ron_biquad_t biquad;
    ron_ratelim_t rate;
    ron_ma_config_t ma_cfg         = filter_proof_ma_config();
    ron_biquad_config_t biquad_cfg = filter_proof_biquad_config();
    ron_ratelim_config_t rate_cfg  = filter_proof_ratelim_config();
    ron_float_t y                  = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_ma_init(&ma, &ma_cfg) == RON_FAULT_NONE, "moving average initialises");
    __CPROVER_assert(ron_ma_step(&ma, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "moving average step terminates within static window bound");
    __CPROVER_assert(ron_biquad_init(&biquad, &biquad_cfg) == RON_FAULT_NONE, "biquad initialises");
    __CPROVER_assert(ron_biquad_step(&biquad, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "biquad step terminates within static section bound");
    __CPROVER_assert(ron_ratelim_init(&rate, &rate_cfg) == RON_FAULT_NONE,
                     "rate limiter initialises");
    __CPROVER_assert(ron_ratelim_step(&rate, FILTER_PROOF_SIGNAL, FILTER_PROOF_DT, &y) ==
                         RON_FAULT_NONE,
                     "rate limiter step terminates without loops");
}
