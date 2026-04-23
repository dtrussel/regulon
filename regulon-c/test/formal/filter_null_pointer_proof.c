/*
 * @file     filter_null_pointer_proof.c
 * @brief    CBMC harness for filter null-pointer guards.
 * @module   filter_null_pointer_proof
 * @doc      RON-TP-001
 * @req      RON-FR-102, RON-SR-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "filter_proof_common.h"

/* Satisfies: RON-FR-102, RON-SR-001 | Test: RON-TC-FILT-003-FV */
void filter_null_pointer_proof(void)
{
    ron_lp1_t lp1;
    ron_lp1_config_t cfg = filter_proof_lp1_config();
    ron_float_t y        = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_lp1_init(0, &cfg) == RON_FAULT_NULL_POINTER,
                     "lp1 init rejects null instance");
    __CPROVER_assert(ron_lp1_init(&lp1, 0) == RON_FAULT_NULL_POINTER,
                     "lp1 init rejects null config");
    __CPROVER_assert(ron_lp1_init(&lp1, &cfg) == RON_FAULT_NONE,
                     "lp1 initialises after null checks");
    __CPROVER_assert(ron_lp1_step(&lp1, FILTER_PROOF_SIGNAL, 0) == RON_FAULT_NULL_POINTER,
                     "lp1 step rejects null output");
    __CPROVER_assert(ron_lp1_step(&lp1, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "lp1 remains usable after null-output rejection");
}
