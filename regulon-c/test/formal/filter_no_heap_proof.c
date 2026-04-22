/*
 * @file     filter_no_heap_proof.c
 * @brief    CBMC harness for filter no-heap behaviour.
 * @module   filter_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-FR-101, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "filter_proof_common.h"

/* Satisfies: RON-FR-101, RON-SR-003, RON-PR-022 | Test: RON-TC-FILT-002-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "filter library must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-101, RON-SR-003, RON-PR-022 | Test: RON-TC-FILT-002-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "filter library must not call free");
}

/* Satisfies: RON-FR-101, RON-SR-003, RON-PR-022 | Test: RON-TC-FILT-002-FV */
void filter_no_heap_proof(void)
{
    ron_lp1_t lp1;
    ron_lp1_config_t cfg = filter_proof_lp1_config();
    ron_float_t y        = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_lp1_init(&lp1, &cfg) == RON_FAULT_NONE,
                     "valid low-pass config initialises without heap");
    __CPROVER_assert(ron_lp1_step(&lp1, FILTER_PROOF_SIGNAL, &y) == RON_FAULT_NONE,
                     "low-pass step completes without heap");
}
