/*
 * @file     filter_proof_common.h
 * @brief    Shared CBMC harness helpers for Regulon filters.
 * @module   filter_proof_common
 * @doc      RON-TP-001
 * @req      RON-FR-100, RON-FR-101, RON-FR-102
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef FILTER_PROOF_COMMON_H
#define FILTER_PROOF_COMMON_H

#include "ron/ron_filter.h"

#define FILTER_PROOF_DT RON_FLOAT_C(0.1)
#define FILTER_PROOF_SIGNAL RON_FLOAT_C(1.0)

extern void __CPROVER_assert(int condition, const char *description);

#if defined(__GNUC__) || defined(__clang__)
#define FILTER_PROOF_UNUSED __attribute__((unused))
#else
#define FILTER_PROOF_UNUSED
#endif

/* Satisfies: RON-FR-110 | Test: RON-TC-FILT-005-FV */
static FILTER_PROOF_UNUSED ron_lp1_config_t filter_proof_lp1_config(void)
{
    ron_lp1_config_t cfg = {RON_FLOAT_C(0.5)};

    return cfg;
}

/* Satisfies: RON-FR-115 | Test: RON-TC-FILT-008-FV */
static FILTER_PROOF_UNUSED ron_ma_config_t filter_proof_ma_config(void)
{
    ron_ma_config_t cfg = {4U};

    return cfg;
}

/* Satisfies: RON-FR-120 | Test: RON-TC-FILT-011-FV */
static FILTER_PROOF_UNUSED ron_biquad_config_t filter_proof_biquad_config(void)
{
    ron_biquad_config_t cfg = {{{RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                 RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)}},
                               1U};

    return cfg;
}

/* Satisfies: RON-FR-130 | Test: RON-TC-FILT-016-FV */
static FILTER_PROOF_UNUSED ron_ratelim_config_t filter_proof_ratelim_config(void)
{
    ron_ratelim_config_t cfg = {RON_FLOAT_C(1.0), RON_FLOAT_C(1.0)};

    return cfg;
}

#endif /* FILTER_PROOF_COMMON_H */
