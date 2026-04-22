/*
 * @file     pid_proof_common.h
 * @brief    Shared CBMC harness helpers for the Regulon PID controller module.
 * @module   pid_proof_common
 * @doc      RON-TP-001
 * @req      RON-FR-001, RON-SR-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef PID_PROOF_COMMON_H
#define PID_PROOF_COMMON_H

#include "ron/ron_pid.h"

#define PID_PROOF_DT RON_FLOAT_C(0.1)
#define PID_PROOF_SIGNAL RON_FLOAT_C(1.0)

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

/* Satisfies: RON-FR-001, RON-SR-001 | Test: RON-TC-SAFE-001-FV */
static ron_pid_config_t pid_proof_make_config(void)
{
    ron_pid_config_t cfg = {
        RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.1), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(10.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0), RON_AW_BACK_CALC,
        RON_FLOAT_C(0.2), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), 0
    };

    return cfg;
}

#endif /* PID_PROOF_COMMON_H */
