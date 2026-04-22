/*
 * @file     pid_nan_inf_proof.c
 * @brief    CBMC harness for PID computed NaN/Inf handling.
 * @module   pid_nan_inf_proof
 * @doc      RON-TP-001
 * @req      RON-SR-020
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011-FV */
void pid_nan_inf_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t fault;

    cfg = pid_proof_make_config();
    cfg.Kp = RON_FLOAT_MAX;
    cfg.Ki = RON_FLOAT_C(0.0);
    cfg.Kd = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "large finite gain config initialises");
    fault = ron_pid_step(&inst, RON_FLOAT_MAX, RON_FLOAT_C(0.0), PID_PROOF_DT,
                         &u, &status);

    __CPROVER_assert((fault & RON_FAULT_OUTPUT_NAN) != RON_FAULT_NONE,
                     "non-finite computed output sets output fault");
    __CPROVER_assert(!RON_ISNAN(u) && !RON_ISINF(u),
                     "safe output remains finite after computed non-finite fault");
}
