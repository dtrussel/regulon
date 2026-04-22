/*
 * @file     pid_array_bounds_proof.c
 * @brief    CBMC harness for PID array bounds safety.
 * @module   pid_array_bounds_proof
 * @doc      RON-TP-001
 * @req      RON-SR-005
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-005 | Test: RON-TC-SAFE-005-FV */
void pid_array_bounds_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "valid config initialises");
    __CPROVER_assert(ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &u, &status) == RON_FAULT_NONE,
                     "step has no out-of-bounds array access");
}
