/*
 * @file     pid_bounded_step_proof.c
 * @brief    CBMC harness for bounded PID step execution.
 * @module   pid_bounded_step_proof
 * @doc      RON-TP-001
 * @req      RON-PR-001, RON-PR-002
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-PR-001, RON-PR-002 | Test: RON-TC-PERF-002-FV */
void pid_bounded_step_proof(void)
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
                     "step terminates within bounded CBMC unwind");
}
