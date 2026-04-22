/*
 * @file     pid_no_recursion_proof.c
 * @brief    CBMC harness for bounded non-recursive PID API calls.
 * @module   pid_no_recursion_proof
 * @doc      RON-TP-001
 * @req      RON-SR-004
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-004 | Test: RON-TC-SAFE-004-FV */
void pid_no_recursion_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t integral = RON_FLOAT_C(0.0);
    ron_float_t last_u = RON_FLOAT_C(0.0);
    ron_float_t last_d = RON_FLOAT_C(0.0);
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t fault = RON_FAULT_NONE;

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "init completes without recursion");
    __CPROVER_assert(ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &u, &status) == RON_FAULT_NONE,
                     "step completes without recursion");
    __CPROVER_assert(ron_pid_get_state(&inst, &integral, &last_u, &last_d, &status,
                                       &fault) == RON_FAULT_NONE,
                     "get_state completes without recursion");
    __CPROVER_assert(ron_pid_reset(&inst) == RON_FAULT_NONE,
                     "reset completes without recursion");
}
