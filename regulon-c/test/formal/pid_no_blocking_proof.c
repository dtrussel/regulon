/*
 * @file     pid_no_blocking_proof.c
 * @brief    CBMC harness for PID no-blocking-call behaviour.
 * @module   pid_no_blocking_proof
 * @doc      RON-TP-001
 * @req      RON-PR-004
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-PR-004 | Test: RON-TC-PERF-004-FV */
void sleep(unsigned long seconds)
{
    (void)seconds;
    __CPROVER_assert(0, "PID library must not call sleep");
}

/* Satisfies: RON-PR-004 | Test: RON-TC-PERF-004-FV */
void pid_no_blocking_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "valid config initialises without blocking");
    __CPROVER_assert(ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &u, &status) == RON_FAULT_NONE,
                     "step completes without blocking");
}
