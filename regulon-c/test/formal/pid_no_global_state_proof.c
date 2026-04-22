/*
 * @file     pid_no_global_state_proof.c
 * @brief    CBMC harness for PID instance-local state behaviour.
 * @module   pid_no_global_state_proof
 * @doc      RON-TP-001
 * @req      RON-QR-030, RON-FR-061
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-QR-030, RON-FR-061 | Test: RON-TC-QUAL-016-FV */
void pid_no_global_state_proof(void)
{
    ron_pid_instance_t first;
    ron_pid_instance_t second;
    ron_pid_config_t cfg;
    ron_float_t first_u = RON_FLOAT_C(0.0);
    ron_float_t second_u = RON_FLOAT_C(0.0);
    ron_status_t first_status = RON_STATUS_OK;
    ron_status_t second_status = RON_STATUS_OK;

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&first, &cfg) == RON_FAULT_NONE,
                     "first instance initialises");
    __CPROVER_assert(ron_pid_init(&second, &cfg) == RON_FAULT_NONE,
                     "second instance initialises");
    __CPROVER_assert(ron_pid_step(&first, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &first_u, &first_status) == RON_FAULT_NONE,
                     "first instance step completes");
    __CPROVER_assert(ron_pid_step(&second, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &second_u, &second_status) == RON_FAULT_NONE,
                     "second instance step completes");
    __CPROVER_assert(first_u == second_u,
                     "matching instances produce matching outputs without global state");
    __CPROVER_assert(first_status == second_status,
                     "matching instances produce matching status without global state");
}
