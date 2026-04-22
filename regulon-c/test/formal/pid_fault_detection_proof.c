/*
 * @file     pid_fault_detection_proof.c
 * @brief    CBMC harness for PID fault flag detection.
 * @module   pid_fault_detection_proof
 * @doc      RON-TP-001
 * @req      RON-SR-010
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-010 | Test: RON-TC-SAFE-007-FV */
void pid_fault_detection_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t bad_input;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t fault;

    cfg = pid_proof_make_config();
    bad_input = nondet_ron_float_t();
    __CPROVER_assume(RON_ISNAN(bad_input) || RON_ISINF(bad_input));

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "valid config initialises");
    fault = ron_pid_step(&inst, bad_input, RON_FLOAT_C(0.0), PID_PROOF_DT, &u, &status);

    __CPROVER_assert((fault & RON_FAULT_INPUT_NAN) != RON_FAULT_NONE,
                     "NaN/Inf input sets input fault");
    __CPROVER_assert((status & RON_STATUS_FAULT) != 0U,
                     "NaN/Inf input sets status fault flag");
    __CPROVER_assert((u >= cfg.u_min) && (u <= cfg.u_max),
                     "fault path applies bounded safe output");
}
