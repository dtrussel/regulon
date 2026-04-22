/*
 * @file     pid_integral_overflow_proof.c
 * @brief    CBMC harness for PID integral overflow fault handling.
 * @module   pid_integral_overflow_proof
 * @doc      RON-TP-001
 * @req      RON-SR-013
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-013 | Test: RON-TC-SAFE-013-FV */
void pid_integral_overflow_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t fault;

    cfg = pid_proof_make_config();
    cfg.Kp = RON_FLOAT_C(0.0);
    cfg.Ki = RON_FLOAT_C(10.0);
    cfg.Kd = RON_FLOAT_C(0.0);
    cfg.I_overflow_thresh = RON_FLOAT_C(0.1);

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "overflow-threshold config initialises");
    fault = ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0), PID_PROOF_DT,
                         &u, &status);

    __CPROVER_assert((fault & RON_FAULT_INTEGRAL_OVERFLOW) != RON_FAULT_NONE,
                     "integral overflow threshold sets fault code");
    __CPROVER_assert((status & RON_STATUS_FAULT) != 0U,
                     "integral overflow sets status fault flag");
}
