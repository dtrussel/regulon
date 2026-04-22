/*
 * @file     pid_division_guard_proof.c
 * @brief    CBMC harness for PID denominator validation.
 * @module   pid_division_guard_proof
 * @doc      RON-TP-001
 * @req      RON-SR-002
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-002 | Test: RON-TC-SAFE-002-FV */
void pid_division_guard_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg = pid_proof_make_config();
    cfg.T_aw = RON_FLOAT_C(0.0);
    __CPROVER_assert(ron_pid_config_validate(&cfg) == RON_FAULT_CONFIG_INVALID,
                     "back-calculation rejects zero T_aw denominator");

    cfg = pid_proof_make_config();
    cfg.normalise = true;
    cfg.in_max = cfg.in_min;
    __CPROVER_assert(ron_pid_config_validate(&cfg) == RON_FAULT_CONFIG_INVALID,
                     "normalisation rejects zero input denominator");

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "valid config initialises");
    __CPROVER_assert(ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.0), &u, &status) ==
                         RON_FAULT_CONFIG_INVALID,
                     "step rejects zero dt before division");
}
