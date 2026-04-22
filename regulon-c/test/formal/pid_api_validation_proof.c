/*
 * @file     pid_api_validation_proof.c
 * @brief    CBMC harness for PID public API parameter validation.
 * @module   pid_api_validation_proof
 * @doc      RON-TP-001
 * @req      RON-SR-001, RON-SR-002
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-001, RON-SR-002 | Test: RON-TC-SAFE-001-FV */
void pid_api_validation_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_pid_config_t invalid_cfg;

    cfg = pid_proof_make_config();
    invalid_cfg = cfg;
    invalid_cfg.Kp = RON_FLOAT_C(-1.0);

    __CPROVER_assert(ron_pid_config_validate((const ron_pid_config_t *)0) ==
                         RON_FAULT_NULL_POINTER,
                     "configuration validation rejects NULL");
    __CPROVER_assert(ron_pid_config_validate(&invalid_cfg) == RON_FAULT_CONFIG_INVALID,
                     "configuration validation rejects invalid gains");
    __CPROVER_assert(ron_pid_init(&inst, &invalid_cfg) == RON_FAULT_CONFIG_INVALID,
                     "initialisation rejects invalid configuration");
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "initialisation accepts valid configuration");
    __CPROVER_assert(ron_pid_set_gains(&inst, RON_FLOAT_C(-1.0), cfg.Ki, cfg.Kd) ==
                         RON_FAULT_CONFIG_INVALID,
                     "runtime gain update rejects invalid candidate");
    __CPROVER_assert(ron_pid_set_limits(&inst, cfg.u_max, cfg.u_min) ==
                         RON_FAULT_CONFIG_INVALID,
                     "runtime limit update rejects inconsistent range");
}
