/*
 * @file     pid_null_pointer_proof.c
 * @brief    CBMC harness for PID null-pointer validation.
 * @module   pid_null_pointer_proof
 * @doc      RON-TP-001
 * @req      RON-SR-001, RON-SR-006
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

extern void __CPROVER_assert(int condition, const char *description);

/* Satisfies: RON-SR-001, RON-SR-006 | Test: RON-TC-SAFE-006-FV */
void pid_null_pointer_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = {
        RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-1.0), RON_FLOAT_C(1.0), RON_AW_NONE,
        RON_FLOAT_C(0.1), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), 0
    };
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    __CPROVER_assert(ron_pid_init((ron_pid_instance_t *)0, &cfg) == RON_FAULT_NULL_POINTER,
                     "init rejects NULL instance");
    __CPROVER_assert(ron_pid_init(&inst, (const ron_pid_config_t *)0) == RON_FAULT_NULL_POINTER,
                     "init rejects NULL config");
    __CPROVER_assert(ron_pid_step((ron_pid_instance_t *)0, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.1), &u, &status) == RON_FAULT_NULL_POINTER,
                     "step rejects NULL instance");
    __CPROVER_assert(ron_pid_step(&inst, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.1), (ron_float_t *)0, &status) == RON_FAULT_NULL_POINTER,
                     "step rejects NULL output pointer");
    __CPROVER_assert(ron_pid_step(&inst, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.1), &u, (ron_status_t *)0) == RON_FAULT_NULL_POINTER,
                     "step rejects NULL status pointer");
}
