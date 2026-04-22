/*
 * @file     pid_multi_instance_proof.c
 * @brief    CBMC harness for PID multi-instance independence.
 * @module   pid_multi_instance_proof
 * @doc      RON-TP-001
 * @req      RON-FR-060, RON-FR-061, RON-FR-062
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

extern void __CPROVER_assert(int condition, const char *description);

/* Satisfies: RON-FR-060, RON-FR-061, RON-FR-062 | Test: RON-TC-PID-036-FV */
void pid_multi_instance_proof(void)
{
    ron_pid_instance_t a_interleaved;
    ron_pid_instance_t b_interleaved;
    ron_pid_instance_t a_alone;
    ron_pid_config_t   cfg = {
        RON_FLOAT_C(2.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-100.0), RON_FLOAT_C(100.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-100.0), RON_FLOAT_C(100.0), RON_AW_NONE,
        RON_FLOAT_C(0.1), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), 0
    };
    ron_float_t        u_a_i = RON_FLOAT_C(0.0);
    ron_float_t        u_b_i = RON_FLOAT_C(0.0);
    ron_float_t        u_a_a = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    __CPROVER_assert(ron_pid_init(&a_interleaved, &cfg) == RON_FAULT_NONE, "instance A initialises");
    __CPROVER_assert(ron_pid_init(&b_interleaved, &cfg) == RON_FAULT_NONE, "instance B initialises");
    __CPROVER_assert(ron_pid_init(&a_alone, &cfg) == RON_FAULT_NONE, "standalone instance A initialises");

    __CPROVER_assert(ron_pid_step(&a_interleaved, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.1), &u_a_i, &status) == RON_FAULT_NONE,
                     "interleaved instance A step completes");
    __CPROVER_assert(ron_pid_step(&b_interleaved, RON_FLOAT_C(-0.5), RON_FLOAT_C(0.25),
                                  RON_FLOAT_C(0.1), &u_b_i, &status) == RON_FAULT_NONE,
                     "interleaved instance B step completes");
    __CPROVER_assert(ron_pid_step(&a_alone, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                  RON_FLOAT_C(0.1), &u_a_a, &status) == RON_FAULT_NONE,
                     "standalone instance A step completes");
    __CPROVER_assert(u_a_i == u_a_a,
                     "interleaving a second instance does not affect instance A output");
}
