/*
 * @file     pid_integral_clamp_proof.c
 * @brief    CBMC harness for PID integral clamp safety.
 * @module   pid_integral_clamp_proof
 * @doc      RON-TP-001
 * @req      RON-FR-035
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

extern void __CPROVER_assert(int condition, const char *description);

/* Satisfies: RON-FR-035 | Test: RON-TC-PID-026-FV */
void pid_integral_clamp_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = {
        RON_FLOAT_C(0.0), RON_FLOAT_C(100.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-100.0), RON_FLOAT_C(100.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-0.5), RON_FLOAT_C(0.5), RON_AW_NONE,
        RON_FLOAT_C(0.1), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), 0
    };
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE, "valid config initialises");
    for (index = 0U; index < 4U; ++index)
    {
        __CPROVER_assert(ron_pid_step(&inst, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.1), &u, &status) == RON_FAULT_NONE,
                         "bounded integral-clamp step completes");
        __CPROVER_assert((inst.state.integral >= cfg.I_min) && (inst.state.integral <= cfg.I_max),
                         "integral remains within configured clamp range");
    }
}
