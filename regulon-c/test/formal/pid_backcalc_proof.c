/*
 * @file     pid_backcalc_proof.c
 * @brief    CBMC harness for PID back-calculation bounded safety.
 * @module   pid_backcalc_proof
 * @doc      RON-TP-001
 * @req      RON-FR-031
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

extern void __CPROVER_assert(int condition, const char *description);

/* Satisfies: RON-FR-031 | Test: RON-TC-PID-022-FV */
void pid_backcalc_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = {
        RON_FLOAT_C(1.0), RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-1000.0), RON_FLOAT_C(1000.0), RON_AW_BACK_CALC,
        RON_FLOAT_C(0.1), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), 0
    };
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;
    unsigned           index;

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE, "back-calculation config initialises");
    for (index = 0U; index < 4U; ++index)
    {
        __CPROVER_assert(ron_pid_step(&inst, RON_FLOAT_C(100.0), RON_FLOAT_C(0.0),
                                      RON_FLOAT_C(0.1), &u, &status) == RON_FAULT_NONE,
                         "bounded saturated step completes");
        __CPROVER_assert((u >= cfg.u_min) && (u <= cfg.u_max),
                         "bounded saturated step respects output limits");
        __CPROVER_assert(RON_ISFINITE(inst.state.integral),
                         "back-calculation keeps the integral state finite");
    }
}
