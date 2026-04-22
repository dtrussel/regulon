/*
 * @file     pid_saturation_proof.c
 * @brief    CBMC harness for PID saturation safety.
 * @module   pid_saturation_proof
 * @doc      RON-TP-001
 * @req      RON-FR-020, RON-SR-010
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_pid.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

/* Satisfies: RON-FR-020, RON-SR-010 | Test: RON-TC-PID-015-FV */
void pid_saturation_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = {
        RON_FLOAT_C(10.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0),
        RON_FLOAT_C(1.0), RON_FLOAT_C(1.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(10.0), RON_AW_NONE,
        RON_FLOAT_C(0.1), RON_INTEG_EULER, RON_DERIV_ON_MEASUREMENT,
        RON_FLOAT_C(0.0), false, RON_FLOAT_C(0.0), RON_FLOAT_C(1.0),
        RON_FLOAT_C(0.0), RON_FLOAT_C(1.0), RON_SAFE_HOLD_LAST,
        RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), 0
    };
    ron_float_t        r = nondet_ron_float_t();
    ron_float_t        y = nondet_ron_float_t();
    ron_float_t        dt = nondet_ron_float_t();
    ron_float_t        u = RON_FLOAT_C(0.0);
    ron_status_t       status = RON_STATUS_OK;

    __CPROVER_assume(RON_ISFINITE(r));
    __CPROVER_assume(RON_ISFINITE(y));
    __CPROVER_assume(RON_ISFINITE(dt));
    __CPROVER_assume(dt > RON_FLOAT_C(0.0));

    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE, "valid config initialises");
    __CPROVER_assert(ron_pid_step(&inst, r, y, dt, &u, &status) == RON_FAULT_NONE,
                     "finite single-step execution does not fault");
    __CPROVER_assert((u >= cfg.u_min) && (u <= cfg.u_max),
                     "output remains within configured hard limits");
}
