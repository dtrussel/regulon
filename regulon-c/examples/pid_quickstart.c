/*
 * @file     pid_quickstart.c
 * @brief    Minimal getting-started example for the Regulon PID controller.
 * @module   example_pid_quickstart
 * @doc      RON-IS-001
 * @req      RON-FR-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Drives a first-order plant to a unit setpoint with a single PID loop, using
 * only the aggregate header <ron/ron.h>.  This file is a host-only documentation
 * example: it uses printf and is therefore excluded from every production gate
 * (MISRA / coverage / complexity / formal) and from the cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/pid_quickstart
 */

#include <stdio.h>

#include "ron/ron.h"

int main(void)
{
    ron_pid_instance_t pid;
    ron_pid_config_t cfg = {0};
    const ron_float_t dt = RON_FLOAT_C(0.01);
    ron_float_t y        = RON_FLOAT_C(0.0); /* plant output */
    unsigned k;

    cfg.Kp          = RON_FLOAT_C(2.0);
    cfg.Ki          = RON_FLOAT_C(5.0);
    cfg.Kd          = RON_FLOAT_C(0.0);
    cfg.b           = RON_FLOAT_C(1.0);
    cfg.c           = RON_FLOAT_C(1.0);
    cfg.u_min       = RON_FLOAT_C(-10.0);
    cfg.u_max       = RON_FLOAT_C(10.0);
    cfg.I_min       = RON_FLOAT_C(-100.0);
    cfg.I_max       = RON_FLOAT_C(100.0);
    cfg.aw_mode     = RON_AW_BACK_CALC;
    cfg.T_aw        = RON_FLOAT_C(0.05);
    cfg.safe_policy = RON_SAFE_HOLD_LAST;

    if (ron_pid_init(&pid, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "PID init failed\n");
        return 1;
    }

    (void) printf("step      r        y        u\n");
    for (k = 0U; k < 300U; ++k) {
        const ron_float_t r   = RON_FLOAT_C(1.0);
        const ron_float_t tau = RON_FLOAT_C(0.2);
        ron_float_t u         = RON_FLOAT_C(0.0);
        ron_status_t status;

        (void) ron_pid_step(&pid, r, y, dt, &u, &status);
        y = y + (dt / tau) * (u - y); /* first-order plant */

        if ((k % 30U) == 0U) {
            (void) printf("%4u  %7.3f  %7.3f  %7.3f\n", k, (double) r, (double) y, (double) u);
        }
    }
    (void) printf("final output y = %.4f (target 1.0)\n", (double) y);
    return 0;
}
