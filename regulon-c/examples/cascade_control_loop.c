/*
 * @file     cascade_control_loop.c
 * @brief    Aggregate example composing trajectory, cascade, health and metrics.
 * @module   example_cascade_control_loop
 * @doc      RON-IS-001
 * @req      RON-FR-401, RON-FR-500, RON-FR-900, RON-FR-950
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Demonstrates several Regulon modules working together in one control loop,
 * mirroring the RON-TC-INT-002 integration test but written for readability:
 *
 *   trapezoidal trajectory  ->  position setpoint
 *   cascade PID (pos/vel)   ->  actuator command driving a two-state plant
 *   health monitor          ->  passive supervision of the loop
 *   metrics accumulator     ->  IAE / ISE / settling-time quality figures
 *
 * Host-only documentation example (uses printf); excluded from all production
 * gates and from the cross-compile builds.  Uses only <ron/ron.h>.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/cascade_control_loop
 */

#include <stdio.h>

#include "ron/ron.h"

/* Build a single PID loop configuration with the given gains and output span. */
static ron_pid_config_t make_pid(ron_float_t kp, ron_float_t ki, ron_float_t umin, ron_float_t umax)
{
    ron_pid_config_t cfg = {0};

    cfg.Kp          = kp;
    cfg.Ki          = ki;
    cfg.b           = RON_FLOAT_C(1.0);
    cfg.c           = RON_FLOAT_C(1.0);
    cfg.u_min       = umin;
    cfg.u_max       = umax;
    cfg.I_min       = RON_FLOAT_C(-1000.0);
    cfg.I_max       = RON_FLOAT_C(1000.0);
    cfg.aw_mode     = RON_AW_BACK_CALC;
    cfg.T_aw        = RON_FLOAT_C(0.05);
    cfg.safe_policy = RON_SAFE_HOLD_LAST;
    return cfg;
}

int main(void)
{
    ron_cascade_instance_t casc;
    ron_trap_t traj;
    ron_health_t mon;
    ron_metrics_t met;

    ron_pid_config_t outer =
        make_pid(RON_FLOAT_C(4.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0));
    ron_pid_config_t inner =
        make_pid(RON_FLOAT_C(8.0), RON_FLOAT_C(40.0), RON_FLOAT_C(-50.0), RON_FLOAT_C(50.0));
    ron_trap_config_t tcfg = {RON_FLOAT_C(2.0), RON_FLOAT_C(4.0)};

    ron_health_config_t hcfg = {
        RON_FLOAT_C(2.0), RON_FLOAT_C(50.0), 12U, RON_FLOAT_C(0.0), RON_FLOAT_C(5.0),
        RON_FLOAT_C(0.1), RON_FLOAT_C(10.0), NULL};
    ron_metrics_config_t mcfg = {RON_METRICS_CUMULATIVE, 0U, RON_FLOAT_C(0.05), RON_FLOAT_C(0.1),
                                 RON_FLOAT_C(0.2)};

    const ron_float_t dt = RON_FLOAT_C(0.005);
    ron_float_t pos      = RON_FLOAT_C(0.0);
    ron_float_t vel      = RON_FLOAT_C(0.0);
    unsigned k;

    if ((ron_cascade_init(&casc, &outer, &inner) != RON_FAULT_NONE) ||
        (ron_trap_init(&traj, &tcfg, RON_FLOAT_C(0.0)) != RON_FAULT_NONE) ||
        (ron_health_init(&mon, &hcfg) != RON_FAULT_NONE) ||
        (ron_metrics_init(&met, &mcfg) != RON_FAULT_NONE)) {
        (void) fprintf(stderr, "initialisation failed\n");
        return 1;
    }
    (void) ron_trap_set_target(&traj, RON_FLOAT_C(1.0));
    (void) ron_metrics_enable(&met, true);

    (void) printf(" time   setpoint   position   command\n");
    for (k = 0U; k < 1200U; ++k) {
        ron_float_t r_pos = RON_FLOAT_C(0.0);
        ron_float_t tvel  = RON_FLOAT_C(0.0);
        ron_float_t tacc  = RON_FLOAT_C(0.0);
        bool finished     = false;
        ron_cascade_status_t status;
        ron_float_t u = RON_FLOAT_C(0.0);

        (void) ron_trap_step(&traj, dt, &r_pos, &tvel, &tacc, &finished);
        (void) ron_cascade_step(&casc, r_pos, pos, vel, dt, &u, &status);

        vel = vel + (dt / RON_FLOAT_C(0.05)) * (u - vel);
        pos = pos + dt * vel;

        (void) ron_health_step(&mon, r_pos, pos, u, dt);
        (void) ron_metrics_step(&met, r_pos, pos, dt);

        if ((k % 150U) == 0U) {
            (void) printf("%5.2f   %8.4f   %8.4f   %8.4f\n", (double) ((ron_float_t) k * dt),
                          (double) r_pos, (double) pos, (double) u);
        }
    }

    {
        ron_health_status_t health = RON_HEALTH_OK;
        ron_metrics_result_t result;

        (void) ron_health_get(&mon, &health);
        (void) ron_metrics_get(&met, &result);
        (void) printf("\nfinal position : %.4f (target 1.0)\n", (double) pos);
        (void) printf("health bitmask : 0x%02X\n", (unsigned) health);
        (void) printf("IAE=%.4f  ISE=%.4f  ITAE=%.4f  settling=%.3f s\n", (double) result.IAE,
                      (double) result.ISE, (double) result.ITAE, (double) result.settling_time);
    }
    return 0;
}
