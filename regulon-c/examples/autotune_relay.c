/*
 * @file     autotune_relay.c
 * @brief    Example: relay-feedback auto-tuning a PID controller.
 * @module   example_autotune_relay
 * @doc      RON-IS-001
 * @req      RON-FR-800, RON-FR-802, RON-FR-803, RON-FR-804
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Runs ron_autotune's relay-feedback excitation against a first-order plant
 * to estimate the ultimate gain/period, applies the Ziegler-Nichols gains to
 * the target PID, then runs a short closed-loop step response with the
 * tuned gains — using only the aggregate header <ron/ron.h>.  Host-only
 * documentation example (uses printf); excluded from all production gates
 * and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/autotune_relay
 */

#include <stdio.h>

#include "ron/ron.h"

int main(void)
{
    ron_pid_instance_t pid;
    ron_pid_config_t pid_cfg = {0};
    ron_at_t at;
    ron_at_config_t at_cfg = {0};
    const ron_float_t dt   = RON_FLOAT_C(0.02);
    const ron_float_t tau  = RON_FLOAT_C(0.3); /* plant time constant */
    ron_float_t y          = RON_FLOAT_C(0.0);
    unsigned k;

    pid_cfg.Kp          = RON_FLOAT_C(0.1); /* arbitrary starting gains; overwritten by apply */
    pid_cfg.u_min       = RON_FLOAT_C(-10.0);
    pid_cfg.u_max       = RON_FLOAT_C(10.0);
    pid_cfg.I_min       = RON_FLOAT_C(-100.0);
    pid_cfg.I_max       = RON_FLOAT_C(100.0);
    pid_cfg.safe_policy = RON_SAFE_HOLD_LAST;

    at_cfg.relay_amplitude = RON_FLOAT_C(2.0);
    at_cfg.hysteresis      = RON_FLOAT_C(0.05);
    at_cfg.u_bias          = RON_FLOAT_C(0.0);
    at_cfg.min_cycles      = 5U;
    at_cfg.timeout_s       = RON_FLOAT_C(30.0);
    at_cfg.tuning_rule     = RON_AT_RULE_ZN;

    if ((ron_pid_init(&pid, &pid_cfg) != RON_FAULT_NONE) ||
        (ron_autotune_init(&at, &at_cfg) != RON_FAULT_NONE) ||
        (ron_autotune_start(&at, &pid) != RON_FAULT_NONE)) {
        (void) fprintf(stderr, "auto-tune initialisation failed\n");
        return 1;
    }

    /* Relay-excitation phase: drive the plant with the relay output until
     * ron_autotune_step() reports the run is done. */
    for (k = 0U; (k < 20000U) && !at.state.done && !at.state.aborted; ++k) {
        ron_float_t u = RON_FLOAT_C(0.0);

        (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), y, dt, &u);
        y = y + (dt / tau) * (u - y); /* first-order plant */
    }

    if (!at.state.done) {
        (void) fprintf(stderr, "auto-tune did not complete (aborted=%d)\n", (int) at.state.aborted);
        return 1;
    }

    (void) printf("Ku=%.3f  Tu=%.3f s\n", (double) at.state.Ku, (double) at.state.Tu);
    (void) printf("tuned gains: Kp=%.3f  Ki=%.3f  Kd=%.3f\n", (double) at.state.Kp_result,
                  (double) at.state.Ki_result, (double) at.state.Kd_result);

    if (ron_autotune_apply(&at, &pid) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "applying tuned gains failed\n");
        return 1;
    }

    /* Closed-loop verification with the tuned gains. */
    y = RON_FLOAT_C(0.0);
    (void) printf("\nstep      r        y        u\n");
    for (k = 0U; k < 300U; ++k) {
        const ron_float_t r = RON_FLOAT_C(1.0);
        ron_float_t u       = RON_FLOAT_C(0.0);
        ron_status_t status;

        (void) ron_pid_step(&pid, r, y, dt, &u, &status);
        y = y + (dt / tau) * (u - y);

        if ((k % 30U) == 0U) {
            (void) printf("%4u  %7.3f  %7.3f  %7.3f\n", k, (double) r, (double) y, (double) u);
        }
    }
    (void) printf("final output y = %.4f (target 1.0)\n", (double) y);
    return 0;
}
