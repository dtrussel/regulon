/*
 * @file     trajectory_motion.c
 * @brief    Example: jerk-limited S-curve motion profile generation.
 * @module   example_trajectory_motion
 * @doc      RON-IS-001
 * @req      RON-FR-510, RON-FR-511
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Generates a jerk-limited S-curve position/velocity/acceleration profile
 * from 0 to a target position using ron_scurve, using only the aggregate
 * header <ron/ron.h>.  Host-only documentation example (uses printf);
 * excluded from all production gates and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/trajectory_motion
 */

#include <stdio.h>

#include "ron/ron.h"

int main(void)
{
    ron_scurve_t traj;
    ron_scurve_config_t cfg = {0};
    const ron_float_t dt    = RON_FLOAT_C(0.01);
    unsigned k;

    cfg.v_max = RON_FLOAT_C(2.0);
    cfg.a_max = RON_FLOAT_C(4.0);
    cfg.j_max = RON_FLOAT_C(20.0);

    if (ron_scurve_init(&traj, &cfg, RON_FLOAT_C(0.0)) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "trajectory init failed\n");
        return 1;
    }
    if (ron_scurve_set_target(&traj, RON_FLOAT_C(1.0)) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "set_target failed\n");
        return 1;
    }

    (void) printf(" time     pos      vel      acc      jrk   phase\n");
    for (k = 0U; k < 300U; ++k) {
        ron_float_t pos;
        ron_float_t vel;
        ron_float_t acc;
        ron_float_t jrk;
        bool finished = false;

        (void) ron_scurve_step(&traj, dt, &pos, &vel, &acc, &jrk, &finished);

        if (((k % 20U) == 0U) || finished) {
            (void) printf("%5.2f   %6.3f   %6.3f   %6.3f   %6.2f   %d\n",
                          (double) ((ron_float_t) k * dt), (double) pos, (double) vel, (double) acc,
                          (double) jrk, (int) traj.state.phase);
        }
        if (finished) {
            break;
        }
    }
    (void) printf("final position: %.4f (target 1.0)\n", (double) traj.state.pos);
    return 0;
}
