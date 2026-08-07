/*
 * @file     statespace_observer.c
 * @brief    Example: state-feedback control with a Luenberger observer.
 * @module   example_statespace_observer
 * @doc      RON-IS-001
 * @req      RON-FR-700, RON-FR-701, RON-FR-720
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Drives a double-integrator plant (position/velocity) to a setpoint using
 * ron_ss with an embedded Luenberger observer (ron_obs) that reconstructs
 * the full state from a position-only measurement, using only the aggregate
 * header <ron/ron.h>.  Host-only documentation example (uses printf);
 * excluded from all production gates and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/statespace_observer
 */

#include <stdio.h>

#include "ron/ron.h"

int main(void)
{
    ron_ss_t ss;
    ron_ss_config_t cfg  = {0};
    const ron_float_t dt = RON_FLOAT_C(0.05);
    ron_float_t pos      = RON_FLOAT_C(0.0); /* true plant state */
    ron_float_t vel      = RON_FLOAT_C(0.0);
    ron_float_t u_prev   = RON_FLOAT_C(0.0); /* command applied over the last interval */
    unsigned k;

    /* Double-integrator plant: x' = [pos, vel], x(k+1) = A x(k) + B u(k). */
    cfg.n               = 2U;
    cfg.source          = RON_SS_SOURCE_LUENBERGER;
    cfg.K[0]            = RON_FLOAT_C(2.0); /* position feedback */
    cfg.K[1]            = RON_FLOAT_C(3.0); /* velocity feedback (damping) */
    cfg.Kr              = RON_FLOAT_C(2.0); /* == K[0], so a step r settles at pos == r */
    cfg.u_min           = RON_FLOAT_C(-5.0);
    cfg.u_max           = RON_FLOAT_C(5.0);
    cfg.obs_cfg.n       = 2U;
    cfg.obs_cfg.m       = 1U;
    cfg.obs_cfg.p       = 1U;
    cfg.obs_cfg.A[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.A[0][1] = dt;
    cfg.obs_cfg.A[1][1] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.B[1][0] = dt;
    cfg.obs_cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.L[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.L[1][0] = RON_FLOAT_C(0.5);

    if (ron_ss_init(&ss, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "state-space init failed\n");
        return 1;
    }

    (void) printf(" time   setpoint   position   est_pos   command\n");
    for (k = 0U; k < 200U; ++k) {
        const ron_float_t r = RON_FLOAT_C(1.0);
        ron_float_t y[RON_SS_MAX_OUTPUTS];
        ron_float_t u_in[RON_SS_MAX_INPUTS];
        ron_float_t u;
        ron_status_t status;
        ron_float_t x_hat[RON_SS_MAX_STATES];

        y[0]    = pos; /* position-only measurement fed to the observer */
        u_in[0] = u_prev;

        (void) ron_ss_observer_step(&ss, y, u_in);
        (void) ron_ss_step(&ss, r, dt, &u, &status);

        vel    = vel + dt * u;
        pos    = pos + dt * vel;
        u_prev = u;

        (void) ron_obs_get_state(&ss.observer, x_hat);

        if ((k % 20U) == 0U) {
            (void) printf("%5.2f   %8.3f   %8.3f   %8.3f   %8.3f\n",
                          (double) ((ron_float_t) k * dt), (double) r, (double) pos,
                          (double) x_hat[0], (double) u);
        }
    }
    (void) printf("final position : %.4f (target 1.0)\n", (double) pos);
    return 0;
}
