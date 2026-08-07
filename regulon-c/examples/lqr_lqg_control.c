/*
 * @file     lqr_lqg_control.c
 * @brief    Example: LQR and LQG optimal control of a double-integrator plant.
 * @module   example_lqr_lqg_control
 * @doc      RON-IS-001
 * @req      RON-FR-730, RON-FR-733, RON-FR-750, RON-FR-756
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Part 1 solves an LQR gain via the DARE solver (RON-FR-733) and regulates a
 * double-integrator plant to a setpoint from an exact external state. Part 2
 * solves the same problem with ron_lqg, which additionally estimates the
 * state from noisy position-only measurements via an embedded Kalman filter
 * (separation principle, RON-FR-756) instead of assuming perfect state
 * knowledge.  Uses only the aggregate header <ron/ron.h>.  Host-only
 * documentation example (uses printf/rand); excluded from all production
 * gates and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/lqr_lqg_control
 */

#include <stdio.h>
#include <stdlib.h>

#include "ron/ron.h"

static ron_float_t noise(ron_float_t amplitude)
{
    ron_float_t unit = (ron_float_t) rand() / (ron_float_t) RAND_MAX;

    return (RON_FLOAT_C(2.0) * unit - RON_FLOAT_C(1.0)) * amplitude;
}

/* Part 1: LQR with an exact external state source. */
static int run_lqr(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg              = {0};
    ron_float_t x[2]                  = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    unsigned k;

    cfg.n             = 2U;
    cfg.m             = 1U;
    cfg.source        = RON_LQR_SOURCE_EXTERNAL;
    cfg.x_ext         = x;
    cfg.gain_mode     = RON_LQR_GAIN_DARE;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.A[0][1]       = RON_FLOAT_C(1.0);
    cfg.A[1][1]       = RON_FLOAT_C(1.0);
    cfg.B[1][0]       = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1]  = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.dare_max_iter = 200U;
    cfg.dare_tol      = RON_FLOAT_C(1e-4);
    cfg.u_min[0]      = RON_FLOAT_C(-10.0);
    cfg.u_max[0]      = RON_FLOAT_C(10.0);

    /* First pass solves K via DARE; Kr = K[0] then gives exact zero
     * steady-state position error for this plant (at equilibrium the
     * velocity state and the command are both zero, so
     * -K[0]*pos + Kr*r == 0 requires Kr == K[0] for pos == r). */
    if (ron_lqr_init(&lqr, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "LQR init failed\n");
        return 1;
    }
    cfg.Kr[0] = lqr.state.K_solved[0][0];
    if (ron_lqr_init(&lqr, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "LQR re-init failed\n");
        return 1;
    }
    (void) printf("LQR DARE gain: K = [%.4f, %.4f], Kr = %.4f\n\n",
                  (double) lqr.state.K_solved[0][0], (double) lqr.state.K_solved[0][1],
                  (double) cfg.Kr[0]);

    (void) printf("Part 1: LQR (exact state)\n step   position   velocity   command\n");
    for (k = 0U; k < 30U; ++k) {
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;

        (void) ron_lqr_step(&lqr, r, RON_FLOAT_C(1.0), u, &status);
        x[0] += x[1];
        x[1] += u[0];

        if ((k % 5U) == 0U) {
            (void) printf("%4u   %8.4f   %8.4f   %8.4f\n", k, (double) x[0], (double) x[1],
                          (double) u[0]);
        }
    }
    (void) printf("final position: %.4f (target 1.0)\n\n", (double) x[0]);
    return 0;
}

/* Part 2: LQG — the same plant and cost, but the controller only sees noisy
 * position measurements and estimates the full state via an embedded
 * Kalman filter. */
static int run_lqg(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg              = {0};
    ron_float_t true_x[2]             = {RON_FLOAT_C(0.0), RON_FLOAT_C(0.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    const ron_float_t meas_noise      = RON_FLOAT_C(0.05);
    unsigned k;

    cfg.n             = 2U;
    cfg.m             = 1U;
    cfg.p             = 1U;
    cfg.gain_mode     = RON_LQG_GAIN_DARE;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.A[0][1]       = RON_FLOAT_C(1.0);
    cfg.A[1][1]       = RON_FLOAT_C(1.0);
    cfg.B[1][0]       = RON_FLOAT_C(1.0);
    cfg.H[0][0]       = RON_FLOAT_C(1.0);
    cfg.Q_noise[0][0] = RON_FLOAT_C(0.001);
    cfg.Q_noise[1][1] = RON_FLOAT_C(0.001);
    cfg.R_noise[0][0] = meas_noise * meas_noise;
    cfg.P0[0][0]      = RON_FLOAT_C(1.0);
    cfg.P0[1][1]      = RON_FLOAT_C(1.0);
    cfg.Q_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.Q_cost[1][1]  = RON_FLOAT_C(1.0);
    cfg.R_cost[0][0]  = RON_FLOAT_C(1.0);
    cfg.dare_max_iter = 200U;
    cfg.dare_tol      = RON_FLOAT_C(1e-4);
    cfg.u_min[0]      = RON_FLOAT_C(-10.0);
    cfg.u_max[0]      = RON_FLOAT_C(10.0);

    /* Same Kr = K[0] reasoning as Part 1 (identical A/B/Q_cost/R_cost, so
     * the separation principle guarantees the same LQR gain). */
    if (ron_lqg_init(&lqg, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "LQG init failed\n");
        return 1;
    }
    cfg.Kr[0] = lqg.K_solved[0][0];
    if (ron_lqg_init(&lqg, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "LQG re-init failed\n");
        return 1;
    }
    srand(1U); /* deterministic demo output */

    (void) printf("Part 2: LQG (noisy measurement)\n step   position   estimate   command\n");
    for (k = 0U; k < 30U; ++k) {
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_float_t z[RON_KF_MAX_MEASUREMENTS];
        ron_float_t x_hat[RON_LQR_MAX_STATES];
        ron_status_t status;

        z[0] = true_x[0] + noise(meas_noise);
        (void) ron_lqg_update(&lqg, z, true);
        (void) ron_lqg_step(&lqg, r, RON_FLOAT_C(1.0), u, &status);

        true_x[0] += true_x[1];
        true_x[1] += u[0];
        (void) ron_lqg_predict(&lqg, u);
        (void) ron_lqg_get_state(&lqg, x_hat);

        if ((k % 5U) == 0U) {
            (void) printf("%4u   %8.4f   %8.4f   %8.4f\n", k, (double) true_x[0], (double) x_hat[0],
                          (double) u[0]);
        }
    }
    (void) printf("final position: %.4f (target 1.0)\n", (double) true_x[0]);
    return 0;
}

int main(void)
{
    if (run_lqr() != 0) {
        return 1;
    }
    return run_lqg();
}
