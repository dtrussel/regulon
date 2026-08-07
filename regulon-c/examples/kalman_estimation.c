/*
 * @file     kalman_estimation.c
 * @brief    Example: tracking a noisy constant-velocity signal with ron_kalman.
 * @module   example_kalman_estimation
 * @doc      RON-IS-001
 * @req      RON-FR-600, RON-FR-602
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * A 2-state (position, velocity) discrete Kalman filter tracks a constant-
 * velocity target from noisy position-only measurements, using only the
 * aggregate header <ron/ron.h>.  Host-only documentation example (uses
 * printf/rand); excluded from all production gates and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_EXAMPLES=ON && cmake --build build
 * Run:    ./build/examples/kalman_estimation
 */

#include <stdio.h>
#include <stdlib.h>

#include "ron/ron.h"

/* Symmetric pseudo-random noise in [-amplitude, amplitude]. */
static ron_float_t noise(ron_float_t amplitude)
{
    ron_float_t unit = (ron_float_t) rand() / (ron_float_t) RAND_MAX; /* [0, 1] */

    return (RON_FLOAT_C(2.0) * unit - RON_FLOAT_C(1.0)) * amplitude;
}

int main(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg          = {0};
    const ron_float_t dt         = RON_FLOAT_C(0.1);
    const ron_float_t true_vel   = RON_FLOAT_C(2.0);
    const ron_float_t meas_noise = RON_FLOAT_C(0.8);
    ron_float_t true_pos         = RON_FLOAT_C(0.0);
    unsigned k;

    cfg.n        = 2U;
    cfg.m        = 1U;
    cfg.p        = 0U;
    cfg.A[0][0]  = RON_FLOAT_C(1.0);
    cfg.A[0][1]  = dt;
    cfg.A[1][1]  = RON_FLOAT_C(1.0);
    cfg.H[0][0]  = RON_FLOAT_C(1.0);
    cfg.Q[0][0]  = RON_FLOAT_C(0.001);
    cfg.Q[1][1]  = RON_FLOAT_C(0.01);
    cfg.R[0][0]  = meas_noise * meas_noise;
    cfg.P0[0][0] = RON_FLOAT_C(10.0);
    cfg.P0[1][1] = RON_FLOAT_C(10.0);

    if (ron_kf_init(&kf, &cfg) != RON_FAULT_NONE) {
        (void) fprintf(stderr, "Kalman init failed\n");
        return 1;
    }

    srand(1U); /* deterministic demo output */

    (void) printf(" time    true_pos   measured   estimate   est_vel\n");
    for (k = 0U; k < 100U; ++k) {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS];
        ron_float_t x_hat[RON_KF_MAX_STATES];

        true_pos += true_vel * dt;
        z[0] = true_pos + noise(meas_noise);

        (void) ron_kf_predict(&kf, NULL);
        (void) ron_kf_update(&kf, z, true);
        (void) ron_kf_get_state(&kf, x_hat);

        if ((k % 10U) == 0U) {
            (void) printf("%5.1f   %8.3f   %8.3f   %8.3f   %8.3f\n",
                          (double) ((ron_float_t) k * dt), (double) true_pos, (double) z[0],
                          (double) x_hat[0], (double) x_hat[1]);
        }
    }
    return 0;
}
