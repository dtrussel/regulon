/*
 * @file     bench_step.c
 * @brief    Host timing benchmark for representative module step functions.
 * @module   bench_step
 * @doc      RON-IS-001
 * @req      RON-PR-001, RON-PR-003
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Reports average and worst-observed per-call wall-clock time for one step
 * of each of the library's heavier (matrix-bearing) modules, against the
 * RON-PR-003 design target of supporting at least a 10 kHz sample rate
 * (a 100 microsecond per-step budget). This is a host smoke benchmark, not
 * a certified WCET analysis: RON-PR-001/RON-PR-003 explicitly defer the
 * certified timing budget to target-specific static or measurement-based
 * analysis during integration. A result far outside the budget on ordinary
 * benchmarking hardware is still a meaningful regression signal.
 *
 * Host-only informational program (uses printf/time.h); excluded from all
 * production gates and cross-compile builds.
 *
 * Build:  cmake -B build -S regulon-c -DRON_BUILD_BENCHMARKS=ON && cmake --build build
 * Run:    ./build/bench/bench_step
 */

#define _POSIX_C_SOURCE 199309L /* clock_gettime/CLOCK_MONOTONIC under strict -std=c11 */

#include <stdio.h>
#include <time.h>

#include "ron/ron.h"

/* Design-target budget from RON-PR-003 (>= 10 kHz sample rate). */
#define BENCH_BUDGET_NS (100000.0) /* 100 microseconds */
/* A gross-regression threshold, not a certified limit: catches an
 * accidental unbounded loop/allocation, not ordinary host/CI jitter. */
#define BENCH_REGRESSION_NS (100000.0 * 1000.0) /* 100 milliseconds */

#define BENCH_ITERATIONS 20000U

static double now_ns(void)
{
    struct timespec ts;

    (void) clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((double) ts.tv_sec * 1.0e9) + (double) ts.tv_nsec;
}

/* Runs `step_once` BENCH_ITERATIONS times, reports average/max time in ns,
 * and returns non-zero only on a gross regression (not a soft budget miss:
 * this host is not the certified target). */
static int bench_report(const char *name, double (*step_once)(void *ctx), void *ctx)
{
    double total_ns = 0.0;
    double max_ns   = 0.0;
    unsigned i;

    for (i = 0U; i < BENCH_ITERATIONS; ++i) {
        double elapsed = step_once(ctx);

        total_ns += elapsed;
        if (elapsed > max_ns) {
            max_ns = elapsed;
        }
    }

    {
        double avg_ns = total_ns / (double) BENCH_ITERATIONS;
        const char *verdict =
            (avg_ns <= BENCH_BUDGET_NS) ? "within 10 kHz budget" : "over 10 kHz budget";

        (void) printf("%-18s avg=%9.1f ns   max=%9.1f ns   (%s)\n", name, avg_ns, max_ns, verdict);
        return (avg_ns > BENCH_REGRESSION_NS) ? 1 : 0;
    }
}

static double step_pid(void *ctx)
{
    ron_pid_instance_t *pid = (ron_pid_instance_t *) ctx;
    ron_float_t u;
    ron_status_t status;
    double t0 = now_ns();

    (void) ron_pid_step(pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.5), RON_FLOAT_C(0.001), &u, &status);
    return now_ns() - t0;
}

static double step_kalman(void *ctx)
{
    ron_kf_t *kf                           = (ron_kf_t *) ctx;
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(1.0)};
    double t0                              = now_ns();

    (void) ron_kf_predict(kf, NULL);
    (void) ron_kf_update(kf, z, true);
    return now_ns() - t0;
}

static double step_statespace(void *ctx)
{
    ron_ss_t *ss = (ron_ss_t *) ctx;
    ron_float_t u;
    ron_status_t status;
    double t0 = now_ns();

    (void) ron_ss_step(ss, RON_FLOAT_C(1.0), RON_FLOAT_C(0.001), &u, &status);
    return now_ns() - t0;
}

static double step_lqr(void *ctx)
{
    ron_lqr_t *lqr                    = (ron_lqr_t *) ctx;
    ron_float_t r[RON_LQR_MAX_INPUTS] = {RON_FLOAT_C(1.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;
    double t0 = now_ns();

    (void) ron_lqr_step(lqr, r, RON_FLOAT_C(0.001), u, &status);
    return now_ns() - t0;
}

static double step_lqg(void *ctx)
{
    ron_lqg_t *lqg                         = (ron_lqg_t *) ctx;
    ron_float_t z[RON_KF_MAX_MEASUREMENTS] = {RON_FLOAT_C(1.0)};
    ron_float_t r[RON_LQR_MAX_INPUTS]      = {RON_FLOAT_C(1.0)};
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_status_t status;
    double t0 = now_ns();

    (void) ron_lqg_update(lqg, z, true);
    (void) ron_lqg_step(lqg, r, RON_FLOAT_C(0.001), u, &status);
    return now_ns() - t0;
}

int main(void)
{
    ron_pid_instance_t pid;
    ron_pid_config_t pid_cfg = {0};
    ron_kf_t kf;
    ron_kf_config_t kf_cfg = {0};
    ron_ss_t ss;
    ron_ss_config_t ss_cfg = {0};
    ron_lqr_t lqr;
    ron_lqr_config_t lqr_cfg = {0};
    ron_lqg_t lqg;
    ron_lqg_config_t lqg_cfg              = {0};
    ron_float_t x_ext[RON_LQR_MAX_STATES] = {0};
    int failures                          = 0;

    pid_cfg.Kp    = RON_FLOAT_C(1.0);
    pid_cfg.Ki    = RON_FLOAT_C(0.5);
    pid_cfg.u_min = RON_FLOAT_C(-10.0);
    pid_cfg.u_max = RON_FLOAT_C(10.0);
    pid_cfg.I_min = RON_FLOAT_C(-100.0);
    pid_cfg.I_max = RON_FLOAT_C(100.0);
    (void) ron_pid_init(&pid, &pid_cfg);

    kf_cfg.n       = (uint8_t) RON_KF_MAX_STATES;
    kf_cfg.m       = 1U;
    kf_cfg.p       = 0U;
    kf_cfg.H[0][0] = RON_FLOAT_C(1.0);
    kf_cfg.R[0][0] = RON_FLOAT_C(1.0);
    {
        uint8_t i;

        for (i = 0U; i < kf_cfg.n; i++) {
            kf_cfg.A[i][i]  = RON_FLOAT_C(1.0);
            kf_cfg.Q[i][i]  = RON_FLOAT_C(0.01);
            kf_cfg.P0[i][i] = RON_FLOAT_C(1.0);
        }
    }
    (void) ron_kf_init(&kf, &kf_cfg);

    ss_cfg.n      = (uint8_t) RON_SS_MAX_STATES;
    ss_cfg.source = RON_SS_SOURCE_EXTERNAL;
    ss_cfg.x_ext  = x_ext;
    ss_cfg.u_min  = RON_FLOAT_C(-10.0);
    ss_cfg.u_max  = RON_FLOAT_C(10.0);
    (void) ron_ss_init(&ss, &ss_cfg);

    lqr_cfg.n         = (uint8_t) RON_LQR_MAX_STATES;
    lqr_cfg.m         = (uint8_t) RON_LQR_MAX_INPUTS;
    lqr_cfg.source    = RON_LQR_SOURCE_EXTERNAL;
    lqr_cfg.x_ext     = x_ext;
    lqr_cfg.gain_mode = RON_LQR_GAIN_PRECOMPUTED;
    {
        uint8_t j;

        for (j = 0U; j < lqr_cfg.m; j++) {
            lqr_cfg.u_min[j] = RON_FLOAT_C(-10.0);
            lqr_cfg.u_max[j] = RON_FLOAT_C(10.0);
        }
    }
    (void) ron_lqr_init(&lqr, &lqr_cfg);

    lqg_cfg.n             = (uint8_t) RON_LQR_MAX_STATES;
    lqg_cfg.m             = (uint8_t) RON_LQR_MAX_INPUTS;
    lqg_cfg.p             = 1U;
    lqg_cfg.gain_mode     = RON_LQG_GAIN_PRECOMPUTED;
    lqg_cfg.H[0][0]       = RON_FLOAT_C(1.0);
    lqg_cfg.R_noise[0][0] = RON_FLOAT_C(1.0);
    {
        uint8_t i;
        uint8_t j;

        for (i = 0U; i < lqg_cfg.n; i++) {
            lqg_cfg.A[i][i]       = RON_FLOAT_C(1.0);
            lqg_cfg.Q_noise[i][i] = RON_FLOAT_C(0.01);
            lqg_cfg.P0[i][i]      = RON_FLOAT_C(1.0);
        }
        for (j = 0U; j < lqg_cfg.m; j++) {
            lqg_cfg.u_min[j] = RON_FLOAT_C(-10.0);
            lqg_cfg.u_max[j] = RON_FLOAT_C(10.0);
        }
    }
    (void) ron_lqg_init(&lqg, &lqg_cfg);

    (void) printf("Regulon step-function timing benchmark (%u iterations each,\n"
                  "RON-PR-003 design target: 10 kHz => %.0f ns/step budget)\n\n",
                  BENCH_ITERATIONS, BENCH_BUDGET_NS);

    failures |= bench_report("pid", step_pid, &pid);
    failures |= bench_report("kalman", step_kalman, &kf);
    failures |= bench_report("statespace", step_statespace, &ss);
    failures |= bench_report("lqr", step_lqr, &lqr);
    failures |= bench_report("lqg", step_lqg, &lqg);

    if (failures != 0) {
        (void) fprintf(stderr, "\nGross timing regression detected (>100 ms/step).\n");
        return 1;
    }
    return 0;
}
