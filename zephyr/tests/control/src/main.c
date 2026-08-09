/*
 * @file     main.c
 * @brief    Behavioural tests for the Regulon Zephyr module.
 * @doc      RON-IS-001
 * SPDX-License-Identifier: MIT
 *
 * These run on the target, not on the host. The host suite under
 * regulon-c/test/ already covers the library's behaviour exhaustively; the
 * point of running here is everything that differs once the code is
 * cross-compiled and executed on an MCU: the target's floating-point
 * behaviour, its ABI and alignment, the Kconfig-selected source set, and the
 * generated ron_modules.h that goes with it.
 *
 * So these check that each module actually *works* on the target rather than
 * merely linking. Tests for optional modules are compiled only when their
 * Kconfig option selected them, which doubles as a check that the generated
 * RON_HAVE_* macros agree with what was built.
 */

#include <zephyr/ztest.h>
#include <ron/ron.h>

/* Tolerance for target floating-point comparisons. Generous enough to hold in
 * single precision on a soft-float target, tight enough to catch a controller
 * that is genuinely misbehaving. */
#define TOL RON_FLOAT_C(0.05)

/* A NaN without <math.h>: the minimal-libc configuration defines no NAN
 * macro, and the whole point of that build is that neither the library nor
 * anything exercising it needs libm. */
static ron_float_t test_nan(void)
{
    return (ron_float_t) __builtin_nan("");
}

ZTEST_SUITE(regulon_module, NULL, NULL, NULL, NULL, NULL);

/* ---------------------------------------------------------------------------
 * PID - the mandatory baseline
 * ------------------------------------------------------------------------- */

static void pid_default_config(ron_pid_config_t *cfg)
{
    *cfg          = (ron_pid_config_t){ 0 };
    cfg->Kp       = RON_FLOAT_C(2.0);
    cfg->Ki       = RON_FLOAT_C(5.0);
    cfg->Kd       = RON_FLOAT_C(0.0);
    cfg->b        = RON_FLOAT_C(1.0);
    cfg->c        = RON_FLOAT_C(1.0);
    cfg->u_min    = -RON_FLOAT_C(10.0);
    cfg->u_max    = RON_FLOAT_C(10.0);
    cfg->I_min    = -RON_FLOAT_C(100.0);
    cfg->I_max    = RON_FLOAT_C(100.0);
    cfg->aw_mode  = RON_AW_BACK_CALC;
    cfg->T_aw     = RON_FLOAT_C(0.05);
}

ZTEST(regulon_module, test_pid_converges_on_target)
{
    ron_pid_config_t cfg;
    ron_pid_instance_t pid;
    ron_float_t y = RON_FLOAT_C(0.0);

    pid_default_config(&cfg);
    zassert_equal(ron_pid_init(&pid, &cfg), RON_FAULT_NONE, "init rejected");

    for (int i = 0; i < 400; i++) {
        ron_float_t u;
        ron_status_t status;

        zassert_equal(ron_pid_step(&pid, RON_FLOAT_C(1.0), y, RON_FLOAT_C(0.01), &u, &status), RON_FAULT_NONE,
                      "step faulted at i=%d", i);
        y += RON_FLOAT_C(0.01) * (u - y);
    }

    zassert_within(y, RON_FLOAT_C(1.0), TOL, "did not converge: y=%d milli", (int) (y * RON_FLOAT_C(1000.0)));
}

ZTEST(regulon_module, test_pid_output_respects_saturation)
{
    ron_pid_config_t cfg;
    ron_pid_instance_t pid;

    pid_default_config(&cfg);
    cfg.u_min = -RON_FLOAT_C(1.0);
    cfg.u_max = RON_FLOAT_C(1.0);
    zassert_equal(ron_pid_init(&pid, &cfg), RON_FAULT_NONE, "init rejected");

    /* A large, persistent error drives the controller hard against its
     * limit; the output must never leave it. */
    for (int i = 0; i < 100; i++) {
        ron_float_t u;
        ron_status_t status;

        zassert_equal(ron_pid_step(&pid, RON_FLOAT_C(1000.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status), RON_FAULT_NONE,
                      "step faulted");
        zassert_true((u >= cfg.u_min) && (u <= cfg.u_max), "output %d milli escaped limits",
                     (int) (u * RON_FLOAT_C(1000.0)));
    }
}

ZTEST(regulon_module, test_pid_fault_latches_and_clears)
{
    ron_pid_config_t cfg;
    ron_pid_instance_t pid;
    ron_float_t u;
    ron_status_t status;

    pid_default_config(&cfg);
    zassert_equal(ron_pid_init(&pid, &cfg), RON_FAULT_NONE, "init rejected");

    /* A non-finite measurement must fault... */
    zassert_not_equal(ron_pid_step(&pid, RON_FLOAT_C(1.0), test_nan(), RON_FLOAT_C(0.01), &u, &status),
                      RON_FAULT_NONE, "NaN input was accepted");

    /* ...and stay faulted on subsequent good input, rather than healing. */
    zassert_not_equal(ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status), RON_FAULT_NONE,
                      "fault did not latch");
    zassert_true((status & RON_STATUS_FAULT) != 0U, "fault status bit not set");

    /* Only an explicit clear resumes control. */
    zassert_equal(ron_pid_fault_clear(&pid), RON_FAULT_NONE, "clear failed");
    zassert_equal(ron_pid_step(&pid, RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.01), &u, &status), RON_FAULT_NONE,
                  "controller did not resume after clear");
}

/* ---------------------------------------------------------------------------
 * Optional modules - compiled only when Kconfig selected them, which also
 * asserts that the generated RON_HAVE_* macros match the built source set.
 * ------------------------------------------------------------------------- */

#if RON_HAVE_FILTER
ZTEST(regulon_module, test_lowpass_tracks_a_step)
{
    ron_lp1_t filter;
    ron_lp1_config_t cfg = { .alpha = RON_FLOAT_C(0.1) };
    ron_float_t y        = RON_FLOAT_C(0.0);

    zassert_equal(ron_lp1_init(&filter, &cfg), RON_FAULT_NONE, "init rejected");

    /* A first-order low-pass must approach a constant input monotonically
     * and never overshoot it. */
    for (int i = 0; i < 200; i++) {
        ron_float_t prev = y;

        zassert_equal(ron_lp1_step(&filter, RON_FLOAT_C(1.0), &y), RON_FAULT_NONE, "step faulted");
        zassert_true(y >= prev, "output decreased on a rising step");
        zassert_true(y <= RON_FLOAT_C(1.0) + TOL, "output overshot the input");
    }
    zassert_within(y, RON_FLOAT_C(1.0), TOL, "did not settle on the input");
}
#endif /* RON_HAVE_FILTER */

#if RON_HAVE_TRAJECTORY
ZTEST(regulon_module, test_trapezoid_reaches_target)
{
    ron_trap_t traj;
    ron_trap_config_t cfg = { .v_max = RON_FLOAT_C(1.0), .a_max = RON_FLOAT_C(2.0) };
    bool finished         = false;
    ron_float_t pos = RON_FLOAT_C(0.0), vel = RON_FLOAT_C(0.0), acc = RON_FLOAT_C(0.0);

    zassert_equal(ron_trap_init(&traj, &cfg, RON_FLOAT_C(0.0)), RON_FAULT_NONE, "init rejected");
    zassert_equal(ron_trap_set_target(&traj, RON_FLOAT_C(5.0)), RON_FAULT_NONE, "target rejected");

    for (int i = 0; (i < 2000) && !finished; i++) {
        zassert_equal(ron_trap_step(&traj, RON_FLOAT_C(0.01), &pos, &vel, &acc, &finished), RON_FAULT_NONE,
                      "step faulted");
        /* The velocity limit is a promise, not a suggestion. */
        zassert_true((vel <= cfg.v_max + TOL) && (vel >= -(cfg.v_max + TOL)),
                     "velocity %d milli exceeded v_max", (int) (vel * RON_FLOAT_C(1000.0)));
    }

    zassert_true(finished, "profile never reported completion");
    zassert_within(pos, RON_FLOAT_C(5.0), TOL, "stopped short of target: %d milli", (int) (pos * RON_FLOAT_C(1000.0)));
}
#endif /* RON_HAVE_TRAJECTORY */

#if RON_HAVE_KALMAN
/* Instances live at file scope: ron_kf_t and especially ron_lqr_t are far
 * too large for a thread stack, and file-scope static is the allocation the
 * library documents for them anyway. */
static ron_kf_t kf_instance;
static ron_kf_config_t kf_cfg;

ZTEST(regulon_module, test_kalman_estimate_converges)
{
    /* Scalar constant-state model: x stays put, z measures it with noise. The
     * estimate must walk from its wrong initial guess to the true value. */
    ron_kf_t *const kf         = &kf_instance;
    ron_kf_config_t *const cfg = &kf_cfg;
    ron_float_t x_hat[RON_KF_MAX_STATES];
    const ron_float_t truth = RON_FLOAT_C(2.0);

    *cfg = (ron_kf_config_t){ 0 };
    cfg->n        = 1U;
    cfg->m        = 1U;
    cfg->p        = 0U;
    cfg->A[0][0]  = RON_FLOAT_C(1.0);
    cfg->H[0][0]  = RON_FLOAT_C(1.0);
    cfg->Q[0][0]  = RON_FLOAT_C(0.001);
    cfg->R[0][0]  = RON_FLOAT_C(0.1);
    cfg->x0[0]    = RON_FLOAT_C(0.0);
    cfg->P0[0][0] = RON_FLOAT_C(1.0);

    zassert_equal(ron_kf_init(kf, cfg), RON_FAULT_NONE, "init rejected");

    for (int i = 0; i < 100; i++) {
        ron_float_t z[RON_KF_MAX_MEASUREMENTS] = { truth };

        zassert_equal(ron_kf_predict(kf, NULL), RON_FAULT_NONE, "predict faulted");
        zassert_equal(ron_kf_update(kf, z, true), RON_FAULT_NONE, "update faulted");
    }

    zassert_equal(ron_kf_get_state(kf, x_hat), RON_FAULT_NONE, "get_state failed");
    zassert_within(x_hat[0], truth, TOL, "estimate did not converge: %d milli",
                   (int) (x_hat[0] * RON_FLOAT_C(1000.0)));
}
#endif /* RON_HAVE_KALMAN */

#if RON_HAVE_LQR
/* ron_lqr_t is ~5.7 kB (it embeds both an observer and a Kalman filter) and
 * ron_lqr_config_t ~3 kB, so these must not go on a stack. */
static ron_lqr_t lqr_instance;
static ron_lqr_config_t lqr_cfg;

ZTEST(regulon_module, test_lqr_dare_solve_stabilises_on_target)
{
    /* The DARE solver is the most numerically demanding thing in the library
     * and the most likely to behave differently on a target FPU, so it is
     * worth running here rather than trusting the host result. */
    ron_lqr_t *const lqr         = &lqr_instance;
    ron_lqr_config_t *const cfg = &lqr_cfg;
    ron_float_t x[RON_LQR_MAX_STATES] = { RON_FLOAT_C(1.0), RON_FLOAT_C(0.0) };

    *cfg = (ron_lqr_config_t){ 0 };

    cfg->n         = 2U;
    cfg->m         = 1U;
    cfg->source    = RON_LQR_SOURCE_EXTERNAL;
    cfg->gain_mode = RON_LQR_GAIN_DARE;
    cfg->x_ext     = x;

    /* Double integrator, dt = 0.1. */
    cfg->A[0][0] = RON_FLOAT_C(1.0); cfg->A[0][1] = RON_FLOAT_C(0.1);
    cfg->A[1][0] = RON_FLOAT_C(0.0); cfg->A[1][1] = RON_FLOAT_C(1.0);
    cfg->B[0][0] = RON_FLOAT_C(0.005);
    cfg->B[1][0] = RON_FLOAT_C(0.1);

    cfg->Q_cost[0][0] = RON_FLOAT_C(1.0);
    cfg->Q_cost[1][1] = RON_FLOAT_C(1.0);
    cfg->R_cost[0][0] = RON_FLOAT_C(1.0);

    /* Single-precision cannot resolve a tolerance near float epsilon at the
     * magnitude of P, so ask for something the arithmetic can reach. */
    cfg->dare_tol      = RON_FLOAT_C(1e-4);
    cfg->dare_max_iter = 500U;

    cfg->u_min[0]  = -RON_FLOAT_C(100.0);
    cfg->u_max[0]  = RON_FLOAT_C(100.0);
    cfg->du_max[0] = RON_FLOAT_C(0.0);

    zassert_equal(ron_lqr_init(lqr, cfg), RON_FAULT_NONE, "DARE solve failed on target");

    /* Closed-loop: the regulator must drive the state to the origin. */
    for (int i = 0; i < 500; i++) {
        ron_float_t r[RON_LQR_MAX_INPUTS] = { RON_FLOAT_C(0.0) };
        ron_float_t u[RON_LQR_MAX_INPUTS];
        ron_status_t status;
        ron_float_t x0 = x[0], x1 = x[1];

        zassert_equal(ron_lqr_step(lqr, r, RON_FLOAT_C(0.1), u, &status), RON_FAULT_NONE, "step faulted");

        x[0] = (cfg->A[0][0] * x0) + (cfg->A[0][1] * x1) + (cfg->B[0][0] * u[0]);
        x[1] = (cfg->A[1][0] * x0) + (cfg->A[1][1] * x1) + (cfg->B[1][0] * u[0]);
    }

    zassert_within(x[0], RON_FLOAT_C(0.0), TOL, "position not regulated: %d milli", (int) (x[0] * RON_FLOAT_C(1000.0)));
    zassert_within(x[1], RON_FLOAT_C(0.0), TOL, "velocity not regulated: %d milli", (int) (x[1] * RON_FLOAT_C(1000.0)));
}
#endif /* RON_HAVE_LQR */

/* ---------------------------------------------------------------------------
 * Build configuration
 * ------------------------------------------------------------------------- */

ZTEST(regulon_module, test_baseline_modules_always_present)
{
    /* The PID core and feed-forward path are the mandatory baseline; no
     * Kconfig combination may switch them off. */
    zassert_equal(RON_HAVE_PID, 1, "PID missing from a build that has the library");
    zassert_equal(RON_HAVE_FEEDFORWARD, 1, "feed-forward missing from the baseline");
}

ZTEST(regulon_module, test_module_dependencies_are_consistent)
{
    /* Kconfig `select` should make these implications hold in the generated
     * header too - if they ever disagree, ron/ron.h would include a header
     * whose implementation was not compiled. */
    if (RON_HAVE_LQG) {
        zassert_equal(RON_HAVE_LQR, 1, "LQG built without LQR");
    }
    if (RON_HAVE_LQR) {
        zassert_equal(RON_HAVE_STATESPACE, 1, "LQR built without state-space");
    }
    if (RON_HAVE_STATESPACE) {
        zassert_equal(RON_HAVE_KALMAN, 1, "state-space built without Kalman");
    }
}
