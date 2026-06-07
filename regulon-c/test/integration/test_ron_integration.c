/*
 * @file     test_ron_integration.c
 * @brief    Full-library integration tests (Phase 11).
 * @module   test_ron_integration
 * @doc      RON-TP-001
 * @req      RON-FR-401, RON-FR-500, RON-FR-701, RON-FR-804, RON-FR-900,
 *           RON-FR-950, RON-QR-031, RON-DC-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * These tests exercise the library as a whole rather than a single module.
 * They include ONLY the aggregate header <ron/ron.h>, so the suite doubles as
 * the single-include / include-topology check (RON-TC-INT-001), and they
 * compose modules into closed control loops to confirm the published APIs work
 * together deterministically (RON-TC-INT-002 .. RON-TC-INT-005).
 */

#include <float.h>
#include <math.h>

#include "ron/ron.h"

#include "unity.h"

#define INT_PI 3.14159265358979323846

void setUp(void)
{
}

void tearDown(void)
{
}

/* =========================================================================
 * Shared helpers
 * ========================================================================= */

static ron_float_t int_abs(ron_float_t v)
{
    return (v < RON_FLOAT_C(0.0)) ? (-v) : v;
}

/* Baseline single-loop PID configuration with wide limits. */
static ron_pid_config_t int_make_pid_cfg(ron_float_t kp, ron_float_t ki, ron_float_t umin,
                                         ron_float_t umax)
{
    ron_pid_config_t cfg;

    cfg.Kp                 = kp;
    cfg.Ki                 = ki;
    cfg.Kd                 = RON_FLOAT_C(0.0);
    cfg.N                  = RON_FLOAT_C(0.0);
    cfg.b                  = RON_FLOAT_C(1.0);
    cfg.c                  = RON_FLOAT_C(1.0);
    cfg.u_min              = umin;
    cfg.u_max              = umax;
    cfg.du_max             = RON_FLOAT_C(0.0);
    cfg.I_min              = RON_FLOAT_C(-1000.0);
    cfg.I_max              = RON_FLOAT_C(1000.0);
    cfg.aw_mode            = RON_AW_BACK_CALC;
    cfg.T_aw               = RON_FLOAT_C(0.05);
    cfg.integ_method       = RON_INTEG_EULER;
    cfg.deriv_mode         = RON_DERIV_ON_MEASUREMENT;
    cfg.tau_sp             = RON_FLOAT_C(0.0);
    cfg.normalise          = false;
    cfg.in_min             = RON_FLOAT_C(0.0);
    cfg.in_max             = RON_FLOAT_C(1.0);
    cfg.out_min            = RON_FLOAT_C(0.0);
    cfg.out_max            = RON_FLOAT_C(1.0);
    cfg.safe_policy        = RON_SAFE_HOLD_LAST;
    cfg.safe_value         = RON_FLOAT_C(0.0);
    cfg.I_overflow_thresh  = RON_FLOAT_C(0.0);
    cfg.sp_reset_threshold = RON_FLOAT_C(0.0);
    cfg.feedforward.mode   = RON_FF_DISABLED;
    cfg.feedforward.gain   = RON_FLOAT_C(0.0);
    cfg.feedforward.N_ff   = RON_FLOAT_C(0.0);
    cfg.fault_cb           = NULL;

    return cfg;
}

/* =========================================================================
 * RON-TC-INT-001 | RON-DC-001 — Aggregate header & include topology
 *
 * The file includes only <ron/ron.h>; reaching one public entry point per
 * module proves every module header is reachable and linkable through the
 * umbrella header with no circular-include or symbol-collision problems.  Each
 * entry point is called with NULL so it returns RON_FAULT_NULL_POINTER without
 * side effects.  Calls are guarded by the generated RON_HAVE_<MODULE> macros.
 * ========================================================================= */
void test_ron_tc_int_001(void)
{
    /* Mandatory baseline. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_pid_init(NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_feedforward_config_validate(NULL));

#if RON_HAVE_FILTER
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_lp1_reset(NULL));
#endif
#if RON_HAVE_GAIN_SCHED
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_gs_init(NULL));
#endif
#if RON_HAVE_TRAJECTORY
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_trap_init(NULL, NULL, RON_FLOAT_C(0.0)));
#endif
#if RON_HAVE_CASCADE
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_cascade_init(NULL, NULL, NULL));
#endif
#if RON_HAVE_KALMAN
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_kf_init(NULL, NULL));
#endif
#if RON_HAVE_STATESPACE
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_obs_init(NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_ss_init(NULL, NULL));
#endif
#if RON_HAVE_AUTOTUNE
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_init(NULL, NULL));
#endif
#if RON_HAVE_HEALTH
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_health_init(NULL, NULL));
#endif
#if RON_HAVE_METRICS
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_metrics_init(NULL, NULL));
#endif
}

/* =========================================================================
 * RON-TC-INT-002 | RON-FR-401 / FR-500 / FR-900 / FR-950 / FR-031
 *
 * End-to-end loop: a trapezoidal trajectory feeds the position setpoint into a
 * cascade (position->velocity) controller driving a two-state plant; a health
 * monitor and a metrics accumulator observe the same loop.  Asserts tracking,
 * fault-free operation, healthy status, finite metrics, and bit-identical
 * determinism across two runs (RON-QR-031).
 * ========================================================================= */

/* One cascade control step against the two-state plant; returns the command u.
 * pos/vel are advanced in place. */
static ron_float_t int_loop2_step(ron_cascade_instance_t *casc, ron_float_t r_pos, ron_float_t *pos,
                                  ron_float_t *vel, ron_float_t dt, ron_cascade_status_t *status,
                                  ron_fault_t *fault)
{
    const ron_float_t tau = RON_FLOAT_C(0.05);
    ron_float_t u         = RON_FLOAT_C(0.0);

    *fault = ron_cascade_step(casc, r_pos, *pos, *vel, dt, &u, status);
    /* Inner velocity plant: first-order lag toward the command u. */
    *vel = *vel + (dt / tau) * (u - *vel);
    /* Outer position plant: integrate velocity. */
    *pos = *pos + dt * (*vel);
    return u;
}

/* Run the full INT-002 loop, capturing the command trajectory in u_log. */
static void int_run_loop2(ron_float_t *u_log, unsigned n, ron_float_t *final_pos,
                          ron_health_status_t *final_health, ron_metrics_result_t *final_metrics,
                          bool *any_fault)
{
    ron_cascade_instance_t casc;
    ron_trap_t traj;
    ron_health_t mon;
    ron_metrics_t met;
    ron_pid_config_t outer =
        int_make_pid_cfg(RON_FLOAT_C(4.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0));
    ron_pid_config_t inner = int_make_pid_cfg(RON_FLOAT_C(8.0), RON_FLOAT_C(40.0),
                                              RON_FLOAT_C(-50.0), RON_FLOAT_C(50.0));
    ron_trap_config_t tcfg = {RON_FLOAT_C(2.0), RON_FLOAT_C(4.0)};
    ron_health_config_t hcfg;
    ron_metrics_config_t mcfg;
    ron_float_t pos      = RON_FLOAT_C(0.0);
    ron_float_t vel      = RON_FLOAT_C(0.0);
    const ron_float_t dt = RON_FLOAT_C(0.005);
    unsigned k;

    hcfg.t_sat_max          = RON_FLOAT_C(2.0);
    hcfg.err_diverge_thresh = RON_FLOAT_C(50.0);
    hcfg.osc_count_thresh   = 12U;
    hcfg.dead_band          = RON_FLOAT_C(0.0);
    hcfg.dropout_time       = RON_FLOAT_C(5.0);
    hcfg.ss_err_thresh      = RON_FLOAT_C(0.1);
    hcfg.settling_time      = RON_FLOAT_C(10.0);
    hcfg.cb                 = NULL;

    mcfg.mode           = RON_METRICS_CUMULATIVE;
    mcfg.window_steps   = 0U;
    mcfg.band_pct       = RON_FLOAT_C(0.05);
    mcfg.settle_confirm = RON_FLOAT_C(0.1);
    mcfg.step_thresh    = RON_FLOAT_C(0.2);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&casc, &outer, &inner));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&traj, &tcfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&traj, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_init(&mon, &hcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_init(&met, &mcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_enable(&met, true));

    *any_fault = false;
    for (k = 0U; k < n; ++k) {
        ron_float_t r_pos = RON_FLOAT_C(0.0);
        ron_float_t tvel  = RON_FLOAT_C(0.0);
        ron_float_t tacc  = RON_FLOAT_C(0.0);
        bool finished     = false;
        ron_cascade_status_t status;
        ron_fault_t fault;
        ron_float_t u;

        (void) ron_trap_step(&traj, dt, &r_pos, &tvel, &tacc, &finished);
        u        = int_loop2_step(&casc, r_pos, &pos, &vel, dt, &status, &fault);
        u_log[k] = u;
        if (fault != RON_FAULT_NONE) {
            *any_fault = true;
        }
        (void) ron_health_step(&mon, r_pos, pos, u, dt);
        (void) ron_metrics_step(&met, r_pos, pos, dt);
    }

    *final_pos = pos;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_health_get(&mon, final_health));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_metrics_get(&met, final_metrics));
}

void test_ron_tc_int_002(void)
{
    enum { N = 1200 };
    static ron_float_t u_a[N];
    static ron_float_t u_b[N];
    ron_float_t pos_a;
    ron_float_t pos_b;
    ron_health_status_t health_a;
    ron_health_status_t health_b;
    ron_metrics_result_t met_a;
    ron_metrics_result_t met_b;
    bool fault_a;
    bool fault_b;
    unsigned k;

    int_run_loop2(u_a, (unsigned) N, &pos_a, &health_a, &met_a, &fault_a);
    int_run_loop2(u_b, (unsigned) N, &pos_b, &health_b, &met_b, &fault_b);

    /* No fault anywhere in the composed loop. */
    TEST_ASSERT_FALSE(fault_a);

    /* Tracking: the plant reaches the 1.0 position setpoint. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.05), RON_FLOAT_C(1.0), pos_a);

    /* The loop is stable: the health monitor reports neither divergence nor
     * oscillation.  (OUTPUT_STUCK / SP_UNREACHABLE may latch once the loop
     * settles and the command holds constant — expected for a quiescent loop.) */
    TEST_ASSERT_EQUAL_UINT8(
        0U, (ron_health_status_t) (health_a & (ron_health_status_t) (RON_HEALTH_DIVERGING |
                                                                     RON_HEALTH_OSCILLATING)));

    /* Metrics are finite, positive integrals and a measured settling time. */
    TEST_ASSERT_TRUE(met_a.IAE > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(met_a.ISE > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(met_a.ITAE > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(met_a.settling_time >= RON_FLOAT_C(0.0));

    /* Determinism (RON-QR-031): two identical runs match bit-for-bit. */
    for (k = 0U; k < (unsigned) N; ++k) {
        TEST_ASSERT_EQUAL_UINT8(0, (u_a[k] == u_b[k]) ? 0 : 1);
    }
    TEST_ASSERT_EQUAL_UINT8(0, (pos_a == pos_b) ? 0 : 1);
}

/* =========================================================================
 * RON-TC-INT-003 | RON-FR-701 — Estimator-in-the-loop
 *
 * A measurement filter (first-order LPF) feeds an embedded Luenberger observer
 * inside a state-space controller that drives a scalar plant.  Asserts the
 * observer estimate converges to the true state and the saturated output never
 * exceeds the configured limits (saturation honored end-to-end).
 * ========================================================================= */
void test_ron_tc_int_003(void)
{
    const ron_float_t a   = RON_FLOAT_C(0.9);
    const ron_float_t b   = RON_FLOAT_C(0.1);
    const ron_float_t dt  = RON_FLOAT_C(0.01);
    const ron_float_t umx = RON_FLOAT_C(2.0);
    ron_ss_t ss;
    ron_ss_config_t cfg = {0};
    ron_lp1_t filt;
    ron_lp1_config_t fcfg = {RON_FLOAT_C(0.5)};
    ron_float_t x         = RON_FLOAT_C(0.0); /* true plant state */
    ron_float_t u_prev    = RON_FLOAT_C(0.0);
    bool saturated_seen   = false;
    unsigned k;

    cfg.n               = 1U;
    cfg.source          = RON_SS_SOURCE_LUENBERGER;
    cfg.K[0]            = RON_FLOAT_C(1.5);
    cfg.Kr              = RON_FLOAT_C(1.5);
    cfg.use_integral    = false;
    cfg.u_min           = -umx;
    cfg.u_max           = umx;
    cfg.du_max          = RON_FLOAT_C(0.0);
    cfg.obs_cfg.n       = 1U;
    cfg.obs_cfg.m       = 1U;
    cfg.obs_cfg.p       = 1U;
    cfg.obs_cfg.A[0][0] = a;
    cfg.obs_cfg.B[0][0] = b;
    cfg.obs_cfg.C[0][0] = RON_FLOAT_C(1.0);
    cfg.obs_cfg.L[0][0] = RON_FLOAT_C(0.5);
    cfg.obs_cfg.x0[0]   = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ss_init(&ss, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_lp1_init(&filt, &fcfg));

    for (k = 0U; k < 600U; ++k) {
        ron_float_t y_meas = x;
        ron_float_t y_filt = RON_FLOAT_C(0.0);
        ron_float_t y_vec[RON_SS_MAX_OUTPUTS];
        ron_float_t u_vec[RON_SS_MAX_INPUTS];
        ron_float_t u = RON_FLOAT_C(0.0);
        ron_status_t status;

        (void) ron_lp1_step(&filt, y_meas, &y_filt);
        y_vec[0] = y_filt;
        u_vec[0] = u_prev;
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_ss_observer_step(&ss, y_vec, u_vec));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_ss_step(&ss, RON_FLOAT_C(5.0), dt, &u, &status));

        /* Output must always respect the configured saturation limits. */
        TEST_ASSERT_TRUE(u <= umx + RON_FLOAT_C(1.0e-4));
        TEST_ASSERT_TRUE(u >= -umx - RON_FLOAT_C(1.0e-4));
        if ((status & RON_STATUS_SATURATED) != 0U) {
            saturated_seen = true;
        }

        x      = (a * x) + (b * u);
        u_prev = u;
    }

    /* The large reference forces the output to saturate at least once. */
    TEST_ASSERT_TRUE(saturated_seen);

    /* The embedded observer estimate has converged to the true plant state. */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.01), x, ss.observer.state.x_hat[0]);
}

/* =========================================================================
 * RON-TC-INT-004 | RON-FR-804 — Auto-tune then deploy
 *
 * Drive the relay auto-tuner with a deterministic synthetic oscillation until
 * it produces gains (the open-loop injection pattern from RON-TC-AT-003,
 * Ku = 4, Tu = 0.5), apply them to the PID, then run the tuned controller on a
 * first-order plant and confirm it converges to the setpoint without fault.
 * ========================================================================= */
void test_ron_tc_int_004(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg =
        int_make_pid_cfg(RON_FLOAT_C(1.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-10.0), RON_FLOAT_C(10.0));
    ron_at_config_t cfg;

    /* A derivative filter is required for the auto-tuned Kd term to be stable. */
    pcfg.N                = RON_FLOAT_C(10.0);
    const ron_float_t dt  = RON_FLOAT_C(0.001);
    const ron_float_t amp = RON_FLOAT_C(0.5) / (ron_float_t) INT_PI;
    ron_float_t u         = RON_FLOAT_C(0.0);
    ron_float_t y         = RON_FLOAT_C(0.0);
    unsigned k;

    cfg.relay_amplitude = RON_FLOAT_C(0.5);
    cfg.hysteresis      = RON_FLOAT_C(0.05);
    cfg.u_bias          = RON_FLOAT_C(0.0);
    cfg.min_cycles      = 5U;
    cfg.timeout_s       = RON_FLOAT_C(30.0);
    cfg.tuning_rule     = RON_AT_RULE_ZN;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    /* Inject a synthetic limit cycle until the estimator completes. */
    for (k = 0U; k < 200000U; ++k) {
        double t      = (double) k * (double) dt;
        ron_float_t s = amp * (ron_float_t) sin((2.0 * INT_PI) * t / 0.5);
        (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), s, dt, &u);
        if (at.state.done || at.state.aborted) {
            break;
        }
    }
    TEST_ASSERT_TRUE(at.state.done);
    TEST_ASSERT_FALSE(at.state.aborted);

    /* Deploy: the computed gains land in the live PID (RON-FR-804). */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_apply(&at, &pid));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Kp_result, pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Ki_result, pid.config.Ki);

    /* Run the tuned loop on a first-order plant; expect stable convergence. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_reset(&pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_pid_set_mode(&pid, RON_MODE_AUTOMATIC, RON_FLOAT_C(0.0)));
    y = RON_FLOAT_C(0.0);
    for (k = 0U; k < 5000U; ++k) {
        const ron_float_t tau = RON_FLOAT_C(0.1);
        ron_status_t status;
        ron_float_t uc = RON_FLOAT_C(0.0);

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_pid_step(&pid, RON_FLOAT_C(1.0), y, dt, &uc, &status));
        y = y + (dt / tau) * (uc - y);
        TEST_ASSERT_TRUE(int_abs(y) < RON_FLOAT_C(100.0)); /* bounded throughout */
    }
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.1), RON_FLOAT_C(1.0), y);
}

/* =========================================================================
 * RON-TC-INT-005 | RON-FR-062 — Multi-instance isolation (no global state)
 *
 * Two independent cascade loops with different setpoints run concurrently; the
 * command trajectory of loop A is bit-identical whether it runs alone or
 * interleaved with loop B, proving the library holds no shared mutable state.
 * ========================================================================= */
void test_ron_tc_int_005(void)
{
    enum { N = 600 };
    static ron_float_t u_solo[N];
    static ron_float_t u_paired[N];
    ron_cascade_instance_t a1;
    ron_cascade_instance_t a2;
    ron_cascade_instance_t b2;
    ron_pid_config_t outer =
        int_make_pid_cfg(RON_FLOAT_C(4.0), RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0), RON_FLOAT_C(5.0));
    ron_pid_config_t inner = int_make_pid_cfg(RON_FLOAT_C(8.0), RON_FLOAT_C(40.0),
                                              RON_FLOAT_C(-50.0), RON_FLOAT_C(50.0));
    const ron_float_t dt   = RON_FLOAT_C(0.005);
    ron_float_t pa1        = RON_FLOAT_C(0.0);
    ron_float_t va1        = RON_FLOAT_C(0.0);
    ron_float_t pa2        = RON_FLOAT_C(0.0);
    ron_float_t va2        = RON_FLOAT_C(0.0);
    ron_float_t pb2        = RON_FLOAT_C(0.0);
    ron_float_t vb2        = RON_FLOAT_C(0.0);
    unsigned k;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&a1, &outer, &inner));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&a2, &outer, &inner));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_cascade_init(&b2, &outer, &inner));

    /* Loop A in isolation. */
    for (k = 0U; k < (unsigned) N; ++k) {
        ron_cascade_status_t st;
        ron_fault_t ft;
        u_solo[k] = int_loop2_step(&a1, RON_FLOAT_C(1.0), &pa1, &va1, dt, &st, &ft);
    }

    /* Loop A interleaved with an independent loop B (different setpoint). */
    for (k = 0U; k < (unsigned) N; ++k) {
        ron_cascade_status_t st;
        ron_fault_t ft;
        u_paired[k] = int_loop2_step(&a2, RON_FLOAT_C(1.0), &pa2, &va2, dt, &st, &ft);
        (void) int_loop2_step(&b2, RON_FLOAT_C(-2.0), &pb2, &vb2, dt, &st, &ft);
    }

    for (k = 0U; k < (unsigned) N; ++k) {
        TEST_ASSERT_EQUAL_UINT8(0, (u_solo[k] == u_paired[k]) ? 0 : 1);
    }
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_int_001);
    RUN_TEST(test_ron_tc_int_002);
    RUN_TEST(test_ron_tc_int_003);
    RUN_TEST(test_ron_tc_int_004);
    RUN_TEST(test_ron_tc_int_005);
    return UNITY_END();
}
