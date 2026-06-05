/*
 * @file     test_ron_autotune.c
 * @brief    Relay-feedback PID auto-tuner unit tests.
 * @module   test_ron_autotune
 * @doc      RON-TP-001
 * @req      RON-FR-800, RON-FR-801, RON-FR-802, RON-FR-803,
 *           RON-FR-804, RON-FR-805, RON-FR-806, RON-FR-807
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include <float.h>
#include <math.h>

#include "ron/ron_autotune.h"

#include "test_ron_pid_common.h"
#include "unity.h"

#define AT_PI 3.14159265358979323846

void setUp(void)
{
}

void tearDown(void)
{
}

/* =========================================================================
 * Test helpers
 * ========================================================================= */

static ron_float_t test_at_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

static ron_float_t test_at_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

static ron_float_t test_at_make_neg_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return -(big * big);
}

/* A valid baseline configuration (5 cycles, ZN rule). */
static ron_at_config_t test_at_default_cfg(void)
{
    ron_at_config_t cfg;

    cfg.relay_amplitude = RON_FLOAT_C(0.5);
    cfg.hysteresis      = RON_FLOAT_C(0.05);
    cfg.u_bias          = RON_FLOAT_C(0.0);
    cfg.min_cycles      = 5U;
    cfg.timeout_s       = RON_FLOAT_C(30.0);
    cfg.tuning_rule     = RON_AT_RULE_ZN;
    return cfg;
}

/*
 * Drive an instance with a deterministic synthetic sine PV of the given
 * amplitude and period (open-loop injection — the relay output is ignored).
 * Returns true once the run reaches the done state.
 *
 * With amplitude A = 0.5/pi and period 0.5 s the estimator yields
 * Ku = 4d/(pi*A) = 4.0 and Tu = 0.5 (d = 0.5), per RON-TC-AT-003.
 */
static bool test_at_run_sine(ron_at_t *at, ron_float_t amp, ron_float_t period, ron_float_t dt)
{
    unsigned k;
    ron_float_t u = RON_FLOAT_C(0.0);

    for (k = 0U; k < 200000U; ++k) {
        double t      = (double) k * (double) dt;
        ron_float_t y = amp * (ron_float_t) sin((2.0 * AT_PI) * t / (double) period);
        (void) ron_autotune_step(at, RON_FLOAT_C(0.0), y, dt, &u);
        if (at->state.done || at->state.aborted) {
            break;
        }
    }
    return at->state.done;
}

/* First-order plant for the closed-loop integration test. */
static ron_float_t test_at_plant_step(ron_float_t y, ron_float_t u, ron_float_t dt)
{
    const ron_float_t tau = RON_FLOAT_C(0.1);
    const ron_float_t k   = RON_FLOAT_C(1.0);

    return y + (dt / tau) * ((k * u) - y);
}

/* =========================================================================
 * RON-TC-AT-001 | RON-FR-800 — Relay excites a sustained plant oscillation
 * ========================================================================= */
void test_ron_tc_at_001(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t u         = RON_FLOAT_C(0.0);
    ron_float_t y         = RON_FLOAT_C(0.0);
    unsigned k;
    ron_float_t Ku = RON_FLOAT_C(0.0);
    ron_float_t Tu = RON_FLOAT_C(0.0);

    cfg.relay_amplitude = RON_FLOAT_C(1.0);
    cfg.hysteresis      = RON_FLOAT_C(0.1);
    cfg.min_cycles      = 3U;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));

    /* start guards */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_start(NULL, &pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_start(&at, NULL));
    {
        ron_at_t un           = {0};
        ron_pid_instance_t up = {0};
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_start(&un, &pid));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_start(&at, &up));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));
    /* PID parked in manual; relay loop closed around a first-order plant */
    TEST_ASSERT_EQUAL_INT(RON_MODE_MANUAL, (int) pid.state.mode);

    for (k = 0U; k < 60000U; ++k) {
        (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), y, RON_FLOAT_C(0.001), &u);
        /* RON-FR-806: relay output bounded throughout */
        TEST_ASSERT_TRUE(u >= (cfg.u_bias - cfg.relay_amplitude));
        TEST_ASSERT_TRUE(u <= (cfg.u_bias + cfg.relay_amplitude));
        y = test_at_plant_step(y, u, RON_FLOAT_C(0.001));
        if (at.state.done) {
            break;
        }
    }

    TEST_ASSERT_TRUE(at.state.done);
    TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_AT_DONE, at.state.phase);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_results(&at, &Ku, &Tu, NULL, NULL, NULL));
    TEST_ASSERT_TRUE(Ku > RON_FLOAT_C(0.0));
    TEST_ASSERT_TRUE(Tu > RON_FLOAT_C(0.0));
}

/* =========================================================================
 * RON-TC-AT-002 | RON-FR-801 — Configuration validation
 * ========================================================================= */
void test_ron_tc_at_002(void)
{
    ron_at_t at;
    ron_at_config_t cfg = test_at_default_cfg();

    /* NULL pointers */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_init(NULL, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_init(&at, NULL));

    /* relay_amplitude <= 0 or non-finite */
    cfg                 = test_at_default_cfg();
    cfg.relay_amplitude = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));
    cfg.relay_amplitude = test_at_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* hysteresis < 0 or non-finite */
    cfg            = test_at_default_cfg();
    cfg.hysteresis = RON_FLOAT_C(-0.1);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));
    cfg.hysteresis = test_at_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* u_bias non-finite */
    cfg        = test_at_default_cfg();
    cfg.u_bias = test_at_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* min_cycles == 0 */
    cfg            = test_at_default_cfg();
    cfg.min_cycles = 0U;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* timeout_s <= 0 or non-finite */
    cfg           = test_at_default_cfg();
    cfg.timeout_s = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));
    cfg.timeout_s = test_at_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* tuning_rule out of range */
    cfg             = test_at_default_cfg();
    cfg.tuning_rule = (ron_at_rule_t) 7;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_init(&at, &cfg));

    /* Valid configuration initialises to IDLE */
    cfg = test_at_default_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_TRUE(at.state.is_initialised);
    TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_AT_IDLE, at.state.phase);
    TEST_ASSERT_FALSE(at.state.done);
}

/* =========================================================================
 * RON-TC-AT-003 | RON-FR-802 — Ku / Tu estimation accuracy
 * ========================================================================= */
void test_ron_tc_at_003(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t amp       = (ron_float_t) (0.5 / AT_PI); /* -> Ku = 4.0 */
    ron_float_t Ku        = RON_FLOAT_C(0.0);
    ron_float_t Tu        = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    TEST_ASSERT_TRUE(test_at_run_sine(&at, amp, RON_FLOAT_C(0.5), RON_FLOAT_C(0.001)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_results(&at, &Ku, &Tu, NULL, NULL, NULL));

    /* Within 10% of the true values Ku = 4.0, Tu = 0.5 s */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.40), RON_FLOAT_C(4.0), Ku);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(0.05), RON_FLOAT_C(0.5), Tu);
}

/* =========================================================================
 * RON-TC-AT-003b | RON-FR-802 — Oscillation too small to measure aborts
 * ========================================================================= */
void test_ron_tc_at_003b(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t u         = RON_FLOAT_C(0.0);
    unsigned k;

    cfg.min_cycles = 2U; /* needs 4 crossings */

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    /* Sign alternates every sample with vanishing amplitude -> insufficient. */
    for (k = 0U; k < 50U; ++k) {
        ron_float_t y = ((k % 2U) == 0U) ? RON_FLOAT_C(1.0e-9) : RON_FLOAT_C(-1.0e-9);
        (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), y, RON_FLOAT_C(0.001), &u);
        if (at.state.aborted) {
            break;
        }
    }

    TEST_ASSERT_TRUE(at.state.aborted);
    TEST_ASSERT_FALSE(at.state.done);
    TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_AT_ABORTED, at.state.phase);
}

/* =========================================================================
 * RON-TC-AT-004 | RON-FR-803 — Tuning-rule selection (ZN/TL/OS/NOS)
 * ========================================================================= */
void test_ron_tc_at_004(void)
{
    const ron_at_rule_t rules[4] = {RON_AT_RULE_ZN, RON_AT_RULE_TL, RON_AT_RULE_SOME_OS,
                                    RON_AT_RULE_NO_OS};
    const ron_float_t kp_f[4]    = {RON_FLOAT_C(0.60), RON_FLOAT_C(0.45), RON_FLOAT_C(0.33),
                                    RON_FLOAT_C(0.20)};
    const ron_float_t ti_f[4]    = {RON_FLOAT_C(0.50), RON_FLOAT_C(2.20), RON_FLOAT_C(0.50),
                                    RON_FLOAT_C(0.50)};
    const ron_float_t td_f[4]    = {RON_FLOAT_C(0.125), RON_FLOAT_C(0.158), RON_FLOAT_C(0.333),
                                    RON_FLOAT_C(0.333)};
    ron_float_t amp              = (ron_float_t) (0.5 / AT_PI);
    unsigned r;

    for (r = 0U; r < 4U; ++r) {
        ron_at_t at;
        ron_pid_instance_t pid;
        ron_pid_config_t pcfg = test_ron_make_pid_cfg();
        ron_at_config_t cfg   = test_at_default_cfg();
        ron_float_t Ku        = RON_FLOAT_C(0.0);
        ron_float_t Tu        = RON_FLOAT_C(0.0);
        ron_float_t Kp        = RON_FLOAT_C(0.0);
        ron_float_t Ki        = RON_FLOAT_C(0.0);
        ron_float_t Kd        = RON_FLOAT_C(0.0);
        ron_float_t exp_kp;
        ron_float_t exp_ki;
        ron_float_t exp_kd;

        cfg.tuning_rule = rules[r];
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));
        TEST_ASSERT_TRUE(test_at_run_sine(&at, amp, RON_FLOAT_C(0.5), RON_FLOAT_C(0.001)));

        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_results(&at, &Ku, &Tu, &Kp, &Ki, &Kd));

        exp_kp = kp_f[r] * Ku;
        exp_ki = exp_kp / (ti_f[r] * Tu);
        exp_kd = exp_kp * (td_f[r] * Tu);

        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-3), exp_kp, Kp);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-3), exp_ki, Ki);
        TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-3), exp_kd, Kd);
    }
}

/* =========================================================================
 * RON-TC-AT-005 | RON-FR-804 — Gains applied only on explicit apply
 * ========================================================================= */
void test_ron_tc_at_005(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t amp       = (ron_float_t) (0.5 / AT_PI);
    ron_float_t u         = RON_FLOAT_C(0.0);

    pcfg.Kp = RON_FLOAT_C(1.0);
    pcfg.Ki = RON_FLOAT_C(0.5);
    pcfg.Kd = RON_FLOAT_C(0.1);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    /* apply before done is rejected */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_apply(&at, &pid));

    TEST_ASSERT_TRUE(test_at_run_sine(&at, amp, RON_FLOAT_C(0.5), RON_FLOAT_C(0.001)));

    /* Before apply: PID gains unchanged from original */
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(1.0), pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.5), pid.config.Ki);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(0.1), pid.config.Kd);

    /* apply guards */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_apply(NULL, &pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_apply(&at, NULL));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_apply(&at, &pid));

    /* After apply: PID gains equal the computed tuning gains, mode restored */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Kp_result, pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Ki_result, pid.config.Ki);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Kd_result, pid.config.Kd);
    TEST_ASSERT_EQUAL_INT(RON_MODE_AUTOMATIC, (int) pid.state.mode);

    /* Stepping after done holds the bias output */
    (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, cfg.u_bias, u);
}

/* =========================================================================
 * RON-TC-AT-006 | RON-FR-805 — Raw Ku / Tu exposed to caller
 * ========================================================================= */
void test_ron_tc_at_006(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t amp       = (ron_float_t) (0.5 / AT_PI);
    ron_float_t Ku        = RON_FLOAT_C(0.0);
    ron_float_t Tu        = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    /* results before done / NULL instance are rejected */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_autotune_results(&at, &Ku, &Tu, NULL, NULL, NULL));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_autotune_results(NULL, &Ku, &Tu, NULL, NULL, NULL));

    TEST_ASSERT_TRUE(test_at_run_sine(&at, amp, RON_FLOAT_C(0.5), RON_FLOAT_C(0.001)));

    /* Raw Ku/Tu match the stored state; NULL outputs are skipped */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_results(&at, &Ku, &Tu, NULL, NULL, NULL));
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Ku, Ku);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-4), at.state.Tu, Tu);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_autotune_results(&at, NULL, NULL, NULL, NULL, NULL));
}

/* =========================================================================
 * RON-TC-AT-007 | RON-FR-806 — Relay output within bias +/- d; step guards
 * ========================================================================= */
void test_ron_tc_at_007(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t u         = RON_FLOAT_C(0.0);
    int i;

    cfg.u_bias          = RON_FLOAT_C(2.0);
    cfg.relay_amplitude = RON_FLOAT_C(1.5);
    cfg.hysteresis      = RON_FLOAT_C(0.25);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));

    /* step before start (IDLE) is rejected */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));

    /* step guards */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_autotune_step(NULL, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NULL_POINTER,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), NULL));
    {
        ron_at_t un = {0};
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_CONFIG_INVALID,
            ron_autotune_step(&un, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u));
    }
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), &u));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), test_at_make_nan(), &u));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, test_at_make_nan(), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), test_at_make_inf(), RON_FLOAT_C(0.001), &u));
    /* -Inf exercises the value >= RON_FLOAT_MIN guard in at_isfinite */
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_autotune_step(&at, RON_FLOAT_C(0.0), test_at_make_neg_inf(), RON_FLOAT_C(0.001), &u));

    /* Sweep the setpoint across the hysteresis band; output stays bounded
     * and exercises the +d / -d / hold branches of the relay law. */
    for (i = -50; i <= 50; ++i) {
        ron_float_t y = (ron_float_t) i * RON_FLOAT_C(0.1);
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE, ron_autotune_step(&at, RON_FLOAT_C(0.0), y, RON_FLOAT_C(0.001), &u));
        TEST_ASSERT_TRUE(u >= (cfg.u_bias - cfg.relay_amplitude));
        TEST_ASSERT_TRUE(u <= (cfg.u_bias + cfg.relay_amplitude));
    }
}

/* =========================================================================
 * RON-TC-AT-008 | RON-FR-807 — Abort restores the PID; timeout aborts
 * ========================================================================= */
void test_ron_tc_at_008(void)
{
    ron_at_t at;
    ron_pid_instance_t pid;
    ron_pid_config_t pcfg = test_ron_make_pid_cfg();
    ron_at_config_t cfg   = test_at_default_cfg();
    ron_float_t u         = RON_FLOAT_C(0.0);
    unsigned k;

    pcfg.Kp = RON_FLOAT_C(7.0);
    pcfg.Ki = RON_FLOAT_C(3.0);
    pcfg.Kd = RON_FLOAT_C(1.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid, &pcfg));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at, &cfg));

    /* abort guards */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_abort(NULL, &pid));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_autotune_abort(&at, NULL));
    {
        ron_at_t un = {0};
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_autotune_abort(&un, &pid));
    }

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at, &pid));
    TEST_ASSERT_EQUAL_INT(RON_MODE_MANUAL, (int) pid.state.mode);

    /* Explicit abort restores gains and operating mode */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_abort(&at, &pid));
    TEST_ASSERT_TRUE(at.state.aborted);
    TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_AT_ABORTED, at.state.phase);
    TEST_ASSERT_EQUAL_INT(RON_MODE_AUTOMATIC, (int) pid.state.mode);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(7.0), pid.config.Kp);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(3.0), pid.config.Ki);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, RON_FLOAT_C(1.0), pid.config.Kd);

    /* Stepping after abort holds the bias output */
    (void) ron_autotune_step(&at, RON_FLOAT_C(0.0), RON_FLOAT_C(0.0), RON_FLOAT_C(0.001), &u);
    TEST_ASSERT_FLOAT_WITHIN(FLT_EPSILON, cfg.u_bias, u);

    /* Timeout abort: constant PV never crosses, run exceeds the budget */
    {
        ron_at_t at2;
        ron_pid_instance_t pid2;
        ron_at_config_t cfg2 = test_at_default_cfg();

        cfg2.timeout_s = RON_FLOAT_C(0.05);
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&pid2, &pcfg));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_init(&at2, &cfg2));
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_autotune_start(&at2, &pid2));

        for (k = 0U; k < 100U; ++k) {
            (void) ron_autotune_step(&at2, RON_FLOAT_C(0.0), RON_FLOAT_C(5.0), RON_FLOAT_C(0.01),
                                     &u);
            if (at2.state.aborted) {
                break;
            }
        }
        TEST_ASSERT_TRUE(at2.state.aborted);
        TEST_ASSERT_EQUAL_UINT8((uint8_t) RON_AT_ABORTED, at2.state.phase);
        TEST_ASSERT_FALSE(at2.state.done);
    }
}

/* =========================================================================
 * Entry point
 * ========================================================================= */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_at_001);
    RUN_TEST(test_ron_tc_at_002);
    RUN_TEST(test_ron_tc_at_003);
    RUN_TEST(test_ron_tc_at_003b);
    RUN_TEST(test_ron_tc_at_004);
    RUN_TEST(test_ron_tc_at_005);
    RUN_TEST(test_ron_tc_at_006);
    RUN_TEST(test_ron_tc_at_007);
    RUN_TEST(test_ron_tc_at_008);
    return UNITY_END();
}
