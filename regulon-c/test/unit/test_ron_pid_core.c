/*
 * @file     test_ron_pid_core.c
 * @brief    Sprint 2 unit tests — PID core computation pipeline.
 * @module   test_ron_pid_core
 * @doc      RON-IS-001
 * @req      RON-FR-001 – RON-FR-013
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Test cases implemented here:
 *   RON-TC-PID-001  Parallel PID form output correctness       | RON-FR-001
 *   RON-TC-PID-002  ISA form ↔ parallel form equivalence       | RON-FR-002
 *   RON-TC-PID-003  Proportional term                          | RON-FR-003
 *   RON-TC-PID-004  Euler integration                          | RON-FR-004
 *   RON-TC-PID-005  Trapezoidal integration                    | RON-FR-004
 *   RON-TC-PID-006  Derivative on Error (kick on SP step)      | RON-FR-005
 *   RON-TC-PID-007  Derivative on Measurement (no kick)        | RON-FR-005
 *   RON-TC-PID-008  Derivative filter attenuates D term        | RON-FR-006
 *   RON-TC-PID-009  N=0 disables derivative filter             | RON-FR-006
 *   RON-TC-PID-010  2DOF setpoint weights (b, c)               | RON-FR-007
 *   RON-TC-PID-011  Input normalisation                        | RON-FR-010
 *   RON-TC-PID-012  Scaling formula                            | RON-FR-011
 *   RON-TC-PID-013  Output denormalisation                     | RON-FR-012
 *   RON-TC-PID-014  normalise=false → raw domain computation   | RON-FR-013
 */

#include "unity.h"
#include "ron/ron_pid.h"

/* Numerical tolerance — wider than FLT_EPSILON to accommodate the full
 * pipeline's accumulated rounding error on float builds.                */
#if defined(RON_USE_DOUBLE) && (RON_USE_DOUBLE == 1)
#  define TOL  RON_FLOAT_C(1.0e-9)
#else
#  define TOL  RON_FLOAT_C(1.0e-4)
#endif

/* =========================================================================
 * Default configuration helper — a vanilla, validation-clean config that
 * individual tests override selectively.
 * ========================================================================= */

static ron_pid_config_t make_default_cfg(void)
{
    ron_pid_config_t cfg = {
        .Kp = RON_FLOAT_C(0.0),
        .Ki = RON_FLOAT_C(0.0),
        .Kd = RON_FLOAT_C(0.0),
        .N  = RON_FLOAT_C(0.0),
        .b  = RON_FLOAT_C(1.0),
        .c  = RON_FLOAT_C(1.0),
        .u_min  = RON_FLOAT_C(-1000.0),
        .u_max  = RON_FLOAT_C( 1000.0),
        .du_max = RON_FLOAT_C(0.0),
        .I_min  = RON_FLOAT_C(-1000.0),
        .I_max  = RON_FLOAT_C( 1000.0),
        .aw_mode      = RON_AW_NONE,
        .T_aw         = RON_FLOAT_C(0.1),
        .integ_method = RON_INTEG_EULER,
        .deriv_mode   = RON_DERIV_ON_MEASUREMENT,
        .tau_sp       = RON_FLOAT_C(0.0),
        .normalise    = false,
        .in_min  = RON_FLOAT_C(0.0),
        .in_max  = RON_FLOAT_C(1.0),
        .out_min = RON_FLOAT_C(0.0),
        .out_max = RON_FLOAT_C(1.0),
        .safe_policy = RON_SAFE_HOLD_LAST,
        .safe_value  = RON_FLOAT_C(0.0),
        .I_overflow_thresh  = RON_FLOAT_C(0.0),
        .sp_reset_threshold = RON_FLOAT_C(0.0),
        .fault_cb = NULL
    };
    return cfg;
}

void setUp(void)    { }
void tearDown(void) { }

/* =========================================================================
 * RON-TC-PID-001 — Parallel PID form output correctness.
 *
 * Pre: Kp=2, Ki=0, Kd=0, no saturation, no AW.
 * Stim: r=1, y=0, dt=0.01.
 * Expected: u = Kp * e = 2.0.
 * ========================================================================= */

/* RON-TC-PID-001 | RON-FR-001 */
void test_ron_tc_pid_001_parallel_form(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, &cfg));

    ron_float_t  u      = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_fault_t  fault  = ron_pid_step(&inst,
                                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, fault);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(2.0), u);
}

/* =========================================================================
 * RON-TC-PID-002 — ISA form ↔ parallel form numerical equivalence.
 *
 * Given Kp=2, Ti=0.5 (→ Ki=Kp/Ti=4), Td=0.25 (→ Kd=Kp*Td=0.5): running the
 * same input through the parallel form with the ISA-derived gains must
 * produce the SAME output.  We verify by checking the parallel-form
 * output against a hand-computed expected value based on the SRS formula.
 * ========================================================================= */

/* RON-TC-PID-002 | RON-FR-002 */
void test_ron_tc_pid_002_isa_equivalence(void)
{
    const ron_float_t Kp  = RON_FLOAT_C(2.0);
    const ron_float_t Ti  = RON_FLOAT_C(0.5);
    const ron_float_t Td  = RON_FLOAT_C(0.25);

    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = Kp;
    cfg.Ki = Kp / Ti;             /* ISA → parallel conversion. */
    cfg.Kd = Kp * Td;
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    cfg.N  = RON_FLOAT_C(0.0);    /* disable filter for analytic check. */
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_pid_init(&inst, &cfg));

    ron_float_t  u      = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;
    ron_float_t  r      = RON_FLOAT_C(1.0);
    ron_float_t  y      = RON_FLOAT_C(0.0);
    ron_float_t  dt     = RON_FLOAT_C(0.1);
    (void)ron_pid_step(&inst, r, y, dt, &u, &status);

    /* Expected: u = Kp*e + Ki*dt*e + Kd*(e - e_prev)/dt
     *             = 2*1  + 4*0.1*1 + 0.5*(1-0)/0.1 = 2 + 0.4 + 5 = 7.4 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(7.4), u);
}

/* =========================================================================
 * RON-TC-PID-003 — Proportional term alone.
 * ========================================================================= */

/* RON-TC-PID-003 | RON-FR-003 */
void test_ron_tc_pid_003_proportional(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(3.0);
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(2.0), RON_FLOAT_C(0.5),
                       RON_FLOAT_C(0.01), &u, &status);
    /* u = Kp * (r - y) = 3 * 1.5 = 4.5 */
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(4.5), u);
}

/* =========================================================================
 * RON-TC-PID-004 — Euler integration.
 *
 * With Kp=0, Kd=0, Ki=1, dt=0.01, r=1, y=0 constant, after N steps the
 * integral should equal N * Ki * dt * e = N * 0.01.
 * ========================================================================= */

/* RON-TC-PID-004 | RON-FR-004 */
void test_ron_tc_pid_004_euler_integration(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.integ_method = RON_INTEG_EULER;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status;
    for (int k = 0; k < 100; ++k)
    {
        (void)ron_pid_step(&inst,
                           RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                           RON_FLOAT_C(0.01), &u, &status);
    }
    /* 100 steps · Ki · dt · e = 100 · 1 · 0.01 · 1 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-3), RON_FLOAT_C(1.0), u);
}

/* =========================================================================
 * RON-TC-PID-005 — Trapezoidal integration.
 *
 * Step 1: e=1, e_prev=0. I_update = Ki · dt · 0.5 · (1+0) = 0.5.
 * Step 2: e=2, e_prev=1. I_update = Ki · dt · 0.5 · (2+1) = 1.5.  I_new = 2.0.
 * ========================================================================= */

/* RON-TC-PID-005 | RON-FR-004 */
void test_ron_tc_pid_005_trapezoidal_integration(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Ki = RON_FLOAT_C(1.0);
    cfg.integ_method = RON_INTEG_TRAPEZOIDAL;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status;

    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(1.0), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(0.5), u);

    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(2.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(1.0), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(2.0), u);
}

/* =========================================================================
 * RON-TC-PID-006 — Derivative on Error (DoE) produces a kick on SP step.
 * ========================================================================= */

/* RON-TC-PID-006 | RON-FR-005 */
void test_ron_tc_pid_006_doe_kick(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kd = RON_FLOAT_C(1.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status;

    /* SP step: r goes 0 → 1 while y stays at 0. e goes 0 → 1. */
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    /* u = Kd * (e - e_prev) / dt = 1 * 1 / 0.01 = 100 */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-2), RON_FLOAT_C(100.0), u);
}

/* =========================================================================
 * RON-TC-PID-007 — Derivative on Measurement (DoM) has NO kick on SP step.
 * ========================================================================= */

/* RON-TC-PID-007 | RON-FR-005 */
void test_ron_tc_pid_007_dom_no_kick(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kd = RON_FLOAT_C(1.0);
    cfg.deriv_mode = RON_DERIV_ON_MEASUREMENT;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u = RON_FLOAT_C(0.0);
    ron_status_t status;

    /* SP step but y stays at 0 → delta(y)=0 → no derivative kick. */
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(0.0), u);
}

/* =========================================================================
 * RON-TC-PID-008 — Derivative low-pass filter attenuates the D term.
 *
 * With N=10, dt=0.01 → Ndt=0.1.  D_f = 0.1/1.1 · D_raw ≈ 0.0909 · D_raw.
 * Compared to the raw 100 (PID-006), filtered result ≈ 9.09.
 * ========================================================================= */

/* RON-TC-PID-008 | RON-FR-006 */
void test_ron_tc_pid_008_d_filter_enabled(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kd = RON_FLOAT_C(1.0);
    cfg.N  = RON_FLOAT_C(10.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    /* Expected: (0.1/1.1)·100 + (1/1.1)·0  ≈ 9.0909… */
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-2),
                             RON_FLOAT_C(9.0909090909), u);
}

/* =========================================================================
 * RON-TC-PID-009 — N=0 disables the derivative filter (D_f == D_raw).
 * ========================================================================= */

/* RON-TC-PID-009 | RON-FR-006 */
void test_ron_tc_pid_009_d_filter_disabled(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kd = RON_FLOAT_C(1.0);
    cfg.N  = RON_FLOAT_C(0.0);
    cfg.deriv_mode = RON_DERIV_ON_ERROR;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-2), RON_FLOAT_C(100.0), u);
}

/* =========================================================================
 * RON-TC-PID-010 — 2DOF setpoint weights modify e_P and e_D only.
 * ========================================================================= */

/* RON-TC-PID-010 | RON-FR-007 */
void test_ron_tc_pid_010_2dof_weights(void)
{
    /* b=0 suppresses the proportional response to setpoint. */
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(5.0);
    cfg.b  = RON_FLOAT_C(0.0);
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    /* e_P = b*r - y = 0 → P = 0, so u = 0 (no I, no D). */
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(0.0), u);

    /* Baseline b=1 gives full proportional response. */
    ron_pid_instance_t inst2;
    cfg.b = RON_FLOAT_C(1.0);
    (void)ron_pid_init(&inst2, &cfg);
    (void)ron_pid_step(&inst2,
                       RON_FLOAT_C(1.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(5.0), u);
}

/* =========================================================================
 * RON-TC-PID-011 — Input normalisation maps [in_min, in_max] → [0, 1].
 *
 * With in_min=0, in_max=10, Kp=1, out_min=0, out_max=1 (identity denorm
 * for values in [0,1]): r_eng=5, y_eng=0 → r_n=0.5, y_n=0 → u_norm=0.5,
 * u_eng=0.5.
 * ========================================================================= */

/* RON-TC-PID-011 | RON-FR-010 */
void test_ron_tc_pid_011_input_normalisation(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(1.0);
    cfg.normalise = true;
    cfg.in_min    = RON_FLOAT_C(0.0);
    cfg.in_max    = RON_FLOAT_C(10.0);
    cfg.out_min   = RON_FLOAT_C(0.0);
    cfg.out_max   = RON_FLOAT_C(1.0);
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(5.0), RON_FLOAT_C(0.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(0.5), u);
    TEST_ASSERT_TRUE((status & (ron_status_t)RON_STATUS_NORMALISED) != 0U);
}

/* =========================================================================
 * RON-TC-PID-012 — Scaling formula: x_norm = (x - x_min)/(x_max - x_min).
 *
 * Verified indirectly through the pipeline: in_min=-10, in_max=10 → 0
 * maps to 0.5.  Kp=2 gives u_norm = 2·(0.5 - 0.25) = 0.5.
 * ========================================================================= */

/* RON-TC-PID-012 | RON-FR-011 */
void test_ron_tc_pid_012_scaling_formula(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.normalise = true;
    cfg.in_min    = RON_FLOAT_C(-10.0);
    cfg.in_max    = RON_FLOAT_C( 10.0);
    cfg.out_min   = RON_FLOAT_C(0.0);
    cfg.out_max   = RON_FLOAT_C(1.0);
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    /* r=0 → 0.5,  y=-5 → 0.25.  e=0.25.  u_norm = 0.5.  u_eng = 0.5. */
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(0.5), u);
}

/* =========================================================================
 * RON-TC-PID-013 — Output denormalisation inverts scaling.
 *
 * Same inputs as PID-012 but with out_min=-100, out_max=100.  u_norm=0.5
 * → u_eng = 0.5·200 + (-100) = 0.
 * ========================================================================= */

/* RON-TC-PID-013 | RON-FR-012 */
void test_ron_tc_pid_013_output_denormalisation(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.normalise = true;
    cfg.in_min  = RON_FLOAT_C(-10.0);
    cfg.in_max  = RON_FLOAT_C( 10.0);
    cfg.out_min = RON_FLOAT_C(-100.0);
    cfg.out_max = RON_FLOAT_C( 100.0);
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(RON_FLOAT_C(1.0e-3), RON_FLOAT_C(0.0), u);
}

/* =========================================================================
 * RON-TC-PID-014 — normalise=false → computation in raw engineering units.
 *
 * Same numeric scenario as PID-013 but with normalise=false: u = Kp · e =
 * 2 · (0 - (-5)) = 10.  Bypasses both normalise and denormalise stages.
 * ========================================================================= */

/* RON-TC-PID-014 | RON-FR-013 */
void test_ron_tc_pid_014_normalise_disabled(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t   cfg = make_default_cfg();
    cfg.Kp = RON_FLOAT_C(2.0);
    cfg.normalise = false;
    (void)ron_pid_init(&inst, &cfg);

    ron_float_t  u;
    ron_status_t status;
    (void)ron_pid_step(&inst,
                       RON_FLOAT_C(0.0), RON_FLOAT_C(-5.0),
                       RON_FLOAT_C(0.01), &u, &status);
    TEST_ASSERT_FLOAT_WITHIN(TOL, RON_FLOAT_C(10.0), u);
    TEST_ASSERT_FALSE((status & (ron_status_t)RON_STATUS_NORMALISED) != 0U);
}

/* =========================================================================
 * Unity test runner
 * ========================================================================= */

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_pid_001_parallel_form);
    RUN_TEST(test_ron_tc_pid_002_isa_equivalence);
    RUN_TEST(test_ron_tc_pid_003_proportional);
    RUN_TEST(test_ron_tc_pid_004_euler_integration);
    RUN_TEST(test_ron_tc_pid_005_trapezoidal_integration);
    RUN_TEST(test_ron_tc_pid_006_doe_kick);
    RUN_TEST(test_ron_tc_pid_007_dom_no_kick);
    RUN_TEST(test_ron_tc_pid_008_d_filter_enabled);
    RUN_TEST(test_ron_tc_pid_009_d_filter_disabled);
    RUN_TEST(test_ron_tc_pid_010_2dof_weights);
    RUN_TEST(test_ron_tc_pid_011_input_normalisation);
    RUN_TEST(test_ron_tc_pid_012_scaling_formula);
    RUN_TEST(test_ron_tc_pid_013_output_denormalisation);
    RUN_TEST(test_ron_tc_pid_014_normalise_disabled);
    return UNITY_END();
}
