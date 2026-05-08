/*
 * @file     test_ron_trajectory.c
 * @brief    Trajectory generator unit tests.
 * @module   test_ron_trajectory
 * @doc      RON-TP-001
 * @req      RON-FR-500, RON-FR-501, RON-FR-502, RON-FR-503,
 *           RON-FR-510, RON-FR-511, RON-FR-512, RON-FR-513
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_trajectory.h"

#include "unity.h"

#define TEST_TRAJ_TOL RON_FLOAT_C(0.001)
#define TEST_TRAJ_DT RON_FLOAT_C(0.01)
#define TEST_TRAJ_MAX_STEPS 5000U

void setUp(void)
{
}

void tearDown(void)
{
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_traj_make_inf(void)
{
    volatile ron_float_t big = RON_FLOAT_MAX;

    return big * big;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_traj_make_neg_inf(void)
{
    return -test_traj_make_inf();
}

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static ron_float_t test_traj_make_nan(void)
{
    volatile ron_float_t zero = RON_FLOAT_C(0.0);

    return zero / zero;
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static ron_trap_config_t test_traj_trap_cfg(void)
{
    ron_trap_config_t cfg;

    cfg.v_max = RON_FLOAT_C(1.0);
    cfg.a_max = RON_FLOAT_C(2.0);
    return cfg;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_scurve_config_t test_traj_scurve_cfg(void)
{
    ron_scurve_config_t cfg;

    cfg.v_max = RON_FLOAT_C(1.5);
    cfg.a_max = RON_FLOAT_C(2.0);
    cfg.j_max = RON_FLOAT_C(8.0);
    return cfg;
}

/* Satisfies: RON-FR-502 | Test: RON-TC-TRAJ-003 */
static ron_float_t test_traj_abs(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? (-value) : value;
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void test_traj_run_trap_to_done(ron_trap_t *trap, ron_float_t *pos, ron_float_t *vel)
{
    ron_float_t acc = RON_FLOAT_C(0.0);
    bool finished   = false;
    uint16_t step   = 0U;

    while (!finished && (step < TEST_TRAJ_MAX_STEPS)) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_trap_step(trap, TEST_TRAJ_DT, pos, vel, &acc, &finished));
        TEST_ASSERT_TRUE(test_traj_abs(*vel) <= (trap->cfg.v_max + RON_FLOAT_C(0.0001)));
        TEST_ASSERT_TRUE(test_traj_abs(acc) <= (trap->cfg.a_max + RON_FLOAT_C(0.0001)));
        step++;
    }

    TEST_ASSERT_TRUE(finished);
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void test_traj_run_scurve_to_done(ron_scurve_t *scurve, ron_float_t *pos, ron_float_t *vel,
                                         ron_float_t *acc, ron_float_t *jrk)
{
    bool finished = false;
    uint16_t step = 0U;

    while (!finished && (step < TEST_TRAJ_MAX_STEPS)) {
        TEST_ASSERT_EQUAL_UINT8(
            RON_FAULT_NONE, ron_scurve_step(scurve, TEST_TRAJ_DT, pos, vel, acc, jrk, &finished));
        TEST_ASSERT_TRUE(test_traj_abs(*vel) <= (scurve->cfg.v_max + RON_FLOAT_C(0.0001)));
        TEST_ASSERT_TRUE(test_traj_abs(*acc) <= (scurve->cfg.a_max + RON_FLOAT_C(0.0001)));
        TEST_ASSERT_TRUE(test_traj_abs(*jrk) <= (scurve->cfg.j_max + RON_FLOAT_C(0.0001)));
        step++;
    }

    TEST_ASSERT_TRUE(finished);
}

/* RON-TC-TRAJ-001 | RON-FR-500 */
void test_ron_tc_traj_001(void)
{
    ron_trap_config_t cfg = test_traj_trap_cfg();
    ron_trap_t trap;
    ron_float_t pos = RON_FLOAT_C(0.0);
    ron_float_t vel = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));

    test_traj_run_trap_to_done(&trap, &pos, &vel);

    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(1.0), pos);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.0), vel);
    TEST_ASSERT_EQUAL_UINT8(RON_TRAP_PHASE_DONE, trap.state.phase);
}

/* RON-TC-TRAJ-002 | RON-FR-501 */
void test_ron_tc_traj_002(void)
{
    ron_trap_config_t cfg;
    ron_trap_t trap;
    ron_float_t pos      = RON_FLOAT_C(0.0);
    ron_float_t vel      = RON_FLOAT_C(0.0);
    ron_float_t acc      = RON_FLOAT_C(0.0);
    ron_float_t peak_vel = RON_FLOAT_C(0.0);
    bool finished        = false;
    uint16_t step        = 0U;

    cfg.v_max = RON_FLOAT_C(10.0);
    cfg.a_max = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(0.1)));

    while (!finished && (step < TEST_TRAJ_MAX_STEPS)) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
        if (test_traj_abs(vel) > peak_vel) {
            peak_vel = test_traj_abs(vel);
        }
        step++;
    }

    TEST_ASSERT_TRUE(finished);
    TEST_ASSERT_TRUE(peak_vel < cfg.v_max);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.1), pos);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.0), vel);
}

/* RON-TC-TRAJ-003 | RON-FR-502 */
void test_ron_tc_traj_003(void)
{
    ron_trap_config_t cfg = test_traj_trap_cfg();
    ron_trap_t trap;
    ron_float_t pos      = RON_FLOAT_C(0.0);
    ron_float_t prev_pos = RON_FLOAT_C(0.0);
    ron_float_t vel      = RON_FLOAT_C(0.0);
    ron_float_t acc      = RON_FLOAT_C(0.0);
    bool finished        = false;
    uint16_t step        = 0U;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));

    while (!finished && (step < 80U)) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
        TEST_ASSERT_TRUE(pos >= prev_pos);
        TEST_ASSERT_TRUE(RON_ISFINITE(pos));
        TEST_ASSERT_TRUE(RON_ISFINITE(vel));
        TEST_ASSERT_TRUE(RON_ISFINITE(acc));
        TEST_ASSERT_TRUE(test_traj_abs(vel) <= (cfg.v_max + RON_FLOAT_C(0.0001)));
        TEST_ASSERT_TRUE(test_traj_abs(acc) <= (cfg.a_max + RON_FLOAT_C(0.0001)));
        prev_pos = pos;
        step++;
    }
}

/* RON-TC-TRAJ-004 | RON-FR-503 */
void test_ron_tc_traj_004(void)
{
    ron_trap_config_t cfg = test_traj_trap_cfg();
    ron_trap_t trap;
    ron_float_t pos        = RON_FLOAT_C(0.0);
    ron_float_t vel        = RON_FLOAT_C(0.0);
    ron_float_t vel_before = RON_FLOAT_C(0.0);
    ron_float_t vel_after  = RON_FLOAT_C(0.0);
    ron_float_t acc        = RON_FLOAT_C(0.0);
    bool finished          = false;
    uint8_t step;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(2.0)));

    for (step = 0U; step < 40U; step++) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    }
    vel_before = vel;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel_after, &acc, &finished));
    TEST_ASSERT_TRUE(test_traj_abs(vel_after - vel_before) <=
                     ((cfg.a_max * TEST_TRAJ_DT) + RON_FLOAT_C(0.0001)));

    test_traj_run_trap_to_done(&trap, &pos, &vel);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(1.0), pos);
}

/* RON-TC-TRAJ-005 | RON-FR-510 */
void test_ron_tc_traj_005(void)
{
    ron_scurve_config_t cfg = test_traj_scurve_cfg();
    ron_scurve_t scurve;
    ron_float_t pos = RON_FLOAT_C(0.0);
    ron_float_t vel = RON_FLOAT_C(0.0);
    ron_float_t acc = RON_FLOAT_C(0.0);
    ron_float_t jrk = RON_FLOAT_C(0.0);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_init(&scurve, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));

    test_traj_run_scurve_to_done(&scurve, &pos, &vel, &acc, &jrk);

    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(1.0), pos);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.0), vel);
    TEST_ASSERT_EQUAL_UINT8(RON_SCURVE_PHASE_DONE, scurve.state.phase);
}

/* RON-TC-TRAJ-006 | RON-FR-511 */
void test_ron_tc_traj_006(void)
{
    ron_scurve_config_t cfg = test_traj_scurve_cfg();
    ron_scurve_t scurve;
    ron_float_t pos = RON_FLOAT_C(0.0);
    ron_float_t vel = RON_FLOAT_C(0.0);
    ron_float_t acc = RON_FLOAT_C(0.0);
    ron_float_t jrk = RON_FLOAT_C(0.0);
    bool finished   = false;
    bool saw_pos    = false;
    bool saw_neg    = false;
    uint16_t step   = 0U;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_init(&scurve, &cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));

    while (!finished && (step < TEST_TRAJ_MAX_STEPS)) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos, &vel,
                                                                &acc, &jrk, &finished));
        if (jrk > RON_FLOAT_C(0.0)) {
            saw_pos = true;
        }
        if (jrk < RON_FLOAT_C(0.0)) {
            saw_neg = true;
        }
        TEST_ASSERT_TRUE(test_traj_abs(jrk) <= (cfg.j_max + RON_FLOAT_C(0.0001)));
        step++;
    }

    TEST_ASSERT_TRUE(saw_pos);
    TEST_ASSERT_TRUE(saw_neg);
}

/* RON-TC-TRAJ-007 | RON-FR-512 */
void test_ron_tc_traj_007(void)
{
    ron_trap_config_t trap_cfg     = test_traj_trap_cfg();
    ron_scurve_config_t scurve_cfg = test_traj_scurve_cfg();
    ron_trap_t trap;
    ron_scurve_t scurve;
    ron_float_t pos = RON_FLOAT_C(0.0);
    ron_float_t vel = RON_FLOAT_C(0.0);
    ron_float_t acc = RON_FLOAT_C(0.0);
    ron_float_t jrk = RON_FLOAT_C(0.0);
    bool finished   = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    TEST_ASSERT_TRUE(finished);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos, &vel, &acc, &jrk, &finished));
    TEST_ASSERT_TRUE(finished);
}

/* RON-TC-TRAJ-008 | RON-FR-513 */
void test_ron_tc_traj_008(void)
{
    ron_trap_config_t trap_cfg     = test_traj_trap_cfg();
    ron_scurve_config_t scurve_cfg = test_traj_scurve_cfg();
    ron_trap_t trap;
    ron_scurve_t scurve;
    ron_float_t pos0 = RON_FLOAT_C(0.0);
    ron_float_t pos1 = RON_FLOAT_C(0.0);
    ron_float_t vel0 = RON_FLOAT_C(0.0);
    ron_float_t vel1 = RON_FLOAT_C(0.0);
    ron_float_t acc0 = RON_FLOAT_C(0.0);
    ron_float_t acc1 = RON_FLOAT_C(0.0);
    ron_float_t jrk0 = RON_FLOAT_C(0.0);
    ron_float_t jrk1 = RON_FLOAT_C(0.0);
    bool finished    = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos0, &vel0, &acc0, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_hold(&trap, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos1, &vel1, &acc1, &finished));
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, pos0, pos1);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, vel0, vel1);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, acc0, acc1);
    TEST_ASSERT_EQUAL_UINT8(RON_TRAP_PHASE_HOLD, trap.state.phase);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_hold(&trap, false));
    test_traj_run_trap_to_done(&trap, &pos1, &vel1);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos0, &vel0,
                                                            &acc0, &jrk0, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_hold(&scurve, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos1, &vel1,
                                                            &acc1, &jrk1, &finished));
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, pos0, pos1);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, vel0, vel1);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, acc0, acc1);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, jrk0, jrk1);
    TEST_ASSERT_TRUE(scurve.state.hold);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_hold(&scurve, false));
    test_traj_run_scurve_to_done(&scurve, &pos1, &vel1, &acc1, &jrk1);
}

/* RON-TC-TRAJ-003, RON-TC-TRAJ-004, RON-TC-TRAJ-005 | RON-FR-502, RON-FR-503, RON-FR-510 */
void test_ron_tc_traj_reverse_and_short_edges(void)
{
    ron_trap_config_t trap_cfg     = test_traj_trap_cfg();
    ron_scurve_config_t scurve_cfg = test_traj_scurve_cfg();
    ron_trap_t trap;
    ron_scurve_t scurve;
    ron_float_t pos        = RON_FLOAT_C(0.0);
    ron_float_t vel        = RON_FLOAT_C(0.0);
    ron_float_t acc        = RON_FLOAT_C(0.0);
    ron_float_t jrk        = RON_FLOAT_C(0.0);
    ron_float_t vel_before = RON_FLOAT_C(0.0);
    ron_float_t vel_after  = RON_FLOAT_C(0.0);
    bool finished          = false;
    uint8_t step;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(-0.25)));
    test_traj_run_trap_to_done(&trap, &pos, &vel);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(-0.25), pos);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.0), vel);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(2.0)));
    for (step = 0U; step < 25U; step++) {
        TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                                ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    }
    vel_before = vel;
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(-0.5)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel_after, &acc, &finished));
    TEST_ASSERT_TRUE(test_traj_abs(vel_after - vel_before) <=
                     ((trap_cfg.a_max * TEST_TRAJ_DT) + RON_FLOAT_C(0.0001)));
    test_traj_run_trap_to_done(&trap, &pos, &vel);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(-0.5), pos);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(0.01)));
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.0),
                             scurve.state.phase_time[RON_SCURVE_PHASE_CONST_VEL]);
    test_traj_run_scurve_to_done(&scurve, &pos, &vel, &acc, &jrk);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(0.01), pos);

    scurve_cfg.v_max = RON_FLOAT_C(8.0);
    scurve_cfg.a_max = RON_FLOAT_C(16.0);
    scurve_cfg.j_max = RON_FLOAT_C(2.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(2.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(-38.0)));
    TEST_ASSERT_TRUE(scurve.state.phase_time[RON_SCURVE_PHASE_CONST_VEL] > RON_FLOAT_C(0.0));
    test_traj_run_scurve_to_done(&scurve, &pos, &vel, &acc, &jrk);
    TEST_ASSERT_FLOAT_WITHIN(TEST_TRAJ_TOL, RON_FLOAT_C(-38.0), pos);
}

/* RON-TC-TRAJ-001, RON-TC-TRAJ-005, RON-TC-TRAJ-007 | RON-FR-500, RON-FR-510, RON-FR-512 */
void test_ron_tc_traj_defensive_branch_paths(void)
{
    ron_trap_config_t trap_cfg     = test_traj_trap_cfg();
    ron_scurve_config_t scurve_cfg = test_traj_scurve_cfg();
    ron_trap_t trap                = {0};
    ron_scurve_t scurve            = {0};
    ron_float_t pos                = RON_FLOAT_C(0.0);
    ron_float_t vel                = RON_FLOAT_C(0.0);
    ron_float_t acc                = RON_FLOAT_C(0.0);
    ron_float_t jrk                = RON_FLOAT_C(0.0);
    bool finished                  = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_trap_set_target(NULL, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_trap_hold(NULL, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_trap_hold(&trap, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(NULL, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(&trap, TEST_TRAJ_DT, NULL, &vel, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, NULL, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, NULL, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, NULL));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_set_target(&trap, RON_FLOAT_C(0.5)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_trap_step(&trap, test_traj_make_inf(),
                                                                    &pos, &vel, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));

    trap_cfg = test_traj_trap_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_set_target(&trap, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));

    trap_cfg       = test_traj_trap_cfg();
    trap_cfg.v_max = test_traj_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap_cfg       = test_traj_trap_cfg();
    trap_cfg.a_max = test_traj_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap_cfg       = test_traj_trap_cfg();
    trap_cfg.a_max = test_traj_make_neg_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap_cfg       = test_traj_trap_cfg();
    trap_cfg.v_max = test_traj_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap_cfg       = test_traj_trap_cfg();
    trap_cfg.a_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));

    trap_cfg = test_traj_trap_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_hold(&trap, false));
    trap.state.target    = RON_FLOAT_C(0.0000005);
    trap.state.direction = RON_FLOAT_C(1.0);
    trap.state.v_peak    = RON_FLOAT_C(1.0);
    trap.state.phase     = RON_TRAP_PHASE_ACCEL;
    trap.state.finished  = false;
    trap.state.vel       = RON_FLOAT_C(0.001);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    TEST_ASSERT_TRUE(finished);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap.state.target    = RON_FLOAT_C(0.0);
    trap.state.direction = RON_FLOAT_C(1.0);
    trap.state.v_peak    = RON_FLOAT_C(1.0);
    trap.state.phase     = RON_TRAP_PHASE_ACCEL;
    trap.state.finished  = false;
    trap.state.vel       = RON_FLOAT_C(-0.001);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    TEST_ASSERT_TRUE(finished);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.001)));
    trap.state.target    = RON_FLOAT_C(0.0);
    trap.state.direction = RON_FLOAT_C(1.0);
    trap.state.v_peak    = RON_FLOAT_C(1.0);
    trap.state.phase     = RON_TRAP_PHASE_ACCEL;
    trap.state.finished  = false;
    trap.state.vel       = RON_FLOAT_C(-0.1);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    TEST_ASSERT_TRUE(!finished);

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_set_target(NULL, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_hold(NULL, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_scurve_hold(&scurve, true));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_scurve_step(NULL, TEST_TRAJ_DT, &pos, &vel, &acc, &jrk, &finished));

    scurve_cfg = test_traj_scurve_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_step(&scurve, TEST_TRAJ_DT, NULL,
                                                                    &vel, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                    NULL, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                    &vel, NULL, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                    &vel, &acc, NULL, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos, &vel, &acc, &jrk, NULL));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(0.5)));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_scurve_step(&scurve, test_traj_make_inf(), &pos, &vel, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                      &vel, &acc, &jrk, &finished));

    scurve_cfg = test_traj_scurve_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_set_target(&scurve, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                      &vel, &acc, &jrk, &finished));

    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.v_max = test_traj_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.a_max = test_traj_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.j_max = test_traj_make_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.j_max = test_traj_make_neg_inf();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.v_max = test_traj_make_nan();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.v_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg       = test_traj_scurve_cfg();
    scurve_cfg.a_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));

    scurve_cfg = test_traj_scurve_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));
    scurve.state.phase                                   = RON_SCURVE_PHASE_JERK_POS_2;
    scurve.state.phase_time[RON_SCURVE_PHASE_JERK_POS_2] = RON_FLOAT_C(0.0);
    scurve.state.finished                                = false;
    scurve.state.hold                                    = false;
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_NONE, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos, &vel, &acc, &jrk, &finished));
    TEST_ASSERT_TRUE(finished);
}

/* RON-TC-TRAJ-001, RON-TC-TRAJ-005 | RON-FR-500, RON-FR-510 */
void test_ron_tc_traj_validation_paths(void)
{
    ron_trap_config_t trap_cfg     = test_traj_trap_cfg();
    ron_scurve_config_t scurve_cfg = test_traj_scurve_cfg();
    ron_trap_t trap                = {0};
    ron_scurve_t scurve            = {0};
    ron_float_t pos                = RON_FLOAT_C(0.0);
    ron_float_t vel                = RON_FLOAT_C(0.0);
    ron_float_t acc                = RON_FLOAT_C(0.0);
    ron_float_t jrk                = RON_FLOAT_C(0.0);
    bool finished                  = false;

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_init(NULL, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_trap_init(&trap, NULL, RON_FLOAT_C(0.0)));
    trap_cfg.v_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    trap_cfg = test_traj_trap_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_init(&trap, &trap_cfg, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_trap_set_target(&trap, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_step(&trap, TEST_TRAJ_DT, &pos, &vel, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE, ron_trap_init(&trap, &trap_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_trap_step(&trap, TEST_TRAJ_DT, NULL, &vel, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_step(&trap, RON_FLOAT_C(0.0), &pos, &vel, &acc, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_trap_set_target(&trap, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_trap_hold(&trap, true));

    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_scurve_init(NULL, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER,
                            ron_scurve_init(&scurve, NULL, RON_FLOAT_C(0.0)));
    scurve_cfg.j_max = RON_FLOAT_C(0.0);
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    scurve_cfg = test_traj_scurve_cfg();
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_init(&scurve, &scurve_cfg, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_set_target(&scurve, RON_FLOAT_C(1.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                      &vel, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NONE,
                            ron_scurve_init(&scurve, &scurve_cfg, RON_FLOAT_C(0.0)));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_NULL_POINTER, ron_scurve_step(&scurve, TEST_TRAJ_DT, &pos,
                                                                    NULL, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(
        RON_FAULT_CONFIG_INVALID,
        ron_scurve_step(&scurve, RON_FLOAT_C(0.0), &pos, &vel, &acc, &jrk, &finished));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID,
                            ron_scurve_set_target(&scurve, test_traj_make_inf()));
    TEST_ASSERT_EQUAL_UINT8(RON_FAULT_CONFIG_INVALID, ron_scurve_hold(&scurve, true));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ron_tc_traj_001);
    RUN_TEST(test_ron_tc_traj_002);
    RUN_TEST(test_ron_tc_traj_003);
    RUN_TEST(test_ron_tc_traj_004);
    RUN_TEST(test_ron_tc_traj_005);
    RUN_TEST(test_ron_tc_traj_006);
    RUN_TEST(test_ron_tc_traj_007);
    RUN_TEST(test_ron_tc_traj_008);
    RUN_TEST(test_ron_tc_traj_reverse_and_short_edges);
    RUN_TEST(test_ron_tc_traj_defensive_branch_paths);
    RUN_TEST(test_ron_tc_traj_validation_paths);
    return UNITY_END();
}
