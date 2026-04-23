/*
 * @file     test_ron_pid_common.h
 * @brief    Shared test helpers for the Regulon PID unit suites.
 * @module   test_ron_pid_common
 * @doc      RON-TP-001
 * @req      RON-FR-001, RON-FR-050, RON-SR-001
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef TEST_RON_PID_COMMON_H
#define TEST_RON_PID_COMMON_H

#include "ron/ron_pid.h"

/* Satisfies: RON-FR-050 | Test: RON-TC-PID-030 */
static inline ron_pid_config_t test_ron_make_pid_cfg(void)
{
    ron_pid_config_t cfg;

    cfg.Kp                 = RON_FLOAT_C(1.0);
    cfg.Ki                 = RON_FLOAT_C(0.0);
    cfg.Kd                 = RON_FLOAT_C(0.0);
    cfg.N                  = RON_FLOAT_C(0.0);
    cfg.b                  = RON_FLOAT_C(1.0);
    cfg.c                  = RON_FLOAT_C(1.0);
    cfg.u_min              = RON_FLOAT_C(-1000.0);
    cfg.u_max              = RON_FLOAT_C(1000.0);
    cfg.du_max             = RON_FLOAT_C(0.0);
    cfg.I_min              = RON_FLOAT_C(-1000.0);
    cfg.I_max              = RON_FLOAT_C(1000.0);
    cfg.aw_mode            = RON_AW_NONE;
    cfg.T_aw               = RON_FLOAT_C(0.1);
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

/* Satisfies: RON-FR-001 | Test: RON-TC-PID-001 */
static inline ron_float_t test_ron_abs(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? (-value) : value;
}

#endif /* TEST_RON_PID_COMMON_H */
