/*
 * @file     ron_cascade.c
 * @brief    Cascade (master/slave) PID controller implementation.
 * @module   ron_cascade
 * @doc      RON-IS-001
 * @req      RON-FR-400, RON-FR-401, RON-FR-402, RON-FR-403,
 *           RON-FR-404, RON-FR-405, RON-FR-406
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_cascade.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Satisfies: RON-SR-020 */
static bool cascade_isfinite(ron_float_t value)
{
    return (value == value) && (value <= RON_FLOAT_MAX) && (value >= RON_FLOAT_MIN);
}

/* Satisfies: RON-SR-006 */
static ron_fault_t cascade_check_inst(const ron_cascade_instance_t *casc)
{
    if (casc == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!casc->outer.state.is_initialised || !casc->inner.state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-406 */
static ron_cascade_status_t cascade_build_status(ron_status_t outer_s, ron_status_t inner_s)
{
    return (
        ron_cascade_status_t) (((ron_cascade_status_t) outer_s << RON_CASCADE_STATUS_OUTER_SHIFT) |
                               ((ron_cascade_status_t) inner_s << RON_CASCADE_STATUS_INNER_SHIFT));
}

/*
 * Back-calculation anti-windup propagation from inner to outer loop.
 *
 * When the inner loop saturates, the outer loop's integrator is corrected by:
 *   ΔI = (dt / T_aw_outer) * (u_inner_sat - u_outer_sat)
 *
 * u_outer_sat is the clamped outer output that was passed as r_inner.
 * u_inner_sat is the inner loop's saturated output.
 * The difference is the tracking error at the inner boundary; a one-sample-lag
 * correction is inherent and acceptable in discrete cascade control.
 *
 * Satisfies: RON-FR-403.
 */
/* Satisfies: RON-FR-403 | Test: RON-TC-CASC-006 */
static void cascade_apply_cross_aw(ron_cascade_instance_t *casc, ron_float_t u_outer,
                                   ron_float_t u_inner, ron_status_t inner_status, ron_float_t dt)
{
    ron_float_t outer_integral;
    ron_float_t delta_I;

    if ((inner_status & RON_STATUS_SATURATED) == (ron_status_t) 0U) {
        return;
    }
    if (casc->outer.config.aw_mode != RON_AW_BACK_CALC) {
        return;
    }

    (void) ron_pid_get_state(&casc->outer, &outer_integral, NULL, NULL, NULL, NULL);
    delta_I = (dt / casc->outer.config.T_aw) * (u_inner - u_outer);
    (void) ron_pid_set_integral(&casc->outer, outer_integral + delta_I);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/* Satisfies: RON-FR-400, RON-FR-402, RON-FR-405 | Test: RON-TC-CASC-001, RON-TC-CASC-002 */
ron_fault_t ron_cascade_init(ron_cascade_instance_t *casc, const ron_pid_config_t *outer_cfg,
                             const ron_pid_config_t *inner_cfg)
{
    ron_fault_t fault;

    if ((casc == NULL) || (outer_cfg == NULL) || (inner_cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = ron_pid_init(&casc->outer, outer_cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    fault = ron_pid_init(&casc->inner, inner_cfg);
    return fault;
}

/* Satisfies: RON-FR-401 – RON-FR-403, RON-FR-406 | Test: RON-TC-CASC-003 – RON-TC-CASC-009 */
ron_fault_t ron_cascade_step(ron_cascade_instance_t *casc, ron_float_t r_out, ron_float_t y_out,
                             ron_float_t y_in, ron_float_t dt, ron_float_t *u_out,
                             ron_cascade_status_t *status)
{
    ron_fault_t outer_fault;
    ron_fault_t inner_fault;
    ron_float_t u_outer;
    ron_float_t u_inner;
    ron_status_t outer_status;
    ron_status_t inner_status;

    if ((casc == NULL) || (u_out == NULL) || (status == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!casc->outer.state.is_initialised || !casc->inner.state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !cascade_isfinite(dt)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    outer_fault = ron_pid_step(&casc->outer, r_out, y_out, dt, &u_outer, &outer_status);
    inner_fault = ron_pid_step(&casc->inner, u_outer, y_in, dt, &u_inner, &inner_status);

    cascade_apply_cross_aw(casc, u_outer, u_inner, inner_status, dt);

    *u_out  = u_inner;
    *status = cascade_build_status(outer_status, inner_status);

    return (ron_fault_t) (outer_fault | inner_fault);
}

/* Satisfies: RON-FR-404 | Test: RON-TC-CASC-007, RON-TC-CASC-008 */
ron_fault_t ron_cascade_set_mode(ron_cascade_instance_t *casc, ron_op_mode_t mode,
                                 ron_float_t manual_inner, ron_float_t manual_outer)
{
    ron_fault_t fault;

    fault = cascade_check_inst(casc);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!cascade_isfinite(manual_inner) || !cascade_isfinite(manual_outer)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    if (mode == RON_MODE_MANUAL) {
        /* AUTO → MANUAL: freeze outer first so it stops advancing while inner winds down */
        fault = ron_pid_set_mode(&casc->outer, RON_MODE_MANUAL, manual_outer);
        fault |= ron_pid_set_mode(&casc->inner, RON_MODE_MANUAL, manual_inner);
    } else {
        /* MANUAL → AUTO: inner resumes tracking first (bumpless), then outer */
        fault = ron_pid_set_mode(&casc->inner, RON_MODE_AUTOMATIC, manual_inner);
        fault |= ron_pid_set_mode(&casc->outer, RON_MODE_AUTOMATIC, manual_outer);
    }

    return fault;
}

/* Satisfies: RON-FR-406 | Test: RON-TC-CASC-010 */
ron_fault_t ron_cascade_get_state(const ron_cascade_instance_t *casc, ron_cascade_status_t *status,
                                  ron_fault_t *outer_fault, ron_fault_t *inner_fault)
{
    ron_status_t outer_s;
    ron_status_t inner_s;

    if (casc == NULL) {
        return RON_FAULT_NULL_POINTER;
    }

    (void) ron_pid_get_state(&casc->outer, NULL, NULL, NULL, &outer_s, outer_fault);
    (void) ron_pid_get_state(&casc->inner, NULL, NULL, NULL, &inner_s, inner_fault);

    if (status != NULL) {
        *status = cascade_build_status(outer_s, inner_s);
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-405 | Test: RON-TC-CASC-011 */
ron_fault_t ron_cascade_fault_clear(ron_cascade_instance_t *casc)
{
    ron_fault_t fault;

    fault = cascade_check_inst(casc);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    (void) ron_pid_fault_clear(&casc->outer);
    (void) ron_pid_fault_clear(&casc->inner);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-405 | Test: RON-TC-CASC-012 */
ron_fault_t ron_cascade_reset(ron_cascade_instance_t *casc)
{
    ron_fault_t fault;

    fault = cascade_check_inst(casc);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }

    (void) ron_pid_reset(&casc->outer);
    (void) ron_pid_reset(&casc->inner);
    return RON_FAULT_NONE;
}
