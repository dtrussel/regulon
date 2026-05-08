/*
 * @file     ron_trajectory_trap.c
 * @brief    Trapezoidal velocity trajectory generator.
 * @module   ron_trajectory
 * @doc      RON-IS-001
 * @req      RON-FR-500, RON-FR-501, RON-FR-502, RON-FR-503,
 *           RON-FR-512, RON-FR-513
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_trajectory.h"

#define TRAP_SQRT_STEPS (16U)
#define TRAP_POS_TOL RON_FLOAT_C(0.000001)

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static bool trap_isfinite(ron_float_t value)
{
    return RON_ISFINITE(value);
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static ron_float_t trap_sqrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x    = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    step = 0U;
    while (step < TRAP_SQRT_STEPS) {
        x = RON_FLOAT_C(0.5) * (x + (value / x));
        step++;
    }

    return x;
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static ron_fault_t trap_validate_config(const ron_trap_config_t *cfg)
{
    if (!trap_isfinite(cfg->v_max) || !trap_isfinite(cfg->a_max)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((cfg->v_max <= RON_FLOAT_C(0.0)) || (cfg->a_max <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-502 | Test: RON-TC-TRAJ-003 */
static ron_float_t trap_abs(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? (-value) : value;
}

/* Satisfies: RON-FR-503 | Test: RON-TC-TRAJ-004 */
static ron_float_t trap_sign_nonzero(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? RON_FLOAT_C(-1.0) : RON_FLOAT_C(1.0);
}

/* Satisfies: RON-FR-501, RON-FR-503 | Test: RON-TC-TRAJ-002, RON-TC-TRAJ-004 */
static void trap_plan(ron_trap_t *t)
{
    ron_float_t distance;
    ron_float_t peak;

    distance = trap_abs(t->state.target - t->state.pos);
    if (distance <= TRAP_POS_TOL) {
        t->state.direction = RON_FLOAT_C(1.0);
        t->state.v_peak    = RON_FLOAT_C(0.0);
        t->state.phase     = RON_TRAP_PHASE_DONE;
        t->state.finished  = true;
        return;
    }

    t->state.direction = trap_sign_nonzero(t->state.target - t->state.pos);
    peak               = trap_sqrt(t->cfg.a_max * distance);
    t->state.v_peak    = ron_clamp(peak, RON_FLOAT_C(0.0), t->cfg.v_max);
    t->state.phase     = RON_TRAP_PHASE_ACCEL;
    t->state.finished  = false;
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void trap_finish(ron_trap_t *t)
{
    t->state.pos      = t->state.target;
    t->state.vel      = RON_FLOAT_C(0.0);
    t->state.acc      = RON_FLOAT_C(0.0);
    t->state.phase    = RON_TRAP_PHASE_DONE;
    t->state.finished = true;
    t->state.status   = RON_STATUS_OK;
}

/* Satisfies: RON-FR-502 | Test: RON-TC-TRAJ-003 */
static void trap_write_outputs(const ron_trap_t *t, ron_float_t *pos, ron_float_t *vel,
                               ron_float_t *acc, bool *finished)
{
    *pos      = t->state.pos;
    *vel      = t->state.vel;
    *acc      = t->state.acc;
    *finished = t->state.finished;
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static ron_fault_t trap_check_step_args(const ron_trap_t *t, const ron_float_t *pos,
                                        const ron_float_t *vel, const ron_float_t *acc,
                                        const bool *finished)
{
    if ((t == NULL) || (pos == NULL) || (vel == NULL) || (acc == NULL) || (finished == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!t->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static bool trap_step_bypass(ron_trap_t *t, ron_float_t *pos, ron_float_t *vel, ron_float_t *acc,
                             bool *finished, ron_fault_t *fault)
{
    *fault = RON_FAULT_NONE;
    if (t->state.fault_code != RON_FAULT_NONE) {
        trap_write_outputs(t, pos, vel, acc, finished);
        *fault = t->state.fault_code;
        return true;
    }
    if (t->state.hold) {
        t->state.phase = RON_TRAP_PHASE_HOLD;
        trap_write_outputs(t, pos, vel, acc, finished);
        return true;
    }
    if (t->state.finished) {
        trap_write_outputs(t, pos, vel, acc, finished);
        return true;
    }

    return false;
}

/* Satisfies: RON-FR-500, RON-FR-501 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-002 */
static ron_float_t trap_select_acceleration(ron_trap_t *t, ron_float_t distance,
                                            ron_float_t speed_along)
{
    ron_float_t decel_distance;
    ron_float_t commanded_acc;

    decel_distance = (speed_along > RON_FLOAT_C(0.0))
                         ? ((speed_along * speed_along) / (RON_FLOAT_C(2.0) * t->cfg.a_max))
                         : RON_FLOAT_C(0.0);
    commanded_acc  = t->state.direction * t->cfg.a_max;
    if (speed_along < RON_FLOAT_C(0.0)) {
        t->state.phase = RON_TRAP_PHASE_ACCEL;
    } else if (distance <= (decel_distance + TRAP_POS_TOL)) {
        commanded_acc  = -t->state.direction * t->cfg.a_max;
        t->state.phase = RON_TRAP_PHASE_DECEL;
    } else if (speed_along >= t->state.v_peak) {
        commanded_acc  = RON_FLOAT_C(0.0);
        t->state.phase = RON_TRAP_PHASE_CONST_VEL;
    } else {
        t->state.phase = RON_TRAP_PHASE_ACCEL;
    }

    return commanded_acc;
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static void trap_apply_velocity_clamps(ron_trap_t *t)
{
    ron_float_t next_speed_along;

    next_speed_along = t->state.vel * t->state.direction;
    if ((t->state.phase == RON_TRAP_PHASE_ACCEL) && (next_speed_along > t->state.v_peak)) {
        t->state.vel = t->state.direction * t->state.v_peak;
        t->state.acc = RON_FLOAT_C(0.0);
    } else if ((t->state.phase == RON_TRAP_PHASE_DECEL) && (next_speed_along < RON_FLOAT_C(0.0))) {
        t->state.vel = RON_FLOAT_C(0.0);
        t->state.acc = RON_FLOAT_C(0.0);
    }
}

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
static void trap_integrate_active(ron_trap_t *t, ron_float_t dt)
{
    ron_float_t distance;
    ron_float_t speed_along;

    distance     = trap_abs(t->state.target - t->state.pos);
    speed_along  = t->state.vel * t->state.direction;
    t->state.acc = trap_select_acceleration(t, distance, speed_along);
    t->state.vel += t->state.acc * dt;
    trap_apply_velocity_clamps(t);
    t->state.pos += t->state.vel * dt;
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void trap_finish_if_reached(ron_trap_t *t, ron_float_t dt)
{
    if ((t->state.direction * (t->state.target - t->state.pos) <= TRAP_POS_TOL) &&
        ((t->state.phase == RON_TRAP_PHASE_DECEL) ||
         (trap_abs(t->state.vel) <= (t->cfg.a_max * dt)))) {
        trap_finish(t);
    }
}

/* Satisfies: RON-FR-500, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_init(ron_trap_t *t, const ron_trap_config_t *cfg, ron_float_t pos0)
{
    ron_fault_t fault;

    if ((t == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = trap_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!trap_isfinite(pos0)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    t->cfg                  = *cfg;
    t->state.pos            = pos0;
    t->state.vel            = RON_FLOAT_C(0.0);
    t->state.acc            = RON_FLOAT_C(0.0);
    t->state.target         = pos0;
    t->state.direction      = RON_FLOAT_C(1.0);
    t->state.v_peak         = RON_FLOAT_C(0.0);
    t->state.phase          = RON_TRAP_PHASE_DONE;
    t->state.fault_code     = RON_FAULT_NONE;
    t->state.status         = RON_STATUS_OK;
    t->state.hold           = false;
    t->state.finished       = true;
    t->state.is_initialised = true;

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-501, RON-FR-503 | Test: RON-TC-TRAJ-002, RON-TC-TRAJ-004 */
ron_fault_t ron_trap_set_target(ron_trap_t *t, ron_float_t target)
{
    if (t == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!t->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (t->state.fault_code != RON_FAULT_NONE) {
        return t->state.fault_code;
    }
    if (!trap_isfinite(target)) {
        t->state.fault_code = RON_FAULT_CONFIG_INVALID;
        t->state.status     = RON_STATUS_FAULT;
        return RON_FAULT_CONFIG_INVALID;
    }

    t->state.target = target;
    trap_plan(t);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-500, RON-FR-502, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-003, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_step(ron_trap_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                          ron_float_t *acc, bool *finished)
{
    ron_fault_t fault;

    fault = trap_check_step_args(t, pos, vel, acc, finished);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !trap_isfinite(dt)) {
        t->state.fault_code = RON_FAULT_CONFIG_INVALID;
        t->state.status     = RON_STATUS_FAULT;
        return RON_FAULT_CONFIG_INVALID;
    }
    if (trap_step_bypass(t, pos, vel, acc, finished, &fault)) {
        return fault;
    }

    trap_integrate_active(t, dt);
    trap_finish_if_reached(t, dt);
    trap_write_outputs(t, pos, vel, acc, finished);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-513 | Test: RON-TC-TRAJ-008 */
ron_fault_t ron_trap_hold(ron_trap_t *t, bool hold)
{
    if (t == NULL) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!t->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if (t->state.fault_code != RON_FAULT_NONE) {
        return t->state.fault_code;
    }

    t->state.hold = hold;
    if (hold) {
        t->state.phase = RON_TRAP_PHASE_HOLD;
    } else if (!t->state.finished) {
        trap_plan(t);
    }

    return RON_FAULT_NONE;
}
