/*
 * @file     ron_trajectory_scurve.c
 * @brief    Jerk-limited S-curve trajectory generator.
 * @module   ron_trajectory
 * @doc      RON-IS-001
 * @req      RON-FR-510, RON-FR-511, RON-FR-512, RON-FR-513
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_trajectory.h"

#define SCURVE_ROOT_STEPS (18U)
#define SCURVE_POS_TOL RON_FLOAT_C(0.000001)
#define SCURVE_ONE_SIXTH RON_FLOAT_C(0.1666666666666667)

/* Satisfies: RON-SR-020 | Test: RON-TC-SAFE-011 */
static bool scurve_isfinite(ron_float_t value)
{
    return RON_ISFINITE(value);
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_float_t scurve_sqrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x    = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    step = 0U;
    while (step < SCURVE_ROOT_STEPS) {
        x = RON_FLOAT_C(0.5) * (x + (value / x));
        step++;
    }

    return x;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_float_t scurve_cbrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x    = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    step = 0U;
    while (step < SCURVE_ROOT_STEPS) {
        x = ((RON_FLOAT_C(2.0) * x) + (value / (x * x))) / RON_FLOAT_C(3.0);
        step++;
    }

    return x;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_float_t scurve_abs(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? (-value) : value;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_float_t scurve_min(ron_float_t lhs, ron_float_t rhs)
{
    return (lhs < rhs) ? lhs : rhs;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_float_t scurve_sign_nonzero(ron_float_t value)
{
    return (value < RON_FLOAT_C(0.0)) ? RON_FLOAT_C(-1.0) : RON_FLOAT_C(1.0);
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_scurve_phase_t scurve_next_phase(ron_scurve_phase_t phase)
{
    ron_scurve_phase_t next;

    next = RON_SCURVE_PHASE_DONE;
    if (phase == RON_SCURVE_PHASE_JERK_POS_1) {
        next = RON_SCURVE_PHASE_ACCEL_HOLD;
    } else if (phase == RON_SCURVE_PHASE_ACCEL_HOLD) {
        next = RON_SCURVE_PHASE_JERK_NEG_1;
    } else if (phase == RON_SCURVE_PHASE_JERK_NEG_1) {
        next = RON_SCURVE_PHASE_CONST_VEL;
    } else if (phase == RON_SCURVE_PHASE_CONST_VEL) {
        next = RON_SCURVE_PHASE_JERK_NEG_2;
    } else if (phase == RON_SCURVE_PHASE_JERK_NEG_2) {
        next = RON_SCURVE_PHASE_DECEL_HOLD;
    } else if (phase == RON_SCURVE_PHASE_DECEL_HOLD) {
        next = RON_SCURVE_PHASE_JERK_POS_2;
    }

    return next;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_fault_t scurve_validate_config(const ron_scurve_config_t *cfg)
{
    if (!scurve_isfinite(cfg->v_max) || !scurve_isfinite(cfg->a_max) ||
        !scurve_isfinite(cfg->j_max)) {
        return RON_FAULT_CONFIG_INVALID;
    }
    if ((cfg->v_max <= RON_FLOAT_C(0.0)) || (cfg->a_max <= RON_FLOAT_C(0.0)) ||
        (cfg->j_max <= RON_FLOAT_C(0.0))) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void scurve_finish(ron_scurve_t *t)
{
    t->state.pos      = t->state.target;
    t->state.vel      = RON_FLOAT_C(0.0);
    t->state.acc      = RON_FLOAT_C(0.0);
    t->state.jrk      = RON_FLOAT_C(0.0);
    t->state.elapsed  = t->state.t_total;
    t->state.t_phase  = RON_FLOAT_C(0.0);
    t->state.phase    = RON_SCURVE_PHASE_DONE;
    t->state.finished = true;
    t->state.status   = RON_STATUS_OK;
}

/* Satisfies: RON-FR-511 | Test: RON-TC-TRAJ-006 */
static void scurve_write_outputs(const ron_scurve_t *t, ron_float_t *pos, ron_float_t *vel,
                                 ron_float_t *acc, ron_float_t *jrk, bool *finished)
{
    *pos      = t->state.pos;
    *vel      = t->state.vel;
    *acc      = t->state.acc;
    *jrk      = t->state.jrk;
    *finished = t->state.finished;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static void scurve_clear_phase_times(ron_scurve_state_t *state)
{
    uint8_t idx;

    idx = 0U;
    while (idx < RON_SCURVE_PHASE_COUNT) {
        state->phase_time[idx] = RON_FLOAT_C(0.0);
        idx++;
    }
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static void scurve_plan(ron_scurve_t *t)
{
    ron_float_t distance;
    ron_float_t t_acc_limit;
    ron_float_t t_vel_limit;
    ron_float_t t_limit;
    ron_float_t t_short;
    ron_float_t cruise_time;

    scurve_clear_phase_times(&t->state);
    distance = scurve_abs(t->state.target - t->state.pos);
    if (distance <= SCURVE_POS_TOL) {
        scurve_finish(t);
        return;
    }

    t->state.direction = scurve_sign_nonzero(t->state.target - t->state.pos);
    t_acc_limit        = t->cfg.a_max / t->cfg.j_max;
    t_vel_limit        = scurve_sqrt(t->cfg.v_max / t->cfg.j_max);
    t_limit            = scurve_min(t_acc_limit, t_vel_limit);
    t_short            = scurve_cbrt(distance / (RON_FLOAT_C(2.0) * t->cfg.j_max));
    t->state.t_phase   = RON_FLOAT_C(0.0);
    t->state.elapsed   = RON_FLOAT_C(0.0);
    t->state.phase     = RON_SCURVE_PHASE_JERK_POS_1;
    t->state.finished  = false;

    if (t_short < t_limit) {
        t_limit     = t_short;
        cruise_time = RON_FLOAT_C(0.0);
    } else {
        ron_float_t base_distance;
        ron_float_t v_peak;

        v_peak        = t->cfg.j_max * t_limit * t_limit;
        base_distance = RON_FLOAT_C(2.0) * t->cfg.j_max * t_limit * t_limit * t_limit;
        cruise_time   = (distance - base_distance) / v_peak;
    }

    t->state.phase_time[RON_SCURVE_PHASE_JERK_POS_1] = t_limit;
    t->state.phase_time[RON_SCURVE_PHASE_ACCEL_HOLD] = RON_FLOAT_C(0.0);
    t->state.phase_time[RON_SCURVE_PHASE_JERK_NEG_1] = t_limit;
    t->state.phase_time[RON_SCURVE_PHASE_CONST_VEL]  = cruise_time;
    t->state.phase_time[RON_SCURVE_PHASE_JERK_NEG_2] = t_limit;
    t->state.phase_time[RON_SCURVE_PHASE_DECEL_HOLD] = RON_FLOAT_C(0.0);
    t->state.phase_time[RON_SCURVE_PHASE_JERK_POS_2] = t_limit;
    t->state.t_total                                 = (RON_FLOAT_C(4.0) * t_limit) + cruise_time;
}

/* Satisfies: RON-FR-511 | Test: RON-TC-TRAJ-006 */
static ron_float_t scurve_phase_jerk(const ron_scurve_t *t)
{
    ron_float_t jerk;

    jerk = RON_FLOAT_C(0.0);
    if ((t->state.phase == RON_SCURVE_PHASE_JERK_POS_1) ||
        (t->state.phase == RON_SCURVE_PHASE_JERK_POS_2)) {
        jerk = t->state.direction * t->cfg.j_max;
    } else if ((t->state.phase == RON_SCURVE_PHASE_JERK_NEG_1) ||
               (t->state.phase == RON_SCURVE_PHASE_JERK_NEG_2)) {
        jerk = -t->state.direction * t->cfg.j_max;
    }

    return jerk;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static void scurve_advance_empty_phases(ron_scurve_t *t)
{
    while ((t->state.phase < RON_SCURVE_PHASE_DONE) &&
           (t->state.phase_time[t->state.phase] <= SCURVE_POS_TOL)) {
        t->state.phase   = scurve_next_phase(t->state.phase);
        t->state.t_phase = RON_FLOAT_C(0.0);
    }
}

/* Satisfies: RON-FR-510, RON-FR-511 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-006 */
static void scurve_integrate_segment(ron_scurve_t *t, ron_float_t h, ron_float_t jerk)
{
    ron_float_t h2;
    ron_float_t h3;

    h2 = h * h;
    h3 = h2 * h;
    t->state.pos += (t->state.vel * h) + (RON_FLOAT_C(0.5) * t->state.acc * h2) +
                    (SCURVE_ONE_SIXTH * jerk * h3);
    t->state.vel += (t->state.acc * h) + (RON_FLOAT_C(0.5) * jerk * h2);
    t->state.acc += jerk * h;
    t->state.jrk = jerk;
}

/* Satisfies: RON-FR-510, RON-FR-511 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-006 */
static void scurve_integrate_step(ron_scurve_t *t, ron_float_t dt)
{
    ron_float_t remaining_dt;
    bool done;
    uint8_t loop_count;

    remaining_dt = dt;
    done         = false;
    for (loop_count = 0U; loop_count < RON_SCURVE_PHASE_COUNT; loop_count++) {
        if (!done) {
            if ((remaining_dt <= RON_FLOAT_C(0.0)) || (t->state.phase >= RON_SCURVE_PHASE_DONE)) {
                done = true;
            }
        }
        if (!done) {
            scurve_advance_empty_phases(t);
            if (t->state.phase >= RON_SCURVE_PHASE_DONE) {
                done = true;
            }
        }
        if (!done) {
            ron_float_t phase_remaining;
            ron_float_t h;
            ron_float_t jerk;

            phase_remaining = t->state.phase_time[t->state.phase] - t->state.t_phase;
            h               = (remaining_dt < phase_remaining) ? remaining_dt : phase_remaining;
            jerk            = scurve_phase_jerk(t);
            scurve_integrate_segment(t, h, jerk);
            t->state.t_phase += h;
            t->state.elapsed += h;
            remaining_dt -= h;

            if ((phase_remaining - h) <= SCURVE_POS_TOL) {
                t->state.phase   = scurve_next_phase(t->state.phase);
                t->state.t_phase = RON_FLOAT_C(0.0);
            }
        }
    }
}

/* Satisfies: RON-FR-510, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_init(ron_scurve_t *t, const ron_scurve_config_t *cfg, ron_float_t pos0)
{
    ron_fault_t fault;

    if ((t == NULL) || (cfg == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }

    fault = scurve_validate_config(cfg);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if (!scurve_isfinite(pos0)) {
        return RON_FAULT_CONFIG_INVALID;
    }

    t->cfg                  = *cfg;
    t->state.pos            = pos0;
    t->state.vel            = RON_FLOAT_C(0.0);
    t->state.acc            = RON_FLOAT_C(0.0);
    t->state.jrk            = RON_FLOAT_C(0.0);
    t->state.target         = pos0;
    t->state.direction      = RON_FLOAT_C(1.0);
    t->state.t_phase        = RON_FLOAT_C(0.0);
    t->state.t_total        = RON_FLOAT_C(0.0);
    t->state.elapsed        = RON_FLOAT_C(0.0);
    t->state.phase          = RON_SCURVE_PHASE_DONE;
    t->state.fault_code     = RON_FAULT_NONE;
    t->state.status         = RON_STATUS_OK;
    t->state.hold           = false;
    t->state.finished       = true;
    t->state.is_initialised = true;
    scurve_clear_phase_times(&t->state);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
ron_fault_t ron_scurve_set_target(ron_scurve_t *t, ron_float_t target)
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
    if (!scurve_isfinite(target)) {
        t->state.fault_code = RON_FAULT_CONFIG_INVALID;
        t->state.status     = RON_STATUS_FAULT;
        return RON_FAULT_CONFIG_INVALID;
    }

    t->state.target = target;
    scurve_plan(t);

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static ron_fault_t scurve_check_step_args(const ron_scurve_t *t, const ron_float_t *pos,
                                          const ron_float_t *vel, const ron_float_t *acc,
                                          const ron_float_t *jrk, const bool *finished)
{
    if ((t == NULL) || (pos == NULL) || (vel == NULL) || (acc == NULL) || (jrk == NULL) ||
        (finished == NULL)) {
        return RON_FAULT_NULL_POINTER;
    }
    if (!t->state.is_initialised) {
        return RON_FAULT_CONFIG_INVALID;
    }

    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
static bool scurve_step_bypass(ron_scurve_t *t, ron_float_t *pos, ron_float_t *vel,
                               ron_float_t *acc, ron_float_t *jrk, bool *finished,
                               ron_fault_t *fault)
{
    *fault = RON_FAULT_NONE;
    if (t->state.fault_code != RON_FAULT_NONE) {
        scurve_write_outputs(t, pos, vel, acc, jrk, finished);
        *fault = t->state.fault_code;
        return true;
    }
    if (t->state.hold || t->state.finished) {
        scurve_write_outputs(t, pos, vel, acc, jrk, finished);
        return true;
    }

    return false;
}

/* Satisfies: RON-FR-512 | Test: RON-TC-TRAJ-007 */
static void scurve_finish_if_done(ron_scurve_t *t)
{
    if ((t->state.elapsed >= (t->state.t_total - SCURVE_POS_TOL)) ||
        (t->state.phase >= RON_SCURVE_PHASE_DONE)) {
        scurve_finish(t);
    }
}

/* Satisfies: RON-FR-510, RON-FR-511, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-006, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_step(ron_scurve_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                            ron_float_t *acc, ron_float_t *jrk, bool *finished)
{
    ron_fault_t fault;

    fault = scurve_check_step_args(t, pos, vel, acc, jrk, finished);
    if (fault != RON_FAULT_NONE) {
        return fault;
    }
    if ((dt <= RON_FLOAT_C(0.0)) || !scurve_isfinite(dt)) {
        t->state.fault_code = RON_FAULT_CONFIG_INVALID;
        t->state.status     = RON_STATUS_FAULT;
        return RON_FAULT_CONFIG_INVALID;
    }
    if (scurve_step_bypass(t, pos, vel, acc, jrk, finished, &fault)) {
        return fault;
    }

    scurve_integrate_step(t, dt);
    scurve_finish_if_done(t);
    scurve_write_outputs(t, pos, vel, acc, jrk, finished);
    return RON_FAULT_NONE;
}

/* Satisfies: RON-FR-513 | Test: RON-TC-TRAJ-008 */
ron_fault_t ron_scurve_hold(ron_scurve_t *t, bool hold)
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
    return RON_FAULT_NONE;
}
