/*
 * @file     ron_trajectory.h
 * @brief    Public API for Regulon trajectory generators.
 * @module   ron_trajectory
 * @doc      RON-IS-001
 * @req      RON-FR-500, RON-FR-501, RON-FR-502, RON-FR-503,
 *           RON-FR-510, RON-FR-511, RON-FR-512, RON-FR-513
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_TRAJECTORY_H
#define RON_TRAJECTORY_H

#include "ron/ron_pid_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Trapezoidal velocity profile
 * ========================================================================= */

/* Satisfies: RON-FR-500, RON-FR-512, RON-FR-513 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-007, RON-TC-TRAJ-008 */
typedef enum {
    RON_TRAP_PHASE_ACCEL     = 0,
    RON_TRAP_PHASE_CONST_VEL = 1,
    RON_TRAP_PHASE_DECEL     = 2,
    RON_TRAP_PHASE_HOLD      = 3,
    RON_TRAP_PHASE_DONE      = 4
} ron_trap_phase_t;

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
typedef struct {
    ron_float_t v_max;
    ron_float_t a_max;
} ron_trap_config_t;

/* Satisfies: RON-FR-502, RON-FR-512, RON-FR-513 | Test: RON-TC-TRAJ-003, RON-TC-TRAJ-007, RON-TC-TRAJ-008 */
typedef struct {
    ron_float_t pos;
    ron_float_t vel;
    ron_float_t acc;
    ron_float_t target;
    ron_float_t direction;
    ron_float_t v_peak;
    ron_trap_phase_t phase;
    ron_fault_t fault_code;
    ron_status_t status;
    bool hold;
    bool finished;
    bool is_initialised;
} ron_trap_state_t;

/* Satisfies: RON-FR-500 | Test: RON-TC-TRAJ-001 */
typedef struct {
    ron_trap_config_t cfg;
    ron_trap_state_t state;
} ron_trap_t;

/**
 * @brief Initialise a trapezoidal (velocity-limited) motion profile generator.
 *
 * The profile starts at rest at @p pos0 with no target set; call
 * ron_trap_set_target() to begin a move.
 *
 * @param[out] t     Generator instance to initialise. Must not be NULL.
 * @param[in]  cfg   Configuration; @c v_max and @c a_max must both be positive and finite. Must not be NULL.
 * @param[in]  pos0  Initial position. Must be finite.
 *
 * @retval RON_FAULT_NONE           Generator ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p t or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A limit was non-positive or non-finite, or
 *                                  @p pos0 was not finite.
 */
/* Satisfies: RON-FR-500, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_init(ron_trap_t *t, const ron_trap_config_t *cfg, ron_float_t pos0);

/**
 * @brief Set a new target position and re-plan the profile.
 *
 * May be called mid-move. The new profile is planned from the current
 * position and velocity, so the generator decelerates and reverses as needed
 * rather than jumping.
 *
 * @param[in,out] t       Initialised generator instance. Must not be NULL.
 * @param[in]     target  Target position. Must be finite.
 *
 * @retval RON_FAULT_NONE           Target accepted and profile re-planned.
 * @retval RON_FAULT_NULL_POINTER   @p t was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised, or
 *                                  @p target was not finite.
 */
/* Satisfies: RON-FR-501, RON-FR-503 | Test: RON-TC-TRAJ-002, RON-TC-TRAJ-004 */
ron_fault_t ron_trap_set_target(ron_trap_t *t, ron_float_t target);

/**
 * @brief Advance the profile by one sample and read the current setpoint.
 *
 * Evaluates the profile incrementally, so the generator holds only its
 * current state and cost per call does not depend on the length of the move.
 * Acceleration is piecewise constant, so the velocity profile is trapezoidal and
 * short moves degenerate to a triangular profile that never reaches @c v_max.
 *
 * Once the target is reached the generator holds position and reports
 * @p finished as @c true on every subsequent call.
 *
 * @param[in,out] t         Initialised generator instance. Must not be NULL.
 * @param[in]     dt        Time step in seconds. Must be positive and finite.
 * @param[out]    pos       Receives the position setpoint. Must not be NULL.
 * @param[out]    vel       Receives the velocity setpoint. Must not be NULL.
 * @param[out]    acc       Receives the acceleration setpoint. Must not be
 *                          NULL.
 * @param[out]    finished  Receives whether the move has completed. Must not
 *                          be NULL.
 *
 * @retval RON_FAULT_NONE           Setpoints written.
 * @retval RON_FAULT_NULL_POINTER   Any output pointer, or @p t, was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised, or
 *                                  @p dt was not positive and finite.
 * @retval RON_FAULT_OUTPUT_NAN     A computed setpoint was not finite; the
 *                                  fault latches.
 */
/* Satisfies: RON-FR-500, RON-FR-502, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-003, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_step(ron_trap_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                          ron_float_t *acc, bool *finished);

/**
 * @brief Pause or resume profile execution.
 *
 * While held, ron_trap_step() keeps returning the current position with
 * zero velocity and acceleration; the target and remaining profile are
 * retained. Releasing the hold resumes the move from where it stopped.
 *
 * @param[in,out] t     Initialised generator instance. Must not be NULL.
 * @param[in]     hold  @c true to pause, @c false to resume.
 *
 * @retval RON_FAULT_NONE           Hold state updated.
 * @retval RON_FAULT_NULL_POINTER   @p t was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised.
 */
/* Satisfies: RON-FR-513 | Test: RON-TC-TRAJ-008 */
ron_fault_t ron_trap_hold(ron_trap_t *t, bool hold);

/* =========================================================================
 * S-curve jerk-limited profile
 * ========================================================================= */

#define RON_SCURVE_PHASE_COUNT (7U)

/* Satisfies: RON-FR-510, RON-FR-512, RON-FR-513 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-007, RON-TC-TRAJ-008 */
typedef enum {
    RON_SCURVE_PHASE_JERK_POS_1 = 0,
    RON_SCURVE_PHASE_ACCEL_HOLD = 1,
    RON_SCURVE_PHASE_JERK_NEG_1 = 2,
    RON_SCURVE_PHASE_CONST_VEL  = 3,
    RON_SCURVE_PHASE_JERK_NEG_2 = 4,
    RON_SCURVE_PHASE_DECEL_HOLD = 5,
    RON_SCURVE_PHASE_JERK_POS_2 = 6,
    RON_SCURVE_PHASE_DONE       = 7
} ron_scurve_phase_t;

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
typedef struct {
    ron_float_t v_max;
    ron_float_t a_max;
    ron_float_t j_max;
} ron_scurve_config_t;

/* Satisfies: RON-FR-511, RON-FR-512, RON-FR-513 | Test: RON-TC-TRAJ-006, RON-TC-TRAJ-007, RON-TC-TRAJ-008 */
typedef struct {
    ron_float_t pos;
    ron_float_t vel;
    ron_float_t acc;
    ron_float_t jrk;
    ron_float_t target;
    ron_float_t direction;
    ron_float_t phase_time[RON_SCURVE_PHASE_COUNT];
    ron_float_t t_phase;
    ron_float_t t_total;
    ron_float_t elapsed;
    ron_scurve_phase_t phase;
    ron_fault_t fault_code;
    ron_status_t status;
    bool hold;
    bool finished;
    bool is_initialised;
} ron_scurve_state_t;

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
typedef struct {
    ron_scurve_config_t cfg;
    ron_scurve_state_t state;
} ron_scurve_t;

/**
 * @brief Initialise a S-curve (jerk-limited) motion profile generator.
 *
 * The profile starts at rest at @p pos0 with no target set; call
 * ron_scurve_set_target() to begin a move.
 *
 * @param[out] t     Generator instance to initialise. Must not be NULL.
 * @param[in]  cfg   Configuration; @c v_max, @c a_max and @c j_max must all be positive and finite. Must not be NULL.
 * @param[in]  pos0  Initial position. Must be finite.
 *
 * @retval RON_FAULT_NONE           Generator ready to step.
 * @retval RON_FAULT_NULL_POINTER   @p t or @p cfg was NULL.
 * @retval RON_FAULT_CONFIG_INVALID A limit was non-positive or non-finite, or
 *                                  @p pos0 was not finite.
 */
/* Satisfies: RON-FR-510, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_init(ron_scurve_t *t, const ron_scurve_config_t *cfg, ron_float_t pos0);

/**
 * @brief Set a new target position and re-plan the profile.
 *
 * May be called mid-move. The new profile is planned from the current
 * position and velocity, so the generator decelerates and reverses as needed
 * rather than jumping.
 *
 * @param[in,out] t       Initialised generator instance. Must not be NULL.
 * @param[in]     target  Target position. Must be finite.
 *
 * @retval RON_FAULT_NONE           Target accepted and profile re-planned.
 * @retval RON_FAULT_NULL_POINTER   @p t was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised, or
 *                                  @p target was not finite.
 */
/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
ron_fault_t ron_scurve_set_target(ron_scurve_t *t, ron_float_t target);

/**
 * @brief Advance the profile by one sample and read the current setpoint.
 *
 * Evaluates the profile incrementally, so the generator holds only its
 * current state and cost per call does not depend on the length of the move.
 * Jerk is bounded, so acceleration ramps rather than stepping - the profile has
 * seven phases and excites far less mechanical resonance than a trapezoidal move.
 *
 * Once the target is reached the generator holds position and reports
 * @p finished as @c true on every subsequent call.
 *
 * @param[in,out] t         Initialised generator instance. Must not be NULL.
 * @param[in]     dt        Time step in seconds. Must be positive and finite.
 * @param[out]    pos       Receives the position setpoint. Must not be NULL.
 * @param[out]    vel       Receives the velocity setpoint. Must not be NULL.
 * @param[out]    acc       Receives the acceleration setpoint. Must not be
 *                          NULL.
 * @param[out]    jrk       Receives the jerk setpoint. Must not be NULL.
 * @param[out]    finished  Receives whether the move has completed. Must not
 *                          be NULL.
 *
 * @retval RON_FAULT_NONE           Setpoints written.
 * @retval RON_FAULT_NULL_POINTER   Any output pointer, or @p t, was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised, or
 *                                  @p dt was not positive and finite.
 * @retval RON_FAULT_OUTPUT_NAN     A computed setpoint was not finite; the
 *                                  fault latches.
 */
/* Satisfies: RON-FR-510, RON-FR-511, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-006, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_step(ron_scurve_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                            ron_float_t *acc, ron_float_t *jrk, bool *finished);

/**
 * @brief Pause or resume profile execution.
 *
 * While held, ron_scurve_step() keeps returning the current position with
 * zero velocity and acceleration; the target and remaining profile are
 * retained. Releasing the hold resumes the move from where it stopped.
 *
 * @param[in,out] t     Initialised generator instance. Must not be NULL.
 * @param[in]     hold  @c true to pause, @c false to resume.
 *
 * @retval RON_FAULT_NONE           Hold state updated.
 * @retval RON_FAULT_NULL_POINTER   @p t was NULL.
 * @retval RON_FAULT_CONFIG_INVALID The generator was never initialised.
 */
/* Satisfies: RON-FR-513 | Test: RON-TC-TRAJ-008 */
ron_fault_t ron_scurve_hold(ron_scurve_t *t, bool hold);

#ifdef __cplusplus
}
#endif

#endif /* RON_TRAJECTORY_H */
