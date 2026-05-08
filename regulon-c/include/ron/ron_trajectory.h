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

/* Satisfies: RON-FR-500, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_init(ron_trap_t *t, const ron_trap_config_t *cfg, ron_float_t pos0);

/* Satisfies: RON-FR-501, RON-FR-503 | Test: RON-TC-TRAJ-002, RON-TC-TRAJ-004 */
ron_fault_t ron_trap_set_target(ron_trap_t *t, ron_float_t target);

/* Satisfies: RON-FR-500, RON-FR-502, RON-FR-512 | Test: RON-TC-TRAJ-001, RON-TC-TRAJ-003, RON-TC-TRAJ-007 */
ron_fault_t ron_trap_step(ron_trap_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                          ron_float_t *acc, bool *finished);

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

/* Satisfies: RON-FR-510, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_init(ron_scurve_t *t, const ron_scurve_config_t *cfg, ron_float_t pos0);

/* Satisfies: RON-FR-510 | Test: RON-TC-TRAJ-005 */
ron_fault_t ron_scurve_set_target(ron_scurve_t *t, ron_float_t target);

/* Satisfies: RON-FR-510, RON-FR-511, RON-FR-512 | Test: RON-TC-TRAJ-005, RON-TC-TRAJ-006, RON-TC-TRAJ-007 */
ron_fault_t ron_scurve_step(ron_scurve_t *t, ron_float_t dt, ron_float_t *pos, ron_float_t *vel,
                            ron_float_t *acc, ron_float_t *jrk, bool *finished);

/* Satisfies: RON-FR-513 | Test: RON-TC-TRAJ-008 */
ron_fault_t ron_scurve_hold(ron_scurve_t *t, bool hold);

#ifdef __cplusplus
}
#endif

#endif /* RON_TRAJECTORY_H */
