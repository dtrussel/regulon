/*
 * @file     ron_gain_sched.h
 * @brief    Public API for the Regulon gain-scheduling extension.
 * @module   ron_gain_sched
 * @doc      RON-IS-001
 * @req      RON-FR-300, RON-FR-301, RON-FR-302, RON-FR-303,
 *           RON-FR-304, RON-FR-305, RON-FR-306
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#ifndef RON_GAIN_SCHED_H
#define RON_GAIN_SCHED_H

#include "ron/ron_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Satisfies: RON-FR-302 | Test: RON-TC-GS-003, RON-TC-GS-004 */
typedef enum { RON_GS_HARD_SWITCH = 0, RON_GS_LINEAR_INTERP = 1 } ron_gs_mode_t;

/* Satisfies: RON-FR-300, RON-FR-301 | Test: RON-TC-GS-001, RON-TC-GS-002 */
typedef struct {
    uint8_t n_points;
    ron_float_t sigma[RON_GS_MAX_BREAKPOINTS];
    ron_pid_config_t configs[RON_GS_MAX_BREAKPOINTS];
    ron_gs_mode_t mode;
    bool reset_integral_on_switch;
} ron_gs_table_t;

/* Satisfies: RON-FR-300, RON-FR-301, RON-FR-306 | Test: RON-TC-GS-001, RON-TC-GS-002, RON-TC-GS-008 */
ron_fault_t ron_gs_init(const ron_gs_table_t *tbl);

/* Satisfies: RON-FR-302, RON-FR-303, RON-FR-304, RON-FR-305 | Test: RON-TC-GS-003 - RON-TC-GS-007 */
ron_fault_t ron_gs_update(const ron_gs_table_t *tbl, ron_pid_instance_t *pid, ron_float_t sigma);

#ifdef __cplusplus
}
#endif

#endif /* RON_GAIN_SCHED_H */
