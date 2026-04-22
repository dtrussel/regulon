/*
 * @file     pid_no_heap_proof.c
 * @brief    CBMC harness for PID no-heap behaviour.
 * @module   pid_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "pid_proof_common.h"

/* Satisfies: RON-SR-003, RON-PR-022 | Test: RON-TC-SAFE-003-FV */
void *malloc(size_t size)
{
    (void)size;
    __CPROVER_assert(0, "PID library must not call malloc");
    return (void *)0;
}

/* Satisfies: RON-SR-003, RON-PR-022 | Test: RON-TC-SAFE-003-FV */
void free(void *ptr)
{
    (void)ptr;
    __CPROVER_assert(0, "PID library must not call free");
}

/* Satisfies: RON-SR-003, RON-PR-022 | Test: RON-TC-SAFE-003-FV */
void *calloc(size_t count, size_t size)
{
    (void)count;
    (void)size;
    __CPROVER_assert(0, "PID library must not call calloc");
    return (void *)0;
}

/* Satisfies: RON-SR-003, RON-PR-022 | Test: RON-TC-SAFE-003-FV */
void *realloc(void *ptr, size_t size)
{
    (void)ptr;
    (void)size;
    __CPROVER_assert(0, "PID library must not call realloc");
    return (void *)0;
}

/* Satisfies: RON-SR-003, RON-PR-022 | Test: RON-TC-SAFE-003-FV */
void pid_no_heap_proof(void)
{
    ron_pid_instance_t inst;
    ron_pid_config_t cfg;
    ron_float_t u = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    cfg = pid_proof_make_config();
    __CPROVER_assert(ron_pid_init(&inst, &cfg) == RON_FAULT_NONE,
                     "valid config initialises without heap");
    __CPROVER_assert(ron_pid_step(&inst, PID_PROOF_SIGNAL, RON_FLOAT_C(0.0),
                                  PID_PROOF_DT, &u, &status) == RON_FAULT_NONE,
                     "step completes without heap");
}
