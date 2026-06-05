/*
 * @file     statespace_sat_proof.c
 * @brief    CBMC harness for state-space output saturation and no-heap safety.
 * @module   statespace_sat_proof
 * @doc      RON-TP-001
 * @req      RON-FR-703, RON-FR-020, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves, for the state-feedback controller, that a bounded finite step keeps
 * the output within the configured hard limits (consistent with the PID
 * saturation semantics) and that the whole state-space path — including the
 * shared bounded matrix helper — performs no dynamic allocation.
 */

#include "ron/ron_statespace.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define SS_SAT_SIGNAL_BOUND RON_FLOAT_C(1000.0)
#define SS_SAT_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-703, RON-SR-003 | Test: RON-TC-SS-004-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "state-space controller must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-703, RON-SR-003 | Test: RON-TC-SS-004-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "state-space controller must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-703, RON-SR-003 | Test: RON-TC-SS-004-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "state-space controller must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-703, RON-SR-003 | Test: RON-TC-SS-004-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "state-space controller must not call free");
}

/* Satisfies: RON-FR-703, RON-FR-020, RON-SR-003 | Test: RON-TC-SS-004-FV */
void statespace_sat_proof(void)
{
    ron_ss_t ss;
    ron_ss_config_t cfg = {0};
    ron_float_t x_ext[1];
    ron_float_t r       = nondet_ron_float_t();
    ron_float_t x0      = nondet_ron_float_t();
    ron_float_t dt      = nondet_ron_float_t();
    ron_float_t u       = RON_FLOAT_C(0.0);
    ron_status_t status = RON_STATUS_OK;

    __CPROVER_assume(RON_ISFINITE(r));
    __CPROVER_assume(RON_ISFINITE(x0));
    __CPROVER_assume(RON_ISFINITE(dt));
    __CPROVER_assume(dt > RON_FLOAT_C(0.0));
    __CPROVER_assume(dt <= SS_SAT_DT_MAX);
    __CPROVER_assume(r >= -SS_SAT_SIGNAL_BOUND);
    __CPROVER_assume(r <= SS_SAT_SIGNAL_BOUND);
    __CPROVER_assume(x0 >= -SS_SAT_SIGNAL_BOUND);
    __CPROVER_assume(x0 <= SS_SAT_SIGNAL_BOUND);

    x_ext[0]   = x0;
    cfg.n      = 1U;
    cfg.source = RON_SS_SOURCE_EXTERNAL;
    cfg.x_ext  = x_ext;
    cfg.K[0]   = RON_FLOAT_C(2.0);
    cfg.Kr     = RON_FLOAT_C(1.0);
    cfg.u_min  = RON_FLOAT_C(-5.0);
    cfg.u_max  = RON_FLOAT_C(5.0);
    cfg.du_max = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_ss_init(&ss, &cfg) == RON_FAULT_NONE, "valid config initialises");
    __CPROVER_assert(ron_ss_step(&ss, r, dt, &u, &status) == RON_FAULT_NONE,
                     "bounded finite single-step execution does not fault");
    __CPROVER_assert((u >= cfg.u_min) && (u <= cfg.u_max),
                     "output remains within configured hard limits");
}
