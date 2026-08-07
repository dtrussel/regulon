/*
 * @file     lqr_saturation_proof.c
 * @brief    CBMC harness for LQR output saturation and no-heap safety.
 * @module   lqr_saturation_proof
 * @doc      RON-TP-001
 * @req      RON-FR-736, RON-FR-737, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves that, for the LQR state-feedback controller, a bounded finite step
 * keeps every output within its configured hard limits (consistent with the
 * PID/state-space saturation semantics) and that the whole LQR step path
 * performs no dynamic allocation.  Uses a pre-computed gain (RON_LQR_GAIN_
 * PRECOMPUTED) so the proof exercises ron_lqr_step() directly without
 * unwinding the bounded-iteration DARE solver, which is out of scope for
 * this property.
 */

#include "ron/ron_lqr.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define LQR_SAT_SIGNAL_BOUND RON_FLOAT_C(1000.0)
#define LQR_SAT_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-736, RON-SR-003 | Test: RON-TC-LQR-010-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "LQR controller must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-736, RON-SR-003 | Test: RON-TC-LQR-010-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "LQR controller must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-736, RON-SR-003 | Test: RON-TC-LQR-010-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "LQR controller must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-736, RON-SR-003 | Test: RON-TC-LQR-010-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "LQR controller must not call free");
}

/* Satisfies: RON-FR-736, RON-FR-737, RON-SR-003 | Test: RON-TC-LQR-010-FV */
void lqr_saturation_proof(void)
{
    ron_lqr_t lqr;
    ron_lqr_config_t cfg = {0};
    ron_float_t x_ext[1];
    ron_float_t r[RON_LQR_MAX_INPUTS];
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_float_t x0      = nondet_ron_float_t();
    ron_float_t r0      = nondet_ron_float_t();
    ron_float_t dt      = nondet_ron_float_t();
    ron_status_t status = RON_STATUS_OK;

    __CPROVER_assume(RON_ISFINITE(x0));
    __CPROVER_assume(RON_ISFINITE(r0));
    __CPROVER_assume(RON_ISFINITE(dt));
    __CPROVER_assume(dt > RON_FLOAT_C(0.0));
    __CPROVER_assume(dt <= LQR_SAT_DT_MAX);
    __CPROVER_assume(x0 >= -LQR_SAT_SIGNAL_BOUND);
    __CPROVER_assume(x0 <= LQR_SAT_SIGNAL_BOUND);
    __CPROVER_assume(r0 >= -LQR_SAT_SIGNAL_BOUND);
    __CPROVER_assume(r0 <= LQR_SAT_SIGNAL_BOUND);

    x_ext[0]      = x0;
    r[0]          = r0;
    cfg.n         = 1U;
    cfg.m         = 1U;
    cfg.source    = RON_LQR_SOURCE_EXTERNAL;
    cfg.gain_mode = RON_LQR_GAIN_PRECOMPUTED;
    cfg.x_ext     = x_ext;
    cfg.K[0][0]   = RON_FLOAT_C(2.0);
    cfg.Kr[0]     = RON_FLOAT_C(1.0);
    cfg.u_min[0]  = RON_FLOAT_C(-5.0);
    cfg.u_max[0]  = RON_FLOAT_C(5.0);
    cfg.du_max[0] = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_lqr_init(&lqr, &cfg) == RON_FAULT_NONE, "valid config initialises");
    __CPROVER_assert(ron_lqr_step(&lqr, r, dt, u, &status) == RON_FAULT_NONE,
                     "bounded finite single-step execution does not fault");
    __CPROVER_assert((u[0] >= cfg.u_min[0]) && (u[0] <= cfg.u_max[0]),
                     "output remains within configured hard limits");
}
