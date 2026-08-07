/*
 * @file     lqg_no_heap_proof.c
 * @brief    CBMC harness for LQG no-heap safety and output saturation.
 * @module   lqg_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-FR-757, RON-FR-759, RON-SR-003
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves that a bounded finite predict/update/step cycle of the LQG
 * controller performs no dynamic allocation, keeps every output within its
 * configured hard limits, and leaves the embedded Kalman state estimate
 * finite.  Uses a pre-computed gain (RON_LQG_GAIN_PRECOMPUTED) so the proof
 * exercises the runtime path directly without unwinding the bounded-
 * iteration DARE solver, which is out of scope for this property.
 */

#include "ron/ron_lqg.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define LQG_SAT_SIGNAL_BOUND RON_FLOAT_C(1000.0)
#define LQG_SAT_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-759, RON-SR-003 | Test: RON-TC-LQG-010-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "LQG controller must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-759, RON-SR-003 | Test: RON-TC-LQG-010-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "LQG controller must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-759, RON-SR-003 | Test: RON-TC-LQG-010-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "LQG controller must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-759, RON-SR-003 | Test: RON-TC-LQG-010-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "LQG controller must not call free");
}

/* Satisfies: RON-FR-757, RON-FR-759, RON-SR-003 | Test: RON-TC-LQG-010-FV */
void lqg_no_heap_proof(void)
{
    ron_lqg_t lqg;
    ron_lqg_config_t cfg = {0};
    ron_float_t predict_u[RON_LQR_MAX_INPUTS];
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];
    ron_float_t r[RON_LQR_MAX_INPUTS];
    ron_float_t u[RON_LQR_MAX_INPUTS];
    ron_float_t x_hat[RON_LQR_MAX_STATES];
    ron_float_t z0      = nondet_ron_float_t();
    ron_float_t r0      = nondet_ron_float_t();
    ron_float_t dt      = nondet_ron_float_t();
    ron_status_t status = RON_STATUS_OK;

    __CPROVER_assume(RON_ISFINITE(z0));
    __CPROVER_assume(RON_ISFINITE(r0));
    __CPROVER_assume(RON_ISFINITE(dt));
    __CPROVER_assume(dt > RON_FLOAT_C(0.0));
    __CPROVER_assume(dt <= LQG_SAT_DT_MAX);
    __CPROVER_assume(z0 >= -LQG_SAT_SIGNAL_BOUND);
    __CPROVER_assume(z0 <= LQG_SAT_SIGNAL_BOUND);
    __CPROVER_assume(r0 >= -LQG_SAT_SIGNAL_BOUND);
    __CPROVER_assume(r0 <= LQG_SAT_SIGNAL_BOUND);

    z[0]              = z0;
    r[0]              = r0;
    predict_u[0]      = RON_FLOAT_C(0.0);
    cfg.n             = 1U;
    cfg.m             = 1U;
    cfg.p             = 1U;
    cfg.gain_mode     = RON_LQG_GAIN_PRECOMPUTED;
    cfg.A[0][0]       = RON_FLOAT_C(1.0);
    cfg.H[0][0]       = RON_FLOAT_C(1.0);
    cfg.Q_noise[0][0] = RON_FLOAT_C(0.01);
    cfg.R_noise[0][0] = RON_FLOAT_C(1.0);
    cfg.P0[0][0]      = RON_FLOAT_C(10.0);
    cfg.K[0][0]       = RON_FLOAT_C(2.0);
    cfg.Kr[0]         = RON_FLOAT_C(1.0);
    cfg.u_min[0]      = RON_FLOAT_C(-5.0);
    cfg.u_max[0]      = RON_FLOAT_C(5.0);
    cfg.du_max[0]     = RON_FLOAT_C(0.0);

    __CPROVER_assert(ron_lqg_init(&lqg, &cfg) == RON_FAULT_NONE, "valid config initialises");
    __CPROVER_assert(ron_lqg_predict(&lqg, predict_u) == RON_FAULT_NONE,
                     "predict completes without heap");
    __CPROVER_assert(ron_lqg_update(&lqg, z, true) == RON_FAULT_NONE,
                     "update completes without heap");
    __CPROVER_assert(ron_lqg_step(&lqg, r, dt, u, &status) == RON_FAULT_NONE,
                     "bounded finite single-step execution does not fault");
    __CPROVER_assert((u[0] >= cfg.u_min[0]) && (u[0] <= cfg.u_max[0]),
                     "output remains within configured hard limits");
    __CPROVER_assert(ron_lqg_get_state(&lqg, x_hat) == RON_FAULT_NONE, "get_state has no heap");
    __CPROVER_assert(RON_ISFINITE(x_hat[0]), "Kalman state estimate remains finite");
}
