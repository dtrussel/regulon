/*
 * @file     kalman_no_heap_proof.c
 * @brief    CBMC harness proving the Kalman filter performs no heap allocation.
 * @module   kalman_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-FR-607, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron/ron_kalman.h"

extern void __CPROVER_assert(int condition, const char *description);

/* Satisfies: RON-FR-607, RON-SR-003 | Test: RON-TC-KF-008-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "Kalman filter must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-607, RON-SR-003 | Test: RON-TC-KF-008-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "Kalman filter must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-607, RON-SR-003 | Test: RON-TC-KF-008-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "Kalman filter must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-607, RON-SR-003 | Test: RON-TC-KF-008-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "Kalman filter must not call free");
}

/* Satisfies: RON-FR-607, RON-SR-003, RON-PR-022 | Test: RON-TC-KF-008-FV */
void kalman_no_heap_proof(void)
{
    ron_kf_t kf;
    ron_kf_config_t cfg;
    ron_float_t z[RON_KF_MAX_MEASUREMENTS];
    ron_float_t x_hat[RON_KF_MAX_STATES];

    cfg.n               = 1U;
    cfg.m               = 1U;
    cfg.p               = 0U;
    cfg.A[0][0]         = RON_FLOAT_C(1.0);
    cfg.H[0][0]         = RON_FLOAT_C(1.0);
    cfg.Q[0][0]         = RON_FLOAT_C(0.01);
    cfg.R[0][0]         = RON_FLOAT_C(1.0);
    cfg.x0[0]           = RON_FLOAT_C(0.0);
    cfg.P0[0][0]        = RON_FLOAT_C(10.0);
    cfg.K_inf[0][0]     = RON_FLOAT_C(0.0);
    cfg.use_joseph_form = false;
    cfg.steady_state    = false;
    z[0]                = RON_FLOAT_C(1.0);

    __CPROVER_assert(ron_kf_init(&kf, &cfg) == RON_FAULT_NONE,
                     "valid Kalman config initialises without heap");
    __CPROVER_assert(ron_kf_predict(&kf, (const ron_float_t *) 0) == RON_FAULT_NONE,
                     "Kalman predict completes without heap");
    __CPROVER_assert(ron_kf_update(&kf, z, true) == RON_FAULT_NONE,
                     "Kalman update completes without heap");
    __CPROVER_assert(ron_kf_get_state(&kf, x_hat) == RON_FAULT_NONE,
                     "Kalman get_state completes without heap");
    __CPROVER_assert(ron_kf_reset(&kf) == RON_FAULT_NONE, "Kalman reset completes without heap");
}
