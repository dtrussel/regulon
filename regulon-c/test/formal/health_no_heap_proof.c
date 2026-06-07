/*
 * @file     health_no_heap_proof.c
 * @brief    CBMC harness for health-monitor passivity bounds and no-heap safety.
 * @module   health_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-FR-903, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves that one health-monitor step over any finite, bounded (r, y, u, dt)
 * completes without fault, never widens the latched status (the monitor only
 * ever sets bits — RON-FR-905), and performs no dynamic allocation
 * (RON-SR-003).  Together with the read-only signature these establish the
 * passive-observer property (RON-FR-903).
 */

#include "ron/ron_health.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define HEALTH_BOUND RON_FLOAT_C(1000.0)
#define HEALTH_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-903, RON-SR-003 | Test: RON-TC-HLTH-008-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "health monitor must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-903, RON-SR-003 | Test: RON-TC-HLTH-008-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "health monitor must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-903, RON-SR-003 | Test: RON-TC-HLTH-008-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "health monitor must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-903, RON-SR-003 | Test: RON-TC-HLTH-008-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "health monitor must not call free");
}

/* Satisfies: RON-FR-903, RON-SR-003 | Test: RON-TC-HLTH-008-FV */
void health_no_heap_proof(void)
{
    ron_health_t h;
    ron_health_config_t cfg;
    ron_health_status_t before;
    ron_health_status_t after;
    ron_float_t r  = nondet_ron_float_t();
    ron_float_t y  = nondet_ron_float_t();
    ron_float_t u  = nondet_ron_float_t();
    ron_float_t dt = nondet_ron_float_t();

    __CPROVER_assume(RON_ISFINITE(r) && (r >= -HEALTH_BOUND) && (r <= HEALTH_BOUND));
    __CPROVER_assume(RON_ISFINITE(y) && (y >= -HEALTH_BOUND) && (y <= HEALTH_BOUND));
    __CPROVER_assume(RON_ISFINITE(u) && (u >= -HEALTH_BOUND) && (u <= HEALTH_BOUND));
    __CPROVER_assume(RON_ISFINITE(dt) && (dt > RON_FLOAT_C(0.0)) && (dt <= HEALTH_DT_MAX));

    cfg.t_sat_max          = RON_FLOAT_C(0.5);
    cfg.err_diverge_thresh = RON_FLOAT_C(1.0);
    cfg.osc_count_thresh   = (uint8_t) 4U;
    cfg.dead_band          = RON_FLOAT_C(0.01);
    cfg.dropout_time       = RON_FLOAT_C(0.5);
    cfg.ss_err_thresh      = RON_FLOAT_C(0.5);
    cfg.settling_time      = RON_FLOAT_C(0.5);
    cfg.cb                 = (ron_health_cb_t) 0;

    __CPROVER_assert(ron_health_init(&h, &cfg) == RON_FAULT_NONE, "valid config initialises");

    before = h.state.status;
    __CPROVER_assert(ron_health_step(&h, r, y, u, dt) == RON_FAULT_NONE,
                     "bounded finite health step does not fault");
    after = h.state.status;

    /* Status only ever gains bits: a passive, monotonic latch. */
    __CPROVER_assert((after & before) == before, "health status only latches, never clears");
}
