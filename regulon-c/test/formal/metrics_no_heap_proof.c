/*
 * @file     metrics_no_heap_proof.c
 * @brief    CBMC harness for metrics-accumulator passivity bounds and no-heap safety.
 * @module   metrics_no_heap_proof
 * @doc      RON-TP-001
 * @req      RON-FR-951, RON-FR-953, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves that one metrics step over any finite, bounded (r, y, dt) completes
 * without fault, never decreases the monotone error integrals (the accumulator
 * only ever adds non-negative contributions — RON-FR-951), and performs no
 * dynamic allocation (RON-SR-003).  Together with the read-only signature these
 * establish the passive-observer property.
 */

#include "ron/ron_metrics.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define METRICS_BOUND RON_FLOAT_C(1000.0)
#define METRICS_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-953, RON-SR-003 | Test: RON-TC-MET-001-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "metrics accumulator must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-953, RON-SR-003 | Test: RON-TC-MET-001-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "metrics accumulator must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-953, RON-SR-003 | Test: RON-TC-MET-001-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "metrics accumulator must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-953, RON-SR-003 | Test: RON-TC-MET-001-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "metrics accumulator must not call free");
}

/* Satisfies: RON-FR-951, RON-FR-953, RON-SR-003 | Test: RON-TC-MET-001-FV */
void metrics_no_heap_proof(void)
{
    ron_metrics_t m;
    ron_metrics_config_t cfg;
    ron_float_t iae_before;
    ron_float_t ise_before;
    ron_float_t r  = nondet_ron_float_t();
    ron_float_t y  = nondet_ron_float_t();
    ron_float_t dt = nondet_ron_float_t();

    __CPROVER_assume(RON_ISFINITE(r) && (r >= -METRICS_BOUND) && (r <= METRICS_BOUND));
    __CPROVER_assume(RON_ISFINITE(y) && (y >= -METRICS_BOUND) && (y <= METRICS_BOUND));
    __CPROVER_assume(RON_ISFINITE(dt) && (dt > RON_FLOAT_C(0.0)) && (dt <= METRICS_DT_MAX));

    cfg.mode           = RON_METRICS_CUMULATIVE;
    cfg.window_steps   = 0U;
    cfg.band_pct       = RON_FLOAT_C(0.02);
    cfg.settle_confirm = RON_FLOAT_C(0.1);
    cfg.step_thresh    = RON_FLOAT_C(0.5);

    __CPROVER_assert(ron_metrics_init(&m, &cfg) == RON_FAULT_NONE, "valid config initialises");
    __CPROVER_assert(ron_metrics_enable(&m, true) == RON_FAULT_NONE, "enable succeeds");

    iae_before = m.iae;
    ise_before = m.ise;
    __CPROVER_assert(ron_metrics_step(&m, r, y, dt) == RON_FAULT_NONE,
                     "bounded finite metrics step does not fault");

    /* The error integrals accumulate non-negative contributions only. */
    __CPROVER_assert(m.iae >= iae_before, "IAE is monotone non-decreasing");
    __CPROVER_assert(m.ise >= ise_before, "ISE is monotone non-decreasing");
}
