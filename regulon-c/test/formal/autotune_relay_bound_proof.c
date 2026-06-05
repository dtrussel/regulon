/*
 * @file     autotune_relay_bound_proof.c
 * @brief    CBMC harness for relay-output bounds and no-heap safety.
 * @module   autotune_relay_bound_proof
 * @doc      RON-TP-001
 * @req      RON-FR-806, RON-SR-003, RON-PR-022
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Proves that one relay-feedback step keeps the control output within
 * [u_bias - d, u_bias + d] for any finite, bounded setpoint / measurement
 * (RON-FR-806), and that the auto-tuner path performs no dynamic allocation
 * (RON-SR-003).
 */

#include "ron/ron_autotune.h"

extern void __CPROVER_assume(int condition);
extern void __CPROVER_assert(int condition, const char *description);
extern ron_float_t nondet_ron_float_t(void);

#define AT_BOUND RON_FLOAT_C(1000.0)
#define AT_DT_MAX RON_FLOAT_C(1.0)

/* Satisfies: RON-FR-806, RON-SR-003 | Test: RON-TC-AT-007-FV */
void *malloc(size_t size)
{
    (void) size;
    __CPROVER_assert(0, "auto-tuner must not call malloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-806, RON-SR-003 | Test: RON-TC-AT-007-FV */
void *calloc(size_t nmemb, size_t size)
{
    (void) nmemb;
    (void) size;
    __CPROVER_assert(0, "auto-tuner must not call calloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-806, RON-SR-003 | Test: RON-TC-AT-007-FV */
void *realloc(void *ptr, size_t size)
{
    (void) ptr;
    (void) size;
    __CPROVER_assert(0, "auto-tuner must not call realloc");
    return (void *) 0;
}

/* Satisfies: RON-FR-806, RON-SR-003 | Test: RON-TC-AT-007-FV */
void free(void *ptr)
{
    (void) ptr;
    __CPROVER_assert(0, "auto-tuner must not call free");
}

/* Satisfies: RON-FR-806, RON-SR-003 | Test: RON-TC-AT-007-FV */
void autotune_relay_bound_proof(void)
{
    ron_at_t at;
    ron_at_config_t cfg;
    ron_float_t d    = nondet_ron_float_t();
    ron_float_t eps  = nondet_ron_float_t();
    ron_float_t bias = nondet_ron_float_t();
    ron_float_t r    = nondet_ron_float_t();
    ron_float_t y    = nondet_ron_float_t();
    ron_float_t dt   = nondet_ron_float_t();
    ron_float_t u    = RON_FLOAT_C(0.0);

    __CPROVER_assume(RON_ISFINITE(d) && (d > RON_FLOAT_C(0.0)) && (d <= AT_BOUND));
    __CPROVER_assume(RON_ISFINITE(eps) && (eps >= RON_FLOAT_C(0.0)) && (eps <= AT_BOUND));
    __CPROVER_assume(RON_ISFINITE(bias) && (bias >= -AT_BOUND) && (bias <= AT_BOUND));
    __CPROVER_assume(RON_ISFINITE(r) && (r >= -AT_BOUND) && (r <= AT_BOUND));
    __CPROVER_assume(RON_ISFINITE(y) && (y >= -AT_BOUND) && (y <= AT_BOUND));
    __CPROVER_assume(RON_ISFINITE(dt) && (dt > RON_FLOAT_C(0.0)) && (dt <= AT_DT_MAX));

    cfg.relay_amplitude = d;
    cfg.hysteresis      = eps;
    cfg.u_bias          = bias;
    cfg.min_cycles      = 5U;
    cfg.timeout_s       = AT_BOUND;
    cfg.tuning_rule     = RON_AT_RULE_ZN;

    __CPROVER_assert(ron_autotune_init(&at, &cfg) == RON_FAULT_NONE, "valid config initialises");

    /* Place the run in the relay phase with a primed (in-bounds) output. */
    at.state.phase        = (uint8_t) RON_AT_RELAY;
    at.state.u_relay_prev = bias + d;

    __CPROVER_assert(ron_autotune_step(&at, r, y, dt, &u) == RON_FAULT_NONE,
                     "bounded finite relay step does not fault");
    __CPROVER_assert((u >= (bias - d)) && (u <= (bias + d)),
                     "relay output stays within [u_bias - d, u_bias + d]");
}
