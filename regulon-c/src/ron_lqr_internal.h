/*
 * @file     ron_lqr_internal.h
 * @brief    Private discrete algebraic Riccati equation (DARE) solver shared
 *           by ron_lqr and ron_lqg.
 * @module   ron_lqr
 * @doc      RON-IS-001
 * @req      RON-FR-731, RON-FR-733, RON-FR-739, RON-FR-756
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * This is a PRIVATE header.  It lives under src/ and is never installed as
 * part of the public API.  ron_lqr.c defines ron_lqr_dare_solve(); ron_lqg.c
 * includes this header to reuse it for the LQR half of the combined LQG
 * gain (RON-FR-756, separation principle).
 */

#ifndef RON_LQR_INTERNAL_H
#define RON_LQR_INTERNAL_H

#include "ron/ron_lqr.h"

/**
 * @brief Solve the discrete algebraic Riccati equation via iterative value
 * recursion (SADS DD-19 — no Schur decomposition) and return the optimal
 * state-feedback gain K and the DARE solution P.
 *
 * All matrix/vector arguments are raw pointers using the same
 * (pointer, stride) convention as ron_mat_load()/ron_mat_store():
 *   - a, q, p_out: n x n, row stride RON_LQR_MAX_STATES.
 *   - b:           n x m, row stride RON_LQR_MAX_INPUTS.
 *   - r:           m x m, row stride RON_LQR_MAX_INPUTS.
 *   - k_out:       m x n, row stride RON_LQR_MAX_STATES.
 *
 * max_iter == 0 selects the default iteration limit (200).  Returns
 * RON_FAULT_NONE with K/P populated on convergence; returns
 * RON_FAULT_CONFIG_INVALID (leaving k_out/p_out untouched) if R + B^T P B is
 * not positive definite at any iteration, or if convergence is not reached
 * within the iteration limit (RON-FR-733).
 *
 * Satisfies: RON-FR-731, RON-FR-733, RON-FR-739, RON-FR-756 | Test: RON-TC-LQR-003, RON-TC-LQG-006
 */
ron_fault_t ron_lqr_dare_solve(const ron_float_t *a, const ron_float_t *b, const ron_float_t *q,
                               const ron_float_t *r, uint8_t n, uint8_t m, uint16_t max_iter,
                               ron_float_t tol, ron_float_t *k_out, ron_float_t *p_out);

#endif /* RON_LQR_INTERNAL_H */
