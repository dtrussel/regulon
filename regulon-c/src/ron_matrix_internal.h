/*
 * @file     ron_matrix_internal.h
 * @brief    Internal bounded fixed-size matrix / vector helpers (no allocation).
 * @module   ron_matrix
 * @doc      RON-IS-001
 * @req      RON-FR-602, RON-FR-603, RON-FR-700, RON-FR-720, RON-SR-003
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 *
 * Shared dense linear-algebra primitives for the Kalman filter
 * (ron_kalman), the state-space controller (ron_statespace), and the
 * Luenberger observer (ron_observer).  Every routine operates on
 * caller-provided buffers of the uniform compile-time stride
 * RON_MAT_MAX_DIM: no dynamic allocation, no recursion, no VLAs, and no
 * <math.h> dependency.
 *
 * This is a PRIVATE header.  It lives under src/ and is never installed as
 * part of the public API; only the library's own translation units include
 * it.  Read-only matrix operands are passed as non-const ron_mat_t because
 * C11 in -pedantic mode rejects the ron_float_t(*)[N] -> const
 * ron_float_t(*)[N] qualifier conversion; read-only intent is preserved by
 * convention.
 */

#ifndef RON_MATRIX_INTERNAL_H
#define RON_MATRIX_INTERNAL_H

#include "ron/ron_platform.h"

/** Uniform working stride for every scratch matrix / vector. */
typedef ron_float_t ron_mat_t[RON_MAT_MAX_DIM][RON_MAT_MAX_DIM];
typedef ron_float_t ron_vec_t[RON_MAT_MAX_DIM];

/**
 * @brief Load a strided source block into a uniform-stride scratch matrix.
 *
 * Copies the leading rows x cols block of `src` (whose physical row stride is
 * `src_cols`) into `dst`.
 *
 * Satisfies: RON-FR-607 | Test: RON-TC-KF-008, RON-TC-SS-001
 */
void ron_mat_load(ron_mat_t dst, const ron_float_t *src, uint8_t src_cols, uint8_t rows,
                  uint8_t cols);

/**
 * @brief Store a uniform-stride scratch matrix back into a strided destination.
 *
 * Satisfies: RON-FR-607 | Test: RON-TC-KF-008
 */
void ron_mat_store(ron_float_t *dst, uint8_t dst_cols, ron_mat_t src, uint8_t rows, uint8_t cols);

/**
 * @brief out(lhs_rows x rhs_cols) = lhs(lhs_rows x inner) * rhs(inner x rhs_cols).
 *
 * Satisfies: RON-FR-602 | Test: RON-TC-KF-003
 */
void ron_mat_mul(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                 uint8_t rhs_cols);

/**
 * @brief out(lhs_rows x rhs_rows) = lhs(lhs_rows x inner) * rhs(rhs_rows x inner)^T.
 *
 * Satisfies: RON-FR-602, RON-FR-604 | Test: RON-TC-KF-003, RON-TC-KF-005
 */
void ron_mat_mul_t(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                   uint8_t rhs_rows);

/**
 * @brief out(lhs_cols x rhs_cols) = lhs(inner x lhs_cols)^T * rhs(inner x rhs_cols).
 *
 * The transposed-left counterpart of ron_mat_mul_t().  Its purpose is to let
 * callers form A^T B without materialising A^T, which would cost a whole
 * scratch matrix of stack on a path where stack is the scarce resource.
 *
 * Satisfies: RON-FR-602, RON-FR-733 | Test: RON-TC-KF-003, RON-TC-LQR-003
 */
void ron_mat_mul_ta(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_cols, uint8_t inner,
                    uint8_t rhs_cols);

/**
 * @brief out(rows x cols) = lhs + rhs (element-wise).
 *
 * `out` may alias `lhs` or `rhs`: each element is written only after both of
 * its operands have been read.  The multiplying operations above give no such
 * guarantee — they write `out` while still reading their operands for later
 * elements, so their destination must be distinct.
 *
 * Satisfies: RON-FR-602 | Test: RON-TC-KF-003
 */
void ron_mat_add(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t rows, uint8_t cols);

/**
 * @brief out(rows) = mat(rows x cols) * vec(cols).
 *
 * Satisfies: RON-FR-602, RON-FR-700, RON-FR-720 | Test: RON-TC-KF-003, RON-TC-SS-001, RON-TC-SS-006
 */
void ron_mat_vec(ron_vec_t out, ron_mat_t mat, const ron_float_t *vec, uint8_t rows, uint8_t cols);

/**
 * @brief Returns true iff all `count` entries of `vec` are finite.
 *
 * Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-SS-007
 */
bool ron_mat_vec_finite(const ron_float_t *vec, uint8_t count);

/**
 * @brief Returns true iff the leading rows x cols block of a strided source is finite.
 *
 * Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-SS-007
 */
bool ron_mat_strided_finite(const ron_float_t *src, uint8_t src_cols, uint8_t rows, uint8_t cols);

/**
 * @brief Fixed-iteration Newton square root.  Precondition: value > 0.
 *
 * Satisfies: RON-FR-603 | Test: RON-TC-KF-004
 */
ron_float_t ron_mat_sqrt(ron_float_t value);

/**
 * @brief In-place Cholesky factorisation.
 *
 * On success the lower triangle of `mat` holds L with mat == L * L^T.  Returns
 * false if `mat` is not numerically positive definite.
 *
 * Satisfies: RON-FR-603 | Test: RON-TC-KF-004
 */
bool ron_mat_cholesky(ron_mat_t mat, uint8_t dim);

/**
 * @brief Solve L * L^T * x = rhs in place on `x` (`x` holds rhs on entry).
 *
 * Satisfies: RON-FR-603 | Test: RON-TC-KF-004
 */
void ron_mat_chol_solve(ron_mat_t lower, ron_float_t *x, uint8_t dim);

#endif /* RON_MATRIX_INTERNAL_H */
