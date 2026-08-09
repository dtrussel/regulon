/*
 * @file     ron_matrix.c
 * @brief    Internal bounded fixed-size matrix / vector helpers (no allocation).
 * @module   ron_matrix
 * @doc      RON-IS-001
 * @req      RON-FR-602, RON-FR-603, RON-FR-700, RON-FR-720, RON-SR-003
 * @version  1.0.0
 * SPDX-License-Identifier: MIT
 */

#include "ron_matrix_internal.h"

#define RON_MAT_SQRT_STEPS (30U)

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008, RON-TC-SS-001 */
void ron_mat_load(ron_mat_t dst, const ron_float_t *src, uint8_t src_cols, uint8_t rows,
                  uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            dst[i][j] = src[((size_t) i * (size_t) src_cols) + (size_t) j];
        }
    }
}

/* Satisfies: RON-FR-607 | Test: RON-TC-KF-008 */
void ron_mat_store(ron_float_t *dst, uint8_t dst_cols, ron_mat_t src, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            dst[((size_t) i * (size_t) dst_cols) + (size_t) j] = src[i][j];
        }
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
void ron_mat_mul(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                 uint8_t rhs_cols)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < lhs_rows; i++) {
        for (j = 0U; j < rhs_cols; j++) {
            ron_float_t sum = RON_FLOAT_C(0.0);

            for (k = 0U; k < inner; k++) {
                sum += lhs[i][k] * rhs[k][j];
            }
            out[i][j] = sum;
        }
    }
}

/* Satisfies: RON-FR-602, RON-FR-604 | Test: RON-TC-KF-003, RON-TC-KF-005 */
void ron_mat_mul_t(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_rows, uint8_t inner,
                   uint8_t rhs_rows)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < lhs_rows; i++) {
        for (j = 0U; j < rhs_rows; j++) {
            ron_float_t sum = RON_FLOAT_C(0.0);

            for (k = 0U; k < inner; k++) {
                sum += lhs[i][k] * rhs[j][k];
            }
            out[i][j] = sum;
        }
    }
}

/* Satisfies: RON-FR-602, RON-FR-733 | Test: RON-TC-KF-003, RON-TC-LQR-003 */
void ron_mat_mul_ta(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t lhs_cols, uint8_t inner,
                    uint8_t rhs_cols)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < lhs_cols; i++) {
        for (j = 0U; j < rhs_cols; j++) {
            ron_float_t sum = RON_FLOAT_C(0.0);

            for (k = 0U; k < inner; k++) {
                sum += lhs[k][i] * rhs[k][j];
            }
            out[i][j] = sum;
        }
    }
}

/* Satisfies: RON-FR-602 | Test: RON-TC-KF-003 */
void ron_mat_add(ron_mat_t out, ron_mat_t lhs, ron_mat_t rhs, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            out[i][j] = lhs[i][j] + rhs[i][j];
        }
    }
}

/* Satisfies: RON-FR-602, RON-FR-700, RON-FR-720 | Test: RON-TC-KF-003, RON-TC-SS-001, RON-TC-SS-006 */
void ron_mat_vec(ron_vec_t out, ron_mat_t mat, const ron_float_t *vec, uint8_t rows, uint8_t cols)
{
    uint8_t i;
    uint8_t k;

    for (i = 0U; i < rows; i++) {
        ron_float_t sum = RON_FLOAT_C(0.0);

        for (k = 0U; k < cols; k++) {
            sum += mat[i][k] * vec[k];
        }
        out[i] = sum;
    }
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-SS-007 */
bool ron_mat_vec_finite(const ron_float_t *vec, uint8_t count)
{
    uint8_t i;

    for (i = 0U; i < count; i++) {
        if (!RON_ISFINITE(vec[i])) {
            return false;
        }
    }

    return true;
}

/* Satisfies: RON-SR-020 | Test: RON-TC-KF-002, RON-TC-SS-007 */
bool ron_mat_strided_finite(const ron_float_t *src, uint8_t src_cols, uint8_t rows, uint8_t cols)
{
    uint8_t i;

    for (i = 0U; i < rows; i++) {
        if (!ron_mat_vec_finite(&src[(size_t) i * (size_t) src_cols], cols)) {
            return false;
        }
    }

    return true;
}

/* Fixed-iteration Newton square root.  Precondition: value > 0. */
/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
ron_float_t ron_mat_sqrt(ron_float_t value)
{
    ron_float_t x;
    uint8_t step;

    x = (value > RON_FLOAT_C(1.0)) ? value : RON_FLOAT_C(1.0);
    for (step = 0U; step < RON_MAT_SQRT_STEPS; step++) {
        x = RON_FLOAT_C(0.5) * (x + (value / x));
    }

    return x;
}

/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
bool ron_mat_cholesky(ron_mat_t mat, uint8_t dim)
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < dim; i++) {
        for (j = 0U; j <= i; j++) {
            ron_float_t sum = mat[i][j];

            for (k = 0U; k < j; k++) {
                sum -= mat[i][k] * mat[j][k];
            }
            if (i == j) {
                if (sum <= RON_FLOAT_C(0.0)) {
                    return false;
                }
                mat[i][j] = ron_mat_sqrt(sum);
            } else {
                mat[i][j] = sum / mat[j][j];
            }
        }
    }

    return true;
}

/* Satisfies: RON-FR-603 | Test: RON-TC-KF-004 */
void ron_mat_chol_solve(ron_mat_t lower, ron_float_t *x, uint8_t dim)
{
    uint8_t i;
    uint8_t k;

    /* Forward substitution: solve L y = rhs. */
    for (i = 0U; i < dim; i++) {
        ron_float_t sum = x[i];

        for (k = 0U; k < i; k++) {
            sum -= lower[i][k] * x[k];
        }
        x[i] = sum / lower[i][i];
    }

    /* Back substitution: solve L^T x = y. */
    for (i = 0U; i < dim; i++) {
        uint8_t row     = (uint8_t) ((dim - 1U) - i);
        ron_float_t sum = x[row];

        for (k = (uint8_t) (row + 1U); k < dim; k++) {
            sum -= lower[k][row] * x[k];
        }
        x[row] = sum / lower[row][row];
    }
}
