//! # `matrix`
//!
//! Fixed-size dense matrices for the estimation and state-feedback modules.
//! Dimensions are const generics, so shapes are checked by the compiler and
//! every loop is bounded at compile time; nothing allocates.
//!
//! **Document:** RON-IS-001
//! **Satisfies:** RON-FR-602, RON-FR-603, RON-FR-607, RON-FR-723, RON-SR-003
//! **Tests:** RON-TC-KF-003, RON-TC-KF-004, RON-TC-SS-009
//! **SPDX-License-Identifier:** MIT

#![deny(clippy::all, clippy::pedantic, missing_docs)]

use crate::platform::{is_finite, sqrt, RonFloat};

/// Largest matrix dimension the estimation and control modules accept. It
/// bounds the per-step cost the way `RON_MAT_MAX_DIM` does in C.
///
/// **Satisfies:** RON-FR-607, RON-FR-723
pub const MATRIX_MAX_DIM: usize = 8;

/// Dense `R x C` matrix stored row-major.
///
/// **Satisfies:** RON-FR-602, RON-FR-607
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix<const R: usize, const C: usize> {
    rows: [[RonFloat; C]; R],
}

impl<const R: usize, const C: usize> Default for Matrix<R, C> {
    fn default() -> Self {
        Self::zeros()
    }
}

impl<const R: usize, const C: usize> Matrix<R, C> {
    /// Builds a matrix from its rows.
    #[must_use]
    pub const fn new(rows: [[RonFloat; C]; R]) -> Self {
        Self { rows }
    }

    /// Returns the zero matrix.
    #[must_use]
    pub const fn zeros() -> Self {
        Self {
            rows: [[0.0; C]; R],
        }
    }

    /// Returns the rows.
    #[must_use]
    pub const fn rows(&self) -> &[[RonFloat; C]; R] {
        &self.rows
    }

    /// Returns the entry at `row`, `col`, or `None` when out of range.
    #[must_use]
    pub fn get(&self, row: usize, col: usize) -> Option<RonFloat> {
        self.rows
            .get(row)
            .and_then(|values| values.get(col))
            .copied()
    }

    /// Returns `true` when every entry is finite.
    ///
    /// **Satisfies:** RON-SR-020
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.rows.iter().all(vector_is_finite)
    }

    /// Returns the transpose.
    #[must_use]
    pub fn transpose(&self) -> Matrix<C, R> {
        let mut out = Matrix::<C, R>::zeros();
        for (i, row) in self.rows.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                out.rows[j][i] = *value;
            }
        }
        out
    }

    /// Returns `self * rhs`.
    ///
    /// **Satisfies:** RON-FR-602
    #[must_use]
    pub fn mul<const K: usize>(&self, rhs: &Matrix<C, K>) -> Matrix<R, K> {
        let mut out = Matrix::<R, K>::zeros();
        for (out_row, lhs_row) in out.rows.iter_mut().zip(self.rows.iter()) {
            for (j, cell) in out_row.iter_mut().enumerate() {
                *cell = lhs_row
                    .iter()
                    .zip(rhs.rows.iter())
                    .fold(0.0, |sum, (a, rhs_row)| sum + (a * rhs_row[j]));
            }
        }
        out
    }

    /// Returns `self * rhs^T` without forming the transpose.
    ///
    /// **Satisfies:** RON-FR-602, RON-FR-604
    #[must_use]
    pub fn mul_transpose<const K: usize>(&self, rhs: &Matrix<K, C>) -> Matrix<R, K> {
        let mut out = Matrix::<R, K>::zeros();
        for (out_row, lhs_row) in out.rows.iter_mut().zip(self.rows.iter()) {
            for (cell, rhs_row) in out_row.iter_mut().zip(rhs.rows.iter()) {
                *cell = dot(lhs_row, rhs_row);
            }
        }
        out
    }

    /// Returns `self * vector`.
    ///
    /// **Satisfies:** RON-FR-602, RON-FR-700, RON-FR-720
    #[must_use]
    pub fn mul_vec(&self, vector: &[RonFloat; C]) -> [RonFloat; R] {
        let mut out = [0.0; R];
        for (cell, row) in out.iter_mut().zip(self.rows.iter()) {
            *cell = dot(row, vector);
        }
        out
    }

    /// Returns `self + rhs`.
    #[must_use]
    pub fn add(&self, rhs: &Self) -> Self {
        self.zip_with(rhs, |a, b| a + b)
    }

    /// Returns `self - rhs`.
    #[must_use]
    pub fn sub(&self, rhs: &Self) -> Self {
        self.zip_with(rhs, |a, b| a - b)
    }

    /// Returns `self * factor`.
    #[must_use]
    pub fn scale(&self, factor: RonFloat) -> Self {
        let mut out = *self;
        for value in out.rows.iter_mut().flatten() {
            *value *= factor;
        }
        out
    }

    fn zip_with(&self, rhs: &Self, op: impl Fn(RonFloat, RonFloat) -> RonFloat) -> Self {
        let mut out = *self;
        for (out_row, rhs_row) in out.rows.iter_mut().zip(rhs.rows.iter()) {
            for (value, other) in out_row.iter_mut().zip(rhs_row.iter()) {
                *value = op(*value, *other);
            }
        }
        out
    }
}

impl<const N: usize> Matrix<N, N> {
    /// Returns the identity matrix.
    #[must_use]
    pub fn identity() -> Self {
        let mut out = Self::zeros();
        for (i, row) in out.rows.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        out
    }

    /// Returns the diagonal matrix with `diagonal` on its diagonal.
    #[must_use]
    pub fn diagonal(diagonal: [RonFloat; N]) -> Self {
        let mut out = Self::zeros();
        for (i, (row, value)) in out.rows.iter_mut().zip(diagonal.iter()).enumerate() {
            row[i] = *value;
        }
        out
    }

    /// Returns `(self + self^T) / 2`, removing round-off asymmetry.
    #[must_use]
    pub fn symmetrized(&self) -> Self {
        self.add(&self.transpose()).scale(0.5)
    }

    /// Factorises a symmetric positive-definite matrix as `L * L^T`.
    ///
    /// Returns `None` when the matrix is not numerically positive definite.
    ///
    /// **Satisfies:** RON-FR-603
    #[must_use]
    pub fn cholesky(&self) -> Option<Cholesky<N>> {
        let mut lower = Self::zeros();
        for i in 0..N {
            for j in 0..=i {
                let partial = (0..j).fold(self.rows[i][j], |sum, k| {
                    sum - (lower.rows[i][k] * lower.rows[j][k])
                });
                if i == j {
                    if !is_finite(partial) || partial <= 0.0 {
                        return None;
                    }
                    lower.rows[i][j] = sqrt(partial);
                } else {
                    lower.rows[i][j] = partial / lower.rows[j][j];
                }
            }
        }
        Some(Cholesky { lower })
    }

    /// Returns the inverse via Cholesky, for symmetric positive-definite
    /// matrices; `None` otherwise.
    ///
    /// **Satisfies:** RON-FR-603
    #[must_use]
    pub fn spd_inverse(&self) -> Option<Self> {
        let factor = self.cholesky()?;
        let identity = Self::identity();
        let mut out = Self::zeros();
        for (column, unit) in identity.rows.iter().enumerate() {
            let solved = factor.solve(unit);
            for (row, value) in out.rows.iter_mut().zip(solved.iter()) {
                row[column] = *value;
            }
        }
        Some(out)
    }
}

/// Cholesky factor `L` of a symmetric positive-definite matrix.
///
/// **Satisfies:** RON-FR-603
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cholesky<const N: usize> {
    lower: Matrix<N, N>,
}

impl<const N: usize> Cholesky<N> {
    /// Returns the lower-triangular factor.
    #[must_use]
    pub const fn lower(&self) -> &Matrix<N, N> {
        &self.lower
    }

    /// Solves `L * L^T * x = rhs` by forward and back substitution.
    ///
    /// **Satisfies:** RON-FR-603
    #[must_use]
    pub fn solve(&self, rhs: &[RonFloat; N]) -> [RonFloat; N] {
        let l = &self.lower.rows;
        let mut x = *rhs;
        for i in 0..N {
            let partial = (0..i).fold(x[i], |sum, k| sum - (l[i][k] * x[k]));
            x[i] = partial / l[i][i];
        }
        for i in (0..N).rev() {
            let partial = (i + 1..N).fold(x[i], |sum, k| sum - (l[k][i] * x[k]));
            x[i] = partial / l[i][i];
        }
        x
    }
}

/// Returns `true` when every entry of `vector` is finite.
///
/// **Satisfies:** RON-SR-020
#[must_use]
pub fn vector_is_finite<const N: usize>(vector: &[RonFloat; N]) -> bool {
    vector.iter().all(|value| is_finite(*value))
}

/// Returns `lhs + rhs` element-wise.
#[must_use]
pub fn vector_add<const N: usize>(lhs: &[RonFloat; N], rhs: &[RonFloat; N]) -> [RonFloat; N] {
    let mut out = *lhs;
    for (value, other) in out.iter_mut().zip(rhs.iter()) {
        *value += *other;
    }
    out
}

/// Returns `lhs - rhs` element-wise.
#[must_use]
pub fn vector_sub<const N: usize>(lhs: &[RonFloat; N], rhs: &[RonFloat; N]) -> [RonFloat; N] {
    let mut out = *lhs;
    for (value, other) in out.iter_mut().zip(rhs.iter()) {
        *value -= *other;
    }
    out
}

fn dot<const N: usize>(lhs: &[RonFloat; N], rhs: &[RonFloat; N]) -> RonFloat {
    lhs.iter()
        .zip(rhs.iter())
        .fold(0.0, |sum, (a, b)| sum + (a * b))
}

/// Returns `true` when every dimension is between 1 and [`MATRIX_MAX_DIM`]
/// (`allow_zero` admits 0, for an absent input vector).
pub(crate) const fn dimension_valid(dimension: usize, allow_zero: bool) -> bool {
    (allow_zero || dimension >= 1) && dimension <= MATRIX_MAX_DIM
}

#[cfg(test)]
mod tests {
    //! Unit tests for the matrix primitives.

    use super::{Matrix, MATRIX_MAX_DIM};
    use crate::RonFloat;

    fn approx(lhs: RonFloat, rhs: RonFloat) {
        assert!((lhs - rhs).abs() <= 1.0e-5, "{lhs} != {rhs}");
    }

    /// RON-TC-KF-003 | RON-FR-602
    #[test]
    fn ron_tc_kf_003_matrix_products() {
        let a = Matrix::new([[1.0, 2.0], [3.0, 4.0]]);
        let b = Matrix::new([[5.0, 6.0], [7.0, 8.0]]);
        assert_eq!(a.mul(&b), Matrix::new([[19.0, 22.0], [43.0, 50.0]]));
        assert_eq!(a.mul_transpose(&b), a.mul(&b.transpose()));
        assert_eq!(a.mul_vec(&[1.0, 1.0]), [3.0, 7.0]);
        assert_eq!(a.add(&b).sub(&b), a);
        let tall = Matrix::new([[1.0], [2.0], [3.0]]);
        assert_eq!(tall.transpose().mul(&tall), Matrix::new([[14.0]]));
        assert_eq!(Matrix::<2, 2>::identity().mul(&a), a);
    }

    /// RON-TC-KF-004 | RON-FR-603
    #[test]
    fn ron_tc_kf_004_cholesky_solve_and_inverse() {
        let spd = Matrix::new([[4.0, 2.0, 0.4], [2.0, 5.0, 1.0], [0.4, 1.0, 3.0]]);
        let factor = spd.cholesky().unwrap();
        let x = factor.solve(&[1.0, 2.0, 3.0]);
        let back = spd.mul_vec(&x);
        for (value, expected) in back.iter().zip([1.0, 2.0, 3.0]) {
            approx(*value, expected);
        }
        let product = spd.mul(&spd.spd_inverse().unwrap());
        for (i, row) in product.rows().iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                approx(*value, if i == j { 1.0 } else { 0.0 });
            }
        }
        assert!(Matrix::new([[1.0, 2.0], [2.0, 1.0]]).cholesky().is_none());
        assert!(Matrix::new([[0.0]]).cholesky().is_none());
        assert!(Matrix::new([[RonFloat::NAN]]).cholesky().is_none());
    }

    /// RON-TC-SS-009 | RON-FR-723
    #[test]
    fn ron_tc_ss_009_dimension_bounds() {
        assert!(super::dimension_valid(1, false));
        assert!(super::dimension_valid(MATRIX_MAX_DIM, false));
        assert!(!super::dimension_valid(0, false));
        assert!(super::dimension_valid(0, true));
        assert!(!super::dimension_valid(MATRIX_MAX_DIM + 1, true));
        assert!(!Matrix::new([[1.0, RonFloat::INFINITY]]).is_finite());
        assert_eq!(Matrix::new([[1.0, 2.0]]).get(0, 1), Some(2.0));
        assert_eq!(Matrix::new([[1.0, 2.0]]).get(1, 0), None);
    }
}
