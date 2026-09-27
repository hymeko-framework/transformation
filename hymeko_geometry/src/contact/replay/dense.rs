//! Dense f64 elimination with partial pivoting shared by the replay experiments and the DH
//! inverse kinematics (`kinematics::ik`).
//!
//! One implementation serves the local 3×3 Alart-Curnier Newton direction and the reduced
//! active-set solve. The operation order is fixed, so the local Newton directions are
//! bit-identical to the former fixed-size elimination.

/// Solve `matrix x = rhs` in place for a row-major `n`×`n` matrix; `rhs` then holds `x`.
///
/// # Preconditions
/// `matrix.len() == n * n` and `rhs.len() == n`.
///
/// # Errors
/// `None` for a nonfinite input or result, or a pivot of at most `64 ε max|a_ij|`.
pub(crate) fn solve_in_place(matrix: &mut [f64], rhs: &mut [f64], n: usize) -> Option<()> {
    debug_assert!(matrix.len() == n * n && rhs.len() == n);
    if matrix.iter().chain(rhs.iter()).any(|v| !v.is_finite()) {
        return None;
    }
    let scale = matrix.iter().fold(0.0f64, |a, x| a.max(x.abs()));
    for k in 0..n {
        let pivot =
            (k..n).max_by(|a, b| matrix[a * n + k].abs().total_cmp(&matrix[b * n + k].abs()))?;
        if matrix[pivot * n + k].abs() <= 64.0 * f64::EPSILON * scale {
            return None;
        }
        if pivot != k {
            for column in 0..n {
                matrix.swap(k * n + column, pivot * n + column);
            }
            rhs.swap(k, pivot);
        }
        for i in k + 1..n {
            let ratio = matrix[i * n + k] / matrix[k * n + k];
            for column in k..n {
                matrix[i * n + column] -= ratio * matrix[k * n + column];
            }
            rhs[i] -= ratio * rhs[k];
        }
    }
    for i in (0..n).rev() {
        rhs[i] = (rhs[i]
            - (i + 1..n)
                .map(|column| matrix[i * n + column] * rhs[column])
                .sum::<f64>())
            / matrix[i * n + i];
    }
    rhs.iter().all(|x| x.is_finite()).then_some(())
}
