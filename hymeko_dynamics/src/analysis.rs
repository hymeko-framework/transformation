//! Modal analysis: what the declared system will do, without running it.
//!
//! The state matrix is `A = (J - R) Q`. Its eigenvalues are the natural frequencies and damping
//! rates; its resolvent at `i omega` is the frequency response. Both are linear algebra over the
//! same assembled structure the time stepping uses, and that is the point:
//!
//! **The analysis predicts and the simulation confirms, over one declaration.** Two entirely
//! different code paths — a resolvent solve and a discrete-gradient trajectory — must agree. If the
//! port-Hamiltonian assembly were wrong they would disagree, because nothing in either path could
//! compensate for a mistake in the other.
//!
//! # Why the eigenvalues are found this way
//!
//! Leverrier-Faddeev for the characteristic polynomial, Durand-Kerner for its roots. That is the
//! right choice for a handful of coordinates and the wrong one for a hundred: Leverrier-Faddeev is
//! `O(n^4)` and loses conditioning as `n` grows, and a real implementation for large systems wants a
//! Hessenberg reduction and a Francis QR. Stated rather than glossed — and the two identities
//! `sum(lambda) = tr(A)` and `prod(lambda) = det(A)` are checked so a bad root set is caught rather
//! than reported.

use crate::system::{Readout, System};

/// A complex number. Written out rather than depended on: `hymeko_numerics` is dependency-free by
/// contract and this crate has no reason to be the one that adds a graph.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub const ZERO: Self = Self { re: 0.0, im: 0.0 };
    pub const ONE: Self = Self { re: 1.0, im: 0.0 };

    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    pub fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
    pub fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
    pub fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
    pub fn div(self, o: Self) -> Self {
        let d = o.re * o.re + o.im * o.im;
        Self::new(
            (self.re * o.re + self.im * o.im) / d,
            (self.im * o.re - self.re * o.im) / d,
        )
    }
    pub fn scale(self, k: f64) -> Self {
        Self::new(self.re * k, self.im * k)
    }
    pub fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }
}

/// One natural mode of the system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mode {
    /// `omega / 2 pi`, in hertz.
    pub frequency_hz: f64,
    /// `-Re(lambda)`, in reciprocal seconds. Positive is decay.
    pub damping_per_s: f64,
    /// `omega / (2 |Re lambda|)`; infinite for a lossless mode.
    pub quality: f64,
}

/// Why an analysis could not be produced.
#[derive(Debug, Clone, PartialEq)]
pub enum AnalysisError {
    /// The root finder did not reproduce the trace and determinant of the matrix it was given.
    Untrustworthy { trace: f64, determinant: f64 },
    /// A frequency response was asked for at a readout that is not a linear functional of the state.
    NotLinear { observable: String },
    /// The resolvent is singular: the drive frequency sits exactly on an undamped pole.
    Resonant { frequency_hz: f64 },
}

/// Every eigenvalue of a small dense matrix.
///
/// Callers pass the energy-normalised form `S = Q^(1/2) (J - R) Q^(1/2)` rather than `A = (J - R) Q`:
/// the two are similar and so have identical eigenvalues, but `S` is scaled to the frequencies it
/// contains and `A` is not. See [`System::normalised_matrix`].
pub fn eigenvalues(a: &[f64], n: usize) -> Result<Vec<Complex>, AnalysisError> {
    // Leverrier-Faddeev: `p(x) = x^n - c_1 x^(n-1) - ... - c_n`.
    let mut m = a.to_vec();
    let mut c = vec![0.0; n + 1];
    for k in 1..=n {
        c[k] = trace(&m, n) / k as f64;
        if k < n {
            // `M <- A (M - c_k I)`
            let mut shifted = m.clone();
            for i in 0..n {
                shifted[i * n + i] -= c[k];
            }
            m = multiply(a, &shifted, n);
        }
    }

    // Monic coefficients, highest degree first: `[1, -c_1, -c_2, ..., -c_n]`.
    let mut coefficients = vec![Complex::ONE];
    for k in 1..=n {
        coefficients.push(Complex::new(-c[k], 0.0));
    }
    // Factor exact zero roots out of the computed polynomial: p(x) = x^z q(x).
    // Iterating those roots can leave tiny imaginary residues that look like
    // oscillations. Never round a nonzero coefficient to zero here.
    let mut degree = n;
    while degree > 0 && coefficients[degree] == Complex::ZERO {
        degree -= 1;
    }
    let mut roots = if degree == 0 {
        Vec::new()
    } else {
        durand_kerner(&coefficients[..=degree], degree)
    };
    if degree < n {
        roots.resize(n, Complex::ZERO);
        roots.sort_by(|a, b| a.re.total_cmp(&b.re).then(a.im.total_cmp(&b.im)));
    }

    // The two identities that catch a bad root set. `sum(lambda) = tr(A)` and
    // `prod(lambda) = det(A)`, and `det(A) = (-1)^n * p(0) = c_n` for this normalisation.
    let sum = roots.iter().fold(Complex::ZERO, |acc, r| acc.add(*r));
    let product = roots.iter().fold(Complex::ONE, |acc, r| acc.mul(*r));
    let want_trace = trace(a, n);
    // `p(lambda) = det(lambda I - A) = lambda^n - c_1 lambda^(n-1) - ... - c_n`, so at `lambda = 0`
    // it reads `det(-A) = (-1)^n det(A) = -c_n`, giving `det(A) = (-1)^(n+1) c_n`. For an even
    // dimension that is `-c_n`, and getting it backwards makes the check fail by exactly a factor of
    // two on a correct root set — which is how this was found.
    let want_determinant = if n % 2 == 0 { -c[n] } else { c[n] };
    let scale = |v: f64| v.abs().max(1.0);
    let trace_error =
        (sum.re - want_trace).abs() / scale(want_trace) + sum.im.abs() / scale(want_trace);
    let determinant_error = (product.re - want_determinant).abs() / scale(want_determinant)
        + product.im.abs() / scale(want_determinant);
    if trace_error > 1e-6 || determinant_error > 1e-6 {
        return Err(AnalysisError::Untrustworthy {
            trace: trace_error,
            determinant: determinant_error,
        });
    }
    Ok(roots)
}

/// The natural modes, one per conjugate pair, ordered by frequency.
///
/// A purely real eigenvalue is not a mode — it is a relaxation, with no frequency to report — and is
/// left out rather than reported at zero hertz.
///
/// "Purely real" has to be a tolerance, not a test against zero. A root finder returns a relaxation
/// with an imaginary part of a few times `1e-9` rather than exactly nothing, and `im > 0.0` lets it
/// through as a mode at 0.000 Hz with a quality factor of zero — which is how this was found.
pub fn modes(system: &System) -> Result<Vec<Mode>, AnalysisError> {
    let n = system.dim();
    let s = system.normalised_matrix();
    let mut modes: Vec<Mode> = eigenvalues(&s, n)?
        .into_iter()
        .filter(|l| l.im > 1e-8 * (l.re.abs() + l.im))
        .map(|l| Mode {
            frequency_hz: l.im / std::f64::consts::TAU,
            damping_per_s: -l.re,
            quality: if l.re == 0.0 {
                f64::INFINITY
            } else {
                l.im / (2.0 * l.re.abs())
            },
        })
        .collect();
    modes.sort_by(|a, b| a.frequency_hz.total_cmp(&b.frequency_hz));
    Ok(modes)
}

/// The steady-state response of one observable to one drive, at an angular frequency.
///
/// For `x' = A x + b cos(omega t)` the steady state is `Re(X e^(i omega t))` with
/// `(i omega I - A) X = b`, so the observable's amplitude is `|c . X|` and its phase is `arg(c . X)`.
/// This is what the transient run must settle to, and comparing the two is what checks the assembly.
pub fn response(
    system: &System,
    drive: usize,
    observable: usize,
    omega: f64,
) -> Result<Complex, AnalysisError> {
    let readout = system.observables[observable].readout;
    if !readout.is_linear() {
        return Err(AnalysisError::NotLinear {
            observable: system.observables[observable].name.clone(),
        });
    }
    let n = system.dim();
    // Solved in the energy-normalised coordinates `z = Q^(1/2) x`, where the system is
    // `z' = S z + Q^(1/2) u` — the same reason `modes` uses them, and the answer is converted back
    // rather than the problem being posed badly.
    let s = system.normalised_matrix();
    let root: Vec<f64> = system
        .coordinates
        .iter()
        .map(|c| (1.0 / c.capacity).sqrt())
        .collect();

    let mut matrix = vec![Complex::ZERO; n * n];
    for row in 0..n {
        for column in 0..n {
            matrix[row * n + column] = Complex::new(-s[row * n + column], 0.0);
        }
        matrix[row * n + row] = matrix[row * n + row].add(Complex::new(0.0, omega));
    }
    let b: Vec<Complex> = system
        .drive_direction(drive)
        .into_iter()
        .enumerate()
        .map(|(k, v)| Complex::new(root[k] * v * system.drives[drive].amplitude, 0.0))
        .collect();
    let z = solve(matrix, b, n).ok_or(AnalysisError::Resonant {
        frequency_hz: omega / std::f64::consts::TAU,
    })?;
    // `x = Q^(-1/2) z`
    let x: Vec<Complex> = z
        .into_iter()
        .enumerate()
        .map(|(k, v)| v.scale(1.0 / root[k]))
        .collect();

    Ok(match readout {
        Readout::Effort { coordinate } => {
            x[coordinate].scale(1.0 / system.coordinates[coordinate].capacity)
        }
        Readout::Store { coordinate } => x[coordinate],
        _ => unreachable!("guarded by is_linear"),
    })
}

// ─── small dense linear algebra ─────────────────────────────────────────────────────────────────

fn trace(m: &[f64], n: usize) -> f64 {
    (0..n).map(|i| m[i * n + i]).sum()
}

fn multiply(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n * n];
    for row in 0..n {
        for k in 0..n {
            let left = a[row * n + k];
            if left == 0.0 {
                continue;
            }
            for column in 0..n {
                out[row * n + column] += left * b[k * n + column];
            }
        }
    }
    out
}

/// Durand-Kerner: every root at once, from a spread of starting points on a circle.
///
/// The starting points are a fixed spiral rather than a random draw, so the same polynomial gives
/// the same roots in the same order on every machine and there is no seed to record.
fn durand_kerner(coefficients: &[Complex], n: usize) -> Vec<Complex> {
    // Scale the variable so the roots sit near the unit circle before iterating.
    //
    // Without this the starting points are placed by a bound on the COEFFICIENTS, and for a
    // physical system those are the roots raised to the degree: a quartic with roots near `5e4` has
    // a constant term near `6e18`, and starting the iteration eighteen decades from the answer is
    // how a correct root finder returns nonsense. Substituting `lambda = rho mu` with
    // `rho = max_k |a_k|^(1/k)` puts every root within a small factor of one.
    let n_f = n as f64;
    let _ = n_f;
    let rho = coefficients[1..]
        .iter()
        .enumerate()
        .map(|(i, c)| c.abs().powf(1.0 / (i + 1) as f64))
        .fold(0.0, f64::max)
        .max(f64::MIN_POSITIVE);
    let scaled: Vec<Complex> = coefficients
        .iter()
        .enumerate()
        .map(|(i, c)| c.scale(rho.powi(-(i as i32))))
        .collect();
    let coefficients = &scaled[..];

    let evaluate = |z: Complex| {
        coefficients
            .iter()
            .fold(Complex::ZERO, |acc, c| acc.mul(z).add(*c))
    };
    // A radius that brackets the roots: Cauchy's bound on the scaled monic polynomial.
    let radius = 1.0
        + coefficients[1..]
            .iter()
            .map(|c| c.abs())
            .fold(0.0, f64::max);
    let seed = Complex::new(0.4, 0.9);
    let mut roots: Vec<Complex> = (0..n)
        .map(|k| {
            let mut z = Complex::ONE;
            for _ in 0..k {
                z = z.mul(seed);
            }
            z.scale(radius / z.abs().max(f64::MIN_POSITIVE))
        })
        .collect();

    for _ in 0..2000 {
        let mut moved: f64 = 0.0;
        for k in 0..n {
            let mut denominator = Complex::ONE;
            for j in 0..n {
                if j != k {
                    denominator = denominator.mul(roots[k].sub(roots[j]));
                }
            }
            if denominator.abs() < 1e-300 {
                continue;
            }
            let step = evaluate(roots[k]).div(denominator);
            roots[k] = roots[k].sub(step);
            moved = moved.max(step.abs());
        }
        if moved < 1e-14 * radius {
            break;
        }
    }
    // Undo the substitution, then a deterministic order so two runs report the same roots in the
    // same places.
    for root in &mut roots {
        *root = root.scale(rho);
    }
    roots.sort_by(|a, b| a.re.total_cmp(&b.re).then(a.im.total_cmp(&b.im)));
    roots
}

/// Complex Gaussian elimination with partial pivoting.
fn solve(mut a: Vec<Complex>, mut b: Vec<Complex>, n: usize) -> Option<Vec<Complex>> {
    for column in 0..n {
        let mut best = column;
        for row in column + 1..n {
            if a[row * n + column].abs() > a[best * n + column].abs() {
                best = row;
            }
        }
        if a[best * n + column].abs() < 1e-300 {
            return None;
        }
        if best != column {
            for c in 0..n {
                a.swap(column * n + c, best * n + c);
            }
            b.swap(column, best);
        }
        for row in column + 1..n {
            let factor = a[row * n + column].div(a[column * n + column]);
            for c in column..n {
                let take = a[column * n + c].mul(factor);
                a[row * n + c] = a[row * n + c].sub(take);
            }
            let take = b[column].mul(factor);
            b[row] = b[row].sub(take);
        }
    }
    let mut x = vec![Complex::ZERO; n];
    for column in (0..n).rev() {
        let mut acc = b[column];
        for c in column + 1..n {
            acc = acc.sub(a[column * n + c].mul(x[c]));
        }
        x[column] = acc.div(a[column * n + column]);
    }
    x.iter()
        .all(|v| v.re.is_finite() && v.im.is_finite())
        .then_some(x)
}
