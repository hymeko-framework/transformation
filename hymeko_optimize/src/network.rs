//! Modified nodal analysis over a declared branch network.
//!
//! # Why the equations are assembled and not written down
//!
//! The model this serves declares a topology — nets, and branches between them — and never a
//! formula. That is deliberate: a voltage divider written into a solver is a claim that the topology
//! is decoration, and it stops being true the moment a third resistor appears. Here the topology IS
//! the equation, and a network of any shape produces its own system.
//!
//! Modified nodal analysis is the standard way and it is the one that survives ideal sources. Plain
//! nodal analysis has no equation for a branch whose current is unknown and whose voltage is fixed;
//! MNA adds that current as an unknown and the voltage as its equation. For `n` nets and `m` sources
//! the system is `(n - 1 + m)` square.
//!
//! The whole assembly and solve runs over [`Dual`], so the sensitivity of a node potential to a
//! resistance comes out of the same arithmetic that produced the potential.

use crate::dual::Dual;

/// A two-terminal element between two nets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Branch {
    /// `I = (V_a - V_b) / R`, current flowing from `a` to `b`.
    Resistor { a: usize, b: usize, resistance: usize },
    /// `V_a - V_b = V`, with the current an unknown the system solves for.
    Source { a: usize, b: usize, voltage: usize },
}

impl Branch {
    pub fn nets(&self) -> (usize, usize) {
        match *self {
            Self::Resistor { a, b, .. } | Self::Source { a, b, .. } => (a, b),
        }
    }
}

/// What a solved network answers.
#[derive(Debug, Clone)]
pub struct Solution {
    /// Potential of each net, ground included (which is zero).
    pub potential: Vec<Dual>,
    /// Current through each branch, positive from `a` to `b`.
    pub current: Vec<Dual>,
}

/// Why a network could not be solved.
#[derive(Debug, Clone, PartialEq)]
pub enum NetworkError {
    /// Two nets are joined by nothing, or a source loop makes the system singular.
    Singular { row: usize },
    /// A branch names a net the network does not have.
    UnknownNet { branch: usize, net: usize },
    /// No net was declared as ground, so every potential is free by a constant.
    NoGround,
}

/// Assemble and solve the network at one set of element values.
///
/// `values[i]` is the resistance or source voltage the branch's third incidence named, already
/// carrying its own gradient. `ground` is the net held at zero.
///
/// # Errors
/// A singular system, an unknown net, or no ground.
pub fn solve(
    nets: usize,
    branches: &[Branch],
    values: &[Dual],
    ground: Option<usize>,
    width: usize,
) -> Result<Solution, NetworkError> {
    let ground = ground.ok_or(NetworkError::NoGround)?;
    for (i, branch) in branches.iter().enumerate() {
        let (a, b) = branch.nets();
        for net in [a, b] {
            if net >= nets {
                return Err(NetworkError::UnknownNet { branch: i, net });
            }
        }
    }

    // Unknown layout: every net but ground, then one current per source.
    let mut column = vec![usize::MAX; nets];
    let mut next = 0usize;
    for net in 0..nets {
        if net != ground {
            column[net] = next;
            next += 1;
        }
    }
    let sources: Vec<usize> = branches
        .iter()
        .enumerate()
        .filter(|(_, b)| matches!(b, Branch::Source { .. }))
        .map(|(i, _)| i)
        .collect();
    let mut source_column = vec![usize::MAX; branches.len()];
    for &i in &sources {
        source_column[i] = next;
        next += 1;
    }
    let n = next;

    let mut a = vec![Dual::zero(width); n * n];
    let mut rhs = vec![Dual::zero(width); n];
    let put = |m: &mut Vec<Dual>, r: usize, c: usize, v: Dual| {
        if r != usize::MAX && c != usize::MAX {
            m[r * n + c] = m[r * n + c].add(&v);
        }
    };

    for (i, branch) in branches.iter().enumerate() {
        match *branch {
            // KCL: a conductance between two nets adds `+g` on each diagonal and `-g` off it.
            Branch::Resistor {
                a: p,
                b: q,
                resistance,
            } => {
                let g = Dual::constant(1.0, width).div(&values[resistance]);
                put(&mut a, column[p], column[p], g.clone());
                put(&mut a, column[q], column[q], g.clone());
                put(&mut a, column[p], column[q], g.neg());
                put(&mut a, column[q], column[p], g.neg());
            }
            // A source's current leaves `a` and enters `b`; its own row states the voltage it holds.
            Branch::Source {
                a: p,
                b: q,
                voltage,
            } => {
                let k = source_column[i];
                let one = Dual::constant(1.0, width);
                put(&mut a, column[p], k, one.clone());
                put(&mut a, column[q], k, one.neg());
                put(&mut a, k, column[p], one.clone());
                put(&mut a, k, column[q], one.neg());
                rhs[k] = values[voltage].clone();
            }
        }
    }

    let x = gaussian(&mut a, &mut rhs, n, width)?;

    let mut potential = vec![Dual::zero(width); nets];
    for net in 0..nets {
        if net != ground {
            potential[net] = x[column[net]].clone();
        }
    }
    let current = branches
        .iter()
        .enumerate()
        .map(|(i, branch)| match *branch {
            Branch::Resistor {
                a: p,
                b: q,
                resistance,
            } => potential[p].sub(&potential[q]).div(&values[resistance]),
            Branch::Source { .. } => x[source_column[i]].clone(),
        })
        .collect();

    Ok(Solution { potential, current })
}

/// Gaussian elimination with partial pivoting, over duals.
///
/// The pivot is chosen on the VALUE and the whole row is eliminated including its gradient, which is
/// what makes the returned sensitivities exact rather than a separately derived formula that could
/// disagree with the values beside it.
fn gaussian(
    a: &mut [Dual],
    b: &mut [Dual],
    n: usize,
    width: usize,
) -> Result<Vec<Dual>, NetworkError> {
    for col in 0..n {
        let mut best = col;
        for row in col + 1..n {
            if a[row * n + col].value.abs() > a[best * n + col].value.abs() {
                best = row;
            }
        }
        if a[best * n + col].value.abs() < 1e-300 {
            return Err(NetworkError::Singular { row: col });
        }
        if best != col {
            for c in 0..n {
                a.swap(col * n + c, best * n + c);
            }
            b.swap(col, best);
        }
        for row in col + 1..n {
            let factor = a[row * n + col].div(&a[col * n + col]);
            if factor.value == 0.0 && factor.grad.iter().all(|g| *g == 0.0) {
                continue;
            }
            for c in col..n {
                let take = a[col * n + c].mul(&factor);
                a[row * n + c] = a[row * n + c].sub(&take);
            }
            let take = b[col].mul(&factor);
            b[row] = b[row].sub(&take);
        }
    }

    let mut x = vec![Dual::zero(width); n];
    for col in (0..n).rev() {
        let mut acc = b[col].clone();
        for c in col + 1..n {
            acc = acc.sub(&a[col * n + c].mul(&x[c]));
        }
        x[col] = acc.div(&a[col * n + col]);
    }
    Ok(x)
}
