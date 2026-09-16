//! What one forward-dynamics step costs, and how it scales.
//!
//! Two ways of answering the same question, on the same model at the same pose:
//!
//!   A  articulated-body       — three sweeps of the tree, `O(n)`
//!   A' inverse dynamics       — the same sweeps minus the third: what torque produces a motion
//!   B  explicit mass matrix   — a finite-difference Jacobian per coordinate (`2n+1` kinematics
//!                               passes), `M = sum_i m_i J_i^T J_i` assembled, then a dense solve
//!
//! B is what this used to do. It is kept here rather than deleted because the interesting number is
//! not either time on its own — it is how the ratio behaves as the mechanism gets bigger, which is
//! the difference between a linear algorithm and a cubic one and cannot be read off a single robot.
//!
//! The MuJoCo baseline for the same models is `mujoco_step.py`, beside this file.

use std::time::Instant;

use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::joints::JointType;
use hymeko_geometry::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (path, root) = (&args[0], &args[1]);
    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(std::path::Path::new(path)).expect("compile");
    let engine = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&engine, root);

    let aba = AbaPlan::new(&model).expect("the topology resolves");
    let plan = FkPlan::new(&model).expect("the topology resolves");
    let n = aba.slots();
    let dof = aba.movable();
    let bodies = model
        .links
        .iter()
        .filter(|l| l.mass.unwrap_or(0.0) > 0.0)
        .count();

    // A pose that is actually somewhere, not the rest configuration: a chain folded flat has
    // Jacobian columns that are accidentally parallel and a mass matrix that solves faster than it
    // has any right to.
    let q: Vec<f64> = (0..n).map(|i| 0.35 * ((i as f64 * 0.7).sin())).collect();
    let qd: Vec<f64> = (0..n).map(|i| 0.9 * ((i as f64 * 1.3).cos())).collect();
    let tau = vec![0.0; n];
    let gravity = [0.0, 0.0, -9.80665];

    eprintln!(
        "{root}: {} links, {} joints ({dof} movable), {bodies} with mass",
        model.links.len(),
        model.joints.len()
    );

    // --- A: articulated body -----------------------------------------------------------------
    let mut scratch = aba.scratch();
    let mut qdd = vec![0.0; n];
    for _ in 0..200 {
        aba.accelerations(&model, &q, &qd, &tau, gravity, &mut scratch, &mut qdd);
    }
    let reps = 20_000;
    let t0 = Instant::now();
    for _ in 0..reps {
        aba.accelerations(&model, &q, &qd, &tau, gravity, &mut scratch, &mut qdd);
        std::hint::black_box(&qdd);
    }
    let a_us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;

    // --- A': inverse dynamics, the same sweeps minus the third ---------------------------------
    let mut tau_out = vec![0.0; n];
    for _ in 0..200 {
        aba.inverse_dynamics(&model, &q, &qd, &qdd, gravity, &mut scratch, &mut tau_out);
    }
    let t0 = Instant::now();
    for _ in 0..reps {
        aba.inverse_dynamics(&model, &q, &qd, &qdd, gravity, &mut scratch, &mut tau_out);
        std::hint::black_box(&tau_out);
    }
    let i_us = t0.elapsed().as_secs_f64() * 1e6 / reps as f64;

    // --- B: explicit mass matrix ---------------------------------------------------------------
    let slots: Vec<usize> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| !matches!(j.joint_type, JointType::Fixed))
        .map(|(i, _)| i)
        .collect();
    for _ in 0..20 {
        mass_matrix_step(&model, &plan, &q, &slots, bodies);
    }
    let reps_b = 2_000;
    let t0 = Instant::now();
    for _ in 0..reps_b {
        std::hint::black_box(mass_matrix_step(&model, &plan, &q, &slots, bodies));
    }
    let b_us = t0.elapsed().as_secs_f64() * 1e6 / reps_b as f64;

    println!(
        "  {root:<12} dof={dof:<3} forward {a_us:>7.2} us   inverse {i_us:>7.2} us            mass-matrix {b_us:>9.2} us   {:>6.1}x",
        b_us / a_us
    );
}

/// The old path, reproduced here so the comparison is against something that still runs rather than
/// against a number in a commit message: the Jacobian of every mass by central differences, the
/// mass matrix assembled from it, and a dense solve. The generalised force is not computed — this
/// measures the cost, and the gravity term is a rounding error beside the assembly.
fn mass_matrix_step(
    model: &KinematicModel,
    plan: &FkPlan,
    q: &[f64],
    slots: &[usize],
    bodies: usize,
) -> f64 {
    let n = slots.len();
    let mut poses = plan.identity_poses_se3();
    let com: Vec<(usize, f64, [f64; 3])> = model
        .links
        .iter()
        .enumerate()
        .filter_map(|(i, l)| {
            let m = l.mass.unwrap_or(0.0);
            (m > 0.0).then(|| {
                let c = l
                    .origin
                    .as_ref()
                    .filter(|o| o.len() >= 3)
                    .map(|o| [o[0], o[1], o[2]])
                    .unwrap_or([0.0; 3]);
                (i, m, c)
            })
        })
        .collect();

    let h = 1e-5;
    let mut probe = q.to_vec();
    let mut jac = vec![[0.0f64; 3]; n * bodies];
    let mut plus = vec![[0.0f64; 3]; bodies];
    let mut minus = vec![[0.0f64; 3]; bodies];
    for (k, &s) in slots.iter().enumerate() {
        let original = probe[s];
        probe[s] = original + h;
        plan.resolve_se3(model, &probe, &mut poses);
        for (i, &(l, _, c)) in com.iter().enumerate() {
            let p = poses[l].apply(&c);
            plus[i] = [p[0], p[1], p[2]];
        }
        probe[s] = original - h;
        plan.resolve_se3(model, &probe, &mut poses);
        for (i, &(l, _, c)) in com.iter().enumerate() {
            let p = poses[l].apply(&c);
            minus[i] = [p[0], p[1], p[2]];
        }
        probe[s] = original;
        for i in 0..bodies {
            for a in 0..3 {
                jac[k * bodies + i][a] = (plus[i][a] - minus[i][a]) / (2.0 * h);
            }
        }
    }

    let mut m = vec![vec![0.0f64; n]; n];
    for (i, &(_, mass, _)) in com.iter().enumerate() {
        for k in 0..n {
            let jk = jac[k * bodies + i];
            for l in 0..n {
                let jl = jac[l * bodies + i];
                m[k][l] += mass * (jk[0] * jl[0] + jk[1] * jl[1] + jk[2] * jl[2]);
            }
        }
    }
    for (k, row) in m.iter_mut().enumerate() {
        row[k] += 1e-9;
    }

    // Gaussian elimination with partial pivoting, the n^3 the whole exercise is about.
    let mut f = vec![1.0f64; n];
    for col in 0..n {
        let mut best = col;
        for r in col + 1..n {
            if m[r][col].abs() > m[best][col].abs() {
                best = r;
            }
        }
        m.swap(col, best);
        f.swap(col, best);
        let pivot = m[col][col];
        if pivot.abs() < 1e-300 {
            continue;
        }
        for r in col + 1..n {
            let factor = m[r][col] / pivot;
            if factor == 0.0 {
                continue;
            }
            for c in col..n {
                m[r][c] -= factor * m[col][c];
            }
            f[r] -= factor * f[col];
        }
    }
    for col in (0..n).rev() {
        let mut acc = f[col];
        for c in col + 1..n {
            acc -= m[col][c] * f[c];
        }
        f[col] = if m[col][col].abs() < 1e-300 {
            0.0
        } else {
            acc / m[col][col]
        };
    }
    f.iter().sum()
}
