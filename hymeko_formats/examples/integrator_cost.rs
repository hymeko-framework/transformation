//! Is first order good enough? The comparison that answers it is error against COST, not against
//! step size.
//!
//! A second-order scheme is not automatically better: if it costs four force evaluations per step,
//! it has to beat a first-order scheme taking four times as many steps. And the force evaluation is
//! the whole cost — the integrator's own arithmetic is nanoseconds inside a step that is
//! microseconds. So the axis that matters is **evaluations per second of simulated time**.
//!
//! Four schemes on a chaotic but perfectly smooth mechanism, against a reference:
//!
//!   symplectic Euler   1 evaluation, first order, what the engine ships
//!   velocity Verlet    1 evaluation, second order for a force that depends only on position —
//!                      and ours depends on velocity too, so the predictor below is where its
//!                      order actually lands
//!   RK4                4 evaluations, fourth order
//!   Dormand-Prince 5   6 evaluations, fifth order, the descendant of NASA's RKF45
//!
//! A double pendulum is the honest test bed for the smooth part of the question: it is chaotic, so
//! errors are amplified rather than forgiven, and it has no contact, so nothing here is about the
//! non-smooth half.
//!
//! # Two things the table does not say
//!
//! **Contact.** Everything here assumes the acceleration is a smooth function of the state. A
//! contact makes it discontinuous, and a scheme whose accuracy comes from sampling across the step
//! samples across the discontinuity: its order collapses to first, and it can do worse than the
//! first-order scheme while costing six times as much. That is why contact engines ship Euler, and
//! it is not a compromise made out of ignorance.
//!
//! **Phase.** Symplectic Euler preserves the phase of an oscillation, and a Runge-Kutta method does
//! not. On a small-angle pendulum's PERIOD, at equal cost, symplectic Euler is the more accurate of
//! the two — the opposite of what this table shows for trajectory error on a chaotic arc. Both are
//! true, and `hymeko_simulator/tests/dynamics.rs` pins each.
//!
//! Run: `cargo run --release --example integrator_cost`

use hymeko::common::ids::DeclId;
use hymeko_geometry::articulated::{AbaPlan, AbaScratch};
use hymeko_geometry::kinematics::joints::{JointInfo, JointType};
use hymeko_geometry::kinematics::kinematic::{KinematicModel, LinkInfo};

const GRAVITY: [f64; 3] = [0.0, 0.0, -9.80665];

fn double_pendulum() -> KinematicModel {
    let link = |name: &str, mass: Option<f64>, com: [f64; 3]| LinkInfo {
        inertia_com: None,
        did: DeclId::NONE,
        name: name.into(),
        mass,
        geometry: None,
        origin: Some(com.to_vec()),
        color: None,
    };
    let hinge = |name: &str, parent: &str, child: &str, at: [f64; 3]| JointInfo {
        did: DeclId::NONE,
        name: name.into(),
        joint_type: JointType::Revolute,
        parent: DeclId::NONE,
        child: DeclId::NONE,
        parent_link: parent.into(),
        child_link: child.into(),
        axis: Some([0.0, 1.0, 0.0]),
        origin_xyz: Some(at),
        origin_rpy_deg: None,
        limits: None,
    };
    KinematicModel {
        actuation: Default::default(),
        name: "double".into(),
        links: vec![
            link("base", None, [0.0; 3]),
            link("upper", Some(2.0), [0.0, 0.0, -0.5]),
            link("lower", Some(1.0), [0.0, 0.0, -0.4]),
        ],
        joints: vec![
            hinge("shoulder", "base", "upper", [0.0; 3]),
            hinge("elbow", "upper", "lower", [0.0, 0.0, -0.5]),
        ],
        frames: Vec::new(),
    }
}

struct Sim<'a> {
    aba: &'a AbaPlan,
    model: &'a KinematicModel,
    scratch: AbaScratch,
    tau: Vec<f64>,
    /// Force evaluations made, which is the only cost worth counting.
    evaluations: usize,
}

impl Sim<'_> {
    fn acceleration(&mut self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        self.evaluations += 1;
        self.aba.accelerations(
            self.model,
            q,
            qd,
            &self.tau,
            GRAVITY,
            &mut self.scratch,
            out,
        );
    }
}

/// `qd += h a; q += h qd` — the acceleration taken at the START of the step, the position at its
/// end. What the engine ships.
fn symplectic_euler(sim: &mut Sim, q: &mut [f64], qd: &mut [f64], h: f64, steps: usize) {
    let n = q.len();
    let mut a = vec![0.0; n];
    for _ in 0..steps {
        sim.acceleration(q, qd, &mut a);
        for k in 0..n {
            qd[k] += h * a[k];
            q[k] += h * qd[k];
        }
    }
}

/// Kick, drift, kick — with the acceleration carried over from the previous step, so it is one
/// evaluation and not two.
///
/// Second order when the force depends on position alone. Ours depends on velocity as well, through
/// the Coriolis terms, and the half-step velocity is used as the predictor for it. Whether that
/// costs the second order is exactly what the table measures.
fn velocity_verlet(sim: &mut Sim, q: &mut [f64], qd: &mut [f64], h: f64, steps: usize) {
    let n = q.len();
    let mut a = vec![0.0; n];
    sim.acceleration(q, qd, &mut a);
    for _ in 0..steps {
        for k in 0..n {
            qd[k] += 0.5 * h * a[k];
            q[k] += h * qd[k];
        }
        sim.acceleration(q, qd, &mut a);
        for k in 0..n {
            qd[k] += 0.5 * h * a[k];
        }
    }
}

fn rk4(sim: &mut Sim, q: &mut [f64], qd: &mut [f64], h: f64, steps: usize) {
    let n = q.len();
    let (mut k1, mut k2, mut k3, mut k4) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
    for _ in 0..steps {
        sim.acceleration(q, qd, &mut k1);
        let q2: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * h * qd[i]).collect();
        let d2: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * h * k1[i]).collect();
        sim.acceleration(&q2, &d2, &mut k2);
        let q3: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * h * d2[i]).collect();
        let d3: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * h * k2[i]).collect();
        sim.acceleration(&q3, &d3, &mut k3);
        let q4: Vec<f64> = (0..n).map(|i| q[i] + h * d3[i]).collect();
        let d4: Vec<f64> = (0..n).map(|i| qd[i] + h * k3[i]).collect();
        sim.acceleration(&q4, &d4, &mut k4);
        for i in 0..n {
            q[i] += h / 6.0 * (qd[i] + 2.0 * d2[i] + 2.0 * d3[i] + d4[i]);
            qd[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
}

/// Dormand-Prince 5(4), the modern embedded Runge-Kutta.
///
/// Descended from Runge-Kutta-Fehlberg 4(5), which Erwin Fehlberg developed at NASA's Marshall
/// centre in 1969 — the first widely used method to carry a second, lower-order solution whose
/// difference from the first estimates the step's own error for free. Dormand and Prince's 1980
/// tableau has smaller error constants and is First Same As Last, so a step costs six new
/// evaluations rather than seven. It is what MATLAB's `ode45` and SciPy's `RK45` are.
///
/// Run here at a FIXED step, so the axis stays comparable. That deliberately gives up the thing it
/// is actually for: the embedded estimate is what lets the step grow through the quiet parts of a
/// trajectory and shrink through the fast ones, and a fixed-step comparison cannot show that.
fn dormand_prince(sim: &mut Sim, q: &mut [f64], qd: &mut [f64], h: f64, steps: usize) {
    // c, and the lower triangle of a, row by row
    const A: [&[f64]; 6] = [
        &[],
        &[1.0 / 5.0],
        &[3.0 / 40.0, 9.0 / 40.0],
        &[44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0],
        &[
            19372.0 / 6561.0,
            -25360.0 / 2187.0,
            64448.0 / 6561.0,
            -212.0 / 729.0,
        ],
        &[
            9017.0 / 3168.0,
            -355.0 / 33.0,
            46732.0 / 5247.0,
            49.0 / 176.0,
            -5103.0 / 18656.0,
        ],
    ];
    // the fifth-order weights, which are also the last row of `a` — that is what FSAL means
    const B: [f64; 6] = [
        35.0 / 384.0,
        0.0,
        500.0 / 1113.0,
        125.0 / 192.0,
        -2187.0 / 6784.0,
        11.0 / 84.0,
    ];

    let n = q.len();
    // `k` is the acceleration at each stage, `r` the velocity — the two halves of `f(q, qdot)`
    let mut k = vec![vec![0.0; n]; 6];
    let mut r = vec![vec![0.0; n]; 6];
    let (mut qs, mut ds) = (vec![0.0; n], vec![0.0; n]);

    for _ in 0..steps {
        for stage in 0..6 {
            qs.copy_from_slice(q);
            ds.copy_from_slice(qd);
            for (j, &coefficient) in A[stage].iter().enumerate() {
                if coefficient == 0.0 {
                    continue;
                }
                for i in 0..n {
                    qs[i] += h * coefficient * r[j][i];
                    ds[i] += h * coefficient * k[j][i];
                }
            }
            r[stage].copy_from_slice(&ds);
            sim.acceleration(&qs, &ds, &mut k[stage]);
        }
        for i in 0..n {
            for (stage, &weight) in B.iter().enumerate() {
                if weight == 0.0 {
                    continue;
                }
                q[i] += h * weight * r[stage][i];
                qd[i] += h * weight * k[stage][i];
            }
        }
    }
}

/// A scheme advances `(q, qdot)` by `steps` steps of `h`, counting its evaluations as it goes.
type Scheme = fn(&mut Sim, &mut [f64], &mut [f64], f64, usize);

fn main() {
    let model = double_pendulum();
    let aba = AbaPlan::new(&model).expect("the topology resolves");
    let n = aba.slots();
    let make = || Sim {
        aba: &aba,
        model: &model,
        scratch: aba.scratch(),
        tau: vec![0.0; n],
        evaluations: 0,
    };
    let start = (vec![1.1, -0.6], vec![0.4, 2.2]);
    let duration = 1.0;

    // the reference: RK4 at a step far below anything compared against it
    let reference = {
        let mut sim = make();
        let (mut q, mut qd) = start.clone();
        let h = 2e-7;
        rk4(&mut sim, &mut q, &mut qd, h, (duration / h) as usize);
        q
    };

    println!("  double pendulum, chaotic and smooth, {duration} s of motion");
    println!(
        "  reference: RK4 at h = 2e-7 ({} evaluations)",
        4 * 5_000_000
    );
    println!();
    println!(
        "  {:<18} {:>9}  {:>11}  {:>12}",
        "scheme", "rate Hz", "evaluations", "error rad"
    );

    let schemes: [(&str, Scheme); 4] = [
        ("symplectic Euler", symplectic_euler),
        ("velocity Verlet", velocity_verlet),
        ("RK4", rk4),
        ("Dormand-Prince 5", dormand_prince),
    ];

    for (name, run) in schemes {
        for rate in [15.0, 60.0, 240.0, 960.0, 3840.0] {
            let h = 1.0 / rate;
            let steps = (duration * rate) as usize;
            let mut sim = make();
            let (mut q, mut qd) = start.clone();
            run(&mut sim, &mut q, &mut qd, h, steps);
            let error = q
                .iter()
                .zip(&reference)
                .fold(0.0f64, |m, (a, b)| m.max((a - b).abs()));
            println!(
                "  {name:<18} {rate:>9.0}  {:>11}  {:>12.3e}",
                sim.evaluations, error
            );
        }
        println!();
    }

    println!(
        "  The axis that matters is the middle column. A scheme is better only if it reaches a"
    );
    println!(
        "  smaller error for the same number of evaluations — the integrator's own arithmetic"
    );
    println!("  is nanoseconds inside a step that is microseconds.");
}
