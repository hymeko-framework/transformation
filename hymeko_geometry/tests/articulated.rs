//! Forward dynamics without a mass matrix.
//!
//! The algorithm never forms `M` and never factorises anything, so the usual reassurance — "it
//! agrees with the matrix I built" — is not available and would be circular anyway. Instead every
//! test below reaches the same answer by a **different road**:
//!
//! * a pendulum's closed form, which Lagrange wrote down and no implementation can argue with;
//! * the symmetry of `M^-1`, recovered one column at a time from the algorithm itself — a fact of
//!   physics that essentially every transform in the sweep has to be right for;
//! * gravity against the gradient of the potential energy, differentiated numerically through
//!   forward kinematics alone;
//! * conservation of energy along a trajectory, with both halves of the energy computed from
//!   positions only, so the velocity-product terms are checked by something that has never seen a
//!   spatial vector.

use std::path::PathBuf;

use hymeko::common::ids::DeclId;
use hymeko_clifford::Se3;
use hymeko_geometry::articulated::AbaPlan;
use hymeko_geometry::forward::FkPlan;
use hymeko_geometry::kinematics::joints::{JointInfo, JointType};
use hymeko_geometry::kinematics::kinematic::{extract_kinematic_model, KinematicModel, LinkInfo};

const G: f64 = 9.81;
const GRAV: [f64; 3] = [0.0, 0.0, -G];

// --- fixtures ----------------------------------------------------------------------------------

fn link(name: &str, mass: Option<f64>, com: [f64; 3]) -> LinkInfo {
    LinkInfo {
        did: DeclId::NONE,
        name: name.to_string(),
        mass,
        geometry: None,
        origin: Some(com.to_vec()),
        color: None,
    }
}

fn hinge(name: &str, parent: &str, child: &str, at: [f64; 3]) -> JointInfo {
    JointInfo {
        did: DeclId::NONE,
        name: name.to_string(),
        joint_type: JointType::Revolute,
        parent: DeclId::NONE,
        child: DeclId::NONE,
        parent_link: parent.to_string(),
        child_link: child.to_string(),
        // about y, so the swing is in the x-z plane and gravity does the obvious thing
        axis: Some([0.0, 1.0, 0.0]),
        origin_xyz: Some(at),
        origin_rpy_deg: None,
        limits: None,
    }
}

fn model(name: &str, links: Vec<LinkInfo>, joints: Vec<JointInfo>) -> KinematicModel {
    KinematicModel {
        name: name.to_string(),
        links,
        joints,
        frames: Vec::new(),
    }
}

/// A point mass `m` hanging `l` below a hinge. The one mechanism whose equation of motion is not
/// open to interpretation.
fn pendulum(l: f64, m: f64) -> KinematicModel {
    model(
        "pendulum",
        vec![
            link("base", None, [0.0; 3]),
            link("bob", Some(m), [0.0, 0.0, -l]),
        ],
        vec![hinge("hinge", "base", "bob", [0.0; 3])],
    )
}

/// Two point masses in series — chaotic, strongly coupled, and with a velocity-product term that a
/// position-Jacobian formulation cannot produce at all.
fn double_pendulum(l1: f64, m1: f64, l2: f64, m2: f64) -> KinematicModel {
    model(
        "double",
        vec![
            link("base", None, [0.0; 3]),
            link("upper", Some(m1), [0.0, 0.0, -l1]),
            link("lower", Some(m2), [0.0, 0.0, -l2]),
        ],
        vec![
            hinge("shoulder", "base", "upper", [0.0; 3]),
            hinge("elbow", "upper", "lower", [0.0, 0.0, -l1]),
        ],
    )
}

/// A torso with two arms. The only shape in which a parent accumulates more than one child's
/// articulated inertia — on a chain that addition happens once and a bug in it has nowhere to show.
fn branched() -> KinematicModel {
    model(
        "branched",
        vec![
            link("torso", Some(3.0), [0.0, 0.0, 0.1]),
            link("left_upper", Some(1.1), [0.0, 0.0, -0.28]),
            link("left_lower", Some(0.7), [0.0, 0.0, -0.24]),
            link("right_upper", Some(0.9), [0.0, 0.0, -0.3]),
            link("right_lower", Some(0.6), [0.0, 0.0, -0.22]),
        ],
        vec![
            hinge("l_shoulder", "torso", "left_upper", [0.0, 0.2, 0.3]),
            hinge("l_elbow", "left_upper", "left_lower", [0.0, 0.0, -0.28]),
            hinge("r_shoulder", "torso", "right_upper", [0.0, -0.2, 0.3]),
            hinge("r_elbow", "right_upper", "right_lower", [0.0, 0.0, -0.3]),
        ],
    )
}

/// A robot from `core`'s library, if this checkout has it beside us. `None` rather than a failure:
/// the sibling repository is not this crate's to require.
fn robot(file: &str, root: &str) -> Option<KinematicModel> {
    use hymeko::module_store::module_store::ModuleStore;
    use hymeko::module_store::source_provider::StdFsProvider;
    use hymeko::util::real_parser::RealParser;
    use hymeko_query::QueryEngine;

    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../core/data/robotics/robots")
        .join(file);
    if !path.exists() {
        return None;
    }
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store.compile(&path).ok()?;
    let engine = QueryEngine::new(&program.ir, &store.it);
    Some(extract_kinematic_model(&engine, root))
}

// --- shared machinery ---------------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn f64(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn signed(&mut self) -> f64 {
        2.0 * self.f64() - 1.0
    }
}

/// The slots that actually move. A fixed joint occupies a slot and absorbs any torque put on it,
/// which is correct but makes it meaningless in a mobility matrix.
fn movable_slots(model: &KinematicModel) -> Vec<usize> {
    model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| !matches!(j.joint_type, JointType::Fixed))
        .map(|(i, _)| i)
        .collect()
}

/// `(link index, mass, centre of mass in the link's frame)` for every body that carries any.
fn bodies(model: &KinematicModel) -> Vec<(usize, f64, [f64; 3])> {
    model
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
        .collect()
}

/// World positions of every mass, through forward kinematics and nothing else.
fn positions(model: &KinematicModel, plan: &FkPlan, q: &[f64], bodies: &[(usize, f64, [f64; 3])]) -> Vec<[f64; 3]> {
    let mut poses = plan.identity_poses_se3();
    plan.resolve_se3(model, q, &mut poses);
    bodies
        .iter()
        .map(|&(i, _, c)| {
            let p = poses[i].apply(&c);
            [p[0], p[1], p[2]]
        })
        .collect()
}

/// Potential energy, from positions alone.
fn potential(model: &KinematicModel, plan: &FkPlan, q: &[f64], b: &[(usize, f64, [f64; 3])]) -> f64 {
    positions(model, plan, q, b)
        .iter()
        .zip(b)
        .map(|(p, &(_, m, _))| m * G * p[2])
        .sum()
}

/// Kinetic energy, from positions alone: differentiate forward kinematics to get each mass's world
/// velocity, then sum `m|v|^2/2`. Deliberately numerical and deliberately ignorant of spatial
/// vectors — it is the second opinion, so it must not share a line of code with the first.
fn kinetic(
    model: &KinematicModel,
    plan: &FkPlan,
    q: &[f64],
    qd: &[f64],
    b: &[(usize, f64, [f64; 3])],
) -> f64 {
    let h = 1e-6;
    let mut vel = vec![[0.0f64; 3]; b.len()];
    let mut probe = q.to_vec();
    for (k, &rate) in qd.iter().enumerate().take(q.len()) {
        if rate == 0.0 {
            continue;
        }
        let original = probe[k];
        probe[k] = original + h;
        let plus = positions(model, plan, &probe, b);
        probe[k] = original - h;
        let minus = positions(model, plan, &probe, b);
        probe[k] = original;
        for i in 0..b.len() {
            for a in 0..3 {
                vel[i][a] += rate * (plus[i][a] - minus[i][a]) / (2.0 * h);
            }
        }
    }
    vel.iter()
        .zip(b)
        .map(|(v, &(_, m, _))| 0.5 * m * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
        .sum()
}

/// The inverse mass matrix, one column at a time: with gravity off and the mechanism at rest, a
/// unit torque on coordinate `k` produces exactly the `k`th column of `M^-1`.
fn inverse_mass_matrix(model: &KinematicModel, aba: &AbaPlan, q: &[f64], slots: &[usize]) -> Vec<Vec<f64>> {
    let mut scratch = aba.scratch();
    let n = aba.slots();
    let zero = vec![0.0; n];
    let mut tau = vec![0.0; n];
    let mut qdd = vec![0.0; n];
    let mut m = vec![vec![0.0; slots.len()]; slots.len()];
    for (col, &s) in slots.iter().enumerate() {
        tau[s] = 1.0;
        aba.accelerations(model, q, &zero, &tau, [0.0; 3], &mut scratch, &mut qdd);
        tau[s] = 0.0;
        for (row, &r) in slots.iter().enumerate() {
            m[row][col] = qdd[r];
        }
    }
    m
}

// --- the closed form ---------------------------------------------------------------------------

#[test]
fn a_pendulum_obeys_the_equation_lagrange_wrote_for_it() {
    // `theta_ddot = -(g/l) sin theta`, for every angle and — since a single point mass at a fixed
    // radius has no velocity-product term — at every angular rate. Independent of mass, which is
    // itself worth checking: if the mass appeared in the answer, the inertia and the generalised
    // force would not be cancelling as they must.
    for (l, m) in [(1.0, 1.0), (0.37, 4.2), (2.5, 0.15)] {
        let model = pendulum(l, m);
        let aba = AbaPlan::new(&model).expect("the topology resolves");
        let mut scratch = aba.scratch();
        let mut qdd = vec![0.0; aba.slots()];
        let tau = vec![0.0; aba.slots()];

        for step in -30..=30 {
            let theta = step as f64 * 0.1;
            for &rate in &[0.0, 1.7, -4.0] {
                aba.accelerations(&model, &[theta], &[rate], &tau, GRAV, &mut scratch, &mut qdd);
                let want = -(G / l) * theta.sin();
                assert!(
                    (qdd[0] - want).abs() < 1e-10 * (1.0 + want.abs()),
                    "l={l} m={m} theta={theta} rate={rate}: {} against {want}",
                    qdd[0]
                );
            }
        }
    }
}

#[test]
fn a_torque_on_a_pendulum_at_rest_divides_by_its_moment_of_inertia() {
    // `I theta_ddot = tau` with `I = m l^2`, the other half of the equation. Together with the test
    // above this pins both the generalised force and the inertia, separately.
    for (l, m) in [(1.0, 1.0), (0.6, 2.5)] {
        let model = pendulum(l, m);
        let aba = AbaPlan::new(&model).expect("the topology resolves");
        let mut scratch = aba.scratch();
        let mut qdd = vec![0.0; 1];
        for &t in &[1.0, -3.5, 0.25] {
            aba.accelerations(&model, &[0.0], &[0.0], &[t], [0.0; 3], &mut scratch, &mut qdd);
            let want = t / (m * l * l);
            assert!(
                (qdd[0] - want).abs() < 1e-12 * (1.0 + want.abs()),
                "l={l} m={m} tau={t}: {} against {want}",
                qdd[0]
            );
        }
    }
}

// --- the structural facts ----------------------------------------------------------------------

#[test]
fn the_mobility_matrix_is_symmetric_and_positive_definite() {
    // `M` is a metric — the kinetic energy's — so `M^-1` is symmetric and positive definite, and
    // the algorithm has to produce one column at a time without ever seeing the whole. Nearly every
    // transform in all three sweeps has to be right for this to hold, and none of it is checked
    // against a stored number.
    let mut rng = Rng(0xABA);
    let mut checked = 0;
    for m in [
        Some(pendulum(0.8, 1.4)),
        Some(double_pendulum(0.5, 2.0, 0.4, 1.0)),
        Some(branched()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let slots = movable_slots(&m);
        if slots.is_empty() {
            continue;
        }
        for _ in 0..4 {
            let q: Vec<f64> = (0..aba.slots()).map(|_| rng.signed() * 1.5).collect();
            let minv = inverse_mass_matrix(&m, &aba, &q, &slots);

            let scale = minv
                .iter()
                .flatten()
                .fold(0.0f64, |a, v| a.max(v.abs()))
                .max(1e-12);
            for (i, row) in minv.iter().enumerate() {
                for (j, &here) in row.iter().enumerate() {
                    let mirror = minv[j][i];
                    assert!(
                        (here - mirror).abs() < 1e-8 * scale,
                        "{}: M^-1 is not symmetric at ({i},{j}): {here} against {mirror}",
                        m.name
                    );
                }
                // and the diagonal is positive: a torque on a coordinate accelerates it forwards
                assert!(
                    row[i] > 0.0,
                    "{}: M^-1 has a non-positive diagonal at {i}: {}",
                    m.name,
                    row[i]
                );
            }
            checked += 1;
        }
    }
    assert!(checked >= 12, "only {checked} configurations were checked");
}

#[test]
fn gravity_is_the_gradient_of_the_potential_energy() {
    // With the mechanism at rest and no torque applied, `M qdd = -dV/dq`. The right-hand side is
    // differentiated numerically through forward kinematics — no spatial vector is involved in
    // producing it — and the left is inverted using the algorithm's own mobility matrix. Anything
    // wrong in how gravity enters as a fictitious base acceleration shows up here.
    let mut rng = Rng(77);
    for m in [
        pendulum(0.8, 1.4),
        double_pendulum(0.5, 2.0, 0.4, 1.0),
        branched(),
    ] {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let plan = FkPlan::new(&m).expect("the topology resolves");
        let b = bodies(&m);
        let slots = movable_slots(&m);
        let n = aba.slots();
        let mut scratch = aba.scratch();
        let mut qdd = vec![0.0; n];

        for _ in 0..6 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.2).collect();

            // -dV/dq, by central differences on the potential
            let h = 1e-6;
            let mut force = vec![0.0; slots.len()];
            let mut probe = q.clone();
            for (k, &s) in slots.iter().enumerate() {
                let original = probe[s];
                probe[s] = original + h;
                let up = potential(&m, &plan, &probe, &b);
                probe[s] = original - h;
                let down = potential(&m, &plan, &probe, &b);
                probe[s] = original;
                force[k] = -(up - down) / (2.0 * h);
            }

            let minv = inverse_mass_matrix(&m, &aba, &q, &slots);
            aba.accelerations(&m, &q, &vec![0.0; n], &vec![0.0; n], GRAV, &mut scratch, &mut qdd);

            for (row, &r) in slots.iter().enumerate() {
                let want: f64 = (0..slots.len()).map(|c| minv[row][c] * force[c]).sum();
                assert!(
                    (qdd[r] - want).abs() < 1e-5 * (1.0 + want.abs()),
                    "{}: coordinate {r}: {} against {want}",
                    m.name,
                    qdd[r]
                );
            }
        }
    }
}

// --- along a trajectory --------------------------------------------------------------------------

#[test]
fn a_double_pendulum_keeps_its_energy_for_a_thousand_steps() {
    // The velocity-product term is the hard part and the one a position-Jacobian formulation omits
    // entirely. Nothing static can see it: at rest it is zero. So the mechanism is set swinging and
    // its total energy is watched — with BOTH halves computed from positions alone, so a wrong
    // Coriolis term cannot hide by being wrong in the reference too.
    let m = double_pendulum(0.5, 2.0, 0.4, 1.0);
    let aba = AbaPlan::new(&m).expect("the topology resolves");
    let plan = FkPlan::new(&m).expect("the topology resolves");
    let b = bodies(&m);
    let mut scratch = aba.scratch();
    let n = aba.slots();
    let tau = vec![0.0; n];

    let mut q = vec![1.1, -0.6];
    let mut qd = vec![0.4, 2.2]; // fast enough that the velocity-product term dominates
    let e0 = kinetic(&m, &plan, &q, &qd, &b) + potential(&m, &plan, &q, &b);
    assert!(e0.abs() > 1.0, "the fixture must actually carry energy");

    let mut acc = |q: &[f64], qd: &[f64]| {
        let mut out = vec![0.0; n];
        aba.accelerations(&m, q, qd, &tau, GRAV, &mut scratch, &mut out);
        out
    };

    let dt = 1e-4;
    let mut worst: f64 = 0.0;
    for _ in 0..1000 {
        // classical RK4 on (q, qd)
        let k1v = acc(&q, &qd);
        let q2: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * dt * qd[i]).collect();
        let d2: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * dt * k1v[i]).collect();
        let k2v = acc(&q2, &d2);
        let q3: Vec<f64> = (0..n).map(|i| q[i] + 0.5 * dt * d2[i]).collect();
        let d3: Vec<f64> = (0..n).map(|i| qd[i] + 0.5 * dt * k2v[i]).collect();
        let k3v = acc(&q3, &d3);
        let q4: Vec<f64> = (0..n).map(|i| q[i] + dt * d3[i]).collect();
        let d4: Vec<f64> = (0..n).map(|i| qd[i] + dt * k3v[i]).collect();
        let k4v = acc(&q4, &d4);

        for i in 0..n {
            q[i] += dt / 6.0 * (qd[i] + 2.0 * d2[i] + 2.0 * d3[i] + d4[i]);
            qd[i] += dt / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);
        }

        let e = kinetic(&m, &plan, &q, &qd, &b) + potential(&m, &plan, &q, &b);
        worst = worst.max(((e - e0) / e0.abs()).abs());
    }

    // The floor here is the reference, not the integrator: the kinetic energy is assembled from
    // central differences with h = 1e-6, so it is only good to about 1e-9 relative, and RK4 at
    // 1e-4 s is far better than that over a tenth of a second.
    assert!(
        worst < 1e-7,
        "energy drifted by {worst:e} relative over 1000 steps"
    );
}

#[test]
fn a_free_mechanism_in_zero_gravity_conserves_energy_too() {
    // The same watch with gravity removed, so the ONLY thing keeping the energy constant is the
    // velocity-product term. With gravity on, a wrong Coriolis can be partly masked by the
    // potential's much larger swing; here it has nowhere to hide.
    let m = branched();
    let aba = AbaPlan::new(&m).expect("the topology resolves");
    let plan = FkPlan::new(&m).expect("the topology resolves");
    let b = bodies(&m);
    let mut scratch = aba.scratch();
    let n = aba.slots();
    let tau = vec![0.0; n];

    let mut q = vec![0.5, -0.9, 0.3, 1.2];
    let mut qd = vec![2.5, -3.0, 1.5, 4.0];
    let t0 = kinetic(&m, &plan, &q, &qd, &b);
    assert!(t0 > 1.0, "the fixture must actually be moving");

    let dt = 1e-4;
    let mut worst: f64 = 0.0;
    let mut out = vec![0.0; n];
    for _ in 0..1000 {
        // Heun, which is enough at this step and keeps the test readable
        aba.accelerations(&m, &q, &qd, &tau, [0.0; 3], &mut scratch, &mut out);
        let k1 = out.clone();
        let qp: Vec<f64> = (0..n).map(|i| q[i] + dt * qd[i]).collect();
        let dp: Vec<f64> = (0..n).map(|i| qd[i] + dt * k1[i]).collect();
        aba.accelerations(&m, &qp, &dp, &tau, [0.0; 3], &mut scratch, &mut out);
        for i in 0..n {
            q[i] += 0.5 * dt * (qd[i] + dp[i]);
            qd[i] += 0.5 * dt * (k1[i] + out[i]);
        }
        worst = worst.max(((kinetic(&m, &plan, &q, &qd, &b) - t0) / t0).abs());
    }
    assert!(
        worst < 1e-4,
        "kinetic energy drifted by {worst:e} relative with no force acting"
    );
}

// --- the plan agrees with the kinematics it was built from ---------------------------------------

#[test]
fn the_dynamics_plan_poses_the_mechanism_the_same_way_the_kinematics_does() {
    // `AbaPlan` builds its own `FkPlan`; if that ever drifted from the one the renderer uses, the
    // robot would be drawn in one place and simulated in another.
    let mut rng = Rng(5);
    for m in [pendulum(0.8, 1.4), double_pendulum(0.5, 2.0, 0.4, 1.0), branched()] {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let plan = FkPlan::new(&m).expect("the topology resolves");
        let q: Vec<f64> = (0..m.joints.len()).map(|_| rng.signed() * 2.0).collect();
        let mut a: Vec<Se3> = plan.identity_poses_se3();
        let mut b: Vec<Se3> = plan.identity_poses_se3();
        aba.kinematics().resolve_se3(&m, &q, &mut a);
        plan.resolve_se3(&m, &q, &mut b);
        for (i, (x, y)) in a.iter().zip(&b).enumerate() {
            assert_eq!(x.rotor, y.rotor, "{}: link {i} rotor", m.name);
            assert_eq!(x.translation, y.translation, "{}: link {i} translation", m.name);
        }
    }
}

// --- the other direction -------------------------------------------------------------------------

#[test]
fn inverse_dynamics_undoes_forward_dynamics_exactly() {
    // THE test for the pair. Ask what torque produces a chosen acceleration, apply it, and the
    // mechanism must accelerate exactly as chosen. The two solvers share the spatial algebra but
    // not a line of their own logic — one sweeps three times and downdates rank-one, the other
    // sweeps twice and sums forces — so a sign error in either has nowhere to cancel.
    //
    // Run at a non-zero velocity, because at rest the velocity-product terms vanish from both and
    // the test would be checking half of each.
    let mut rng = Rng(0x1D);
    let mut checked = 0;
    for m in [
        Some(pendulum(0.8, 1.4)),
        Some(double_pendulum(0.5, 2.0, 0.4, 1.0)),
        Some(branched()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let slots = movable_slots(&m);
        let mut scratch = aba.scratch();
        let mut tau = vec![0.0; n];
        let mut back = vec![0.0; n];

        for _ in 0..6 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.5).collect();
            let qd: Vec<f64> = (0..n).map(|_| rng.signed() * 3.0).collect();
            let mut qdd = vec![0.0; n];
            for &s in &slots {
                qdd[s] = rng.signed() * 5.0;
            }

            aba.inverse_dynamics(&m, &q, &qd, &qdd, GRAV, &mut scratch, &mut tau);
            aba.accelerations(&m, &q, &qd, &tau, GRAV, &mut scratch, &mut back);

            let scale = qdd.iter().fold(1.0f64, |a, v| a.max(v.abs()));
            for &s in &slots {
                assert!(
                    (back[s] - qdd[s]).abs() < 1e-7 * scale,
                    "{}: slot {s}: asked for {} and got {} back",
                    m.name,
                    qdd[s],
                    back[s]
                );
            }
            checked += 1;
        }
    }
    assert!(checked >= 18, "only {checked} configurations were checked");
}

#[test]
fn the_gravity_compensation_torque_holds_a_mechanism_still() {
    // `inverse_dynamics(q, 0, 0)` is the torque that exactly cancels gravity. Apply it and nothing
    // must move — which is the whole point of computing it, and the first thing any arm controller
    // wants.
    let mut rng = Rng(606);
    for m in [
        Some(branched()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let mut scratch = aba.scratch();
        let zero = vec![0.0; n];
        let mut tau = vec![0.0; n];
        let mut qdd = vec![0.0; n];

        for _ in 0..5 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.5).collect();

            // how hard it would fall if nothing held it
            aba.accelerations(&m, &q, &zero, &zero, GRAV, &mut scratch, &mut qdd);
            let falling = qdd.iter().fold(0.0f64, |a, v| a.max(v.abs()));
            assert!(falling > 0.5, "{}: the fixture must actually sag", m.name);

            aba.inverse_dynamics(&m, &q, &zero, &zero, GRAV, &mut scratch, &mut tau);
            aba.accelerations(&m, &q, &zero, &tau, GRAV, &mut scratch, &mut qdd);
            let held = qdd.iter().fold(0.0f64, |a, v| a.max(v.abs()));
            assert!(
                held < 1e-9 * falling.max(1.0),
                "{}: held still to only {held:e} rad/s^2 while it would have fallen at {falling:e}",
                m.name
            );
        }
    }
}

#[test]
fn a_pendulums_holding_torque_is_the_one_statics_gives() {
    // `tau = m l^2 theta_ddot + m g l sin theta`, straight out of the same Lagrangian the forward
    // test used, and reached here from the opposite end.
    for (l, m) in [(1.0, 1.0), (0.37, 4.2)] {
        let model = pendulum(l, m);
        let aba = AbaPlan::new(&model).expect("the topology resolves");
        let mut scratch = aba.scratch();
        let mut tau = vec![0.0; 1];
        for step in -20..=20 {
            let theta = step as f64 * 0.15;
            for &acc in &[0.0, 2.5, -7.0] {
                aba.inverse_dynamics(&model, &[theta], &[0.0], &[acc], GRAV, &mut scratch, &mut tau);
                let want = m * l * l * acc + m * G * l * theta.sin();
                assert!(
                    (tau[0] - want).abs() < 1e-10 * (1.0 + want.abs()),
                    "l={l} m={m} theta={theta} qdd={acc}: {} against {want}",
                    tau[0]
                );
            }
        }
    }
}

#[test]
// A matrix product walks one operand by column, which no iterator over rows expresses; the lint is
// right about loops that index a row and wrong about this one.
#[allow(clippy::needless_range_loop)]
fn the_mass_matrix_from_torques_inverts_the_one_from_accelerations() {
    // Column `k` of `M` is `inverse_dynamics(q, 0, e_k)` with gravity off; column `k` of `M^-1` is
    // `accelerations(q, 0, e_k)` the same way. Their product is the identity — and neither was ever
    // assembled from a finite difference. This is also the honest way to get `M` for anything that
    // genuinely needs one, at `n` exact sweeps rather than a numerical Jacobian.
    let mut rng = Rng(4242);
    for m in [double_pendulum(0.5, 2.0, 0.4, 1.0), branched()] {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let slots = movable_slots(&m);
        let d = slots.len();
        let mut scratch = aba.scratch();
        let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.4).collect();

        let mut mass = vec![vec![0.0; d]; d];
        let mut unit = vec![0.0; n];
        let mut out = vec![0.0; n];
        for (col, &s) in slots.iter().enumerate() {
            unit[s] = 1.0;
            aba.inverse_dynamics(&m, &q, &vec![0.0; n], &unit, [0.0; 3], &mut scratch, &mut out);
            unit[s] = 0.0;
            for (row, &r) in slots.iter().enumerate() {
                mass[row][col] = out[r];
            }
        }
        let minv = inverse_mass_matrix(&m, &aba, &q, &slots);

        for (i, row) in mass.iter().enumerate().take(d) {
            for j in 0..d {
                let got: f64 = (0..d).map(|k| row[k] * minv[k][j]).sum();
                let want = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (got - want).abs() < 1e-9,
                    "{}: (M M^-1)[{i}][{j}] = {got}, not {want}",
                    m.name
                );
            }
        }
    }
}

// --- what the fuzz found ------------------------------------------------------------------------

/// A joint with every field spelled out, for fixtures the fuzzer emits.
fn joint(
    name: &str,
    parent: &str,
    child: &str,
    joint_type: JointType,
    axis: [f64; 3],
    xyz: [f64; 3],
    rpy_deg: [f64; 3],
) -> JointInfo {
    JointInfo {
        did: DeclId::NONE,
        name: name.to_string(),
        joint_type,
        parent: DeclId::NONE,
        child: DeclId::NONE,
        parent_link: parent.to_string(),
        child_link: child.to_string(),
        axis: Some(axis),
        origin_xyz: Some(xyz),
        origin_rpy_deg: Some(rpy_deg),
        limits: None,
    }
}

/// The exact mechanism the fuzz failed on, emitted by `dynamics_fuzz --dump 42278` rather than
/// reconstructed by hand.
///
/// That distinction is the point. A hand-written approximation of this chain — same mass pattern,
/// same length, invented axes and origins — was written first and **passed against the very bug it
/// was written for**. The geometry matters as much as the masses: whether `S^T I^A S` cancels to
/// below its own rounding depends on where the axes point, and guessing does not reproduce it.
/// Generated by `dynamics_fuzz --dump 42278` (Chain, 22 links).
fn fuzz_42278() -> KinematicModel {
    let links = vec![
        link("l0", Some(1.0686539629801426), [0.1, 0.0, 0.05]),
        link("l1", Some(1.665162420006937e-6), [0.004072874985085842, -0.019942312345726992, 0.03809665437642861]),
        link("l2", Some(1515.6824119409232), [0.2080494292556652, 0.16831045850183549, 0.11120304960014213]),
        link("l3", None, [0.190022416633039, 0.18981685442232038, 0.06002791327564229]),
        link("l4", None, [-0.029866372359223715, -0.033103362440638086, -0.004230649938305005]),
        link("l5", None, [-0.008412034695668912, 0.01793410278741527, 0.14168303974496507]),
        link("l6", None, [-0.14847685558291487, -0.08891840598798148, 0.022881973449231967]),
        link("l7", None, [0.05500431708208536, 0.011869053850301438, -0.13113662120457603]),
        link("l8", Some(1.4261193486483636), [-0.26602248184211164, -0.011912929061859732, -0.19875977608420223]),
        link("l9", Some(1.9155270064816497e-6), [-0.12115905097526111, 0.08749313473610584, 0.12193914301833703]),
        link("l10", Some(1.4093992543428068e-6), [-0.07320220635605273, -0.2956285491457427, -0.038956859310341256]),
        link("l11", Some(1567.5926307683233), [-0.03130986465254089, 0.01505140390785114, 0.030409449609866517]),
        link("l12", None, [-0.025214435820454885, -0.007700624505403768, -0.002870777338014192]),
        link("l13", None, [-0.2863902959590721, 0.035711463935185206, 0.18170076609651198]),
        link("l14", None, [0.15828892374396697, 0.02982545946933389, 0.10696057429899312]),
        link("l15", None, [-0.12116988520671146, 0.3695468438653225, -0.029282796078196466]),
        link("l16", Some(1.868811316647581e-6), [0.032978149495420266, 0.024019312338577573, -0.06017508634485475]),
        link("l17", None, [-0.010870561336877825, -0.015481336607428224, -0.019446706469226985]),
        link("l18", None, [-0.10722051824124712, -0.2057680525919364, -0.24653363046860954]),
        link("l19", None, [0.09978528886103318, 0.06584029401389024, -0.10067415510185279]),
        link("l20", Some(1164.2449122129844), [-0.14320743312652667, -0.03458796012538523, 0.09374448894512971]),
        link("l21", None, [0.04083248299453884, -0.06802455216295121, 0.22979813606523558]),
    ];
    let joints = vec![
        joint("j1", "l0", "l1", JointType::Revolute, [-0.09429469276766066, 0.4617019237278209, -0.8820089821208636], [-0.2911326589400976, -0.3010731791027572, -0.33675366967357095], [-69.91306854453268, 93.7770471280729, 154.45460686932475]),
        joint("j2", "l1", "l2", JointType::Fixed, [-0.1188126025153556, -0.1988030732721555, 0.9728108261841426], [0.19603168542638053, 0.1969284129182885, -0.06438450816762291], [170.79172805976802, 22.40394944357948, -112.16059253822714]),
        joint("j3", "l2", "l3", JointType::Continuous, [-0.6904553058123876, -0.6897083859406163, -0.21811422016620538], [0.32875049607821377, -0.06687136679470677, 0.25364112110776676], [-61.812945721790825, -33.889587884146074, 75.25689718632256]),
        joint("j4", "l3", "l4", JointType::Prismatic, [0.666877569447568, 0.7391553824961564, 0.09446495590781141], [0.13616142138025467, 0.23133441937662785, -0.2516332790937029], [130.9224798910552, 18.140538857935287, 11.125330356735867]),
        joint("j5", "l4", "l5", JointType::Revolute, [0.05880029399395085, -0.1253597440534447, -0.990367335889608], [0.24276479056439648, -0.3006758640192755, 0.23505513372594677], [-93.82519957766056, -123.9643353712847, 146.61109844262103]),
        joint("j6", "l5", "l6", JointType::Fixed, [0.10717065118321557, -0.8666571602491028, -0.487257445416616], [0.29704734955501516, 0.02227275860608291, -0.098142219326939], [-52.28890966991382, -162.13473781539903, -151.64035193307498]),
        joint("j7", "l6", "l7", JointType::Revolute, [0.7067126434550459, -0.6919577447059745, -0.14748464029248076], [0.05946715133562304, -0.15484050290197643, -0.10848477077200283], [-19.2948223006147, 66.54661329877104, -60.66249304118273]),
        joint("j8", "l7", "l8", JointType::Revolute, [-0.8005780134747122, -0.03585121459271747, -0.598155109276106], [-0.286106840913914, 0.21410882423193034, 0.24703715209086294], [-65.66462830125418, 8.834238101200476, -176.8058707604503]),
        joint("j9", "l8", "l9", JointType::Revolute, [-0.559492215119936, 0.5974066773758014, -0.5745204287464395], [0.04114454323965662, 0.024983894341827375, 0.1595945782781735], [83.97758128782951, -134.2186147465839, -15.618073652255093]),
        joint("j10", "l9", "l10", JointType::Revolute, [0.23841402492427963, 0.9628397256435142, 0.12687953121811044], [0.3763805747093236, -0.1589626469872707, -0.04819001077786425], [-65.84609150058327, 19.787644689806193, -66.81543462816369]),
        joint("j11", "l10", "l11", JointType::Revolute, [-0.6781560768962256, 0.32600591344621166, 0.6586535354545829], [-0.0728222321310235, -0.29795683574784854, -0.3437111398062375], [-146.17315136839323, -133.72002544651357, 174.62192656208828]),
        joint("j12", "l11", "l12", JointType::Revolute, [-0.9507716940120434, -0.2903707962410126, -0.10824964921804452], [-0.04679517955518229, -0.09112721762800656, -0.09413837678585057], [58.02667139019896, 68.19750061934015, -102.10543329667111]),
        joint("j13", "l12", "l13", JointType::Revolute, [-0.5847896790386312, 0.37367861579844025, -0.7199898078340113], [0.374484064203336, 0.2790448628439883, -0.10217658315058094], [154.79245547840387, 86.06842302171647, -155.2234578122691]),
        joint("j14", "l13", "l14", JointType::Revolute, [0.07753737083338412, -0.8116109935956514, 0.579029836190611], [-0.16554323482571603, 0.32062638337581223, 0.1717902437345253], [-95.78090527546756, 81.08954203013292, -87.91322772448538]),
        joint("j15", "l14", "l15", JointType::Revolute, [0.31068742452709924, -0.9475420148064623, 0.07508298353973505], [0.2905483708491216, -0.32132903945513125, -0.12586601968774716], [49.468784943068876, 120.63157573737593, -73.53781652628193]),
        joint("j16", "l15", "l16", JointType::Revolute, [0.45360950455755283, 0.3303820419404779, -0.8276992954801229], [-0.2837600207647008, 0.23837910756971636, 0.24281731981219226], [74.78561803142472, -54.49624014456302, 132.46814667366516]),
        joint("j17", "l16", "l17", JointType::Revolute, [-0.4006900478421658, -0.5706437150441338, -0.7168076701883653], [-0.03281527932223218, 0.037088280686743684, 0.24850386426453133], [115.4714732199007, 43.54631309129707, -54.83625083740458]),
        joint("j18", "l17", "l18", JointType::Revolute, [-0.01642613421781036, -0.8551668328339805, 0.5180925304763209], [-0.25525516425767575, 0.2491553142078952, 0.2976734891219485], [-152.02646674616386, 22.82353150408838, -15.315096841395714]),
        joint("j19", "l18", "l19", JointType::Revolute, [-0.5167885978828781, 0.343737642118917, -0.7840752377729935], [-0.0764163464779733, 0.14208062344170128, -0.3599071983928404], [-164.32608638097446, -7.875705757831608, 170.08188221391848]),
        joint("j20", "l19", "l20", JointType::Fixed, [-0.5836793064303288, -0.5770946166750472, -0.5712094805320468], [-0.3355343060601462, 0.23335048943942446, 0.26681856966570344], [17.94071176975528, -159.01656136124203, -166.96384956825375]),
        joint("j21", "l20", "l21", JointType::Fixed, [0.5899571175928204, -0.7979412477041, -0.12345187165043243], [-0.1773071941358751, 0.24294420036183217, 0.3118052041390896], [-9.283831205300412, -53.60590671399806, 143.67895609565008]),
    ];
    model("fuzz42278", links, joints)
}

#[test]
fn a_chain_of_wildly_mismatched_masses_stays_finite() {
    // The regularisation used to be `d.max(1e-12)`, an absolute floor. That turns a NEGATIVE `d` —
    // which only means the accumulated inertia has drifted off positive semi-definite, as it must —
    // into a small positive one, so `1/d` lands at 1e12 with the wrong sign and the rank-one
    // downdate ADDS an enormous term. On this chain the magnitudes went
    // 3.9e-8 -> 1.6e3 -> 1.3e4 -> 3.4e18 -> 7.5e46 -> 7.1e105 -> inf, one level at a time.
    //
    // Nothing about it is exotic except the mass ratio, and the three real robots never showed it.
    let m = fuzz_42278();
    let aba = AbaPlan::new(&m).expect("the topology resolves");
    let n = aba.slots();
    let slots = movable_slots(&m);
    let mut scratch = aba.scratch();
    let mut rng = Rng(0xBADBAD);

    let mut tau = vec![0.0; n];
    let mut qdd = vec![0.0; n];
    for round in 0..40 {
        let q: Vec<f64> = (0..n).map(|_| rng.signed() * 2.5).collect();
        let qd: Vec<f64> = (0..n).map(|_| rng.signed() * 2.5).collect();
        let mut want = vec![0.0; n];
        for &s in &slots {
            want[s] = rng.signed() * 3.0;
        }

        aba.inverse_dynamics(&m, &q, &qd, &want, GRAV, &mut scratch, &mut tau);
        assert!(
            tau.iter().all(|v| v.is_finite()),
            "round {round}: inverse dynamics produced {tau:?}"
        );
        aba.accelerations(&m, &q, &qd, &tau, GRAV, &mut scratch, &mut qdd);
        assert!(
            qdd.iter().all(|v| v.is_finite()),
            "round {round}: forward dynamics produced {qdd:?}"
        );

        // and it is not merely finite — the mobility matrix keeps its shape
        let minv = inverse_mass_matrix(&m, &aba, &q, &slots);
        let scale = minv
            .iter()
            .flatten()
            .fold(0.0f64, |a, v| a.max(v.abs()))
            .max(1e-30);
        assert!(scale.is_finite(), "round {round}: M^-1 is not finite");
        for (i, row) in minv.iter().enumerate() {
            for (j, &here) in row.iter().enumerate() {
                assert!(
                    (here - minv[j][i]).abs() < 1e-7 * scale,
                    "round {round}: M^-1 asymmetric at ({i},{j})"
                );
            }
        }
    }
}

#[test]
fn a_joint_with_no_inertia_about_its_own_axis_is_simply_free() {
    // The degenerate case named directly: a single link whose entire mass sits ON the axis it turns
    // about. There is nothing to accelerate and nothing to resist, so the acceleration is zero and
    // the torque needed to produce any motion is zero — not a large number, and not a NaN.
    let axis = [0.0, 0.0, 1.0];
    let m = model(
        "on_axis",
        vec![
            link("base", None, [0.0; 3]),
            // 2 kg sitting 0.4 m along z, which is exactly the rotation axis
            link("spinner", Some(2.0), [0.0, 0.0, 0.4]),
        ],
        vec![JointInfo {
            did: DeclId::NONE,
            name: "spin".into(),
            joint_type: JointType::Revolute,
            parent: DeclId::NONE,
            child: DeclId::NONE,
            parent_link: "base".into(),
            child_link: "spinner".into(),
            axis: Some(axis),
            origin_xyz: Some([0.0; 3]),
            origin_rpy_deg: None,
            limits: None,
        }],
    );
    let aba = AbaPlan::new(&m).expect("the topology resolves");
    let mut scratch = aba.scratch();
    let mut qdd = vec![0.0; 1];
    let mut tau = vec![0.0; 1];

    for &t in &[0.0, 5.0, -100.0] {
        aba.accelerations(&m, &[0.7], &[1.3], &[t], GRAV, &mut scratch, &mut qdd);
        assert_eq!(
            qdd[0], 0.0,
            "a torque of {t} on a joint with no inertia about its axis must not produce a number"
        );
    }
    aba.inverse_dynamics(&m, &[0.7], &[1.3], &[4.0], GRAV, &mut scratch, &mut tau);
    assert_eq!(
        tau[0], 0.0,
        "and no torque is needed to accelerate something with no inertia"
    );
}

// --- the constraint Jacobian ----------------------------------------------------------------------

#[test]
fn a_force_at_a_point_produces_the_torques_virtual_work_demands() {
    // `tau_k = f . dp/dq_k`, and `dp/dq_k` is differentiated numerically through forward kinematics
    // alone. That is the definition of a generalised force, reached without a spatial vector, and it
    // is the identity the whole constraint machinery rests on: a contact force enters the dynamics
    // through this and nothing else.
    let mut rng = Rng(0x5EA1);
    let mut checked = 0;
    for m in [
        Some(double_pendulum(0.5, 2.0, 0.4, 1.0)),
        Some(branched()),
        robot("arm_gripper.hymeko", "robot"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let plan = FkPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let mut scratch = aba.scratch();
        let mut tau = vec![0.0; n];
        let mut poses = plan.identity_poses_se3();

        for round in 0..4 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.5).collect();
            plan.resolve_se3(&m, &q, &mut poses);

            // a point rigidly attached to some link, and a direction to push it in
            let body = (round * 3 + 1) % m.links.len();
            let local = [rng.signed() * 0.2, rng.signed() * 0.2, rng.signed() * 0.2];
            let w = poses[body].apply(&local);
            let point = [w[0], w[1], w[2]];
            let mut dir = [rng.signed(), rng.signed(), rng.signed()];
            let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
            for c in dir.iter_mut() {
                *c /= len;
            }

            aba.prepare_transforms(&m, &q, &mut scratch);
            aba.constraint_force(&poses, body, point, dir, &mut scratch, &mut tau);

            // dp/dq_k by central differences, where p is the SAME material point on the body
            let h = 1e-6;
            let mut probe = q.clone();
            let mut moved = plan.identity_poses_se3();
            for k in 0..n {
                let original = probe[k];
                probe[k] = original + h;
                plan.resolve_se3(&m, &probe, &mut moved);
                let up = moved[body].apply(&local);
                probe[k] = original - h;
                plan.resolve_se3(&m, &probe, &mut moved);
                let down = moved[body].apply(&local);
                probe[k] = original;

                let want = (0..3)
                    .map(|a| dir[a] * (up[a] - down[a]) / (2.0 * h))
                    .sum::<f64>();
                assert!(
                    (tau[k] - want).abs() < 1e-5 * (1.0 + want.abs()),
                    "{}: round {round}, body {body}, joint {k}: {} against {want}",
                    m.name,
                    tau[k]
                );
            }
            checked += 1;
        }
    }
    assert!(checked >= 12, "only {checked} configurations were checked");
}

/// How many joints lie between a link and the root.
fn joints_to_root(model: &KinematicModel, link: &str) -> usize {
    let mut name = link.to_string();
    let mut d = 0;
    while let Some(j) = model.joints.iter().find(|j| j.child_link == name) {
        d += 1;
        name = j.parent_link.clone();
        if d > model.links.len() {
            break;
        }
    }
    d
}

#[test]
fn a_contact_reaches_only_the_joints_between_it_and_the_root() {
    // The structural claim, and the reason this is a sweep rather than a Jacobian: a force on one
    // foot of a quadruped must produce ZERO torque on the other three legs. Not a small torque —
    // exactly zero, because those joints are not on the path.
    let Some(m) = robot("quadruped.hymeko", "quadruped") else {
        return;
    };
    let aba = AbaPlan::new(&m).expect("the topology resolves");
    let plan = FkPlan::new(&m).expect("the topology resolves");
    let n = aba.slots();
    let mut scratch = aba.scratch();
    let mut tau = vec![0.0; n];
    let mut poses = plan.identity_poses_se3();
    let q = vec![0.3; n];
    plan.resolve_se3(&m, &q, &mut poses);

    // The DEEPEST leaf, which on a quadruped is a foot.
    //
    // Not the first leaf found: on this model that is `mid_body`, welded to the torso, whose only
    // movable ancestor is a vertical-axis joint at the base. A vertical force through a point
    // produces no torque about a vertical axis — correct, and useless as a test of reach. Depth is
    // what makes the chain long enough for the claim to mean anything.
    let leaf = (0..m.links.len())
        .filter(|&i| {
            let name = m.links[i].name.as_str();
            m.joints.iter().any(|j| j.child_link == name)
                && m.joints.iter().all(|j| j.parent_link != name)
        })
        .max_by_key(|&i| joints_to_root(&m, &m.links[i].name))
        .expect("some link is a leaf");
    let reach = joints_to_root(&m, &m.links[leaf].name);
    assert!(
        reach >= 3,
        "the deepest leaf is only {reach} joints from the root"
    );

    let w = poses[leaf].apply(&[0.0, 0.0, -0.05]);
    aba.prepare_transforms(&m, &q, &mut scratch);
    aba.constraint_force(
        &poses,
        leaf,
        [w[0], w[1], w[2]],
        // a direction off every axis, so a single unlucky alignment cannot make the answer zero
        [0.4082482904638631, 0.4082482904638631, 0.8164965809277261],
        &mut scratch,
        &mut tau,
    );

    let touched = tau.iter().filter(|t| **t != 0.0).count();
    assert!(
        touched > 0,
        "a force on link {leaf} produced no torque anywhere"
    );
    assert!(
        touched < n / 2,
        "a force on one leaf reached {touched} of {n} joints; it should reach only its own chain"
    );
}

#[test]
fn the_factorised_solve_gives_what_the_full_one_does() {
    // `factorise` stops the articulated recursion after the two sweeps that depend only on the
    // configuration, and `solve_factorised` finishes it per right-hand side. That is a refactor of
    // the hot path of the constraint solver, and the physics tests above would not catch a subtle
    // error in it — they exercise `accelerations`, which is the other path.
    //
    // So: the same torques, at rest, with gravity off, through both routes.
    let mut rng = Rng(0xFAC7);
    let mut checked = 0;
    for m in [
        Some(pendulum(0.8, 1.4)),
        Some(double_pendulum(0.5, 2.0, 0.4, 1.0)),
        Some(branched()),
        Some(fuzz_42278()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let mut scratch = aba.scratch();
        let quiet = vec![0.0; n];
        let mut full = vec![0.0; n];
        let mut fast = vec![0.0; n];

        for _ in 0..5 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.5).collect();
            aba.factorise(&m, &q, &mut scratch);

            // several right-hand sides against ONE factorisation, which is the whole point
            for _ in 0..4 {
                let tau: Vec<f64> = (0..n).map(|_| rng.signed() * 3.0).collect();
                aba.solve_factorised(&tau, &mut scratch, &mut fast);
                // the full route needs its own scratch state, so it goes second
                aba.accelerations(&m, &q, &quiet, &tau, [0.0; 3], &mut scratch, &mut full);

                let scale = full.iter().fold(1.0f64, |a, v| a.max(v.abs()));
                for k in 0..n {
                    assert!(
                        (fast[k] - full[k]).abs() < 1e-10 * scale,
                        "{}: slot {k}: factorised {} against full {}",
                        m.name,
                        fast[k],
                        full[k]
                    );
                }
                // and the factorisation survives being read again after the full solve clobbered
                // the scratch's bias fields
                aba.factorise(&m, &q, &mut scratch);
                checked += 1;
            }
        }
    }
    assert!(checked >= 60, "only {checked} solves were compared");
}

#[test]
fn the_restricted_sweep_agrees_with_the_full_one_everywhere_it_claims_to() {
    // `solve_factorised_over` sweeps only the links that can matter: up along the applied torque's
    // own path, down over the union of the paths any constraint will read. The claim is that where
    // it writes, it writes what the full sweep would — and the values outside are stale by design,
    // so the test must not look at them either.
    let mut rng = Rng(0x5A3E);
    let mut checked = 0;
    for m in [
        Some(branched()),
        Some(fuzz_42278()),
        robot("arm_gripper.hymeko", "robot"),
        robot("humanoid.hymeko", "humanoid"),
        robot("quadruped.hymeko", "quadruped"),
    ]
    .into_iter()
    .flatten()
    {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let plan = FkPlan::new(&m).expect("the topology resolves");
        let n = aba.slots();
        let mut scratch = aba.scratch();
        let mut full = vec![0.0; n];
        let mut restricted = vec![0.0; n];
        let mut poses = plan.identity_poses_se3();
        let mut force = vec![0.0; n];
        let mut reach = Vec::new();

        for round in 0..4 {
            let q: Vec<f64> = (0..n).map(|_| rng.signed() * 1.2).collect();
            plan.resolve_se3(&m, &q, &mut poses);
            aba.factorise(&m, &q, &mut scratch);

            // a couple of contacts, so the union is a real union and not one path
            let bodies: Vec<usize> = (0..3)
                .map(|k| (round * 5 + k * 7 + 1) % m.links.len())
                .collect();
            let mut union: Vec<bool> = vec![false; aba.links()];
            for &b in &bodies {
                aba.reach(b, &mut reach);
                for &link in &reach {
                    union[link] = true;
                }
            }
            let descend: Vec<usize> = aba
                .order()
                .iter()
                .copied()
                .filter(|&i| union[i])
                .collect();

            for &body in &bodies {
                let w = poses[body].apply(&[0.1, -0.05, 0.2]);
                aba.reach(body, &mut reach);
                aba.constraint_force(
                    &poses,
                    body,
                    [w[0], w[1], w[2]],
                    [0.408_248_290_463_863_1, 0.408_248_290_463_863_1, 0.816_496_580_927_726_1],
                    &mut scratch,
                    &mut force,
                );

                aba.solve_factorised_over(&force, &reach, &descend, &mut scratch, &mut restricted);
                aba.solve_factorised(&force, &mut scratch, &mut full);

                let scale = full.iter().fold(1.0f64, |a, v| a.max(v.abs()));
                for &link in &descend {
                    let Some(joint) = aba.inbound_joint(link) else {
                        continue;
                    };
                    assert!(
                        (restricted[joint] - full[joint]).abs() < 1e-11 * scale,
                        "{}: round {round}, body {body}, joint {joint}: restricted {} against full {}",
                        m.name,
                        restricted[joint],
                        full[joint]
                    );
                }
                // the factorisation survives, since the full solve above clobbered the bias fields
                aba.factorise(&m, &q, &mut scratch);
                checked += 1;
            }
        }
    }
    assert!(checked >= 40, "only {checked} sweeps were compared");
}

#[test]
fn a_reach_is_a_path_that_ends_at_the_root() {
    // The precondition the restricted sweep relies on: closed under taking parents, and finite.
    for m in [branched(), double_pendulum(0.5, 2.0, 0.4, 1.0)] {
        let aba = AbaPlan::new(&m).expect("the topology resolves");
        let mut reach = Vec::new();
        for body in 0..m.links.len() {
            aba.reach(body, &mut reach);
            assert_eq!(reach[0], body, "a reach starts at its own link");
            assert!(reach.len() <= m.links.len(), "a reach revisited a link");
            for pair in reach.windows(2) {
                let parent = aba
                    .kinematics_parent(pair[0])
                    .expect("every link but the last has a parent in the reach");
                assert_eq!(parent, pair[1], "a reach must step to the parent");
            }
        }
    }
}
