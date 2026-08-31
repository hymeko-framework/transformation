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
