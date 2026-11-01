//! Structural fuzz of the dynamics: random mechanisms, not random configurations of fixed ones.
//!
//! The suite in `hymeko_mechanism/tests/articulated.rs` is strong on identities and thin on shapes.
//! It knows three hand-built fixtures and three real robots, and randomises the pose. Everything it
//! has never seen is structural: chains twenty deep, a root with eight children, a mass ratio of
//! `1e6` between neighbouring links, a centre of mass sitting exactly on the axis that is supposed
//! to swing it, a subtree with no mass at all hanging off one with plenty.
//!
//! This generates the mechanism too, and runs the same identities over it until told to stop. Every
//! case is addressed by a seed, so a failure is a line you can paste into a test.
//!
//! Two populations, because they deserve different tolerances and it is dishonest to mix them:
//!
//! * **well posed** — every link carries mass, every centre of mass sits off its own axis. The
//!   identities must hold TIGHTLY. A failure here is a bug.
//! * **adversarial** — massless links, masses on the axis, mass ratios spanning a million. The
//!   articulated inertia is legitimately singular in places and the floor takes over, so the
//!   quantitative identities are not owed. What IS owed is that nothing returns a NaN, nothing
//!   panics, and the mobility matrix stays symmetric — a wrong answer is a finding, a wrong SHAPE
//!   of answer is a bug regardless of conditioning.
//!
//! Run: `cargo run --release --example dynamics_fuzz -- [seconds] [start_seed]`

use std::time::{Duration, Instant};

use hymeko::common::ids::DeclId;
use kawakern::articulated::AbaPlan;
use hymeko_mechanism::forward::FkPlan;
use hymeko_mechanism::kinematics::joints::{JointInfo, JointType};
use hymeko_mechanism::kinematics::kinematic::{KinematicModel, LinkInfo};

const GRAV: [f64; 3] = [0.0, 0.0, -9.80665];

// --- the generator ------------------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        // splitmix so that consecutive seeds do not produce correlated streams
        let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        Self((z ^ (z >> 31)) | 1)
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn f64(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn signed(&mut self) -> f64 {
        2.0 * self.f64() - 1.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn vec3(&mut self) -> [f64; 3] {
        [self.signed(), self.signed(), self.signed()]
    }
    /// A unit vector, so a joint axis is never degenerate by accident.
    fn axis(&mut self) -> [f64; 3] {
        loop {
            let v = self.vec3();
            let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            if n > 1e-3 {
                return [v[0] / n, v[1] / n, v[2] / n];
            }
        }
    }
}

/// How the tree is shaped, which is the whole point of generating one.
#[derive(Clone, Copy, Debug)]
enum Shape {
    /// Every link hangs off the last — a twenty-deep chain is where transforms compound.
    Chain,
    /// Everything hangs off the root — where a parent accumulates many children's inertias.
    Star,
    /// Anything in between.
    Random,
}

fn generate(seed: u64, adversarial: bool) -> (KinematicModel, Shape) {
    let mut rng = Rng::new(seed);
    let n = 2 + rng.below(22); // 2..24 links
    let shape = match rng.below(3) {
        0 => Shape::Chain,
        1 => Shape::Star,
        _ => Shape::Random,
    };

    let mut links = Vec::with_capacity(n);
    let mut joints = Vec::with_capacity(n - 1);
    links.push(LinkInfo {
        inertia_com: None,
        did: DeclId::NONE,
        name: "l0".into(),
        mass: Some(1.0 + rng.f64()),
        geometry: None,
        origin: Some(vec![0.1, 0.0, 0.05]),
        color: None,
    });

    for i in 1..n {
        let parent = match shape {
            Shape::Chain => i - 1,
            Shape::Star => 0,
            Shape::Random => rng.below(i),
        };

        // mass: ordinary, or spanning six decades, or absent entirely
        let mass = if adversarial {
            match rng.below(4) {
                0 => None,
                1 => Some(1e-6 * (1.0 + rng.f64())),
                2 => Some(1e3 * (1.0 + rng.f64())),
                _ => Some(0.2 + 2.0 * rng.f64()),
            }
        } else {
            Some(0.2 + 2.0 * rng.f64())
        };

        let axis = rng.axis();
        // the centre of mass: off the axis where it must be resolvable, ON it where the articulated
        // inertia is meant to go singular
        let com = if adversarial && rng.below(3) == 0 {
            let d = rng.signed() * 0.4;
            [axis[0] * d, axis[1] * d, axis[2] * d]
        } else {
            // Push it clear of the axis, by a GUARANTEED distance and not merely a likely one.
            //
            // This used to scale down whatever perpendicular component the random vector happened to
            // have, which is a guarantee in expectation and nothing at all in the tail. An overnight
            // run of 233 million mechanisms found the tail: a vector that landed within a few
            // microradians of the axis produced a mass 1.4 um off it, in the population whose entire
            // definition is that no mass sits on its axis. It was reported as a solver failure, and
            // the solver was innocent.
            let raw = rng.vec3();
            let along = raw[0] * axis[0] + raw[1] * axis[1] + raw[2] * axis[2];
            let mut perp = [
                raw[0] - along * axis[0],
                raw[1] - along * axis[1],
                raw[2] - along * axis[2],
            ];
            let n = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
            if n < 1e-6 {
                // it landed on the axis; take any direction off it, deterministically
                let alt = if axis[0].abs() < 0.9 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 1.0, 0.0]
                };
                let d = alt[0] * axis[0] + alt[1] * axis[1] + alt[2] * axis[2];
                perp = [
                    alt[0] - d * axis[0],
                    alt[1] - d * axis[1],
                    alt[2] - d * axis[2],
                ];
            }
            let n = (perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]).sqrt();
            // at least 5 cm off the axis, up to 35
            let reach = 0.05 + 0.3 * rng.f64();
            [
                perp[0] / n * reach + 0.05 * axis[0],
                perp[1] / n * reach + 0.05 * axis[1],
                perp[2] / n * reach + 0.05 * axis[2],
            ]
        };

        links.push(LinkInfo {
            inertia_com: None,
            did: DeclId::NONE,
            name: format!("l{i}"),
            mass,
            geometry: None,
            origin: Some(com.to_vec()),
            color: None,
        });
        joints.push(JointInfo {
            did: DeclId::NONE,
            name: format!("j{i}"),
            joint_type: match rng.below(8) {
                0 => JointType::Fixed,
                1 => JointType::Prismatic,
                2 => JointType::Continuous,
                _ => JointType::Revolute,
            },
            parent: DeclId::NONE,
            child: DeclId::NONE,
            parent_link: format!("l{parent}"),
            child_link: format!("l{i}"),
            axis: Some(axis),
            origin_xyz: Some([rng.signed() * 0.4, rng.signed() * 0.4, rng.signed() * 0.4]),
            origin_rpy_deg: Some([
                rng.signed() * 180.0,
                rng.signed() * 180.0,
                rng.signed() * 180.0,
            ]),
            limits: None,
        });
    }

    (
        KinematicModel {
            actuation: Default::default(),
            name: format!("fuzz{seed}"),
            links,
            joints,
            frames: Vec::new(),
        },
        shape,
    )
}

// --- the checks ---------------------------------------------------------------------------------

struct Failure {
    seed: u64,
    what: String,
    detail: String,
}

fn check(seed: u64, adversarial: bool) -> Option<Failure> {
    let (model, shape) = generate(seed, adversarial);
    let population = if adversarial {
        "adversarial"
    } else {
        "well posed"
    };
    let fail = |what: &str, detail: String| {
        Some(Failure {
            seed,
            what: format!(
                "{what} [{population}, {shape:?}, {} links]",
                model.links.len()
            ),
            detail,
        })
    };

    let Ok(aba) = AbaPlan::new(&model) else {
        // a structural rejection is a legitimate answer, not a failure
        return None;
    };
    let Ok(plan) = FkPlan::new(&model) else {
        return fail("AbaPlan built a topology FkPlan refused", String::new());
    };
    let _ = plan;

    let mut rng = Rng::new(seed ^ 0xF0F0);
    let n = aba.slots();
    let slots: Vec<usize> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| !matches!(j.joint_type, JointType::Fixed))
        .map(|(i, _)| i)
        .collect();
    if slots.is_empty() {
        return None;
    }

    let mut scratch = aba.scratch();
    let q: Vec<f64> = (0..n).map(|_| rng.signed() * 2.0).collect();
    let qd: Vec<f64> = (0..n).map(|_| rng.signed() * 2.0).collect();
    let mut qdd = vec![0.0; n];
    for &s in &slots {
        qdd[s] = rng.signed() * 4.0;
    }
    let mut tau = vec![0.0; n];
    let mut back = vec![0.0; n];

    // --- nothing may be NaN, in either population ---
    aba.inverse_dynamics(&model, &q, &qd, &qdd, GRAV, &mut scratch, &mut tau);
    if let Some(k) = tau.iter().position(|v| !v.is_finite()) {
        return fail(
            "inverse dynamics produced a non-finite torque",
            format!("slot {k}"),
        );
    }
    aba.accelerations(&model, &q, &qd, &tau, GRAV, &mut scratch, &mut back);
    if let Some(k) = back.iter().position(|v| !v.is_finite()) {
        return fail(
            "forward dynamics produced a non-finite acceleration",
            format!("slot {k}"),
        );
    }

    // --- the mobility matrix must be symmetric, in BOTH populations ---
    // Symmetry is a shape, not a value: it follows from `M` being a metric, and no amount of bad
    // conditioning excuses losing it.
    let zero = vec![0.0; n];
    let mut unit = vec![0.0; n];
    let mut col = vec![0.0; n];
    let mut minv = vec![vec![0.0f64; slots.len()]; slots.len()];
    for (c, &s) in slots.iter().enumerate() {
        unit[s] = 1.0;
        aba.accelerations(&model, &q, &zero, &unit, [0.0; 3], &mut scratch, &mut col);
        unit[s] = 0.0;
        for (r, &rs) in slots.iter().enumerate() {
            minv[r][c] = col[rs];
        }
    }
    let scale = minv
        .iter()
        .flatten()
        .fold(0.0f64, |a, v| a.max(v.abs()))
        .max(1e-30);
    for (i, row) in minv.iter().enumerate() {
        for (j, &here) in row.iter().enumerate().take(i) {
            let mirror = minv[j][i];
            if (here - mirror).abs() > 1e-7 * scale {
                return fail(
                    "M^-1 is not symmetric",
                    format!("({i},{j}): {here:e} against {mirror:e}, scale {scale:e}"),
                );
            }
        }
    }

    // --- the three ways to get a mobility column must agree ------------------------------------
    //
    // `accelerations` at rest with no gravity, `factorise` + `solve_factorised`, and the same
    // restricted to the paths that can matter. Three code paths, one answer — and the restricted one
    // is new, subtle, and otherwise checked on five hand-picked mechanisms.
    //
    // Checked in BOTH populations: the sparse sweep must not disagree with the dense one just
    // because the inertia is badly conditioned. Where the floor takes over, all three take it over
    // together.
    {
        let mut reach = Vec::new();
        let mut touched = vec![false; model.links.len()];
        let bodies: Vec<usize> = (0..3)
            .map(|k| (rng.below(model.links.len().max(1)) + k) % model.links.len())
            .collect();
        for &b in &bodies {
            aba.reach(b, &mut reach);
            for &link in &reach {
                touched[link] = true;
            }
        }
        let descend: Vec<usize> = aba
            .order()
            .iter()
            .copied()
            .filter(|&i| touched[i])
            .collect();

        let mut tau = vec![0.0; n];
        for &s in &slots {
            tau[s] = rng.signed() * 2.0;
        }
        let mut full = vec![0.0; n];
        let mut factorised = vec![0.0; n];
        let mut restricted = vec![0.0; n];

        aba.factorise(&model, &q, &mut scratch);
        aba.solve_factorised(&tau, &mut scratch, &mut factorised);
        aba.factorise(&model, &q, &mut scratch);
        // the restricted sweep is only owed its answer where it writes, so `tau` is confined to the
        // same set — otherwise the two are not being asked the same question
        let mut confined = vec![0.0; n];
        for &link in &descend {
            if let Some(joint) = aba.inbound_joint(link) {
                confined[joint] = tau[joint];
            }
        }
        // `up` must run deepest first, so a child's contribution reaches its parent before the
        // parent is processed; `down` runs the other way. Handing the same ordering to both is the
        // obvious mistake and this fuzz found it within a minute of being asked.
        let ascend: Vec<usize> = descend.iter().rev().copied().collect();
        aba.solve_factorised_over(&confined, &ascend, &descend, &mut scratch, &mut restricted);
        aba.accelerations(
            &model,
            &q,
            &zero,
            &confined,
            [0.0; 3],
            &mut scratch,
            &mut full,
        );

        let scale = full.iter().fold(1.0f64, |a, v| a.max(v.abs()));
        for &link in &descend {
            let Some(joint) = aba.inbound_joint(link) else {
                continue;
            };
            if !(restricted[joint] - full[joint]).abs().le(&(1e-9 * scale)) {
                return fail(
                    "the restricted sweep disagrees with the full solve",
                    format!(
                        "joint {joint}: restricted {:e}, full {:e}, scale {scale:e}",
                        restricted[joint], full[joint]
                    ),
                );
            }
        }
        // and the unrestricted factorised path against the same, over every joint
        aba.factorise(&model, &q, &mut scratch);
        aba.solve_factorised(&confined, &mut scratch, &mut factorised);
        for &s in &slots {
            if !(factorised[s] - full[s]).abs().le(&(1e-9 * scale)) {
                return fail(
                    "the factorised solve disagrees with the full one",
                    format!(
                        "slot {s}: factorised {:e}, full {:e}",
                        factorised[s], full[s]
                    ),
                );
            }
        }
    }

    if adversarial {
        // The quantitative identities are not owed where the inertia floor is doing the work.
        return None;
    }

    // --- inverse and forward must undo each other, tightly ---
    let want = qdd.iter().fold(1.0f64, |a, v| a.max(v.abs()));
    for &s in &slots {
        if (back[s] - qdd[s]).abs() > 1e-6 * want {
            return fail(
                "the round trip lost the acceleration",
                format!("slot {s}: asked {:e}, got {:e}", qdd[s], back[s]),
            );
        }
    }

    // --- a diagonal of M^-1 must be positive: a torque accelerates its own joint forwards ---
    for (i, &s) in slots.iter().enumerate() {
        if minv[i][i] <= 0.0 {
            return fail(
                "M^-1 has a non-positive diagonal",
                format!("slot {s}: {:e}", minv[i][i]),
            );
        }
    }

    // --- gravity compensation must hold it still ---
    aba.inverse_dynamics(&model, &q, &zero, &zero, GRAV, &mut scratch, &mut tau);
    aba.accelerations(&model, &q, &zero, &tau, GRAV, &mut scratch, &mut back);
    aba.accelerations(&model, &q, &zero, &zero, GRAV, &mut scratch, &mut col);
    let falling = col.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    let held = back.iter().fold(0.0f64, |a, v| a.max(v.abs()));
    if held > 1e-7 * falling.max(1.0) {
        return fail(
            "gravity compensation did not hold it still",
            format!("held at {held:e} while it would have fallen at {falling:e}"),
        );
    }

    None
}

// --- the exponential map, over the same seeds ------------------------------------------------

// `!(a < b)` rather than `a >= b` throughout, and deliberately: a NaN compares false to everything,
// so the negated form catches it and the direct form silently passes it. In a fuzzer that is the
// difference between finding a bug and reporting a clean run.
#[allow(clippy::neg_cmp_op_on_partial_ord)]
fn check_exponential(seed: u64) -> Option<Failure> {
    use hymeko_clifford::Twist;
    let mut rng = Rng::new(seed ^ 0xE4E4);
    for _ in 0..8 {
        // sweep the magnitude across the series crossover and out to the wrap at pi
        let decade = rng.below(14) as i32;
        let magnitude = 10f64.powi(-decade) * 3.0;
        let a = rng.axis();
        let t = Twist {
            w: [a[0] * magnitude, a[1] * magnitude, a[2] * magnitude],
            v: rng.vec3(),
        };
        let round = t.exp().log();
        let err = (0..3)
            .map(|k| (round.w[k] - t.w[k]).abs().max((round.v[k] - t.v[k]).abs()))
            .fold(0.0f64, f64::max);
        let scale = (0..3)
            .map(|k| t.w[k].abs().max(t.v[k].abs()))
            .fold(1.0f64, f64::max);
        if !(err < 1e-9 * scale) {
            return Some(Failure {
                seed,
                what: "log(exp) round trip".into(),
                detail: format!("|w| = {magnitude:e}, error {err:e}, scale {scale:e}"),
            });
        }

        // and the one-parameter subgroup law
        let split = t.scale(0.4).exp().compose(&t.scale(0.6).exp());
        let whole = t.exp();
        for k in 0..3 {
            let d = (split.translation[k] - whole.translation[k]).abs();
            if !(d < 1e-9 * (1.0 + whole.translation[k].abs())) {
                return Some(Failure {
                    seed,
                    what: "exp(0.4 v) exp(0.6 v) != exp(v)".into(),
                    detail: format!("|w| = {magnitude:e}, translation[{k}] off by {d:e}"),
                });
            }
        }
    }
    None
}

// --- the run ------------------------------------------------------------------------------------

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let seconds: u64 = args.first().and_then(|s| s.parse().ok()).unwrap_or(10);
    let start: u64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(1);

    // Emit a finding as Rust source, so "a failure is a line you can paste into a test" is literally
    // true. A hand-written approximation of a failing model is not a regression guard — mine passed
    // against the very bug it was written for, because the axes and origins matter as much as the
    // masses.
    if args.first().map(|s| s == "--dump").unwrap_or(false) {
        let only: u64 = args[1].parse().expect("a seed");
        let adversarial = args.get(2).map(|s| s == "adversarial").unwrap_or(true);
        let (m, shape) = generate(only, adversarial);
        println!(
            "/// Generated by `dynamics_fuzz --dump {only}` ({shape:?}, {} links).",
            m.links.len()
        );
        println!("fn fuzz_{only}() -> KinematicModel {{");
        println!("    let links = vec![");
        for l in &m.links {
            let c = l.origin.as_ref().unwrap();
            let mass = match l.mass {
                Some(v) => format!("Some({v:?})"),
                None => "None".into(),
            };
            println!(
                "        link({:?}, {mass}, [{:?}, {:?}, {:?}]),",
                l.name, c[0], c[1], c[2]
            );
        }
        println!("    ];");
        println!("    let joints = vec![");
        for j in &m.joints {
            let a = j.axis.unwrap();
            let o = j.origin_xyz.unwrap();
            let r = j.origin_rpy_deg.unwrap();
            println!(
                "        joint({:?}, {:?}, {:?}, JointType::{:?}, [{:?}, {:?}, {:?}], [{:?}, {:?}, {:?}], [{:?}, {:?}, {:?}]),",
                j.name,
                j.parent_link,
                j.child_link,
                j.joint_type,
                a[0],
                a[1],
                a[2],
                o[0],
                o[1],
                o[2],
                r[0],
                r[1],
                r[2]
            );
        }
        println!("    ];");
        println!("    model({:?}, links, joints)", format!("fuzz{only}"));
        println!("}}");
        return;
    }

    if args.first().map(|s| s == "--seed").unwrap_or(false) {
        let only: u64 = args[1].parse().expect("a seed");
        for adversarial in [false, true] {
            let (m, shape) = generate(only, adversarial);
            let masses: Vec<String> = m
                .links
                .iter()
                .map(|l| match l.mass {
                    Some(v) => format!("{v:.2e}"),
                    None => "-".into(),
                })
                .collect();
            println!(
                "seed {only} {}: {shape:?}, {} links, masses [{}]",
                if adversarial {
                    "adversarial"
                } else {
                    "well posed"
                },
                m.links.len(),
                masses.join(" ")
            );
            match check(only, adversarial) {
                Some(f) => println!("  FAILS: {}  --  {}", f.what, f.detail),
                None => println!("  passes"),
            }
        }
        return;
    }

    println!("dynamics fuzz: {seconds} s from seed {start}");
    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut seed = start;
    let (mut well, mut adv, mut expo) = (0u64, 0u64, 0u64);
    let mut failures: Vec<Failure> = Vec::new();
    let mut last_report = Instant::now();

    while Instant::now() < deadline && failures.len() < 20 {
        for _ in 0..200 {
            if let Some(f) = check(seed, false) {
                failures.push(f);
            }
            well += 1;
            if let Some(f) = check(seed, true) {
                failures.push(f);
            }
            adv += 1;
            if let Some(f) = check_exponential(seed) {
                failures.push(f);
            }
            expo += 1;
            seed = seed.wrapping_add(1);
        }
        if last_report.elapsed() > Duration::from_secs(300) {
            println!(
                "  ... {well} well posed, {adv} adversarial, {expo} exponential, \
                 {} failures, at seed {seed}",
                failures.len()
            );
            last_report = Instant::now();
        }
    }

    println!(
        "  {well} well posed, {adv} adversarial, {expo} exponential mechanisms, \
         seeds {start}..{seed}"
    );
    if failures.is_empty() {
        println!("  no failures");
    } else {
        println!("  {} FAILURES:", failures.len());
        for f in &failures {
            println!("    seed {:<12} {}  --  {}", f.seed, f.what, f.detail);
        }
        std::process::exit(1);
    }
}
