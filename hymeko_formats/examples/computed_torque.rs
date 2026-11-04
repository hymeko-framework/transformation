//! What the linear-time dynamics is actually FOR: a robot that follows a trajectory.
//!
//! Two controllers drive the same declared arm along the same reference motion, integrated by the
//! same forward solver at the same rate. The only difference is what they know about the mechanism.
//!
//!   PD           tau = Kp e + Kd edot
//!                A spring and a damper per joint. It knows nothing about the arm — not that a
//!                shoulder carries the whole limb while a wrist carries a gripper, not that gravity
//!                pulls harder at some angles than others, not that swinging one link throws the
//!                next one. Every one of those is a disturbance it can only react to after the fact.
//!
//!   computed     tau = InverseDynamics(q, qdot, qddot_ref + Kp e + Kd edot)
//!   torque       The same feedback, but asked of the MECHANISM rather than of each joint alone.
//!                Gravity, the varying inertia, and the coupling between links are all computed
//!                in closed form and cancelled before they become error.
//!
//! The second is only affordable because inverse dynamics is `O(n)`: it runs once per control tick,
//! and a tick is 1 ms.
//!
//! Run: `cargo run --release --example computed_torque -- <robot.hymeko> <root>`

use std::time::Instant;

use kawakern::articulated::AbaPlan;
use hymeko_mechanism::kinematics::joints::JointType;
use hymeko_mechanism::kinematics::kinematic::{KinematicModel, extract_kinematic_model};
use hymeko_query::QueryEngine;

/// Control rate. A real arm runs its inner loop here or faster.
const RATE: f64 = 1000.0;
const SECONDS: f64 = 4.0;
const GRAVITY: [f64; 3] = [0.0, 0.0, -9.80665];

/// The gain ladder both controllers are swept over.
///
/// Neither is handed a tuning. Each is run at every gain and reported at its own BEST STABLE one,
/// which is the only fair comparison: a fixed pair would either flatter whichever it happened to
/// suit or, worse, be the gain at which one of them diverges.
///
/// It reaches absurdly low so that plain PD can report SOMETHING rather than only diverging — a
/// number it tracks badly at is more informative than a failure.
///
/// The ladder is where the difference shows up as more than an error figure. Explicit integration of
/// a damper needs `dt < 2I/Kd`, and `I` — the diagonal of the mass matrix — varies by orders of
/// magnitude across one arm, because a shoulder carries the whole limb and a fingertip carries
/// almost nothing. So no single gain suits both ends: soft enough for the fingers is far too soft to
/// hold the shoulder up. Computed torque divides the inertia out before the feedback ever sees it,
/// and the same ladder behaves completely differently.
///
/// The usual workaround — hand-tuning a gain per joint, scaled by that joint's inertia — is
/// computed torque with the model kept in the tuner's head instead of in the program, and it stops
/// working the moment the inertia depends on the configuration, which it does.
const LADDER: [f64; 11] = [
    0.0025, 0.01, 0.1, 1.0, 10.0, 25.0, 100.0, 200.0, 400.0, 800.0, 1600.0,
];

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
    let slots: Vec<usize> = model
        .joints
        .iter()
        .enumerate()
        .filter(|(_, j)| !matches!(j.joint_type, JointType::Fixed))
        .map(|(i, _)| i)
        .collect();

    println!(
        "  {root}: {} links, {} movable joints, control at {RATE:.0} Hz for {SECONDS:.0} s",
        model.links.len(),
        slots.len()
    );

    // Why the two behave so differently is not a matter of tuning, and the arm itself says so.
    // A joint's own inertia is the diagonal of M, and explicit integration of a damper needs
    // `dt < 2I/Kd`. Across one arm that I spans orders of magnitude — a shoulder carries the whole
    // limb, a fingertip carries almost nothing — so no single Kd can suit both ends.
    let mut scratch = aba.scratch();
    let mut unit = vec![0.0; aba.slots()];
    let mut out = vec![0.0; aba.slots()];
    let (mut lo, mut hi) = (f64::INFINITY, 0.0f64);
    let mut degenerate = 0;
    for &s in &slots {
        unit[s] = 1.0;
        aba.inverse_dynamics(
            &model,
            &vec![0.0; aba.slots()],
            &vec![0.0; aba.slots()],
            &unit,
            [0.0; 3],
            &mut scratch,
            &mut out,
        );
        unit[s] = 0.0;
        let i = out[s].abs();
        if i < 1e-9 {
            degenerate += 1;
            continue;
        }
        lo = lo.min(i);
        hi = hi.max(i);
    }
    println!(
        "  joint inertia spans {lo:.2e} to {hi:.2e} kg m^2 ({:.0}x) -- a plain damper is stable only below Kd = {:.2} at this rate",
        hi / lo,
        2.0 * lo * RATE
    );
    if degenerate > 0 {
        // Not a solver problem: the model declares a joint whose only mass sits exactly on its own
        // rotation axis, so nothing resists it. Worth saying out loud, because a controller will
        // find it long before a person reading the description does.
        println!(
            "  {degenerate} joint(s) have NO inertia about their own axis -- their mass is declared on the axis"
        );
    }

    let pd = best(&model, &aba, &slots, false);
    let ct = best(&model, &aba, &slots, true);

    for (label, r) in [("PD only        ", &pd), ("computed torque", &ct)] {
        match r {
            Some(r) => println!(
                "  {label}  Kp {:>8.4}  rms {:>9.5} rad   worst {:>9.5} rad   {:>6.2} us/tick   (stable to Kp {:.4})",
                r.kp, r.rms, r.worst, r.us, r.stable_to
            ),
            None => println!("  {label}  diverged at every gain on the ladder"),
        }
    }
    if let (Some(p), Some(c)) = (&pd, &ct) {
        println!(
            "  -> {:.0}x tighter tracking, for {:+.2} us per tick ({:.1}% of the 1 ms budget), and stable {:.0}x further up the ladder",
            p.rms / c.rms.max(1e-12),
            c.us - p.us,
            100.0 * (c.us - p.us) / 1000.0,
            c.stable_to / p.stable_to
        );
    }
}

struct Report {
    kp: f64,
    rms: f64,
    worst: f64,
    us: f64,
    /// The highest gain on the ladder this controller survived.
    stable_to: f64,
}

/// Sweep the ladder and keep the tightest tracking that did not diverge.
fn best(model: &KinematicModel, aba: &AbaPlan, slots: &[usize], computed: bool) -> Option<Report> {
    let mut winner: Option<Report> = None;
    let mut stable_to = 0.0f64;
    for kp in LADDER {
        // critically damped for a unit inertia, which is the textbook starting point and keeps the
        // ladder a one-parameter sweep rather than a search
        let kd = 2.0 * kp.sqrt();
        let Some(r) = run(model, aba, slots, computed, kp, kd) else {
            continue;
        };
        stable_to = stable_to.max(kp);
        if winner.as_ref().is_none_or(|w| r.rms < w.rms) {
            winner = Some(r);
        }
    }
    winner.map(|mut w| {
        w.stable_to = stable_to;
        w
    })
}

/// The reference trajectory: every joint sweeps a sine, at a different rate so the arm actually has
/// to coordinate rather than move as one rigid piece.
fn reference(slot_index: usize, t: f64) -> (f64, f64, f64) {
    let amp = 0.6;
    let w = 2.0 * std::f64::consts::PI * (0.35 + 0.06 * slot_index as f64);
    let phase = 0.4 * slot_index as f64;
    (
        amp * (w * t + phase).sin(),
        amp * w * (w * t + phase).cos(),
        -amp * w * w * (w * t + phase).sin(),
    )
}

/// One run. `None` if it diverged — which is a result, not a failure.
fn run(
    model: &KinematicModel,
    aba: &AbaPlan,
    slots: &[usize],
    computed: bool,
    kp: f64,
    kd: f64,
) -> Option<Report> {
    let n = aba.slots();
    let mut scratch = aba.scratch();
    let mut q = vec![0.0; n];
    let mut qd = vec![0.0; n];
    let mut tau = vec![0.0; n];
    let mut want = vec![0.0; n];
    let mut qdd = vec![0.0; n];

    // start ON the trajectory, so what is measured is tracking and not a transient
    for (k, &s) in slots.iter().enumerate() {
        let (r, rd, _) = reference(k, 0.0);
        q[s] = r;
        qd[s] = rd;
    }

    let dt = 1.0 / RATE;
    let steps = (SECONDS * RATE) as usize;
    let mut sum_sq = 0.0;
    let mut worst: f64 = 0.0;
    let mut samples = 0usize;

    let t0 = Instant::now();
    for step in 0..steps {
        let t = step as f64 * dt;

        // --- the control law ---
        for (k, &s) in slots.iter().enumerate() {
            let (r, rd, rdd) = reference(k, t);
            let e = r - q[s];
            let ed = rd - qd[s];
            if computed {
                // ask the mechanism, not the joint: this is the acceleration we want
                want[s] = rdd + kp * e + kd * ed;
            } else {
                tau[s] = kp * e + kd * ed;
            }
        }
        if computed {
            aba.inverse_dynamics(model, &q, &qd, &want, GRAVITY, &mut scratch, &mut tau);
        }

        // --- the plant ---
        aba.accelerations(model, &q, &qd, &tau, GRAVITY, &mut scratch, &mut qdd);
        for &s in slots {
            qd[s] += qdd[s] * dt;
            q[s] += qd[s] * dt;
            // Divergence is the VELOCITY running away, not the angle getting large: these joints
            // have no limits, so a controller too soft to hold the arm lets it spin, and the angle
            // then grows without bound while nothing is numerically wrong. That is a terrible
            // result, and it must be reported as a terrible result rather than as a failure to run.
            if !q[s].is_finite() || qd[s].abs() > 1e6 {
                return None;
            }
        }

        // --- what it cost us ---
        // Skipping the first tenth of a second lets the two share a fair starting point even though
        // neither is exactly on the trajectory after one step.
        if t > 0.1 {
            for (k, &s) in slots.iter().enumerate() {
                let (r, _, _) = reference(k, t + dt);
                let e = (r - q[s]).abs();
                sum_sq += e * e;
                worst = worst.max(e);
                samples += 1;
            }
        }
    }
    let us = t0.elapsed().as_secs_f64() * 1e6 / steps as f64;

    Some(Report {
        kp,
        rms: (sum_sq / samples as f64).sqrt(),
        worst,
        us,
        stable_to: kp,
    })
}
