use hymeko::{module_store::execution::CompiledModel, util::real_parser::RealParser};
use hymeko_dynamics::scenario::{RefusalCode, inspection::*};
use hymeko_sources::MemoryProvider;
use std::path::Path;

const SOURCE: &str = include_str!("fixtures/inspection/task.hymeko");
const META: &str = include_str!("fixtures/inspection/meta.hymeko");
const DT: u64 = 100_000_000;

struct Runner {
    model: CompiledModel,
    session: InspectionSession,
    sequence: u64,
    time: u64,
}
impl Runner {
    fn open(source: &str) -> Result<InspectionSession, hymeko_dynamics::scenario::Refusal> {
        Self::build(source).map(|(_, session)| session)
    }
    fn build(
        source: &str,
    ) -> Result<(CompiledModel, InspectionSession), hymeko_dynamics::scenario::Refusal> {
        let model = CompiledModel::compile(
            MemoryProvider::new()
                .with("task.hymeko", source)
                .with("meta.hymeko", META),
            RealParser,
            Path::new("task.hymeko"),
        )
        .unwrap();
        Self::admit(model)
    }
    fn admit(
        model: CompiledModel,
    ) -> Result<(CompiledModel, InspectionSession), hymeko_dynamics::scenario::Refusal> {
        let root = model
            .program()
            .ir
            .decl_nodes
            .iter()
            .position(|d| model.interner().resolve(d.name) == "inspection")
            .unwrap();
        let session = InspectionSession::open(&model.program().ir, model.interner(), root)?;
        Ok((model, session))
    }
    fn new() -> Self {
        let (model, session) = Self::build(SOURCE).unwrap();
        Self {
            model,
            session,
            sequence: 0,
            time: 0,
        }
    }
    fn sample(&mut self, pose: Pose) -> PoseObservation {
        self.sequence += 1;
        self.time += DT;
        PoseObservation {
            source: "gazebo-ur5".into(),
            epoch: "run-a".into(),
            frame: "world".into(),
            sequence: self.sequence,
            acquired_ns: self.time,
            pose,
        }
    }
    fn observe(&mut self, pose: Pose) -> InspectionSnapshot {
        let sample = self.sample(pose);
        self.session.observe(sample, self.time).unwrap()
    }
    fn at_target(&mut self) -> InspectionSnapshot {
        self.observe(self.session.target().unwrap().pose)
    }
    fn qualify(&mut self) -> InspectionSnapshot {
        let pose = self.session.target().unwrap().pose;
        for _ in 0..6 {
            self.observe(pose);
        }
        self.observe(pose)
    }
    fn projected(&self) -> CompiledModel {
        self.model
            .with_value_updates(&self.session.value_updates())
            .unwrap()
    }
    fn output<'a>(model: &'a CompiledModel, name: &str) -> &'a hymeko::ir::ir::ValueR {
        let ir = &model.program().ir;
        let node = ir
            .decl_nodes
            .iter()
            .find(|node| {
                model.interner().resolve(node.name) == name
                    && node.parent.is_some()
                    && model
                        .interner()
                        .resolve(ir.decl_nodes[node.parent.raw()].name)
                        == "observed"
            })
            .unwrap();
        ir[node.anno].value.as_ref().unwrap()
    }
    fn number(model: &CompiledModel, name: &str) -> f64 {
        let hymeko::ir::ir::ValueR::Num(value) = Self::output(model, name) else {
            panic!("numeric output required")
        };
        *value
    }
}

#[test]
fn declaration_order_pose_and_units_survive_projection() {
    let r = Runner::new();
    let model = r.session.model();
    assert_eq!(model.source, "gazebo-ur5");
    assert_eq!(model.frame, "world");
    assert_eq!(model.cycles, 4);
    assert_eq!(model.max_gap_ns, 200_000_000);
    assert_eq!(model.expiry_ns, 500_000_000);
    assert_eq!(
        model
            .stations
            .iter()
            .map(|s| s.pose.position_m)
            .collect::<Vec<_>>(),
        [[0.1, 0., 0.], [0., 0.1, 0.], [0., 0., 0.1]]
    );
    assert_eq!(model.home.pose.position_m, [0.; 3]);
    assert_eq!(model.target_at(12).unwrap().identity.name, "home_pose");
    assert!(model.target_at(13).is_none());
    for station in &model.stations {
        assert_eq!(station.pose.quaternion_xyzw, [0., 0., 0., 1.]);
        assert_eq!(station.criteria.dwell_ns, 500_000_000);
        assert_eq!(station.criteria.speed_m_s, 0.02);
    }
}

#[test]
fn explicit_source_loss_blocks_without_advancing_the_source_clock() {
    let mut r = Runner::new();
    r.at_target();
    let time = r.session.snapshot().acquired_ns;
    let blocked = r.session.invalidate().unwrap();
    assert_eq!(blocked.acquired_ns, time);
    assert_eq!(blocked.phase, InspectionPhase::Blocked);
    assert!(!blocked.fresh);
    assert!(r.session.resume(r.time).is_err());
    assert_eq!(r.at_target().phase, InspectionPhase::Blocked);
    r.session.resume(r.time).unwrap();
    assert_eq!(r.qualify().completed_visits, 1);
}

#[test]
fn imported_target_paths_keep_the_canonical_namespace() {
    let model = CompiledModel::compile(
        MemoryProvider::new()
            .with("wrapper.hymeko", "wrapper { @\"task.hymeko\" -> demo; }")
            .with("task.hymeko", SOURCE)
            .with("meta.hymeko", META),
        RealParser,
        Path::new("wrapper.hymeko"),
    )
    .unwrap();
    let (model, task) = Runner::admit(model).unwrap();
    assert_eq!(task.target().unwrap().identity.name, "demo.points.a");
    let projected = model.with_value_updates(&task.value_updates()).unwrap();
    let hymeko::ir::ir::ValueR::Str(path) = Runner::output(&projected, "target_path") else {
        panic!("target path")
    };
    assert_eq!(projected.interner().resolve(*path), "demo.points.a");
}

#[test]
fn compiled_task_finishes_twelve_visits_and_observed_home_once() {
    let mut r = Runner::new();
    assert_eq!(r.session.model().required_visits(), 12);
    assert_eq!(r.session.snapshot().phase, InspectionPhase::Missing);
    for completed in 1..=12 {
        let snapshot = r.qualify();
        assert_eq!(snapshot.completed_visits, completed);
        assert!(!snapshot.returned_home);
    }
    assert_eq!(r.session.snapshot().phase, InspectionPhase::Returning);
    let home = r.session.target().unwrap().pose;
    assert_eq!(r.qualify().phase, InspectionPhase::Complete);
    assert!(r.session.target().is_none());
    assert_eq!(r.observe(home).revision, 13);
    assert!(r.session.tick(r.time + 10 * DT).unwrap().returned_home);
    let trace = r.session.transitions(Some(0)).unwrap();
    assert_eq!(trace.events.len(), 13);
    assert_eq!(
        trace.observers.iter().map(|o| o.value).collect::<Vec<_>>(),
        [12, 1]
    );
    assert!(trace.invariants.iter().all(|i| i.valid));
}

#[test]
fn invalid_schema_and_workflow_are_refused_by_the_consumer() {
    for (from, to) in [
        ("cycles (CYCLES)", "cycles 1.5"),
        ("max_gap_s 0.2", "max_gap_s 0.6"),
        ("expiry_s 0.5", "expiry_s 0"),
        ("completed_visits 0", "completed_visits \"zero\""),
        ("position_m 0.01", "position_mm 0.01"),
        (
            "stations [points.a, points.b, points.c]",
            "stations [points.a, points.a]",
        ),
        ("k.inspection_task", "k.inspection_station"),
        ("capacity (VISITS + 1)", "capacity 100"),
        ("op \"add\"; value 1", "op \"set\"; value 1"),
        (
            "quaternion_xyzw [0, 0, 0, 1]",
            "quaternion_xyzw [0, 0, 0, 2]",
        ),
    ] {
        assert!(
            Runner::open(&SOURCE.replace(from, to)).is_err(),
            "{from} -> {to}"
        );
    }
}

#[test]
fn canonical_value_updates_preserve_topology_and_exact_large_stamps() {
    use hymeko::{ir::ir::ValueR, module_store::execution::ValueUpdate};
    let mut r = Runner::new();
    let initial = r.projected();
    assert_eq!(Runner::number(&initial, "sample_present"), 0.);
    assert_eq!(Runner::number(&initial, "required_visits"), 12.);
    let pose = r.session.target().unwrap().pose;
    let time = u64::MAX - 2 * DT;
    let sample = PoseObservation {
        source: "gazebo-ur5".into(),
        epoch: "large-stamp".into(),
        frame: "world".into(),
        sequence: u64::MAX - 1,
        acquired_ns: time,
        pose,
    };
    r.session.observe(sample, time).unwrap();
    let current = r.projected();
    let sequence = ((Runner::number(&current, "sequence_high") as u64) << 32)
        | Runner::number(&current, "sequence_low") as u64;
    let stamp = Runner::number(&current, "acquired_seconds") as u64 * 1_000_000_000
        + Runner::number(&current, "acquired_nanoseconds") as u64;
    assert_eq!(sequence, u64::MAX - 1);
    assert_eq!(stamp, time);
    assert_eq!(
        current.program().ir.decl_nodes.len(),
        r.model.program().ir.decl_nodes.len()
    );
    assert!(
        matches!(Runner::output(&current, "target"), ValueR::Ref(id) if id.raw() == r.session.target().unwrap().identity.id)
    );
    assert_eq!(
        r.session
            .value_updates()
            .iter()
            .filter(|u| matches!(u, ValueUpdate::Text { .. }))
            .count(),
        3
    );
    assert_eq!(Runner::number(&current, "fresh"), 1.);
    assert_eq!(Runner::number(&current, "speed_valid"), 0.);
}

#[test]
fn projected_progress_and_stale_flags_are_observed_not_predicted() {
    let mut r = Runner::new();
    r.qualify();
    let current = r.projected();
    assert_eq!(Runner::number(&current, "completed_visits"), 1.);
    assert_eq!(Runner::number(&current, "evidence_valid"), 1.);
    r.session
        .tick(r.time + r.session.model().expiry_ns + 1)
        .unwrap();
    let stale = r.projected();
    assert_eq!(Runner::number(&stale, "completed_visits"), 1.);
    assert_eq!(Runner::number(&stale, "evidence_valid"), 0.);
    assert_eq!(Runner::number(&stale, "qualified"), 0.);
    let hymeko::ir::ir::ValueR::Str(phase) = Runner::output(&stale, "phase") else {
        panic!("phase text")
    };
    assert_eq!(stale.interner().resolve(*phase), "blocked");
}

#[test]
fn duplicate_epoch_frame_and_invalid_pose_never_change_state() {
    let mut r = Runner::new();
    let pose = r.session.target().unwrap().pose;
    let first = r.sample(pose);
    r.session.observe(first.clone(), r.time).unwrap();
    let before = r.session.snapshot();
    assert!(r.session.observe(first, r.time + DT).is_err());
    for mutation in 0..8 {
        let mut sample = r.sample(pose);
        match mutation {
            0 => sample.epoch = "run-b".into(),
            1 => sample.source = "other".into(),
            2 => sample.frame = "room".into(),
            3 => sample.pose.position_m[0] = f64::NAN,
            4 => sample.pose.quaternion_xyzw = [0.; 4],
            5 => sample.acquired_ns = 0,
            6 => sample.acquired_ns = r.time + 1,
            _ => sample.sequence = 0,
        }
        assert!(r.session.observe(sample, r.time).is_err());
        assert_eq!(r.session.snapshot(), before);
    }
    assert!(r.session.tick(0).is_err());
    assert_eq!(r.session.snapshot(), before);
}

#[test]
fn sample_gap_breaks_dwell_and_expiry_latches_until_explicit_resume() {
    let mut r = Runner::new();
    for _ in 0..4 {
        r.at_target();
    }
    assert_eq!(r.session.snapshot().evidence.unwrap().dwell_ns, 2 * DT);
    r.time += 2 * DT;
    assert_eq!(r.at_target().evidence.unwrap().dwell_ns, 0);
    r.time += 6 * DT;
    assert_eq!(
        r.session.tick(r.time).unwrap().phase,
        InspectionPhase::Blocked
    );
    let before = r.session.snapshot();
    assert!(r.session.resume(r.time).is_err());
    assert_eq!(r.session.snapshot(), before);
    assert_eq!(r.at_target().phase, InspectionPhase::Blocked);
    assert_eq!(r.at_target().completed_visits, 0);
    assert_eq!(
        r.session.resume(r.time).unwrap().phase,
        InspectionPhase::Approaching
    );
    assert!(r.session.resume(r.time).is_err());
    assert_eq!(r.qualify().completed_visits, 1);
}

#[test]
fn fresh_sample_after_outage_does_not_silently_resume() {
    let mut r = Runner::new();
    r.at_target();
    r.time += 6 * DT;
    assert_eq!(r.at_target().phase, InspectionPhase::Blocked);
    assert!(r.session.snapshot().fresh);
    assert_eq!(r.session.transitions(Some(0)).unwrap().events.len(), 1);
}

#[test]
fn quaternion_sign_is_equivalent_but_orientation_and_speed_fail_qualification() {
    let mut r = Runner::new();
    let mut pose = r.session.target().unwrap().pose;
    pose.quaternion_xyzw = [0., 0., 0., -1.];
    for _ in 0..6 {
        r.observe(pose);
    }
    assert_eq!(r.observe(pose).completed_visits, 1);
    let mut wrong = r.session.target().unwrap().pose;
    wrong.quaternion_xyzw = [0., 0., 1., 0.];
    for _ in 0..8 {
        assert_eq!(r.observe(wrong).completed_visits, 1);
    }
    assert!(r.session.snapshot().evidence.unwrap().orientation_error_rad > 3.);
    let target = r.session.target().unwrap().pose;
    r.observe(target);
    let mut near = target;
    near.position_m[0] += 0.009;
    let evidence = r.observe(near).evidence.unwrap();
    assert!(evidence.position_error_m < 0.01);
    assert!(!evidence.qualified);
}

#[test]
fn boundary_timestamps_and_state_history_remain_deterministic() {
    let mut a = Runner::new();
    let mut b = Runner::new();
    for _ in 0..3 {
        assert_eq!(a.qualify(), b.qualify());
    }
    assert_eq!(
        a.session.transitions(Some(0)).unwrap(),
        b.session.transitions(Some(0)).unwrap()
    );
    let at_expiry = a.time + a.session.model().expiry_ns;
    assert!(a.session.tick(at_expiry).unwrap().fresh);
    assert_eq!(
        a.session.tick(at_expiry + 1).unwrap().phase,
        InspectionPhase::Blocked
    );
    assert_eq!(a.session.transitions(Some(3)).unwrap().events.len(), 1);
}

#[test]
fn history_budget_refusal_is_explicit_and_does_not_award_progress() {
    let mut r = Runner::new();
    assert_eq!(
        r.session.set_log_budget(0).unwrap_err().code,
        RefusalCode::Limit
    );
    r.session.set_log_budget(r.session.log_bytes()).unwrap();
    let pose = r.session.target().unwrap().pose;
    for _ in 0..6 {
        r.observe(pose);
    }
    let sample = r.sample(pose);
    let before = r.session.snapshot();
    assert_eq!(
        r.session.observe(sample.clone(), r.time).unwrap_err().code,
        RefusalCode::Limit
    );
    assert_eq!(r.session.snapshot(), before);
    r.session.set_log_budget(16 * 1024 * 1024).unwrap();
    assert_eq!(
        r.session.observe(sample, r.time).unwrap().completed_visits,
        1
    );
}

#[test]
fn tiny_angular_tolerances_do_not_round_a_wrong_pose_into_success() {
    let source = SOURCE.replace(
        "orientation_rad 0.08726646259971647",
        "orientation_rad 0.000000001",
    );
    let (model, session) = Runner::build(&source).unwrap();
    let mut r = Runner {
        model,
        session,
        sequence: 0,
        time: 0,
    };
    let mut pose = r.session.target().unwrap().pose;
    pose.quaternion_xyzw = [0., 0., 1e-9_f64.sin(), 1e-9_f64.cos()];
    for _ in 0..7 {
        r.observe(pose);
    }
    assert_eq!(r.session.snapshot().completed_visits, 0);
    let angle = r.session.snapshot().evidence.unwrap().orientation_error_rad;
    assert!((angle - 2e-9).abs() < 1e-15);
}

#[test]
fn thresholds_reject_nearby_outside_samples_and_accept_inside_samples() {
    for (factor, expected) in [
        (0., true),
        (0.5, true),
        (0.999, true),
        (1.001, false),
        (2., false),
    ] {
        for angular in [false, true] {
            let mut r = Runner::new();
            let target = r.session.target().unwrap();
            let mut pose = target.pose;
            if angular {
                let half = target.criteria.orientation_rad * factor * 0.5;
                pose.quaternion_xyzw = [0., 0., half.sin(), half.cos()];
            } else {
                pose.position_m[0] += target.criteria.position_m * factor;
            }
            for _ in 0..7 {
                r.observe(pose);
            }
            assert_eq!(
                r.session.snapshot().completed_visits == 1,
                expected,
                "factor={factor}, angular={angular}"
            );
        }
    }
}
