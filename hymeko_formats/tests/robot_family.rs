//! Robot families: declared spaces, refused designs, determinism, the reference arm, fixed
//! topology, legged counts, the retired generator's outputs and the cell's station IK.
use hymeko_formats::robot_family;

use hymeko::{
    generators::{GraphGenerator, SyntheticGenerator},
    module_store::{execution::CompiledModel, source_provider::StdFsProvider},
    util::real_parser::RealParser,
};
use hymeko_formats::robot_projection::RobotProjection;
use hymeko_geometry::forward::{JointValues, forward_kinematics};
use robot_family::{
    FamilyError, FamilyGenerator, FamilySampler, JointKind, LinkSpec, ParameterSpace, RobotFamily, RobotSource, Shape,
    cell::DesignCell,
    check::{Candidate, candidate},
    legged::LeggedPairs,
    manipulator::SerialSixAxis,
};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    sync::atomic::{AtomicUsize, Ordering},
};

#[test]
fn rooted_tree_generator_uses_the_existing_robot_projection() {
    use robot_family::source::TreeRobotGenerator;
    use hymeko::module_store::source_provider::MemProvider;
    for (links, branching) in [(1, 1), (10, 1), (13, 3)] {
        let generator = TreeRobotGenerator::new("tree_case", "meta_kinematics.hymeko", links, branching).unwrap();
        let source = generator.generate().unwrap();
        assert_eq!(source, generator.generate().unwrap());
        let provider = MemProvider::default().with_file("/case.hymeko", source.as_str())
            .with_file("/meta_kinematics.hymeko", include_str!("../../data/robotics/meta_kinematics.hymeko"));
        let model = CompiledModel::compile(provider, RealParser, Path::new("/case.hymeko")).unwrap();
        let view = RobotProjection::new(&model, "tree_case").unwrap();
        assert_eq!(view.model().links.len(), links);
        assert_eq!(view.model().joints.len(), links);
        assert!(!view.mjcf().unwrap().is_empty());
    }
    for (name, vocabulary, links, branching) in [
        ("", "meta.hymeko", 2, 1), ("0bad", "meta.hymeko", 2, 1),
        ("bad-name", "meta.hymeko", 2, 1), ("ok", "bad\"path", 2, 1),
        ("ok", "meta.hymeko", 0, 1), ("ok", "meta.hymeko", 5001, 1),
        ("ok", "meta.hymeko", 2, 0), ("ok", "meta.hymeko", 2, 9),
    ] { assert!(TreeRobotGenerator::new(name, vocabulary, links, branching).is_err()); }
}

fn data() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../data")
}

fn scratch(label: &str) -> PathBuf {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let dir = std::env::temp_dir().join(format!("robot-family-{}-{}-{label}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

fn arm() -> SerialSixAxis {
    SerialSixAxis::new(ParameterSpace::load(&data().join("robotics/families/manipulator_family.hymeko"), "serial_six_axis").unwrap()).unwrap()
}

fn legged(legs: usize) -> Result<LeggedPairs, FamilyError> {
    LeggedPairs::new(ParameterSpace::load(&data().join("robotics/families/legged_family.hymeko"), "legged").unwrap(), legs)
}

fn checked<F: RobotFamily>(family: &F, values: Option<BTreeMap<String, f64>>) -> Result<Candidate, FamilyError> {
    let design = match values {
        Some(v) => family.space().admit(&v)?,
        None => family.space().defaults(),
    };
    candidate(family, design, &data(), &scratch("candidate"))
}

#[test]
fn the_declared_space_is_read_and_refuses_what_it_does_not_declare() {
    let family = arm();
    let space = family.space();
    assert_eq!(space.parameters.iter().map(|p| p.name.as_str()).collect::<Vec<_>>(), ["pedestal_height", "upper_arm_length", "forearm_length", "upper_arm_radius", "forearm_radius"]);
    assert!(space.parameters.iter().all(|p| p.unit == "m" && p.resolution == 0.0001));
    let defaults = space.defaults().values().clone();
    let with = |k: &str, v: f64| {
        let mut d = defaults.clone();
        d.insert(k.into(), v);
        d
    };
    assert!(matches!(space.admit(&with("pedestal_height", 0.5)), Err(FamilyError::Rejected(_))), "out of bounds");
    assert!(matches!(space.admit(&with("pedestal_height", 0.33005)), Err(FamilyError::Rejected(_))), "off the 0.1 mm grid");
    assert!(matches!(space.admit(&with("pedestal_height", f64::NAN)), Err(FamilyError::Rejected(_))), "not finite");
    assert!(matches!(space.admit(&with("wing_span", 1.0)), Err(FamilyError::Rejected(_))), "undeclared");
    let mut missing = defaults.clone();
    missing.remove("forearm_radius");
    assert!(matches!(space.admit(&missing), Err(FamilyError::Rejected(_))), "missing");
    assert!(space.admit(&with("pedestal_height", 0.3512)).is_ok());
}

#[test]
fn generation_is_deterministic_and_sampling_reproducible() {
    let family = arm();
    let design = family.space().defaults();
    let a = FamilyGenerator { family: &family, design: design.clone() }.generate().unwrap();
    let b = FamilyGenerator { family: &family, design }.generate().unwrap();
    assert_eq!(a.source, b.source);
    let mut sampler = FamilySampler::new(&family, 7);
    let (first, again) = (sampler.generate().unwrap(), sampler.generate().unwrap());
    assert_eq!(first, again, "the same seed gives the same design");
    sampler.reseed(8);
    assert_eq!(sampler.seed(), 8);
    assert_ne!(sampler.generate().unwrap().design, first.design);
    for seed in 0..32 {
        let point = FamilySampler::new(&family, seed).design().unwrap();
        assert!(family.space().admit(point.values()).is_ok(), "samples are admissible grid designs");
    }
}

/// The default design is the reference FANUC cell arm: same links, joints, frames and masses;
/// only the declared inertias are new.
#[test]
fn the_default_arm_is_the_reference_robot() {
    let generated = checked(&arm(), None).unwrap();
    assert!(generated.valid(), "{:#?}", generated.checks);
    let reference_path = data().join("robotics/sim/dual_fanuc/fanuc_lrmate200id.hymeko");
    let reference = CompiledModel::compile(StdFsProvider::new(), RealParser, &reference_path).unwrap();
    let (r, g) = (RobotProjection::new(&reference, "robot").unwrap(), RobotProjection::new(&generated.model, "robot").unwrap());
    let (r, g) = (r.model(), g.model());
    for link in &r.links {
        let other = g.links.iter().find(|l| l.name == link.name).unwrap_or_else(|| panic!("{} absent", link.name));
        assert!((link.mass.unwrap() - other.mass.unwrap()).abs() < 1e-9, "{} mass", link.name);
        assert_eq!(link.origin, other.origin, "{} origin", link.name);
        assert_eq!(link.geometry.as_ref().unwrap().dimensions, other.geometry.as_ref().unwrap().dimensions, "{} geometry", link.name);
        assert!(link.inertia_com.is_none() && other.inertia_com.is_some());
    }
    for joint in r.joints.iter().filter(|j| j.name != "j_fix") {
        let other = g.joints.iter().find(|j| j.name == joint.name).unwrap();
        assert_eq!((&joint.parent_link, &joint.child_link, joint.axis, joint.origin_xyz), (&other.parent_link, &other.child_link, other.axis, other.origin_xyz));
        let (a, b) = (joint.limits.as_ref().unwrap(), other.limits.as_ref().unwrap());
        assert_eq!((a.lower, a.upper, a.effort, a.velocity), (b.lower, b.upper, b.effort, b.velocity));
    }
    for k in 0..8 {
        let q: JointValues = (1..=6).map(|j| (format!("j{j}"), 0.4 * ((k * 7 + j) as f64).sin())).collect();
        let (pr, pg) = (forward_kinematics(r, &q).unwrap(), forward_kinematics(g, &q).unwrap());
        let (a, b) = (pr.get("tool_flange").unwrap(), pg.get("tool_flange").unwrap());
        assert!((0..3).all(|i| (a.translation[i] - b.translation[i]).abs() < 1e-12));
    }
}

#[test]
fn continuous_parameters_keep_the_structure_and_change_the_values() {
    let family = arm();
    let base = checked(&family, None).unwrap();
    let mut values = family.space().defaults().values().clone();
    values.insert("upper_arm_length".into(), 0.36);
    values.insert("forearm_radius".into(), 0.045);
    let moved = checked(&family, Some(values)).unwrap();
    assert!(moved.valid(), "{:#?}", moved.checks);
    assert_eq!(base.structure(), moved.structure(), "fixed topology");
    assert_ne!(base.digest(), moved.digest(), "different design, different semantic identity");
    assert!(moved.total_mass() > base.total_mass(), "a longer, thicker arm is heavier under the tube model");
}

#[test]
fn a_flexible_arm_is_refused_by_its_stiffness_requirement() {
    let family = arm();
    let mut values = family.space().defaults().values().clone();
    values.insert("upper_arm_radius".into(), 0.026);
    let thin = checked(&family, Some(values)).unwrap();
    let stiffness = &thin.requirements[0];
    assert!(!stiffness.passed && stiffness.value > stiffness.bound && !thin.valid(), "{stiffness:?}");
    let reference = &checked(&family, None).unwrap().requirements[0];
    assert!(reference.passed && reference.value < reference.bound);
}

/// A family that breaks its own promises: the checks must say which.
struct Faulty(SerialSixAxis, &'static str);
impl RobotFamily for Faulty {
    fn space(&self) -> &ParameterSpace {
        self.0.space()
    }
    fn module(&self) -> String {
        self.0.module()
    }
    fn emit(&self, d: &robot_family::DesignPoint) -> Result<String, FamilyError> {
        let text = self.0.emit(d)?;
        Ok(match self.1 {
            "axis" => text.replacen("- meta_kinematics.kinematics.axes.AXIS_Y", "- meta_kinematics.kinematics.axes.AXIS_X", 1),
            "mass" => text.replacen("mass 4.5;", "mass 45;", 1),
            "dangling" => text.replacen("- link_wrist,", "- link_wrist_missing,", 1),
            _ => text,
        })
    }
    fn expectation(&self) -> robot_family::Expectation {
        self.0.expectation()
    }
    fn requirements(&self, d: &robot_family::DesignPoint) -> Result<Vec<robot_family::Requirement>, FamilyError> {
        self.0.requirements(d)
    }
    fn link_specs(&self, d: &robot_family::DesignPoint) -> Result<Vec<LinkSpec>, FamilyError> {
        self.0.link_specs(d)
    }
}

#[test]
fn broken_robots_are_refused_by_name() {
    for (fault, check) in [("axis", "axes"), ("mass", "mass_model"), ("dangling", "kinematic_tree")] {
        let result = checked(&Faulty(arm(), fault), None);
        match result {
            Ok(c) => {
                let failed: Vec<_> = c.checks.iter().filter(|k| !k.passed).map(|k| k.name.as_str()).collect();
                assert!(!c.valid() && failed.contains(&check), "{fault}: {failed:?}");
            }
            Err(e) => assert!(fault == "dangling", "{fault}: {e}"),
        }
    }
}

#[test]
fn every_declared_leg_count_is_a_valid_robot_and_other_counts_are_refused() {
    let mut structures = Vec::new();
    for legs in [2, 4, 6, 8] {
        let family = legged(legs).unwrap();
        let c = checked(&family, None).unwrap();
        assert!(c.valid(), "{legs}: {:#?} {:#?}", c.checks.iter().filter(|k| !k.passed).collect::<Vec<_>>(), c.requirements);
        assert_eq!(RobotProjection::new(&c.model, &family.module()).unwrap().model().links.len(), 1 + 4 * legs);
        structures.push(c.structure());
    }
    structures.dedup();
    assert_eq!(structures.len(), 4, "the leg count is a structural change");
    assert!(matches!(legged(3), Err(FamilyError::Rejected(_))));
    let family = legged(8).unwrap();
    let mut short = family.space().defaults().values().clone();
    short.insert("body_length".into(), 0.4);
    assert!(matches!(checked(&family, Some(short)), Err(FamilyError::Rejected(_))), "pairs beyond the body are refused");
}

/// The retired Python generator's 2026-10-15 cases, rebuilt with [`RobotSource`]: same Core
/// semantic digest as the retained sources. Point HYMEKO_LEGACY_FIXTURES at their directory.
#[test]
fn the_retired_generator_cases_are_reproduced() {
    let Some(fixtures) = std::env::var_os("HYMEKO_LEGACY_FIXTURES").map(PathBuf::from).filter(|p| p.join("meta_kinematics.hymeko").exists()) else {
        eprintln!("HYMEKO_LEGACY_FIXTURES not set; legacy comparison skipped");
        return;
    };
    let vocabulary = std::fs::read_to_string(fixtures.join("meta_kinematics.hymeko")).unwrap();
    let mut compared = 0;
    for (name, source) in legacy_cases() {
        let retained = fixtures.join(format!("{name}.hymeko"));
        let old = CompiledModel::compile(StdFsProvider::new(), RealParser, &retained).unwrap();
        let files = BTreeMap::from([("meta_kinematics.hymeko".to_string(), vocabulary.clone()), (format!("{name}.hymeko"), source)]);
        let new = robot_family::compile_snapshot(&scratch(&name), &files, &format!("{name}.hymeko")).unwrap();
        assert_eq!(format!("{:?}", old.semantic_digest()), format!("{:?}", new.semantic_digest()), "{name}");
        compared += 1;
    }
    assert_eq!(compared, 62);
}

fn legacy_cases() -> Vec<(String, String)> {
    let shapes = |s: &str| match s {
        "box" => Shape::Box([0.06, 0.08, 0.3]),
        "cylinder" => Shape::Cylinder { radius: 0.035, length: 0.3 },
        _ => Shape::Sphere(0.08),
    };
    let link = |name: &str, shape: Shape, origin: [f64; 3], mass: f64| LinkSpec { name: name.into(), mass, shape, origin, inertia: None, color: None };
    let mut out = Vec::new();
    for shape in ["box", "cylinder", "sphere"] {
        for axis in ["X", "Y", "Z", "M_X", "M_Y", "M_Z"] {
            for yaw in [0., 90.] {
                let name = format!("primitive_{shape}_{}_yaw{}", axis.to_lowercase(), yaw as i32);
                let mut s = RobotSource::new(&name, "meta_kinematics.hymeko");
                s.link(&link("base", Shape::Box([0.2, 0.2, 0.1]), [0.; 3], 2.)).link(&link("tip", shapes(shape), [0., 0., 0.15], 1.25));
                s.joint("hinge", "base", "tip", JointKind::Continuous { axis: &format!("AXIS_{axis}") }, [0.1, 0.05, 0.2], [0., 0., yaw]);
                out.push((name, s.finish()));
            }
        }
    }
    for legs in [2usize, 4, 6, 8] {
        for shape in ["box", "cylinder", "sphere"] {
            for bent in [false, true] {
                let name = format!("legged_{legs}_{shape}_{}", if bent { "bent" } else { "straight" });
                let mut s = RobotSource::new(&name, "meta_kinematics.hymeko");
                s.frame("world").link(&link("body", Shape::Box([0.6, 0.24, 0.16]), [0.; 3], 5.));
                s.joint("base", "world", "body", JointKind::Fixed, [0., 0., 0.72], [0.; 3]);
                let xs: &[f64] = match legs {
                    2 => &[0.],
                    4 => &[0.22, -0.22],
                    6 => &[0.24, 0., -0.24],
                    _ => &[0.27, 0.09, -0.09, -0.27],
                };
                for (pair, x) in xs.iter().enumerate() {
                    for (side, y) in [("l", 0.17), ("r", -0.17)] {
                        let sfx = format!("{side}{pair}");
                        let [hip, upper, lower, foot] = ["hip", "upper", "lower", "foot"].map(|p| format!("{p}_{sfx}"));
                        s.link(&link(&hip, Shape::Sphere(0.045), [0.; 3], 0.125));
                        s.link(&link(&upper, shapes(shape), [0., 0., -0.15], 1.));
                        s.link(&link(&lower, shapes(shape), [0., 0., -0.15], 0.5));
                        s.link(&link(&foot, Shape::Box([0.18, 0.08, 0.05]), [0.06, 0., -0.025], 0.125));
                        s.joint(&format!("abduct_{sfx}"), "body", &hip, JointKind::Continuous { axis: "AXIS_X" }, [*x, y, -0.05], [0.; 3]);
                        s.joint(&format!("flex_{sfx}"), &hip, &upper, JointKind::Continuous { axis: "AXIS_Y" }, [0.; 3], [0., if bent { 15. } else { 0. }, 0.]);
                        s.joint(&format!("knee_{sfx}"), &upper, &lower, JointKind::Continuous { axis: "AXIS_Y" }, [0., 0., -0.3], [0., if bent { -30. } else { 0. }, 0.]);
                        s.joint(&format!("ankle_{sfx}"), &lower, &foot, JointKind::Fixed, [0., 0., -0.3], [0.; 3]);
                    }
                }
                out.push((name, s.finish()));
            }
        }
    }
    out.push({
        let name = "primitive_capsule_y_yaw0".to_string();
        let mut s = RobotSource::new(&name, "meta_kinematics.hymeko");
        s.link(&link("base", Shape::Box([0.2, 0.2, 0.1]), [0.; 3], 2.)).link(&link("tip", Shape::Capsule { length: 0.3, radius: 0.035 }, [0., 0., 0.15], 1.25));
        s.joint("hinge", "base", "tip", JointKind::Continuous { axis: "AXIS_Y" }, [0.1, 0.05, 0.2], [0.; 3]);
        (name, s.finish())
    });
    out.push({
        let name = "primitive_box_oblique_yaw0".to_string();
        let mut s = RobotSource::new(&name, "meta_kinematics.hymeko");
        s.link(&link("base", Shape::Box([0.2, 0.2, 0.1]), [0.; 3], 2.)).link(&link("tip", shapes("box"), [0., 0., 0.15], 1.25));
        s.joint("hinge", "base", "tip", JointKind::Continuous { axis: "AXIS_OBLIQUE" }, [0.1, 0.05, 0.2], [0.; 3]);
        (name, s.finish())
    });
    out
}

#[test]
fn the_cell_puts_every_design_on_the_same_stations() {
    let family = arm();
    let reference = checked(&family, None).unwrap();
    let cell = DesignCell::prepare(&reference, &data(), &scratch("cell")).unwrap();
    for s in &cell.stations {
        assert!(s.converged && s.q.iter().zip(&s.reference_q).all(|(a, b)| (a - b).abs() < 1e-9), "{s:?}");
    }
    let mut values = family.space().defaults().values().clone();
    values.insert("upper_arm_length".into(), 0.36);
    values.insert("pedestal_height".into(), 0.3);
    let moved = checked(&family, Some(values)).unwrap();
    let other = DesignCell::prepare(&moved, &data(), &scratch("cell")).unwrap();
    let (_, manifest) = other.artifacts().unwrap();
    assert_eq!(cell.stations.iter().map(|s| &s.tcp).collect::<Vec<_>>(), other.stations.iter().map(|s| &s.tcp).collect::<Vec<_>>(), "the task did not move");
    assert!(other.stations.iter().zip(&cell.stations).any(|(a, b)| a.q != b.q), "the joint targets did");
    assert_eq!(manifest["joint_names"], serde_json::json!(["j1", "j2", "j3", "j4", "j5", "j6"]));
    assert_eq!(
        format!("{:?}", cell.bundle.model().program().canon.structure()),
        format!("{:?}", other.bundle.model().program().canon.structure()),
        "the checked joint-target update and the geometry change keep the cell's structure"
    );
    let mut far = family.space().defaults().values().clone();
    far.insert("upper_arm_length".into(), 0.22);
    far.insert("forearm_length".into(), 0.2);
    let short = checked(&family, Some(far)).unwrap();
    assert!(matches!(DesignCell::prepare(&short, &data(), &scratch("cell")), Err(FamilyError::Rejected(m)) if m.contains("unreachable")));
}

#[test]
fn snapshots_must_be_new() {
    let dir = scratch("twice");
    let files = BTreeMap::from([("a.hymeko".to_string(), "a_description {}\na {}\n".to_string())]);
    let _ = robot_family::compile_snapshot(&dir, &files, "a.hymeko");
    assert!(matches!(robot_family::compile_snapshot(&dir, &files, "a.hymeko"), Err(FamilyError::Declaration(_))));
    let _ = Path::new(&dir);
}
