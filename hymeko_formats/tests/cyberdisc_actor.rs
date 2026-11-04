//! Actor/course import parity through Core, Geometry and visual FK; no simulation stepping.
use hymeko::module_store::{
    module_store::ModuleStore,
    source_provider::{MemProvider, StdFsProvider},
};
use hymeko::util::real_parser::RealParser;
use hymeko_mechanism::{
    forward::FkPlan,
    kinematics::{
        kinematic::{KinematicModel, extract_kinematic_model},
        visual::VisualSet,
    },
};
use hymeko_query::QueryEngine;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

struct Projection {
    model: KinematicModel,
    visuals: VisualSet,
}

impl Projection {
    fn load(file: &str) -> Self {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../robotics/examples/cyberdisc")
            .join(file);
        let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
        let program = store.compile(&path).expect("Cyberdisc source compiles");
        let engine = QueryEngine::new(&program.ir, &store.it);
        let model = extract_kinematic_model(&engine, "cyberdisc");
        let visuals = VisualSet::extract(&engine, &model).expect("visual projection");
        Self { model, visuals }
    }

    fn before(source: &str) -> Self {
        let source = source.replace(
            "../../../../hymeko_simulator/data/robotics/meta_kinematics.hymeko",
            "/meta.hymeko",
        );
        let provider = MemProvider::default()
            .with_file("/before.hymeko", source)
            .with_file(
                "/meta.hymeko",
                include_str!("../../data/robotics/meta_kinematics.hymeko"),
            );
        let mut store = ModuleStore::new(provider, RealParser);
        let program = store
            .compile(Path::new("/before.hymeko"))
            .expect("original source compiles");
        let engine = QueryEngine::new(&program.ir, &store.it);
        let model = extract_kinematic_model(&engine, "cyberdisc");
        let visuals = VisualSet::extract(&engine, &model).unwrap();
        Self { model, visuals }
    }

    // Compare authored/numerical facts, independent of import-order DeclIds and vector ordering.
    fn signature(&self) -> BTreeMap<String, Value> {
        let mut facts = BTreeMap::new();
        for link in &self.model.links {
            facts.insert(format!("link/{}", link.name), json!({
                "mass":link.mass,"inertia":link.inertia_com,"geometry":format!("{:?}",link.geometry),
                "origin":link.origin,"color":link.color
            }));
        }
        for joint in &self.model.joints {
            facts.insert(format!("joint/{}",joint.name),json!({
                "type":format!("{:?}",joint.joint_type),"parent":joint.parent_link,"child":joint.child_link,
                "axis":joint.axis,"xyz":joint.origin_xyz,"rpy":joint.origin_rpy_deg,"limits":format!("{:?}",joint.limits)
            }));
        }
        let plan = FkPlan::new(&self.model).expect("single anchored mechanism");
        let mut q = plan.zero_values();
        for (name, value) in [
            ("base_x", 1.25),
            ("base_y", -2.0),
            ("base_z", 0.261),
            ("base_yaw", 0.37),
            ("drive_fl", 0.61),
            ("drive_rr", -0.48),
        ] {
            q[plan.slot(name).unwrap()] = value;
        }
        let mut poses = plan.identity_poses_se3();
        plan.resolve_se3(&self.model, &q, &mut poses);
        for (link, pose) in self.model.links.iter().zip(&poses) {
            facts.insert(
                format!("pose/{}", link.name),
                json!(
                    [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
                        .map(|p| pose.apply(&p))
                ),
            );
        }
        for visual in &self.visuals.instances {
            let link = &self.model.links[visual.link_index];
            let pose = poses[visual.link_index].compose(&visual.local_pose);
            facts.insert(
                format!("visual/{}/{}", link.name, visual.name),
                json!({
                    "geometry":format!("{:?}",visual.geometry),"color":visual.color,
                    "pose":([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]].map(|p|pose.apply(&p)))
                }),
            );
        }
        assert!(self.model.actuation.errors.is_empty());
        assert!(self.model.actuation.drives.is_empty());
        facts
    }
}

#[test]
#[ignore = "local robotics checkout integration; run explicitly with --ignored --exact"]
fn reusable_actor_has_only_the_original_mobile_mechanism() {
    let actor = Projection::load("actor.hymeko");
    assert_eq!(
        (
            actor.model.links.len(),
            actor.model.joints.len(),
            actor.visuals.instances.len()
        ),
        (13, 12, 17)
    );
    assert_eq!(
        actor
            .visuals
            .instances
            .iter()
            .filter(|v| v.geometry.is_some())
            .count(),
        11
    );
    let facts = actor.signature();
    assert!(!facts.contains_key("link/falling_ball"));
    assert!(!facts.contains_key("link/block_a"));
    assert_eq!(facts["link/chassis"]["mass"], 8.0);
    assert_eq!(
        facts["link/chassis"]["inertia"],
        json!([0.337, 0.337, 0.64, 0., 0., 0.])
    );
    assert_eq!(facts["pose/chassis"][0], json!([1.25, -2.0, 0.261]));
    for name in ["drive_fl", "drive_rl", "drive_fr", "drive_rr"] {
        assert_eq!(facts[&format!("joint/{name}")]["parent"], "chassis");
    }
}

#[test]
#[ignore = "local robotics checkout integration; run explicitly with --ignored --exact"]
fn legacy_course_imports_the_same_actor_and_anchors_every_course_body() {
    let actor = Projection::load("actor.hymeko");
    let course = Projection::load("robot.hymeko");
    assert_eq!(
        (course.model.links.len(), course.model.joints.len()),
        (28, 27)
    );
    let actor_facts = actor.signature();
    let course_facts = course.signature();
    for (name, value) in actor_facts {
        assert_eq!(course_facts[&name], value, "{name}");
    }
    let world = course
        .model
        .links
        .iter()
        .find(|l| l.name == "world")
        .unwrap()
        .did;
    for joint in course
        .model
        .joints
        .iter()
        .filter(|j| j.name.ends_with("_pose") || j.name == "drop_z")
    {
        assert_eq!(
            joint.parent, world,
            "{} must reference the imported world identity",
            joint.name
        );
    }
    assert_eq!(course_facts["pose/block_a"][0], json!([-1.7, -1.3, 0.25]));
    assert_eq!(
        course_facts["pose/falling_ball"][0],
        json!([1.2, -2.8, 2.5])
    );
}

#[test]
#[ignore = "migration audit requires HYMEKO_CYBERDISC_BEFORE pointing at the preserved original source"]
fn factored_course_matches_preserved_original_projection() {
    let path = std::env::var_os("HYMEKO_CYBERDISC_BEFORE").expect("original source path");
    let before = Projection::before(&std::fs::read_to_string(path).unwrap());
    let after = Projection::load("robot.hymeko");
    assert_eq!(before.signature(), after.signature());
}
