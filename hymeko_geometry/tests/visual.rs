use hymeko::{module_store::module_store::ModuleStore, util::real_parser::RealParser};
use hymeko_geometry::kinematics::{
    kinematic::{KinematicModel, extract_kinematic_model},
    visual::VisualSet,
};
use hymeko_query::QueryEngine;
use hymeko_sources::MemoryProvider;
use std::path::Path;

const SOURCE: &str = r#"
visual_fixture {}
elements { link {} }
geometry { box {} sphere {} capsule {} cylinder {} }
robot: elements {
    white [0.8, 0.9, 1.0, 1.0];
    body: elements.link {
        mass 2; origin [0, 0, 1]; color -> robot.white;
        link_geometry: geometry.box { dimension [1, 2, 3]; }
        collision -> link_geometry; visual -> looks;
        looks {
            shell: geometry.capsule { dimension [0.2, 0.1]; origin [0.1, 0, 0.2]; rpy_deg [0, 90, 0]; }
            eye: geometry.sphere { dimension [0.03]; color [0.1, 0.2, 0.3, 1]; }
        }
    }
    legacy: elements.link {
        mass 1; origin [0, 1, 0]; color [0.3, 0.4, 0.5];
        link_geometry: geometry.sphere { dimension [0.2]; }
    }
}
"#;

fn project(source: &str) -> Result<(KinematicModel, VisualSet), String> {
    let mut store = ModuleStore::new(
        MemoryProvider::new().with("visual.hymeko", source),
        RealParser,
    );
    let program = store
        .compile(Path::new("visual.hymeko"))
        .map_err(|e| format!("{e:?}"))?;
    let engine = QueryEngine::new(&program.ir, &store.it);
    let model = extract_kinematic_model(&engine, "robot");
    let visuals = VisualSet::extract(&engine, &model)?;
    Ok((model, visuals))
}

#[test]
fn grouped_visuals_preserve_physical_model_and_declaration_identity() {
    let (model, visuals) = project(SOURCE).unwrap();
    let (legacy, _) = project(&SOURCE.replace("visual -> looks;", "")).unwrap();
    assert_eq!(model.links.len(), 2);
    for (a, b) in model.links.iter().zip(&legacy.links) {
        assert_eq!(a.name, b.name);
        assert_eq!(a.mass, b.mass);
        assert_eq!(a.origin, b.origin);
        assert_eq!(a.inertia_com, b.inertia_com);
        assert_eq!(format!("{:?}", a.geometry), format!("{:?}", b.geometry));
    }
    assert_eq!(visuals.instances.len(), 3);
    let shell = &visuals.instances[0];
    let eye = &visuals.instances[1];
    assert_eq!(shell.link_index, eye.link_index);
    assert_ne!(shell.did, eye.did);
    assert_eq!(shell.name, "shell");
    assert_eq!(shell.color, [0.8, 0.9, 1.0, 1.0]);
    let point = shell.local_pose.apply(&[0.0, 0.0, 1.0]);
    assert!((point[0] - 1.1).abs() < 1e-9 && (point[2] - 0.2).abs() < 1e-9);
    assert_eq!(eye.local_pose.apply(&[0.0; 3]), [0.0, 0.0, 1.0]);
    assert_eq!(visuals.instances[2].color, [0.3, 0.4, 0.5, 1.0]);
}

#[test]
fn single_reference_and_missing_visual_keep_legacy_placement() {
    let (_, visuals) =
        project(&SOURCE.replace("visual -> looks;", "visual -> link_geometry;")).unwrap();
    assert_eq!(visuals.instances.len(), 2);
    assert_eq!(
        visuals.instances[0].local_pose.apply(&[0.0; 3]),
        [0.0, 0.0, 1.0]
    );
    let (_, no_geometry) =
        project(&SOURCE.replace("link_geometry: geometry.sphere { dimension [0.2]; }", ""))
            .unwrap();
    assert!(no_geometry.instances[2].geometry.is_none());
}

#[test]
fn invalid_visuals_are_errors_instead_of_silent_fallbacks() {
    for (from, to) in [
        ("visual -> looks", "visual 5"),
        ("visual -> looks", "visual -> missing"),
        ("dimension [0.2, 0.1]", "dimension [-0.2, 0.1]"),
        ("dimension [0.2, 0.1]", "dimension [0.2, 0.1, 2]"),
        ("dimension [0.2, 0.1]", "dimension [0.2, \"bad\"]"),
        ("origin [0.1, 0, 0.2]", "origin [0.1, 0]"),
        ("rpy_deg [0, 90, 0]", "rpy_deg [0, \"bad\", 0]"),
        ("color [0.1, 0.2, 0.3, 1]", "color [0.1, 2, 0.3, 1]"),
        ("shell: geometry.capsule", "shell: elements.link"),
        ("shell: geometry.capsule", "shell"),
    ] {
        assert!(project(&SOURCE.replace(from, to)).is_err(), "{to}");
    }
}

#[test]
fn visual_groups_are_flat_and_bounded() {
    let source = |links: usize, parts: usize| {
        let components: String = (0..parts)
            .map(|i| format!("part_{i}: geometry.sphere {{ dimension [0.1]; }}"))
            .collect();
        let bodies: String = (0..links)
            .map(|i| format!("body_{i}: elements.link {{ visual -> robot.looks; }}"))
            .collect();
        format!(
            "fixture {{}} elements {{ link {{}} }} geometry {{ sphere {{}} }} robot: elements {{ looks {{ {components} }} {bodies} }}"
        )
    };
    assert!(project(&source(1, 0)).is_err());
    assert_eq!(project(&source(1, 64)).unwrap().1.instances.len(), 64);
    assert!(project(&source(1, 65)).is_err());
    assert_eq!(project(&source(16, 64)).unwrap().1.instances.len(), 1024);
    assert!(project(&source(17, 64)).is_err());
}

#[test]
fn inherited_material_and_geometry_match_explicit_links_with_local_overrides() {
    let prefix = "m {} types { link {} sphere {} } blue { color [0.1,0.2,0.9,1]; } base { mass 2; origin [0,0,0]; link_geometry: types.sphere { dimension [0.1]; } }";
    let inherited = format!("{prefix} robot {{ a: types.link, blue, base {{}} b: types.link, blue, base {{ mass 3; color [1,0,0,1]; link_geometry: types.sphere {{ dimension [0.2]; }} }} }}");
    let explicit = "m {} types { link {} sphere {} } robot { a: types.link { mass 2; origin [0,0,0]; color [0.1,0.2,0.9,1]; link_geometry: types.sphere { dimension [0.1]; } } b: types.link { mass 3; origin [0,0,0]; color [1,0,0,1]; link_geometry: types.sphere { dimension [0.2]; } } }";
    let (a, av) = project(&inherited).unwrap();
    let (b, bv) = project(explicit).unwrap();
    assert_eq!(a.links.len(), 2, "property templates must not become physical bodies");
    for (a, b) in a.links.iter().zip(&b.links) {
        assert_eq!((&a.name, a.mass, &a.origin, &a.color), (&b.name, b.mass, &b.origin, &b.color));
        assert_eq!(format!("{:?}", a.geometry), format!("{:?}", b.geometry));
    }
    assert_eq!(av.instances.len(), bv.instances.len());
    for (a, b) in av.instances.iter().zip(&bv.instances) {
        assert_eq!(a.color, b.color);
        assert_eq!(a.local_pose.apply(&[0.;3]), b.local_pose.apply(&[0.;3]));
    }
    assert!(project(&inherited.replace("color [1,0,0,1]", "color [2,0,0,1]")).is_err(), "invalid own material cannot silently fall back to a base");
}
