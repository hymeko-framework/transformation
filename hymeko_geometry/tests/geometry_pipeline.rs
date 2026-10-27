use hymeko_geometry::{
    mesh::{
        MeshLod, TriangleMesh,
        fair::{FairOptions, SurfaceFairing},
    },
    pipeline::{GeometryRecipe, GeometryStage},
};
use std::collections::BTreeMap;

fn grid() -> TriangleMesh {
    let mut mesh = TriangleMesh {
        positions: vec![],
        normals: vec![],
        uv: vec![],
        tangents: vec![],
        lods: vec![MeshLod {
            indices: vec![],
            error: 0.,
        }],
    };
    for y in 0..5 {
        for x in 0..5 {
            mesh.positions.push([
                x as f32 * 0.01,
                y as f32 * 0.01,
                if x == 2 && y == 2 { 0.002 } else { 0. },
            ]);
            mesh.normals.push([0., 0., 1.]);
            mesh.uv.push([x as f32 / 4., y as f32 / 4.]);
        }
    }
    for y in 0..4 {
        for x in 0..4 {
            let a = y * 5 + x;
            mesh.lods[0]
                .indices
                .extend([a, a + 1, a + 5, a + 1, a + 6, a + 5]);
        }
    }
    mesh
}
#[test]
fn fairing_preserves_pins_uv_and_input_on_refusal() {
    let mut mesh = grid();
    let original = mesh.clone();
    let report = SurfaceFairing::apply(
        &mut mesh,
        &FairOptions {
            iterations: 12,
            recovery: 0.1,
            max_displacement: 0.001,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(report.moved_vertices > 0 && report.maximum_displacement <= 0.001000001);
    assert!(mesh.positions[12][2] < original.positions[12][2]);
    for i in 0..25 {
        if i % 5 == 0 || i % 5 == 4 || !(5..20).contains(&i) {
            assert_eq!(mesh.positions[i], original.positions[i]);
        }
    }
    assert_eq!(mesh.uv, original.uv);
    assert_eq!(mesh.lods[0].indices, original.lods[0].indices);
    let snapshot = serde_json::to_value(&mesh).unwrap();
    assert!(
        SurfaceFairing::apply(
            &mut mesh,
            &FairOptions {
                iterations: 97,
                ..Default::default()
            }
        )
        .is_err()
    );
    assert_eq!(serde_json::to_value(&mesh).unwrap(), snapshot);
}

#[test]
fn weighted_fairing_keeps_attribute_seams_closed_and_skips_degenerate_work_faces() {
    use hymeko_geometry::mesh::fair::BoundaryMode;
    let mut mesh = grid();
    mesh.positions.push(mesh.positions[12]);
    mesh.normals.push(mesh.normals[12]);
    mesh.uv.push([0.9, 0.9]);
    *mesh.lods[0].indices.iter_mut().find(|i| **i == 12).unwrap() = 25;
    mesh.lods[0].indices.extend([0, 0, 1]);
    let original = mesh.clone();
    let mut influence = vec![0.; 26];
    influence[25] = 1.;
    SurfaceFairing::apply_weighted(&mut mesh, &FairOptions::default(), Some(&influence)).unwrap();
    assert_eq!(mesh.positions, original.positions);
    let report = SurfaceFairing::apply_weighted(
        &mut mesh,
        &FairOptions {
            recovery: 0.1,
            boundary_mode: BoundaryMode::Free,
            ..Default::default()
        },
        Some(&influence),
    )
    .unwrap();
    assert_eq!(report.skipped_degenerate_triangles, 1);
    assert_eq!(mesh.positions[12], mesh.positions[25]);
    assert!(mesh.positions[12][2] < original.positions[12][2]);
    for i in 0..25 {
        if i != 12 {
            assert_eq!(mesh.positions[i], original.positions[i]);
            assert_eq!(mesh.normals[i], original.normals[i]);
        }
    }
    assert_eq!(mesh.uv, original.uv);
    assert_eq!(mesh.lods[0].indices, original.lods[0].indices);
    let saved = serde_json::to_value(&mesh).unwrap();
    influence[0] = f64::NAN;
    assert!(
        SurfaceFairing::apply_weighted(&mut mesh, &FairOptions::default(), Some(&influence))
            .is_err()
    );
    assert_eq!(serde_json::to_value(mesh).unwrap(), saved);
}

#[test]
fn fairing_backtracks_before_a_triangle_collapses() {
    use hymeko_geometry::mesh::fair::BoundaryMode;
    let mut mesh = TriangleMesh {
        positions: vec![[0., 0., 0.], [0.01, 0., 0.], [0., 0.01, 0.]],
        normals: vec![[0., 0., 1.]; 3],
        uv: vec![],
        tangents: vec![],
        lods: vec![MeshLod {
            indices: vec![0, 1, 2],
            error: 0.,
        }],
    };
    let report = SurfaceFairing::apply(
        &mut mesh,
        &FairOptions {
            iterations: 96,
            recovery: 0.,
            max_displacement: 0.05,
            boundary_mode: BoundaryMode::Free,
            ..Default::default()
        },
    )
    .unwrap();
    assert!(report.step_scale > 0. && report.step_scale < 1.);
    let [a, b, c] = [mesh.positions[0], mesh.positions[1], mesh.positions[2]];
    assert!((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0.);
    mesh.validate().unwrap();
}

#[test]
fn changed_normals_invalidate_tangents_without_discarding_geometric_lods() {
    use hymeko_geometry::mesh::{fair::FairNormals, normals::NormalSmoothing};
    let mut mesh = grid();
    mesh.tangents = vec![[1., 0., 0., 1.]; mesh.positions.len()];
    mesh.lods.push(mesh.lods[0].clone());
    let original = mesh.positions.clone();
    SurfaceFairing::apply(
        &mut mesh,
        &FairOptions {
            iterations: 0,
            normals: FairNormals::Rebuild,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(mesh.positions, original);
    assert_eq!(mesh.lods.len(), 2);
    assert!(mesh.tangents.is_empty());
    mesh.tangents = vec![[1., 0., 0., 1.]; mesh.positions.len()];
    NormalSmoothing::apply(&mut mesh, 8, 65.).unwrap();
    assert!(mesh.tangents.is_empty());
    assert_eq!(mesh.positions, original);
}
#[test]
fn ordered_pipeline_is_repeatable_and_rejects_unknown_options() {
    let source = grid();
    let saved = serde_json::to_value(&source).unwrap();
    let mut recipe = GeometryRecipe {
        schema_version: 1,
        length_unit: "m".into(),
        up_axis: "Z".into(),
        workflow: "surface".into(),
        name: "geometry".into(),
        settings: BTreeMap::new(),
        stages: vec![
            GeometryStage {
                name: "smooth".into(),
                operation: "fair".into(),
                parameters: BTreeMap::from([("recovery".into(), serde_json::json!(0.1))]),
            },
            GeometryStage {
                name: "shade".into(),
                operation: "normals".into(),
                parameters: BTreeMap::new(),
            },
        ],
    };
    let mut names = vec![];
    let a = recipe
        .run(&source, |s| {
            names.push(s.name.clone());
            Ok(())
        })
        .unwrap();
    let b = recipe.run(&source, |_| Ok(())).unwrap();
    assert_eq!(names, ["smooth", "shade"]);
    assert_eq!(
        serde_json::to_value(a.mesh).unwrap(),
        serde_json::to_value(b.mesh).unwrap()
    );
    assert_eq!(serde_json::to_value(source).unwrap(), saved);
    recipe.stages[0]
        .parameters
        .insert("typo".into(), serde_json::json!(1));
    assert!(recipe.validate().is_err());
}

#[cfg(feature = "authoring")]
#[test]
fn canonical_recipe_compiles_inheritance_and_rejects_order_or_missing_imports() {
    use hymeko_geometry::pipeline::RecipeProject;
    let source = "model {} defaults { schema_version 1; length_unit \"m\"; workflow \"surface\"; settings { note \"inherited group\"; } } geometry: defaults { stages { surface { order 0; operation \"fair\"; parameters { iterations 4; max_displacement 0.001; } } shading { order 1; operation \"normals\"; } } }";
    let project = RecipeProject {
        entry: "main.hymeko".into(),
        files: BTreeMap::from([("main.hymeko".into(), source.into())]),
    };
    let compiled = project.compile("geometry").unwrap();
    assert_eq!(compiled.recipe.settings["note"], "inherited group");
    assert_eq!(compiled.recipe.stages[0].parameters["iterations"], 4);
    assert!(!compiled.semantic_digest.is_empty());
    let mut broken = project.clone();
    broken
        .files
        .insert("main.hymeko".into(), source.replace("order 1", "order 0"));
    assert!(broken.compile("geometry").is_err());
    broken.files.insert(
        "main.hymeko".into(),
        "model { @\"missing.hymeko\"; } geometry {}".into(),
    );
    assert!(broken.compile("geometry").is_err());
}

#[cfg(feature = "authoring")]
#[test]
fn shipped_recipes_resolve_real_imports_and_execute() {
    use hymeko_geometry::pipeline::RecipeProject;
    for source in [
        include_str!("../data/surface-smooth.hymeko"),
        include_str!("../data/surface-refine.hymeko"),
    ] {
        let project = RecipeProject {
            entry: "main.hymeko".into(),
            files: BTreeMap::from([
                ("main.hymeko".into(), source.into()),
                (
                    "meta_geometry.hymeko".into(),
                    include_str!("../data/meta_geometry.hymeko").into(),
                ),
            ]),
        };
        let compiled = project.compile("geometry").unwrap();
        assert_eq!(compiled.recipe.up_axis, "Z");
        let input = grid();
        let result = compiled.recipe.run(&input, |_| Ok(())).unwrap();
        result.mesh.validate().unwrap();
        assert_eq!(result.stages.len(), compiled.recipe.stages.len());
        if source.contains("regularize") {
            assert!(result.quad_surface.is_some());
        }
    }
}
