//! Recipes read out of HyMeKo IR and executed by hymeko_mesh::recipe. The IR-free recipe tests
//! moved to the Geometry repository with the runner.
#![cfg(feature = "authoring")]
use hymeko_mechanism::mesh::{MeshLod, TriangleMesh};
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

#[cfg(feature = "authoring")]
#[test]
fn canonical_recipe_compiles_inheritance_and_rejects_order_or_missing_imports() {
    use hymeko_mechanism::recipe::RecipeProject;
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
    use hymeko_mechanism::recipe::RecipeProject;
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
