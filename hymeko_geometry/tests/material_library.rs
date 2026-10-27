use hymeko_geometry::material::{
    RgbaImage,
    library::{MaterialLibrary, PhysicalProperty},
    procedural::SurfacePattern,
};
use std::collections::BTreeMap;

#[test]
fn library_roundtrips_optional_physics_without_inventing_defaults() {
    let mut library = MaterialLibrary::builtin().clone();
    assert_eq!(library.materials.len(), 8);
    assert!(library.materials.values().all(|m| m.physical.is_empty()));
    library.materials.get_mut("oak").unwrap().physical.insert(
        "mechanics.density".into(),
        PhysicalProperty {
            value: 600.,
            unit: "kg/m^3".into(),
            source: "Synthetic test assumption, not material data".into(),
            conditions: BTreeMap::from([("moisture".into(), "unspecified test condition".into())]),
        },
    );
    let bytes = serde_json::to_vec(&library).unwrap();
    let back = MaterialLibrary::from_json(&bytes).unwrap();
    let property = &back.materials["oak"].physical["mechanics.density"];
    assert_eq!(property.value, 600.);
    assert_eq!(property.unit, "kg/m^3");
    assert!(property.source.starts_with("Synthetic"));
    assert_eq!(property.conditions.len(), 1);
    library
        .materials
        .get_mut("oak")
        .unwrap()
        .physical
        .get_mut("mechanics.density")
        .unwrap()
        .value = f64::NAN;
    assert!(library.validate().is_err());
    library.schema_version = 2;
    assert!(library.validate().is_err());
    assert!(
        MaterialLibrary::from_json(br#"{"schema_version":1,"materials":{},"typo":true}"#).is_err()
    );
}

#[test]
fn recipes_are_periodic_seeded_and_bounded_before_allocation() {
    for material in MaterialLibrary::builtin().materials.values() {
        let Some(mut recipe) = material.procedural.clone() else {
            continue;
        };
        recipe.cycles = 8;
        for axis in 0..2 {
            let mut uv = [0.417, 0.619];
            uv[axis] = 1e-8;
            let a = recipe.sample(uv).unwrap();
            uv[axis] = 1. - 1e-8;
            let b = recipe.sample(uv).unwrap();
            assert!((a.height_m - b.height_m).abs() < 1e-7);
            for (a, b) in a.color_srgb.into_iter().zip(b.color_srgb) {
                assert!((a - b).abs() < 1e-4);
            }
        }
        for uv in [[0., 0.], [0.231, 0.618], [-0.14, 1.36], [0.99999, 0.5001]] {
            let a = recipe.sample(uv).unwrap();
            let b = recipe.sample([uv[0] + 2., uv[1] - 3.]).unwrap();
            assert!((a.height_m - b.height_m).abs() < 1e-8);
            for (a, b) in a.color_srgb.into_iter().zip(b.color_srgb) {
                assert!((a - b).abs() < 1e-6);
            }
        }
        let first = recipe
            .bake(
                64,
                material.tile_m,
                material.appearance.roughness,
                material.appearance.metallic,
            )
            .unwrap();
        let second = recipe
            .bake(
                64,
                material.tile_m,
                material.appearance.roughness,
                material.appearance.metallic,
            )
            .unwrap();
        assert!(first.color.pixels == second.color.pixels);
        assert!(first.normal.pixels == second.normal.pixels);
        assert!(first.metal_rough.pixels == second.metal_rough.pixels);
        let mut alternate = recipe.clone();
        alternate.seed += 1;
        assert!(
            alternate
                .bake(64, material.tile_m, 0.5, 0.)
                .unwrap()
                .color
                .pixels
                != first.color.pixels
        );
        for size in [0, 31, 63, 1025, u32::MAX] {
            assert!(recipe.bake(size, 1., 0.5, 0.).is_err());
        }
        assert!(recipe.bake(64, f32::NAN, 0.5, 0.).is_err());
        assert!(recipe.sample([f64::INFINITY, 0.]).is_err());
    }
}

#[test]
fn height_gradient_matches_normal_direction_scale_and_wrap() {
    let mut recipe = MaterialLibrary::builtin().materials["rug"]
        .procedural
        .clone()
        .unwrap();
    recipe.cycles = 4;
    recipe.height_m = 0.001;
    recipe.roughness_variation = 0.;
    recipe.pattern = SurfacePattern::Weave;
    let (size, tile) = (64usize, 0.5);
    let baked = recipe.bake(size as u32, tile, 0.6, 0.2).unwrap();
    let wider = recipe.bake(size as u32, tile * 2., 0.6, 0.2).unwrap();
    let mut slopes = [0f64; 2];
    for y in 0..size {
        for x in 0..size {
            let i = y * size + x;
            let decoded_height=(baked.normal.pixels[4*i+3] as f32-128.)/127.*baked.height_scale_m;
            assert!((decoded_height-baked.height_m[i]).abs()<=baked.height_scale_m/254.+1e-8);
            let h = |x: usize, y: usize| baked.height_m[(y % size) * size + x % size];
            let dx = (h(x + 1, y) - h(x + size - 1, y)) * size as f32 / (2. * tile);
            let dy = (h(x, y + 1) - h(x, y + size - 1)) * size as f32 / (2. * tile);
            let len = (dx * dx + dy * dy + 1.).sqrt();
            let expected = [-dx / len, -dy / len, 1. / len];
            let decoded = std::array::from_fn::<_, 3, _>(|k| {
                baked.normal.pixels[4 * i + k] as f32 / 255. * 2. - 1.
            });
            for (a, b) in expected.into_iter().zip(decoded) {
                assert!((a - b).abs() <= 1. / 255. + 1e-6);
            }
            let norm = decoded.iter().map(|x| x * x).sum::<f32>().sqrt();
            assert!((norm - 1.).abs() < 0.007);
            for (j, image) in [&baked.normal, &wider.normal].into_iter().enumerate() {
                let nx = image.pixels[4 * i] as f64 / 255. * 2. - 1.;
                let nz = image.pixels[4 * i + 2] as f64 / 255. * 2. - 1.;
                slopes[j] += (nx / nz).abs();
            }
            assert_eq!(
                &baked.metal_rough.pixels[4 * i..4 * i + 4],
                &[255, 153, 51, 255]
            );
        }
    }
    assert!(
        (slopes[0] / slopes[1] - 2.).abs() < 0.08,
        "tile scale must halve slope"
    );
    recipe.height_m = 0.;
    assert!(
        recipe
            .bake(64, tile, 0.5, 0.)
            .unwrap()
            .normal
            .pixels
            .chunks_exact(4)
            .all(|p| p == [128, 128, 255, 255])
    );
}

#[test]
fn maps_keep_absolute_mr_and_four_k_character_payloads_remain_valid() {
    let avatar = MaterialLibrary::builtin().materials["avatar-atlas"]
        .bake(512)
        .unwrap();
    assert_eq!(
        (avatar.metallic, avatar.roughness, avatar.double_sided),
        (0.08, 0.52, true)
    );
    assert!(avatar.textures.iter().all(Option::is_none));
    let material = &MaterialLibrary::builtin().materials["metal"];
    let baked = material.bake(64).unwrap();
    assert_eq!((baked.roughness, baked.metallic), (1., 1.));
    assert!(
        baked.textures[1]
            .as_ref()
            .unwrap()
            .image
            .pixels
            .chunks_exact(4)
            .all(|p| p[2] == 204)
    );
    for (w, h) in [(4096, 1), (1, 4096), (4096, 4096)] {
        let mut image = RgbaImage {
            width: w,
            height: h,
            pixels: vec![127; (w * h * 4) as usize],
        };
        assert!(image.validate().is_ok());
        image.pixels.pop();
        assert!(image.validate().is_err());
    }
    for (w, h) in [(4097, 1), (1, 4097), (0, 1), (u32::MAX, u32::MAX)] {
        assert!(
            RgbaImage {
                width: w,
                height: h,
                pixels: vec![]
            }
            .validate()
            .is_err()
        );
    }
}
