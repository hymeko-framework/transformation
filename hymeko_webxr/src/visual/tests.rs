use std::path::Path;

use hymeko::ir::ir::Ir;
use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::MemProvider;
use hymeko::resolution::interner::Interner;
use hymeko::util::real_parser::RealParser;

use super::*;
use crate::META_SPATIAL;
use crate::room::PoseSampleError;

/// Compile `body` beside the spatial vocabulary and hand its query to `check`.
fn with_query<R>(body: &str, check: impl FnOnce(&SpatialQuery) -> R) -> R {
    let (ir, it) = compile(body);
    check(&SpatialQuery::new(&ir, &it).unwrap())
}

fn compile(body: &str) -> (Ir, Interner) {
    let source =
        format!("fixture {{ @\"meta_spatial.hymeko\"; using meta_spatial.spatial as s; }}\n{body}");
    let provider = MemProvider::default()
        .with_file("/meta_spatial.hymeko", META_SPATIAL)
        .with_file("/scene.hymeko", source.as_str());
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store
        .compile(Path::new("/scene.hymeko"))
        .unwrap_or_else(|e| panic!("{e:?}"));
    (program.ir.clone(), store.it)
}

fn names<'a>(query: &SpatialQuery<'a>, decls: impl IntoIterator<Item = DeclId>) -> Vec<&'a str> {
    decls
        .into_iter()
        .map(|decl| query.name(decl).unwrap())
        .collect()
}

#[test]
fn visuals_are_the_instances_of_the_visual_kind_with_effective_templates() {
    with_query(
        r#"
        grey: + <isa> s.kinds.visual { r 0.5; g 0.5; b 0.5; sx 0.1; }
        floor: + <isa> s.kinds.visual { kind "Plane"; sx 2; sy "2"; sz 0.01; px 1; py "-1.5"; pz 0; }
        bolt: + <isa> grey { kind "Box"; a 0.4; qw 0; qx 0; qy 0; qz 1; }
        named { kind "Box"; px 0; }
        "#,
        |query| {
            let visuals = query.visuals().unwrap();
            assert_eq!(
                names(&query, visuals.iter().map(|v| v.decl)),
                ["floor", "bolt"],
                "a profile others inherit from is not drawn, and `kind` + `px` alone does not make a visual"
            );
            let [floor, bolt] = &visuals[..] else {
                unreachable!()
            };
            assert_eq!(floor.shape, Some("Plane"));
            assert_eq!(
                floor.size,
                [2.0, 2.0, 0.01],
                "numeric strings read as numbers"
            );
            assert_eq!(floor.position_m, [1.0, -1.5, 0.0]);
            assert_eq!(
                floor.orientation_wxyz,
                [1.0, 0.0, 0.0, 0.0],
                "an absent orientation is the identity"
            );
            assert_eq!(
                (floor.rgb, floor.alpha),
                ([0.0; 3], 1.0),
                "absent colour is 0, absent alpha 1"
            );
            assert_eq!(
                bolt.rgb, [0.5; 3],
                "the profile's colour is the instance's default"
            );
            assert_eq!(bolt.size, [0.1, 0.0, 0.0]);
            assert_eq!(
                (bolt.alpha, bolt.orientation_wxyz),
                (0.4, [0.0, 0.0, 0.0, 1.0])
            );
            assert_eq!((bolt.pose_source, bolt.observation), (bolt.decl, None));
        },
    );
}

#[test]
fn a_pose_source_shares_its_placement_alpha_and_observation() {
    with_query(
        r#"
        room {
            sample [100, 3, 1, 2, 3, 4, 1, 0, 0, 0, 0.65, 1790000000000];
            base: + <isa> s.kinds.visual { kind "Box"; px 1; a 0.7; pose_anchor -> sample; }
            decal: + <isa> s.kinds.visual { kind "Box"; px 9; r 1; mesh_asset "decal.glb"; mesh_group "robot"; pose_source -> base; }
        }
        "#,
        |query| {
            let visuals = query.visuals().unwrap();
            let [base, decal] = &visuals[..] else {
                panic!("two visuals")
            };
            assert_eq!(decal.pose_source, base.decl);
            assert_eq!(
                (decal.position_m, decal.alpha, decal.rgb),
                ([1.0, 0.0, 0.0], 0.7, [1.0, 0.0, 0.0])
            );
            assert_eq!(
                (decal.mesh, decal.mesh_group, decal.mesh_fallback),
                (Some("decal.glb"), Some("robot"), None)
            );
            let Some(Observation::Anchor { leaf, sample }) = decal.observation else {
                panic!("anchored")
            };
            assert_eq!(query.name(leaf), Some("sample"));
            assert_eq!(
                (sample.epoch, sample.sequence, sample.valid),
                (100, 3, true)
            );
            assert_eq!(
                (sample.position_m, sample.confidence, sample.observed_ms),
                ([2.0, 3.0, 4.0], Some(0.65), Some(1_790_000_000_000))
            );
            assert_eq!(base.observation, decal.observation);
        },
    );
}

#[test]
fn malformed_pose_bindings_are_refused() {
    let refused = |body: &str| with_query(body, |query| query.visuals().unwrap_err());
    let chained = refused(
        r#"a: + <isa> s.kinds.visual { kind "Box"; }
           b: + <isa> s.kinds.visual { kind "Box"; pose_source -> a; }
           c: + <isa> s.kinds.visual { kind "Box"; pose_source -> b; }"#,
    );
    assert!(
        matches!(chained, SpatialQueryError::InvalidAttribute { .. }),
        "a pose source must place itself"
    );
    let not_visual =
        refused(r#"marker {} v: + <isa> s.kinds.visual { kind "Box"; pose_source -> marker; }"#);
    assert!(matches!(
        not_visual,
        SpatialQueryError::InvalidAttribute { .. }
    ));
    let both = refused(
        r#"sample [1, 1, 1, 0, 0, 0, 1, 0, 0, 0];
           v: + <isa> s.kinds.visual { kind "Box"; pose_sample -> sample; pose_anchor -> sample; }"#,
    );
    assert!(
        matches!(both, SpatialQueryError::InvalidAttribute { .. }),
        "live or surveyed, not both"
    );
    let bad = refused(
        r#"sample [1, 1, 0.5, 0, 0, 0, 1, 0, 0, 0];
           v: + <isa> s.kinds.visual { kind "Box"; pose_sample -> sample; }"#,
    );
    assert!(matches!(
        bad,
        SpatialQueryError::InvalidSample {
            reason: PoseSampleError::Validity,
            ..
        }
    ));
    let text = refused(r#"v: + <isa> s.kinds.visual { kind "Box"; px "left"; }"#);
    assert!(
        matches!(text, SpatialQueryError::InvalidAttribute { .. }),
        "a non-numeric string is refused, not zero"
    );
}

#[test]
fn a_pose_sample_is_ten_or_twelve_checked_numbers() {
    let good = [
        100.0,
        1.0,
        1.0,
        2.0,
        3.0,
        1.0,
        0.7071067811865476,
        0.0,
        0.0,
        0.7071067811865476,
    ];
    let sample = PoseSample::from_values(&good).unwrap();
    assert_eq!(
        (sample.epoch, sample.sequence, sample.valid),
        (100, 1, true)
    );
    assert_eq!((sample.confidence, sample.observed_ms), (None, None));
    let with = |at: usize, value: f64| {
        let mut values = good.to_vec();
        values.extend([0.5, 12.0]);
        values[at] = value;
        PoseSample::from_values(&values)
    };
    assert_eq!(with(10, 0.5).unwrap().observed_ms, Some(12));
    for (at, value, reason) in [
        (0, -1.0, PoseSampleError::Identity),
        (1, 1.5, PoseSampleError::Identity),
        (2, 0.5, PoseSampleError::Validity),
        (3, 1000.5, PoseSampleError::Bounds),
        (6, 0.0, PoseSampleError::Quaternion),
        (4, f64::NAN, PoseSampleError::NonFinite),
        (10, 1.1, PoseSampleError::Quality),
        (11, 12.5, PoseSampleError::Quality),
    ] {
        assert_eq!(with(at, value), Err(reason), "entry {at} = {value}");
    }
    assert_eq!(
        PoseSample::from_values(&good[..9]),
        Err(PoseSampleError::Layout)
    );
}

#[test]
fn lights_bind_by_kind_and_refuse_an_unknown_light_kind() {
    with_query(
        r#"
        sun: + <isa> s.kinds.light { light_kind "directional"; intensity 50000; casts_shadow 1; qw 1; }
        lamp: + <isa> s.kinds.light { light_kind "point"; intensity "800"; range 3; px 0.5; pz 1; r 1; g 0.8; b 0.6; }
        cone: + <isa> s.kinds.light { light_kind "spot"; intensity 10; range 2; inner 0.2; outer 0.3; }
        nameless { light_kind "point"; intensity 1; }
        "#,
        |query| {
            let lights = query.lights().unwrap();
            assert_eq!(
                names(&query, lights.iter().map(|l| l.decl)),
                ["sun", "lamp", "cone"],
                "`light_kind` alone is not a light"
            );
            assert_eq!(
                (lights[0].kind, lights[0].rgb, lights[0].casts_shadow),
                (LightKind::Directional, None, true)
            );
            assert_eq!(
                (lights[1].kind, lights[1].intensity, lights[1].range),
                (LightKind::Point, 800.0, 3.0)
            );
            assert_eq!(
                (lights[1].rgb, lights[1].position_m),
                (Some([1.0, 0.8, 0.6]), [0.5, 0.0, 1.0])
            );
            assert_eq!(
                (lights[2].kind, lights[2].inner, lights[2].outer),
                (LightKind::Spot, 0.2, 0.3)
            );
            assert_eq!(lights[2].orientation_wxyz, [1.0, 0.0, 0.0, 0.0]);
        },
    );
    for (body, reason) in [
        (
            r#"x: + <isa> s.kinds.light { light_kind "laser"; }"#,
            "an unknown kind is refused, not guessed",
        ),
        (
            r#"x: + <isa> s.kinds.light { intensity 1; }"#,
            "a light must choose its kind",
        ),
    ] {
        assert!(
            with_query(body, |query| query.lights().is_err()),
            "{reason}"
        );
    }
}
