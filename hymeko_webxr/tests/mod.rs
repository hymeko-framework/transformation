//! Source-first V&V for the WebXR semantic contract, over the production `hymeko_webxr::validate`.
//!
//! `*.hymeko` → RealParser → ModuleStore::compile → canonical Ir → coherence →
//! `hymeko_webxr::validate`.
//! Positive: the demo and a minimal scene validate clean. Negative: one fault per in-memory scene,
//! each asserting the exact `SemanticError` (NaN/Inf and dangling refs, which source cannot express,
//! are injected into a cloned Ir). Metamorphic: declaration-order and import-alias invariance.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{Ir, RefAtomR, SignedRefR, ValueR};
use hymeko::module_store::module_store::{CompiledProgram, ModuleStore};
use hymeko::module_store::source_provider::{MemProvider, StdFsProvider};
use hymeko::resolution::interner::Interner;
use hymeko::util::real_parser::RealParser;
use hymeko_webxr::{RoleReason, SemanticError, SpatialQuery, validate};

fn data_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("data")
        .join("webxr")
}

struct Compiled<P: hymeko::module_store::source_provider::SourceProvider> {
    store: ModuleStore<P, RealParser>,
    prog: Arc<CompiledProgram>,
}
impl<P: hymeko::module_store::source_provider::SourceProvider> Compiled<P> {
    fn ir(&self) -> &Ir {
        &self.prog.ir
    }
    fn it(&self) -> &Interner {
        &self.store.it
    }
}

fn compile_demo() -> Compiled<StdFsProvider> {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let prog = store
        .compile(&data_dir().join("webxr_hypergraph_demo.hymeko"))
        .unwrap_or_else(|e| panic!("demo must compile: {e:?}"));
    Compiled { store, prog }
}

fn compile_scene(scene_src: &str) -> Compiled<MemProvider> {
    let spatial = std::fs::read_to_string(data_dir().join("meta_spatial.hymeko")).unwrap();
    let webxr = std::fs::read_to_string(data_dir().join("meta_webxr.hymeko")).unwrap();
    let provider = MemProvider::default()
        .with_file("/meta_spatial.hymeko", spatial)
        .with_file("/meta_webxr.hymeko", webxr)
        .with_file("/scene.hymeko", scene_src.to_string());
    let mut store = ModuleStore::new(provider, RealParser);
    let prog = store
        .compile(Path::new("/scene.hymeko"))
        .unwrap_or_else(|e| panic!("scene must compile: {e:?}"));
    Compiled { store, prog }
}

const HEAD: &str = r#"
neg_scene {
    @"meta_webxr.hymeko";
    @"meta_spatial.hymeko";
    using meta_spatial.spatial as s;
    using meta_webxr.webxr as x;
}
"#;

fn wrap(body: &str) -> String {
    format!("{HEAD}\nscene: + <isa> s.kinds.scene {{\n{body}\n}}\n")
}

fn errs(body: &str) -> Vec<SemanticError> {
    let c = compile_scene(&wrap(body));
    validate(c.ir(), c.it())
}

const VALID_BODY: &str = r#"
    @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
    frame: + <isa> s.kinds.spatial_frame {}
    ent:   + <isa> s.kinds.spatial_entity {}
    @place: s.relations.placement { (+ ent, - frame); position [0.0, 0.0, 0.0]; orientation [0.0, 0.0, 0.0, 1.0]; scale [1.0, 1.0, 1.0]; }
"#;

// ---------------------------------------------------------------------------------------------
// positive
// ---------------------------------------------------------------------------------------------

#[test]
fn demo_validates_clean() {
    let c = compile_demo();
    let v = validate(c.ir(), c.it());
    assert!(v.is_empty(), "the demo must be valid, got: {v:#?}");
}

#[test]
fn latest_maxwell_fiducial_demo_compiles_and_queries() {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = store
        .compile(&data_dir().join("maxwell_fiducial_demo.hymeko"))
        .unwrap_or_else(|error| panic!("Maxwell/fiducial demo must compile: {error:?}"));
    let query = SpatialQuery::new(&program.ir, &store.it)
        .expect("the relocated spatial vocabulary must be discoverable");

    assert_eq!(query.camera_frames().unwrap().len(), 1);
    assert_eq!(query.fiducial_bindings().unwrap().len(), 2);
    assert_eq!(query.placements().unwrap().len(), 2);
}

#[test]
fn minimal_valid_scene_validates_clean() {
    let v = errs(VALID_BODY);
    assert!(v.is_empty(), "minimal scene must be valid, got: {v:#?}");
}

// ---------------------------------------------------------------------------------------------
// negative — one fault per scene, exact SemanticError
// ---------------------------------------------------------------------------------------------

#[test]
fn missing_session_mode() {
    let v =
        errs(r#"@xr_session: x.session.session_config { (- x.reference_spaces.local_floor); }"#);
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::SessionModeCardinality { found: 0, .. })),
        "{v:#?}"
    );
}

#[test]
fn two_session_modes() {
    let v = errs(
        r#"@xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, + x.session_modes.immersive_vr, - x.reference_spaces.local_floor); }"#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::SessionModeCardinality { found: 2, .. })),
        "{v:#?}"
    );
}

#[test]
fn missing_reference_space() {
    let v = errs(r#"@xr_session: x.session.session_config { (+ x.session_modes.immersive_ar); }"#);
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::ReferenceSpaceCardinality { found: 0, .. })),
        "{v:#?}"
    );
}

#[test]
fn capability_required_optional_conflict() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        @req: x.session.capability_request { (+ scene, - x.capabilities.hit_test); requirement "required"; }
        @opt: x.session.capability_request { (+ scene, - x.capabilities.hit_test); requirement "optional"; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::CapabilityRequiredOptionalOverlap { .. })),
        "{v:#?}"
    );
}

#[test]
fn unknown_capability() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        notcap: + <isa> s.kinds.spatial_frame {}
        @req: x.session.capability_request { (+ scene, - notcap); requirement "optional"; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::UnknownCapability { .. })),
        "{v:#?}"
    );
}

#[test]
fn bad_capability_requirement() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        @req: x.session.capability_request { (+ scene, - x.capabilities.hit_test); requirement "maybe"; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::BadCapabilityRequirement { .. })),
        "{v:#?}"
    );
}

#[test]
fn position_wrong_length() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        ent:   + <isa> s.kinds.spatial_entity {}
        @place: s.relations.placement { (+ ent, - frame); position [0.0, 0.0]; }
    "#,
    );
    assert!(
        v.iter().any(|e| matches!(
            e,
            SemanticError::PoseBadLength {
                expected: 3,
                found: 2,
                ..
            }
        )),
        "{v:#?}"
    );
}

#[test]
fn zero_quaternion() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        ent:   + <isa> s.kinds.spatial_entity {}
        @place: s.relations.placement { (+ ent, - frame); orientation [0.0, 0.0, 0.0, 0.0]; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::QuaternionNotNormalizable { .. })),
        "{v:#?}"
    );
}

#[test]
fn scale_non_positive() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        ent:   + <isa> s.kinds.spatial_entity {}
        @place: s.relations.placement { (+ ent, - frame); scale [-1.0, 1.0, 1.0]; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::ScaleNonPositive { index: 0, .. })),
        "{v:#?}"
    );
}

#[test]
fn frame_cycle() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        fa: + <isa> s.kinds.spatial_frame {}
        fb: + <isa> s.kinds.spatial_frame {}
        @p1: s.relations.parent_frame { (+ fa, - fb); }
        @p2: s.relations.parent_frame { (+ fb, - fa); }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::FrameCycle { .. })),
        "{v:#?}"
    );
}

#[test]
fn placement_two_subjects_is_wrong_role() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        e1: + <isa> s.kinds.spatial_entity {}
        e2: + <isa> s.kinds.spatial_entity {}
        @place: s.relations.placement { (+ e1, + e2, - frame); }
    "#,
    );
    assert!(
        v.iter().any(|e| matches!(
            e,
            SemanticError::RelationRole {
                reason: RoleReason::SubjectCount { found: 2 },
                ..
            }
        )),
        "{v:#?}"
    );
}

#[test]
fn placement_frame_not_a_frame() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        e1: + <isa> s.kinds.spatial_entity {}
        e2: + <isa> s.kinds.spatial_entity {}
        @place: s.relations.placement { (+ e1, - e2); }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::FrameMissingOrNotAFrame { .. })),
        "{v:#?}"
    );
}

#[test]
fn interaction_missing_input() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        ent: + <isa> s.kinds.spatial_entity {}
        act: + <isa> x.actions.select {}
        @ix: s.relations.interaction { (~ act, - ent); }
    "#,
    );
    assert!(
        v.iter().any(|e| matches!(
            e,
            SemanticError::RelationRole {
                reason: RoleReason::SubjectCount { found: 0 },
                ..
            }
        )),
        "{v:#?}"
    );
}

#[test]
fn unknown_action() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        ent: + <isa> s.kinds.spatial_entity {}
        inp: + <isa> x.input.controller {}
        notact: + <isa> s.kinds.spatial_frame {}
        @ix: s.relations.interaction { (+ inp, ~ notact, - ent); }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::UnknownAction { .. })),
        "{v:#?}"
    );
}

#[test]
fn asset_uri_not_string() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        asset: + <isa> s.kinds.visual_asset { uri [1.0]; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::AssetUriNotString { .. })),
        "{v:#?}"
    );
}

#[test]
fn runtime_handle_field_rejected() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        ent: + <isa> s.kinds.spatial_entity { button_index 3; }
    "#,
    );
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::RuntimeHandleField { .. })),
        "{v:#?}"
    );
}

// ---------------------------------------------------------------------------------------------
// negative — hand-injected (source cannot express NaN/Inf or a dangling DeclId)
// ---------------------------------------------------------------------------------------------

/// Overwrite the first attribute named `attr` with `value` (in a cloned Ir).
fn inject_attr(ir: &mut Ir, it: &Interner, attr: &str, value: ValueR) {
    let sym = it.get_id(attr).expect("attr interned");
    for node in ir.decl_nodes.iter_mut() {
        if node.name == sym {
            node.anno.value = Some(value);
            return;
        }
    }
    panic!("no `{attr}` attribute to inject");
}

#[test]
fn nan_position_is_rejected() {
    let c = compile_scene(&wrap(VALID_BODY));
    let mut ir = c.ir().clone();
    inject_attr(
        &mut ir,
        c.it(),
        "position",
        ValueR::List(vec![
            ValueR::Num(f64::NAN),
            ValueR::Num(0.0),
            ValueR::Num(0.0),
        ]),
    );
    let v = validate(&ir, c.it());
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::PoseNonFinite { index: 0, .. })),
        "{v:#?}"
    );
}

#[test]
fn inf_orientation_is_rejected() {
    let c = compile_scene(&wrap(VALID_BODY));
    let mut ir = c.ir().clone();
    inject_attr(
        &mut ir,
        c.it(),
        "orientation",
        ValueR::List(vec![
            ValueR::Num(f64::INFINITY),
            ValueR::Num(0.0),
            ValueR::Num(0.0),
            ValueR::Num(1.0),
        ]),
    );
    let v = validate(&ir, c.it());
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::PoseNonFinite { .. })),
        "{v:#?}"
    );
}

#[test]
fn dangling_reference_is_rejected() {
    let c = compile_scene(&wrap(VALID_BODY));
    let mut ir = c.ir().clone();
    // point the last arc's first participant at a non-existent decl (the placement's arc)
    let bogus = DeclId::new(ir.decl_nodes.len() + 500);
    let arc = ir.arcs.last_mut().expect("an arc exists");
    arc.refs[0] = SignedRefR::Plus(RefAtomR::plain(bogus));
    let v = validate(&ir, c.it());
    assert!(
        v.iter()
            .any(|e| matches!(e, SemanticError::DanglingReference { .. })),
        "{v:#?}"
    );
}

// ---------------------------------------------------------------------------------------------
// metamorphic — order / alias invariance
// ---------------------------------------------------------------------------------------------

#[test]
fn declaration_order_permutation_is_invariant() {
    let permuted = r#"
        @place: s.relations.placement { (+ ent, - frame); position [0.0, 0.0, 0.0]; orientation [0.0, 0.0, 0.0, 1.0]; scale [1.0, 1.0, 1.0]; }
        ent:   + <isa> s.kinds.spatial_entity {}
        frame: + <isa> s.kinds.spatial_frame {}
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
    "#;
    assert_eq!(
        errs(VALID_BODY),
        errs(permuted),
        "reordering declarations must not change the verdict"
    );
    assert!(errs(permuted).is_empty());
}

#[test]
fn import_alias_change_is_invariant() {
    // same scene, different aliases (sp/wx instead of s/x)
    let aliased = "alias_scene {\n@\"meta_webxr.hymeko\";\n@\"meta_spatial.hymeko\";\nusing meta_spatial.spatial as sp;\nusing meta_webxr.webxr as wx;\n}\nscene: + <isa> sp.kinds.scene {\n@xr_session: wx.session.session_config { (+ wx.session_modes.immersive_ar, - wx.reference_spaces.local_floor); }\nframe: + <isa> sp.kinds.spatial_frame {}\nent: + <isa> sp.kinds.spatial_entity {}\n@place: sp.relations.placement { (+ ent, - frame); position [0.0, 0.0, 0.0]; }\n}\n".to_string();
    let c = compile_scene(&aliased);
    assert!(
        validate(c.ir(), c.it()).is_empty(),
        "alias choice must not change the verdict"
    );
}

// ---------------------------------------------------------------------------------------------
// benchmark (ignored; run with --release --ignored --nocapture)
// ---------------------------------------------------------------------------------------------

/// Generate a scene with `n` spatial entities, each placed in one of two frames.
fn gen_scene(n: usize) -> String {
    use std::fmt::Write as _;
    let mut s = String::from(HEAD);
    s.push_str("scene: + <isa> s.kinds.scene {\n");
    s.push_str("@xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }\n");
    s.push_str("fa: + <isa> s.kinds.spatial_frame {}\n");
    s.push_str("fb: + <isa> s.kinds.spatial_frame {}\n");
    s.push_str("@pf: s.relations.parent_frame { (+ fb, - fa); }\n");
    for i in 0..n {
        let _ = writeln!(s, "e{i}: + <isa> s.kinds.spatial_entity {{}}");
        let frame = if i % 2 == 0 { "fa" } else { "fb" };
        let _ = writeln!(
            s,
            "@p{i}: s.relations.placement {{ (+ e{i}, - {frame}); position [0.0, 1.0, -2.0]; orientation [0.0, 0.0, 0.0, 1.0]; scale [1.0, 1.0, 1.0]; }}"
        );
    }
    s.push_str("}\n");
    s
}

fn median_ms(iters: usize, mut f: impl FnMut()) -> f64 {
    let mut xs: Vec<f64> = Vec::new();
    for _ in 0..iters {
        let t = std::time::Instant::now();
        f();
        xs.push(t.elapsed().as_secs_f64() * 1e3);
    }
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs[xs.len() / 2]
}

#[test]
#[ignore = "measurement bench; run with --release --ignored --nocapture"]
fn bench_compile_vs_validate() {
    println!("\n=== WebXR semantic bench (ms; median-of-3) — parse+lowering vs validation ===");
    println!("n_entities | parse+lower ms | validate ms | validate us/entity");
    for &n in &[100usize, 1_000, 10_000, 100_000] {
        let src = gen_scene(n);
        // parse + lowering (median over fresh compiles)
        let compile_ms = median_ms(3, || {
            let _ = compile_scene(&src);
        });
        // validate on the already-compiled Ir
        let c = compile_scene(&src);
        let (ir, it) = (c.ir(), c.it());
        let validate_ms = median_ms(3, || {
            let v = validate(ir, it);
            assert!(v.is_empty(), "generated scene must be valid at n={n}");
        });
        println!(
            "{n:>9} | {compile_ms:>13.1} | {validate_ms:>10.2} | {:>6.2}",
            validate_ms * 1000.0 / n as f64
        );
    }
    println!();
}

// ---------------------------------------------------------------------------------------------
// coverage-closing: vocabulary-missing, anchoring roles, error Display
// ---------------------------------------------------------------------------------------------

#[test]
fn vocabulary_missing_when_not_imported() {
    // a scene that does not import the webxr vocabulary at all
    let provider = MemProvider::default()
        .with_file("/bare.hymeko", "bare {}\nroot { a {} b {} }\n".to_string());
    let mut store = ModuleStore::new(provider, RealParser);
    let prog = store.compile(Path::new("/bare.hymeko")).unwrap();
    let v = validate(&prog.ir, &store.it);
    assert_eq!(v, vec![SemanticError::VocabularyMissing]);
}

#[test]
fn anchoring_without_anchor_is_wrong_role() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        ent:   + <isa> s.kinds.spatial_entity {}
        @an: s.relations.anchoring { (+ ent, - frame); }
    "#,
    );
    assert!(
        v.iter().any(|e| matches!(
            e,
            SemanticError::RelationRole {
                reason: RoleReason::NoTarget,
                ..
            }
        )),
        "{v:#?}"
    );
}

#[test]
fn anchoring_non_entity_subject_is_wrong_kind() {
    let v = errs(
        r#"
        @xr_session: x.session.session_config { (+ x.session_modes.immersive_ar, - x.reference_spaces.local_floor); }
        frame: + <isa> s.kinds.spatial_frame {}
        anc:   + <isa> s.kinds.semantic_anchor {}
        notent: + <isa> s.kinds.spatial_frame {}
        @an: s.relations.anchoring { (+ notent, - frame, ~ anc); }
    "#,
    );
    assert!(
        v.iter().any(|e| matches!(
            e,
            SemanticError::RelationRole {
                reason: RoleReason::WrongKind { .. },
                ..
            }
        )),
        "{v:#?}"
    );
}

#[test]
fn error_display_is_non_empty_and_ids_only() {
    use hymeko::common::ids::SymId;
    let samples = [
        SemanticError::VocabularyMissing,
        SemanticError::SessionModeCardinality {
            scene: DeclId::new(1),
            found: 0,
        },
        SemanticError::FrameCycle {
            frame: DeclId::new(2),
        },
        SemanticError::QuaternionNotNormalizable {
            relation: DeclId::new(3),
        },
        SemanticError::RelationRole {
            relation: DeclId::new(4),
            archetype: SymId::new(5),
            reason: RoleReason::NoTarget,
        },
        SemanticError::RuntimeHandleField {
            decl: DeclId::new(6),
            field: SymId::new(7),
        },
    ];
    for e in samples {
        let s = format!("{e}");
        assert!(!s.is_empty());
        let _: &dyn std::error::Error = &e;
    }
}
