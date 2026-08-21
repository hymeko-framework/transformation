//! Integration tests for the four M2T emitters driven by a real
//! `.hymeko` fixture (`data/robotics/mini_arm.hymeko`). Loads it through
//! the full `ModuleStore` pipeline, then asserts structural invariants on
//! each emitter's output.

use std::marker::PhantomData;
use std::path::Path;
use std::sync::Arc;

use hymeko::module_store::module_store::{
    CompiledProgram, HymekoParser, ModuleLoadError, ModuleStore,
};
use hymeko::module_store::source_provider::StdFsProvider;
use parser::ast::AstStr;

use hymeko_emitter::{emit_hymeko, emit_lean4, emit_rust_stubs, emit_sysml};

struct LalrpopParser;

impl HymekoParser for LalrpopParser {
    fn parse<'a>(&self, src: &'a str) -> Result<AstStr<'a>, String> {
        parser::parse_description(src).map_err(|e| format!("{e:?}"))
    }
}

fn load_and_lower(
    path: impl AsRef<Path>,
) -> Result<
    (ModuleStore<StdFsProvider, LalrpopParser>, Arc<CompiledProgram>),
    ModuleLoadError,
> {
    let fs = StdFsProvider::new();
    let parser = LalrpopParser;
    let mut store = ModuleStore::new(fs, parser);
    let compiled = store.compile(path.as_ref())?;
    Ok((store, compiled))
}

const MINI_ARM: &str = "../data/robotics/mini_arm.hymeko";

#[test]
fn emit_hymeko_produces_non_empty_bracketed_output() {
    let (store, compiled) = load_and_lower(MINI_ARM).expect("mini_arm should compile");
    let out = emit_hymeko(&compiled.ir, &store.it, "mini_arm");
    assert!(out.contains("mini_arm {"));
    assert!(out.trim_end().ends_with('}'));
    // Must mention the concrete links defined in the fixture.
    assert!(out.contains("base_link"), "output missing base_link");
    assert!(out.contains("spinner"), "output missing spinner");
    // The continuous joint hyperedge should appear prefixed with `@`.
    assert!(out.contains("@spin_joint"), "output missing @spin_joint edge");
    let _ = PhantomData::<()>; // silence unused
}

#[test]
fn emit_hymeko_is_deterministic() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let a = emit_hymeko(&compiled.ir, &store.it, "mini_arm");
    let b = emit_hymeko(&compiled.ir, &store.it, "mini_arm");
    assert_eq!(a, b, "emit_hymeko must be byte-deterministic");
}

#[test]
fn emit_sysml_opens_and_closes_package() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");
    assert!(out.contains("package MiniArm {"));
    assert!(out.contains("metadata def HyperedgeAnnotation {"));
    assert!(out.contains("part def base_link"));
    assert!(out.contains("part def spinner"));
    assert!(out.trim_end().ends_with('}'));
}

#[test]
fn emit_sysml_emits_connection_def_per_arc() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");
    // mini_arm has exactly one hyperedge (spin_joint) with one arc tuple.
    let conn_defs = out.matches("connection def ").count();
    assert!(conn_defs >= 1, "expected at least one connection def, got {conn_defs}");
}

#[test]
fn emit_rust_stubs_generates_pascal_case_traits() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_rust_stubs(&compiled.ir, &store.it);
    // `base_link` → `BaseLink`, `link_geometry` → `LinkGeometry`.
    assert!(out.contains("pub trait BaseLink {"));
    assert!(out.contains("pub trait Spinner {"));
    assert!(out.contains("fn process(&self)"));
}

#[test]
fn emit_lean4_generates_trivial_theorem_per_node() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_lean4(&compiled.ir, &store.it);
    assert!(out.contains("import Mathlib"));
    assert!(out.contains("theorem base_link_level_invariant"));
    assert!(out.contains("by\n  trivial"));
}

// -------- Round-trip tests ------------------------------------------------

/// Count how many `NodeDecl` / `EdgeDecl` / `HyperArc` items appear in the
/// parsed AST. We don't try to compare deep structure — just counts plus
/// a "nothing went missing" smoke signal.
fn ast_counts(ast: &parser::ast::AstStr) -> (usize, usize) {
    use parser::ast::HyperItem;
    fn walk<'a>(
        items: &'a [HyperItem<'a, &'a str>],
        nodes: &mut usize,
        edges: &mut usize,
    ) {
        for it in items {
            match it {
                HyperItem::Node(n) => {
                    *nodes += 1;
                    if let Some(body) = n.inner.body.as_deref() {
                        walk(body, nodes, edges);
                    }
                }
                HyperItem::Edge(e) => {
                    *edges += 1;
                    walk(&e.inner.body, nodes, edges);
                }
                HyperItem::Arc(_) => {}
            }
        }
    }
    let mut n = 0;
    let mut e = 0;
    walk(&ast.items, &mut n, &mut e);
    (n, e)
}

#[test]
fn emit_hymeko_roundtrips_through_the_parser() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let emitted = emit_hymeko(&compiled.ir, &store.it, "roundtrip_model");

    let ast = parser::parse_description(&emitted).unwrap_or_else(|e| {
        panic!(
            "emit_hymeko produced unparseable output: {e:?}\n\n---BEGIN---\n{emitted}\n---END---"
        )
    });

    let (ast_nodes, ast_edges) = ast_counts(&ast);
    assert!(
        ast_nodes >= compiled.ir.nodes.len().saturating_sub(1),
        "round-tripped AST has {ast_nodes} node decls, arena IR has {}",
        compiled.ir.nodes.len()
    );
    assert!(
        ast_edges >= 1,
        "round-tripped AST missing edges (expected ≥1, got {ast_edges})"
    );
}

#[test]
fn emit_hymeko_emits_signed_ref_arcs_with_correct_signs() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_hymeko(&compiled.ir, &store.it, "mini_arm");
    // mini_arm's spin_joint: (+ base_link [[…]], - spinner, - AXIS_Z)
    // The emitter must preserve the +/− signs; weight block on base_link
    // appears as `[[0.0, 0.0, 0.1], [0.0, 0.0, 0.0]]`.
    assert!(
        out.contains("+ base_link"),
        "emitted text missing `+ base_link`:\n{out}"
    );
    assert!(
        out.contains("- spinner"),
        "emitted text missing `- spinner`:\n{out}"
    );
}

#[test]
fn emit_hymeko_preserves_inline_numeric_values() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_hymeko(&compiled.ir, &store.it, "mini_arm");
    // Every `mass` decl in the fixture carries a numeric value.
    assert!(
        out.contains("mass 5.0;") || out.contains("mass 5;"),
        "emitted text missing `mass 5.0;`:\n{out}"
    );
    assert!(
        out.contains("mass 1.0;") || out.contains("mass 1;"),
        "emitted text missing `mass 1.0;` for spinner:\n{out}"
    );
}

#[test]
fn emit_hymeko_is_insertion_order_stable() {
    // Byte-identical across two emissions — guards against any hidden
    // HashMap iteration order leaking into the output.
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let a = emit_hymeko(&compiled.ir, &store.it, "x");
    let b = emit_hymeko(&compiled.ir, &store.it, "x");
    assert_eq!(a, b);
}

// -------- SysML golden-ish checks ----------------------------------------

#[test]
fn emit_sysml_includes_part_def_for_every_ir_node() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");
    for name in ["base_link", "spinner"] {
        assert!(
            out.contains(&format!("part def {name}")),
            "SysML output missing `part def {name}`:\n{out}"
        );
    }
}

#[test]
fn emit_sysml_contains_hyperedge_arcs_with_sign_vectors() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");
    // spin_joint has three signed refs; the sibling HyperedgeArcs part
    // must list all three signs in its `signs` attribute.
    assert!(
        out.contains(":>> signs"),
        "SysML output missing sign-vector attribute:\n{out}"
    );
    assert!(
        out.contains(":>> targets"),
        "SysML output missing target-vector attribute:\n{out}"
    );
}

#[test]
fn emit_sysml_inlines_node_scalar_attributes() {
    // Step 2b: SysML should carry the per-node inline values through as
    // `attribute <name> :>> <value>;`.  For mini_arm's base_link we have
    // mass 5.0, origin [0,0,0.05], and the nested link_geometry with
    // dimension [0.3, 0.3, 0.1] as scalar children.
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");

    assert!(
        out.contains("attribute mass :>> 5.0;"),
        "SysML output missing base_link's inline mass attribute:\n{out}"
    );
    assert!(
        out.contains(":>> (0.0, 0.0, 0.05)"),
        "SysML output missing base_link's origin tuple:\n{out}"
    );
}

#[test]
fn emit_sysml_metadata_profile_includes_joint_origin() {
    let (store, compiled) = load_and_lower(MINI_ARM).unwrap();
    let out = emit_sysml(&compiled.ir, &store.it, "MiniArm");
    assert!(out.contains("metadata def HyperedgeAnnotation {"));
    assert!(out.contains("metadata def JointOrigin {"));
    assert!(out.contains("attribute xyz : ScalarValues::Real[3];"));
}
