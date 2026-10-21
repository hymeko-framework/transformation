use std::path::Path;

use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{Ir, PortDirection};
use hymeko::module_store::{module_store::ModuleStore, source_provider::StdFsProvider};
use hymeko::resolution::string_table::StringTable;
use hymeko::util::real_parser::RealParser;
use hymeko_formats::snapshot::{dot_escape, emit_dot_graph};

fn fixture() -> (Ir, StringTable) {
    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/diagram.hymeko");
    let program = store.compile(&path).expect("real source compiles");
    (program.ir.clone(), StringTable::from_interner(&store.it))
}

fn named(ir: &Ir, st: &StringTable, name: &str) -> DeclId {
    DeclId::new(
        ir.decl_nodes
            .iter()
            .position(|d| st.resolve(d.name) == name)
            .unwrap(),
    )
}

#[test]
fn source_values_references_inheritance_and_duplicate_names_survive() {
    let (ir, st) = fixture();
    let out = emit_dot_graph(&ir, &st, "source diagram");
    assert!(out.contains("label=\"length\\n= 1\""));
    assert!(out.contains("label=\"dimension\\n= length\""));
    for kind in ["scope", "isa", "ref"] {
        assert!(out.contains(&format!("label=\"{kind}\"")));
    }
    let same_names: Vec<_> = ir
        .decl_nodes
        .iter()
        .enumerate()
        .filter(|(_, d)| st.resolve(d.name) == "a")
        .map(|(id, _)| id)
        .collect();
    assert_eq!(same_names.len(), 2);
    for id in same_names {
        assert!(out.contains(&format!("\"n{id}\" [id=\"decl-{id}\", label=\"a\"")));
    }
    let alias = named(&ir, &st, "alias");
    let hymeko::ir::ir::ValueR::Ref(target) =
        ir.decl_annotation(alias).unwrap().value.as_ref().unwrap()
    else {
        panic!()
    };
    assert!(out.lines().any(|line| line.starts_with(&format!(
        "  \"n{}\" -> \"n{}\"",
        alias.raw(),
        target.raw()
    )) && line.contains("label=\"ref\"")));
}

#[test]
fn arc_groups_repeated_members_weights_and_edge_targets_remain_distinct() {
    let (ir, st) = fixture();
    let out = emit_dot_graph(&ir, &st, "arcs");
    let constraint = named(&ir, &st, "constraint");
    let edge = &ir.edges[ir.as_edge(constraint).unwrap().raw()];
    assert_eq!(edge.arcs.len(), 2);
    assert_eq!(out.matches("id=\"arc-").count(), 3);
    assert_eq!(out.matches("id=\"incidence-").count(), 7);
    let first = edge.arcs[0].raw();
    let a = named(&ir, &st, "a").raw();
    assert_eq!(out.matches(&format!("\"a{first}\" -> \"n{a}\"")).count(), 2);
    assert!(out.contains("label=\"+ [2] w=[2]\""));
    assert!(out.lines().any(
        |line| line.contains(&format!("-> \"e{}\"", constraint.raw()))
            && line.contains("incidence-")
    ));
    let empty = named(&ir, &st, "empty").raw();
    assert!(out.contains(&format!("id=\"decl-{empty}\"")));
}

#[test]
fn arrows_follow_explicit_direction_independently_of_all_three_signs() {
    let (mut ir, st) = fixture();
    let constraint = named(&ir, &st, "constraint");
    let aid = ir.edges[ir.as_edge(constraint).unwrap().raw()].arcs[0].raw();
    let original = ir.arcs[aid].refs[0];
    for (direction, dot_dir) in [
        (PortDirection::In, "back"),
        (PortDirection::Out, "forward"),
        (PortDirection::Unspecified, "none"),
    ] {
        for sign in [1, -1, 0] {
            let kind = match sign {
                1 => hymeko::ir::ir::IncidenceSign::Plus,
                -1 => hymeko::ir::ir::IncidenceSign::Minus,
                _ => hymeko::ir::ir::IncidenceSign::Neutral,
            };
            ir.arcs[aid].refs[0] = original.with_direction(direction).with_sign(kind);
            let out = emit_dot_graph(&ir, &st, "directions");
            let line = out
                .lines()
                .find(|line| line.contains(&format!("id=\"incidence-{aid}-0\"")))
                .unwrap();
            assert!(line.contains(&format!("dir={dot_dir},")), "{line}");
            let glyph = match sign {
                1 => "+",
                -1 => "-",
                _ => "~",
            };
            assert!(line.contains(&format!("label=\"{glyph} [0]\"")), "{line}");
        }
    }
}

#[test]
fn labels_escape_dot_syntax_and_do_not_inject_statements() {
    let text = "árvíz\"\\N\n\r\t[]<>";
    assert_eq!(dot_escape(text), "árvíz\\\"\\\\N\\n\\r    []<>");
    let (ir, st) = fixture();
    let out = emit_dot_graph(&ir, &st, text);
    assert!(out.lines().nth(1).unwrap().contains(&dot_escape(text)));
    assert!(!out.contains('\r'));
    assert!(!out.contains('\t'));
}

#[test]
fn a_reference_to_a_hyperarc_uses_its_existing_junction() {
    let (mut ir, st) = fixture();
    let (decl, aid) = ir
        .decl_nodes
        .iter()
        .enumerate()
        .find_map(|(i, _)| ir.as_arc(DeclId::new(i)).map(|aid| (DeclId::new(i), aid)))
        .expect("compiled arc has a declaration");
    let wrapper = named(&ir, &st, "wrapper");
    let wrapper_arc = ir.edges[ir.as_edge(wrapper).unwrap().raw()].arcs[0].raw();
    ir.arcs[wrapper_arc].refs[0] = ir.arcs[wrapper_arc].refs[0]
        .with_target(decl)
        .with_sign(hymeko::ir::ir::IncidenceSign::Neutral);
    let out = emit_dot_graph(&ir, &st, "arc target");
    assert!(out.contains(&format!(
        "\"a{wrapper_arc}\" -> \"a{}\" [id=\"incidence-{wrapper_arc}-0\"",
        aid.raw()
    )));
    assert!(!out.contains(&format!("\"n{}\"", decl.raw())));
}

#[test]
fn output_is_deterministic_and_does_not_mutate_the_ir() {
    let (ir, st) = fixture();
    let before = serde_json::to_string(&ir).unwrap();
    assert_eq!(
        emit_dot_graph(&ir, &st, "same"),
        emit_dot_graph(&ir, &st, "same")
    );
    assert_eq!(before, serde_json::to_string(&ir).unwrap());
}

#[test]
fn empty_ir_has_no_fabricated_declarations_or_incidences() {
    let out = emit_dot_graph(&Ir::default(), &StringTable::from_vec(vec![]), "empty");
    assert!(out.contains("digraph \"empty\""));
    assert!(out.ends_with("}\n"));
    assert!(!out.contains("id=\""));
}
