//! End-to-end trace scenarios use the real Core snapshot and registered format backends.
use hymeko::{
    ir::ir::ValueR,
    module_store::{
        execution::{CompiledModel, ExecutionState, ValueUpdate},
        source_provider::MemProvider,
    },
    util::real_parser::RealParser,
};
use hymeko_formats::traceability::TraceabilityExport;
use hymeko_query::{Predicate, QueryEngine};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

const SOURCE: &str = include_str!("../../data/paper/traceability_verification.hymeko");
const META: &str = include_str!("../../data/paper/meta_sysml_trace.hymeko");
const EXTENSION: &str = r#"
    R_logging: meta_sysml_trace.sysml_trace.elements.requirement { text "Persist the audit log"; }
    Logger: meta_sysml_trace.sysml_trace.elements.block {}
    T_logging: meta_sysml_trace.sysml_trace.elements.test_case { outcome 1; }
    @sat_logging: meta_sysml_trace.sysml_trace.satisfies { (+ Logger, - R_logging); }
    @verify_logging: meta_sysml_trace.sysml_trace.verifies { (+ T_logging, - R_logging); }
    @alloc_logging: meta_sysml_trace.sysml_trace.allocated_to { (+ R_logging, - Logger); }
"#;

struct TraceCase {
    model: CompiledModel,
}

impl TraceCase {
    fn compile(source: &str) -> Self {
        let provider = MemProvider::default()
            .with_file("/model.hymeko", source)
            .with_file("/meta_sysml_trace.hymeko", META);
        Self {
            model: CompiledModel::compile(provider, RealParser, Path::new("/model.hymeko"))
                .unwrap(),
        }
    }

    fn capture(model: &CompiledModel, name: &str) -> (Value, BTreeMap<String, String>) {
        let ir_before = serde_json::to_vec(&model.program().ir).unwrap();
        let export = TraceabilityExport::new(model, "verification_cell").unwrap();
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../transforms");
        let views = export.emit(&root).unwrap();
        assert_eq!(views, export.emit(&root).unwrap());
        assert_eq!(ir_before, serde_json::to_vec(&model.program().ir).unwrap());
        let reference = export.reference();
        if let Some(directory) = std::env::var_os("HYMEKO_TRACE_EVIDENCE") {
            let folder = PathBuf::from(directory).join(name);
            std::fs::create_dir_all(&folder).unwrap();
            for (ext, text) in &views {
                std::fs::write(folder.join(format!("trace.{ext}")), text).unwrap();
            }
            std::fs::write(
                folder.join("reference.json"),
                serde_json::to_vec_pretty(&reference).unwrap(),
            )
            .unwrap();
            std::fs::write(folder.join("snapshot.json"), serde_json::to_vec_pretty(&json!({
                "semantic_digest":format!("{:?}",model.semantic_digest()),"repeat_equal":true,"ir_unchanged":true
            })).unwrap()).unwrap();
        }
        (reference, views)
    }
}

#[test]
fn full_reference_includes_named_parallel_edges_and_all_required_views() {
    let case = TraceCase::compile(SOURCE);
    let (reference, views) = TraceCase::capture(&case.model, "baseline");
    let expected: Value =
        serde_json::from_str(include_str!("../../tools/traceability/expected.json")).unwrap();
    assert_eq!(reference, expected);
    assert_eq!(views.len(), 3);
    assert_eq!(reference["entities"].as_object().unwrap().len(), 19);
    assert_eq!(reference["relations"].as_object().unwrap().len(), 19);
    assert_eq!(
        reference["relations"]["sat_stop"],
        reference["relations"]["sat_stop_review"]
    );
    for view in views.values() {
        assert!(view.contains("sat_stop_review"));
    }
}

#[test]
fn checked_outcome_update_publishes_three_fresh_views_and_retains_old_readers() {
    let state = ExecutionState::new(TraceCase::compile(SOURCE).model);
    let old = state.snapshot().unwrap();
    let (mut expected, before) = TraceCase::capture(old.model(), "value-before");
    let query = QueryEngine::new(&old.model().program().ir, old.model().interner());
    let test = query
        .query_first(&Predicate::Named("T_cycle".into()))
        .unwrap()
        .id;
    let slot = query.own_child(test, "outcome").unwrap();
    let updated = old
        .model()
        .with_value_updates(&[ValueUpdate::Set {
            declaration: slot,
            value: ValueR::Num(1.),
        }])
        .unwrap();
    assert_eq!(
        old.model().program().canon.structure(),
        updated.program().canon.structure()
    );
    assert_ne!(old.model().semantic_digest(), updated.semantic_digest());
    let published = state
        .commit(state.prepare_model_at(&old, updated).unwrap())
        .unwrap();
    let (reference, after) = TraceCase::capture(published.model(), "value-after");
    expected["entities"]["T_cycle"]["outcome"] = json!(1);
    assert_eq!(reference, expected);
    for ext in ["sysml", "dot", "csv"] {
        assert_ne!(before[ext], after[ext]);
    }
    assert_eq!(
        before,
        TraceCase::capture(old.model(), "retained-value-before").1
    );
    let stale = old.model().with_value_updates(&[]).unwrap();
    assert!(state.prepare_model_at(&old, stale).is_err());
}

#[test]
fn structural_recompile_propagates_the_new_requirement_component_and_test() {
    let state = ExecutionState::new(TraceCase::compile(SOURCE).model);
    let old = state.snapshot().unwrap();
    let (mut expected, before) = TraceCase::capture(old.model(), "structure-before");
    let source = format!(
        "{}\n{EXTENSION}\n}}",
        SOURCE.trim_end().strip_suffix('}').unwrap()
    );
    let updated = TraceCase::compile(&source).model;
    assert_ne!(
        old.model().program().canon.structure(),
        updated.program().canon.structure()
    );
    let published = state
        .commit(state.prepare_model_at(&old, updated).unwrap())
        .unwrap();
    let (reference, after) = TraceCase::capture(published.model(), "structure-after");
    for (name, kind, text, outcome) in [
        (
            "R_logging",
            "requirement",
            Some("Persist the audit log"),
            None,
        ),
        ("Logger", "component", None, None),
        ("T_logging", "test_case", None, Some(1)),
    ] {
        expected["entities"][name] = json!({"kind":kind,"text":text,"outcome":outcome});
    }
    for (name, kind, source, target) in [
        ("sat_logging", "satisfies", "Logger", "R_logging"),
        ("verify_logging", "verifies", "T_logging", "R_logging"),
        ("alloc_logging", "allocated_to", "R_logging", "Logger"),
    ] {
        expected["relations"][name] = json!({"kind":kind,"source":source,"target":target});
    }
    assert_eq!(reference, expected);
    for ext in ["sysml", "dot", "csv"] {
        assert_ne!(before[ext], after[ext]);
        assert!(after[ext].contains("T_logging"));
    }
    assert_eq!(
        before,
        TraceCase::capture(old.model(), "retained-structure-before").1
    );
}

#[test]
fn reordered_source_preserves_the_full_reference() {
    let original = TraceCase::compile(SOURCE);
    let mut body: Vec<_> = SOURCE
        .lines()
        .skip(3)
        .filter(|line| !line.trim().starts_with("//") && line.trim() != "}")
        .collect();
    body.reverse();
    let source = format!(
        "{}\n{}\n}}",
        SOURCE.lines().take(3).collect::<Vec<_>>().join("\n"),
        body.join("\n")
    );
    let reordered = TraceCase::compile(&source);
    assert_eq!(
        original.model.semantic_digest(),
        reordered.model.semantic_digest()
    );
    assert_eq!(
        TraceabilityExport::new(&original.model, "verification_cell")
            .unwrap()
            .reference(),
        TraceCase::capture(&reordered.model, "reordered").0
    );
}

#[test]
fn unsupported_inputs_fail_before_any_truncated_export() {
    let base = "(+ SafetyController, - R_stop);";
    let mut cases = vec![
        (
            "polarity",
            SOURCE.replacen(base, "(- SafetyController, + R_stop);", 1),
        ),
        (
            "neutral",
            SOURCE.replacen(base, "(~ SafetyController, - R_stop);", 1),
        ),
        (
            "nary",
            SOURCE.replacen(base, "(+ SafetyController, + Arm, - R_stop);", 1),
        ),
        (
            "multi_arc",
            SOURCE.replacen(
                base,
                "(+ SafetyController, - R_stop); (+ Arm, - R_accuracy);",
                1,
            ),
        ),
        (
            "weight",
            SOURCE.replacen(base, "(+ SafetyController [2], - R_stop);", 1),
        ),
        (
            "same_sign",
            SOURCE.replacen(base, "(+ SafetyController, + R_stop);", 1),
        ),
        (
            "wrong_kind",
            SOURCE.replacen(base, "(+ T_stop, - R_stop);", 1),
        ),
        (
            "unknown_outcome",
            SOURCE.replacen("outcome 0;", "outcome 2;", 1),
        ),
        ("missing_outcome", SOURCE.replacen("outcome 0;", "", 1)),
        (
            "unsafe_text",
            SOURCE.replacen("Stop within 200 ms", "Stop */ injected", 1),
        ),
        (
            "nested_scope",
            format!(
                "{SOURCE}\nother {{ X: meta_sysml_trace.sysml_trace.elements.requirement {{}} }}"
            ),
        ),
        (
            "duplicate_local_name",
            format!(
                "{SOURCE}\nother {{ R_stop: meta_sysml_trace.sysml_trace.elements.requirement {{}} }}"
            ),
        ),
        (
            "unrelated_named_type",
            format!("{SOURCE}\nother {{ requirement {{}} X: requirement {{}} }}"),
        ),
    ];
    cases.push((
        "empty_scope",
        SOURCE.replace("verification_cell:", "unused:") + "\nverification_cell {}",
    ));
    for (name, source) in &cases {
        let case = TraceCase::compile(source);
        assert!(
            TraceabilityExport::new(&case.model, "verification_cell").is_err(),
            "{name}"
        );
    }
    if let Some(directory) = std::env::var_os("HYMEKO_TRACE_EVIDENCE") {
        std::fs::create_dir_all(&directory).unwrap();
        let record: Vec<_> = cases
            .iter()
            .map(|(name, _)| json!({"case":name,"rejected":true}))
            .collect();
        std::fs::write(
            PathBuf::from(directory).join("negative-inputs.json"),
            serde_json::to_vec_pretty(&record).unwrap(),
        )
        .unwrap();
    }
}

#[test]
fn missing_template_is_an_error_not_a_cached_success() {
    let case = TraceCase::compile(SOURCE);
    let export = TraceabilityExport::new(&case.model, "verification_cell").unwrap();
    assert!(
        export
            .emit(Path::new("__absent_trace_templates__"))
            .is_err()
    );
}

#[test]
fn original_paper_witness_remains_supported_without_invented_test_results() {
    let case = TraceCase::compile(include_str!("../../data/paper/traceability_smc.hymeko"));
    let export = TraceabilityExport::new(&case.model, "pick_place_cell").unwrap();
    assert_eq!(export.projection().entities.len(), 8);
    assert_eq!(export.projection().relations.len(), 7);
    assert!(
        export
            .projection()
            .entities
            .iter()
            .all(|e| e.outcome.is_none())
    );
    assert_eq!(
        export
            .emit(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../transforms"))
            .unwrap()
            .len(),
        3
    );
}
