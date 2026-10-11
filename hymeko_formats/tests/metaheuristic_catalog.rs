//! Public, Core-compiled controls for descriptive structural identification.
use hymeko_formats::metaheuristic::{catalog, compile};
use serde_json::Value;

const CORPUS: &[(&str, &str)] = &[
    (
        "de",
        include_str!("../data/metaheuristics/catalog/de.hymeko"),
    ),
    (
        "ga",
        include_str!("../data/metaheuristics/catalog/ga.hymeko"),
    ),
    (
        "pso",
        include_str!("../data/metaheuristics/catalog/pso.hymeko"),
    ),
    (
        "hho",
        include_str!("../data/metaheuristics/catalog/hho.hymeko"),
    ),
    (
        "gwo",
        include_str!("../data/metaheuristics/catalog/gwo.hymeko"),
    ),
    (
        "woa",
        include_str!("../data/metaheuristics/catalog/woa.hymeko"),
    ),
    (
        "ba",
        include_str!("../data/metaheuristics/catalog/ba.hymeko"),
    ),
    (
        "fa",
        include_str!("../data/metaheuristics/catalog/fa.hymeko"),
    ),
    (
        "cs",
        include_str!("../data/metaheuristics/catalog/cs.hymeko"),
    ),
    (
        "abc",
        include_str!("../data/metaheuristics/catalog/abc.hymeko"),
    ),
];

fn project(source: &str) -> Value {
    catalog::project(&compile(source).unwrap()).unwrap()
}

#[test]
fn ten_descriptors_validate_and_roundtrip_through_core_and_emitter() {
    for (name, source) in CORPUS {
        let model = compile(source).unwrap();
        let first = catalog::project(&model).unwrap();
        assert_eq!(first["schema"], "hymeko.metaheuristic-structure/1");
        assert!(
            first["query_checks"]
                .as_array()
                .unwrap()
                .iter()
                .all(|c| c["passed"] == true)
        );
        let emitted =
            hymeko_emitter::emit_hymeko(&model.program().ir, model.interner(), "roundtrip");
        let second = project(&emitted);
        assert_eq!(
            first["model_semantic_digest"], second["model_semantic_digest"],
            "{name}"
        );
        assert_eq!(first["features"], second["features"], "{name}");
    }
}

#[test]
fn names_metadata_and_raw_declaration_order_cannot_drive_identification() {
    for (name, source) in CORPUS {
        let first = project(source);
        let disguised = source
            .replace("slot_", "anonymous_")
            .replace("op_", "transition_")
            .replace(first["label"].as_str().unwrap(), "Unrelated animal label");
        let second = project(&disguised);
        assert_ne!(
            first["model_semantic_digest"],
            second["model_semantic_digest"]
        );
        assert_eq!(first["features"], second["features"], "{name}");
        // A controlled fixture mutation, not a source parser: reverse one-line state declarations.
        let mut lines: Vec<_> = source.lines().collect();
        let indices: Vec<_> = lines
            .iter()
            .enumerate()
            .filter_map(|(i, l)| l.trim_start().starts_with("slot_").then_some(i))
            .collect();
        let reverse: Vec<_> = indices.iter().rev().map(|&i| lines[i]).collect();
        for (&i, line) in indices.iter().zip(reverse) {
            lines[i] = line;
        }
        assert_eq!(
            first["features"],
            project(&lines.join("\n"))["features"],
            "{name}"
        );
    }
}

#[test]
fn malformed_roles_foreign_states_and_duplicate_or_missing_stages_are_rejected() {
    let source = CORPUS[0].1;
    for altered in [
        source.replacen(
            "@reads: vocabulary.read_role",
            "@reads: vocabulary.write_role",
            1,
        ),
        source.replacen(
            "~ method.states.slot_bounds",
            "+ method.states.slot_bounds",
            1,
        ),
        source.replacen("~ method.states.slot_bounds", "~ vocabulary.state_kind", 1),
        source.replacen(
            "~ method.states.slot_bounds",
            "~ method.states.slot_random",
            1,
        ),
        source.replacen("stage 1;", "stage 0;", 1),
        source.replacen("stage 1;", "stage 99;", 1),
        source.replacen("@op_initialize:", "op_initialize:", 1),
        source.replacen(
            "family \"uniform_initialization\"",
            "family \"animal_name\"",
            1,
        ),
    ] {
        assert_ne!(source, altered);
        if let Ok(model) = compile(&altered) {
            assert!(catalog::project(&model).is_err());
        }
    }
}

#[test]
fn formula_counterexample_explicitly_limits_structural_identification() {
    let source = CORPUS[0].1;
    let altered = source.replace("v = x_a + F*(x_b-x_c).", "v = x_a + F*(x_b+x_c).");
    assert_ne!(source, altered);
    let (first, second) = (project(source), project(&altered));
    assert_ne!(
        first["model_semantic_digest"],
        second["model_semantic_digest"]
    );
    assert_eq!(
        first["features"], second["features"],
        "a motif projection is deliberately not arithmetic equivalence"
    );
    assert_ne!(first["operators"], second["operators"]);
}

#[test]
fn mechanism_changes_are_distinguished_even_when_support_is_identical() {
    let source = CORPUS[0].1;
    let altered = source.replace(
        "family \"difference_mutation\"",
        "family \"uniform_perturbation\"",
    );
    let (first, second) = (project(source), project(&altered));
    assert_eq!(first["features"]["support"], second["features"]["support"]);
    assert_ne!(
        first["features"]["mechanisms"],
        second["features"]["mechanisms"]
    );
}

#[test]
fn sharing_a_state_is_visible_beyond_local_operator_degree_counts() {
    let source = CORPUS[0].1;
    // Both have the same two scalar states and one scalar read at this operator.
    // Rewiring changes the shared-state neighborhoods while preserving local degrees.
    let first = source.replace(
        "    states {",
        "    states {\n        slot_unused: vocabulary.state_kind { space \"scalar\"; }",
    );
    let second = first.replacen(
        "~ method.states.slot_parameters);",
        "~ method.states.slot_unused);",
        1,
    );
    assert_ne!(first, second);
    assert_ne!(
        project(&first)["features"]["support"],
        project(&second)["features"]["support"]
    );
}

#[test]
fn state_and_operator_caps_accept_the_boundary_and_reject_the_next_member() {
    let source = CORPUS[0].1;
    let baseline = project(source);
    let states = baseline["states"].as_array().unwrap().len();
    let operators = baseline["operators"].as_array().unwrap().len();
    for limit in [64, 65] {
        let extra_states: String = (states..limit)
            .map(|i| format!("extra_{i}: vocabulary.state_kind {{ space \"scalar\"; }}\n"))
            .collect();
        let changed = source.replacen("states {", &format!("states {{\n{extra_states}"), 1);
        assert_eq!(
            catalog::project(&compile(&changed).unwrap()).is_ok(),
            limit == 64
        );

        let extra_operators: String = (operators..limit)
            .map(|i| {
                format!(
                    "@extra_{i}: vocabulary.operator_kind {{ stage {i}; category \"vary\"; \
             family \"uniform_perturbation\"; guard \"always\"; rule \"bounded fixture\"; \
             @reads: vocabulary.read_role {{ (~ method.states.slot_random); }} \
             @writes: vocabulary.write_role {{ (~ method.states.slot_population); }} }}\n"
                )
            })
            .collect();
        let changed = source.replacen(
            "operators {",
            &format!("operators {{\n{extra_operators}"),
            1,
        );
        assert_eq!(
            catalog::project(&compile(&changed).unwrap()).is_ok(),
            limit == 64
        );
    }
}
