use hymeko_formats::metaheuristic::{HHO_KEY, compile, validate};

const SOURCE: &str = include_str!("../data/metaheuristics/hho_sequential_budget.hymeko");

#[test]
fn hho_profile_roundtrips_with_exact_budget_clock_and_observation_contracts() {
    let model = compile(SOURCE).unwrap();
    let contract = validate(&model).unwrap();
    let operators = contract["operators"].as_object().unwrap();
    let states = contract["states"].as_object().unwrap();
    assert_eq!((contract["profile"].as_str(), operators.len(), states.len()),
               (Some(HHO_KEY), 11, 26));
    assert!(operators.contains_key("hho.reevaluate_parent"));
    let text = hymeko_emitter::emit_hymeko(&model.program().ir, model.interner(), "hho_contract");
    let second = validate(&compile(&text).unwrap()).unwrap();
    assert_eq!(
        contract["model_semantic_digest"],
        second["model_semantic_digest"]
    );
    assert_eq!(contract["profile_fields"], second["profile_fields"]);
}

#[test]
fn silently_changed_accounting_clock_and_parent_dependencies_fail() {
    for (before, after) in [
        (
            "initialization_and_repeated_parent_calls_included",
            "initialization_free",
        ),
        ("factor_frozen_after_scan", "factor_recomputed_per_target"),
        ("clamp_each_objective_input", "clamp_only_population_scans"),
        (
            "charge_current_parent_again_even_if_cached",
            "use_cached_parent_score",
        ),
        (
            "~ states.positions, ~ states.target_index, ~ states.bounds",
            "~ states.proposal, ~ states.target_index, ~ states.bounds",
        ),
        (
            "@reads: vocabulary.read_role",
            "@reads: vocabulary.write_role",
        ),
    ] {
        let altered = SOURCE.replacen(before, after, 1);
        assert_ne!(SOURCE, altered);
        assert!(validate(&compile(&altered).unwrap()).is_err(), "{before}");
    }
}
