//! Fixed, independently checked expectations; not an executable IR.

pub const DE_KEY: &str = "de-rand-1-bin-clamp-deferred-v1";
mod hho;
pub const HHO_KEY: &str = hho::KEY;

pub const GA_KEY: &str = "ga-tournament2-uniform-reset-elitist-v1";

pub(super) struct Profile {
    pub(super) key: &'static str,
    pub(super) fields: &'static [(&'static str, &'static str)],
    pub(super) operators: &'static [OperatorRule],
}

pub(super) fn get(key: &str) -> Option<Profile> {
    let (key, fields, operators) = match key {
        DE_KEY => (DE_KEY, DE_FIELDS, DE_OPERATORS),
        GA_KEY => (GA_KEY, GA_FIELDS, GA_OPERATORS),
        HHO_KEY => (HHO_KEY, hho::FIELDS, hho::OPERATORS),
        _ => return None,
    };
    Some(Profile {
        key,
        fields,
        operators,
    })
}

const GA_FIELDS: &[(&str, &str)] = &[
    ("key", GA_KEY),
    ("optimization", "minimize"),
    ("arithmetic", "python_binary64"),
    ("dimensions", "1<=D<=64,4<=N<=8,N<=B<=80"),
    (
        "parameters",
        "finite_bounds_lo<hi;reset_probability=1/D;shared_Config_validation",
    ),
    ("update", "frozen_generation;stable_mu_plus_lambda"),
    ("boundary", "inherit_bounded_parent_or_uniform_reset"),
    (
        "crossover",
        "independent_coordinate_parent;u>=0.5_selects_parent1",
    ),
    (
        "acceptance",
        "stable_lowest_N;parents_before_children;preserve_input_order_on_ties",
    ),
    (
        "parents",
        "two_independent_tournaments;two_distinct_indices_each;min_score_then_index",
    ),
    (
        "randomness",
        "ordered_uniform_pairs_without_replacement;D_origin_uniforms;D_reset_gates;conditional_uniform_reset_values",
    ),
    ("budget", "initialization_included;stop_before_B_plus_one"),
    ("early_stop", "none"),
    (
        "partial_generation",
        "select_N_from_parents_plus_completed_children",
    ),
    ("target_order", "child_creation_order"),
    (
        "archive",
        "host_ledger_min_all_evaluated;ties_keep_previous_archive_vector",
    ),
    ("result_tie", "first_survivor_index_at_minimum"),
    (
        "event_coverage",
        "objective_evaluations_and_reported_choice_results;raw_uniforms_unobserved",
    ),
];

// Tuple-like construction avoids duplicating the validation machinery per method.
const fn op(
    key: &'static str,
    rule: &'static str,
    reads: &'static [&'static str],
    writes: &'static [&'static str],
) -> OperatorRule {
    OperatorRule {
        key,
        rule,
        reads,
        writes,
    }
}

const GA_OPERATORS: &[OperatorRule] = &[
    op(
        "ga.initialize",
        "copy_bounded_initial_population;evaluate_once_each_in_order;generation_and_child_index_zero",
        &["initial_population", "bounds", "objective", "budget_limit"],
        &[
            "population_old",
            "parent_scores",
            "evaluation_count",
            "generation",
            "child_index",
            "evaluated_candidate",
            "evaluated_score",
        ],
    ),
    op(
        "ga.prepare_generation",
        "freeze_parents;empty_children;lambda=min(N,B-count)",
        &[
            "population_old",
            "parent_scores",
            "evaluation_count",
            "budget_limit",
        ],
        &["children", "child_scores", "child_index", "offspring_count"],
    ),
    op(
        "ga.parent_selection",
        "two_distinct_index_tournaments;winner=min(score,index);parents_may_coincide",
        &[
            "population_old",
            "parent_scores",
            "decision_source",
            "generation",
            "child_index",
        ],
        &["tournament_pairs", "parent_indices"],
    ),
    op(
        "ga.crossover_reset",
        "inherit_coordinate_from_recorded_parent;reset_selected_coordinates_within_bounds;require_finite",
        &[
            "population_old",
            "parent_indices",
            "bounds",
            "decision_source",
        ],
        &["coordinate_parent", "reset_coordinates", "child"],
    ),
    op(
        "ga.evaluate_child",
        "require_count<B;count_objective_call;append_child_and_score;advance_child_index",
        &[
            "child",
            "objective",
            "evaluation_count",
            "budget_limit",
            "children",
            "child_scores",
            "child_index",
        ],
        &[
            "children",
            "child_scores",
            "child_index",
            "evaluation_count",
            "evaluated_candidate",
            "evaluated_score",
        ],
    ),
    op(
        "ga.select",
        "after_lambda_children;stable_sort_parents_then_children_by_score;keep_first_N;publish;increment_generation_iff_lambda=N;reset_child_index_iff_complete",
        &[
            "population_old",
            "parent_scores",
            "children",
            "child_scores",
            "child_index",
            "offspring_count",
            "generation",
        ],
        &[
            "population_old",
            "parent_scores",
            "generation",
            "child_index",
        ],
    ),
    op(
        "ga.archive",
        "host_after_each_evaluation_keep_min;equal_value_keeps_previous_vector",
        &["evaluated_candidate", "evaluated_score", "archive_best"],
        &["archive_best"],
    ),
    op(
        "ga.stop",
        "continue_iff_count<B;return_first_survivor_index_at_minimum;report_complete_and_partial_generations",
        &[
            "evaluation_count",
            "budget_limit",
            "population_old",
            "parent_scores",
            "generation",
            "child_index",
        ],
        &["run_status"],
    ),
];

/// Fixed-profile expectations are a conformance oracle, not another executable IR.
const DE_FIELDS: &[(&str, &str)] = &[
    ("key", DE_KEY),
    ("optimization", "minimize"),
    ("arithmetic", "python_binary64"),
    ("dimensions", "D>=1,N>=4,B>=N"),
    ("parameters", "0<F<=2,0<=CR<=1,finite_bounds_lo<hi"),
    ("update", "deferred_generation"),
    ("boundary", "componentwise_clamp_after_crossover"),
    ("crossover", "uniform_lt_CR_or_forced_coordinate"),
    ("acceptance", "trial_le_parent"),
    ("donors", "three_distinct_excluding_target"),
    (
        "randomness",
        "named_decisions;3_ordered_donors;forced_coordinate;D_uniforms_[0,1)",
    ),
    ("budget", "initialization_included;stop_before_B_plus_one"),
    ("early_stop", "none"),
    ("partial_generation", "retain_unprocessed_parents"),
    ("target_order", "ascending_index"),
    (
        "archive",
        "min_all_evaluated;ties_keep_previous_archive_vector",
    ),
    ("result_tie", "first_population_index_at_minimum"),
    ("event_coverage", "objective_evaluations_only"),
];

pub(super) struct OperatorRule {
    pub(super) key: &'static str,
    pub(super) rule: &'static str,
    pub(super) reads: &'static [&'static str],
    pub(super) writes: &'static [&'static str],
}

const DE_OPERATORS: &[OperatorRule] = &[
    OperatorRule {
        key: "de.initialize",
        rule: "copy_provided_population;evaluate_each_initial_candidate_once",
        reads: &["initial_population", "objective", "budget_limit"],
        writes: &[
            "population_old",
            "parent_scores",
            "evaluation_count",
            "generation",
            "evaluated_candidate",
            "evaluated_score",
        ],
    },
    OperatorRule {
        key: "de.prepare_generation",
        rule: "copy_old_population_and_scores_to_next;set_target_index_zero",
        reads: &["population_old", "parent_scores"],
        writes: &["population_next", "next_scores", "target_index"],
    },
    OperatorRule {
        key: "de.donor_selection",
        rule: "draw_named_decision_and_resolve_ordered_donors_from_old_population",
        reads: &[
            "population_old",
            "generation",
            "target_index",
            "decision_source",
        ],
        writes: &[
            "donor_base",
            "donor_positive",
            "donor_negative",
            "target_old",
            "forced_coordinate",
            "crossover_uniforms",
        ],
    },
    OperatorRule {
        key: "de.mutate",
        rule: "mutant=donor_base+F*(donor_positive-donor_negative);require_finite",
        reads: &[
            "donor_base",
            "donor_positive",
            "donor_negative",
            "differential_weight",
        ],
        writes: &["mutant"],
    },
    OperatorRule {
        key: "de.crossover_repair",
        rule: "trial[j]=clamp(mutant[j])_if_j_forced_or_u[j]<CR_else_target_old[j]",
        reads: &[
            "mutant",
            "target_old",
            "forced_coordinate",
            "crossover_uniforms",
            "crossover_probability",
            "bounds",
        ],
        writes: &["trial"],
    },
    OperatorRule {
        key: "de.evaluate_trial",
        rule: "require_count<B;count_objective_call;require_finite_result",
        reads: &["trial", "objective", "budget_limit", "evaluation_count"],
        writes: &[
            "trial_score",
            "evaluation_count",
            "evaluated_candidate",
            "evaluated_score",
        ],
    },
    OperatorRule {
        key: "de.select",
        rule: "replace_next_target_and_score_iff_trial_score<=parent_score;old_population_unchanged;advance_target",
        reads: &[
            "trial",
            "trial_score",
            "target_index",
            "parent_scores",
            "population_next",
            "next_scores",
        ],
        writes: &["population_next", "next_scores", "target_index"],
    },
    OperatorRule {
        key: "de.archive",
        rule: "after_each_evaluation_keep_min;equal_value_keeps_previous_vector",
        reads: &["evaluated_candidate", "evaluated_score", "archive_best"],
        writes: &["archive_best"],
    },
    OperatorRule {
        key: "de.publish",
        rule: "after_target_loop_publish_next_snapshot;retain_unprocessed_parents;increment_generation_only_if_complete",
        reads: &[
            "population_next",
            "next_scores",
            "target_index",
            "population_old",
            "parent_scores",
            "generation",
        ],
        writes: &["population_old", "parent_scores", "generation"],
    },
    OperatorRule {
        key: "de.stop",
        rule: "continue_iff_count<B;no_target_early_stop;return_first_population_index_at_minimum",
        reads: &[
            "evaluation_count",
            "budget_limit",
            "population_old",
            "parent_scores",
        ],
        writes: &["run_status"],
    },
];
