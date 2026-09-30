//! The architectural verdicts, as tests rather than as claims.
//!
//! Every heading below is one of the deliverables. A claim about where semantics live is worth
//! nothing if the only evidence is a paragraph saying so, and each of these fails if the property it
//! names stops holding — including the ones about the source tree, which are checked by reading it.
//!
//! One boundary is drawn deliberately and is worth stating out loud. These tests scan `src/` and
//! `backends/` — the pipeline — and NOT `tests/`. A test must be able to name what it checks, so
//! `oracle.rs` says `resistor_divider.R1` and must. The property being defended is that no file
//! which RUNS as part of a solve knows what the model is; a test is not part of a solve.

mod common;

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use hymeko::ir::ir::DeclKind;
use hymeko_optimize::backend::{AugmentedLagrangian, Backend, multistart, starts};
use hymeko_optimize::problem::{Kind, Problem};
use hymeko_optimize::read::project;

fn crate_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

macro_rules! model {
    () => {
        match common::compile("resistor_divider.hymeko") {
            Some(pair) => pair,
            None => {
                eprintln!("skipped: core/data/optimization is not in this checkout");
                return;
            }
        }
    };
}

// ─── HymekoSemanticOwner ────────────────────────────────────────────────────────────────────────

/// Editing the `.hymeko` — and only the `.hymeko` — changes the design.
///
/// The variant below differs from the shipped model by ONE number: the open-circuit limit moves from
/// `2.0 V` to `2.5 V`. No Rust changes, no Python changes, no configuration. The new optimum was
/// derived on paper from that one edit: the limit gives `R1 >= 1.4 R2`, the operating contract keeps
/// `R1 = 3 Rp` with `Rp = 3R2/(R2+3)`, and the two meet at `R2 = 24/7`, `R1 = 4.8`, `P_loss = 4.875 W`.
///
/// If the design lived anywhere but the model, this test would return the old answer.
#[test]
fn verdict_hymeko_semantic_owner() {
    let Some(variant) = variant_of("resistor_divider.hymeko", &[("value 2.0;", "value 2.5;")])
    else {
        eprintln!("skipped: core/data/optimization is not in this checkout");
        return;
    };
    let problem = project(&variant.0, &variant.1).expect("the variant projects");
    problem.check().expect("the variant is well posed");

    let solved = AugmentedLagrangian::default()
        .solve(&problem, &problem.start())
        .expect("the variant converges");

    assert!(solved.feasible());
    assert!(
        (solved.decisions[0].value - 4.8).abs() < 1e-6,
        "R1 moved to {}",
        solved.decisions[0].value
    );
    assert!(
        (solved.decisions[1].value - 24.0 / 7.0).abs() < 1e-6,
        "R2 moved to {}",
        solved.decisions[1].value
    );
    assert!(
        (solved.objective - 4.875).abs() < 1e-6,
        "{}",
        solved.objective
    );
    assert!(
        (solved.value_of("V_A_disconnected").expect("a readout") - 2.5).abs() < 1e-6,
        "the new limit is the one that binds"
    );
    // The operating contract is untouched by the edit and still holds.
    assert!((solved.value_of("V_A_connected").expect("a readout") - 1.5).abs() < 1e-6);
}

/// A model the contract cannot satisfy is REPORTED, not quietly approximated.
///
/// Removing the bulb can only raise node A, so `V_A_open >= V_A_connected` always. Asking for
/// `2.5 V` connected under a `2.0 V` open-circuit limit is therefore impossible at any `R1, R2`, and
/// the honest answer is a failure. A backend that returned a plausible design here would be
/// inventing one.
#[test]
fn infeasible_contract_is_reported() {
    let Some(variant) = variant_of(
        "resistor_divider.hymeko",
        &[(
            "kind.parameter { unit u.volt; value 1.5; }",
            "kind.parameter { unit u.volt; value 2.5; }",
        )],
    ) else {
        eprintln!("skipped: core/data/optimization is not in this checkout");
        return;
    };
    let problem = project(&variant.0, &variant.1).expect("the variant projects");
    problem.check().expect("the variant is well posed");

    // A short budget: the claim is that no start REACHES a feasible design, and a search that has
    // not found one in sixty outer rounds on a two-variable problem is not about to.
    let backend = AugmentedLagrangian {
        outer_iterations: 60,
        ..AugmentedLagrangian::default()
    };
    for start in starts(&problem, 5) {
        match backend.solve(&problem, &start) {
            Err(_) => {}
            Ok(solved) => panic!(
                "an impossible contract was reported satisfied from {start:?}: {:?}",
                solved.decisions
            ),
        }
    }
}

// ─── DuplicateEditableSemantics ─────────────────────────────────────────────────────────────────

/// No pipeline file names anything in this particular circuit.
///
/// The distinction being drawn: `network.rs` knows what a RESISTOR LAW is, because the meta-model
/// declares `opt.laws.resistor` and somebody has to turn it into a nodal stamp — that is numerical
/// execution and it belongs here. What it must not know is which nets, which values, which scenario:
/// `R1`, `node_a`, `open_circuit_limit`. Those are this design, and this design has one home.
#[test]
fn verdict_no_duplicate_editable_semantics() {
    // Identifiers that exist only in `resistor_divider.hymeko`.
    const INSTANCE: &[&str] = &[
        "R1",
        "R2",
        "Vs",
        "Vb",
        "Ib",
        "Rb",
        "node_a",
        "supply",
        "branch_r1",
        "branch_r2",
        "branch_bulb",
        "target_node_a",
        "open_circuit_limit",
        "V_A_connected",
        "V_A_disconnected",
        "P_resistor_loss_connected",
        "efficiency_connected",
        "resistor_divider",
    ];

    let mut offences = Vec::new();
    for file in sources(&crate_root().join("src"), "rs")
        .into_iter()
        .chain(sources(&crate_root().join("backends"), "py"))
    {
        let text = strip_comments(&std::fs::read_to_string(&file).expect("readable"), &file);
        for needle in INSTANCE {
            if contains_word(&text, needle) {
                offences.push(format!("{} names {needle}", file.display()));
            }
        }
    }
    assert!(offences.is_empty(), "{offences:#?}");
}

/// The design is declared in exactly one file, and no second editable copy exists beside it.
///
/// Scanned across `core/data`, which is where a duplicate would most naturally end up: a JSON
/// "config", a YAML "scenario set", a second `.hymeko` with the same contract in it.
#[test]
fn verdict_one_editable_declaration() {
    let Some(root) = data_root() else {
        eprintln!("skipped: core/data is not in this checkout");
        return;
    };
    let mut declaring = Vec::new();
    let mut stack = vec![root];
    while let Some(directory) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&directory) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if let Ok(text) = std::fs::read_to_string(&path) {
                if contains_word(&text, "open_circuit_limit") {
                    declaring.push(path);
                }
            }
        }
    }
    assert_eq!(
        declaring.len(),
        1,
        "exactly one file may declare this design: {declaring:#?}"
    );
    assert!(declaring[0].ends_with("resistor_divider.hymeko"));

    // And this crate carries no copy of the model or the meta-model.
    for file in sources(&crate_root(), "hymeko") {
        panic!(
            "a copy of a model lives in the consumer: {}",
            file.display()
        );
    }
}

// ─── ScenarioIdentityParity ─────────────────────────────────────────────────────────────────────

/// The two scenarios are ONE model, sharing identities rather than values.
///
/// Two copies of a topology with the same numbers in them would pass a value comparison. This test
/// compares the DECLARATIONS behind the shared branches, which two copies could not share, and
/// checks that the difference is one participation edge and nothing else.
#[test]
fn verdict_scenario_identity_parity() {
    let (ir, it) = model!();
    let problem = project(&ir, &it).expect("the model projects");

    let connected = problem
        .scenarios
        .iter()
        .find(|s| s.name.ends_with(".connected"))
        .expect("a connected scenario");
    let disconnected = problem
        .scenarios
        .iter()
        .find(|s| s.name.ends_with(".disconnected"))
        .expect("a disconnected scenario");

    // The shared branches are the same branches, not equal ones: same index into one branch list,
    // which came from one declaration each.
    let shared: HashSet<usize> = disconnected.branches.iter().copied().collect();
    assert!(shared.iter().all(|b| connected.branches.contains(b)));
    assert_eq!(shared.len(), connected.branches.len() - 1);

    // Everything else is unconditionally shared: there is one quantity list, one constraint list,
    // one objective, one net list. A per-scenario copy of any of them would show up as a duplicate
    // declaration id.
    let ids: HashSet<_> = problem.quantities.iter().map(|q| q.decl).collect();
    assert_eq!(
        ids.len(),
        problem.quantities.len(),
        "no quantity is declared twice"
    );
    let ids: HashSet<_> = problem.nets.iter().collect();
    assert_eq!(ids.len(), problem.nets.len(), "no net is declared twice");

    // And the model says the difference once, in one hyperedge.
    let participations = ir
        .decl_nodes
        .iter()
        .enumerate()
        .filter(|(raw, node)| {
            node.kind == DeclKind::Edge && inherits_named(&ir, &it, *raw, "participation")
        })
        .count();
    assert_eq!(
        participations, 1,
        "the whole scenario difference is one participation edge"
    );
}

// ─── TypedProjectionParity ──────────────────────────────────────────────────────────────────────

/// The projection neither invents nor drops anything, and keeps declared identity and units.
#[test]
fn verdict_typed_projection_parity() {
    let (ir, it) = model!();
    let problem = project(&ir, &it).expect("the model projects");

    // Every quantity in the problem is a live declaration in the IR, once.
    let ids: HashSet<_> = problem.quantities.iter().map(|q| q.decl).collect();
    assert_eq!(ids.len(), problem.quantities.len());
    for quantity in &problem.quantities {
        assert!(quantity.decl.raw() < ir.decl_nodes.len());
        assert_eq!(ir.decl_nodes[quantity.decl.raw()].kind, DeclKind::Node);
    }

    // And every declaration the IR types as a quantity is in the problem: the projection reads the
    // whole model, not the part it happens to recognise.
    let declared = (0..ir.decl_nodes.len())
        .filter(|&raw| {
            ir.decl_nodes[raw].kind == DeclKind::Node
                && ["decision", "parameter", "derived"]
                    .iter()
                    .any(|k| inherits_named(&ir, &it, raw, k))
        })
        .count();
    assert_eq!(declared, problem.quantities.len(), "nothing was skipped");

    // The kinds came from the type, and the unit check passes on the declared units.
    assert_eq!(
        problem
            .quantities
            .iter()
            .filter(|q| q.kind == Kind::Decision)
            .count(),
        problem.decisions.len()
    );
    problem.check().expect("declared units are consistent");

    // A unit that is wrong in the MODEL is caught HERE, which is the point of a typed projection:
    // the language cannot check it, and the backend must never see it.
    let Some(variant) = variant_of(
        "resistor_divider.hymeko",
        &[(
            "V_A_connected:    kind.derived { unit u.volt; }",
            "V_A_connected:    kind.derived { unit u.ohm; }",
        )],
    ) else {
        return;
    };
    let broken = project(&variant.0, &variant.1).expect("a wrong unit still projects");
    assert!(
        broken.check().is_err(),
        "a volt readout declared in ohms must be refused"
    );
}

// ─── OptimizationOracleParity ───────────────────────────────────────────────────────────────────

/// The solved design matches the hand-derived optimum on every published quantity.
///
/// The derivation is in `oracle.rs`; this is the verdict that names it as a deliverable.
#[test]
fn verdict_optimization_oracle_parity() {
    let (ir, it) = model!();
    let problem = project(&ir, &it).expect("the model projects");
    let solved = AugmentedLagrangian::default()
        .solve(&problem, &problem.start())
        .expect("it converges");

    for (name, expected) in [
        ("resistor_divider.R1", 3.0),
        ("resistor_divider.R2", 1.5),
        ("resistor_divider.V_A_connected", 1.5),
        ("resistor_divider.V_A_disconnected", 2.0),
        ("resistor_divider.P_resistor_loss_connected", 8.25),
        ("resistor_divider.P_bulb_connected", 0.75),
        ("resistor_divider.P_source_connected", 9.0),
        ("resistor_divider.efficiency_connected", 1.0 / 12.0),
    ] {
        let got = solved.value_of(name).expect("in the result");
        assert!(
            (got - expected).abs() < 1e-6,
            "{name}: expected {expected}, got {got}"
        );
    }
}

// ─── DeterministicMultistart ────────────────────────────────────────────────────────────────────

/// The same model gives the same starting points and, from each of them, the same answer.
///
/// Two properties, and they are not the same one. REPRODUCIBILITY is that a start replayed returns
/// bit-for-bit what it returned before — there is no seed to record because there is no randomness,
/// so this must hold exactly. AGREEMENT is that different starts reach the same optimum, and that
/// can only hold to the solver's tolerance: an iterative method stops when its residual is small,
/// and two paths stopping at the same point in exact arithmetic stop a few ulp apart in floating
/// point. Asserting bit equality across DIFFERENT starts would be asserting something no solver has.
#[test]
fn verdict_deterministic_multistart() {
    let (ir, it) = model!();
    let problem = project(&ir, &it).expect("the model projects");
    assert_eq!(starts(&problem, 9), starts(&problem, 9));

    let backend = AugmentedLagrangian::default();
    let runs = multistart(&backend, &problem, 9);
    let second = multistart(&backend, &problem, 9);
    assert_eq!(runs.len(), 9);

    // Reproducibility: exact, per start.
    for (a, b) in runs.iter().zip(&second) {
        match (a, b) {
            (Ok(a), Ok(b)) => {
                assert_eq!(a.start, b.start);
                for (p, q) in a.decisions.iter().zip(&b.decisions) {
                    assert_eq!(
                        p.value.to_bits(),
                        q.value.to_bits(),
                        "{} is not reproducible from {:?}",
                        p.name,
                        a.start
                    );
                }
            }
            (Err(a), Err(b)) => assert_eq!(a, b, "a failure is not reproducible"),
            _ => panic!("one start converged in one run and not the other"),
        }
    }

    // Agreement: to tolerance, across starts spanning ten decades of the declared box.
    let converged: Vec<_> = runs.iter().filter_map(|r| r.as_ref().ok()).collect();
    assert!(
        converged.len() >= 5,
        "only {}/9 lattice points converged",
        converged.len()
    );
    let mut spread: f64 = 0.0;
    for solved in &converged {
        for (k, expected) in [3.0_f64, 1.5].iter().enumerate() {
            let got = solved.decisions[k].value;
            spread = spread.max((got - expected).abs() / expected);
        }
    }
    println!(
        "multistart: {}/9 converged, worst relative spread {spread:e}",
        converged.len()
    );
    assert!(
        spread < 1e-7,
        "starts disagreed by {spread:e}: {:#?}",
        converged
            .iter()
            .map(|s| (
                s.start.clone(),
                s.decisions.iter().map(|d| d.value).collect::<Vec<_>>()
            ))
            .collect::<Vec<_>>()
    );
}

// ─── CleanCheckoutQualification ─────────────────────────────────────────────────────────────────

/// This crate consumes its neighbours through their public APIs, and vendors nothing.
#[test]
fn verdict_clean_checkout_qualification() {
    let manifest = std::fs::read_to_string(crate_root().join("Cargo.toml")).expect("readable");
    assert!(
        manifest.contains("hymeko_core = { workspace = true }"),
        "the core must come from the workspace, not from a local copy"
    );
    assert!(
        !manifest.contains("path = \"../../core"),
        "a path override to core would pin this to one layout: {manifest}"
    );

    // No vendored source from another owner. Scoped to what RUNS in a solve; this file names the
    // marker it looks for and would otherwise match itself.
    for directory in ["src", "backends"] {
        for file in sources(&crate_root().join(directory), "rs")
            .into_iter()
            .chain(sources(&crate_root().join(directory), "py"))
        {
            let text = std::fs::read_to_string(&file).expect("readable");
            assert!(
                !text.contains("// vendored") && !text.contains("# vendored"),
                "{} is vendored",
                file.display()
            );
        }
    }

    // The model lives in its owner's repository and is reached by a relative path from this crate,
    // so a checkout without it skips rather than fails.
    assert!(
        common::model_path("resistor_divider.hymeko")
            .to_string_lossy()
            .contains("core")
    );
}

// ─── CoreIRChangeRequired ───────────────────────────────────────────────────────────────────────

/// Verdict: NO. The whole slice runs on an unmodified core.
///
/// The evidence is that `hymeko_core` is consumed from one published Git revision, including any
/// legacy-URL redirects, and that the two models compile and project on it. If the
/// slice had needed a core change, this crate could not have been built without one.
#[test]
fn verdict_core_ir_change_not_required() {
    let workspace = std::fs::read_to_string(crate_root().join("../Cargo.toml")).expect("readable");
    assert!(
        workspace.contains("hymeko_core = { git ="),
        "core is a git dependency, taken as published"
    );
    // A Git source redirect may unify a legacy URL with the same published Core.
    // It must not introduce a local path or a second source/revision.
    let mut declarations = workspace
        .lines()
        .map(str::trim)
        .filter(|line| line.starts_with("hymeko_core ="));
    let canonical = declarations.next().expect("workspace Core declaration");
    assert!(!canonical.contains("path ="));
    assert!(canonical.contains("rev ="), "Core must use an exact revision");
    for declaration in declarations {
        assert_eq!(declaration, canonical, "Core redirects must keep one identity");
    }

    let (ir, it) = model!();
    let problem = project(&ir, &it).expect("an unmodified core compiles and projects the model");
    assert!(!problem.quantities.is_empty());
    assert!(!problem.branches.is_empty());
    assert!(!problem.scenarios.is_empty());
}

// ─── helpers ────────────────────────────────────────────────────────────────────────────────────

/// Compile a temporary variant of a model with textual substitutions applied.
///
/// The copy is a test fixture and is deleted with the temporary directory. It is not a second
/// authority: nothing reads it but the assertion that made it, and the shipped model is untouched.
fn variant_of(
    file: &str,
    edits: &[(&str, &str)],
) -> Option<(hymeko::ir::ir::Ir, hymeko::resolution::interner::Interner)> {
    let source = common::model_path(file);
    if !source.exists() {
        return None;
    }
    let mut text = std::fs::read_to_string(&source).expect("readable");
    for (from, to) in edits {
        assert!(text.contains(from), "the model no longer contains {from:?}");
        text = text.replace(from, to);
    }

    // Named by a hash of the CONTENT. Length is not enough: every substitution here is
    // length-preserving on purpose, so two different variants would otherwise land in one directory
    // and race each other copying the meta-model into it.
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for byte in text.as_bytes() {
        hash = (hash ^ *byte as u64).wrapping_mul(0x0000_0100_0000_01b3);
    }
    let directory = std::env::temp_dir().join(format!(
        "hymeko-optimize-variant-{}-{hash:016x}",
        std::process::id()
    ));
    std::fs::create_dir_all(&directory).expect("a temporary directory");
    // The meta-model comes along because the variant imports it by relative path.
    std::fs::copy(
        common::model_path("meta_optimization.hymeko"),
        directory.join("meta_optimization.hymeko"),
    )
    .expect("the meta-model copies");
    let path = directory.join(file);
    std::fs::write(&path, text).expect("the variant writes");

    let mut store = hymeko::module_store::module_store::ModuleStore::new(
        hymeko::module_store::source_provider::StdFsProvider::new(),
        hymeko::util::real_parser::RealParser,
    );
    let program = store.compile(&path).expect("the variant compiles");
    Some((program.ir.clone(), store.it))
}

fn data_root() -> Option<PathBuf> {
    let root = crate_root().join("../../core/data");
    root.exists().then_some(root)
}

/// Every file with this extension under a directory.
fn sources(root: &Path, extension: &str) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(directory) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&directory) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if path.file_name().is_some_and(|n| n == "target") {
                    continue;
                }
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == extension) {
                out.push(path);
            }
        }
    }
    out
}

/// Remove comments and docstrings, so a prose explanation is not read as code.
///
/// A file is allowed to EXPLAIN what it is not allowed to DO — the module documentation in
/// `backend.rs` says the word "resistor" while arguing that no resistor may appear in it. Scanning
/// the raw text would make that argument impossible to write down.
fn strip_comments(text: &str, path: &Path) -> String {
    let python = path.extension().is_some_and(|e| e == "py");
    let mut out = String::with_capacity(text.len());
    let bytes: Vec<char> = text.chars().collect();
    let mut i = 0;
    while i < bytes.len() {
        if python {
            if bytes[i] == '#' {
                while i < bytes.len() && bytes[i] != '\n' {
                    i += 1;
                }
                continue;
            }
            if i + 2 < bytes.len() && bytes[i..i + 3].iter().collect::<String>() == "\"\"\"" {
                i += 3;
                while i + 2 < bytes.len() && bytes[i..i + 3].iter().collect::<String>() != "\"\"\""
                {
                    i += 1;
                }
                i = (i + 3).min(bytes.len());
                continue;
            }
        } else {
            if i + 1 < bytes.len() && bytes[i] == '/' && bytes[i + 1] == '/' {
                while i < bytes.len() && bytes[i] != '\n' {
                    i += 1;
                }
                continue;
            }
            if i + 1 < bytes.len() && bytes[i] == '/' && bytes[i + 1] == '*' {
                i += 2;
                while i + 1 < bytes.len() && !(bytes[i] == '*' && bytes[i + 1] == '/') {
                    i += 1;
                }
                i = (i + 2).min(bytes.len());
                continue;
            }
        }
        out.push(bytes[i]);
        i += 1;
    }
    out
}

/// Is this identifier present as a whole word?
fn contains_word(text: &str, needle: &str) -> bool {
    let boundary = |c: Option<char>| !c.is_some_and(|c| c.is_alphanumeric() || c == '_');
    let mut from = 0;
    while let Some(at) = text[from..].find(needle) {
        let start = from + at;
        let end = start + needle.len();
        if boundary(text[..start].chars().next_back()) && boundary(text[end..].chars().next()) {
            return true;
        }
        from = start + 1;
    }
    false
}

/// Does a declaration's base chain reach one of this name?
///
/// A small duplicate of the reader's own walk, on purpose: a test that used the reader's answer to
/// check the reader would agree with itself no matter what either of them did.
fn inherits_named(
    ir: &hymeko::ir::ir::Ir,
    it: &hymeko::resolution::interner::Interner,
    raw: usize,
    name: &str,
) -> bool {
    use hymeko::common::ids::DeclId;
    use hymeko::ir::ir::SignedRefR;

    let bases_of = |decl: DeclId| -> &[SignedRefR] {
        if let Some(node) = ir.as_node(decl) {
            return &ir.nodes[node.raw()].bases;
        }
        if let Some(edge) = ir.as_edge(decl) {
            return &ir.edges[edge.raw()].bases;
        }
        &[]
    };
    let target = |r: &SignedRefR| match r {
        SignedRefR::Plus(a) | SignedRefR::Minus(a) | SignedRefR::Neutral(a) => a.target,
    };

    let mut seen = HashSet::new();
    let mut stack = vec![DeclId::new(raw)];
    while let Some(here) = stack.pop() {
        if !seen.insert(here) {
            continue;
        }
        for base in bases_of(here) {
            let to = target(base);
            if it.resolve(ir.decl_nodes[to.raw()].name) == name {
                return true;
            }
            stack.push(to);
        }
    }
    false
}

/// Keeps `Problem` referenced from this file's imports, which the verdicts use through `project`.
const _: Option<fn(&Problem) -> usize> = Some(Problem::dimension);
