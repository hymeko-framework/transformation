use hymeko::module_store::{module_store::ModuleStore, source_provider::MemProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_dynamics::scenario::*;
use std::path::Path;

const CELL: &str = include_str!("fixtures/reference-cell.hymeko");
fn open(source: &str, options: OpenOptions) -> Result<ScenarioSession, Refusal> {
    let mut provider = MemProvider::default();
    provider.insert_file("cell.hymeko", source);
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store.compile(Path::new("cell.hymeko")).unwrap();
    let root = program
        .ir
        .decl_nodes
        .iter()
        .position(|n| store.it.resolve(n.name) == "cell")
        .unwrap();
    ScenarioSession::open(&program.ir, &store.it, root, options)
}
fn cell() -> ScenarioSession {
    open(CELL, OpenOptions::default()).unwrap()
}
fn observer(snapshot: &Snapshot, suffix: &str) -> i64 {
    snapshot
        .observers
        .iter()
        .find(|o| o.identity.name.ends_with(suffix))
        .unwrap()
        .value
}

#[test]
fn reference_cell_fault_causality_and_replay() {
    let mut session = cell();
    assert_eq!(session.model().elements.len(), 4);
    assert_eq!(session.step(3).unwrap().applied, 3);
    assert_eq!(
        observer(&session.snapshot(Some(0)).unwrap(), ".completed"),
        1
    );
    session.step(2).unwrap();
    session
        .intervene("cell.interventions.faultMachine2", None)
        .unwrap();
    let snapshot = session.snapshot(Some(0)).unwrap();
    assert_eq!(observer(&snapshot, ".faults"), 1);
    assert_eq!(observer(&snapshot, ".available"), 1);
    assert!(
        !snapshot
            .transitions
            .iter()
            .find(|t| t.identity.name.ends_with("finish"))
            .unwrap()
            .enabled
    );
    assert!(
        !snapshot
            .invariants
            .iter()
            .find(|i| i.identity.name.ends_with("secondMachineAvailable"))
            .unwrap()
            .valid
    );
    let fault = session.events().last().unwrap();
    assert_eq!(fault.cause.label, "physical fault");
    assert!(fault.guards.iter().all(|g| g.satisfied));
    assert_ne!(fault.before.values, fault.after.values);
    let log = session.export_log().unwrap();
    let mut replay = cell();
    replay.import_log(&log).unwrap();
    assert_eq!(replay.export_log().unwrap(), log);
    assert_eq!(replay.snapshot(Some(0)).unwrap(), snapshot);
    for revision in 0..=session.revision() {
        assert_eq!(
            replay.state_at(revision).unwrap(),
            session.state_at(revision).unwrap()
        );
    }
    let mut duplicate = cell();
    duplicate.step(3).unwrap();
    duplicate.step(2).unwrap();
    duplicate
        .intervene("cell.interventions.faultMachine2", None)
        .unwrap();
    assert_eq!(duplicate.export_log().unwrap(), log);
}

#[test]
fn typed_refusals_are_atomic_and_repair_can_recover_a_declared_halt() {
    let mut session = cell();
    let original = session.export_log().unwrap();
    assert_eq!(
        session.fire("cell.transitions.finish").unwrap_err().code,
        RefusalCode::Disabled
    );
    assert_eq!(
        session.fire("unknown").unwrap_err().code,
        RefusalCode::InvalidRequest
    );
    assert!(session.intervene("unknown", None).is_err());
    assert!(
        session
            .intervene("cell.interventions.changeSupply", Some(2))
            .is_err()
    );
    assert!(
        session
            .intervene("cell.interventions.changeSupply", None)
            .is_err()
    );
    assert!(
        session
            .intervene("cell.interventions.faultMachine2", Some(1))
            .is_err()
    );
    assert!(session.step(0).is_err());
    assert!(session.reset(1).is_err());
    assert!(session.snapshot(Some(1)).is_err());
    assert_eq!(session.export_log().unwrap(), original);
    session
        .intervene("cell.interventions.changeSupply", Some(0))
        .unwrap();
    assert!(!session.snapshot(Some(0)).unwrap().transitions[0].enabled);
    let mut halted = open(
        &CELL.replace(
            "maximum 1; violation \"record\"",
            "maximum 1; violation \"halt\"",
        ),
        OpenOptions::default(),
    )
    .unwrap();
    halted
        .intervene("cell.interventions.faultMachine2", None)
        .unwrap();
    assert!(halted.state().halted);
    assert_eq!(
        halted.step(1).unwrap().stopped.unwrap().code,
        RefusalCode::Halted
    );
    halted
        .intervene("cell.interventions.repairMachine2", None)
        .unwrap();
    assert!(!halted.state().halted);
    assert_eq!(halted.step(1).unwrap().applied, 1);
}

#[test]
fn seed_policy_reset_and_paging_are_deterministic() {
    for seed in [0, 1, 42, u32::MAX] {
        for policy in [ChoicePolicy::SourceOrder, ChoicePolicy::RoundRobin] {
            let options = OpenOptions {
                seed,
                policy: Some(policy),
            };
            let mut a = open(CELL, options.clone()).unwrap();
            let mut b = open(CELL, options.clone()).unwrap();
            a.step(260).unwrap();
            b.step(260).unwrap();
            assert_eq!(a.export_log().unwrap(), b.export_log().unwrap());
            assert_eq!(a.revision(), 256);
            let page = a.snapshot(None).unwrap();
            assert!(page.capped);
            assert_eq!(page.events.len(), 128);
            assert_eq!(page.through_revision, 128);
            // An explicit historical read cannot consume the default cursor.
            assert_eq!(a.snapshot(Some(0)).unwrap().through_revision, 128);
            assert_eq!(a.snapshot(None).unwrap().since_revision, 128);
            assert!(a.snapshot(None).unwrap().events.is_empty());
            let state = a.state_at(3).unwrap().clone();
            a.reset(3).unwrap();
            assert_eq!(*a.state(), state);
            assert_eq!(a.revision(), 257);
            assert_eq!(a.events().last().unwrap().cause.label, "operator action");
            a.reset(0).unwrap();
            assert_eq!(*a.state(), *a.state_at(0).unwrap());
            let mut replay = open(CELL, options).unwrap();
            replay.import_log(&a.export_log().unwrap()).unwrap();
            assert_eq!(replay.export_log().unwrap(), a.export_log().unwrap());
        }
    }
}

#[test]
fn tampered_replay_schema_and_initial_state_are_rejected_without_adoption() {
    let mut source = cell();
    source.step(3).unwrap();
    let exported = source.export_log().unwrap();
    let baseline: serde_json::Value = serde_json::from_str(&exported).unwrap();
    for pointer in [
        "/initial/values/0",
        "/events/0/before/values/0",
        "/events/1/after/observers/0",
        "/events/2/revision",
        "/events/0/guards/0/actual",
    ] {
        let mut bad = baseline.clone();
        *bad.pointer_mut(pointer).unwrap() = serde_json::json!(123456);
        let mut target = cell();
        let before = target.export_log().unwrap();
        assert!(target.import_log(&bad.to_string()).is_err(), "{pointer}");
        assert_eq!(target.export_log().unwrap(), before);
    }
    let mut target = cell();
    assert_eq!(
        target
            .import_log(&exported.replace(SCHEMA, "unsupported"))
            .unwrap_err()
            .code,
        RefusalCode::SchemaMismatch
    );
    target.import_log(&exported).unwrap();
    assert!(target.import_log(&exported).is_err());
}

#[test]
fn canonical_cbor_json_schema_and_no_presentation_fields() {
    let mut session = cell();
    session.step(4).unwrap();
    let snapshot = session.snapshot(Some(0)).unwrap();
    let json = snapshot.to_json().unwrap();
    let cbor = snapshot.to_cbor().unwrap();
    let decoded: Snapshot = ciborium::de::from_reader(cbor.as_slice()).unwrap();
    assert_eq!(decoded, snapshot);
    assert_eq!(decoded.to_cbor().unwrap(), cbor);
    assert_eq!(snapshot.schema_version, 1);
    let value: ciborium::value::Value = ciborium::de::from_reader(cbor.as_slice()).unwrap();
    fn check(value: &ciborium::value::Value) {
        use ciborium::value::{CanonicalValue, Value};
        match value {
            Value::Map(entries) => {
                let keys: Vec<_> = entries
                    .iter()
                    .map(|(k, _)| CanonicalValue::from(k.clone()))
                    .collect();
                assert!(keys.windows(2).all(|w| w[0] < w[1]));
                for (k, v) in entries {
                    assert!(matches!(k, Value::Text(_)));
                    check(v);
                }
            }
            Value::Array(a) => {
                for v in a {
                    check(v);
                }
            }
            Value::Float(_) => panic!("schema must not contain floats"),
            _ => (),
        }
    }
    check(&value);
    for forbidden in [
        "geometry",
        "coordinates",
        "color",
        "layout",
        "camera",
        "position",
    ] {
        assert!(!json.contains(&format!("\"{forbidden}\":")));
    }
}

#[test]
fn invalid_source_contract_and_capacity_refusals() {
    for (old, new) in [
        ("hymeko-scenario/v1", "hymeko-scenario/v2"),
        ("untimed", "seconds"),
        ("roundRobin", "random"),
        ("initial 1; capacity 1;", "initial 2; capacity 1;"),
        ("op \"eq\"", "op \"unknown\""),
        ("minimum 0; maximum 2", "minimum 3; maximum 2"),
    ] {
        assert!(
            open(&CELL.replace(old, new), OpenOptions::default()).is_err(),
            "{new}"
        );
    }
    let mut session = open(
        &CELL.replace("op \"add\"; value 1", "op \"add\"; value 2"),
        OpenOptions::default(),
    )
    .unwrap();
    let before = session.export_log().unwrap();
    assert_eq!(
        session.fire("cell.transitions.feed").unwrap_err().code,
        RefusalCode::Capacity
    );
    assert_eq!(session.export_log().unwrap(), before);
}

#[test]
fn log_and_step_caps_are_explicit_and_do_not_discard_history() {
    let mut session = cell();
    assert!(session.step(usize::MAX).unwrap().capped);
    for _ in 1..(MAX_EVENTS / MAX_STEPS) {
        session.step(MAX_STEPS).unwrap();
    }
    assert_eq!(session.events().len(), MAX_EVENTS);
    let result = session.step(1).unwrap();
    assert!(result.capped);
    assert_eq!(result.applied, 0);
    assert_eq!(session.revision(), MAX_EVENTS as u32);
    assert_eq!(session.reset(0).unwrap_err().code, RefusalCode::Limit);
    assert!(session.export_log().unwrap().len() <= MAX_LOG_BYTES);
}

#[test]
fn host_log_budget_refuses_atomically_and_replay_respects_it() {
    let mut source = cell();
    source.step(3).unwrap();
    let log = source.export_log().unwrap();
    let mut limited = cell();
    limited.set_log_budget(limited.log_bytes()).unwrap();
    let before = limited.export_log().unwrap();
    assert_eq!(
        limited.step(1).unwrap().stopped.unwrap().code,
        RefusalCode::Limit
    );
    assert_eq!(
        limited.import_log(&log).unwrap_err().code,
        RefusalCode::Limit
    );
    assert_eq!(limited.export_log().unwrap(), before);
    assert!(limited.set_log_budget(0).is_err());
    limited.set_log_budget(MAX_LOG_BYTES).unwrap();
    limited.import_log(&log).unwrap();
    assert_eq!(limited.export_log().unwrap(), log);
}

#[test]
fn element_count_boundary_and_explicit_choice_are_observable() {
    let extra = |count| {
        (0..count)
            .map(|i| format!("extra{i} {{ initial 0; capacity 1; }}\n"))
            .collect::<String>()
    };
    let source = CELL.replace(
        "elements {",
        &format!("elements {{ {}", extra(MAX_ELEMENTS - 4)),
    );
    assert_eq!(
        open(&source, OpenOptions::default())
            .unwrap()
            .model()
            .elements
            .len(),
        MAX_ELEMENTS
    );
    let too_many = CELL.replace(
        "elements {",
        &format!("elements {{ {}", extra(MAX_ELEMENTS - 3)),
    );
    assert_eq!(
        open(&too_many, OpenOptions::default()).err().unwrap().code,
        RefusalCode::Limit
    );
    // Two initially enabled transitions: feed and finish; seed is an explicit scan offset.
    let choices = CELL.replace("robot { initial 0", "robot { initial 1");
    let mut first = open(
        &choices,
        OpenOptions {
            seed: 2,
            policy: Some(ChoicePolicy::SourceOrder),
        },
    )
    .unwrap();
    let mut rotated = open(
        &choices,
        OpenOptions {
            seed: 2,
            policy: Some(ChoicePolicy::RoundRobin),
        },
    )
    .unwrap();
    first.step(1).unwrap();
    rotated.step(1).unwrap();
    assert_eq!(
        first.events()[0].action,
        EventAction::Fire {
            transition: 0,
            automatic: true
        }
    );
    assert_eq!(
        rotated.events()[0].action,
        EventAction::Fire {
            transition: 2,
            automatic: true
        }
    );
}
