use serde::{Deserialize, Serialize};

/// The scenario runtime a model names, and a snapshot's schema: named without a version.
pub const SCHEMA: &str = "hymeko-scenario";
/// Whether a model names this runtime; one authored before 2026-10-14 says `hymeko-scenario/v1`.
pub fn runtime_is(name: &str) -> bool {
    name == SCHEMA || name.strip_suffix("/v1") == Some(SCHEMA)
}
pub const MAX_ELEMENTS: usize = 256;
pub const MAX_ACTIONS: usize = 256;
pub const MAX_OBSERVERS: usize = 64;
pub const MAX_STEPS: usize = 256;
pub const MAX_EVENTS: usize = 4096;
pub const MAX_SNAPSHOT_EVENTS: usize = 128;
pub const MAX_LOG_BYTES: usize = 16 * 1024 * 1024;
pub const MAX_VALUE: i64 = 1_000_000_000;

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Identity {
    pub id: usize,
    pub name: String,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Cause {
    pub identity: Identity,
    pub label: String,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ChoicePolicy {
    SourceOrder,
    RoundRobin,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OpenOptions {
    pub seed: u32,
    pub policy: Option<ChoicePolicy>,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum Comparison {
    Eq,
    Ne,
    Ge,
    Le,
}
impl Comparison {
    pub(crate) fn holds(self, actual: i64, expected: i64) -> bool {
        match self {
            Self::Eq => actual == expected,
            Self::Ne => actual != expected,
            Self::Ge => actual >= expected,
            Self::Le => actual <= expected,
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum EffectKind {
    Set,
    Add,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ObserverKind {
    Sum,
    Accumulate,
    Count,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum ViolationPolicy {
    Record,
    Halt,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Element {
    pub identity: Identity,
    pub initial: i64,
    pub capacity: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Guard {
    pub target: usize,
    pub op: Comparison,
    pub value: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Effect {
    pub target: usize,
    pub op: EffectKind,
    pub value: i64,
    pub input: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Action {
    pub identity: Identity,
    pub cause: usize,
    pub guards: Vec<Guard>,
    pub effects: Vec<Effect>,
    pub input_range: Option<[i64; 2]>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Observer {
    pub identity: Identity,
    pub kind: ObserverKind,
    pub targets: Vec<usize>,
    pub cause: Option<usize>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Invariant {
    pub identity: Identity,
    pub targets: Vec<usize>,
    pub minimum: i64,
    pub maximum: i64,
    pub violation: ViolationPolicy,
}
/// Immutable projection with canonical declaration identities, not a replacement IR.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ScenarioModel {
    pub schema: String,
    pub identity: Identity,
    pub policy: ChoicePolicy,
    pub reset_cause: usize,
    pub causes: Vec<Cause>,
    pub elements: Vec<Element>,
    pub transitions: Vec<Action>,
    pub interventions: Vec<Action>,
    pub observers: Vec<Observer>,
    pub invariants: Vec<Invariant>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct State {
    pub values: Vec<i64>,
    pub observers: Vec<i64>,
    pub logical_step: u32,
    pub choice_cursor: usize,
    pub halted: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct GuardEvidence {
    pub guard: Guard,
    pub actual: i64,
    pub satisfied: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct InvariantStatus {
    pub identity: Identity,
    pub actual: i64,
    pub valid: bool,
    pub violation: ViolationPolicy,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "camelCase", deny_unknown_fields)]
pub enum EventAction {
    Fire { transition: usize, automatic: bool },
    Intervention { point: usize, input: Option<i64> },
    Reset { target: u32 },
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Event {
    pub revision: u32,
    pub action: EventAction,
    pub cause: Cause,
    pub before: State,
    pub after: State,
    pub guards: Vec<GuardEvidence>,
    pub invariants: Vec<InvariantStatus>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct EventLog {
    pub schema: String,
    pub model: ScenarioModel,
    pub options: OpenOptions,
    pub initial: State,
    pub events: Vec<Event>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Refusal {
    pub code: RefusalCode,
    pub reason: String,
    pub capped: bool,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub enum RefusalCode {
    InvalidModel,
    InvalidRequest,
    Disabled,
    Halted,
    Capacity,
    Limit,
    SchemaMismatch,
    ReplayMismatch,
    Closed,
}
impl Refusal {
    pub(crate) fn new(code: RefusalCode, reason: impl Into<String>) -> Self {
        Self {
            code,
            reason: reason.into(),
            capped: code == RefusalCode::Limit,
        }
    }
}
impl std::fmt::Display for Refusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}: {}", self.code, self.reason)
    }
}
impl std::error::Error for Refusal {}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MutationResult {
    pub revision: u32,
    pub applied: usize,
    pub capped: bool,
    pub stopped: Option<Refusal>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ElementState {
    pub identity: Identity,
    pub value: i64,
    pub capacity: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ObserverValue {
    pub identity: Identity,
    pub value: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct TransitionStatus {
    pub identity: Identity,
    pub enabled: bool,
    pub refusal: Option<Refusal>,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Snapshot {
    pub schema: String,
    pub scenario: Identity,
    pub revision: u32,
    pub logical_step: u32,
    pub halted: bool,
    pub policy: ChoicePolicy,
    pub seed: u32,
    pub elements: Vec<ElementState>,
    pub transitions: Vec<TransitionStatus>,
    pub interventions: Vec<Action>,
    pub causes: Vec<Cause>,
    pub observers: Vec<ObserverValue>,
    pub invariants: Vec<InvariantStatus>,
    pub events: Vec<Event>,
    pub since_revision: u32,
    pub through_revision: u32,
    pub capped: bool,
}
