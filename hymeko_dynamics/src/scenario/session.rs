mod history;
use super::model::*;
use hymeko::{ir::ir::Ir, resolution::interner::Interner};

/// A bounded execution session. The immutable initial state plus log determines all
/// semantic state; the snapshot delivery cursor is the only presentation-independent
/// transport state. Closing/dropping a session releases its complete log.
pub struct ScenarioSession {
    pub(crate) log: EventLog,
    state: State,
    snapshot_cursor: u32,
    log_bytes: usize,
    log_budget: usize,
}
impl ScenarioSession {
    /// Opens a source-declared scenario; does not alter the compiled document.
    pub fn open(
        ir: &Ir,
        it: &Interner,
        root: usize,
        options: OpenOptions,
    ) -> Result<Self, Refusal> {
        Self::from_model(ScenarioModel::from_ir(ir, it, root)?, options)
    }
    fn from_model(model: ScenarioModel, mut options: OpenOptions) -> Result<Self, Refusal> {
        options.policy = Some(options.policy.unwrap_or(model.policy));
        let mut initial = State {
            values: model.elements.iter().map(|e| e.initial).collect(),
            observers: vec![0; model.observers.len()],
            logical_step: 0,
            choice_cursor: options.seed as usize % model.transitions.len().max(1),
            halted: false,
        };
        for (i, observer) in model.observers.iter().enumerate() {
            if observer.kind == ObserverKind::Sum {
                initial.observers[i] = Self::sum(&initial, &observer.targets);
            }
        }
        initial.halted = Self::invariants(&model, &initial)
            .iter()
            .any(|i| !i.valid && i.violation == ViolationPolicy::Halt);
        let log = EventLog {
            schema: SCHEMA.into(),
            model,
            options,
            initial: initial.clone(),
            events: Vec::new(),
        };
        let bytes = serde_json::to_vec(&log)
            .map_err(super::wire::encoding_error)?
            .len();
        if bytes > MAX_LOG_BYTES {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "model exceeds log byte cap",
            ));
        }
        Ok(Self {
            log,
            state: initial,
            snapshot_cursor: 0,
            log_bytes: bytes,
            log_budget: MAX_LOG_BYTES,
        })
    }
    pub fn revision(&self) -> u32 {
        self.log.events.len() as u32
    }
    pub fn model(&self) -> &ScenarioModel {
        &self.log.model
    }
    pub fn state(&self) -> &State {
        &self.state
    }
    pub fn events(&self) -> &[Event] {
        &self.log.events
    }
    /// Conservative serialized log size, including the immutable model and initial state.
    pub fn log_bytes(&self) -> usize {
        self.log_bytes
    }
    /// A host may impose a smaller shared resource budget. Existing history is never removed.
    pub fn set_log_budget(&mut self, bytes: usize) -> Result<(), Refusal> {
        if bytes < self.log_bytes {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "existing log exceeds available host budget",
            ));
        }
        self.log_budget = bytes.min(MAX_LOG_BYTES);
        Ok(())
    }
    fn sum(state: &State, targets: &[usize]) -> i64 {
        targets.iter().map(|&t| state.values[t]).sum()
    }
    fn invariants(model: &ScenarioModel, state: &State) -> Vec<InvariantStatus> {
        model
            .invariants
            .iter()
            .map(|i| {
                let actual = Self::sum(state, &i.targets);
                InvariantStatus {
                    identity: i.identity.clone(),
                    actual,
                    valid: actual >= i.minimum && actual <= i.maximum,
                    violation: i.violation,
                }
            })
            .collect()
    }
    fn evidence(&self, action: &Action) -> Vec<GuardEvidence> {
        action
            .guards
            .iter()
            .map(|guard| {
                let actual = self.state.values[guard.target];
                GuardEvidence {
                    guard: guard.clone(),
                    actual,
                    satisfied: guard.op.holds(actual, guard.value),
                }
            })
            .collect()
    }
    fn candidate(
        &self,
        action: &Action,
        input: Option<i64>,
        intervention: bool,
    ) -> Result<State, Refusal> {
        if self.state.halted && !intervention {
            return Err(Refusal::new(
                RefusalCode::Halted,
                "declared invariant halted automatic execution; declared interventions/reset remain available",
            ));
        }
        match (action.input_range, input) {
            (None, None) => (),
            (Some([min, max]), Some(v)) if v >= min && v <= max => (),
            _ => {
                return Err(Refusal::new(
                    RefusalCode::InvalidRequest,
                    "input does not match declared range",
                ));
            }
        }
        if let Some(failed) = self.evidence(action).into_iter().find(|g| !g.satisfied) {
            return Err(Refusal::new(
                RefusalCode::Disabled,
                format!(
                    "guard on {}: observed {} {:?} {} is false",
                    self.model().elements[failed.guard.target].identity.name,
                    failed.actual,
                    failed.guard.op,
                    failed.guard.value
                ),
            ));
        }
        let mut after = self.state.clone();
        for effect in &action.effects {
            let value = if effect.input {
                input.ok_or_else(|| Refusal::new(RefusalCode::InvalidRequest, "input required"))?
            } else {
                effect.value
            };
            let next = match effect.op {
                EffectKind::Set => value,
                EffectKind::Add => after.values[effect.target] + value,
            };
            if next < 0 || next > self.model().elements[effect.target].capacity {
                return Err(Refusal::new(
                    RefusalCode::Capacity,
                    format!(
                        "{} would exceed declared content/capacity bounds",
                        self.model().elements[effect.target].identity.name
                    ),
                ));
            }
            after.values[effect.target] = next;
        }
        after.logical_step += 1;
        for (i, observer) in self.model().observers.iter().enumerate() {
            let sum = Self::sum(&after, &observer.targets);
            after.observers[i] = match observer.kind {
                ObserverKind::Sum => sum,
                ObserverKind::Accumulate => after.observers[i] + sum,
                ObserverKind::Count => {
                    after.observers[i] + i64::from(observer.cause.is_none_or(|c| c == action.cause))
                }
            };
        }
        after.halted = Self::invariants(self.model(), &after)
            .iter()
            .any(|i| !i.valid && i.violation == ViolationPolicy::Halt);
        Ok(after)
    }
    fn selected(&self) -> Result<usize, Refusal> {
        let n = self.model().transitions.len();
        let start = match self.log.options.policy {
            Some(ChoicePolicy::RoundRobin) => self.state.choice_cursor,
            _ => 0,
        };
        for offset in 0..n {
            let i = (start + offset) % n;
            if self
                .candidate(&self.model().transitions[i], None, false)
                .is_ok()
            {
                return Ok(i);
            }
        }
        Err(Refusal::new(
            if self.state.halted {
                RefusalCode::Halted
            } else {
                RefusalCode::Disabled
            },
            "no transition is enabled",
        ))
    }
    fn event(&self, operation: EventAction) -> Result<Event, Refusal> {
        let (after, cause, guards) = match &operation {
            EventAction::Reset { target } => {
                let state = self.state_at(*target)?.clone();
                (state, self.model().reset_cause, Vec::new())
            }
            EventAction::Fire {
                transition,
                automatic,
            } => {
                if *automatic && self.selected()? != *transition {
                    return Err(Refusal::new(
                        RefusalCode::ReplayMismatch,
                        "transition violates choice policy",
                    ));
                }
                let action = self.model().transitions.get(*transition).ok_or_else(|| {
                    Refusal::new(RefusalCode::InvalidRequest, "unknown transition")
                })?;
                let mut after = self.candidate(action, None, false)?;
                after.choice_cursor = (*transition + 1) % self.model().transitions.len();
                (after, action.cause, self.evidence(action))
            }
            EventAction::Intervention { point, input } => {
                let action = self.model().interventions.get(*point).ok_or_else(|| {
                    Refusal::new(RefusalCode::InvalidRequest, "unknown intervention point")
                })?;
                (
                    self.candidate(action, *input, true)?,
                    action.cause,
                    self.evidence(action),
                )
            }
        };
        Ok(Event {
            revision: self.revision() + 1,
            action: operation,
            cause: self.model().causes[cause].clone(),
            before: self.state.clone(),
            invariants: Self::invariants(self.model(), &after),
            after,
            guards,
        })
    }
    fn append(&mut self, event: Event) -> Result<(), Refusal> {
        let bytes = serde_json::to_vec(&event)
            .map_err(super::wire::encoding_error)?
            .len()
            + 1;
        if self.log.events.len() >= MAX_EVENTS || self.log_bytes + bytes > self.log_budget {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "append-only log cap reached; export and open another session",
            ));
        }
        self.log_bytes += bytes;
        self.state = event.after.clone();
        self.log.events.push(event);
        Ok(())
    }
    fn commit(&mut self, operation: EventAction) -> Result<MutationResult, Refusal> {
        let event = self.event(operation)?;
        self.append(event)?;
        Ok(MutationResult {
            revision: self.revision(),
            applied: 1,
            capped: false,
            stopped: None,
        })
    }
    /// Bounded logical stepping. Partial progress and the exact stop/cap reason are explicit.
    pub fn step(&mut self, steps: usize) -> Result<MutationResult, Refusal> {
        if steps == 0 {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "steps must be positive",
            ));
        }
        let mut result = MutationResult {
            revision: self.revision(),
            applied: 0,
            capped: steps > MAX_STEPS,
            stopped: None,
        };
        for _ in 0..steps.min(MAX_STEPS) {
            let next = self.selected().and_then(|transition| {
                self.commit(EventAction::Fire {
                    transition,
                    automatic: true,
                })
            });
            match next {
                Ok(_) => result.applied += 1,
                Err(refusal) => {
                    result.capped |= refusal.capped;
                    result.stopped = Some(refusal);
                    break;
                }
            }
        }
        result.revision = self.revision();
        Ok(result)
    }
    /// Explicit firing is an ordered external command; its chosen identity is logged.
    pub fn fire(&mut self, name: &str) -> Result<MutationResult, Refusal> {
        let transition = self
            .model()
            .transitions
            .iter()
            .position(|a| a.identity.name == name)
            .ok_or_else(|| Refusal::new(RefusalCode::InvalidRequest, "unknown transition name"))?;
        self.commit(EventAction::Fire {
            transition,
            automatic: false,
        })
    }
    pub fn intervene(&mut self, name: &str, input: Option<i64>) -> Result<MutationResult, Refusal> {
        let point = self
            .model()
            .interventions
            .iter()
            .position(|a| a.identity.name == name)
            .ok_or_else(|| {
                Refusal::new(RefusalCode::InvalidRequest, "unknown intervention name")
            })?;
        self.commit(EventAction::Intervention { point, input })
    }
    /// Restore any recorded state by appending a reset event. Never truncates the log.
    pub fn reset(&mut self, revision: u32) -> Result<MutationResult, Refusal> {
        self.commit(EventAction::Reset { target: revision })
    }
}
