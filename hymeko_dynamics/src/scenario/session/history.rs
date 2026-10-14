//! Historical reads, snapshot delivery and verified replay.
use super::*;
impl ScenarioSession {
    pub fn state_at(&self, revision: u32) -> Result<&State, Refusal> {
        if revision == 0 {
            return Ok(&self.log.initial);
        }
        self.log
            .events
            .get(revision as usize - 1)
            .map(|e| &e.after)
            .ok_or_else(|| Refusal::new(RefusalCode::InvalidRequest, "revision is not recorded"))
    }
    /// One complete frame plus a bounded page of events. `through_revision` is the
    /// continuation cursor; it may be below the current state's `revision`.
    /// `None` resumes this session's delivery cursor. Explicit cursors do not alter it.
    pub fn snapshot(&mut self, since: Option<u32>) -> Result<Snapshot, Refusal> {
        let start = since.unwrap_or(self.snapshot_cursor);
        if start > self.revision() {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "snapshot revision is in the future",
            ));
        }
        let end = (start as usize + MAX_SNAPSHOT_EVENTS).min(self.log.events.len());
        let result = Snapshot {
            schema: SCHEMA.into(),
            scenario: self.model().identity.clone(),
            revision: self.revision(),
            logical_step: self.state.logical_step,
            halted: self.state.halted,
            policy: self.log.options.policy.unwrap_or(self.model().policy),
            seed: self.log.options.seed,
            elements: self
                .model()
                .elements
                .iter()
                .zip(&self.state.values)
                .map(|(e, &value)| ElementState {
                    identity: e.identity.clone(),
                    value,
                    capacity: e.capacity,
                })
                .collect(),
            transitions: self
                .model()
                .transitions
                .iter()
                .map(|a| {
                    let refusal = self.candidate(a, None, false).err();
                    TransitionStatus {
                        identity: a.identity.clone(),
                        enabled: refusal.is_none(),
                        refusal,
                    }
                })
                .collect(),
            interventions: self.model().interventions.clone(),
            causes: self.model().causes.clone(),
            observers: self
                .model()
                .observers
                .iter()
                .zip(&self.state.observers)
                .map(|(o, &value)| ObserverValue {
                    identity: o.identity.clone(),
                    value,
                })
                .collect(),
            invariants: Self::invariants(self.model(), &self.state),
            events: self.log.events[start as usize..end].to_vec(),
            since_revision: start,
            through_revision: end as u32,
            capped: end < self.log.events.len(),
        };
        if since.is_none() {
            self.snapshot_cursor = end as u32;
        }
        Ok(result)
    }
    /// Import into a fresh session only, preserving monotonic revision and append-only
    /// history. Recompute every event, compare complete evidence, and adopt atomically.
    pub fn import_log(&mut self, json: &str) -> Result<MutationResult, Refusal> {
        if self.revision() != 0 {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "import requires a fresh session",
            ));
        }
        if json.len() > self.log_budget {
            return Err(Refusal::new(
                RefusalCode::Limit,
                "log import exceeds byte cap",
            ));
        }
        let imported: EventLog = serde_json::from_str(json)
            .map_err(|e| Refusal::new(RefusalCode::InvalidRequest, e.to_string()))?;
        if imported.schema != SCHEMA {
            return Err(Refusal::new(
                RefusalCode::SchemaMismatch,
                "unsupported log schema",
            ));
        }
        if imported.model != self.log.model || imported.options != self.log.options {
            return Err(Refusal::new(
                RefusalCode::ReplayMismatch,
                "scenario definition, seed or policy differs",
            ));
        }
        if imported.events.len() > MAX_EVENTS {
            return Err(Refusal::new(RefusalCode::Limit, "log event cap exceeded"));
        }
        let mut replay = Self::from_model(self.log.model.clone(), self.log.options.clone())?;
        replay.set_log_budget(self.log_budget)?;
        if imported.initial != replay.log.initial {
            return Err(Refusal::new(
                RefusalCode::ReplayMismatch,
                "initial state differs",
            ));
        }
        for recorded in imported.events {
            let expected = replay.event(recorded.action.clone())?;
            if recorded != expected {
                return Err(Refusal::new(
                    RefusalCode::ReplayMismatch,
                    format!("event {} evidence differs", recorded.revision),
                ));
            }
            replay.append(expected)?;
        }
        let result = MutationResult {
            revision: replay.revision(),
            applied: replay.log.events.len(),
            capped: false,
            stopped: None,
        };
        *self = replay;
        Ok(result)
    }
}
