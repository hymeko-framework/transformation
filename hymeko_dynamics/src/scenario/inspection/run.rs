use super::super::{OpenOptions, Refusal, RefusalCode, ScenarioSession, Snapshot};
use super::{model::*, read::InspectionReader};
use hymeko::{common::ids::DeclId, ir::ir::ValueR, module_store::execution::ValueUpdate};
use hymeko::{ir::ir::Ir, resolution::interner::Interner};

/// One observed sequence. Discrete progress/history is owned by ScenarioSession.
/// Only the previous sample and current qualification state are retained here.
pub struct InspectionSession {
    model: InspectionModel,
    workflow: ScenarioSession,
    last: Option<PoseObservation>,
    now_ns: u64,
    good: bool,
    dwell_ns: u64,
    evidence: Option<Evidence>,
    available: bool,
}
impl InspectionSession {
    /// Open an already compiled coherent model; no source parsing or IR mutation.
    /// Rejects unsupported schema, unbounded inputs and incompatible workflow effects.
    pub fn open(ir: &Ir, names: &Interner, root: usize) -> Result<Self, Refusal> {
        let (model, workflow) = InspectionReader::new(ir, names)?.read(root)?;
        let workflow = ScenarioSession::open(ir, names, workflow, OpenOptions::default())?;
        Ok(Self {
            model,
            workflow,
            last: None,
            now_ns: 0,
            good: false,
            dwell_ns: 0,
            evidence: None,
            available: false,
        })
    }
    /// Immutable configuration, preserving canonical declaration identities.
    pub fn model(&self) -> &InspectionModel {
        &self.model
    }
    fn completed(&self) -> usize {
        self.workflow.state().values[self.model.workflow.completed] as usize
    }
    fn blocked(&self) -> bool {
        self.workflow.state().values[self.model.workflow.blocked] != 0
    }
    fn complete(&self) -> bool {
        self.completed() > self.model.required_visits()
    }
    /// Next authored goal, or None only after observed home qualification.
    pub fn target(&self) -> Option<&Station> {
        self.model.target_at(self.completed())
    }
    fn clock(&self, now_ns: u64) -> Result<(), Refusal> {
        if now_ns < self.now_ns {
            Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "clock moved backwards; open a new run",
            ))
        } else {
            Ok(())
        }
    }
    fn fresh_at(&self, now_ns: u64) -> bool {
        self.available
            && self
                .last
                .as_ref()
                .is_some_and(|last| now_ns.saturating_sub(last.acquired_ns) <= self.model.expiry_ns)
    }
    fn validate(&self, sample: &PoseObservation, now_ns: u64) -> Result<(), Refusal> {
        self.clock(now_ns)?;
        sample.pose.validate()?;
        if sample.source != self.model.source
            || sample.frame != self.model.frame
            || sample.epoch.is_empty()
            || sample.epoch.len() > 128
            || !sample
                .epoch
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b"._-/".contains(&b))
            || sample.sequence == 0
            || sample.acquired_ns > now_ns
            || now_ns - sample.acquired_ns > self.model.expiry_ns
        {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "source/frame/epoch/time or freshness mismatch",
            ));
        }
        if self.last.as_ref().is_some_and(|last| {
            sample.epoch != last.epoch
                || sample.sequence <= last.sequence
                || sample.acquired_ns <= last.acquired_ns
        }) {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "duplicate, old sample or changed epoch; open a new run for a new epoch",
            ));
        }
        Ok(())
    }
    fn expire(&mut self, now_ns: u64) -> Result<(), Refusal> {
        if self.last.is_some() && !self.fresh_at(now_ns) && !self.blocked() && !self.complete() {
            self.workflow.intervene(&self.model.workflow.block, None)?;
            self.good = false;
            self.dwell_ns = 0;
        }
        Ok(())
    }
    fn qualify(&self, sample: &PoseObservation, target: &Station) -> Evidence {
        let interval = self
            .last
            .as_ref()
            .map(|last| sample.acquired_ns - last.acquired_ns);
        let speed = self
            .last
            .as_ref()
            .zip(interval)
            .filter(|(_, dt)| *dt <= self.model.max_gap_ns)
            .map(|(last, dt)| sample.pose.distance(&last.pose) / (dt as f64 * 1e-9));
        let position = sample.pose.distance(&target.pose);
        let angle = sample.pose.angle(&target.pose);
        let qualified = position <= target.criteria.position_m
            && angle <= target.criteria.orientation_rad
            && speed.is_some_and(|v| v <= target.criteria.speed_m_s);
        let dwell_ns = if qualified && self.good {
            self.dwell_ns + interval.unwrap_or(0)
        } else {
            0
        };
        Evidence {
            target: target.identity.clone(),
            position_error_m: position,
            orientation_error_rad: angle,
            speed_m_s: speed,
            qualified,
            dwell_ns,
        }
    }
    /// Admit one genuinely new acquisition in the authored frame/source.
    /// Invalid input leaves all state unchanged. A source gap latches progress
    /// blocking even if this new sample is fresh; it cannot silently resume.
    pub fn observe(
        &mut self,
        sample: PoseObservation,
        now_ns: u64,
    ) -> Result<InspectionSnapshot, Refusal> {
        self.validate(&sample, now_ns)?;
        self.expire(now_ns)?;
        let evidence = self.target().map(|target| self.qualify(&sample, target));
        let mut advanced = false;
        if !self.blocked()
            && let (Some(evidence), Some(target)) = (&evidence, self.target())
            && evidence.qualified
            && evidence.dwell_ns >= target.criteria.dwell_ns
        {
            let action = if self.completed() == self.model.required_visits() {
                &self.model.workflow.finish
            } else {
                &self.model.workflow.advance
            };
            self.workflow.fire(action)?;
            advanced = true;
        }
        self.good = !advanced && !self.blocked() && evidence.as_ref().is_some_and(|e| e.qualified);
        self.dwell_ns = if self.good {
            evidence.as_ref().map_or(0, |e| e.dwell_ns)
        } else {
            0
        };
        self.evidence = evidence.map(|mut e| {
            if self.blocked() {
                e.qualified = false;
                e.dwell_ns = 0;
            }
            e
        });
        self.last = Some(sample);
        self.available = true;
        self.now_ns = now_ns;
        Ok(self.snapshot())
    }
    /// Advance the same source clock to detect absence. No samples are invented.
    pub fn tick(&mut self, now_ns: u64) -> Result<InspectionSnapshot, Refusal> {
        self.clock(now_ns)?;
        self.expire(now_ns)?;
        self.now_ns = now_ns;
        Ok(self.snapshot())
    }
    /// Invalidate an unavailable source without inventing source-clock progress.
    /// New evidence restores availability; progress still needs explicit resume.
    pub fn invalidate(&mut self) -> Result<InspectionSnapshot, Refusal> {
        if !self.blocked() && !self.complete() {
            self.workflow.intervene(&self.model.workflow.block, None)?;
        }
        self.available = false;
        self.good = false;
        self.dwell_ns = 0;
        Ok(self.snapshot())
    }
    /// Explicitly resume observation-driven progress after fresh data returns.
    /// This is NOT a robot execution or a confirmation of physical stopping.
    pub fn resume(&mut self, now_ns: u64) -> Result<InspectionSnapshot, Refusal> {
        self.clock(now_ns)?;
        if !self.blocked() || !self.fresh_at(now_ns) || self.complete() {
            return Err(Refusal::new(
                RefusalCode::InvalidRequest,
                "resume requires blocked progress and fresh admitted evidence",
            ));
        }
        self.workflow.intervene(&self.model.workflow.resume, None)?;
        self.good = false;
        self.dwell_ns = 0;
        self.evidence = None;
        self.now_ns = now_ns;
        Ok(self.snapshot())
    }
    /// Current view at the last admitted clock boundary. Call tick for absence.
    pub fn snapshot(&self) -> InspectionSnapshot {
        let done = self.completed();
        let required = self.model.required_visits();
        let phase = if self.complete() {
            InspectionPhase::Complete
        } else if self.blocked() {
            InspectionPhase::Blocked
        } else if self.last.is_none() {
            InspectionPhase::Missing
        } else if done == required {
            InspectionPhase::Returning
        } else if self.good {
            InspectionPhase::Qualifying
        } else {
            InspectionPhase::Approaching
        };
        InspectionSnapshot {
            task: self.model.identity.clone(),
            phase,
            completed_visits: done.min(required),
            required_visits: required,
            target: self.target().map(|t| t.identity.clone()),
            returned_home: self.complete(),
            fresh: self.fresh_at(self.now_ns),
            sample_sequence: self.last.as_ref().map(|s| s.sequence),
            acquired_ns: self.last.as_ref().map(|s| s.acquired_ns),
            evidence: self.evidence.clone(),
            revision: self.workflow.revision(),
        }
    }
    /// Project current evidence into the authored output leaves of the same IR.
    /// The host must commit against the admitted model basis and reconcile on
    /// failure. Validity flags distinguish missing values from numeric zero.
    /// No timestamp strings are interned; all u64 parts are exactly representable.
    pub fn value_updates(&self) -> Vec<ValueUpdate<'_>> {
        let state = self.snapshot();
        let output = &self.model.output;
        let phase = match state.phase {
            InspectionPhase::Missing => "missing",
            InspectionPhase::Approaching => "approaching",
            InspectionPhase::Qualifying => "qualifying",
            InspectionPhase::Returning => "returning",
            InspectionPhase::Blocked => "blocked",
            InspectionPhase::Complete => "complete",
        };
        let evidence = state.evidence.as_ref();
        let valid = evidence.is_some() && state.fresh;
        let speed = evidence.and_then(|e| e.speed_m_s);
        let sequence = state.sample_sequence.unwrap_or(0);
        let acquired = state.acquired_ns.unwrap_or(0);
        let flag = |value: bool| f64::from(u8::from(value));
        let values = [
            state.completed_visits as f64,
            state.required_visits as f64,
            flag(state.fresh),
            flag(state.returned_home),
            flag(state.target.is_some()),
            flag(valid),
            flag(valid && evidence.is_some_and(|e| e.qualified)),
            evidence.map_or(0., |e| e.position_error_m),
            evidence.map_or(0., |e| e.orientation_error_rad),
            flag(valid && speed.is_some()),
            speed.unwrap_or(0.),
            evidence.map_or(0., |e| e.dwell_ns as f64 * 1e-9),
            flag(state.sample_sequence.is_some()),
            (sequence >> 32) as f64,
            (sequence & u32::MAX as u64) as f64,
            (acquired / 1_000_000_000) as f64,
            (acquired % 1_000_000_000) as f64,
        ];
        let mut updates = output
            .numbers
            .iter()
            .copied()
            .zip(values)
            .map(|(declaration, value)| ValueUpdate::Set {
                declaration,
                value: ValueR::Num(value),
            })
            .collect::<Vec<_>>();
        updates.push(ValueUpdate::Text {
            declaration: output.phase,
            value: phase,
        });
        updates.push(ValueUpdate::Text {
            declaration: output.target_path,
            value: self
                .target()
                .map_or("", |target| target.identity.name.as_str()),
        });
        updates.push(ValueUpdate::Text {
            declaration: output.evaluated_target_path,
            value: self
                .evidence
                .as_ref()
                .map_or("", |e| e.target.name.as_str()),
        });
        for (declaration, target) in [
            (
                output.target,
                state
                    .target
                    .as_ref()
                    .map_or(self.model.home.identity.id, |t| t.id),
            ),
            (
                output.evaluated_target,
                evidence.map_or(self.model.home.identity.id, |e| e.target.id),
            ),
        ] {
            updates.push(ValueUpdate::Set {
                declaration,
                value: ValueR::Ref(DeclId::new(target)),
            });
        }
        updates
    }
    /// Existing bounded discrete transition evidence, not a replay of raw samples.
    pub fn transitions(&mut self, since: Option<u32>) -> Result<Snapshot, Refusal> {
        self.workflow.snapshot(since)
    }
    /// Current bounded transition-log size, for a caller-owned resource budget.
    pub fn log_bytes(&self) -> usize {
        self.workflow.log_bytes()
    }
    /// Preserve the scenario owner's budget; a host may only impose a lower cap.
    pub fn set_log_budget(&mut self, bytes: usize) -> Result<(), Refusal> {
        self.workflow.set_log_budget(bytes)
    }
}
