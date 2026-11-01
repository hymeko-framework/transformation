use super::{StateError, binding::Bindings};
use crate::collide::{Placement, place};
use hymeko::{ir::ir::ValueR, module_store::execution::ValueUpdate};
use hymeko_clifford::Se3;

pub(super) struct PoseState {
    pub coordinates: Vec<f64>,
    candidate: Vec<f64>,
    scratch: Vec<Se3>,
    pub links: Vec<Se3>,
    pub visuals: Vec<Se3>,
    pub attachments: Vec<Se3>,
    pub colliders: Vec<Placement>,
    root: Se3,
}
impl PoseState {
    pub(super) fn new(bindings: &Bindings) -> Result<Self, StateError> {
        let mut state = Self {
            coordinates: bindings.initial.clone(),
            candidate: bindings.initial.clone(),
            scratch: bindings.plan.identity_poses_se3(),
            links: bindings.plan.identity_poses_se3(),
            visuals: vec![Se3::identity(); bindings.visuals.instances.len()],
            attachments: vec![Se3::identity(); bindings.attachments.len()],
            colliders: Vec::with_capacity(bindings.colliders.len()),
            root: Se3::identity(),
        };
        state.resolve(bindings, Se3::identity())?;
        Ok(state)
    }

    pub(super) fn set(
        &mut self,
        bindings: &Bindings,
        coordinates: &[f64],
        root: Se3,
    ) -> Result<(), StateError> {
        if coordinates.len() != self.coordinates.len() {
            return Err(StateError::JointCount {
                expected: self.coordinates.len(),
                received: coordinates.len(),
            });
        }
        self.candidate.copy_from_slice(coordinates);
        self.resolve(bindings, root)
    }

    pub(super) fn apply(
        &mut self,
        bindings: &Bindings,
        updates: &[ValueUpdate<'_>],
    ) -> Result<bool, StateError> {
        self.candidate.copy_from_slice(&self.coordinates);
        for update in updates {
            let declaration = match update {
                ValueUpdate::Set { declaration, .. }
                | ValueUpdate::Text { declaration, .. }
                | ValueUpdate::Clear { declaration } => *declaration,
            };
            let Some(indices) = bindings.inputs.get(&declaration) else {
                continue;
            };
            let ValueUpdate::Set {
                value: ValueR::Num(value),
                ..
            } = update
            else {
                return Err(StateError::InvalidCoordinateSample { declaration });
            };
            if !value.is_finite() {
                return Err(StateError::InvalidCoordinateSample { declaration });
            }
            for &index in indices {
                self.candidate[index] = *value;
            }
        }
        if self.candidate == self.coordinates {
            return Ok(false);
        }
        self.resolve(bindings, self.root)?;
        Ok(true)
    }

    fn resolve(&mut self, bindings: &Bindings, root: Se3) -> Result<(), StateError> {
        for (index, value) in self.candidate.iter().enumerate() {
            if !value.is_finite() {
                return Err(StateError::NonFiniteCoordinate { index });
            }
        }
        if !Self::valid(&root) {
            return Err(StateError::InvalidPlacement);
        }
        bindings
            .plan
            .resolve_se3(&bindings.model, &self.candidate, &mut self.scratch);
        for (link, pose) in self.scratch.iter_mut().enumerate() {
            *pose = root.compose(pose);
            if !Self::valid(pose) {
                return Err(StateError::NonFinitePose { link });
            }
        }
        std::mem::swap(&mut self.scratch, &mut self.links);
        self.coordinates.copy_from_slice(&self.candidate);
        self.root = root;
        for (pose, visual) in self.visuals.iter_mut().zip(&bindings.visuals.instances) {
            *pose = self.links[visual.link_index].compose(&visual.local_pose);
        }
        for (pose, frame) in self.attachments.iter_mut().zip(&bindings.attachments) {
            *pose = self.links[frame.link_index].compose(&frame.local_pose);
        }
        place(&bindings.colliders, &self.links, &mut self.colliders);
        Ok(())
    }

    fn valid(pose: &Se3) -> bool {
        let q = pose.quaternion();
        pose.translation.iter().all(|n| n.is_finite())
            && q.iter().all(|n| n.is_finite())
            && (q.iter().map(|n| n * n).sum::<f64>() - 1.0).abs() <= 1.0e-6
    }
}
