//! Retained, renderer-independent bindings for one IR-authored mechanism.
//!
//! The same facade binds an arm, hinge, slider or rigid object. Geometry owns FK and collision;
//! hosts supply admitted coordinate updates and the registration transform, then borrow poses.
mod attachments;
mod binding;
mod error;
mod state;

pub use attachments::FrameAttachment;
pub use error::{AssemblyError, StateError};

use super::{kinematic::KinematicModel, visual::VisualInstance};
use binding::Bindings;
use hymeko::{common::ids::DeclId, module_store::execution::ValueUpdate};
use hymeko_clifford::Se3;
use hymeko_query::{QueryEngine, traits::NameResolver};
use state::PoseState;

const ELEMENT_LIMIT: usize = 1024;

/// One bound mechanism, with cold declaration bindings separate from its reusable numeric state.
///
/// Declaration IDs belong to the admitted document structure used by `read`. Hosts check source,
/// epoch and structure before supplying deltas, and rebind after a structural replacement.
pub struct KinematicAssembly {
    bindings: Bindings,
    pose: PoseState,
}

impl KinematicAssembly {
    /// Compile bindings once from one canonical scope; no source or canonical IR is cloned.
    ///
    /// Joint `coordinate` references optionally name numeric leaves (radians/metres). Unbound
    /// coordinates start at zero. Optional `attachment_kind` names an authored edge kind whose
    /// instances bind `(+ body, - frame)` with local `origin` and `rpy_deg` fields.
    pub fn read<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        scope: DeclId,
    ) -> Result<Self, AssemblyError> {
        let bindings = Bindings::read(engine, scope)?;
        let pose = PoseState::new(&bindings)?;
        Ok(Self { bindings, pose })
    }

    /// Canonical identity of this assembly's selected group.
    pub fn scope(&self) -> DeclId {
        self.bindings.scope
    }
    /// Existing model projection, including declared geometry and collision categories.
    pub fn model(&self) -> &KinematicModel {
        &self.bindings.model
    }
    /// Resolve the vector slot of a canonical joint, once when a producer binds to this assembly.
    pub fn joint_index(&self, joint: DeclId) -> Option<usize> {
        self.bindings.joints.get(&joint).copied()
    }
    /// Currently accepted joint coordinates; radians for angular joints, metres for prismatic.
    pub fn coordinates(&self) -> &[f64] {
        &self.pose.coordinates
    }
    /// Observed coordinates and a rigid root placement, validated before changing published poses.
    /// Declared position limits are not used to clamp observations.
    pub fn set_state(&mut self, coordinates: &[f64], root: Se3) -> Result<(), StateError> {
        self.pose.set(&self.bindings, coordinates, root)
    }
    /// Apply numeric changes to the bound `coordinate` leaves. Unrelated/unchanged values are cached.
    /// Missing, textual, nonnumeric or nonfinite coordinate writes refuse the entire pose update.
    pub fn apply_values(&mut self, updates: &[ValueUpdate<'_>]) -> Result<bool, StateError> {
        self.pose.apply(&self.bindings, updates)
    }
    /// Link frame in the same world coordinates used by every visual/collider/attachment.
    pub fn link_pose(&self, link: DeclId) -> Option<&Se3> {
        self.bindings
            .links
            .get(&link)
            .and_then(|&index| self.pose.links.get(index))
    }
    /// Owner-projected visual metadata, paired by index with `visual_poses()`.
    pub fn visuals(&self) -> &[VisualInstance] {
        &self.bindings.visuals.instances
    }
    /// World poses of visuals, including each visual's authored local offset.
    pub fn visual_poses(&self) -> &[Se3] {
        &self.pose.visuals
    }
    /// Existing collider metadata; no render geometry is promoted into collision geometry.
    pub fn colliders(&self) -> &[crate::collide::Collider] {
        &self.bindings.colliders
    }
    /// Collision placements produced by the existing collision owner from those same link poses.
    pub fn collider_placements(&self) -> &[crate::collide::Placement] {
        &self.pose.colliders
    }
    /// Named frame attachments, such as a tool, sensor or grasp reference.
    pub fn attachments(&self) -> &[FrameAttachment] {
        &self.bindings.attachments
    }
    /// World pose of a canonical attached frame.
    pub fn attachment_pose(&self, frame: DeclId) -> Option<&Se3> {
        self.bindings
            .frames
            .get(&frame)
            .and_then(|&index| self.pose.attachments.get(index))
    }
}
