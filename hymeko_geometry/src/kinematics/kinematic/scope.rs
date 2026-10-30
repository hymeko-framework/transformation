//! Strict group projection through the query owner's canonical scope view.
use super::{KinematicModel, extract_kinematic_model};
use hymeko::common::ids::DeclId;
use hymeko_query::{Predicate, QueryEngine, engine::scope::ScopeError, traits::NameResolver};
use std::collections::HashSet;

/// A scoped mechanism refers to an endpoint it does not own or cannot anchor to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ScopedModelError {
    Scope(ScopeError),
    MissingEndpoint { joint: DeclId },
    OutsideGroup { joint: DeclId, endpoint: DeclId },
}
impl std::fmt::Display for ScopedModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Scope(error) => error.fmt(f),
            Self::MissingEndpoint { joint } => {
                write!(f, "joint {} has no resolved parent/child", joint.raw())
            }
            Self::OutsideGroup { joint, endpoint } => write!(
                f,
                "joint {} refers to body {} outside its group",
                joint.raw(),
                endpoint.raw()
            ),
        }
    }
}
impl std::error::Error for ScopedModelError {}
impl From<ScopeError> for ScopedModelError {
    fn from(error: ScopeError) -> Self {
        Self::Scope(error)
    }
}

impl KinematicModel {
    /// Project one canonical declaration group without collecting neighboring mechanisms.
    ///
    /// The existing extractor remains the single implementation of link/joint/property reading.
    /// An outside parent is allowed only when it is a declared frame, never another group's body.
    /// Shared axes, geometry and vocabulary references keep their original document context.
    pub fn from_scope<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        root: DeclId,
    ) -> Result<Self, ScopedModelError> {
        let selected = engine.scoped(root)?;
        let name = engine
            .resolver()
            .resolve(engine.ir().decl_node(root).unwrap().name);
        let mut model = extract_kinematic_model(&selected, name);
        let boundary = ModelBoundary {
            metadata: QueryEngine::new(engine.ir(), engine.resolver()),
            bodies: model.links.iter().map(|link| link.did).collect(),
        };
        for joint in &model.joints {
            if joint.parent.is_none() || joint.child.is_none() {
                return Err(ScopedModelError::MissingEndpoint { joint: joint.did });
            }
            boundary.require_body(joint.did, joint.child)?;
            if !boundary.bodies.contains(&joint.parent) {
                let parent = boundary.external_frame(joint.did, joint.parent)?;
                if !model.frames.contains(&parent) {
                    model.frames.push(parent);
                }
            }
        }
        Ok(model)
    }
}

struct ModelBoundary<'ir, R: NameResolver> {
    metadata: QueryEngine<'ir, R>,
    bodies: HashSet<DeclId>,
}
impl<R: NameResolver> ModelBoundary<'_, R> {
    fn require_body(&self, joint: DeclId, endpoint: DeclId) -> Result<(), ScopedModelError> {
        self.bodies
            .contains(&endpoint)
            .then_some(())
            .ok_or(ScopedModelError::OutsideGroup { joint, endpoint })
    }

    fn external_frame(&self, joint: DeclId, endpoint: DeclId) -> Result<String, ScopedModelError> {
        if self
            .metadata
            .query_decl(endpoint, &Predicate::inherits("link"))
            .is_none()
            && let Some(frame) = self
                .metadata
                .query_decl(endpoint, &Predicate::inherits("frame"))
        {
            return Ok(frame.name);
        }
        Err(ScopedModelError::OutsideGroup { joint, endpoint })
    }
}
