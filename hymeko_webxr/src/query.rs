//! Borrowed queries over the canonical HyMeKo spatial vocabulary.
//!
//! This is the public semantic read boundary for renderers and perception adapters. It keeps
//! [`DeclId`] as identity and borrows text from the core interner; it does not create another scene
//! graph or retain runtime tracking state.

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{Ir, ValueR};
use hymeko::resolution::interner::Interner;

use super::{attr_value, child_named, edge_participants_into, find_root, isa_transitive};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub scale: [f64; 3],
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            position_m: [0.0; 3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0; 3],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FiducialKind {
    Aruco,
    Qr,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub relation: DeclId,
    pub entity: DeclId,
    pub frame: DeclId,
    pub pose: Pose,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameParent {
    pub relation: DeclId,
    pub child: DeclId,
    pub parent: DeclId,
    /// Child-frame pose relative to its parent.
    pub pose: Pose,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpatialFrame<'a> {
    pub decl: DeclId,
    pub name: &'a str,
    pub external_frame_id: Option<&'a str>,
}

/// Authored binding between one external camera stream and its optical coordinate frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CameraFrame<'a> {
    pub decl: DeclId,
    pub name: &'a str,
    pub external_frame_id: &'a str,
    pub camera_id: u32,
    pub image_encoding: &'a str,
    pub color_space: &'a str,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiducialBinding<'a> {
    pub relation: DeclId,
    pub tracked_frame: DeclId,
    pub anchor: DeclId,
    pub kind: FiducialKind,
    pub family: &'a str,
    pub identifier: &'a str,
    pub size_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpatialQueryError {
    VocabularyMissing,
    MalformedRelation { relation: DeclId },
    MissingAttribute { decl: DeclId, attr: SymId },
    InvalidAttribute { decl: DeclId, attr: SymId },
}

impl std::fmt::Display for SpatialQueryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for SpatialQueryError {}

struct SpatialRegistry {
    spatial_frame: DeclId,
    camera_frame: DeclId,
    spatial_entity: DeclId,
    fiducial_anchor: DeclId,
    placement: DeclId,
    parent_frame: DeclId,
    fiducial_registration: DeclId,
    position: Option<SymId>,
    orientation: Option<SymId>,
    scale: Option<SymId>,
    external_frame_id: SymId,
    camera_id: SymId,
    image_encoding: SymId,
    color_space: SymId,
    fiducial_kind: SymId,
    family: SymId,
    identifier: SymId,
    size_m: SymId,
}

impl SpatialRegistry {
    fn resolve(ir: &Ir, it: &Interner) -> Option<Self> {
        let spatial = find_root(ir, it, "spatial")?;
        let kinds = child_named(ir, it, spatial, "kinds")?;
        let relations = child_named(ir, it, spatial, "relations")?;
        Some(Self {
            spatial_frame: child_named(ir, it, kinds, "spatial_frame")?,
            camera_frame: child_named(ir, it, kinds, "camera_frame")?,
            spatial_entity: child_named(ir, it, kinds, "spatial_entity")?,
            fiducial_anchor: child_named(ir, it, kinds, "fiducial_anchor")?,
            placement: child_named(ir, it, relations, "placement")?,
            parent_frame: child_named(ir, it, relations, "parent_frame")?,
            fiducial_registration: child_named(ir, it, relations, "fiducial_registration")?,
            position: it.get_id("position"),
            orientation: it.get_id("orientation"),
            scale: it.get_id("scale"),
            external_frame_id: it.get_id("external_frame_id")?,
            camera_id: it.get_id("camera_id")?,
            image_encoding: it.get_id("image_encoding")?,
            color_space: it.get_id("color_space")?,
            fiducial_kind: it.get_id("fiducial_kind")?,
            family: it.get_id("family")?,
            identifier: it.get_id("identifier")?,
            size_m: it.get_id("size_m")?,
        })
    }
}

pub struct SpatialQuery<'a> {
    ir: &'a Ir,
    interner: &'a Interner,
    registry: SpatialRegistry,
}

impl<'a> SpatialQuery<'a> {
    pub fn new(ir: &'a Ir, interner: &'a Interner) -> Result<Self, SpatialQueryError> {
        let registry =
            SpatialRegistry::resolve(ir, interner).ok_or(SpatialQueryError::VocabularyMissing)?;
        Ok(Self {
            ir,
            interner,
            registry,
        })
    }

    pub fn name(&self, decl: DeclId) -> Option<&'a str> {
        self.ir
            .decl_node(decl)
            .map(|node| self.interner.resolve(node.name))
    }

    /// External optical/runtime name assigned to an authored spatial frame.
    pub fn external_frame_id(&self, frame: DeclId) -> Option<&'a str> {
        string_attr(
            self.ir,
            self.interner,
            frame,
            self.registry.external_frame_id,
        )
        .filter(|value| !value.is_empty())
    }

    pub fn frames(&self) -> Vec<SpatialFrame<'a>> {
        self.ir
            .decls()
            .filter(|(decl, _)| {
                *decl != self.registry.spatial_frame
                    && *decl != self.registry.camera_frame
                    && isa_transitive(self.ir, *decl, self.registry.spatial_frame)
            })
            .map(|(decl, node)| SpatialFrame {
                decl,
                name: self.interner.resolve(node.name),
                external_frame_id: self.external_frame_id(decl),
            })
            .collect()
    }

    pub fn camera_frames(&self) -> Result<Vec<CameraFrame<'a>>, SpatialQueryError> {
        self.ir
            .decls()
            .filter(|(decl, _)| {
                *decl != self.registry.camera_frame
                    && isa_transitive(self.ir, *decl, self.registry.camera_frame)
            })
            .map(|(decl, node)| {
                let external_frame_id = required_string(
                    self.ir,
                    self.interner,
                    decl,
                    self.registry.external_frame_id,
                )?;
                let camera_id = required_u32(self.ir, decl, self.registry.camera_id)?;
                let image_encoding =
                    required_string(self.ir, self.interner, decl, self.registry.image_encoding)?;
                let color_space =
                    required_string(self.ir, self.interner, decl, self.registry.color_space)?;
                Ok(CameraFrame {
                    decl,
                    name: self.interner.resolve(node.name),
                    external_frame_id,
                    camera_id,
                    image_encoding,
                    color_space,
                })
            })
            .collect()
    }

    pub fn placements(&self) -> Result<Vec<Placement>, SpatialQueryError> {
        let mut result = Vec::new();
        let mut participants = Vec::new();
        for (relation, _) in self.ir.decls() {
            if relation == self.registry.placement
                || !isa_transitive(self.ir, relation, self.registry.placement)
            {
                continue;
            }
            edge_participants_into(self.ir, relation, &mut participants);
            let entity = exactly_one(&participants, 1)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.spatial_entity));
            let frame = exactly_one(&participants, -1)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.spatial_frame));
            let (Some(entity), Some(frame)) = (entity, frame) else {
                return Err(SpatialQueryError::MalformedRelation { relation });
            };
            result.push(Placement {
                relation,
                entity,
                frame,
                pose: pose_of(self.ir, &self.registry, relation)?,
            });
        }
        Ok(result)
    }

    pub fn frame_parents(&self) -> Result<Vec<FrameParent>, SpatialQueryError> {
        let mut result = Vec::new();
        let mut participants = Vec::new();
        for (relation, _) in self.ir.decls() {
            if relation == self.registry.parent_frame
                || !isa_transitive(self.ir, relation, self.registry.parent_frame)
            {
                continue;
            }
            edge_participants_into(self.ir, relation, &mut participants);
            let child = exactly_one(&participants, 1)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.spatial_frame));
            let parent = exactly_one(&participants, -1)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.spatial_frame));
            let (Some(child), Some(parent)) = (child, parent) else {
                return Err(SpatialQueryError::MalformedRelation { relation });
            };
            result.push(FrameParent {
                relation,
                child,
                parent,
                pose: pose_of(self.ir, &self.registry, relation)?,
            });
        }
        Ok(result)
    }

    pub fn fiducial_bindings(&self) -> Result<Vec<FiducialBinding<'a>>, SpatialQueryError> {
        let mut result = Vec::new();
        let mut participants = Vec::new();
        for (relation, _) in self.ir.decls() {
            if relation == self.registry.fiducial_registration
                || !isa_transitive(self.ir, relation, self.registry.fiducial_registration)
            {
                continue;
            }
            edge_participants_into(self.ir, relation, &mut participants);
            let tracked_frame = exactly_one(&participants, 1)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.spatial_frame));
            let anchor = exactly_one(&participants, 0)
                .filter(|decl| isa_transitive(self.ir, *decl, self.registry.fiducial_anchor));
            let (Some(tracked_frame), Some(anchor)) = (tracked_frame, anchor) else {
                return Err(SpatialQueryError::MalformedRelation { relation });
            };
            let kind_text =
                required_string(self.ir, self.interner, anchor, self.registry.fiducial_kind)?;
            let kind = match kind_text {
                "aruco" => FiducialKind::Aruco,
                "qr" => FiducialKind::Qr,
                _ => {
                    return Err(SpatialQueryError::InvalidAttribute {
                        decl: anchor,
                        attr: self.registry.fiducial_kind,
                    });
                }
            };
            let family = required_string(self.ir, self.interner, anchor, self.registry.family)?;
            let identifier =
                required_string(self.ir, self.interner, anchor, self.registry.identifier)?;
            let size_m = match attr_value(self.ir, anchor, self.registry.size_m) {
                Some(ValueR::Num(value)) if value.is_finite() && *value > 0.0 => *value,
                Some(_) => {
                    return Err(SpatialQueryError::InvalidAttribute {
                        decl: anchor,
                        attr: self.registry.size_m,
                    });
                }
                None => {
                    return Err(SpatialQueryError::MissingAttribute {
                        decl: anchor,
                        attr: self.registry.size_m,
                    });
                }
            };
            result.push(FiducialBinding {
                relation,
                tracked_frame,
                anchor,
                kind,
                family,
                identifier,
                size_m,
            });
        }
        Ok(result)
    }
}

fn exactly_one(participants: &[(i8, DeclId)], sign: i8) -> Option<DeclId> {
    let mut values = participants
        .iter()
        .filter(|(candidate, _)| *candidate == sign)
        .map(|(_, decl)| *decl);
    let first = values.next()?;
    values.next().is_none().then_some(first)
}

fn required_string<'a>(
    ir: &Ir,
    interner: &'a Interner,
    decl: DeclId,
    attr: SymId,
) -> Result<&'a str, SpatialQueryError> {
    match attr_value(ir, decl, attr) {
        Some(ValueR::Str(value)) => {
            let text = interner.resolve(*value);
            if text.is_empty() {
                Err(SpatialQueryError::InvalidAttribute { decl, attr })
            } else {
                Ok(text)
            }
        }
        Some(_) => Err(SpatialQueryError::InvalidAttribute { decl, attr }),
        None => Err(SpatialQueryError::MissingAttribute { decl, attr }),
    }
}

fn required_u32(ir: &Ir, decl: DeclId, attr: SymId) -> Result<u32, SpatialQueryError> {
    match attr_value(ir, decl, attr) {
        Some(ValueR::Num(value))
            if value.is_finite()
                && *value >= 0.0
                && *value <= u32::MAX as f64
                && value.fract() == 0.0 =>
        {
            Ok(*value as u32)
        }
        Some(_) => Err(SpatialQueryError::InvalidAttribute { decl, attr }),
        None => Err(SpatialQueryError::MissingAttribute { decl, attr }),
    }
}

fn string_attr<'a>(ir: &Ir, interner: &'a Interner, decl: DeclId, attr: SymId) -> Option<&'a str> {
    match attr_value(ir, decl, attr) {
        Some(ValueR::Str(value)) => Some(interner.resolve(*value)),
        _ => None,
    }
}

fn pose_of(
    ir: &Ir,
    registry: &SpatialRegistry,
    relation: DeclId,
) -> Result<Pose, SpatialQueryError> {
    Ok(Pose {
        position_m: vector::<3>(ir, relation, registry.position, [0.0; 3])?,
        orientation_xyzw: vector::<4>(ir, relation, registry.orientation, [0.0, 0.0, 0.0, 1.0])?,
        scale: vector::<3>(ir, relation, registry.scale, [1.0; 3])?,
    })
}

fn vector<const N: usize>(
    ir: &Ir,
    decl: DeclId,
    attr: Option<SymId>,
    default: [f64; N],
) -> Result<[f64; N], SpatialQueryError> {
    let Some(attr) = attr else { return Ok(default) };
    let Some(value) = attr_value(ir, decl, attr) else {
        return Ok(default);
    };
    let ValueR::List(items) = value else {
        return Err(SpatialQueryError::InvalidAttribute { decl, attr });
    };
    if items.len() != N {
        return Err(SpatialQueryError::InvalidAttribute { decl, attr });
    }
    let mut values = [0.0; N];
    for (index, value) in items.iter().enumerate() {
        let ValueR::Num(value) = value else {
            return Err(SpatialQueryError::InvalidAttribute { decl, attr });
        };
        if !value.is_finite() {
            return Err(SpatialQueryError::InvalidAttribute { decl, attr });
        }
        values[index] = *value;
    }
    Ok(values)
}

#[cfg(test)]
#[path = "query/tests.rs"]
mod tests;
