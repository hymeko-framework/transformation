//! Rooms over the spatial vocabulary (`meta_room.hymeko`): a room frame registered by at most one
//! root fiducial, the room's other fiducials as landmarks, and the tracked devices that report
//! poses into it. Everything is bound by kind; no declaration name or path decides a role.
//!
//! Observation slots are the instance's own `sample` child. A slot inherited from a kind is not
//! a slot of the instance: every instance of that kind would share it.

use std::collections::HashSet;

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::Ir;
use hymeko::resolution::interner::Interner;

use super::query::{FiducialKind, SpatialQuery, SpatialQueryError};
use super::{MAX_WALK_DEPTH, child_named, each_base, find_root, isa_transitive};

/// An ArUco fiducial: dictionary family, decimal marker id and physical side length.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArucoMarker<'a> {
    pub decl: DeclId,
    pub family: &'a str,
    pub id: u32,
    pub size_m: f64,
    pub convention: &'a str,
    pub sample: Option<DeclId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TrackedDevice<'a> {
    pub decl: DeclId,
    pub transport: Option<&'a str>,
    pub sample: Option<DeclId>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Room<'a> {
    pub decl: DeclId,
    pub frame: DeclId,
    /// The fiducial that registers the room frame, if the room declares one.
    pub root: Option<ArucoMarker<'a>>,
    /// Every other ArUco fiducial inside the room.
    pub landmarks: Vec<ArucoMarker<'a>>,
    pub devices: Vec<TrackedDevice<'a>>,
}

struct RoomKinds {
    /// Vocabulary roots; declarations inside them are kinds, not instances.
    vocabularies: [DeclId; 2],
    room: DeclId,
    room_frame: DeclId,
    aruco_marker: DeclId,
    tracked_device: DeclId,
    sample: Option<SymId>,
    transport: Option<SymId>,
    convention: SymId,
    fiducial_kind: SymId,
    identifier: SymId,
}

impl RoomKinds {
    fn resolve(ir: &Ir, it: &Interner) -> Option<Self> {
        let root = find_root(ir, it, "xr_room")?;
        let kinds = child_named(ir, it, root, "kinds")?;
        Some(Self {
            vocabularies: [root, find_root(ir, it, "spatial")?],
            room: child_named(ir, it, kinds, "room")?,
            room_frame: child_named(ir, it, kinds, "room_frame")?,
            aruco_marker: child_named(ir, it, kinds, "aruco_marker")?,
            tracked_device: child_named(ir, it, kinds, "tracked_device")?,
            sample: it.get_id("sample"),
            transport: it.get_id("transport"),
            convention: it.get_id("convention")?,
            fiducial_kind: it.get_id("fiducial_kind")?,
            identifier: it.get_id("identifier")?,
        })
    }
}

pub struct RoomQuery<'a> {
    spatial: SpatialQuery<'a>,
    kinds: RoomKinds,
    /// Declarations some other declaration inherits from: kinds and shared profiles.
    bases: HashSet<DeclId>,
}

impl<'a> RoomQuery<'a> {
    /// `VocabularyMissing` unless both `meta_spatial` and `meta_room` are part of the model.
    pub fn new(ir: &'a Ir, interner: &'a Interner) -> Result<Self, SpatialQueryError> {
        let spatial = SpatialQuery::new(ir, interner)?;
        let kinds = RoomKinds::resolve(ir, interner).ok_or(SpatialQueryError::VocabularyMissing)?;
        let mut bases = HashSet::new();
        for (decl, _) in ir.decls() {
            each_base(ir, decl, &mut |base| { bases.insert(base); });
        }
        Ok(Self { spatial, kinds, bases })
    }

    pub fn spatial(&self) -> &SpatialQuery<'a> {
        &self.spatial
    }

    /// Every ArUco fiducial of the model, inside or outside a room.
    pub fn markers(&self) -> Result<Vec<ArucoMarker<'a>>, SpatialQueryError> {
        self.instances(self.kinds.aruco_marker)
            .map(|decl| self.marker(decl))
            .collect()
    }

    pub fn rooms(&self) -> Result<Vec<Room<'a>>, SpatialQueryError> {
        let registrations = self.spatial.fiducial_bindings()?;
        self.instances(self.kinds.room)
            .map(|room| {
                let frames: Vec<_> = self.instances(self.kinds.room_frame)
                    .filter(|frame| self.within(*frame, room))
                    .collect();
                let [frame] = frames[..] else {
                    return Err(SpatialQueryError::RoomFrame { room, found: frames.len() as u32 });
                };
                let roots: Vec<_> = registrations.iter().filter(|r| r.tracked_frame == frame).collect();
                let root = match roots[..] {
                    [] => None,
                    [registration] => Some(self.marker(registration.anchor)?),
                    _ => return Err(SpatialQueryError::RoomRoot { room, found: roots.len() as u32 }),
                };
                let landmarks = self.instances(self.kinds.aruco_marker)
                    .filter(|marker| self.within(*marker, room) && root.is_none_or(|r| r.decl != *marker))
                    .map(|marker| self.marker(marker))
                    .collect::<Result<_, _>>()?;
                let devices = self.instances(self.kinds.tracked_device)
                    .filter(|device| self.within(*device, room))
                    .map(|decl| TrackedDevice {
                        decl,
                        transport: self.kinds.transport.and_then(|t| self.spatial.string_attr(decl, t)),
                        sample: self.own(decl, self.kinds.sample),
                    })
                    .collect();
                Ok(Room { decl: room, frame, root, landmarks, devices })
            })
            .collect()
    }

    /// An ArUco marker instance: `aruco_marker` kind, `fiducial_kind "aruco"`, decimal identifier.
    fn marker(&self, decl: DeclId) -> Result<ArucoMarker<'a>, SpatialQueryError> {
        let (kind, family, identifier, size_m) = self.spatial.fiducial(decl)?;
        let invalid = |attr| SpatialQueryError::InvalidAttribute { decl, attr };
        if kind != FiducialKind::Aruco || !isa_transitive(self.spatial.ir(), decl, self.kinds.aruco_marker) {
            return Err(invalid(self.kinds.fiducial_kind));
        }
        let id = identifier.parse::<u32>().map_err(|_| invalid(self.kinds.identifier))?;
        Ok(ArucoMarker {
            decl,
            family,
            id,
            size_m,
            convention: self.spatial.required_string(decl, self.kinds.convention)?,
            sample: self.own(decl, self.kinds.sample),
        })
    }

    /// The model's instances of `kind`: not inside a vocabulary root and not itself inherited
    /// from (a declaration others inherit from is a kind or a shared profile, not an instance).
    fn instances(&self, kind: DeclId) -> impl Iterator<Item = DeclId> + '_ {
        let ir = self.spatial.ir();
        ir.decls().map(|(decl, _)| decl).filter(move |decl| {
            isa_transitive(ir, *decl, kind)
                && !self.bases.contains(decl)
                && !self.kinds.vocabularies.iter().any(|root| self.within(*decl, *root))
        })
    }

    /// `decl` is `ancestor` or nested below it (bounded parent walk).
    fn within(&self, decl: DeclId, ancestor: DeclId) -> bool {
        let ir = self.spatial.ir();
        let mut current = Some(decl);
        for _ in 0..MAX_WALK_DEPTH {
            match current {
                Some(d) if d == ancestor => return true,
                Some(d) => current = ir.parent_of(d),
                None => return false,
            }
        }
        false
    }

    /// The declaration's own child `name`, never an inherited one.
    fn own(&self, decl: DeclId, name: Option<SymId>) -> Option<DeclId> {
        let ir = self.spatial.ir();
        let name = name?;
        ir.child_decls(decl).find(|child| ir.decl_node(*child).is_some_and(|node| node.name == name))
    }
}

#[cfg(test)]
#[path = "room/tests.rs"]
mod tests;
