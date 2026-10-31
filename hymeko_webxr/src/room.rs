//! Rooms over the spatial vocabulary (`meta_room.hymeko`): a room frame registered by at most one
//! root fiducial, the room's other fiducials as landmarks, and the tracked devices that report
//! poses into it. Everything is bound by kind; no declaration name or path decides a role.
//!
//! Observation slots are the instance's own `sample` child. A slot inherited from a kind is not
//! a slot of the instance: every instance of that kind would share it.

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::{Ir, ValueR};
use hymeko::resolution::interner::Interner;

use super::query::{FiducialKind, SpatialQuery, SpatialQueryError};
use super::{child_named, find_root, isa_transitive};

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

/// An object people may move: `manipulable` kind with its own placement `sample`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Manipulable {
    pub decl: DeclId,
    pub sample: DeclId,
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
    room: DeclId,
    room_frame: DeclId,
    aruco_marker: DeclId,
    tracked_device: DeclId,
    /// Absent from vocabulary copies that predate it; such a model has no manipulable objects.
    manipulable: Option<DeclId>,
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
            room: child_named(ir, it, kinds, "room")?,
            room_frame: child_named(ir, it, kinds, "room_frame")?,
            aruco_marker: child_named(ir, it, kinds, "aruco_marker")?,
            tracked_device: child_named(ir, it, kinds, "tracked_device")?,
            manipulable: child_named(ir, it, kinds, "manipulable"),
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
}

impl<'a> RoomQuery<'a> {
    /// `VocabularyMissing` unless both `meta_spatial` and `meta_room` are part of the model.
    pub fn new(ir: &'a Ir, interner: &'a Interner) -> Result<Self, SpatialQueryError> {
        let spatial = SpatialQuery::new(ir, interner)?;
        let kinds = RoomKinds::resolve(ir, interner).ok_or(SpatialQueryError::VocabularyMissing)?;
        Ok(Self { spatial, kinds })
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

    /// Every manipulable instance with its own `sample`, anywhere in the model. One without its
    /// own slot is refused: a slot inherited from a profile would move every object sharing it.
    pub fn manipulable(&self) -> Result<Vec<Manipulable>, SpatialQueryError> {
        let Some(kind) = self.kinds.manipulable else { return Ok(Vec::new()) };
        // With no `sample` anywhere in the model, the kind's own name names the missing field.
        let attr = self.kinds.sample.or_else(|| Some(self.spatial.ir().decl_node(kind)?.name));
        self.instances(kind)
            .map(|decl| {
                let sample = self.own(decl, self.kinds.sample).ok_or(match attr {
                    Some(attr) => SpatialQueryError::MissingAttribute { decl, attr },
                    None => SpatialQueryError::VocabularyMissing,
                })?;
                Ok(Manipulable { decl, sample })
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

    fn instances(&self, kind: DeclId) -> impl Iterator<Item = DeclId> + '_ {
        self.spatial.instances(kind)
    }

    fn within(&self, decl: DeclId, ancestor: DeclId) -> bool {
        self.spatial.within(decl, ancestor)
    }

    fn own(&self, decl: DeclId, name: Option<SymId>) -> Option<DeclId> {
        self.spatial.own(decl, name)
    }
}

/// One pose observation as a numeric sample, in room axes and metres:
/// `[epoch, sequence, valid, x, y, z, qw, qx, qy, qz]`, optionally followed by
/// `[confidence, observed_unix_ms]`, the quality a surveyed marker reports. The one reader of
/// this layout.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseSample {
    pub epoch: u64,
    pub sequence: u64,
    pub valid: bool,
    pub position_m: [f64; 3],
    /// Unit quaternion, w first as the sample writes it.
    pub orientation_wxyz: [f64; 4],
    pub confidence: Option<f64>,
    pub observed_ms: Option<u64>,
}

/// Why a numeric list is not a pose sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoseSampleError {
    /// Neither 10 nor 12 entries, or an entry that is not a number.
    Layout,
    /// An entry is NaN or infinite.
    NonFinite,
    /// Epoch or sequence is not an exact integer in `0..=2^53-1`.
    Identity,
    /// Validity is neither 0 nor 1.
    Validity,
    /// A coordinate lies beyond 1000 m.
    Bounds,
    /// The quaternion is not unit length within 1e-3.
    Quaternion,
    /// Confidence outside `[0, 1]`, or an observation time that is not an exact integer in
    /// `0..=2^53-1`.
    Quality,
}

/// The largest integer every JavaScript and JSON reader holds exactly.
const EXACT_INTEGER: f64 = 9_007_199_254_740_991.0;

fn exact_integer(value: f64) -> bool {
    (0.0..=EXACT_INTEGER).contains(&value) && value.fract() == 0.0
}

impl PoseSample {
    /// Encode the same validated numeric layout read by `read`. Optional quality is a pair.
    pub fn to_values(&self) -> Result<Vec<f64>, PoseSampleError> {
        if self.epoch > EXACT_INTEGER as u64 || self.sequence > EXACT_INTEGER as u64 {
            return Err(PoseSampleError::Identity);
        }
        let mut values = vec![self.epoch as f64, self.sequence as f64, f64::from(self.valid)];
        values.extend(self.position_m);
        values.extend(self.orientation_wxyz);
        match (self.confidence, self.observed_ms) {
            (None, None) => {},
            (Some(confidence), Some(time)) if time <= EXACT_INTEGER as u64 => values.extend([confidence, time as f64]),
            _ => return Err(PoseSampleError::Quality),
        }
        Self::from_values(&values)?;
        Ok(values)
    }
    /// Encode an IR numeric list through the same validated layout.
    pub fn to_value(&self) -> Result<ValueR, PoseSampleError> {
        Ok(ValueR::List(self.to_values()?.into_iter().map(ValueR::Num).collect()))
    }
    /// Read a sample from its numbers.
    pub fn from_values(v: &[f64]) -> Result<Self, PoseSampleError> {
        if !matches!(v.len(), 10 | 12) {
            return Err(PoseSampleError::Layout);
        }
        if v.iter().any(|n| !n.is_finite()) {
            return Err(PoseSampleError::NonFinite);
        }
        if !exact_integer(v[0]) || !exact_integer(v[1]) {
            return Err(PoseSampleError::Identity);
        }
        if v[2] != 0.0 && v[2] != 1.0 {
            return Err(PoseSampleError::Validity);
        }
        if v[3..6].iter().any(|n| n.abs() > 1000.0) {
            return Err(PoseSampleError::Bounds);
        }
        if (v[6..10].iter().map(|n| n * n).sum::<f64>() - 1.0).abs() > 1e-3 {
            return Err(PoseSampleError::Quaternion);
        }
        let quality = v.get(10..12);
        if quality.is_some_and(|q| !(0.0..=1.0).contains(&q[0]) || !exact_integer(q[1])) {
            return Err(PoseSampleError::Quality);
        }
        Ok(Self {
            epoch: v[0] as u64,
            sequence: v[1] as u64,
            valid: v[2] == 1.0,
            position_m: [v[3], v[4], v[5]],
            orientation_wxyz: [v[6], v[7], v[8], v[9]],
            confidence: quality.map(|q| q[0]),
            observed_ms: quality.map(|q| q[1] as u64),
        })
    }

    /// Read a sample from an IR value: a list of numbers.
    pub fn read(value: &ValueR) -> Result<Self, PoseSampleError> {
        let ValueR::List(items) = value else { return Err(PoseSampleError::Layout) };
        let mut numbers = [0.0; 12];
        if items.len() > numbers.len() {
            return Err(PoseSampleError::Layout);
        }
        for (out, item) in numbers.iter_mut().zip(items) {
            let ValueR::Num(n) = item else { return Err(PoseSampleError::Layout) };
            *out = *n;
        }
        Self::from_values(&numbers[..items.len()])
    }
}

#[cfg(test)]
mod sample_encoding {
    use super::*;
    #[test]
    fn writer_round_trips_both_layouts_and_rejects_lossy_identity_and_partial_quality() {
        let mut sample = PoseSample::from_values(&[1.,2.,1.,0.2,0.,0.,1.,0.,0.,0.]).unwrap();
        assert_eq!(PoseSample::read(&sample.to_value().unwrap()).unwrap(), sample);
        assert_eq!(PoseSample::from_values(&sample.to_values().unwrap()).unwrap(), sample);
        sample.confidence = Some(0.75);
        assert_eq!(sample.to_value(), Err(PoseSampleError::Quality));
        sample.observed_ms = Some(1790000000000);
        assert_eq!(PoseSample::read(&sample.to_value().unwrap()).unwrap(), sample);
        sample.sequence = 9_007_199_254_740_993;
        assert_eq!(sample.to_value(), Err(PoseSampleError::Identity));
    }
}

#[cfg(test)]
#[path = "room/tests.rs"]
mod tests;
