//! Drawn visuals and declared lights: instances of `meta_spatial`'s `visual` and `light` kinds.
//! Everything is bound by kind; no declaration name decides a role.
//!
//! A visual's template is read as effective fields (its own value, else its kind's or profile's):
//! primitive `kind`, size `sx sy sz`, colour `r g b a`, position `px py pz`, orientation
//! `qw qx qy qz`, and optional `mesh_asset`, `mesh_group`, `mesh_fallback`, `name`, `role`. A number
//! may be written as a number or as a numeric string. An absent number is 0, except alpha (1), and an
//! absent or all-zero orientation is the identity.
//!
//! The placement comes from the pose source: the visual itself, or the visual its own `pose_source`
//! references, so the parts of one object share one observed pose (alpha follows the source too).
//! A pose source may follow one observation through its own `pose_sample` (live) or `pose_anchor`
//! (surveyed) reference to a numeric [`PoseSample`] leaf. These bindings are the declaration's own
//! children, never inherited ones: an inherited binding would move every instance sharing it.

use hymeko::common::ids::{DeclId, SymId};
use hymeko::ir::ir::ValueR;

use super::isa_transitive;
use super::query::{SpatialQuery, SpatialQueryError};
use super::room::PoseSample;

/// A drawn entity, with its placement already taken from its pose source.
#[derive(Clone, Debug, PartialEq)]
pub struct Visual<'a> {
    pub decl: DeclId,
    /// The primitive to draw (`Box`, `Sphere`, ...), as declared.
    pub shape: Option<&'a str>,
    pub size: [f64; 3],
    pub rgb: [f64; 3],
    /// The pose source's alpha.
    pub alpha: f64,
    /// The visual whose placement this one uses: `decl` itself unless it declares `pose_source`.
    pub pose_source: DeclId,
    pub position_m: [f64; 3],
    pub orientation_wxyz: [f64; 4],
    /// The observation the pose source follows, if any.
    pub observation: Option<Observation>,
    pub mesh: Option<&'a str>,
    pub mesh_group: Option<&'a str>,
    pub mesh_fallback: Option<&'a str>,
    pub name: Option<&'a str>,
    pub role: Option<&'a str>,
}

/// A pose source's observation: the sample leaf it references and that leaf's value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Observation {
    /// A live tracked sample, fresh only while its sequence advances.
    Live { leaf: DeclId, sample: PoseSample },
    /// A surveyed rigid anchor; it persists without new samples.
    Anchor { leaf: DeclId, sample: PoseSample },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightKind {
    Directional,
    Point,
    Spot,
}

/// A declared light, in glTF `KHR_lights_punctual` terms.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeclaredLight {
    pub decl: DeclId,
    pub kind: LightKind,
    /// Linear colour; `None` when undeclared, which means white.
    pub rgb: Option<[f64; 3]>,
    /// Lux for a directional light, candela for point and spot lights.
    pub intensity: f64,
    pub range: f64,
    /// Spot cone half-angles in radians.
    pub inner: f64,
    pub outer: f64,
    pub casts_shadow: bool,
    pub position_m: [f64; 3],
    /// The light aims along the local -Z of this orientation.
    pub orientation_wxyz: [f64; 4],
}

impl<'a> SpatialQuery<'a> {
    /// Every drawn visual, in declaration order.
    pub fn visuals(&self) -> Result<Vec<Visual<'a>>, SpatialQueryError> {
        let Some(kind) = self.visual_kind() else {
            return Ok(Vec::new());
        };
        self.instances(kind)
            .map(|decl| self.visual(decl, kind))
            .collect()
    }

    /// Every declared light, in declaration order.
    pub fn lights(&self) -> Result<Vec<DeclaredLight>, SpatialQueryError> {
        let Some(kind) = self.light_kind() else {
            return Ok(Vec::new());
        };
        self.instances(kind)
            .map(|decl| {
                let kind = match self.text(decl, "light_kind") {
                    Some("directional") => LightKind::Directional,
                    Some("point") => LightKind::Point,
                    Some("spot") => LightKind::Spot,
                    _ => return Err(self.invalid(decl, "light_kind")),
                };
                let (position_m, orientation_wxyz) = self.placement(decl)?;
                Ok(DeclaredLight {
                    decl,
                    kind,
                    rgb: match self.value(decl, "r") {
                        Some(_) => Some(self.numbers(decl, ["r", "g", "b"])?),
                        None => None,
                    },
                    intensity: self.number(decl, "intensity", 0.0)?,
                    range: self.number(decl, "range", 0.0)?,
                    inner: self.number(decl, "inner", 0.0)?,
                    outer: self.number(decl, "outer", 0.0)?,
                    casts_shadow: self.number(decl, "casts_shadow", 0.0)? >= 0.5,
                    position_m,
                    orientation_wxyz,
                })
            })
            .collect()
    }

    fn visual(&self, decl: DeclId, kind: DeclId) -> Result<Visual<'a>, SpatialQueryError> {
        let source = match self.reference(decl, "pose_source")? {
            None => decl,
            Some(target)
                if isa_transitive(self.ir(), target, kind)
                    && self.own(target, self.sym("pose_source")).is_none() =>
            {
                target
            }
            Some(_) => return Err(self.invalid(decl, "pose_source")),
        };
        let observation = match (
            self.reference(source, "pose_sample")?,
            self.reference(source, "pose_anchor")?,
        ) {
            (None, None) => None,
            (Some(leaf), None) => Some(Observation::Live {
                leaf,
                sample: self.sample(leaf)?,
            }),
            (None, Some(leaf)) => Some(Observation::Anchor {
                leaf,
                sample: self.sample(leaf)?,
            }),
            (Some(_), Some(_)) => return Err(self.invalid(source, "pose_anchor")),
        };
        let (position_m, orientation_wxyz) = self.placement(source)?;
        Ok(Visual {
            decl,
            shape: self.text(decl, "kind"),
            size: self.numbers(decl, ["sx", "sy", "sz"])?,
            rgb: self.numbers(decl, ["r", "g", "b"])?,
            alpha: self.number(source, "a", 1.0)?,
            pose_source: source,
            position_m,
            orientation_wxyz,
            observation,
            mesh: self.text(decl, "mesh_asset"),
            mesh_group: self.text(decl, "mesh_group"),
            mesh_fallback: self.text(decl, "mesh_fallback"),
            name: self.text(decl, "name"),
            role: self.text(decl, "role"),
        })
    }

    /// Position and orientation from the flat template.
    fn placement(&self, decl: DeclId) -> Result<([f64; 3], [f64; 4]), SpatialQueryError> {
        let q = self.numbers(decl, ["qw", "qx", "qy", "qz"])?;
        let identity = q.iter().all(|v| *v == 0.0);
        Ok((
            self.numbers(decl, ["px", "py", "pz"])?,
            if identity { [1.0, 0.0, 0.0, 0.0] } else { q },
        ))
    }

    /// The pose sample a leaf holds.
    fn sample(&self, leaf: DeclId) -> Result<PoseSample, SpatialQueryError> {
        let value = self
            .ir()
            .decl_node(leaf)
            .and_then(|node| node.anno.value.as_ref());
        let value = value.ok_or(SpatialQueryError::InvalidSample {
            leaf,
            reason: super::room::PoseSampleError::Layout,
        })?;
        PoseSample::read(value).map_err(|reason| SpatialQueryError::InvalidSample { leaf, reason })
    }

    /// The declaration the own child `name` references (`name -> target;`).
    fn reference(&self, decl: DeclId, name: &str) -> Result<Option<DeclId>, SpatialQueryError> {
        let Some(child) = self.own(decl, self.sym(name)) else {
            return Ok(None);
        };
        match self
            .ir()
            .decl_node(child)
            .and_then(|node| node.anno.value.as_ref())
        {
            Some(ValueR::Ref(target)) if self.ir().decl_node(*target).is_some() => {
                Ok(Some(*target))
            }
            _ => Err(self.invalid(decl, name)),
        }
    }

    fn numbers<const N: usize>(
        &self,
        decl: DeclId,
        names: [&str; N],
    ) -> Result<[f64; N], SpatialQueryError> {
        let mut values = [0.0; N];
        for (out, name) in values.iter_mut().zip(names) {
            *out = self.number(decl, name, 0.0)?;
        }
        Ok(values)
    }

    /// An effective number, written as a number or a numeric string.
    fn number(&self, decl: DeclId, name: &str, default: f64) -> Result<f64, SpatialQueryError> {
        let number = match self.value(decl, name) {
            None => return Ok(default),
            Some(ValueR::Num(n)) => Some(*n),
            Some(ValueR::Str(s)) => self.interner().resolve(*s).trim().parse().ok(),
            Some(_) => None,
        };
        number
            .filter(|n: &f64| n.is_finite())
            .ok_or_else(|| self.invalid(decl, name))
    }

    fn text(&self, decl: DeclId, name: &str) -> Option<&'a str> {
        match self.value(decl, name) {
            Some(ValueR::Str(s)) => Some(self.interner().resolve(*s)),
            _ => None,
        }
    }

    fn sym(&self, name: &str) -> Option<SymId> {
        self.interner().get_id(name)
    }

    fn invalid(&self, decl: DeclId, name: &str) -> SpatialQueryError {
        self.sym(name)
            .map_or(SpatialQueryError::VocabularyMissing, |attr| {
                SpatialQueryError::InvalidAttribute { decl, attr }
            })
    }
}

#[cfg(test)]
#[path = "visual/tests.rs"]
mod tests;
