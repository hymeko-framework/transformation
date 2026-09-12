//! Two-attachment force transmissions projected from resolved HyMeKo incidences.
//! These attach to the existing body model; they introduce no pose tree or IR.
use hymeko::common::ids::DeclId;
use hymeko::ir::ir::{Ir, ValueR};
use hymeko_query::{Predicate, QueryEngine, traits::NameResolver};

use super::{joints::JointInfo, kinematic::LinkInfo};

#[derive(Clone, Debug)]
pub struct Attachment {
    pub did: DeclId,
    pub name: String,
    pub link: DeclId,
    pub position: [f64; 3],
    /// Optional spherical tendon guide, rigidly attached to this link. Mass is part of the link inertia.
    pub radius_m: Option<f64>,
}

#[derive(Clone, Debug)]
pub enum DriveLaw {
    /// MuJoCo-compatible Hill-type force-length/velocity law with activation dynamics.
    Muscle {
        force: f64,
        activation_s: f64,
        deactivation_s: f64,
    },
    /// Equal-area double-acting cylinder: filtered differential pressure times area.
    /// No fluid circuit, valve flow or bulk-modulus model is implied.
    Hydraulic {
        area_m2: f64,
        pressure_pa: f64,
        response_s: f64,
        damping_ns_m: f64,
    },
}

#[derive(Clone, Debug)]
pub struct LinearDrive {
    pub did: DeclId,
    pub name: String,
    pub origin: DeclId,
    pub insertion: DeclId,
    /// Optional spherical guide and its preferred-side attachment, both resolved declarations.
    pub wrapping: Option<[DeclId; 2]>,
    /// Explicit control coordinate; geometry, not this reference, determines the moment arm.
    pub joint: DeclId,
    pub length_range_m: [f64; 2],
    pub radius_m: f64,
    pub law: DriveLaw,
}

#[derive(Clone, Debug, Default)]
pub struct Actuation {
    pub attachments: Vec<Attachment>,
    pub drives: Vec<LinearDrive>,
    /// Malformed authored connections must fail projection, never disappear silently.
    pub errors: Vec<String>,
}

impl Actuation {
    pub(super) fn extract<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        root: &str,
        links: &[LinkInfo],
        joints: &[JointInfo],
    ) -> Self {
        let ir = engine.ir();
        let res = engine.resolver();
        let mut out = Self::default();
        for found in engine.query(&Predicate::node().and(Predicate::inherits("attachment"))) {
            let parent = ir.decl_nodes[found.id.raw()].parent;
            if !links.iter().any(|l| l.did == parent) {
                continue;
            }
            let fields = Fields {
                ir,
                res,
                did: found.id,
            };
            let result = (|| -> Result<Attachment, String> {
                Ok(Attachment {
                    did: found.id,
                    name: format!(
                        "{}.{}",
                        links.iter().find(|l| l.did == parent).unwrap().name,
                        found.name
                    ),
                    link: parent,
                    position: fields.xyz()?,
                    radius_m: if fields.has("radius_m") {
                        Some(fields.positive("radius_m")?)
                    } else {
                        None
                    },
                })
            })();
            match result {
                Ok(attachment) => out.attachments.push(attachment),
                Err(error) => out
                    .errors
                    .push(format!("attachment {}: {error}", found.name)),
            }
        }
        for found in engine.query(&Predicate::edge().and(Predicate::inherits("linear_drive"))) {
            let parent = ir.decl_nodes[found.id.raw()].parent;
            if parent.is_none() || res.resolve(ir.decl_nodes[parent.raw()].name) != root {
                continue;
            }
            let result = (|| -> Result<LinearDrive, String> {
                let origins: Vec<_> = found.arc_bindings.iter().filter(|b| b.sign == 1).collect();
                let insertions: Vec<_> =
                    found.arc_bindings.iter().filter(|b| b.sign == -1).collect();
                if origins.len() != 1 || insertions.len() != 1 || found.arc_bindings.len() != 2 {
                    return Err(
                        "expected exactly one + origin and one - insertion attachment".into(),
                    );
                }
                let origin = origins[0].target;
                let insertion = insertions[0].target;
                for id in [origin, insertion] {
                    if !out.attachments.iter().any(|a| a.did == id) {
                        return Err("force endpoint is not a valid attachment on this robot".into());
                    }
                }
                if origin == insertion {
                    return Err("force endpoints must differ".into());
                }
                let fields = Fields {
                    ir,
                    res,
                    did: found.id,
                };
                let wrapping = if fields.has("wrap") || fields.has("side") {
                    let guide_id = fields.reference("wrap")?;
                    let side_id = fields.reference("side")?;
                    let guide = out
                        .attachments
                        .iter()
                        .find(|a| a.did == guide_id)
                        .ok_or("wrap must reference a spherical attachment")?;
                    let radius = guide.radius_m.ok_or("wrap attachment requires radius_m")?;
                    let side = out
                        .attachments
                        .iter()
                        .find(|a| a.did == side_id)
                        .ok_or("side must reference an attachment")?;
                    if guide.link != side.link
                        || guide
                            .position
                            .iter()
                            .zip(side.position)
                            .map(|(a, b)| (a - b).powi(2))
                            .sum::<f64>()
                            <= radius * radius
                    {
                        return Err(
                            "preferred side must lie outside the guide on the same link".into()
                        );
                    }
                    Some([guide_id, side_id])
                } else {
                    None
                };
                let joint = fields.reference("joint")?;
                if !joints.iter().any(|j| j.did == joint) {
                    return Err("control joint is outside the robot".into());
                }
                let parameters = Fields {
                    ir,
                    res,
                    did: fields.reference("parameters")?,
                };
                let length_range_m = [
                    parameters.positive("length_min_m")?,
                    parameters.positive("length_max_m")?,
                ];
                if length_range_m[0] >= length_range_m[1] {
                    return Err("empty length range".into());
                }
                let law = match parameters.text("law")? {
                    "muscle" => DriveLaw::Muscle {
                        force: parameters.positive("force_n")?,
                        activation_s: parameters.positive("activation_s")?,
                        deactivation_s: parameters.positive("deactivation_s")?,
                    },
                    "hydraulic" => DriveLaw::Hydraulic {
                        area_m2: parameters.positive("area_m2")?,
                        pressure_pa: parameters.positive("pressure_pa")?,
                        response_s: parameters.positive("response_s")?,
                        damping_ns_m: parameters.positive("damping_ns_m")?,
                    },
                    other => return Err(format!("unknown drive law {other}")),
                };
                Ok(LinearDrive {
                    did: found.id,
                    name: found.name.clone(),
                    origin,
                    insertion,
                    wrapping,
                    joint,
                    length_range_m,
                    radius_m: parameters.positive("radius_m")?,
                    law,
                })
            })();
            match result {
                Ok(drive) => out.drives.push(drive),
                Err(error) => out.errors.push(format!("drive {}: {error}", found.name)),
            }
        }
        out
    }
}

pub(crate) struct Fields<'a, R> {
    pub(crate) ir: &'a Ir,
    pub(crate) res: &'a R,
    pub(crate) did: DeclId,
}
impl<R: NameResolver> Fields<'_, R> {
    pub(crate) fn has(&self, key: &str) -> bool {
        self.ir
            .decl_children(self.did)
            .any(|c| self.res.resolve(self.ir.decl_nodes[c.raw()].name) == key)
    }
    pub(crate) fn value(&self, key: &str) -> Result<&ValueR, String> {
        self.ir
            .decl_children(self.did)
            .find(|c| self.res.resolve(self.ir.decl_nodes[c.raw()].name) == key)
            .and_then(|c| self.ir.decl_nodes[c.raw()].anno.value.as_ref())
            .ok_or_else(|| format!("missing {key}"))
    }
    pub(crate) fn number(&self, key: &str) -> Result<f64, String> {
        match self.value(key)? {
            ValueR::Num(n) if n.is_finite() => Ok(*n),
            _ => Err(format!("{key} must be finite")),
        }
    }
    pub(crate) fn positive(&self, key: &str) -> Result<f64, String> {
        let n = self.number(key)?;
        if n > 0.0 {
            Ok(n)
        } else {
            Err(format!("{key} must be positive"))
        }
    }
    pub(crate) fn integer(&self, key: &str, lo: u32, hi: u32) -> Result<u32, String> {
        let n = self.number(key)?;
        if n.fract() == 0.0 && n >= lo as f64 && n <= hi as f64 {
            Ok(n as u32)
        } else {
            Err(format!("invalid {key}"))
        }
    }
    pub(crate) fn array<const N: usize>(&self, key: &str) -> Result<[f64; N], String> {
        let mut value = self.value(key)?;
        if let ValueR::Ref(target) = value {
            value = self
                .ir
                .decl_nodes
                .get(target.raw())
                .and_then(|d| d.anno.value.as_ref())
                .ok_or_else(|| format!("{key} references no value"))?;
        }
        let ValueR::List(values) = value else {
            return Err(format!("{key} must be a numeric vector"));
        };
        if values.len() != N {
            return Err(format!("{key} requires {N} components"));
        }
        let mut result = [0.0; N];
        for (out, value) in result.iter_mut().zip(values) {
            match value {
                ValueR::Num(n) if n.is_finite() => *out = *n,
                _ => return Err(format!("{key} must be finite")),
            }
        }
        Ok(result)
    }
    fn xyz(&self) -> Result<[f64; 3], String> {
        Ok([self.number("x")?, self.number("y")?, self.number("z")?])
    }
    pub(crate) fn text(&self, key: &str) -> Result<&str, String> {
        match self.value(key)? {
            ValueR::Str(s) => Ok(self.res.resolve(*s)),
            _ => Err(format!("{key} must be text")),
        }
    }
    pub(crate) fn reference(&self, key: &str) -> Result<DeclId, String> {
        match self.value(key)? {
            ValueR::Ref(d) if !d.is_none() => Ok(*d),
            _ => Err(format!("{key} must reference a declaration")),
        }
    }
}
