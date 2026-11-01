//! Force transmissions projected from resolved HyMeKo incidences: two-attachment drives, and
//! hydraulic braided muscles whose path runs through ordered via points.
//! These attach to the existing body model; they introduce no pose tree or IR. An attachment is
//! either a child of its link or a `mount` hyperedge onto it (an actuation layer that leaves a
//! shared body unedited); a drive belongs to a robot when declared under its root or when its
//! path runs between that robot's attachments.
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
    /// Hydraulic braided (McKibben-type) muscle: the pressurised sleeve is the contractile element,
    /// in series with a tendon spring and in parallel with a passive spring and damper (the
    /// three-element Hill arrangement). The valve lags the commanded pressure by `response_s`.
    /// Pull only. Simulated by Kawakern (`engine::muscle`); MJCF has no such element.
    HydraulicMuscle {
        pressure_pa: f64,
        response_s: f64,
        sleeve: BraidedSleeve,
        tendon_stiffness_n_m: f64,
        tendon_slack_m: f64,
        parallel_stiffness_n_m: f64,
        parallel_rest_m: f64,
        damping_ns_m: f64,
    },
}

/// The declared geometry of a braided muscle sleeve. Inextensible threads of length `b` wound
/// `n` turns fix the sleeve's shape by its length alone: at length `x`, `cos(theta) = x / b`
/// and the diameter is `b sin(theta) / (n pi)`. Pressure shortens the sleeve and so widens it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BraidedSleeve {
    pub braid_length_m: f64,
    pub turns: f64,
}

impl BraidedSleeve {
    /// Braid angle `theta` at sleeve length `length` (clamped to the braid's reach).
    pub fn braid_angle(&self, length: f64) -> f64 {
        (length / self.braid_length_m).clamp(0.0, 1.0).acos()
    }

    /// Sleeve diameter at length `length`.
    pub fn diameter(&self, length: f64) -> f64 {
        self.braid_length_m * self.braid_angle(length).sin() / (self.turns * std::f64::consts::PI)
    }

    /// Pulling force per unit pressure, `b^2 / (4 pi n^2) (3 cos^2 theta - 1)` (Chou and
    /// Hannaford's ideal braided muscle), zero once the braid passes `acos(1 / sqrt 3)`.
    pub fn force_per_pressure(&self, length: f64) -> f64 {
        let c = (length / self.braid_length_m).clamp(0.0, 1.0);
        let b = self.braid_length_m;
        (b * b / (4.0 * std::f64::consts::PI * self.turns * self.turns) * (3.0 * c * c - 1.0)).max(0.0)
    }

    /// Shortest length at which the sleeve still pulls: `b / sqrt 3`.
    pub fn shortest_pulling_length(&self) -> f64 {
        self.braid_length_m / 3f64.sqrt()
    }
}

#[derive(Clone, Debug)]
pub struct LinearDrive {
    pub did: DeclId,
    pub name: String,
    pub origin: DeclId,
    /// Ordered via points between origin and insertion (hydraulic muscles only); empty otherwise.
    pub via: Vec<DeclId>,
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
        // Mount hyperedges: one + arc onto a link of this robot, the point as its first weight.
        for found in engine.query(&Predicate::edge().and(Predicate::inherits("mount"))) {
            let Some(link) = found
                .arc_bindings
                .iter()
                .find_map(|b| links.iter().find(|l| l.did == b.target))
            else {
                continue; // another robot's mount
            };
            let fields = Fields {
                ir,
                res,
                did: found.id,
            };
            let result = (|| -> Result<Attachment, String> {
                let [arc] = found.arc_bindings.as_slice() else {
                    return Err("a mount has exactly one + arc onto its link".into());
                };
                if arc.sign != 1 {
                    return Err("a mount has exactly one + arc onto its link".into());
                }
                let point = arc
                    .weights
                    .as_ref()
                    .and_then(|w| w.first())
                    .ok_or("a mount needs its point in the link frame as the arc's weight")?;
                let ValueR::List(values) = point else {
                    return Err("a mount point must be [x, y, z]".into());
                };
                let numbers: Vec<f64> = values
                    .iter()
                    .filter_map(|v| match v {
                        ValueR::Num(n) if n.is_finite() => Some(*n),
                        _ => None,
                    })
                    .collect();
                let [x, y, z] = numbers[..] else {
                    return Err("a mount point must be three finite numbers".into());
                };
                Ok(Attachment {
                    did: found.id,
                    name: format!("{}.{}", link.name, found.name),
                    link: link.did,
                    position: [x, y, z],
                    radius_m: if fields.has("radius_m") {
                        Some(fields.positive("radius_m")?)
                    } else {
                        None
                    },
                })
            })();
            match result {
                Ok(attachment) => out.attachments.push(attachment),
                Err(error) => out.errors.push(format!("mount {}: {error}", found.name)),
            }
        }
        for found in engine.query(&Predicate::edge().and(Predicate::inherits("linear_drive"))) {
            let parent = ir.decl_nodes[found.id.raw()].parent;
            let under_root = !parent.is_none() && res.resolve(ir.decl_nodes[parent.raw()].name) == root;
            let touches = found
                .arc_bindings
                .iter()
                .any(|b| out.attachments.iter().any(|a| a.did == b.target));
            // Declared under the root, or a layer's drive between this robot's attachments.
            if !under_root && !touches {
                continue;
            }
            let result = (|| -> Result<LinearDrive, String> {
                let origins: Vec<_> = found.arc_bindings.iter().filter(|b| b.sign == 1).collect();
                let insertions: Vec<_> =
                    found.arc_bindings.iter().filter(|b| b.sign == -1).collect();
                if origins.is_empty() || insertions.len() != 1 || origins.len() + 1 != found.arc_bindings.len() {
                    return Err(
                        "expected exactly one + origin and one - insertion attachment".into(),
                    );
                }
                let origin = origins[0].target;
                let via: Vec<DeclId> = origins[1..].iter().map(|b| b.target).collect();
                let insertion = insertions[0].target;
                let path: Vec<DeclId> = std::iter::once(origin).chain(via.iter().copied()).chain([insertion]).collect();
                for id in &path {
                    if !out.attachments.iter().any(|a| a.did == *id) {
                        return Err("force endpoint is not a valid attachment on this robot".into());
                    }
                }
                if path.windows(2).any(|pair| pair[0] == pair[1]) || origin == insertion {
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
                    "hydraulic_muscle" => {
                        let sleeve = Fields {
                            ir,
                            res,
                            did: parameters.reference("sleeve")?,
                        };
                        DriveLaw::HydraulicMuscle {
                            pressure_pa: parameters.positive("pressure_pa")?,
                            response_s: parameters.positive("response_s")?,
                            sleeve: BraidedSleeve {
                                braid_length_m: sleeve.positive("braid_length_m")?,
                                turns: sleeve.positive("turns")?,
                            },
                            tendon_stiffness_n_m: parameters.positive("tendon_stiffness_n_m")?,
                            tendon_slack_m: parameters.positive("tendon_slack_m")?,
                            parallel_stiffness_n_m: parameters.positive("parallel_stiffness_n_m")?,
                            parallel_rest_m: parameters.positive("parallel_rest_m")?,
                            damping_ns_m: parameters.positive("damping_ns_m")?,
                        }
                    }
                    other => return Err(format!("unknown drive law {other}")),
                };
                if !via.is_empty() && !matches!(law, DriveLaw::HydraulicMuscle { .. }) {
                    return Err("via points are defined for hydraulic muscles only".into());
                }
                Ok(LinearDrive {
                    did: found.id,
                    name: found.name.clone(),
                    origin,
                    via,
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
    /// Physical attributes prefer an own declaration, then bases in authored order
    /// (the shared [`QueryEngine::effective_child`] rule).
    pub(crate) fn child(&self, key: &str) -> Option<DeclId> {
        QueryEngine::new(self.ir, self.res).effective_child(self.did, key)
    }
    pub(crate) fn has(&self, key: &str) -> bool {
        self.child(key).is_some()
    }
    pub(crate) fn value(&self, key: &str) -> Result<&ValueR, String> {
        self.child(key)
            .and_then(|c| self.ir.decl_annotation(c)?.value.as_ref())
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
                .and_then(|d| self.ir[d.anno].value.as_ref())
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
