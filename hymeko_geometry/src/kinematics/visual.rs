//! Render geometry attached to existing physical links. No new bodies or collisions.
use super::actuation::Fields;
use super::kinematic::{GeometryInfo, GeometryShape, KinematicModel, LinkInfo, geometry_decl};
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
};
use hymeko_clifford::Se3;
use hymeko_query::{QueryEngine, traits::NameResolver};

/// One declared visual, in its physical link's local frame.
#[derive(Debug, Clone)]
pub struct VisualInstance {
    pub link_index: usize,
    pub did: DeclId,
    pub name: String,
    pub geometry: Option<GeometryInfo>,
    pub local_pose: Se3,
    pub color: [f32; 4],
}

/// A bounded, flat projection of `visual -> geometry_or_group` references.
#[derive(Debug, Clone)]
pub struct VisualSet {
    pub instances: Vec<VisualInstance>,
}

impl VisualSet {
    /// Read visuals using the SAME compiled program as `model`.
    ///
    /// Missing `visual` preserves legacy geometry/origin. A referenced primitive
    /// or flat group may specify local `origin`, `rpy_deg` and `color` attributes.
    /// Groups do not recurse; at most 64 visuals/link and 1024/model are accepted.
    /// Physical model, collision shapes and joint topology are never modified.
    /// # Errors
    /// Invalid references, shapes, dimensions, transforms, colours or size caps.
    pub fn extract<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        model: &KinematicModel,
    ) -> Result<Self, String> {
        let reader = Reader {
            ir: engine.ir(),
            res: engine.resolver(),
        };
        let mut instances = Vec::new();
        for (link_index, link) in model.links.iter().enumerate() {
            let first = instances.len();
            let fields = Fields {
                ir: reader.ir,
                res: reader.res,
                did: link.did,
            };
            if !fields.has("visual") {
                instances.push(reader.legacy(link_index, link)?);
                if instances.len() > 1024 {
                    return Err("more than 1024 visuals/model".into());
                }
                continue;
            }
            let target = fields.reference("visual")?;
            if target.is_none() || target.raw() >= reader.ir.decl_nodes.len() {
                return Err(format!("{}.visual has an unresolved target", link.name));
            }
            if let Some(geometry) = geometry_decl(reader.ir, reader.res, target)? {
                instances.push(reader.instance(link_index, link, target, geometry)?);
            } else {
                for child in reader.ir.decl_children(target) {
                    if instances.len() - first >= 64 {
                        return Err(format!("{}: more than 64 visuals/link", link.name));
                    }
                    let geometry =
                        geometry_decl(reader.ir, reader.res, child)?.ok_or_else(|| {
                            format!("{}: visual group children must be primitives", link.name)
                        })?;
                    instances.push(reader.instance(link_index, link, child, geometry)?);
                }
                if instances.len() == first {
                    return Err(format!("{}: empty visual group", link.name));
                }
            }
            if instances.len() > 1024 {
                return Err("more than 1024 visuals/model".into());
            }
        }
        Ok(Self { instances })
    }
}

struct Reader<'a, R> {
    ir: &'a Ir,
    res: &'a R,
}

impl<R: NameResolver> Reader<'_, R> {
    fn vector<const N: usize>(
        &self,
        parent: DeclId,
        name: &str,
        default: [f64; N],
    ) -> Result<[f64; N], String> {
        let fields = Fields {
            ir: self.ir,
            res: self.res,
            did: parent,
        };
        let out = if fields.has(name) {
            fields.array(name)?
        } else {
            default
        };
        if out.iter().any(|v| v.abs() > f32::MAX as f64) {
            return Err(format!("{name}: values must fit f32"));
        }
        Ok(out)
    }

    fn colour(&self, did: DeclId, fallback: [f64; 4]) -> Result<[f32; 4], String> {
        let fields = Fields {
            ir: self.ir,
            res: self.res,
            did,
        };
        let color = if fields.has("color") {
            fields
                .array::<4>("color")
                .or_else(|_| fields.array::<3>("color").map(|c| [c[0], c[1], c[2], 1.0]))?
        } else {
            fallback
        };
        if color.iter().any(|v| !(0.0..=1.0).contains(v)) {
            return Err("color: RGBA must be in 0..1".into());
        }
        Ok(color.map(|v| v as f32))
    }

    fn legacy(&self, link_index: usize, link: &LinkInfo) -> Result<VisualInstance, String> {
        let origin = self.vector(link.did, "origin", [0.0; 3])?;
        Ok(VisualInstance {
            link_index,
            did: link.did,
            name: link.name.clone(),
            geometry: link.geometry.clone(),
            local_pose: Se3::rpy_xyz([0.0; 3], origin),
            color: self.colour(link.did, [0.72, 0.74, 0.80, 1.0])?,
        })
    }

    fn instance(
        &self,
        link_index: usize,
        link: &LinkInfo,
        did: DeclId,
        geometry: GeometryInfo,
    ) -> Result<VisualInstance, String> {
        let count = match geometry.shape {
            GeometryShape::Box => 3,
            GeometryShape::Sphere => 1,
            _ => 2,
        };
        // Check the raw IR too: the legacy physical reader skips non-numeric list members.
        let fields = Fields {
            ir: self.ir,
            res: self.res,
            did,
        };
        let ValueR::List(values) = fields.value("dimension")? else {
            return Err("visual: missing dimension list".into());
        };
        if values.len() != count
            || values
                .iter()
                .any(|v| !matches!(v, ValueR::Num(n) if n.is_finite() && *n > 0.0 && *n <= 1.0e6))
        {
            return Err(format!(
                "visual {:?}: expected {count} positive finite dimensions <= 1e6",
                geometry.shape
            ));
        }
        let legacy_origin = self.vector(link.did, "origin", [0.0; 3])?;
        let origin = self.vector(did, "origin", legacy_origin)?;
        let rpy = self.vector(did, "rpy_deg", [0.0; 3])?.map(f64::to_radians);
        let base_color = self
            .colour(link.did, [0.72, 0.74, 0.80, 1.0])?
            .map(f64::from);
        Ok(VisualInstance {
            link_index,
            did,
            name: self
                .res
                .resolve(self.ir.decl_nodes[did.raw()].name)
                .to_owned(),
            geometry: Some(geometry),
            local_pose: Se3::rpy_xyz(rpy, origin),
            color: self.colour(did, base_color)?,
        })
    }
}
