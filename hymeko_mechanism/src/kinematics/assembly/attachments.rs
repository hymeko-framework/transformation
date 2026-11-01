use super::ELEMENT_LIMIT;
use crate::kinematics::actuation::Fields;
use hymeko::{common::ids::DeclId, ir::ir::DeclKind};
use hymeko_clifford::Se3;
use hymeko_query::{Predicate, QueryEngine, traits::NameResolver};
use std::collections::{BTreeMap, HashSet};

/// An oriented frame mounted on a body by one explicitly typed authored hyperedge.
#[derive(Debug, Clone)]
pub struct FrameAttachment {
    pub relation: DeclId,
    pub frame: DeclId,
    pub link: DeclId,
    pub name: String,
    pub local_pose: Se3,
    pub(super) link_index: usize,
}
impl FrameAttachment {
    pub(super) fn read<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        scope: DeclId,
        links: &BTreeMap<DeclId, usize>,
    ) -> Result<Vec<Self>, String> {
        let root = Fields {
            ir: engine.ir(),
            res: engine.resolver(),
            did: scope,
        };
        if !root.has("attachment_kind") {
            return Ok(Vec::new());
        }
        let kind = root.reference("attachment_kind")?;
        if engine
            .ir()
            .decl_node(kind)
            .is_none_or(|d| d.kind != DeclKind::Edge)
        {
            return Err("attachment_kind must reference an edge kind".into());
        }
        let predicate = Predicate::edge().and(Predicate::InheritsFromId(kind));
        let mut result = Vec::new();
        let mut seen = HashSet::new();
        for relation in engine.query_iter(&predicate) {
            if result.len() == ELEMENT_LIMIT {
                return Err("assembly exceeds 1024 frame attachments".into());
            }
            let [first, second] = relation.arc_bindings.as_slice() else {
                return Err("frame attachment must have exactly two participants".into());
            };
            let (parent, target) = match (first.sign, second.sign) {
                (1, -1) => (first, second),
                (-1, 1) => (second, first),
                _ => return Err("frame attachment needs (+ body, - frame)".into()),
            };
            let Some(&link_index) = links.get(&parent.target) else {
                return Err("frame attachment body is outside its group".into());
            };
            let Some(frame) = engine.query_decl(target.target, &Predicate::inherits("frame"))
            else {
                return Err("frame attachment target must be a frame in its group".into());
            };
            if links.contains_key(&frame.id) || !seen.insert(frame.id) {
                return Err("attached frame must be distinct and mounted once".into());
            }
            let fields = Fields {
                ir: engine.ir(),
                res: engine.resolver(),
                did: relation.id,
            };
            let position = if fields.has("origin") {
                fields.array::<3>("origin")?
            } else {
                [0.0; 3]
            };
            let rpy = if fields.has("rpy_deg") {
                fields.array::<3>("rpy_deg")?
            } else {
                [0.0; 3]
            };
            if position.iter().any(|n| n.abs() > 1.0e6) {
                return Err("frame attachment offset exceeds 1e6 metres".into());
            }
            result.push(Self {
                relation: relation.id,
                frame: frame.id,
                link: parent.target,
                name: frame.name,
                local_pose: Se3::rpy_xyz(rpy.map(f64::to_radians), position),
                link_index,
            });
        }
        Ok(result)
    }
}
