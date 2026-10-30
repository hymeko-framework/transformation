use super::{AssemblyError, ELEMENT_LIMIT, FrameAttachment};
use crate::{
    collide::{Collider, colliders_of},
    forward::FkPlan,
    kinematics::{actuation::Fields, kinematic::KinematicModel, visual::VisualSet},
};
use hymeko::{common::ids::DeclId, ir::ir::ValueR};
use hymeko_query::{QueryEngine, traits::NameResolver};
use std::collections::BTreeMap;

pub(super) struct Bindings {
    pub scope: DeclId,
    pub model: KinematicModel,
    pub plan: FkPlan,
    pub visuals: VisualSet,
    pub colliders: Vec<Collider>,
    pub attachments: Vec<FrameAttachment>,
    pub links: BTreeMap<DeclId, usize>,
    pub joints: BTreeMap<DeclId, usize>,
    pub frames: BTreeMap<DeclId, usize>,
    pub inputs: BTreeMap<DeclId, Vec<usize>>,
    pub initial: Vec<f64>,
}
impl Bindings {
    pub(super) fn read<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        scope: DeclId,
    ) -> Result<Self, AssemblyError> {
        let model = KinematicModel::from_scope(engine, scope)?;
        if model.links.len() > ELEMENT_LIMIT || model.joints.len() > ELEMENT_LIMIT {
            return Err("assembly exceeds 1024 links/joints".to_owned().into());
        }
        for link in &model.links {
            if link.geometry.as_ref().is_some_and(|g| {
                g.dimensions
                    .iter()
                    .any(|n| !n.is_finite() || *n <= 0.0 || *n > 1.0e6)
            }) {
                return Err(
                    format!("link {} has invalid physical dimensions", link.did.raw()).into(),
                );
            }
        }
        if let Some(error) = model.actuation.errors.first() {
            return Err(error.clone().into());
        }
        let plan = FkPlan::new(&model)?;
        let visuals = VisualSet::extract(engine, &model)?;
        let links: BTreeMap<_, _> = model
            .links
            .iter()
            .enumerate()
            .map(|(i, link)| (link.did, i))
            .collect();
        let joints = model
            .joints
            .iter()
            .enumerate()
            .map(|(i, joint)| (joint.did, i))
            .collect();
        let selected = engine
            .scoped(scope)
            .map_err(crate::kinematics::kinematic::scope::ScopedModelError::from)?;
        let attachments = FrameAttachment::read(&selected, scope, &links)?;
        let frames = attachments
            .iter()
            .enumerate()
            .map(|(i, frame)| (frame.frame, i))
            .collect();
        let mut inputs: BTreeMap<DeclId, Vec<usize>> = BTreeMap::new();
        let mut initial = vec![0.0; model.joints.len()];
        for (index, joint) in model.joints.iter().enumerate() {
            let fields = Fields {
                ir: engine.ir(),
                res: engine.resolver(),
                did: joint.did,
            };
            if fields.has("coordinate") {
                let leaf = fields.reference("coordinate")?;
                let value = engine
                    .ir()
                    .decl_annotation(leaf)
                    .and_then(|a| a.value.as_ref());
                let Some(ValueR::Num(n)) = value else {
                    return Err(format!("coordinate {} needs a numeric leaf", leaf.raw()).into());
                };
                if !n.is_finite() {
                    return Err(format!("coordinate {} is nonfinite", leaf.raw()).into());
                }
                initial[index] = *n;
                inputs.entry(leaf).or_default().push(index);
            }
        }
        let colliders = colliders_of(&model);
        Ok(Self {
            scope,
            model,
            plan,
            visuals,
            colliders,
            attachments,
            links,
            joints,
            frames,
            inputs,
            initial,
        })
    }
}
