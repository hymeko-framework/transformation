//! Robot TP parameter charts are hyperedges over resolved scheduling-axis declarations.
//! This is a borrowed IR projection, not a tensor decomposition or another pose graph.
use super::{actuation::Fields, joints::JointType, kinematic::KinematicModel};
use hymeko::common::ids::DeclId;
use hymeko_query::{Predicate, QueryEngine, traits::NameResolver};

/// One physical scheduling coordinate, bound to an authored joint.
#[derive(Debug, Clone)]
pub struct Axis {
    pub did: DeclId,
    pub joint: DeclId,
    pub range_rad: [f64; 2],
}

/// A bounded two-parameter chart for the full robot's rest-acceleration operator.
#[derive(Debug, Clone)]
pub struct Chart {
    pub did: DeclId,
    pub name: String,
    pub axes: [Axis; 2],
    pub samples: usize,
    pub rank: usize,
    pub cr_grid: usize,
}

/// Read the declared charts. Incidence order defines tensor mode order.
/// Unsupported arity/operator or invalid references fail instead of silently dropping a chart.
pub fn project<R: NameResolver>(
    engine: &QueryEngine<'_, R>,
    model: &KinematicModel,
) -> Result<Vec<Chart>, String> {
    let ir = engine.ir();
    let res = engine.resolver();
    let mut result = Vec::new();
    for found in engine.query(&Predicate::edge().and(Predicate::inherits("tp_chart"))) {
        let parent = ir.decl_nodes[found.id.raw()].parent;
        if parent.is_none() || res.resolve(ir.decl_nodes[parent.raw()].name) != model.name {
            continue;
        }
        let read = Fields {
            ir,
            res,
            did: found.id,
        };
        if read.text("operator")? != "rest_acceleration" {
            return Err("unsupported robot TP operator".into());
        }
        if found.arc_bindings.len() != 2 || found.arc_bindings.iter().any(|b| b.sign != 1) {
            return Err(format!("{} requires two + scheduling axes", found.name));
        }
        let mut axes = Vec::new();
        for binding in &found.arc_bindings {
            let axis = Fields {
                ir,
                res,
                did: binding.target,
            };
            let joint = axis.reference("joint")?;
            let j = model
                .joints
                .iter()
                .find(|j| j.did == joint)
                .ok_or("TP axis joint is outside the robot")?;
            if !matches!(j.joint_type, JointType::Revolute | JointType::Continuous) {
                return Err("TP axis requires a revolute coordinate".into());
            }
            let lo = axis.number("lower_deg")?;
            let hi = axis.number("upper_deg")?;
            if lo >= hi {
                return Err("empty TP axis domain".into());
            }
            if j.limits
                .as_ref()
                .is_some_and(|l| lo < l.lower || hi > l.upper)
            {
                return Err(format!("TP domain exceeds joint limits on {}", j.name));
            }
            axes.push(Axis {
                did: binding.target,
                joint,
                range_rad: [lo.to_radians(), hi.to_radians()],
            });
        }
        if axes[0].joint == axes[1].joint {
            return Err("repeated TP scheduling coordinate".into());
        }
        let integer = |name, lo, hi| -> Result<usize, String> {
            let n = read.positive(name)?;
            if n.fract() != 0.0 || n < lo as f64 || n > hi as f64 {
                return Err(format!("invalid {name}"));
            }
            Ok(n as usize)
        };
        let samples = integer("samples", 5, 33)?;
        result.push(Chart {
            did: found.id,
            name: found.name,
            axes: axes.try_into().unwrap(),
            samples,
            rank: integer("rank", 1, 4.min(samples))?,
            cr_grid: integer("cr_grid", 3, 257)?,
        });
    }
    Ok(result)
}

/// Each body induces a hyperedge of incident moving joints (the dual incidence).
/// The returned handles refer to the existing kinematic model; no poses are stored here.
pub fn body_joint_hyperedges(model: &KinematicModel) -> Vec<(DeclId, Vec<DeclId>)> {
    model
        .links
        .iter()
        .filter_map(|link| {
            let joints: Vec<_> = model
                .joints
                .iter()
                .filter(|j| {
                    j.joint_type != JointType::Fixed
                        && (j.parent == link.did || j.child == link.did)
                })
                .map(|j| j.did)
                .collect();
            (!joints.is_empty()).then_some((link.did, joints))
        })
        .collect()
}
