//! Closed roads read out of HyMeKo IR. The road itself — its arc-length table, sampling,
//! projection and obstacles — is kawakern's `track`; this module reads the declaration into a
//! [`kawakern::track::TrackSpec`] and builds it, with Core declaration handles.
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
};
use hymeko_mechanism::{Fields, kinematics::IrHandles};
use hymeko_query::traits::NameResolver;
use kawakern::track::{ObstacleSpec, TrackControl, TrackSpec};

/// A road read from IR; its ids are Core declarations.
pub type Track = kawakern::track::Track<IrHandles>;

/// Read the road declared at `id`: width, sample count, degree, knots, controls and obstacles.
/// # Errors
/// Missing or mistyped fields, and every bound [`kawakern::track::Track::new`] checks.
pub fn read_track<R: NameResolver>(ir: &Ir, res: &R, id: DeclId) -> Result<Track, String> {
    if id.is_none() || id.raw() >= ir.decl_nodes.len() {
        return Err("invalid track declaration".into());
    }
    let fields = Fields { ir, res, did: id };
    let width = fields.number("width")?;
    let samples = fields.integer("samples", 256, 8192)? as usize;
    let degree = fields.integer("degree", 1, 5)? as usize;
    let knots = read_knots(fields.value("knots")?)?;
    let controls_id = child(ir, res, id, "controls").ok_or("missing track controls")?;
    let controls = read_controls(ir, res, controls_id)?;
    let obstacles = match child(ir, res, id, "obstacles") {
        Some(parent) => read_obstacles(ir, res, parent)?,
        None => Vec::new(),
    };
    Track::new(TrackSpec { id, width, samples, degree, knots, controls, obstacles })
}

fn child<R: NameResolver>(ir: &Ir, res: &R, parent: DeclId, name: &str) -> Option<DeclId> {
    ir.decl_children(parent)
        .find(|c| res.resolve(ir.decl_nodes[c.raw()].name) == name)
}

fn read_controls<R: NameResolver>(
    ir: &Ir,
    res: &R,
    parent: DeclId,
) -> Result<Vec<TrackControl<IrHandles>>, String> {
    let mut controls = Vec::new();
    for did in ir.decl_children(parent) {
        if controls.len() == 256 {
            return Err("more than 256 control points".into());
        }
        let point = Fields { ir, res, did };
        controls.push(TrackControl {
            id: did,
            point: point.array("point")?,
            weight: point.positive("weight")?,
        });
    }
    Ok(controls)
}

fn read_obstacles<R: NameResolver>(
    ir: &Ir,
    res: &R,
    parent: DeclId,
) -> Result<Vec<ObstacleSpec<IrHandles>>, String> {
    let mut obstacles = Vec::new();
    for did in ir.decl_children(parent) {
        if obstacles.len() == 32 {
            return Err("more than 32 obstacles".into());
        }
        let f = Fields { ir, res, did };
        obstacles.push(ObstacleSpec {
            id: did,
            fraction: f.number("fraction")?,
            offset: f.number("offset")?,
            radius: f.positive("radius")?,
            height: f.positive("height")?,
        });
    }
    Ok(obstacles)
}

fn read_knots(value: &ValueR) -> Result<Vec<f64>, String> {
    let ValueR::List(values) = value else {
        return Err("knots must be a numeric list".into());
    };
    if values.len() > 262 {
        return Err("more than 262 knots".into());
    }
    values
        .iter()
        .map(|v| match v {
            ValueR::Num(n) if n.is_finite() => Ok(*n),
            _ => Err("knots must be finite".to_string()),
        })
        .collect()
}
