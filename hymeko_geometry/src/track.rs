//! Closed road projection from canonical IR. Tensor owns rational spline evaluation.
use crate::kinematics::actuation::Fields;
use hymeko::{
    common::ids::DeclId,
    ir::ir::{Ir, ValueR},
};
use hymeko_query::traits::NameResolver;
use hymeko_spline::nurbs::NurbsCurve;

/// A canonical control declaration and its planar world coordinates.
#[derive(Clone, Debug)]
pub struct TrackControl {
    pub id: DeclId,
    pub point: [f64; 2],
    pub weight: f64,
}
/// Static cylindrical obstacle in road coordinates; positive offset is left.
#[derive(Clone, Debug)]
pub struct TrackObstacle {
    pub id: DeclId,
    pub station: f64,
    pub offset: f64,
    pub radius: f64,
    pub height: f64,
    pub position: [f64; 2],
}
/// Road frame in metres; curvature is signed and measured in inverse metres.
#[derive(Clone, Copy, Debug)]
pub struct TrackSample {
    pub position: [f64; 2],
    pub tangent: [f64; 2],
    pub curvature: f64,
    pub station: f64,
    pub parameter: f64,
}
impl TrackSample {
    /// Point offset to the left of the centreline, in metres.
    pub fn offset(self, lateral: f64) -> [f64; 2] {
        [
            self.position[0] - lateral * self.tangent[1],
            self.position[1] + lateral * self.tangent[0],
        ]
    }
}
/// Nearest road position; station wraps at the closed seam.
#[derive(Clone, Copy, Debug)]
pub struct TrackProjection {
    pub station: f64,
    pub lateral: f64,
    pub separation: f64,
}
/// Immutable bounded road. The lookup table is constructed once, never per tick.
#[derive(Clone, Debug)]
pub struct Track {
    pub id: DeclId,
    pub width: f64,
    pub controls: Vec<TrackControl>,
    pub obstacles: Vec<TrackObstacle>,
    curve: NurbsCurve<2>,
    samples: Vec<TrackSample>,
    length: f64,
}
impl Track {
    /// Project a closed rational curve and obstacles from the same canonical IR.
    /// # Preconditions
    /// Degree 1..5, 2..256 ordered controls, finite positive weights, sorted knots.
    /// Road width 4..40 m; 256..8192 arc intervals; at most 32 obstacles.
    /// # Postconditions
    /// Curve positions/tangents close, every table frame is regular, original IDs survive.
    /// # Errors
    /// Invalid fields, degenerate/open curves, excessive size or invalid obstacles.
    pub fn from_ir<R: NameResolver>(ir: &Ir, res: &R, id: DeclId) -> Result<Self, String> {
        if id.is_none() || id.raw() >= ir.decl_nodes.len() {
            return Err("invalid track declaration".into());
        }
        let fields = Fields { ir, res, did: id };
        let width = fields.number("width")?;
        if !(4.0..=40.0).contains(&width) {
            return Err("road width must be 4..40 metres".into());
        }
        let count = fields.integer("samples", 256, 8192)? as usize;
        let controls_id = child(ir, res, id, "controls").ok_or("missing track controls")?;
        let controls = read_controls(ir, res, controls_id)?;
        let curve = read_curve(&fields, &controls)?;
        let samples = Self::table(&curve, count)?;
        let length = validate_closed(width, &samples)?;
        let mut track = Self {
            id,
            width,
            controls,
            obstacles: Vec::new(),
            curve,
            samples,
            length,
        };
        track.read_obstacles(ir, res)?;
        Ok(track)
    }
    fn read_obstacles<R: NameResolver>(&mut self, ir: &Ir, res: &R) -> Result<(), String> {
        let Some(parent) = child(ir, res, self.id, "obstacles") else {
            return Ok(());
        };
        for did in ir.decl_children(parent) {
            if self.obstacles.len() == 32 {
                return Err("more than 32 obstacles".into());
            }
            let f = Fields { ir, res, did };
            let fraction = f.number("fraction")?;
            let offset = f.number("offset")?;
            let radius = f.positive("radius")?;
            let height = f.positive("height")?;
            if !(0.0..1.0).contains(&fraction)
                || offset.abs() > self.width * 0.5
                || radius > 40.0
                || height > 10.0
            {
                return Err("invalid obstacle road coordinates or dimensions".into());
            }
            let station = fraction * self.length;
            let position = self.sample(station)?.offset(offset);
            self.obstacles.push(TrackObstacle {
                id: did,
                station,
                offset,
                radius,
                height,
                position,
            });
        }
        Ok(())
    }
    fn frame(curve: &NurbsCurve<2>, parameter: f64, station: f64) -> Result<TrackSample, String> {
        let s = curve.sample(parameter).map_err(|e| e.to_string())?;
        let speed = norm(s.first);
        if !speed.is_finite() || speed < 1e-9 {
            return Err("singular track tangent".into());
        }
        let curvature = (s.first[0] * s.second[1] - s.first[1] * s.second[0]) / speed.powi(3);
        if !curvature.is_finite() {
            return Err("nonfinite track curvature".into());
        }
        Ok(TrackSample {
            position: s.position,
            tangent: [s.first[0] / speed, s.first[1] / speed],
            curvature,
            station,
            parameter,
        })
    }
    fn table(curve: &NurbsCurve<2>, count: usize) -> Result<Vec<TrackSample>, String> {
        let [lo, hi] = curve.domain();
        let mut samples: Vec<TrackSample> = Vec::with_capacity(count + 1);
        for i in 0..=count {
            let u = if i == count {
                hi
            } else {
                lo + (hi - lo) * (i as f64 / count as f64)
            };
            let mut s = Self::frame(curve, u, 0.0)?;
            if let Some(previous) = samples.last() {
                let ds = norm(sub(s.position, previous.position));
                if !ds.is_finite() || ds < 1e-9 {
                    return Err("degenerate arc interval".into());
                }
                s.station = previous.station + ds;
            }
            samples.push(s);
        }
        Ok(samples)
    }
    /// Centreline arc length estimated by the validated polyline table, in metres.
    pub fn length(&self) -> f64 {
        self.length
    }
    /// Cached frames including the duplicated seam, for one-time mesh construction.
    pub fn samples(&self) -> &[TrackSample] {
        &self.samples
    }
    /// Evaluate the rational curve at a wrapped arc distance in O(log N), allocation-free.
    /// # Errors
    /// Nonfinite station or an ill-conditioned spline sample.
    pub fn sample(&self, station: f64) -> Result<TrackSample, String> {
        if !station.is_finite() {
            return Err("station must be finite".into());
        }
        let station = station.rem_euclid(self.length);
        let i = self
            .samples
            .partition_point(|s| s.station <= station)
            .clamp(1, self.samples.len() - 1);
        let a = self.samples[i - 1];
        let b = self.samples[i];
        let fraction = (station - a.station) / (b.station - a.station);
        Self::frame(
            &self.curve,
            a.parameter + fraction * (b.parameter - a.parameter),
            station,
        )
    }
    /// Project onto the cached centreline. An optional prior station and search radius
    /// restrict the search to a local arc window; None performs global initialization.
    /// Returns None for invalid inputs or an empty search window.
    pub fn project(&self, point: [f64; 2], window: Option<(f64, f64)>) -> Option<TrackProjection> {
        if point.iter().any(|v| !v.is_finite())
            || window.is_some_and(|(s, r)| !s.is_finite() || !r.is_finite() || r <= 0.0)
        {
            return None;
        }
        let mut best = None;
        let mut best_distance = f64::INFINITY;
        // Binary search selects bounded windows, avoiding a full-track scan per tick.
        let (start, end) = window.map_or((0, self.samples.len() - 1), |(s, r)| {
            let s = s.rem_euclid(self.length);
            if r >= self.length * 0.5 || s - r < 0.0 || s + r >= self.length {
                (0, self.samples.len() - 1)
            } else {
                (
                    self.samples
                        .partition_point(|x| x.station < s - r)
                        .saturating_sub(1),
                    self.samples
                        .partition_point(|x| x.station <= s + r)
                        .min(self.samples.len() - 1),
                )
            }
        });
        for pair in self.samples[start..=end].windows(2) {
            let a = pair[0];
            let b = pair[1];
            if let Some((s, r)) = window {
                let delta =
                    (a.station - s + self.length * 0.5).rem_euclid(self.length) - self.length * 0.5;
                if delta.abs() > r + (b.station - a.station) {
                    continue;
                }
            }
            let d = sub(b.position, a.position);
            let q = sub(point, a.position);
            let t = ((q[0] * d[0] + q[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0.0, 1.0);
            let residual = [q[0] - t * d[0], q[1] - t * d[1]];
            let separation = norm(residual);
            if separation < best_distance {
                best_distance = separation;
                best = Some(TrackProjection {
                    station: (a.station + t * (b.station - a.station)).rem_euclid(self.length),
                    lateral: (d[0] * residual[1] - d[1] * residual[0]) / norm(d),
                    separation,
                });
            }
        }
        best
    }
}
fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
fn norm(v: [f64; 2]) -> f64 {
    v[0].hypot(v[1])
}

fn child<R: NameResolver>(ir: &Ir, res: &R, parent: DeclId, name: &str) -> Option<DeclId> {
    ir.decl_children(parent)
        .find(|c| res.resolve(ir.decl_nodes[c.raw()].name) == name)
}
fn read_controls<R: NameResolver>(
    ir: &Ir,
    res: &R,
    parent: DeclId,
) -> Result<Vec<TrackControl>, String> {
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
fn read_curve<R: NameResolver>(
    fields: &Fields<'_, R>,
    controls: &[TrackControl],
) -> Result<NurbsCurve<2>, String> {
    let degree = fields.integer("degree", 1, 5)? as usize;
    let knots = read_knots(fields.value("knots")?)?;
    let points: Vec<_> = controls.iter().map(|c| c.point).collect();
    let weights: Vec<_> = controls.iter().map(|c| c.weight).collect();
    NurbsCurve::new(degree, &points, &weights, &knots).map_err(|e| e.to_string())
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
fn validate_closed(width: f64, samples: &[TrackSample]) -> Result<f64, String> {
    let last = *samples.last().ok_or("track has no samples")?;
    let first = samples[0];
    let length = last.station;
    if !(100.0..=100_000.0).contains(&length)
        || norm(sub(first.position, last.position)) > 1e-6
        || norm(sub(first.tangent, last.tangent)) > 1e-6
    {
        return Err("track must close in position and tangent, length 100..100000 metres".into());
    }
    if samples
        .iter()
        .any(|s| s.curvature.abs() * width * 0.5 >= 0.8)
    {
        return Err("track turn radius too small for its road width".into());
    }
    Ok(length)
}
