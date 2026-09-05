//! Calibrated robot cameras attached to existing links through resolved incidence.
//! No world poses are stored: the dynamics engine owns their changing placement.
use super::{actuation::Fields, kinematic::KinematicModel};
use hymeko::common::ids::DeclId;
use hymeko_query::{Predicate, QueryEngine, traits::NameResolver};
use std::collections::HashSet;

/// An ideal pinhole camera in its mounting link's axes (metres).
#[derive(Debug, Clone)]
pub struct Camera {
    pub did: DeclId,
    pub name: String,
    pub link: DeclId,
    pub camera_id: u32,
    pub frame_id: String,
    pub size: [u32; 2],
    pub fovy_deg: f64,
    pub rate_hz: f64,
    pub clip: [f64; 2],
    pub position: [f64; 3],
    /// Camera right and image up; optical forward = right cross up, negated.
    pub xy_axes: [f64; 6],
}

impl Camera {
    /// Pixel centres use integer coordinates; pixels are square and undistorted.
    pub fn intrinsics(&self) -> [f64; 9] {
        let [w, h] = self.size.map(f64::from);
        let f = h / (2.0 * (self.fovy_deg.to_radians() / 2.0).tan());
        [
            f,
            0.0,
            (w - 1.0) / 2.0,
            0.0,
            f,
            (h - 1.0) / 2.0,
            0.0,
            0.0,
            1.0,
        ]
    }
}

/// A rectified pair: + left, - right, with separate canonical sensor identities.
#[derive(Debug, Clone)]
pub struct StereoPair {
    pub did: DeclId,
    pub name: String,
    pub left: DeclId,
    pub right: DeclId,
    pub baseline_m: f64,
}

/// A validated sensor projection over one existing robot, retaining canonical handles.
#[derive(Debug, Default)]
pub struct CameraSet {
    pub cameras: Vec<Camera>,
    pub pairs: Vec<StereoPair>,
}

impl CameraSet {
    pub fn project<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        model: &KinematicModel,
    ) -> Result<Self, String> {
        let ir = engine.ir();
        let res = engine.resolver();
        let belongs = |did: DeclId| {
            let p = ir.decl_nodes[did.raw()].parent;
            !p.is_none() && res.resolve(ir.decl_nodes[p.raw()].name) == model.name
        };
        let mut out = Self::default();
        let mut ids = HashSet::new();
        let mut frames = HashSet::new();
        for found in engine.query(&Predicate::node().and(Predicate::inherits("rgb_camera"))) {
            if !belongs(found.id) {
                continue;
            }
            let fields = Fields {
                ir,
                res,
                did: found.id,
            };
            let camera = Self::read_camera(engine, model, &fields, found.name)?;
            if !ids.insert(camera.camera_id) || !frames.insert(camera.frame_id.clone()) {
                return Err("camera transport ids and optical frame names must be unique".into());
            }
            out.cameras.push(camera);
        }
        if out.cameras.len() > 8 {
            return Err("at most 8 cameras per robot".into());
        }
        let mut paired = HashSet::new();
        for found in engine.query(&Predicate::edge().and(Predicate::inherits("stereo_pair"))) {
            if !belongs(found.id) {
                continue;
            }
            let left: Vec<_> = found.arc_bindings.iter().filter(|a| a.sign == 1).collect();
            let right: Vec<_> = found.arc_bindings.iter().filter(|a| a.sign == -1).collect();
            if found.arc_bindings.len() != 2 || left.len() != 1 || right.len() != 1 {
                return Err("stereo pair requires + left and - right camera".into());
            }
            let l = out
                .cameras
                .iter()
                .find(|c| c.did == left[0].target)
                .ok_or("left camera outside robot")?;
            let r = out
                .cameras
                .iter()
                .find(|c| c.did == right[0].target)
                .ok_or("right camera outside robot")?;
            if !paired.insert(l.did) || !paired.insert(r.did) {
                return Err("camera repeated in stereo pairs".into());
            }
            out.pairs.push(StereoPair {
                did: found.id,
                name: found.name,
                left: l.did,
                right: r.did,
                baseline_m: Self::baseline(l, r)?,
            });
        }
        Ok(out)
    }

    fn read_camera<R: NameResolver>(
        engine: &QueryEngine<'_, R>,
        model: &KinematicModel,
        read: &Fields<'_, R>,
        name: String,
    ) -> Result<Camera, String> {
        let mut links = Vec::new();
        for edge in engine.query(&Predicate::edge().and(Predicate::inherits("sensor_connection"))) {
            if !edge.arc_bindings.iter().any(|a| a.target == read.did) {
                continue;
            }
            let mounts: Vec<_> = edge
                .arc_bindings
                .iter()
                .filter(|a| a.sign == -1 && model.links.iter().any(|l| l.did == a.target))
                .collect();
            if edge.arc_bindings.len() != 2
                || mounts.len() != 1
                || !edge
                    .arc_bindings
                    .iter()
                    .any(|a| a.sign == 1 && a.target == read.did)
            {
                return Err(format!(
                    "{name}: sensor_connection requires + camera and - robot link"
                ));
            }
            links.push(mounts[0].target);
        }
        if links.len() != 1 {
            return Err(format!("{name}: camera needs exactly one mounting link"));
        }
        let xy_axes = read.array::<6>("xy_axes")?;
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
        if (dot(&xy_axes[..3], &xy_axes[..3]) - 1.0).abs() > 1e-8
            || (dot(&xy_axes[3..], &xy_axes[3..]) - 1.0).abs() > 1e-8
            || dot(&xy_axes[..3], &xy_axes[3..]).abs() > 1e-8
        {
            return Err("camera right/up axes must be orthonormal".into());
        }
        let fovy_deg = read.positive("fovy_deg")?;
        let clip = read.array::<2>("clip")?;
        let rate_hz = read.positive("update_rate")?;
        let frame_id = read.text("optical_frame")?.to_string();
        if read.has("distortion") && read.array::<5>("distortion")?.iter().any(|&d| d != 0.0) {
            return Err("ideal camera projection supports zero distortion only".into());
        }
        if fovy_deg >= 170.0
            || clip[0] <= 0.0
            || clip[0] >= clip[1]
            || rate_hz > 120.0
            || frame_id.trim().is_empty()
        {
            return Err("invalid pinhole camera calibration/rate/frame".into());
        }
        Ok(Camera {
            did: read.did,
            name,
            link: links[0],
            camera_id: read.integer("camera_id", 0, u32::MAX)?,
            frame_id,
            size: [
                read.integer("width", 1, 2048)?,
                read.integer("height", 1, 2048)?,
            ],
            fovy_deg,
            clip,
            rate_hz,
            position: read.array("position")?,
            xy_axes,
        })
    }

    fn baseline(l: &Camera, r: &Camera) -> Result<f64, String> {
        if l.link != r.link
            || l.size != r.size
            || l.fovy_deg != r.fovy_deg
            || l.rate_hz != r.rate_hz
            || l.xy_axes != r.xy_axes
            || l.clip != r.clip
        {
            return Err(
                "rectified stereo cameras require one link, identical intrinsics, axes and rate"
                    .into(),
            );
        }
        let delta: Vec<_> = r
            .position
            .iter()
            .zip(l.position)
            .map(|(a, b)| a - b)
            .collect();
        let baseline: f64 = delta.iter().zip(l.xy_axes).map(|(d, x)| d * x).sum();
        if baseline <= 1e-6
            || delta
                .iter()
                .zip(l.xy_axes)
                .any(|(d, x)| (d - baseline * x).abs() > 1e-8)
        {
            return Err("stereo baseline must point along camera right".into());
        }
        Ok(baseline)
    }
}
