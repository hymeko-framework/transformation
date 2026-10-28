//! Inverse kinematics of a serial chain by damped least squares.
//!
//! The geometric Jacobian comes straight from one forward pass: a revolute joint turns about its
//! world axis through a point on it, a prismatic one slides along it. [`SerialChain`] supplies that
//! pass. A standard-DH chain ([`solve_standard_dh`]) reads joint `j`'s axis as the z axis of frame
//! `j - 1` from [`standard_dh_chain`], so a target on frame `k` depends on joints `1..=k` only; an
//! origin/axis [`KinematicModel`] path ([`ModelChain`]) reads it off [`forward_kinematics`]. Every
//! iteration costs one forward pass and one small (3x3 or 6x6) symmetric solve. Damping keeps the
//! step bounded near singularities (an aligned wrist, a stretched arm); a step cap and the joint
//! limits bound it further. The solver never extrapolates beyond the limits and reports what it
//! could not reach instead of failing. A target may be a region ([`Envelope`]): each iteration
//! then steers toward the region's pose nearest to the frame, and a frame inside it is reached.
use super::dh::{DhChainError, standard_dh_chain};
use super::joints::{JointEncoding, JointInfo, JointType};
use super::kinematic::KinematicModel;
use crate::forward::{JointValues, forward_kinematics};
use crate::collide::cross;
use hymeko_tensor::linalg::dense::damped_least_squares;
use hymeko_clifford::Se3;
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI};

/// Where frame `frame` (1-based: the child of joint `frame`) should be.
#[derive(Debug, Clone, Copy)]
pub struct IkTarget {
    pub frame: usize,
    pub pose: Se3,
    /// Also match the frame's orientation; otherwise only its origin, and the envelope's rotations
    /// do not apply.
    pub orientation: bool,
    /// How far from `pose` the frame may stay; [`Envelope::POINT`] asks for `pose` itself.
    pub envelope: Envelope,
}

/// A target region (a task-space region): `[lower, upper]` bounds per degree of freedom on the
/// constrained frame's displacement from the target pose. Translations x, y, z are in metres along
/// the target's own axes; rotations roll, pitch, yaw are in radians, the frame's rotation relative
/// to the target as fixed-axis x-y-z angles (`Rz(yaw) Ry(pitch) Rx(roll)`), within `[-pi, pi]`,
/// `[-pi/2, pi/2]` and `[-pi, pi]`. A free spin about the tool axis is `yaw [-pi, pi]`.
///
/// The solver aims at the region's pose nearest to the frame, each displacement clamped into its
/// bounds, so it moves only as far as the region's near edge and a frame already inside stays put.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Envelope {
    pub bounds: [[f64; 2]; 6],
}

impl Envelope {
    /// The target pose itself.
    pub const POINT: Self = Self { bounds: [[0.; 2]; 6] };

    /// Every bound finite and ordered, the rotations within their ranges; the solver refuses
    /// ([`IkError::Target`]) an envelope that is not.
    pub fn is_valid(&self) -> bool {
        let span = [f64::INFINITY, f64::INFINITY, f64::INFINITY, PI, FRAC_PI_2, PI];
        self.bounds.iter().zip(span).all(|([lo, hi], limit)| lo.is_finite() && hi.is_finite() && lo <= hi && -limit <= *lo && *hi <= limit)
    }

    /// The pose within the envelope about `target` (position, row-major rotation) nearest to the
    /// frame at `at` and `current`; with `orientation` false only its position is bounded.
    pub fn nearest(&self, position: &[f64; 3], rotation: &[f64; 9], at: &[f64; 3], current: &[f64; 9], orientation: bool) -> ([f64; 3], [f64; 9]) {
        let clamp = |v: f64, [lo, hi]: [f64; 2]| v.clamp(lo, hi);
        // Displacements in the target's axes: R^T (at - position) and R^T current.
        let offset: [f64; 3] = std::array::from_fn(|i| at[i] - position[i]);
        let local: [f64; 3] = std::array::from_fn(|i| clamp((0..3).map(|m| rotation[m * 3 + i] * offset[m]).sum(), self.bounds[i]));
        let near = std::array::from_fn(|i| position[i] + (0..3).map(|m| rotation[i * 3 + m] * local[m]).sum::<f64>());
        if !orientation {
            return (near, *current);
        }
        let d: [f64; 9] = std::array::from_fn(|i| {
            let (row, col) = (i / 3, i % 3);
            (0..3).map(|m| rotation[m * 3 + row] * current[m * 3 + col]).sum()
        });
        let roll = clamp(d[7].atan2(d[8]), self.bounds[3]);
        let pitch = clamp((-d[6]).atan2(d[7].hypot(d[8])), self.bounds[4]);
        let yaw = clamp(d[3].atan2(d[0]), self.bounds[5]);
        let ((sr, cr), (sp, cp), (sy, cy)) = (roll.sin_cos(), pitch.sin_cos(), yaw.sin_cos());
        let bounded = [
            cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp, cp * sr, cp * cr,
        ];
        let turned = std::array::from_fn(|i| {
            let (row, col) = (i / 3, i % 3);
            (0..3).map(|m| rotation[row * 3 + m] * bounded[m * 3 + col]).sum()
        });
        (near, turned)
    }
}

impl Default for Envelope {
    fn default() -> Self {
        Self::POINT
    }
}

/// Solver settings; [`IkOptions::ARM`] (the [`Default`]) suits arm-sized chains in metres and
/// radians.
#[derive(Debug, Clone, Copy)]
pub struct IkOptions {
    /// Metres of position error one radian of orientation error weighs as.
    pub orientation_weight: f64,
    /// Levenberg damping `lambda` in `J^T (J J^T + lambda^2 I)^-1 e`.
    pub damping: f64,
    /// Largest joint change per iteration (radians or metres).
    pub max_step: f64,
    pub iterations: usize,
    /// Convergence: position error in metres, orientation error in radians.
    pub tolerance: [f64; 2],
}
impl IkOptions {
    pub const ARM: Self = Self { orientation_weight: 0.2, damping: 0.05, max_step: 0.25, iterations: 64, tolerance: [1e-5, 1e-4] };
}
impl Default for IkOptions {
    fn default() -> Self {
        Self::ARM
    }
}

/// The coordinates reached and how far from the target (its envelope) they leave the frame.
#[derive(Debug, Clone, PartialEq)]
pub struct IkSolution {
    pub coordinates: Vec<f64>,
    pub position_error_m: f64,
    /// Zero when the target ignores orientation.
    pub orientation_error_rad: f64,
    pub iterations: usize,
    /// Both errors within the tolerances.
    pub converged: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IkError {
    Chain(DhChainError),
    /// The target frame is not a joint's child frame of this chain.
    Frame { frame: usize, joints: usize },
    /// Limits must pair `lower <= upper` for every joint, starts must be finite.
    Limits { index: usize },
    Coordinates { joints: usize, coordinates: usize },
    /// The target pose or a setting is not finite or not positive where it must be, or the
    /// envelope's bounds are not ordered within their ranges.
    Target,
    /// Model chain: joint `index` of the request is absent, immovable, repeated, or not on the
    /// end link's path from the root, in root-to-end order.
    Path { index: usize },
    /// Model chain: forward kinematics could not pose the model or the end link.
    Model,
}

/// One forward evaluation of a serial chain in base coordinates: for each coordinate, a point on
/// its joint axis and the axis' unit direction, and the pose of the frame the target constrains.
#[derive(Debug, Clone, Default)]
pub struct ChainFrames {
    pub origins: Vec<[f64; 3]>,
    pub axes: Vec<[f64; 3]>,
    pub kinds: Vec<JointType>,
    pub end_position: [f64; 3],
    /// Row-major rotation of the constrained frame.
    pub end_rotation: [f64; 9],
}

/// A serial chain the damped-least-squares solver drives. `frames` lists the coordinates the
/// constrained frame depends on, first to last; the solver leaves the remaining ones at their
/// (clamped) start values.
pub trait SerialChain {
    fn coordinates(&self) -> usize;
    fn frames(&self, q: &[f64], out: &mut ChainFrames) -> Result<(), IkError>;
}

/// The prefix `1..=frame` of a standard-DH chain.
struct DhPrefix<'a> {
    joints: &'a [(JointEncoding, JointType)],
    frame: usize,
}

impl SerialChain for DhPrefix<'_> {
    fn coordinates(&self) -> usize {
        self.joints.len()
    }
    fn frames(&self, q: &[f64], out: &mut ChainFrames) -> Result<(), IkError> {
        let poses = standard_dh_chain(self.joints, q).map_err(IkError::Chain)?;
        out.origins.clear();
        out.axes.clear();
        out.kinds.clear();
        for j in 0..self.frame {
            let frame = matrix(&poses[j]);
            out.origins.push(poses[j].translation);
            out.axes.push([frame[2], frame[5], frame[8]]);
            out.kinds.push(self.joints[j].1);
        }
        out.end_position = poses[self.frame].translation;
        out.end_rotation = matrix(&poses[self.frame]);
        Ok(())
    }
}

/// A root-to-`end` path of an origin/axis [`KinematicModel`], driven by the named movable joints.
/// Joints off the path, and any not named, stay at zero.
pub struct ModelChain<'m> {
    model: &'m KinematicModel,
    joints: Vec<&'m JointInfo>,
    end: String,
}

impl<'m> ModelChain<'m> {
    /// `joints` must be exactly the movable joints on `end`'s path from the root, in that order.
    pub fn new(model: &'m KinematicModel, joints: &[&str], end: &str) -> Result<Self, IkError> {
        Self::from_link(model, None, joints, end)
    }

    /// A limb below `base` (a floating-base robot's leg from its pelvis): `joints` must be exactly
    /// the movable joints on `end`'s path from `base`, in that order. Coordinates stay in the
    /// model root's frame with every joint outside the chain at zero, so a target given relative
    /// to `base` is composed with `base`'s zero-coordinate pose first.
    pub fn below(model: &'m KinematicModel, base: &str, joints: &[&str], end: &str) -> Result<Self, IkError> {
        if !model.links.iter().any(|l| l.name == base) {
            return Err(IkError::Model);
        }
        Self::from_link(model, Some(base), joints, end)
    }

    /// The movable joints on `end`'s path from `base` (the root when `None`), in root-to-end order.
    pub fn path_names(model: &'m KinematicModel, base: Option<&str>, end: &str) -> Vec<&'m str> {
        let by_child: HashMap<&str, &JointInfo> = model.joints.iter().map(|j| (j.child_link.as_str(), j)).collect();
        let mut names = Vec::new();
        let mut link = end;
        while let Some(joint) = by_child.get(link) {
            if Some(link) == base || names.len() > model.joints.len() {
                break;
            }
            if joint.joint_type != JointType::Fixed {
                names.push(joint.name.as_str());
            }
            link = joint.parent_link.as_str();
        }
        names.reverse();
        names
    }

    fn from_link(model: &'m KinematicModel, base: Option<&str>, joints: &[&str], end: &str) -> Result<Self, IkError> {
        let by_child: HashMap<&str, &JointInfo> = model.joints.iter().map(|j| (j.child_link.as_str(), j)).collect();
        let mut path = Vec::new();
        let mut link = end;
        while let Some(joint) = by_child.get(link) {
            if Some(link) == base {
                break;
            }
            if path.len() > model.joints.len() {
                return Err(IkError::Path { index: 0 });
            }
            if joint.joint_type != JointType::Fixed {
                path.push(*joint);
            }
            link = joint.parent_link.as_str();
        }
        path.reverse();
        if !model.links.iter().any(|l| l.name == end) {
            return Err(IkError::Model);
        }
        for (index, name) in joints.iter().enumerate() {
            if path.get(index).is_none_or(|j| j.name != *name) {
                return Err(IkError::Path { index });
            }
        }
        if path.len() != joints.len() {
            return Err(IkError::Path { index: joints.len() });
        }
        Ok(Self { model, joints: path, end: end.to_owned() })
    }
}

impl SerialChain for ModelChain<'_> {
    fn coordinates(&self) -> usize {
        self.joints.len()
    }
    fn frames(&self, q: &[f64], out: &mut ChainFrames) -> Result<(), IkError> {
        let values: JointValues = self.joints.iter().zip(q).map(|(j, v)| (j.name.clone(), *v)).collect();
        let poses = forward_kinematics(self.model, &values).map_err(|_| IkError::Model)?;
        out.origins.clear();
        out.axes.clear();
        out.kinds.clear();
        for joint in &self.joints {
            // The child frame sits on the joint axis, and turning about the axis leaves it fixed.
            let child = poses.get(&joint.child_link).ok_or(IkError::Model)?;
            let r = child.rotation_matrix();
            let a = joint.axis.unwrap_or([1., 0., 0.]);
            let length = norm(&a);
            if !(length > 0.) {
                return Err(IkError::Model);
            }
            let axis = std::array::from_fn(|i| (r[i * 3] * a[0] + r[i * 3 + 1] * a[1] + r[i * 3 + 2] * a[2]) / length);
            out.origins.push(std::array::from_fn(|i| child.translation[i]));
            out.axes.push(axis);
            out.kinds.push(joint.joint_type);
        }
        let end = poses.get(&self.end).ok_or(IkError::Model)?;
        out.end_position = std::array::from_fn(|i| end.translation[i]);
        out.end_rotation = std::array::from_fn(|i| end.rotation_matrix()[i]);
        Ok(())
    }
}

impl std::fmt::Display for IkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Chain(error) => write!(f, "{error}"),
            Self::Frame { frame, joints } => write!(f, "IK frame {frame} is not in 1..={joints}"),
            Self::Limits { index } => write!(f, "IK limits of joint {index} are invalid"),
            Self::Coordinates { joints, coordinates } => write!(f, "IK chain has {joints} joints but {coordinates} coordinates"),
            Self::Target => write!(f, "IK target and settings must be finite (tolerances, steps and weights positive)"),
            Self::Path { index } => write!(f, "IK joint {index} is not the next movable joint on the end link's path"),
            Self::Model => write!(f, "IK forward kinematics could not pose the model or its end link"),
        }
    }
}

impl std::error::Error for IkError {}

/// Move joints `1..=target.frame` from `start` so that frame `target.frame` reaches the target,
/// keeping every coordinate within `limits` (`[lower, upper]` per joint). Later joints keep their
/// start values.
pub fn solve_standard_dh(
    joints: &[(JointEncoding, JointType)],
    limits: &[[f64; 2]],
    start: &[f64],
    target: &IkTarget,
    options: &IkOptions,
) -> Result<IkSolution, IkError> {
    let n = joints.len();
    if start.len() != n || limits.len() != n {
        return Err(IkError::Coordinates { joints: n, coordinates: start.len().min(limits.len()) });
    }
    if target.frame == 0 || target.frame > n {
        return Err(IkError::Frame { frame: target.frame, joints: n });
    }
    let frame = matrix(&target.pose);
    let goal = Goal {
        position: target.pose.translation,
        rotation: target.orientation.then_some(frame),
        envelope: (target.envelope != Envelope::POINT).then_some((target.envelope, frame)),
    };
    solve(&DhPrefix { joints, frame: target.frame }, limits, start, &goal, options)
}

/// Move the chain's coordinates from `start` so its constrained frame reaches `position` (and,
/// given, the row-major `rotation`), keeping every coordinate within `limits`. The contracts of
/// [`solve_standard_dh`] apply; it is this solver on a DH prefix.
pub fn solve_serial<C: SerialChain + ?Sized>(
    chain: &C,
    limits: &[[f64; 2]],
    start: &[f64],
    position: &[f64; 3],
    rotation: Option<&[f64; 9]>,
    options: &IkOptions,
) -> Result<IkSolution, IkError> {
    solve(chain, limits, start, &Goal { position: *position, rotation: rotation.copied(), envelope: None }, options)
}

/// A target pose, or with an envelope (and the target's rotation it is laid along) a region.
struct Goal {
    position: [f64; 3],
    rotation: Option<[f64; 9]>,
    envelope: Option<(Envelope, [f64; 9])>,
}

impl Goal {
    /// Position error and rotation vector from the frame to the target, or to the region's pose
    /// nearest to the frame.
    fn error(&self, at: &[f64; 3], current: &[f64; 9]) -> ([f64; 3], [f64; 3]) {
        let Some((envelope, rotation)) = &self.envelope else {
            return error(at, current, &self.position, self.rotation.as_ref());
        };
        let (position, nearest) = envelope.nearest(&self.position, rotation, at, current, self.rotation.is_some());
        error(at, current, &position, self.rotation.is_some().then_some(&nearest))
    }

    fn valid(&self) -> bool {
        let region = self.envelope.as_ref().is_none_or(|(envelope, rotation)| envelope.is_valid() && rotation.iter().all(|v| v.is_finite()));
        region && self.position.iter().chain(self.rotation.iter().flatten()).all(|v| v.is_finite())
    }
}

fn solve<C: SerialChain + ?Sized>(
    chain: &C,
    limits: &[[f64; 2]],
    start: &[f64],
    goal: &Goal,
    options: &IkOptions,
) -> Result<IkSolution, IkError> {
    let n = chain.coordinates();
    if start.len() != n || limits.len() != n {
        return Err(IkError::Coordinates { joints: n, coordinates: start.len().min(limits.len()) });
    }
    if let Some(index) = (0..n).find(|&i| limits[i].iter().any(|v| v.is_nan()) || limits[i][0] > limits[i][1] || !start[i].is_finite()) {
        return Err(IkError::Limits { index });
    }
    let positive = [options.orientation_weight, options.damping, options.max_step, options.tolerance[0], options.tolerance[1]];
    if !goal.valid() || !positive.iter().all(|v| v.is_finite()) || positive.iter().any(|v| *v <= 0.) {
        return Err(IkError::Target);
    }
    let rows = if goal.rotation.is_some() { 6 } else { 3 };
    let mut q: Vec<f64> = start.iter().zip(limits).map(|(q, [lo, hi])| q.clamp(*lo, *hi)).collect();
    let mut frames = ChainFrames::default();
    let mut iterations = 0;
    loop {
        chain.frames(&q, &mut frames)?;
        let (position, orientation) = goal.error(&frames.end_position, &frames.end_rotation);
        let (dp, dr) = (norm(&position), norm(&orientation));
        let converged = dp <= options.tolerance[0] && dr <= options.tolerance[1];
        if converged || iterations == options.iterations {
            return Ok(IkSolution { coordinates: q, position_error_m: dp, orientation_error_rad: dr, iterations, converged });
        }
        iterations += 1;
        // Weighted task error and Jacobian rows: position in metres, orientation scaled to metres.
        let w = options.orientation_weight;
        let e: Vec<f64> = position.iter().copied().chain(orientation.iter().map(|v| v * w)).take(rows).collect();
        let end = frames.end_position;
        let columns: Vec<[f64; 6]> = (0..frames.axes.len())
            .map(|j| {
                let z = frames.axes[j];
                let origin = frames.origins[j];
                match frames.kinds[j] {
                    JointType::Revolute | JointType::Continuous => {
                        let lever = [end[0] - origin[0], end[1] - origin[1], end[2] - origin[2]];
                        let linear = cross(z, lever);
                        [linear[0], linear[1], linear[2], z[0] * w, z[1] * w, z[2] * w]
                    }
                    JointType::Prismatic => [z[0], z[1], z[2], 0., 0., 0.],
                    JointType::Fixed => [0.; 6],
                }
            })
            .collect();
        // dq = J^T (J J^T + lambda^2 I)^-1 e, the Tensor pillar's damped least squares on the
        // row-major Jacobian (same operation order as the former inline solve).
        let jacobian: Vec<f64> = (0..rows).flat_map(|r| columns.iter().map(move |col| col[r])).collect();
        let mut step = vec![0.; columns.len()];
        if damped_least_squares(&jacobian, rows, columns.len(), &e, options.damping, &mut step).is_none() {
            return Ok(IkSolution { coordinates: q, position_error_m: dp, orientation_error_rad: dr, iterations, converged: false });
        }
        let size = norm(&step);
        if size > options.max_step {
            step.iter_mut().for_each(|s| *s *= options.max_step / size);
        }
        for (j, s) in step.into_iter().enumerate() {
            q[j] = (q[j] + s).clamp(limits[j][0], limits[j][1]);
        }
    }
}

fn matrix(pose: &Se3) -> [f64; 9] {
    let mut m = [0.; 9];
    pose.rotation_matrix_into(&mut m);
    m
}

/// Position error and the rotation vector taking the frame's orientation to the target's.
fn error(at: &[f64; 3], current: &[f64; 9], position: &[f64; 3], rotation: Option<&[f64; 9]>) -> ([f64; 3], [f64; 3]) {
    let p = std::array::from_fn(|i| position[i] - at[i]);
    let Some(rotation) = rotation else {
        return (p, [0.; 3]);
    };
    // R = target * current^T, row-major.
    let r: [f64; 9] = std::array::from_fn(|i| {
        let (row, col) = (i / 3, i % 3);
        (0..3).map(|m| rotation[row * 3 + m] * current[col * 3 + m]).sum()
    });
    (p, rotation_vector(&r))
}

/// The logarithm of a rotation matrix as axis times angle, through its quaternion (stable at pi).
fn rotation_vector(r: &[f64; 9]) -> [f64; 3] {
    let trace = r[0] + r[4] + r[8];
    let (w, x, y, z) = if trace > 0. {
        let s = (trace + 1.).sqrt() * 2.;
        (0.25 * s, (r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s)
    } else if r[0] > r[4] && r[0] > r[8] {
        let s = (1. + r[0] - r[4] - r[8]).sqrt() * 2.;
        ((r[7] - r[5]) / s, 0.25 * s, (r[1] + r[3]) / s, (r[2] + r[6]) / s)
    } else if r[4] > r[8] {
        let s = (1. + r[4] - r[0] - r[8]).sqrt() * 2.;
        ((r[2] - r[6]) / s, (r[1] + r[3]) / s, 0.25 * s, (r[5] + r[7]) / s)
    } else {
        let s = (1. + r[8] - r[0] - r[4]).sqrt() * 2.;
        ((r[3] - r[1]) / s, (r[2] + r[6]) / s, (r[5] + r[7]) / s, 0.25 * s)
    };
    let (w, v) = if w < 0. { (-w, [-x, -y, -z]) } else { (w, [x, y, z]) };
    let sin = norm(&v);
    if sin < 1e-12 {
        return [2. * v[0], 2. * v[1], 2. * v[2]];
    }
    let angle = 2. * sin.atan2(w);
    v.map(|c| c / sin * angle)
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}
