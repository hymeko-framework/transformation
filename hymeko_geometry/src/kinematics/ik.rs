//! Inverse kinematics of a serial standard-DH chain by damped least squares.
//!
//! The geometric Jacobian comes straight from the chain [`standard_dh_chain`] already evaluates:
//! joint `j` turns (or slides) along the z axis of frame `j - 1`, through that frame's origin. So a
//! target on frame `k` depends on joints `1..=k` only, and every iteration costs one forward pass
//! and one small (3x3 or 6x6) symmetric solve. Damping keeps the step bounded near singularities
//! (an aligned wrist, a stretched arm); a step cap and the joint limits bound it further. The solver
//! never extrapolates beyond the limits and reports what it could not reach instead of failing.
use super::dh::{DhChainError, standard_dh_chain};
use super::joints::{JointEncoding, JointType};
use crate::{collide::cross, contact::replay::dense::solve_in_place};
use hymeko_clifford::Se3;

/// Where frame `frame` (1-based: the child of joint `frame`) should be.
#[derive(Debug, Clone, Copy)]
pub struct IkTarget {
    pub frame: usize,
    pub pose: Se3,
    /// Also match the frame's orientation; otherwise only its origin.
    pub orientation: bool,
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

/// The coordinates reached and how far from the target they leave the frame.
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
    /// The target pose or a setting is not finite or not positive where it must be.
    Target,
}

impl std::fmt::Display for IkError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Chain(error) => write!(f, "{error}"),
            Self::Frame { frame, joints } => write!(f, "IK frame {frame} is not in 1..={joints}"),
            Self::Limits { index } => write!(f, "IK limits of joint {index} are invalid"),
            Self::Coordinates { joints, coordinates } => write!(f, "IK chain has {joints} joints but {coordinates} coordinates"),
            Self::Target => write!(f, "IK target and settings must be finite (tolerances, steps and weights positive)"),
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
    if let Some(index) = (0..n).find(|&i| limits[i].iter().any(|v| v.is_nan()) || limits[i][0] > limits[i][1] || !start[i].is_finite()) {
        return Err(IkError::Limits { index });
    }
    let rotation = matrix(&target.pose);
    let positive = [options.orientation_weight, options.damping, options.max_step, options.tolerance[0], options.tolerance[1]];
    if !target.pose.translation.iter().chain(&rotation).chain(&positive).all(|v| v.is_finite()) || positive.iter().any(|v| *v <= 0.) {
        return Err(IkError::Target);
    }
    let k = target.frame;
    let rows = if target.orientation { 6 } else { 3 };
    let mut q: Vec<f64> = start.iter().zip(limits).map(|(q, [lo, hi])| q.clamp(*lo, *hi)).collect();
    let mut iterations = 0;
    loop {
        let poses = standard_dh_chain(joints, &q).map_err(IkError::Chain)?;
        let (position, orientation) = error(&poses[k], &target.pose.translation, &rotation, target.orientation);
        let (dp, dr) = (norm(&position), norm(&orientation));
        let converged = dp <= options.tolerance[0] && dr <= options.tolerance[1];
        if converged || iterations == options.iterations {
            return Ok(IkSolution { coordinates: q, position_error_m: dp, orientation_error_rad: dr, iterations, converged });
        }
        iterations += 1;
        // Weighted task error and Jacobian rows: position in metres, orientation scaled to metres.
        let w = options.orientation_weight;
        let e: Vec<f64> = position.iter().copied().chain(orientation.iter().map(|v| v * w)).take(rows).collect();
        let end = poses[k].translation;
        let columns: Vec<[f64; 6]> = (0..k)
            .map(|j| {
                let frame = matrix(&poses[j]);
                let z = [frame[2], frame[5], frame[8]];
                let origin = poses[j].translation;
                match joints[j].1 {
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
        // (J J^T + lambda^2 I) y = e, then dq = J^T y.
        let mut a = vec![0.; rows * rows];
        for r in 0..rows {
            for c in 0..rows {
                a[r * rows + c] = columns.iter().map(|col| col[r] * col[c]).sum::<f64>();
            }
            a[r * rows + r] += options.damping * options.damping;
        }
        let mut y = e;
        if solve_in_place(&mut a, &mut y, rows).is_none() {
            return Ok(IkSolution { coordinates: q, position_error_m: dp, orientation_error_rad: dr, iterations, converged: false });
        }
        let mut step: Vec<f64> = columns.iter().map(|col| (0..rows).map(|r| col[r] * y[r]).sum()).collect();
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
fn error(frame: &Se3, position: &[f64; 3], rotation: &[f64; 9], orientation: bool) -> ([f64; 3], [f64; 3]) {
    let p = std::array::from_fn(|i| position[i] - frame.translation[i]);
    if !orientation {
        return (p, [0.; 3]);
    }
    let current = matrix(frame);
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
