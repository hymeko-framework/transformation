//! Analytic two-bone inverse kinematics with a pole: shoulder-elbow-wrist, hip-knee-ankle.
//!
//! Closed form, no iteration and no dependency, so it is compiled without the `simulation`
//! feature: Kawamirai's skeletal animation (its `ik::Limb` binds this solver to a skeleton) and
//! robot controllers share one implementation. Serial chains of more joints use the damped
//! least squares of `kinematics::ik`.
//!
//! The pole picks the bend plane, including at full extension where the target alone leaves it
//! undetermined. Bend limits (zero is straight, pi is folded) bound the reach from both sides: a
//! target too far or too near is replaced by the nearest reachable point on the same ray and the
//! distance to the request is reported, never an error.

/// Why a limb or a request was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TwoBoneError {
    /// Lengths must lie in `1e-4..=1000`, bend limits in `0..=pi` with `lower <= upper`.
    Geometry,
    /// Root, target and pole must be finite and within ten kilometres of the origin.
    Points,
}

impl TwoBoneError {
    /// A human-readable reason, stable for consumers that report it.
    pub fn message(self) -> &'static str {
        match self {
            Self::Geometry => "IK requires positive lengths and ordered bend limits in 0..pi",
            Self::Points => "IK points must be finite and bounded",
        }
    }
}

impl std::fmt::Display for TwoBoneError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.message())
    }
}

impl std::error::Error for TwoBoneError {}

/// Middle and end joint positions reached for a request.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoBoneSolution {
    /// The middle joint (elbow, knee).
    pub elbow: [f64; 3],
    /// The end joint (wrist, ankle), on the root-target ray.
    pub end: [f64; 3],
    /// Distance from the requested end point after reach and bend clamping.
    pub residual: f64,
    /// Bend at the middle joint: zero is straight, pi is folded.
    pub bend: f64,
}

/// Bone lengths and bend limits of one limb.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoBone {
    lengths: [f64; 2],
    bend: [f64; 2],
}

impl TwoBone {
    /// `lengths` are root-middle and middle-end; `bend` the lower and upper bend limits.
    pub fn new(lengths: [f64; 2], bend: [f64; 2]) -> Result<Self, TwoBoneError> {
        let lengths_ok = lengths.iter().all(|v| v.is_finite() && (1e-4..=1000.).contains(v));
        let bend_ok = bend.iter().all(|v| v.is_finite() && (0. ..=std::f64::consts::PI).contains(v)) && bend[0] <= bend[1];
        if lengths_ok && bend_ok { Ok(Self { lengths, bend }) } else { Err(TwoBoneError::Geometry) }
    }

    /// Root-middle and middle-end lengths.
    pub fn lengths(&self) -> [f64; 2] {
        self.lengths
    }

    /// Lower and upper bend limits.
    pub fn bend_limits(&self) -> [f64; 2] {
        self.bend
    }

    /// Root-to-end distance at a bend angle (law of cosines).
    pub fn reach(&self, bend: f64) -> f64 {
        let [a, b] = self.lengths;
        ((a - b) * (a - b) + 2. * a * b * (1. + bend.cos())).max(0.).sqrt()
    }

    /// Place the middle and end joints for `target`, bending in the plane through the root-target
    /// line toward `pole`. A pole on that line falls back to world +Y (or +Z when the line is
    /// near vertical); a target at the root folds the limb toward the pole.
    pub fn solve(&self, root: [f64; 3], target: [f64; 3], pole: [f64; 3]) -> Result<TwoBoneSolution, TwoBoneError> {
        if [root, target, pole].iter().any(|p| p.iter().any(|v| !v.is_finite() || v.abs() > 10000.)) {
            return Err(TwoBoneError::Points);
        }
        let [a, b] = self.lengths;
        let offset = sub(target, root);
        let direction = normalized(offset).unwrap_or([0., -1., 0.]);
        let toward_pole = sub(pole, root);
        let mut plane = sub(toward_pole, scale(direction, dot(direction, toward_pole)));
        if dot(plane, plane) < 1e-10 {
            let axis = if direction[1].abs() < 0.9 { [0., 1., 0.] } else { [0., 0., 1.] };
            plane = sub(axis, scale(direction, dot(direction, axis)));
        }
        let plane = normalized(plane).unwrap_or([1., 0., 0.]);
        let distance = norm(offset).clamp(self.reach(self.bend[1]), self.reach(self.bend[0]));
        if distance < 1e-6 {
            return Ok(TwoBoneSolution {
                elbow: add(root, scale(plane, a)),
                end: root,
                residual: norm(offset),
                bend: std::f64::consts::PI,
            });
        }
        let along = (distance * distance + (a - b) * (a + b)) / (2. * distance);
        let height = ((a - along) * (a + along)).max(0.).sqrt();
        let end = add(root, scale(direction, distance));
        let cosine = ((distance * distance - a * a - b * b) / (2. * a * b)).clamp(-1., 1.);
        Ok(TwoBoneSolution {
            elbow: add(add(root, scale(direction, along)), scale(plane, height)),
            end,
            residual: norm(sub(end, target)),
            bend: cosine.acos(),
        })
    }
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalized(a: [f64; 3]) -> Option<[f64; 3]> {
    let n = norm(a);
    (n.is_finite() && n > 0.).then(|| scale(a, 1. / n))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
        norm(sub(a, b))
    }

    #[test]
    fn reaches_a_reachable_target_with_bone_lengths_kept_and_the_bend_toward_the_pole() {
        let limb = TwoBone::new([0.4, 0.41], [0.0, 2.5]).unwrap();
        let (root, target, pole) = ([0., 0., 1.], [0.2, 0., 0.35], [1., 0., 0.6]);
        let s = limb.solve(root, target, pole).unwrap();
        assert!(s.residual < 1e-12, "{s:?}");
        assert!((distance(root, s.elbow) - 0.4).abs() < 1e-12 && (distance(s.elbow, s.end) - 0.41).abs() < 1e-12);
        // The elbow lies on the pole's side of the root-target line.
        let line = normalized(sub(target, root)).unwrap();
        let side = sub(sub(s.elbow, root), scale(line, dot(line, sub(s.elbow, root))));
        let pole_side = sub(sub(pole, root), scale(line, dot(line, sub(pole, root))));
        assert!(dot(side, pole_side) > 0.);
        // Law of cosines for the reported bend.
        assert!((limb.reach(s.bend) - distance(root, target)).abs() < 1e-12);
    }

    #[test]
    fn clamps_too_far_and_too_near_targets_by_the_bend_limits_and_reports_the_residual() {
        let limb = TwoBone::new([0.5, 0.5], [0.2, 2.0]).unwrap();
        let far = limb.solve([0.; 3], [3., 0., 0.], [0., 1., 0.]).unwrap();
        assert!((far.bend - 0.2).abs() < 1e-12 && (far.residual - (3. - limb.reach(0.2))).abs() < 1e-12);
        let near = limb.solve([0.; 3], [0.1, 0., 0.], [0., 1., 0.]).unwrap();
        assert!((near.bend - 2.0).abs() < 1e-9 && (norm(near.end) - limb.reach(2.0)).abs() < 1e-12);
    }

    #[test]
    fn a_pole_on_the_line_and_a_target_at_the_root_still_give_a_definite_limb() {
        let limb = TwoBone::new([0.3, 0.3], [0.0, std::f64::consts::PI]).unwrap();
        let on_line = limb.solve([0.; 3], [0., 0., -0.5], [0., 0., -2.]).unwrap();
        assert!(on_line.residual < 1e-12 && on_line.elbow.iter().all(|v| v.is_finite()));
        let folded = limb.solve([0.; 3], [0.; 3], [1., 0., 0.]).unwrap();
        assert_eq!(folded.bend, std::f64::consts::PI);
        assert!((folded.elbow[0] - 0.3).abs() < 1e-12);
    }

    #[test]
    fn refuses_invalid_limbs_and_points_with_stable_messages() {
        assert_eq!(TwoBone::new([0.0, 1.0], [0.0, 1.0]), Err(TwoBoneError::Geometry));
        assert_eq!(TwoBone::new([1.0, 1.0], [1.0, 0.5]), Err(TwoBoneError::Geometry));
        let limb = TwoBone::new([1.0, 1.0], [0.0, 1.0]).unwrap();
        assert_eq!(limb.solve([f64::NAN, 0., 0.], [1.; 3], [0.; 3]), Err(TwoBoneError::Points));
        assert_eq!(TwoBoneError::Points.message(), "IK points must be finite and bounded");
    }
}
