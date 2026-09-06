//! Bounded spatial force laws, independent of GUI and integrator.

/// A field's physical action. Strength is N for uniform force, N/m for
/// attraction/repulsion and Ns/m for drag. Direction applies to uniform force.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldKind {
    Uniform,
    Attractor,
    Repulsor,
    Drag,
}

/// An editable, world-frame spherical field. Radius is its finite support.
#[derive(Clone, Debug, PartialEq)]
pub struct ForceField {
    pub kind: FieldKind,
    pub origin: [f64; 3],
    pub direction: [f64; 3],
    pub radius: f64,
    pub strength: f64,
    pub enabled: bool,
}

impl ForceField {
    /// Reject parameters that exceed the interactive experiment's resource envelope.
    pub fn validate(&self) -> Result<(), &'static str> {
        if !self
            .origin
            .iter()
            .all(|x| x.is_finite() && x.abs() <= 1000.0)
            || !self
                .direction
                .iter()
                .all(|x| x.is_finite() && x.abs() <= 1000.0)
            || !self.radius.is_finite()
            || !(0.001..=100.0).contains(&self.radius)
            || !self.strength.is_finite()
            || !(0.0..=1000.0).contains(&self.strength)
        {
            return Err(
                "field requires finite position/direction, radius 0.001..100 m and strength 0..1000",
            );
        }
        Ok(())
    }

    /// Call `validate` before evaluating user-supplied parameters.
    /// Force at a world point with world velocity. Outside support, the force is zero.
    /// Attractor potential is k*(r²-R²)/2 inside; repulsor potential is k*(R-r)²/2.
    /// Both meet zero potential at the boundary. The repulsor centre returns zero
    /// because an exactly coincident point has no distinguished radial direction.
    pub fn force_at(&self, point: [f64; 3], velocity: [f64; 3]) -> [f64; 3] {
        if !self.enabled || !point.iter().chain(&velocity).all(|v| v.is_finite()) {
            return [0.0; 3];
        }
        let delta: [f64; 3] = std::array::from_fn(|i| point[i] - self.origin[i]);
        let r = delta.iter().map(|v| v * v).sum::<f64>().sqrt();
        if r >= self.radius {
            return [0.0; 3];
        }
        match self.kind {
            FieldKind::Attractor => delta.map(|v| -self.strength * v),
            FieldKind::Repulsor if r > 1e-12 => {
                delta.map(|v| self.strength * (self.radius - r) * v / r)
            }
            FieldKind::Drag => velocity.map(|v| -self.strength * v),
            FieldKind::Uniform => {
                let n = self.direction.iter().map(|v| v * v).sum::<f64>().sqrt();
                if n > 1e-12 {
                    self.direction.map(|v| self.strength * v / n)
                } else {
                    [0.0; 3]
                }
            }
            _ => [0.0; 3],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fields_have_bounded_support_and_physical_direction() {
        let mut f = ForceField {
            kind: FieldKind::Attractor,
            origin: [0.0; 3],
            direction: [0.0, 2.0, 0.0],
            radius: 2.0,
            strength: 3.0,
            enabled: true,
        };
        assert!(f.validate().is_ok());
        assert_eq!(f.force_at([1.0, 0.0, 0.0], [0.0; 3]), [-3.0, 0.0, 0.0]);
        assert_eq!(f.force_at([2.0, 0.0, 0.0], [0.0; 3]), [0.0; 3]);
        f.kind = FieldKind::Repulsor;
        assert_eq!(f.force_at([1.0, 0.0, 0.0], [0.0; 3]), [3.0, 0.0, 0.0]);
        assert_eq!(f.force_at([0.0; 3], [0.0; 3]), [0.0; 3]);
        f.kind = FieldKind::Uniform;
        assert_eq!(f.force_at([0.0; 3], [0.0; 3]), [0.0, 3.0, 0.0]);
        f.kind = FieldKind::Drag;
        let v = [1.0, -2.0, 3.0];
        assert!(
            f.force_at([0.0; 3], v)
                .iter()
                .zip(v)
                .map(|(f, v)| f * v)
                .sum::<f64>()
                <= 0.0
        );
        f.enabled = false;
        assert_eq!(f.force_at([0.0; 3], v), [0.0; 3]);
        f.radius = f64::NAN;
        assert!(f.validate().is_err());
    }
}
