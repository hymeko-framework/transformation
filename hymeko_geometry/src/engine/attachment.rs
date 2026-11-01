//! Optional physical joint properties and a bilateral attachment in the existing solve.
use super::*;

/// Six velocity constraints holding body B at `body A * relative`.
/// Angular drift uses a local SO(3) correction; activation requires less than
/// sixty degrees of orientation error to exclude its antipodal stationary point.
#[derive(Clone, Copy, Debug)]
pub struct RigidAttachment {
    pub bodies: [usize; 2],
    pub relative: Se3,
}

impl Engine {
    /// Effective nonnegative viscous coefficient at a valid joint slot.
    /// A per-joint configuration takes precedence over the global default.
    pub fn damping_at(&self, slot: usize) -> Option<f64> {
        if slot >= self.aba.slots() {
            return None;
        }
        let value = self
            .joint_damping
            .as_ref()
            .map_or(Some(self.damping), |d| d.get(slot).copied())?;
        (value.is_finite() && value >= 0.).then_some(value)
    }

    /// Instantaneous viscous loss from the active joint configuration, in watts.
    /// Invalid public state/configuration is unavailable rather than zero loss.
    pub fn viscous_power(&self) -> Option<f64> {
        if self.qd.len() != self.aba.slots() {
            return None;
        }
        let mut total = 0.;
        for (slot, &rate) in self.qd.iter().enumerate() {
            let damping = self.damping_at(slot)?;
            if !rate.is_finite() {
                return None;
            }
            total += damping * rate * rate;
        }
        total.is_finite().then_some(total)
    }

    /// Set physical rotor inertia and viscous damping by joint slot. Invalid
    /// inputs leave the engine unchanged. Defaults retain global `damping`.
    pub fn set_joint_dynamics(&mut self, inertia: &[f64], damping: &[f64]) -> Result<(), String> {
        if inertia.len() != self.q.len()
            || damping.len() != self.q.len()
            || inertia
                .iter()
                .chain(damping)
                .any(|x| !x.is_finite() || *x < 0.)
        {
            return Err("joint dynamics require finite nonnegative values for every slot".into());
        }
        self.rotor_inertia.copy_from_slice(inertia);
        self.joint_damping = Some(damping.to_vec());
        self.aba.set_armature(inertia);
        Ok(())
    }

    /// Restrict the existing horizontal support plane to named link indices.
    /// The host owns finite tabletop admission; this is still an infinite plane.
    pub fn select_ground_links(&mut self, links: &[usize]) -> Result<(), String> {
        let mut selected = vec![false; self.model.links.len()];
        for &link in links {
            if link >= selected.len() || selected[link] {
                return Err("ground links must be distinct model links".into());
            }
            selected[link] = true;
        }
        self.ground_links = Some(selected);
        Ok(())
    }

    /// Activate or release a rigid attachment without changing positions or
    /// velocities. Its equal/opposite impulses share the contact/stop solve.
    pub fn set_attachment(&mut self, attachment: Option<RigidAttachment>) -> Result<(), String> {
        if let Some(a) = attachment {
            if a.bodies[0] == a.bodies[1]
                || a.bodies.iter().any(|&b| b >= self.model.links.len())
                || a.relative
                    .translation
                    .iter()
                    .chain(&a.relative.rotor)
                    .any(|x| !x.is_finite())
                || (a.relative.rotor.iter().map(|x| x * x).sum::<f64>() - 1.).abs() > 1e-8
                || [1, 2, 4, 7]
                    .iter()
                    .any(|&i| a.relative.rotor[i].abs() > 1e-12)
            {
                return Err(
                    "attachment requires distinct valid bodies and a finite unit rigid transform"
                        .into(),
                );
            }
            let (desired, actual) = self.attachment_poses(a);
            let trace: f64 = (0..3)
                .map(|axis| {
                    let mut unit = [0.; 3];
                    unit[axis] = 1.;
                    desired
                        .rotate(&unit)
                        .iter()
                        .zip(actual.rotate(&unit))
                        .map(|(a, b)| a * b)
                        .sum::<f64>()
                })
                .sum();
            if trace <= 2. {
                return Err("attachment orientation error must be below sixty degrees".into());
            }
        }
        self.attachment = attachment;
        Ok(())
    }

    pub(super) fn filter_ground_contacts(&mut self) {
        if let Some(links) = &self.ground_links {
            self.contacts
                .retain(|c| links[self.colliders[c.collider].link]);
        }
    }

    pub(super) fn expand_ground_manifold(&mut self, height: f64) {
        let original = self.contacts.len();
        for index in 0..original {
            let collider = self.contacts[index].collider;
            let shape = &self.colliders[collider];
            let Convex::Cuboid { half } = shape.shape else {
                continue;
            };
            let mut first = true;
            for corner in 0..8 {
                let local: [f64; 3] = std::array::from_fn(|axis| {
                    shape.offset[axis]
                        + if corner & (1 << axis) == 0 {
                            -half[axis]
                        } else {
                            half[axis]
                        }
                });
                let point = self.poses[shape.link].apply(&local);
                let predicted = self.predicted_poses[shape.link].apply(&local);
                let separation = point[2] - height;
                if separation <= self.contact.margin || predicted[2] - height <= self.contact.margin
                {
                    let contact = GroundContact {
                        collider,
                        point,
                        separation,
                    };
                    if first {
                        self.contacts[index] = contact;
                        first = false;
                    } else {
                        self.contacts.push(contact);
                    }
                }
            }
        }
    }

    fn attachment_poses(&mut self, a: RigidAttachment) -> (Se3, Se3) {
        self.plan.resolve_se3(&self.model, &self.q, &mut self.poses);
        (
            self.poses[a.bodies[0]].compose(&a.relative),
            self.poses[a.bodies[1]],
        )
    }

    pub(super) fn attachment_constraints(&mut self, h: f64) {
        let Some(a) = self.attachment else { return };
        let (wanted, actual) = self.attachment_poses(a);
        let mut angular_error = [0.; 3];
        for axis in 0..3 {
            let mut unit = [0.; 3];
            unit[axis] = 1.;
            let p = wanted.rotate(&unit);
            let q = actual.rotate(&unit);
            for i in 0..3 {
                angular_error[i] +=
                    0.5 * (p[(i + 1) % 3] * q[(i + 2) % 3] - p[(i + 2) % 3] * q[(i + 1) % 3]);
            }
        }
        let reach: Vec<_> = self.aba.order().iter().rev().copied().collect();
        for angular in [false, true] {
            for axis in 0..3 {
                let mut force = vec![0.; self.q.len()];
                let mut column = force.clone();
                let mut direction = [0.; 3];
                direction[axis] = 1.;
                for (side, sign) in [(0, -1.), (1, 1.)] {
                    let body = a.bodies[side];
                    let point = if side == 0 {
                        wanted.translation
                    } else {
                        actual.translation
                    };
                    if angular {
                        // A force couple is a pure world-axis torque. Reuse the
                        // articulated wrench propagation, including fixed joints.
                        let mut r = point;
                        r[(axis + 1) % 3] += 1.;
                        let mut f = [0.; 3];
                        f[(axis + 2) % 3] = 1.;
                        self.aba.constraint_force(
                            &self.poses,
                            body,
                            r,
                            f,
                            &mut self.scratch,
                            &mut column,
                        );
                        for (out, value) in force.iter_mut().zip(&column) {
                            *out += sign * value;
                        }
                        self.aba.constraint_force(
                            &self.poses,
                            body,
                            point,
                            f,
                            &mut self.scratch,
                            &mut column,
                        );
                        for (out, value) in force.iter_mut().zip(&column) {
                            *out -= sign * value;
                        }
                    } else {
                        self.aba.constraint_force(
                            &self.poses,
                            body,
                            point,
                            direction,
                            &mut self.scratch,
                            &mut column,
                        );
                        for (out, value) in force.iter_mut().zip(&column) {
                            *out += sign * value;
                        }
                    }
                }
                let error = if angular {
                    angular_error[axis]
                } else {
                    actual.translation[axis] - wanted.translation[axis]
                };
                self.constraints.push(Constraint {
                    force,
                    reach: reach.clone(),
                    bias: 0.2 * error / h,
                    limit: Limit::Free,
                });
            }
        }
    }
}
