use super::*;

impl PowerGraph {
    /// Nonlinear articulated state. Body energies use the actual spatial inertias;
    /// generalized momentum uses RNEA with zero velocity/gravity and acceleration qd.
    /// Gravity energy is relative to the world origin. Contact impulse work is not
    /// available from the current solver and is explicitly not reported as zero.
    pub fn mechanism(engine: &Engine, time: f64, scope: Option<usize>) -> Option<Self> {
        if !time.is_finite()
            || !engine.damping.is_finite()
            || engine
                .gravity
                .iter()
                .chain(&engine.torque)
                .any(|v| !v.is_finite())
        {
            return None;
        }
        let mut out = Self::mechanical(
            engine.model(),
            engine.dynamics(),
            &engine.q,
            Some(&engine.qd),
            (engine.gravity, time),
            scope,
        )?;
        out.supplied_power = Some(
            engine
                .torque
                .iter()
                .zip(&engine.qd)
                .map(|(t, v)| t * v)
                .sum(),
        );
        out.dissipated_power = Some(engine.damping * engine.qd.iter().map(|v| v * v).sum::<f64>());
        if scope.is_some() {
            out.supplied_power = None;
            out.dissipated_power = None;
        }
        out.note="Nonlinear articulated Hamiltonian. D reports joint viscous loss only. Internal joint reaction powers, contact friction and impact energy jumps are not yet measured.".into();
        Some(out)
    }
    /// Structural power view for a posed mechanism without a dynamical velocity state.
    /// Does not interpret missing velocities or environmental gravity as measured zero.
    pub fn kinematics(model: &KinematicModel, q: &[f64], scope: Option<usize>) -> Option<Self> {
        let plan = AbaPlan::new(model).ok()?;
        let mut out = Self::mechanical(model, &plan, q, None, ([0.0; 3], 0.0), scope)?;
        out.energy = None;
        for n in &mut out.nodes {
            n.energy = None;
            if n.unit == "J" {
                n.value = None;
            }
        }
        out.note="Structural mechanical power projection; velocity, gravity and effort measurements require the physical producer.".into();
        Some(out)
    }
    /// Observe articulated state using its existing owner plan, without integration.
    /// The plan must belong to this model; q and velocity use its joint slots.
    /// Numerical damping armatures must be removed before observing physical momentum.
    /// Returns None for malformed/nonfinite inputs or stale scopes. Boundary powers
    /// remain unknown until supplied by the calling physical producer.
    pub fn mechanical(
        model: &KinematicModel,
        plan: &AbaPlan,
        q: &[f64],
        velocity: Option<&[f64]>,
        environment: ([f64; 3], f64),
        scope: Option<usize>,
    ) -> Option<Self> {
        let (gravity, time) = environment;
        if !time.is_finite() || gravity.iter().any(|v| !v.is_finite())
            || scope.is_some_and(|i| i >= model.links.len())
            || q.len() != plan.slots()
            || q.iter().any(|v| !v.is_finite())
        {
            return None;
        }
        let zero = vec![0.0; plan.slots()];
        let qd = velocity.unwrap_or(&zero);
        if qd.len() != q.len() || qd.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let kinetic = plan.kinetic_energy_by_link(model, q, qd);
        let mut poses = plan.kinematics().identity_poses_se3();
        plan.kinematics().resolve_se3(model, q, &mut poses);
        let potential: Vec<_> = model
            .links
            .iter()
            .zip(&poses)
            .map(|(l, p)| {
                let center = l
                    .origin
                    .as_ref()
                    .filter(|v| v.len() >= 3)
                    .map(|v| [v[0], v[1], v[2]])
                    .unwrap_or([0.0; 3]);
                -l.mass.unwrap_or(0.0)
                    * p.apply(&center)
                        .iter()
                        .zip(gravity)
                        .map(|(x, g)| x * g)
                        .sum::<f64>()
            })
            .collect();
        let h = scope
            .map(|i| kinetic[i] + potential[i])
            .unwrap_or_else(|| kinetic.iter().chain(&potential).sum());
        let mut out = Self::new(model.name.clone(), scope, time, velocity.map(|_| h));
        if let Some(i) = scope {
            let body = &model.links[i];
            out.name = body.name.clone();
            out.selection = Some(body.did);
            out.push(Node::scalar(
                "kinetic".into(),
                body.did,
                "Kinetic storage".into(),
                velocity.map(|_| kinetic[i]),
                "J",
            ));
            out.push(Node::scalar(
                "potential".into(),
                body.did,
                "Gravity potential".into(),
                Some(potential[i]),
                "J",
            ));
            let mut momentum = zero.clone();
            let mut scratch = plan.scratch();
            plan.inverse_dynamics(model, q, &zero, qd, [0.0; 3], &mut scratch, &mut momentum);
            for (j, joint) in model
                .joints
                .iter()
                .enumerate()
                .filter(|(_, j)| j.child_link == body.name || j.parent_link == body.name)
            {
                let linear = joint.joint_type == hymeko_geometry::JointType::Prismatic;
                let mut node = Node::scalar(
                    format!("joint{j}"),
                    joint.did,
                    joint.name.clone(),
                    Some(q[j]),
                    if linear { "m" } else { "rad" },
                );
                node.ports.push(Port {
                    decl: joint.did,
                    name: if linear {
                        "force / velocity"
                    } else {
                        "torque / angular velocity"
                    }
                    .into(),
                });
                out.push(node);
                out.push(Node::scalar(
                    format!("p{j}"),
                    joint.did,
                    "Generalized momentum".into(),
                    velocity.map(|_| momentum[j]),
                    if linear { "kg m/s" } else { "kg m^2/s" },
                ));
                out.connect(format!("joint{j}"), format!("p{j}"));
                out.connect(format!("p{j}"), "kinetic".into());
            }
        } else {
            for (i, l) in model.links.iter().enumerate() {
                let mut node = Node::scalar(format!("body{i}"), l.did, l.name.clone(), None, "J");
                node.scope = Some(i);
                node.energy = velocity.map(|_| kinetic[i] + potential[i]);
                node.ports = model
                    .joints
                    .iter()
                    .filter(|j| j.child_link == l.name || j.parent_link == l.name)
                    .map(|j| Port {
                        decl: j.did,
                        name: j.name.clone(),
                    })
                    .collect();
                out.push(node);
            }
            for j in &model.joints {
                if let (Some(a), Some(b)) = (
                    model.links.iter().position(|l| l.name == j.parent_link),
                    model.links.iter().position(|l| l.name == j.child_link),
                ) {
                    out.connect(format!("body{a}"), format!("body{b}"));
                }
            }
        }
        Some(out)
    }
}

