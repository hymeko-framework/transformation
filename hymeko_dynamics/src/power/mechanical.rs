use super::*;

impl PowerGraph {
    /// Nonlinear articulated state. Body energies use the actual spatial inertias;
    /// generalized momentum uses RNEA with zero velocity/gravity and acceleration qd.
    /// Gravity energy is relative to the world origin. Contact impulse work is not
    /// available from the current solver and is explicitly not reported as zero.
    pub fn mechanism(engine: &Engine, time: f64, scope: Option<usize>) -> Option<Self> {
        if !time.is_finite()
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
        out.dissipated_power = Some(engine.viscous_power()?);
        if scope.is_none() {
            out.audit = engine.energy_audit().cloned();
        }
        if scope.is_some() {
            out.supplied_power = None;
            out.dissipated_power = None;
        }
        out.note="Nonlinear articulated Hamiltonian. D reports instantaneous joint viscous loss. An enabled energy audit separately records constraint impulse work, the numerical mobility correction and integration residual; these are not continuous friction powers.".into();
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
        for equation in &mut out.equations {
            equation.value = None;
        }
        out.equation_note.push_str(" Kinematikai nézet: sebesség és fizikai gravitáció nélkül T/V/L/H értéke nem mért adat.");
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
        if !time.is_finite()
            || gravity.iter().any(|v| !v.is_finite())
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
        let mut kinetic = plan.kinetic_energy_by_link(model, q, qd);
        // Reflected joint inertia is physical storage. Attribute it to the
        // child for the displayed partition; it is not a second body inertia.
        let indices: std::collections::HashMap<_, _> = model
            .links
            .iter()
            .enumerate()
            .map(|(i, link)| (link.name.as_str(), i))
            .collect();
        for ((joint, &inertia), &rate) in model.joints.iter().zip(plan.armature()).zip(qd) {
            kinetic[*indices.get(joint.child_link.as_str())?] += 0.5 * inertia * rate * rate;
        }
        let potential = plan.potential_energy_by_link(model, q, gravity);
        let h = scope
            .map(|i| kinetic[i] + potential[i])
            .unwrap_or_else(|| kinetic.iter().chain(&potential).sum());
        let mut out = Self::new(model.name.clone(), scope, time, velocity.map(|_| h));
        let t = scope
            .map(|i| kinetic[i])
            .unwrap_or_else(|| kinetic.iter().sum());
        let v = scope
            .map(|i| potential[i])
            .unwrap_or_else(|| potential.iter().sum());
        out.equations = vec![
            Equation::new(
                "Mozgási energia",
                "T = ½ Σᵢ (mᵢ vᵢ·vᵢ + ωᵢ·Iᵢωᵢ) + ½ Σⱼ Iʳⱼ q̇ⱼ²",
                velocity.map(|_| t),
                "J",
            ),
            Equation::new("Gravitációs energia", "V(q) = −Σᵢ mᵢ g·rᵢ(q)", Some(v), "J"),
            Equation::new(
                "Lagrange-függvény",
                "L(q,q̇) = T(q,q̇) − V(q)",
                velocity.map(|_| t - v),
                "J",
            ),
            Equation::new("Hamilton-energia", "H = T + V", velocity.map(|_| h), "J"),
        ];
        if scope.is_none() {
            out.equations.extend([
                Equation::new(
                    "Legendre-kapcsolat",
                    "p = ∂L/∂q̇ = M(q)q̇;  H(q,p) = ½ pᵀM(q)⁻¹p + V(q)",
                    None,
                    "",
                ),
                Equation::new(
                    "Euler–Lagrange-egyenlet",
                    "d/dt (∂L/∂q̇ᵢ) − ∂L/∂qᵢ = Qᵢ",
                    None,
                    "",
                ),
                Equation::new(
                    "Hamilton-egyenletek",
                    "q̇ = ∂H/∂p;  ṗ = −∂H/∂q + Q",
                    None,
                    "",
                ),
            ]);
            out.equation_note = "A T/V/L/H értékek a meglévő testtehetetlenségből, tömegközéppontból és élő állapotból jönnek; Iᵢ és ωᵢ azonos koordinátarendszerben értendő. Az M(q)⁻¹ és a kanonikus Hamilton-alak feltétele: független, reguláris koordináták, invertálható tömegmátrix és időben rögzített kényszerek. E feltételeket ez a megfigyelés nem bizonyítja. Q az összes nem gravitációs általánosított erő, beleértve a hajtást, csillapítást és reakciókat. Az ütközési impulzus külön esemény, nem sima Euler–Lagrange-lépés. A képletek az owner mechanikai modellcsaládját írják le; ez nem modellenként kifejtett szimbolikus deriválás.".into();
        } else {
            out.equation_note = "A Σ csak a kijelölt testre vonatkozik; T/V/L/H a teljes rendszerhez adott hozzájárulás. A test sebessége a kapcsolt rendszer koordinátáitól függ. Ez nem önálló kanonikus alrendszer: a reakcióportok nélkül külön Euler–Lagrange- vagy Hamilton-egyenletet nem állítunk elő.".into();
        }
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
                    // Kinematic joints declare (+ parent, - child, - axis).
                    // This view projects the two body incidences only.
                    out.connect_identified(
                        format!("body{b}"),
                        format!("body{a}"),
                        Some(j.did),
                        None,
                    );
                }
            }
        }
        Some(out)
    }
}
