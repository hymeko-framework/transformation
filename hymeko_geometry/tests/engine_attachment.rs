use hymeko::common::ids::DeclId;
use hymeko_clifford::Se3;
use hymeko_geometry::engine::RigidAttachment;
use hymeko_geometry::{
    Engine, FreeFlight, GeometryInfo, GeometryShape, JointInfo, JointType, KinematicModel, LinkInfo,
};

struct Rig {
    engine: Engine,
    bodies: [usize; 2],
}
impl Rig {
    fn new() -> Self {
        let link = |name: String, mass| LinkInfo {
            did: DeclId::NONE,
            name,
            mass: Some(mass),
            origin: Some(vec![0.; 3]),
            color: None,
            inertia_com: (mass > 0.).then_some([0.1, 0.1, 0.1, 0., 0., 0.]),
            geometry: None,
        };
        let mut model = KinematicModel {
            name: "attachment".into(),
            links: vec![link("base".into(), 0.)],
            joints: vec![],
            frames: vec![],
            actuation: Default::default(),
        };
        let mut bodies = [0; 2];
        for (body, index) in bodies.iter_mut().enumerate() {
            let mut parent = "base".to_string();
            for axis in 0..6 {
                let name = format!("b{body}_{axis}");
                let mut l = link(name.clone(), if axis == 5 { 2. } else { 0. });
                if axis == 5 {
                    l.geometry = Some(GeometryInfo {
                        shape: GeometryShape::Sphere,
                        dimensions: vec![0.1],
                        collision: None,
                    });
                }
                *index = model.links.len();
                model.links.push(l);
                let mut direction = [0.; 3];
                direction[axis % 3] = 1.;
                model.joints.push(JointInfo {
                    did: DeclId::NONE,
                    name: name.clone(),
                    joint_type: if axis < 3 {
                        JointType::Prismatic
                    } else {
                        JointType::Continuous
                    },
                    parent: DeclId::NONE,
                    child: DeclId::NONE,
                    parent_link: parent,
                    child_link: name.clone(),
                    axis: Some(direction),
                    origin_xyz: None,
                    origin_rpy_deg: None,
                    limits: None,
                });
                parent = name;
            }
        }
        let mut engine = Engine::new(model).unwrap();
        engine.ground = None;
        engine.gravity = [0.; 3];
        engine.free_flight = FreeFlight::Never;
        engine.contact_step = 0.001;
        Self { engine, bodies }
    }
    fn attach(&mut self) {
        self.engine
            .set_attachment(Some(RigidAttachment {
                bodies: self.bodies,
                relative: Se3::identity(),
            }))
            .unwrap();
    }
    fn steps(&mut self, n: usize) {
        for _ in 0..n {
            self.engine.advance(0.001);
        }
    }
    fn assert_pair_motion(&self, slot: usize, velocity: f64) {
        for i in [slot, slot + 6] {
            assert!((self.engine.qd[i] - velocity).abs() < 1e-8);
        }
        assert!((self.engine.q[slot] - self.engine.q[slot + 6]).abs() < 1e-9);
    }
    fn configure_drive(&mut self) -> Vec<f64> {
        let mut inertia = vec![0.; 12];
        let mut damping = vec![0.; 12];
        inertia[0] = 3.;
        damping[0] = 2.;
        self.engine.set_joint_dynamics(&inertia, &damping).unwrap();
        self.engine.qd[0] = 1.;
        self.engine.torque[0] = 7.;
        inertia
    }
    fn audited_step(&mut self) -> &hymeko_geometry::engine::EnergyAudit {
        self.engine.enable_energy_audit().unwrap();
        self.steps(1);
        let audit = self.engine.energy_audit().unwrap();
        assert!(audit.complete);
        assert!(audit.maximum_impulse_residual < 1e-11);
        audit
    }
}

#[test]
fn attachment_transmits_reaction_and_release_preserves_state() {
    let mut r = Rig::new();
    r.attach();
    r.engine.free_flight = FreeFlight::DormandPrince { step: 0.02 };
    r.engine.torque[0] = 4.;
    r.steps(100);
    r.assert_pair_motion(0, 0.1);
    let before = (r.engine.q.clone(), r.engine.qd.clone());
    r.engine.set_attachment(None).unwrap();
    assert_eq!(before, (r.engine.q.clone(), r.engine.qd.clone()));
    r.steps(100);
    assert!((r.engine.qd[0] - 0.3).abs() < 1e-8);
    assert!((r.engine.qd[6] - 0.1).abs() < 1e-8);
}

#[test]
fn attachment_holds_orientation_under_applied_torque() {
    for axis in 3..6 {
        let mut r = Rig::new();
        r.attach();
        r.engine.torque[axis] = 0.2;
        r.steps(100);
        r.assert_pair_motion(axis, 0.1);
    }
}

#[test]
fn attachment_and_selected_support_share_the_solve() {
    let mut r = Rig::new();
    r.engine.gravity = [0., 0., -9.80665];
    r.engine.ground = Some(0.);
    r.engine.q[2] = 0.1;
    r.engine.q[8] = 0.1;
    r.engine.contact.friction = 0.;
    r.engine.select_ground_links(&[r.bodies[1]]).unwrap();
    r.attach();
    r.steps(100);
    assert!((r.engine.q[2] - 0.1).abs() < 1e-7);
    assert!((r.engine.q[8] - 0.1).abs() < 1e-7);
    assert_eq!(r.engine.contacts().len(), 1);
    r.engine.set_attachment(None).unwrap();
    r.steps(50);
    assert!(r.engine.q[2] < 0.09);
    assert!((r.engine.q[8] - 0.1).abs() < 1e-7);
}

#[test]
fn joint_dynamics_matches_implicit_damping_and_retains_physical_armature() {
    let mut r = Rig::new();
    let inertia = r.configure_drive();
    r.engine.qd[6] = 1.;
    r.steps(1);
    assert!((r.engine.qd[0] - (5. + 0.007) / (5. + 0.002)).abs() < 1e-12);
    assert_eq!(r.engine.qd[6], 1.);
    assert_eq!(r.engine.dynamics().armature(), inertia);
}

#[test]
fn invalid_configuration_is_atomic_and_attachment_has_an_orientation_admission_bound() {
    let mut r = Rig::new();
    r.attach();
    let before = r.engine.dynamics().armature().to_vec();
    assert!(r.engine.set_joint_dynamics(&[1.], &[1.]).is_err());
    let mut bad = vec![0.; 12];
    bad[0] = f64::NAN;
    assert!(r.engine.set_joint_dynamics(&bad, &vec![0.; 12]).is_err());
    assert_eq!(r.engine.dynamics().armature(), before);
    assert!(r.engine.select_ground_links(&[999]).is_err());
    assert!(r.engine.select_ground_links(&[1, 1]).is_err());
    assert!(
        r.engine
            .set_attachment(Some(RigidAttachment {
                bodies: [1, 1],
                relative: Se3::identity()
            }))
            .is_err()
    );
    assert!(
        r.engine
            .set_attachment(Some(RigidAttachment {
                bodies: r.bodies,
                relative: Se3::axis_angle([0., 0., 1.], std::f64::consts::PI)
            }))
            .is_err()
    );
    r.engine.torque[0] = 4.;
    r.steps(10);
    assert!((r.engine.qd[0] - r.engine.qd[6]).abs() < 1e-9);
}

#[test]
fn selected_ground_also_applies_to_free_flight_admission() {
    let mut r = Rig::new();
    r.engine.ground = Some(0.);
    r.engine.select_ground_links(&[r.bodies[1]]).unwrap();
    r.engine.q[2] = -1.;
    r.engine.q[8] = 2.;
    r.engine.qd[8] = -1.;
    r.engine.free_flight = FreeFlight::DormandPrince { step: 0.01 };
    r.engine.advance(0.01);
    assert!((r.engine.q[8] - 1.99).abs() < 1e-12);
    assert!(r.engine.contacts().is_empty());
}

#[test]
fn supported_box_does_not_generate_spurious_rotation_at_rest() {
    let mut r = Rig::new();
    let body = r.bodies[1];
    let mut model = r.engine.model().clone();
    model.links[body].geometry = Some(GeometryInfo {
        shape: GeometryShape::Box,
        dimensions: vec![0.2; 3],
        collision: None,
    });
    r.engine = Engine::new(model).unwrap();
    r.engine.free_flight = FreeFlight::Never;
    r.engine.contact_step = 0.001;
    r.engine.ground_manifold = true;
    r.engine.contact.iterations = 128;
    r.engine.contact.tolerance = 1e-10;
    r.engine.select_ground_links(&[body]).unwrap();
    r.engine.q[8] = 0.1;
    r.engine.contact.friction = 0.8;
    r.steps(100);
    assert!(
        r.engine.qd[9..12].iter().all(|w| w.abs() < 1e-8),
        "spurious angular velocity {:?}",
        &r.engine.qd[9..12]
    );
    assert!((r.engine.q[8] - 0.1).abs() < 1e-8);
}

#[test]
fn energy_audit_preserves_trajectory_and_exposes_euler_residual() {
    let mut observed = Rig::new();
    let mut reference = Rig::new();
    for rig in [&mut observed, &mut reference] {
        rig.configure_drive();
    }
    assert!(reference.engine.energy_audit().is_none());
    observed.engine.enable_energy_audit().unwrap();
    observed.steps(1);
    reference.steps(1);
    assert_eq!(observed.engine.q, reference.engine.q);
    assert_eq!(observed.engine.qd, reference.engine.qd);
    let a = observed.engine.energy_audit().unwrap();
    let v = (5. + 0.007) / (5. + 0.002);
    assert!(a.complete);
    assert_eq!(a.steps, 1);
    assert!((a.energy - 2.5 * v * v).abs() < 1e-12);
    assert!((a.residual + 2.5 * (v - 1.).powi(2)).abs() < 1e-12);
    assert!((a.free_velocity_numerical_loss - 2.5 * (v - 1.).powi(2)).abs() < 1e-12);
    assert!((a.maximum_free_velocity_increment - (v - 1.).abs()).abs() < 1e-12);
    assert!(a.viscous_work > 0.);
    assert_eq!(a.impulse_kinetic_change, 0.);
}

#[test]
fn attachment_impulse_work_accounts_for_kinetic_jump_and_numerical_metric() {
    for damping in [0., 2.] {
        let mut r = Rig::new();
        r.engine.qd[0] = 1.;
        r.attach();
        let mut d = vec![0.; 12];
        d[0] = damping;
        r.engine.set_joint_dynamics(&vec![0.; 12], &d).unwrap();
        let a = r.audited_step();
        assert_eq!(a.attachment_steps, 1);
        assert!(a.constraints.attachment_work < 0.);
        assert!(a.maximum_impulse_residual < 1e-12);
        assert!(
            (a.impulse_kinetic_change
                - a.constraints.attachment_work
                - a.constraints.implicit_metric_work)
                .abs()
                < 1e-12
        );
        if damping == 0. {
            assert!((a.constraints.attachment_work + 0.5).abs() < 1e-12);
            assert!(a.residual.abs() < 1e-12);
        } else {
            // Removing the numerical metric correction is a discriminating fault.
            assert!((a.impulse_kinetic_change - a.constraints.attachment_work).abs() > 1e-5);
        }
    }
}

#[test]
fn normal_and_friction_impulse_work_are_observed_separately() {
    let mut r = Rig::new();
    r.engine.q[8] = 0.1;
    r.engine.qd[6] = 1.;
    r.engine.qd[8] = -1.;
    r.engine.gravity = [0., 0., -9.80665];
    r.engine.ground = Some(0.);
    r.engine.contact.friction = 0.5;
    r.engine.contact.iterations = 128;
    r.engine.select_ground_links(&[r.bodies[1]]).unwrap();
    let a = r.audited_step();
    assert_eq!(a.contact_steps, 1);
    assert!(a.constraints.normal_work < 0.);
    assert!(a.constraints.friction_work < 0.);
    assert_eq!(a.constraints.attachment_work, 0.);
    assert!(a.maximum_impulse_residual < 1e-11);
    assert!(
        (a.impulse_kinetic_change - a.constraints.normal_work - a.constraints.friction_work).abs()
            < 1e-11
    );
}

#[test]
fn audit_marks_unobserved_free_flight_and_external_energy_changes_incomplete() {
    let mut r = Rig::new();
    r.engine.enable_energy_audit().unwrap();
    r.engine.free_flight = FreeFlight::DormandPrince { step: 0.01 };
    r.engine.advance(0.01);
    let a = r.engine.energy_audit().unwrap();
    assert!(!a.complete);
    assert_eq!(a.unobserved_steps, 1);
    let mut r = Rig::new();
    r.engine.enable_energy_audit().unwrap();
    r.engine.qd[0] = 2.;
    r.steps(1);
    assert!(!r.engine.energy_audit().unwrap().complete);
    assert!(r.engine.energy_at(&[], &[]).is_none());
    r.engine.qd[0] = f64::NAN;
    assert!(r.engine.enable_energy_audit().is_err());
    assert!(r.engine.damping_at(12).is_none());
}

#[test]
fn stop_impulses_are_not_mislabelled_as_ground_or_attachment_work() {
    let source = Rig::new();
    let mut model = source.engine.model().clone();
    model.joints[0].limits = Some(hymeko_geometry::kinematics::joints::JointLimits {
        lower: -1.,
        upper: 0.,
        effort: 10.,
        velocity: 10.,
    });
    let mut e = Engine::new(model).unwrap();
    e.ground = None;
    e.gravity = [0.; 3];
    e.free_flight = FreeFlight::Never;
    e.qd[0] = 1.;
    e.enable_energy_audit().unwrap();
    e.advance(0.001);
    let a = e.energy_audit().unwrap();
    assert!(a.complete);
    assert!((a.constraints.limit_work + 1.).abs() < 1e-12);
    assert_eq!(a.constraints.normal_work, 0.);
    assert_eq!(a.constraints.attachment_work, 0.);
    assert!(a.maximum_impulse_residual < 1e-12);
    assert!(a.residual.abs() < 1e-12);
}
