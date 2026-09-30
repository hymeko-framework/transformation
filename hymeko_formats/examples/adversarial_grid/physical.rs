//! Compose the existing Cyberdisc actor with a checked grid; physics remains in Kawakern.
use super::*;
use hymeko::module_store::{execution::CompiledModel, source_provider::StdFsProvider};
use hymeko_geometry::{
    collide::colliders_of, forward::FkPlan, kinematics::kinematic::extract_kinematic_model,
};

const CELL_SIZE: f64 = 2.0;
const MAX_OBSTACLES: usize = 64;
const MAX_WAYPOINTS: usize = 32;

fn quoted_path(path: &Path) -> Result<String> {
    Ok(serde_json::to_string(
        &path.canonicalize()?.to_string_lossy().replace('\\', "/"),
    )?)
}

fn physical_point(layout: &Layout, point: Point) -> [f64; 2] {
    [
        (point.x as f64 - (layout.width as f64 - 1.) * 0.5) * CELL_SIZE,
        (point.y as f64 - (layout.height as f64 - 1.) * 0.5) * CELL_SIZE,
    ]
}

struct ActorContract {
    links: usize,
    joints: usize,
    radius: f64,
}
impl ActorContract {
    fn read(path: &Path) -> Result<Self> {
        let document = CompiledModel::compile(StdFsProvider::new(), RealParser, path)
            .map_err(|e| format!("actor compile: {e:?}"))?;
        let query = QueryEngine::new(&document.program().ir, document.interner());
        let model = extract_kinematic_model(&query, "cyberdisc");
        let plan = FkPlan::new(&model)?;
        for name in [
            "base_x", "base_y", "base_z", "base_yaw", "drive_fl", "drive_rl", "drive_fr",
            "drive_rr",
        ] {
            if plan.slot(name).is_none() {
                return Err(format!("Cyberdisc actor lacks {name}").into());
            }
        }
        if model.links.len() != 13 || model.joints.len() != 12 {
            return Err("use the reusable Cyberdisc actor, without its old course".into());
        }
        let chassis = model
            .links
            .iter()
            .position(|l| l.name == "chassis")
            .ok_or("missing chassis")?;
        let mut values = plan.zero_values();
        values[plan.slot("base_z").ok_or("missing base_z")?] = 0.261;
        let mut poses = plan.identity_poses_se3();
        plan.resolve_se3(&model, &values, &mut poses);
        let centre = poses[chassis].apply(&[0.; 3]);
        let radius = colliders_of(&model)
            .iter()
            .map(|c| {
                let p = hymeko_geometry::collide::Placement::of(c, &poses[c.link]).centre;
                p.iter()
                    .zip(centre)
                    .map(|(a, b)| (a - b).powi(2))
                    .sum::<f64>()
                    .sqrt()
                    + c.shape.bounding_radius()
            })
            .fold(0., f64::max)
            + 0.04;
        // Conservative yaw-independent envelope plus the existing controller's arrival tolerance.
        if !radius.is_finite() || radius + 0.16 >= CELL_SIZE * 0.5 {
            return Err("robot footprint/tolerance does not fit the declared cell scale".into());
        }
        Ok(Self {
            links: model.links.len(),
            joints: model.joints.len(),
            radius,
        })
    }
}

impl CompiledWorld {
    pub(super) fn export_physics(&self, actor: &Path, output: &Path) -> Result<()> {
        if output.exists() {
            return Err("physics output directory must be new".into());
        }
        let actor = actor.canonicalize()?;
        let contract = ActorContract::read(&actor)?;
        let projection = self.project()?;
        let evaluation = projection.evaluate(self, false)?;
        if !evaluation.feasible {
            return Err("physical demo requires a verified path witness".into());
        }
        let layout = &projection.layout;
        let waypoints = &evaluation.path[1..];
        if layout.walls.len() > MAX_OBSTACLES || waypoints.len() > MAX_WAYPOINTS {
            return Err(
                "world exceeds 64 obstacles or 32 controller waypoints; nothing was truncated"
                    .into(),
            );
        }
        let meta =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/robotics/meta_kinematics.hymeko");
        let mut model = format!(
            "generated_robot_description {{ @{} -> actor; @{}; using actor.cyberdisc as mobile; }}\ngenerated_environment {{\n",
            quoted_path(&actor)?,
            quoted_path(&meta)?
        );
        for (i, wall) in layout.walls.iter().enumerate() {
            let [x, y] = physical_point(layout, *wall);
            writeln!(
                model,
                "grid_wall_{i}: meta_kinematics.kinematics.elements.link {{ color [0.25,0.35,0.45,1]; origin [0,0,0]; link_geometry: meta_kinematics.kinematics.geometry.box {{ dimension [{CELL_SIZE},{CELL_SIZE},1]; }} visual -> link_geometry; collision -> link_geometry; }}"
            )?;
            writeln!(
                model,
                "@grid_wall_mount_{i}: meta_kinematics.kinematics.fixed_joint {{ (+ mobile.world [[{x},{y},0.5],[0,0,0]], - grid_wall_{i}); }}"
            )?;
        }
        for (i, point) in waypoints.iter().enumerate() {
            let [x, y] = physical_point(layout, *point);
            writeln!(
                model,
                "grid_target_{i}: meta_kinematics.kinematics.elements.link {{}}\n@grid_target_mount_{i}: meta_kinematics.kinematics.fixed_joint {{ (+ mobile.world [[{x},{y},0],[0,0,0]], - grid_target_{i}); }}"
            )?;
        }
        model.push_str("}\n");
        fs::create_dir_all(output)?;
        let model_path = output.join("model.hymeko");
        fs::write(&model_path, model)?;
        let compiled = CompiledModel::compile(StdFsProvider::new(), RealParser, &model_path)
            .map_err(|e| format!("physical composition: {e:?}"))?;
        let query = QueryEngine::new(&compiled.program().ir, compiled.interner());
        let combined = extract_kinematic_model(&query, "cyberdisc");
        FkPlan::new(&combined)?;
        let colliders = colliders_of(&combined);
        for i in 0..layout.walls.len() {
            let name = format!("grid_wall_{i}");
            let index = combined
                .links
                .iter()
                .position(|l| l.name == name)
                .ok_or("missing physical obstacle")?;
            let geometry = combined.links[index]
                .geometry
                .as_ref()
                .ok_or("obstacle has no owner-projected collision geometry")?;
            if !matches!(geometry.shape, hymeko_geometry::GeometryShape::Box)
                || geometry.dimensions != [CELL_SIZE, CELL_SIZE, 1.]
                || !colliders.iter().any(|c| c.link == index)
            {
                return Err(
                    "obstacle geometry was lost or changed during physical projection".into(),
                );
            }
        }
        if combined.links.len() != contract.links + layout.walls.len() + waypoints.len()
            || combined.joints.len() != contract.joints + layout.walls.len() + waypoints.len()
        {
            return Err("physical composition lost or duplicated links/joints".into());
        }
        let [sx, sy] = physical_point(layout, layout.start);
        let floor = layout.width.max(layout.height) as f64 * CELL_SIZE * 0.5 + 1.;
        let mut settings = format!(
            "source {}; model \"cyberdisc\"; q_base_x {sx}; q_base_y {sy}; q_base_z 0.261; q_base_yaw 90; force_body \"chassis\"; force_duration 0; damping 0; gravity_enabled 1; gravity 9.80665; ground_enabled 1; ground_height 0; ground_friction 0.8; physics_hz 240; contact_iterations 64; floor {floor}; drive_body \"chassis\"; drive_yaw \"base_yaw\"; drive_left_front \"drive_fl\"; drive_left_rear \"drive_rl\"; drive_right_front \"drive_fr\"; drive_right_rear \"drive_rr\"; drive_speed 0.65; drive_gain 3; drive_torque 6; drive_tolerance 0.16; drive_obstacles {}; drive_waypoints {};\n",
            quoted_path(&model_path)?,
            layout.walls.len(),
            waypoints.len()
        );
        for i in 0..layout.walls.len() {
            writeln!(settings, "drive_obstacle_{i} \"grid_wall_{i}\";")?;
        }
        for i in 0..waypoints.len() {
            writeln!(settings, "drive_target_{i} \"grid_target_{i}\";")?;
        }
        let scene = format!(
            "physical_grid_scenes {{}}\ngenerated {{ title \"Cyberdisc - generated world - Kawakern\"; producer \"robot.forces\"; settings {{ {settings} }} view {{ eye [-26,-36,32]; target [0,0,0]; up [0,0,1]; fovy 43; znear 0.05; zfar 150; }} look {{ shadows 1; ao 1; bounce 1; environment 1; reflective 1; procedural 1; bump 1; }} }}\ninspection {{ title \"Cyberdisc - physical wheels\"; producer \"robot.forces\"; settings -> generated.settings; look -> generated.look; view {{ eye [{},{},5]; target [{},{},0.3]; up [0,0,1]; fovy 48; znear 0.03; zfar 150; }} }}\n",
            sx + 4.,
            sy - 6.,
            sx + 2.5,
            sy
        );
        fs::write(output.join("scene.hymeko"), scene)?;
        let receipt = json!({"source_grid_digest":self.digest(),"query_checks":check_receipts(&projection.checks),"actor_source":actor,
            "actor_links":contract.links,"actor_joints":contract.joints,"physical_links":combined.links.len(),"physical_joints":combined.joints.len(),
            "obstacles":layout.walls.len(),"waypoints":waypoints.len(),"cell_size_m":CELL_SIZE,"centred":true,"robot_clearance_radius_m":contract.radius,
            "arrival_tolerance_m":0.16,"path_grid":evaluation.path,"initial_yaw_degrees":90,"scope":"real Kawakern wheel drive; collision guard stops at last safe state, not wall impulse response"});
        fs::write(
            output.join("composition.json"),
            serde_json::to_vec_pretty(&receipt)?,
        )?;
        println!(
            "Physical scene: {} links, {} obstacles, {} waypoints -> {}",
            combined.links.len(),
            layout.walls.len(),
            waypoints.len(),
            output.display()
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "pure actor projection requiring the sibling robotics checkout; no simulation"]
    fn footprint_includes_authored_collision_offset() {
        let actor = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../robotics/examples/cyberdisc/actor.hymeko");
        let meta =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../data/robotics/meta_kinematics.hymeko");
        let original = fs::read_to_string(&actor).unwrap();
        let modified = original
            .replace(
                "\"../../../transformation/data/robotics/meta_kinematics.hymeko\"",
                &quoted_path(&meta).unwrap(),
            )
            .replacen("origin [0, 0, 0];", "origin [2, 0, 0];", 1);
        assert!(modified.contains("origin [2, 0, 0];"));
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("hymeko-actor-offset-{unique}.hymeko"));
        fs::write(&path, modified).unwrap();
        let error = match ActorContract::read(&path) {
            Ok(_) => panic!("offset collider incorrectly admitted"),
            Err(e) => e.to_string(),
        };
        assert!(error.contains("footprint"), "{error}");
        fs::remove_file(path).unwrap();
    }

    #[test]
    #[ignore = "pure import/projection test requiring the sibling Cyberdisc actor; no simulation"]
    fn physical_export_has_real_collision_geometry() {
        let actor = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../robotics/examples/cyberdisc/actor.hymeko");
        let layout = Layout {
            width: 4,
            height: 4,
            start: Point { x: 0, y: 0 },
            goal: Point { x: 3, y: 3 },
            walls: vec![Point { x: 1, y: 1 }],
        };
        let world = CompiledWorld::compile(&layout.source().unwrap()).unwrap();
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let out = std::env::temp_dir().join(format!(
            "hymeko-physical-export-{}-{unique}",
            std::process::id()
        ));
        world.export_physics(&actor, &out).unwrap();
        let receipt: Value =
            serde_json::from_str(&fs::read_to_string(out.join("composition.json")).unwrap())
                .unwrap();
        assert_eq!(receipt["obstacles"], 1);
        assert_eq!(receipt["physical_links"], 20);
        for file in ["model.hymeko", "scene.hymeko", "composition.json"] {
            fs::remove_file(out.join(file)).unwrap();
        }
        fs::remove_dir(out).unwrap();
    }

    #[test]
    fn centring_and_scale_preserve_neighbour_spacing_and_clear_corridors() {
        let layout = Layout {
            width: 16,
            height: 16,
            start: Point { x: 0, y: 0 },
            goal: Point { x: 15, y: 15 },
            walls: vec![],
        };
        assert_eq!(physical_point(&layout, layout.start), [-15., -15.]);
        assert_eq!(physical_point(&layout, layout.goal), [15., 15.]);
        let a = physical_point(&layout, Point { x: 3, y: 4 });
        let b = physical_point(&layout, Point { x: 4, y: 4 });
        assert_eq!([b[0] - a[0], b[1] - a[1]], [2., 0.]);
    }
}
