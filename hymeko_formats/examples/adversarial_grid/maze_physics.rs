//! Scene composition over the checked maze DTO; Core/Query/Geometry own projection.
use super::maze::{MazeGeometry, MazeWall};
use super::*;
use hymeko::module_store::{execution::CompiledModel, source_provider::StdFsProvider};
use hymeko_geometry::{
    collide::{Convex, Placement, intersects},
    kinematics::kinematic::extract_kinematic_model,
    transforms::{DomainTransform, ModelView, TransformConfig},
};

struct Body {
    name: String,
    centre: [f64; 3],
    size: [f64; 3],
    yaw: f64,
    dynamic: bool,
    velocity: [f64; 3],
    color: [f64; 4],
}

fn placement(centre: [f64; 3], yaw: f64) -> Placement {
    let (s, c) = yaw.sin_cos();
    Placement {
        centre,
        rotation: [c, -s, 0., s, c, 0., 0., 0., 1.],
    }
}

/// Bounded fixture selection only; intersection belongs to Geometry, not this client.
fn stimulus(maze: &MazeGeometry) -> Result<(usize, Vec<Body>)> {
    for wall in &maze.walls {
        if wall.full_size_m[0] < 1.2 {
            continue;
        }
        let cell = maze
            .cells
            .iter()
            .min_by(|a, b| {
                let distance =
                    |p: [f64; 2]| (p[0] - wall.centre_m[0]).hypot(p[1] - wall.centre_m[1]);
                distance(a.centre_m).total_cmp(&distance(b.centre_m))
            })
            .ok_or("maze has no cells")?;
        let (s, c) = wall.yaw_rad.sin_cos();
        let sign = if -s * (cell.centre_m[0] - wall.centre_m[0])
            + c * (cell.centre_m[1] - wall.centre_m[1])
            >= 0.
        {
            1.
        } else {
            -1.
        };
        let normal = [-s * sign, c * sign];
        let trials = [
            ("striker", 0.92, 0., 0.35, 3.),
            ("target", 0.34, 0., 0.35, 0.),
            ("drop", 0.92, 0.65, 1.2, 0.),
        ];
        let bodies: Vec<_> = trials
            .iter()
            .enumerate()
            .map(|(i, (name, offset, along, z, speed))| Body {
                name: (*name).into(),
                centre: [
                    wall.centre_m[0] + offset * normal[0] + along * c,
                    wall.centre_m[1] + offset * normal[1] + along * s,
                    *z,
                ],
                size: [0.24; 3],
                yaw: wall.yaw_rad,
                dynamic: true,
                velocity: [-speed * normal[0], -speed * normal[1], 0.],
                color: [
                    [0.95, 0.3, 0.12, 1.],
                    [0.12, 0.65, 0.9, 1.],
                    [0.95, 0.8, 0.15, 1.],
                ][i],
            })
            .collect();
        let clear = bodies.iter().all(|body| {
            maze.walls.iter().all(|w| {
                !intersects(
                    &Convex::Sphere { radius: 0.23 },
                    &placement(body.centre, 0.),
                    &Convex::Cuboid {
                        half: w.full_size_m.map(|v| v * 0.5),
                    },
                    &placement(w.centre_m, w.yaw_rad),
                )
            })
        });
        if clear {
            return Ok((wall.index, bodies));
        }
    }
    Err("no checked wall-side fixture fits; no dynamic objects were silently omitted".into())
}

impl Body {
    fn wall(wall: &MazeWall) -> Self {
        Self {
            name: format!("wall_{}", wall.index),
            centre: wall.centre_m,
            size: wall.full_size_m,
            yaw: wall.yaw_rad,
            dynamic: false,
            velocity: [0.; 3],
            color: [0.25, 0.4, 0.5, 1.],
        }
    }

    fn append(&self, source: &mut String) -> Result<()> {
        let Self {
            name,
            centre: [x, y, z],
            size: [a, b, c],
            yaw,
            dynamic,
            velocity: [vx, vy, vz],
            color,
        } = self;
        writeln!(
            source,
            "{name}: meta_kinematics.kinematics.elements.link {{\norigin [0,0,0]; dynamics_role {}; velocity_x {vx}; velocity_y {vy}; velocity_z {vz}; color [{},{},{},{}];",
            u8::from(*dynamic),
            color[0],
            color[1],
            color[2],
            color[3]
        )?;
        if *dynamic {
            source.push_str(
                "mass 1; inertia_com { xx 0.0096; yy 0.0096; zz 0.0096; xy 0; xz 0; yz 0; }\n",
            );
        }
        writeln!(
            source,
            "link_geometry: meta_kinematics.kinematics.geometry.box {{ dimension [{a},{b},{c}]; }} visual -> link_geometry; collision -> link_geometry; }}\n@mount_{name}: meta_kinematics.kinematics.fixed_joint {{ (+ world [[{x},{y},{z}],[0,0,{}]], - {name}); }}",
            yaw.to_degrees()
        )?;
        Ok(())
    }
}

/// Emit the canonical physical model and the existing owner's MJCF, then query runtime settings.
pub(super) fn export(maze: &MazeGeometry, output: &Path, actor: Option<&Path>) -> Result<()> {
    for name in [
        "model.hymeko",
        "meta_kinematics.hymeko",
        "scene.xml",
        "physics.json",
        "robot.xml",
        "robot.json",
    ] {
        if output.join(name).exists() {
            return Err(format!("refusing to replace {name}").into());
        }
    }
    let (selected_wall, mut bodies) = stimulus(maze)?;
    let robot = actor.map(|path| robot_projection(maze, path)).transpose()?;
    if let Some((_, profile)) = &robot {
        let p = profile["push_box_position_m"]
            .as_array()
            .ok_or("missing push position")?;
        bodies[0].centre = [p[0].as_f64().ok_or("x")?, p[1].as_f64().ok_or("y")?, 0.121];
        bodies[0].velocity = [0.; 3];
    }
    bodies.extend(maze.walls.iter().map(Body::wall));
    let extent = maze
        .walls
        .iter()
        .flat_map(|w| w.centre_m[..2].iter())
        .map(|v| v.abs())
        .fold(0., f64::max)
        + 3.;
    bodies.push(Body {
        name: "floor".into(),
        centre: [0., 0., -0.05],
        size: [2. * extent, 2. * extent, 0.1],
        yaw: 0.,
        dynamic: false,
        velocity: [0.; 3],
        color: [0.72, 0.72, 0.66, 1.],
    });
    let mut source = String::from(
        "maze_physics_description { @\"meta_kinematics.hymeko\"; }\nmaze_physics { timestep_s 0.002; duration_s 3; sample_every 10; contact_time_constant_s 0.008; contact_damping_ratio 1; sliding_friction 0.45; world: meta_kinematics.kinematics.elements.link {}\n",
    );
    for body in &bodies {
        body.append(&mut source)?;
    }
    source.push_str("}\n");
    fs::write(
        output.join("meta_kinematics.hymeko"),
        include_str!("../../../data/robotics/meta_kinematics.hymeko"),
    )?;
    fs::write(output.join("model.hymeko"), source)?;
    let document = CompiledModel::compile(
        StdFsProvider::new(),
        RealParser,
        &output.join("model.hymeko"),
    )
    .map_err(|e| format!("maze physics compile: {e:?}"))?;
    let query = QueryEngine::new(&document.program().ir, document.interner());
    let profile_id = query
        .query_first(&Predicate::named("maze_physics"))
        .ok_or("missing physical profile")?
        .id;
    let number = |decl, key: &str| -> Result<f64> {
        match query.effective_value(decl, key) {
            Some(ValueR::Num(v)) if v.is_finite() => Ok(*v),
            _ => Err(format!("missing physical field {key}").into()),
        }
    };
    let model = extract_kinematic_model(&query, "maze_physics");
    if model.links.len() != bodies.len() + 1 || model.joints.len() != bodies.len() {
        return Err("physical projection lost a declared body or mounting joint".into());
    }
    let objects = model.links.iter().filter(|link| link.name != "world").map(|link| -> Result<Value> {
        let role = number(link.did, "dynamics_role")?;
        if role != 0. && role != 1. { return Err("unknown dynamics_role".into()); }
        Ok(json!({"name":link.name,"decl_id":link.did.raw(),"dynamic":role==1.,
            "initial_velocity_m_s":[number(link.did,"velocity_x")?,number(link.did,"velocity_y")?,number(link.did,"velocity_z")?]}))
    }).collect::<Result<Vec<_>>>()?;
    let xml = hymeko_formats::MjcfTransform
        .emit(
            &ModelView::Kinematic(model),
            &TransformConfig::default().with_name("maze_physics"),
        )
        .ok_or("owner MJCF projection rejected the physical model")?;
    fs::write(output.join("scene.xml"), xml)?;
    let digest: String = Canon::new(&document.program().ir, document.interner())
        .hash()
        .0
        .0
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    fs::write(
        output.join("physics.json"),
        serde_json::to_vec_pretty(&json!({
            "schema":"hymeko-maze-physics/v1", "profile":maze.profile, "seed":maze.seed,
            "canonical_digest":digest, "objects":objects, "selected_wall":format!("wall_{selected_wall}"),
            "mjcf":"scene.xml", "source":"model.hymeko", "backend":"mujoco",
            "mobility_boundary":"dynamics_role=1 selects a six-DOF freejoint on an owner-projected root body",
            "timestep_s":number(profile_id,"timestep_s")?,"duration_s":number(profile_id,"duration_s")?,
            "sample_every":number(profile_id,"sample_every")?,
            "contact_time_constant_s":number(profile_id,"contact_time_constant_s")?,
            "contact_damping_ratio":number(profile_id,"contact_damping_ratio")?,
            "sliding_friction":number(profile_id,"sliding_friction")?
        }))?,
    )?;
    if let Some((xml, profile)) = robot {
        fs::write(output.join("robot.xml"), xml)?;
        fs::write(
            output.join("robot.json"),
            serde_json::to_vec_pretty(&profile)?,
        )?;
    }
    Ok(())
}

/// Reuse the actor contract and the existing kinematic/visual projections.
fn robot_projection(maze: &MazeGeometry, path: &Path) -> Result<(String, Value)> {
    use hymeko_geometry::kinematics::{kinematic::GeometryShape, visual::VisualSet};
    let contract = super::physical::ActorContract::read(path)?;
    if contract.radius >= maze.clearance_radius_m || maze.route_points_m.len() < 2 {
        return Err("robot needs a validated, sufficiently wide route segment".into());
    }
    let (segment, points) = maze
        .route_points_m
        .windows(2)
        .enumerate()
        .find(|(_, pair)| (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]) >= 2.4)
        .ok_or("route has no segment long enough for the bounded wheel-drive pulse")?;
    let start = points[0];
    let next = points[1];
    let direction = [next[0] - start[0], next[1] - start[1]];
    let length = direction[0].hypot(direction[1]);
    let document = CompiledModel::compile(StdFsProvider::new(), RealParser, path)
        .map_err(|e| format!("robot compile: {e:?}"))?;
    let query = QueryEngine::new(&document.program().ir, document.interner());
    let model = extract_kinematic_model(&query, "cyberdisc");
    let visuals = VisualSet::extract(&query, &model)?;
    let visual_rows = visuals
        .instances
        .iter()
        .filter_map(|v| v.geometry.as_ref().map(|g| (v, g)))
        .map(|(v, g)| -> Result<Value> {
            let (kind, size) = match g.shape {
                GeometryShape::Box => (
                    "box",
                    g.dimensions.iter().map(|x| x * 0.5).collect::<Vec<_>>(),
                ),
                GeometryShape::Cylinder => {
                    ("cylinder", vec![g.dimensions[0], g.dimensions[1] * 0.5])
                }
                GeometryShape::Sphere => ("sphere", g.dimensions.clone()),
                _ => return Err("unsupported actor visual in this bounded fixture".into()),
            };
            let mut rotation = [0.; 9];
            v.local_pose.rotation_matrix_into(&mut rotation);
            Ok(
                json!({"body":model.links[v.link_index].name,"decl_id":v.did.raw(),
                "kind":kind,"size":size,"position":v.local_pose.translation,
                "rotation":rotation,"rgba":v.color}),
            )
        })
        .collect::<Result<Vec<_>>>()?;
    let links: Vec<_> = model
        .links
        .iter()
        .map(|l| json!({"name":l.name,"decl_id":l.did.raw()}))
        .collect();
    let digest = Canon::new(&document.program().ir, document.interner())
        .hash()
        .0
        .0
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<String>();
    let xml = hymeko_formats::MjcfTransform
        .emit(
            &ModelView::Kinematic(model),
            &TransformConfig::default().with_name("cyberdisc"),
        )
        .ok_or("robot MJCF projection failed")?;
    let profile = json!({"schema":"hymeko-maze-robot/v1","actor_source":path.canonicalize()?,
        "canonical_digest":digest,"links":links,"visuals":visual_rows,
        "chassis":"chassis","wheels":["drive_fl","drive_rl","drive_fr","drive_rr"],
        "position_m":[start[0],start[1],0.261],"yaw_rad":direction[1].atan2(direction[0]),
        "route_segment_index":segment,
        "push_box_position_m":[start[0]+0.9*direction[0]/length,start[1]+0.9*direction[1]/length,0.121],
        "clearance_radius_m":contract.radius,"duration_s":6.0,
        "drive_begin_s":0.5,"drive_end_s":3.5,"wheel_speed_rad_s":3.0,
        "velocity_gain":3.0,"max_torque_nm":6.0,
        "scope":"bounded straight wheel-drive and push, not full maze navigation"});
    Ok((xml, profile))
}

#[cfg(test)]
mod tests {
    use super::super::maze::MazeCell;
    use super::*;

    fn fixture() -> MazeGeometry {
        MazeGeometry {
            profile: "fixture".into(),
            seed: 0,
            clearance_radius_m: 0.85,
            cells: vec![MazeCell {
                index: 0,
                centre_m: [0., 1.],
            }],
            passages: vec![],
            walls: vec![MazeWall {
                index: 0,
                centre_m: [0., 0., 0.5],
                full_size_m: [4., 0.12, 1.],
                yaw_rad: 0.,
            }],
            start: 0,
            goal: 0,
            route_cell_indices: vec![0],
            route_points_m: vec![[0., 1.]],
        }
    }

    #[test]
    #[ignore = "pure actor projection requiring the sibling robotics checkout; no simulation"]
    fn robot_projection_preserves_actor_and_rejects_short_route() {
        let actor = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../robotics/examples/cyberdisc/actor.hymeko");
        let mut maze = fixture();
        assert!(robot_projection(&maze, &actor).is_err());
        maze.route_points_m.push([0., 4.]);
        let (xml, profile) = robot_projection(&maze, &actor).unwrap();
        assert_eq!(profile["links"].as_array().unwrap().len(), 13);
        assert_eq!(profile["visuals"].as_array().unwrap().len(), 11);
        assert!(xml.contains("mass=\"8\""));
        assert!(xml.contains("name=\"drive_fl\""));
        assert_eq!(profile["push_box_position_m"], json!([0., 1.9, 0.121]));
        maze.clearance_radius_m = 0.1;
        assert!(robot_projection(&maze, &actor).is_err());
    }

    #[test]
    fn stimulus_preserves_clearance_and_directs_striker_at_wall() {
        let (wall, bodies) = stimulus(&fixture()).unwrap();
        assert_eq!(wall, 0);
        assert_eq!(bodies.len(), 3);
        assert_eq!(bodies[0].velocity, [0., -3., 0.]);
        assert!(bodies[0].centre[1] > bodies[1].centre[1]);
        assert!(bodies[2].centre[2] > 1.);
    }

    #[test]
    fn missing_room_refuses_instead_of_dropping_objects() {
        let mut maze = fixture();
        maze.walls[0].full_size_m[0] = 0.5;
        assert!(stimulus(&maze).is_err());
        maze.walls[0].full_size_m[0] = 4.;
        maze.cells.clear();
        assert!(stimulus(&maze).is_err());
    }

    #[test]
    fn projection_roundtrip_keeps_roles_and_uses_mjcf_owner() {
        let output =
            std::env::temp_dir().join(format!("hymeko-maze-physics-{}", std::process::id()));
        fs::create_dir(&output).unwrap();
        export(&fixture(), &output, None).unwrap();
        let manifest: Value =
            serde_json::from_slice(&fs::read(output.join("physics.json")).unwrap()).unwrap();
        let objects = manifest["objects"].as_array().unwrap();
        assert_eq!(manifest["contact_time_constant_s"], 0.008);
        assert_eq!(objects.len(), 5);
        assert_eq!(objects.iter().filter(|o| o["dynamic"] == true).count(), 3);
        assert!(
            fs::read_to_string(output.join("scene.xml"))
                .unwrap()
                .contains("<mujoco")
        );
        assert!(export(&fixture(), &output, None).is_err());
        for name in [
            "model.hymeko",
            "meta_kinematics.hymeko",
            "scene.xml",
            "physics.json",
        ] {
            fs::remove_file(output.join(name)).unwrap();
        }
        fs::remove_dir(output).unwrap();
    }
}
