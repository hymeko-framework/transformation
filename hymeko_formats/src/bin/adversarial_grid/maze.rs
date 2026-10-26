use hymeko_formats::environment::{
    Result,
    locomotion::LeggedRobot,
    maze::{Profile, Topology},
    terrain::{self, TerrainWorld},
};
use serde_json::json;
use std::{fs, path::Path};

pub(super) fn run(args: &[String]) -> Result<()> {
    let (args, loops) = if args.len() >= 5 && args[args.len() - 2] == "--loops" {
        (
            &args[..args.len() - 2],
            args[args.len() - 1].parse::<usize>()?,
        )
    } else {
        (args, 0)
    };
    let (args, weights_path) = if args.len() >= 5 && args[args.len() - 2] == "--weights" {
        (
            &args[..args.len() - 2],
            Some(Path::new(&args[args.len() - 1])),
        )
    } else {
        (args, None)
    };
    if !(3..=4).contains(&args.len()) {
        return Err(
            "usage: adversarial_grid --generate-maze square|hex|polar SEED NEW_DIRECTORY [ACTOR.hymeko] [--weights GENES.json | --loops COUNT]"
                .into(),
        );
    }
    let profile = Profile::parse(&args[0])?;
    let seed = args[1].parse()?;
    let output = Path::new(&args[2]);
    let host = std::process::Command::new("hostname").output()?;
    if !host.status.success()
        || !String::from_utf8_lossy(&host.stdout)
            .trim()
            .eq_ignore_ascii_case("amaterasu")
    {
        return Err("explicit maze generation runs on Amaterasu; local unit tests only".into());
    }
    if output.exists() {
        return Err("maze output directory must be new".into());
    }
    let topology = Topology::new(profile);
    if loops > topology.max_loops() || (loops > 0 && weights_path.is_some()) {
        return Err("loop count out of range or combined with priority genes".into());
    }
    let genes = weights_path
        .map(|path| -> Result<Vec<f64>> {
            if fs::metadata(path)?.len() > 8192 {
                return Err("priority input exceeds 8192 bytes".into());
            }
            Ok(serde_json::from_slice(&fs::read(path)?)?)
        })
        .transpose()?;
    let (world, source, geometry, checks) = match &genes {
        Some(genes) => topology.generate_priorities(seed, genes)?,
        None => topology.generate_loops(seed, loops)?,
    };
    let view = geometry.visual_source()?;
    fs::create_dir_all(output)?;
    fs::write(output.join("source.hymeko"), source)?;
    fs::write(output.join("view.hymeko"), view)?;
    fs::write(
        output.join("geometry.json"),
        serde_json::to_vec_pretty(&geometry)?,
    )?;
    let receipt = json!({"schema_version":1,"semantic_digest":world.digest(),"query_checks":checks,"cell_count":geometry.cells().len(),"passage_count":geometry.passages().len(),"wall_count":geometry.walls().len(),"route_cells":geometry.route_cell_indices(),"route_points_m":geometry.route_points_m(),"clearance_radius_m":geometry.clearance_radius_m(),"clearance_owner":"hymeko_geometry::collide::intersects; swept capsule against actual yaw boxes","priority_pairs":topology.priority_pairs(),"priority_encoding":"lexicographic passage pairs; [0,1] mapped to exact 53-bit integer weights","loops":loops,"scope":"rank-2 connected maze with declared cycle rank; geometric route witness, not controller success or uniformly sampled trees"});
    fs::write(
        output.join("validation.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    if let Some(genes) = genes {
        fs::write(
            output.join("genome.json"),
            serde_json::to_vec_pretty(&genes)?,
        )?;
    }
    hymeko_formats::environment::maze_physics::export(
        &geometry,
        output,
        args.get(3).map(Path::new),
    )?;
    println!("{}", serde_json::to_string(&receipt)?);
    Ok(())
}

fn amaterasu_only() -> Result<()> {
    let host = std::process::Command::new("hostname").output()?;
    if !host.status.success()
        || !String::from_utf8_lossy(&host.stdout)
            .trim()
            .eq_ignore_ascii_case("amaterasu")
    {
        return Err("explicit world generation runs on Amaterasu; local unit tests only".into());
    }
    Ok(())
}

/// Split a trailing `--flag VALUE` pair off the argument list.
fn trailing<'a>(args: &'a [String], flag: &str) -> (&'a [String], Option<&'a str>) {
    match args.iter().position(|a| a == flag) {
        Some(i) if i + 2 == args.len() => (&args[..i], Some(args[i + 1].as_str())),
        _ => (args, None),
    }
}

/// `--project-legged PROFILE.hymeko NEW_DIRECTORY [--horizon S]`: flat physical pretest scene.
pub(super) fn run_project_legged(args: &[String]) -> Result<()> {
    let (args, horizon) = trailing(args, "--horizon");
    if args.len() != 2 {
        return Err("usage: adversarial_grid --project-legged PROFILE.hymeko NEW_DIRECTORY [--horizon S]".into());
    }
    amaterasu_only()?;
    let output = Path::new(&args[1]);
    if output.exists() {
        return Err("output directory must be new".into());
    }
    let robot = LeggedRobot::read(Path::new(&args[0]))?;
    let horizon = horizon.map(str::parse::<f64>).transpose()?.unwrap_or(30.0);
    fs::create_dir_all(output)?;
    terrain::export_flat(&robot, horizon, output)?;
    println!("{}", serde_json::to_string(&robot.binding()["measured"])?);
    Ok(())
}

/// `--generate-terrain square|hex|polar SEED NEW_DIRECTORY PROFILE.hymeko [--weights G.json] [--horizon S] [--task T]`.
pub(super) fn run_terrain(args: &[String]) -> Result<()> {
    let (args, task) = trailing(args, "--task");
    let (args, horizon) = trailing(args, "--horizon");
    let (args, weights) = trailing(args, "--weights");
    if args.len() != 4 {
        return Err("usage: adversarial_grid --generate-terrain square|hex|polar SEED NEW_DIRECTORY PROFILE.hymeko [--weights GENES.json] [--horizon S] [--task reach|coverage|team]".into());
    }
    let task = terrain::TaskKind::parse(task.unwrap_or("reach"))?;
    amaterasu_only()?;
    let profile = Profile::parse(&args[0])?;
    let seed = args[1].parse()?;
    let output = Path::new(&args[2]);
    if output.exists() {
        return Err("terrain output directory must be new".into());
    }
    let robot = LeggedRobot::read(Path::new(&args[3]))?;
    let genes = weights
        .map(|path| -> Result<Vec<f64>> {
            if fs::metadata(path)?.len() > 8192 {
                return Err("gene input exceeds 8192 bytes".into());
            }
            Ok(serde_json::from_slice(&fs::read(path)?)?)
        })
        .transpose()?;
    let horizon = horizon.map(str::parse::<f64>).transpose()?;
    let world = TerrainWorld::generate_task(profile, seed, &robot, genes.as_deref(), horizon, task)?;
    fs::create_dir_all(output)?;
    fs::write(output.join("source.hymeko"), &world.source)?;
    let mut receipt = world.receipt();
    receipt["dimension"] = json!(TerrainWorld::dimension_for(profile, &robot, task)?);
    receipt["robot"] = robot.binding()["profile"].clone();
    fs::write(output.join("validation.json"), serde_json::to_vec_pretty(&receipt)?)?;
    if let Some(genes) = genes {
        fs::write(output.join("genome.json"), serde_json::to_vec_pretty(&genes)?)?;
    }
    // An invalid placement has no complete body set; infeasible worlds remain exportable.
    if world.category != terrain::Category::InvalidWorld {
        world.export(&robot, output)?;
    }
    println!("{}", json!({"category": world.category, "semantic_digest": world.world.digest()}));
    Ok(())
}
