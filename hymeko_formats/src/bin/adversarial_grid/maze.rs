use hymeko_formats::environment::{
    Result,
    maze::{Profile, Topology},
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
            "usage: adversarial_grid --generate-maze hex|polar SEED NEW_DIRECTORY [ACTOR.hymeko] [--weights GENES.json | --loops COUNT]"
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
