//! Export a compiled robot through the public Formats projection API.
use hymeko::module_store::{execution::CompiledModel, source_provider::StdFsProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_formats::robot_projection::{Result, RobotProjection};
use std::{fs, path::Path};

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if !(args.len() == 3 || (args.len() == 4 && args[3] == "--benchmark-fk")) {
        return Err("usage: robot_projection ROBOT.hymeko ROOT OUTPUT_DIR [--benchmark-fk]".into());
    }
    let (path, root, out) = (&args[0], &args[1], Path::new(&args[2]));
    // The model label is a file stem here, never a relative or absolute path.
    if root.is_empty() || root.contains(['/', '\\', ':']) || root == "." || root == ".." {
        return Err("model label must be a file stem, not a path".into());
    }
    for suffix in [".xml", "_model.json", "_fk.json", "_timing.txt"] {
        if out.join(format!("{root}{suffix}")).exists() {
            return Err("projection output must be new".into());
        }
    }
    let document = CompiledModel::compile(StdFsProvider::new(), RealParser, Path::new(path))
        .map_err(|error| format!("Core compilation: {error:?}"))?;
    let projection = RobotProjection::new(&document, root)?;
    let xml = projection.mjcf()?;
    let q = projection.diagnostic_pose();
    let poses = projection.forward_poses(&q)?;
    fs::create_dir_all(out)?;
    fs::write(out.join(format!("{root}.xml")), xml)?;
    fs::write(
        out.join(format!("{root}_model.json")),
        serde_json::to_vec_pretty(&projection.metadata())?,
    )?;
    fs::write(
        out.join(format!("{root}_fk.json")),
        serde_json::to_vec_pretty(&poses)?,
    )?;
    if args.len() == 4 {
        let started = std::time::Instant::now();
        for _ in 0..20_000 {
            std::hint::black_box(hymeko_geometry::forward::forward_kinematics(
                projection.model(),
                &q,
            )?);
        }
        let us = started.elapsed().as_secs_f64() * 1e6 / 20_000.;
        fs::write(out.join(format!("{root}_timing.txt")), us.to_string())?;
        eprintln!("hymeko FK: {us:.2} us/solve");
    }
    Ok(())
}
