//! Bounded campaign composition; generation, admission and search stay in their owners.
use hymeko_formats::environment::{
    Result,
    maze::{Profile, Topology, search::MazeNavigation},
};
use hymeko_graph::rand_lcg::Lcg;
use serde_json::{Value, json};
use std::{fs, path::Path, time::Instant};

fn save(path: &Path, value: &Value) -> Result<()> {
    fs::write(path, serde_json::to_vec_pretty(value)?)?;
    Ok(())
}

pub(super) fn run(args: &[String]) -> Result<()> {
    if args.len() != 4 {
        return Err("usage: --benchmark-mazes NEW_DIRECTORY SEEDS PAIRS REPETITIONS".into());
    }
    let output = Path::new(&args[0]);
    let seeds: usize = args[1].parse()?;
    let pairs: usize = args[2].parse()?;
    let repeats: usize = args[3].parse()?;
    if !(1..=30).contains(&seeds) || !(1..=64).contains(&pairs) || !(1..=1000).contains(&repeats) {
        return Err("caps: seeds 1..30, pairs 1..64, repetitions 1..1000".into());
    }
    if 18 * seeds * pairs * repeats > 1_000_000 {
        return Err("campaign exceeds one million timed calls".into());
    }
    let host = std::process::Command::new("hostname").output()?;
    if !host.status.success()
        || !String::from_utf8_lossy(&host.stdout)
            .trim()
            .eq_ignore_ascii_case("amaterasu")
    {
        return Err("maze benchmarks run on Amaterasu".into());
    }
    if output.exists() {
        return Err("output directory must be new".into());
    }
    fs::create_dir_all(output)?;
    let clock = Instant::now();
    let mut tasks = Vec::new();
    for (name, profile) in [("hex", Profile::Hex), ("polar", Profile::Polar)] {
        let topology = Topology::new(profile);
        for seed in 0..seeds {
            let mut rng = Lcg::new(seed as u64 + 20261009);
            let count = if profile == Profile::Hex { 19 } else { 12 };
            let mut endpoints: Vec<_> = (0..count)
                .flat_map(|a| (0..count).filter(move |b| *b != a).map(move |b| [a, b]))
                .collect();
            for i in 0..pairs {
                let j = i + rng.next_in_range((endpoints.len() - i) as u32) as usize;
                endpoints.swap(i, j);
            }
            for loops in [0, 3, topology.max_loops()] {
                let label = format!("{name}-seed{seed}-loops{loops}");
                let directory = output.join(&label);
                let generation = Instant::now();
                let (world, source, geometry, checks) =
                    topology.generate_loops(seed as u64, loops)?;
                let generation_ms = generation.elapsed().as_secs_f64() * 1000.0;
                let navigation = MazeNavigation::new(&world)?;
                fs::create_dir(&directory)?;
                fs::write(directory.join("source.hymeko"), source)?;
                fs::write(directory.join("view.hymeko"), geometry.visual_source()?)?;
                save(
                    &directory.join("geometry.json"),
                    &serde_json::to_value(&geometry)?,
                )?;
                save(
                    &directory.join("validation.json"),
                    &json!({"digest":world.digest(),"query_checks":checks,"generation_ms":generation_ms,"loops":loops}),
                )?;
                for (pair_index, &[start, goal]) in endpoints[..pairs].iter().enumerate() {
                    let runs = navigation.evaluate(start, goal, repeats)?;
                    tasks.push(json!({"map":label,"profile":name,"seed":seed,"loops":loops,"pair_index":pair_index,"start":start,"goal":goal,"cells":geometry.cells().len(),"passages":geometry.passages().len(),"runs":runs}));
                }
                println!("{label}: {pairs} paired searches passed");
            }
        }
    }
    let receipt = json!({"schema":"hymeko.maze-search/1","seeds":seeds,"pairs":pairs,"repetitions":repeats,"map_count":6*seeds,"task_count":tasks.len(),"timed_calls":tasks.len()*3*repeats,"warmup_calls":tasks.len()*3,"wall_seconds":clock.elapsed().as_secs_f64(),"cost":"one per passage; minimum hops, not physical distance","heuristic":"Euclidean chord to goal / largest open-passage chord (rounded upward)","scope":"synthetic topology generation; not a Kubota/GNG reproduction or robot simulation","tasks":tasks});
    save(&output.join("results.json"), &receipt)?;
    let plotter = Path::new(env!("CARGO_MANIFEST_DIR")).join("tools/plot_maze_search.py");
    let status = std::process::Command::new("python3")
        .arg(plotter)
        .arg(output)
        .status()?;
    if !status.success() {
        return Err("maze plotting failed; raw results retained".into());
    }
    save(
        &output.join("completed.json"),
        &json!({"task_count":tasks.len(),"all_costs_and_clearance_checked":true}),
    )?;
    Ok(())
}
