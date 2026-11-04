//! Bounded research application. Domain operations live in Formats; this owns campaign policy.
#[path = "adversarial_grid/maze.rs"]
mod maze;
#[path = "adversarial_grid/search.rs"]
mod search;
use hymeko::ir::semantic_digest::{SEMANTIC_DIGEST_ALGO_VERSION, SEMANTIC_DIGEST_SCHEMA_VERSION};
use hymeko_formats::environment::{
    CompiledWorld, Evaluation, Layout, Point, Result, check_receipts,
};
use hymeko_graph::rand_lcg::Lcg;
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    fs,
    io::Write as _,
    path::{Path, PathBuf},
    time::Instant,
};

fn random_layout(config: &Config, rng: &mut Lcg) -> Layout {
    let mut cells: Vec<usize> = (1..config.width * config.height - 1).collect();
    let wall_count = (config.width * config.height) as f64 * config.wall_fraction;
    let wall_count = wall_count.floor() as usize;
    for i in 0..wall_count {
        let offset = rng.next_in_range((cells.len() - i) as u32) as usize;
        cells.swap(i, i + offset);
    }
    cells.truncate(wall_count);
    cells.sort_unstable();
    Layout {
        width: config.width,
        height: config.height,
        start: Point { x: 0, y: 0 },
        goal: Point {
            x: config.width - 1,
            y: config.height - 1,
        },
        walls: cells
            .into_iter()
            .map(|i| Point {
                x: i % config.width,
                y: i / config.width,
            })
            .collect(),
    }
}

#[derive(Clone, Debug, Serialize)]
struct Sample {
    seed: u64,
    candidate_index: usize,
    wall_count: usize,
    source_path: String,
    semantic_digest: String,
    layout: Layout,
    query_checks: Vec<Value>,
    #[serde(flatten)]
    evaluation: Evaluation,
    timings_ms: Value,
}

impl Sample {
    fn score(&self) -> Option<usize> {
        self.evaluation.astar.as_ref().map(|s| s.expanded_nodes)
    }

    fn export(&self, output: &Path, name: &str) -> Result<Value> {
        let source = fs::read_to_string(output.join(&self.source_path))?;
        let world = CompiledWorld::compile(&source)?;
        if world.digest() != self.semantic_digest {
            return Err("selected source digest does not match its scored candidate".into());
        }
        let prefix = format!("seed_{}/{name}", self.seed);
        let source_path = format!("{prefix}.hymeko");
        let world_path = format!("{prefix}.world.sdf");
        fs::write(output.join(&source_path), source)?;
        fs::write(output.join(&world_path), world.world_sdf()?)?;
        let mut value = serde_json::to_value(self)?;
        value["source_path"] = json!(source_path);
        value["world_path"] = json!(world_path);
        Ok(value)
    }
}

#[derive(Clone, Debug, Serialize)]
struct Config {
    width: usize,
    height: usize,
    seeds: usize,
    proposals: usize,
    wall_fraction: f64,
    first_seed: u64,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            width: 16,
            height: 16,
            seeds: 10,
            proposals: 100,
            wall_fraction: 0.25,
            first_seed: 0,
        }
    }
}
impl Config {
    fn validate(&self) -> Result<()> {
        if !(2..=16).contains(&self.width) || !(2..=16).contains(&self.height) {
            return Err("width and height must be in 2..=16".into());
        }
        if self.seeds == 0
            || self.proposals == 0
            || self
                .seeds
                .checked_mul(self.proposals)
                .is_none_or(|n| n > 1000)
        {
            return Err("campaign must contain 1..=1000 total proposals".into());
        }
        if !self.wall_fraction.is_finite() || !(0.0..=0.5).contains(&self.wall_fraction) {
            return Err("wall fraction must be finite and in 0..=0.5".into());
        }
        self.first_seed
            .checked_add(self.seeds as u64 - 1)
            .ok_or("seed range overflows")?;
        Ok(())
    }
}

struct Campaign {
    config: Config,
    output: PathBuf,
}
impl Campaign {
    fn parse(args: impl IntoIterator<Item = String>) -> Result<Self> {
        let mut config = Config::default();
        let mut output = None;
        let mut enabled = false;
        let mut args = args.into_iter();
        while let Some(flag) = args.next() {
            if flag == "--run-campaign" {
                enabled = true;
                continue;
            }
            let value = args
                .next()
                .ok_or_else(|| format!("missing value after {flag}"))?;
            match flag.as_str() {
                "--output" => output = Some(PathBuf::from(value)),
                "--width" => config.width = value.parse()?,
                "--height" => config.height = value.parse()?,
                "--seeds" => config.seeds = value.parse()?,
                "--proposals" => config.proposals = value.parse()?,
                "--wall-fraction" => config.wall_fraction = value.parse()?,
                "--first-seed" => config.first_seed = value.parse()?,
                _ => return Err(format!("unknown option {flag}").into()),
            }
        }
        if !enabled {
            return Err("measurement is opt-in: pass --run-campaign --output DIR [--width 16 --height 16 --seeds 10 --proposals 100 --wall-fraction 0.25 --first-seed 0]".into());
        }
        config.validate()?;
        Ok(Self {
            config,
            output: output.ok_or("--output is required")?,
        })
    }

    fn candidate(&self, seed: u64, candidate_index: usize, rng: &mut Lcg) -> Result<Sample> {
        let started = Instant::now();
        let proposal = random_layout(&self.config, rng);
        let authored = proposal.source()?;
        let generate_ms = started.elapsed().as_secs_f64() * 1000.0;
        let started = Instant::now();
        let (world, canonical) = CompiledWorld::canonical(&authored)?;
        let projection = world.project()?;
        if &proposal != projection.layout() {
            return Err("compiled layout disagrees with proposed occupancy".into());
        }
        let compile_ms = started.elapsed().as_secs_f64() * 1000.0;
        let evaluation = projection.evaluate(true)?;
        let source_path = format!("seed_{seed}/candidates/candidate_{candidate_index}.hymeko");
        fs::write(self.output.join(&source_path), canonical)?;
        Ok(Sample {
            seed,
            candidate_index,
            wall_count: projection.layout().walls.len(),
            source_path,
            semantic_digest: world.digest(),
            query_checks: check_receipts(projection.checks()),
            layout: projection.layout().clone(),
            evaluation,
            timings_ms: json!({"generate":generate_ms,"compile_emit_recompile":compile_ms}),
        })
    }

    fn seed(&self, seed: u64, samples: &mut fs::File, start: Instant) -> Result<Value> {
        fs::create_dir_all(self.output.join(format!("seed_{seed}/candidates")))?;
        let mut rng = Lcg::new(seed);
        let mut baseline: Option<Sample> = None;
        let mut selected: Option<Sample> = None;
        let mut feasible_count = 0;
        for candidate_index in 0..self.config.proposals {
            if start.elapsed().as_secs() >= 300 {
                return Err(
                    "campaign exceeded the 300-second cap; partial samples retained".into(),
                );
            }
            let sample = self.candidate(seed, candidate_index, &mut rng)?;
            serde_json::to_writer(&mut *samples, &sample)?;
            writeln!(samples)?;
            samples.flush()?;
            if sample.evaluation.feasible {
                feasible_count += 1;
                if baseline.is_none() {
                    baseline = Some(sample.clone());
                }
                if selected
                    .as_ref()
                    .is_none_or(|best| sample.score() > best.score())
                {
                    selected = Some(sample);
                }
            }
            if candidate_index % 10 == 0 {
                eprintln!(
                    "seed={seed} candidate={candidate_index}/{} feasible={feasible_count}",
                    self.config.proposals
                );
            }
        }
        Ok(
            json!({"seed":seed,"feasible_count":feasible_count,"infeasible_count":self.config.proposals-feasible_count,
            "baseline_candidate":baseline.as_ref().map(|s|s.candidate_index),"selected_candidate":selected.as_ref().map(|s|s.candidate_index),
            "baseline":baseline.as_ref().map(|s|s.export(&self.output,"baseline")).transpose()?,
            "selected":selected.as_ref().map(|s|s.export(&self.output,"adversarial")).transpose()?}),
        )
    }

    fn run(&self) -> Result<()> {
        if self.output.exists() && fs::read_dir(&self.output)?.next().is_some() {
            return Err("output directory must be empty to preserve previous runs".into());
        }
        fs::create_dir_all(&self.output)?;
        let mut samples = fs::File::create(self.output.join("samples.jsonl"))?;
        let started = Instant::now();
        let mut seeds = Vec::new();
        for offset in 0..self.config.seeds {
            seeds.push(self.seed(
                self.config.first_seed + offset as u64,
                &mut samples,
                started,
            )?);
        }
        let summary = json!({"schema_version":1,"config":self.config,"seeds":seeds,"total_elapsed_ms":started.elapsed().as_secs_f64()*1000.0,
            "semantic_digest":{"owner":"hymeko_core","schema":SEMANTIC_DIGEST_SCHEMA_VERSION,"algorithm":SEMANTIC_DIGEST_ALGO_VERSION},
            "search":{"owner":"hymeko_graph::direct_ir","cost":"UnitCost","policy":"Undirected","heuristic":"Manhattan","tie_break":"f, zero discrete tie, canonical declaration/arc/slot IDs"},
            "selection":"maximum A* expanded_nodes among feasible proposals; earliest proposal wins ties; baseline is first feasible",
            "scope":"best of a fixed proposal budget; no general hardness or physical solvability claim","samples_path":"samples.jsonl"});
        fs::write(
            self.output.join("summary.json"),
            serde_json::to_vec_pretty(&summary)?,
        )?;
        eprintln!(
            "completed {} candidates; {}",
            self.config.seeds * self.config.proposals,
            self.output.join("summary.json").display()
        );
        Ok(())
    }
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.first().is_some_and(|arg| arg == "--benchmark-mazes") {
        return search::run(&args[1..]);
    }
    if args.first().is_some_and(|arg| arg == "--generate-maze") {
        return maze::run(&args[1..]);
    }
    if args.first().is_some_and(|arg| arg == "--generate-terrain") {
        return maze::run_terrain(&args[1..]);
    }
    if args.first().is_some_and(|arg| arg == "--project-legged") {
        return maze::run_project_legged(&args[1..]);
    }
    if args.first().is_some_and(|arg| arg == "--export-physics") {
        if args.len() != 4 {
            return Err(
                "usage: adversarial_grid --export-physics GRID.hymeko ACTOR.hymeko NEW_DIRECTORY"
                    .into(),
            );
        }
        let world = CompiledWorld::compile(&fs::read_to_string(&args[1])?)?;
        world.export_physics(Path::new(&args[2]), Path::new(&args[3]))?;
        return Ok(());
    }
    if args.first().is_some_and(|arg| arg == "--export-scene") {
        if args.len() != 3 {
            return Err("usage: adversarial_grid --export-scene INPUT.hymeko OUTPUT.hymeko".into());
        }
        let output = Path::new(&args[2]);
        let receipt = output.with_extension("validation.json");
        if output.exists() || receipt.exists() {
            return Err("scene output and validation receipt must be new".into());
        }
        let world = CompiledWorld::compile(&fs::read_to_string(&args[1])?)?;
        let (source, validation) = world.visual_source()?;
        fs::write(output, source)?;
        fs::write(receipt, serde_json::to_vec_pretty(&validation)?)?;
        println!(
            "Exported query-validated visual scene to {}",
            output.display()
        );
        return Ok(());
    }
    if args.first().is_some_and(|arg| arg == "--check") {
        if args.len() != 2 {
            return Err("usage: adversarial_grid --check FILE.hymeko".into());
        }
        let world = CompiledWorld::compile(&fs::read_to_string(&args[1])?)?;
        // project() also checks the domain's exact integer/uniqueness requirements.
        match world.project() {
            Ok(projection) => {
                println!(
                    "{}",
                    serde_json::to_string_pretty(&json!({
                        "passed": true, "semantic_digest": world.digest(),
                        "checks": check_receipts(projection.checks()),
                        "scope": "static grid contract; no reachability or simulation run",
                    }))?
                );
                return Ok(());
            }
            Err(error) => {
                println!("{}", json!({"passed":false,"error":error.to_string()}));
                return Err(error);
            }
        }
    }
    Campaign::parse(args)?.run()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cli_requires_opt_in_and_bounds() {
        let args = |s: &str| s.split_whitespace().map(str::to_owned).collect::<Vec<_>>();
        assert!(Campaign::parse(args("--output somewhere")).is_err());
        assert!(Campaign::parse(args("--run-campaign --output somewhere --width 17")).is_err());
        assert!(Campaign::parse(args("--run-campaign --output somewhere --seeds 11")).is_err());
        assert!(
            Campaign::parse(args(
                "--run-campaign --output somewhere --wall-fraction NaN"
            ))
            .is_err()
        );
        let campaign=Campaign::parse(args("--run-campaign --output somewhere --width 4 --height 4 --seeds 1 --proposals 2 --first-seed 42")).unwrap();
        assert_eq!(campaign.config.first_seed, 42);
        assert_eq!(campaign.config.proposals, 2);
    }

    #[test]
    fn seed_replay_variation_and_exact_wall_count() {
        let config = Config::default();
        let mut a = Lcg::new(42);
        let mut b = Lcg::new(42);
        let mut c = Lcg::new(43);
        let first = random_layout(&config, &mut a);
        assert_eq!(first, random_layout(&config, &mut b));
        assert_ne!(first, random_layout(&config, &mut c));
        assert_ne!(first, random_layout(&config, &mut a));
        assert_eq!(first.walls.len(), 64);
        first.validate().unwrap();
    }
}
