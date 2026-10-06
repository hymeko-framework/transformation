//! Bounded, explicitly invoked research client of Core, Graph and the format emitters.
//! No search engine or IR implementation lives here. All scored topology is compiled source.
//! Run campaigns and simulation on Amaterasu; ordinary example tests perform correctness only.

#[path = "adversarial_grid/maze.rs"]
mod maze;
#[path = "adversarial_grid/maze_physics.rs"]
mod maze_physics;
#[path = "adversarial_grid/physical.rs"]
mod physical;
#[path = "adversarial_grid/visual.rs"]
mod visual;

use hymeko::{
    common::ids::{DeclId, NodeId},
    ir::{
        canonical_hash::Canon,
        incidence::build_incidence_index,
        ir::{Ir, ValueR},
        semantic_digest::{SEMANTIC_DIGEST_ALGO_VERSION, SEMANTIC_DIGEST_SCHEMA_VERSION},
    },
    module_store::{
        module_store::{CompiledProgram, ModuleStore},
        source_provider::MemProvider,
    },
    util::real_parser::RealParser,
};
use hymeko_graph::{
    direct_ir::{
        BergeHeuristic, HyperTraversal, InformedSearchWorkspace, SearchStats, TraversalWorkspace,
        Undirected, UnitCost, astar_berge_path, dijkstra_berge_path,
    },
    rand_lcg::Lcg,
};
use hymeko_query::{
    MatchExpectation, NamedQuery, Predicate, QueryCheck, QueryEngine, ValuePredicate,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    error::Error,
    fmt::Write as _,
    fs,
    io::Write as _,
    path::{Path, PathBuf},
    sync::Arc,
    time::Instant,
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
struct Point {
    x: usize,
    y: usize,
}

/// A proposal/observation DTO. The canonical compiled IR owns all scored connectivity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
struct Layout {
    width: usize,
    height: usize,
    start: Point,
    goal: Point,
    walls: Vec<Point>,
}

impl Layout {
    fn random(config: &Config, rng: &mut Lcg) -> Self {
        let mut cells: Vec<usize> = (1..config.width * config.height - 1).collect();
        let wall_count = (config.width * config.height) as f64 * config.wall_fraction;
        let wall_count = wall_count.floor() as usize;
        for i in 0..wall_count {
            let offset = rng.next_in_range((cells.len() - i) as u32) as usize;
            cells.swap(i, i + offset);
        }
        cells.truncate(wall_count);
        cells.sort_unstable();
        Self {
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

    fn blocked(&self, point: Point) -> bool {
        self.walls.contains(&point)
    }

    fn validate(&self) -> Result<()> {
        if !(2..=16).contains(&self.width) || !(2..=16).contains(&self.height) {
            return Err("grid dimensions must be in 2..=16".into());
        }
        let mut seen = vec![false; self.width * self.height];
        for point in self.walls.iter().chain([&self.start, &self.goal]) {
            if point.x >= self.width || point.y >= self.height {
                return Err("out-of-grid coordinate".into());
            }
        }
        for wall in &self.walls {
            let slot = wall.y * self.width + wall.x;
            if seen[slot] {
                return Err("duplicate obstacle coordinate".into());
            }
            seen[slot] = true;
        }
        if self.blocked(self.start) || self.blocked(self.goal) || self.start == self.goal {
            return Err("endpoints must be distinct free cells".into());
        }
        Ok(())
    }

    fn source(&self) -> Result<String> {
        self.validate()?;
        let mut source =
            String::from("adversarial_grid {}\nvocabulary { grid_cell {} gazebo_static_box {} }\n");
        writeln!(
            source,
            "grid {{ width {}; height {}; start_x {}; start_y {}; goal_x {}; goal_y {}; }}",
            self.width, self.height, self.start.x, self.start.y, self.goal.x, self.goal.y
        )?;
        for y in 0..self.height {
            for x in 0..self.width {
                let blocked = self.blocked(Point { x, y });
                writeln!(
                    source,
                    "cell_{x}_{y}: vocabulary.grid_cell {{ x {x}; y {y}; blocked {}; }}",
                    usize::from(blocked)
                )?;
                if blocked {
                    writeln!(
                        source,
                        "wall_{x}_{y}: vocabulary.gazebo_static_box {{ x {x}; y {y}; z 0.5; size_x 1.0; size_y 1.0; size_z 1.0; }}"
                    )?;
                }
            }
        }
        for y in 0..self.height {
            for x in 0..self.width {
                let from = Point { x, y };
                for to in [Point { x: x + 1, y }, Point { x, y: y + 1 }] {
                    if to.x < self.width
                        && to.y < self.height
                        && !self.blocked(from)
                        && !self.blocked(to)
                    {
                        writeln!(
                            source,
                            "@step_{x}_{y}_{}_{} {{ (~ cell_{x}_{y}, ~ cell_{}_{}); }}",
                            to.x, to.y, to.x, to.y
                        )?;
                    }
                }
            }
        }
        Ok(source)
    }
}

struct CompiledWorld {
    store: ModuleStore<MemProvider, RealParser>,
    program: Arc<CompiledProgram>,
}

impl CompiledWorld {
    fn compile(source: &str) -> Result<Self> {
        let provider = MemProvider::default().with_file("/grid.hymeko", source);
        let mut store = ModuleStore::new(provider, RealParser);
        let program = store
            .compile(Path::new("/grid.hymeko"))
            .map_err(|e| format!("grid compilation: {e:?}"))?;
        Ok(Self { store, program })
    }

    fn digest(&self) -> String {
        Canon::new(&self.program.ir, &self.store.it)
            .hash()
            .0
            .0
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect()
    }

    fn canonical(source: &str) -> Result<(Self, String)> {
        let original = Self::compile(source)?;
        let (recompiled, emitted) = original.reemit("adversarial_grid")?;
        if original.project()?.layout != recompiled.project()?.layout {
            return Err("canonical source round trip changed the projected layout".into());
        }
        Ok((recompiled, emitted))
    }

    fn reemit(&self, header: &str) -> Result<(Self, String)> {
        let emitted = hymeko_emitter::emit_hymeko(&self.program.ir, &self.store.it, header);
        let recompiled = Self::compile(&emitted)?;
        if self.digest() != recompiled.digest() {
            return Err("canonical source round trip changed the semantic digest".into());
        }
        Ok((recompiled, emitted))
    }

    fn scene_scalar(&self, decl: DeclId, key: &str) -> Result<f64> {
        let engine = QueryEngine::new(&self.program.ir, &self.store.it);
        match engine.effective_value(decl, key) {
            Some(ValueR::Num(n)) if n.is_finite() && n.abs() <= 1000.0 => Ok(*n),
            _ => Err(format!("invalid scene field {key}").into()),
        }
    }

    fn project(&self) -> Result<Projection> {
        let ir = &self.program.ir;
        let engine = QueryEngine::new(ir, &self.store.it);
        let grid = engine
            .query_first(&Predicate::named("grid"))
            .ok_or("grid declaration missing")?
            .id;
        let number = |decl, key| -> Result<usize> {
            match engine.effective_value(decl, key) {
                Some(ValueR::Num(value))
                    if value.is_finite()
                        && *value >= 0.0
                        && value.fract() == 0.0
                        && *value <= 256.0 =>
                {
                    Ok(*value as usize)
                }
                _ => Err(format!("missing/invalid integer field {key}").into()),
            }
        };
        let mut layout = Layout {
            width: number(grid, "width")?,
            height: number(grid, "height")?,
            start: Point {
                x: number(grid, "start_x")?,
                y: number(grid, "start_y")?,
            },
            goal: Point {
                x: number(grid, "goal_x")?,
                y: number(grid, "goal_y")?,
            },
            walls: vec![],
        };
        layout.validate()?;
        let checks = self.structure_checks(layout.width, layout.height)?;
        let failed: Vec<_> = checks.iter().filter(|check| !check.passed).collect();
        if !failed.is_empty() {
            return Err(format!("query contract failed: {failed:?}").into());
        }
        let mut nodes = vec![None; layout.width * layout.height];
        let mut positions = vec![None; ir.decl_nodes.len()];
        for cell in engine.query(&Predicate::node().and(Predicate::inherits("grid_cell"))) {
            let point = Point {
                x: number(cell.id, "x")?,
                y: number(cell.id, "y")?,
            };
            if point.x >= layout.width || point.y >= layout.height {
                return Err("cell outside declared grid".into());
            }
            let slot = point.y * layout.width + point.x;
            if nodes[slot].is_some() {
                return Err("multiple cells at one coordinate".into());
            }
            nodes[slot] = Some(ir.as_node(cell.id).ok_or("grid cell is not a node")?);
            positions[cell.id.raw()] = Some(point);
            match number(cell.id, "blocked")? {
                0 => {}
                1 => layout.walls.push(point),
                _ => return Err("blocked must be 0 or 1".into()),
            }
        }
        layout.walls.sort_unstable_by_key(|p| (p.y, p.x));
        layout.validate()?;
        let nodes = nodes
            .into_iter()
            .collect::<Option<Vec<_>>>()
            .ok_or("grid has missing cells")?;
        Ok(Projection {
            layout,
            nodes,
            positions,
            checks,
        })
    }

    /// Reusable query assertions over canonical declarations, before search or export.
    fn structure_checks(&self, width: usize, height: usize) -> Result<Vec<QueryCheck>> {
        let engine = QueryEngine::new(&self.program.ir, &self.store.it);
        let patterns = hymeko_query::interpret::interpret_query_source(include_str!(
            "../data/adversarial_grid_queries.hymeko"
        ))?;
        let pattern = |label: &str| -> Result<Predicate> {
            patterns
                .iter()
                .find(|q| q.label == label)
                .map(|q| q.predicate.clone())
                .ok_or_else(|| format!("missing query pattern {label}").into())
        };
        let cells = pattern("cells")?;
        let field =
            |name: &str, comparison| Predicate::EffectiveChildValue(name.into(), comparison);
        let blocked = cells
            .clone()
            .and(field("blocked", ValuePredicate::NumEq(1.0)));
        let valid_fields = field("x", ValuePredicate::NumGte(0.0))
            .and(field("x", ValuePredicate::NumLt(width as f64)))
            .and(field("y", ValuePredicate::NumGte(0.0)))
            .and(field("y", ValuePredicate::NumLt(height as f64)))
            .and(
                field("blocked", ValuePredicate::NumEq(0.0))
                    .or(field("blocked", ValuePredicate::NumEq(1.0))),
            );
        let blocked_count = engine
            .check_query(
                &NamedQuery {
                    label: "blocked_cells".into(),
                    predicate: blocked.clone(),
                },
                MatchExpectation::AtMost(width * height),
                0,
            )
            .actual;
        let rules = [
            (
                "one_grid_record",
                Predicate::node().and(Predicate::named("grid")),
                MatchExpectation::Exactly(1),
            ),
            (
                "cell_count",
                cells.clone(),
                MatchExpectation::Exactly(width * height),
            ),
            (
                "invalid_cell_fields",
                cells.clone().and(valid_fields.not()),
                MatchExpectation::Exactly(0),
            ),
            (
                "move_to_blocked_cell",
                Predicate::edge().and(Predicate::HasRef(Box::new(blocked))),
                MatchExpectation::Exactly(0),
            ),
            (
                "move_to_non_cell",
                Predicate::edge().and(Predicate::HasRef(Box::new(cells.not()))),
                MatchExpectation::Exactly(0),
            ),
            (
                "non_neutral_move",
                Predicate::edge().and(
                    Predicate::HasPlusRef(Box::new(Predicate::Any))
                        .or(Predicate::HasMinusRef(Box::new(Predicate::Any))),
                ),
                MatchExpectation::Exactly(0),
            ),
            (
                "obstacle_count",
                pattern("obstacles")?,
                MatchExpectation::Exactly(blocked_count),
            ),
        ];
        Ok(rules
            .into_iter()
            .map(|(label, predicate, expected)| {
                engine.check_query(
                    &NamedQuery {
                        label: label.into(),
                        predicate,
                    },
                    expected,
                    8,
                )
            })
            .collect())
    }

    fn world_sdf(&self) -> Result<String> {
        let registry = hymeko_formats::default_registry();
        let config = hymeko_geometry::transforms::TransformConfig::default()
            .with_name("adversarial_grid")
            .with_option("world_name", "adversarial_grid");
        registry
            .render_from_templates(
                "gazebo",
                &self.program.ir,
                &self.store.it,
                &config,
                &hymeko_formats::gazebo::default_transforms_root(),
            )
            .ok_or("gazebo template is not registered")?
            .map_err(Into::into)
    }
}

struct Projection {
    layout: Layout,
    nodes: Vec<NodeId>,
    positions: Vec<Option<Point>>,
    checks: Vec<QueryCheck>,
}

fn check_receipts(checks: &[QueryCheck]) -> Vec<Value> {
    checks.iter().map(|c| json!({
        "rule": c.label, "expected": format!("{:?}", c.expected), "matches": c.actual,
        "passed": c.passed, "witness_decl_ids": c.witnesses.iter().map(|id| id.raw()).collect::<Vec<_>>(),
        "witnesses_truncated": c.witnesses_truncated,
    })).collect()
}

impl Projection {
    fn node(&self, point: Point) -> NodeId {
        self.nodes[point.y * self.layout.width + point.x]
    }

    fn point(&self, ir: &Ir, node: NodeId) -> Result<Point> {
        self.positions[ir.nodes[node.raw()].decl.raw()]
            .ok_or_else(|| "path leaves the declared grid cells".into())
    }

    fn evaluate(&self, world: &CompiledWorld, measure: bool) -> Result<Evaluation> {
        let ir = &world.program.ir;
        let index = build_incidence_index(ir)?;
        let traversal = HyperTraversal::new(&index);
        let (start, goal) = (self.node(self.layout.start), self.node(self.layout.goal));
        let bfs_clock = measure.then(Instant::now);
        let bfs = traversal.berge_shortest_path(&mut TraversalWorkspace::new(), start, goal)?;
        let bfs_ms = bfs_clock.map(|t| t.elapsed().as_secs_f64() * 1000.0);
        let Some(bfs) = bfs else {
            return Ok(Evaluation {
                feasible: false,
                shortest_path_length: None,
                path: vec![],
                astar: None,
                zero: None,
                bfs_ms,
                astar_ms: None,
                zero_ms: None,
            });
        };
        let mut workspace = InformedSearchWorkspace::new();
        let heuristic = Manhattan {
            positions: &self.positions,
            goal: self.layout.goal,
        };
        let astar_clock = measure.then(Instant::now);
        let astar = astar_berge_path(
            &traversal,
            start,
            goal,
            &UnitCost,
            &heuristic,
            &Undirected,
            &mut workspace,
        )?
        .ok_or("A* disagrees with BFS reachability")?;
        let astar_ms = astar_clock.map(|t| t.elapsed().as_secs_f64() * 1000.0);
        let astar_stats = Counters::from(workspace.stats());
        let zero_clock = measure.then(Instant::now);
        let zero = dijkstra_berge_path(
            &traversal,
            start,
            goal,
            &UnitCost,
            &Undirected,
            &mut workspace,
        )?
        .ok_or("zero heuristic disagrees with BFS reachability")?;
        let zero_ms = zero_clock.map(|t| t.elapsed().as_secs_f64() * 1000.0);
        let distance = bfs.hyperedge_len();
        if astar.total_cost != distance as f64 || zero.total_cost != distance as f64 {
            return Err("A*/zero/BFS shortest-path costs disagree".into());
        }
        let path: Vec<Point> = astar
            .nodes()
            .map(|n| self.point(ir, n))
            .collect::<Result<_>>()?;
        if path.first() != Some(&self.layout.start)
            || path.last() != Some(&self.layout.goal)
            || path.iter().any(|p| self.layout.blocked(*p))
            || path
                .windows(2)
                .any(|pair| pair[0].x.abs_diff(pair[1].x) + pair[0].y.abs_diff(pair[1].y) != 1)
        {
            return Err("path witness violates grid navigation semantics".into());
        }
        Ok(Evaluation {
            feasible: true,
            shortest_path_length: Some(distance),
            path,
            astar: Some(astar_stats),
            zero: Some(Counters::from(workspace.stats())),
            bfs_ms,
            astar_ms,
            zero_ms,
        })
    }
}

struct Manhattan<'a> {
    positions: &'a [Option<Point>],
    goal: Point,
}
impl BergeHeuristic for Manhattan<'_> {
    const ADMISSIBLE: bool = true;
    const CONSISTENT: bool = true;
    fn h(&self, _ir: &Ir, node: DeclId) -> f64 {
        self.positions[node.raw()].map_or(0.0, |p| {
            (p.x.abs_diff(self.goal.x) + p.y.abs_diff(self.goal.y)) as f64
        })
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
struct Counters {
    expanded_nodes: usize,
    expanded_arcs: usize,
    resolved_incidences: usize,
    pushes: usize,
    pops: usize,
    stale_pops: usize,
    heuristic_evals: usize,
    reopened: usize,
}
impl From<SearchStats> for Counters {
    fn from(s: SearchStats) -> Self {
        Self {
            expanded_nodes: s.expanded_nodes,
            expanded_arcs: s.expanded_arcs,
            resolved_incidences: s.resolved_incidences,
            pushes: s.pushes,
            pops: s.pops,
            stale_pops: s.stale_pops,
            heuristic_evals: s.heuristic_evals,
            reopened: s.reopened,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct Evaluation {
    feasible: bool,
    shortest_path_length: Option<usize>,
    path: Vec<Point>,
    astar: Option<Counters>,
    zero: Option<Counters>,
    bfs_ms: Option<f64>,
    astar_ms: Option<f64>,
    zero_ms: Option<f64>,
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
        let proposal = Layout::random(&self.config, rng);
        let authored = proposal.source()?;
        let generate_ms = started.elapsed().as_secs_f64() * 1000.0;
        let started = Instant::now();
        let (world, canonical) = CompiledWorld::canonical(&authored)?;
        let projection = world.project()?;
        if proposal != projection.layout {
            return Err("compiled layout disagrees with proposed occupancy".into());
        }
        let compile_ms = started.elapsed().as_secs_f64() * 1000.0;
        let evaluation = projection.evaluate(&world, true)?;
        let source_path = format!("seed_{seed}/candidates/candidate_{candidate_index}.hymeko");
        fs::write(self.output.join(&source_path), canonical)?;
        Ok(Sample {
            seed,
            candidate_index,
            wall_count: projection.layout.walls.len(),
            source_path,
            semantic_digest: world.digest(),
            query_checks: check_receipts(&projection.checks),
            layout: projection.layout,
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
    if args.first().is_some_and(|arg| arg == "--generate-maze") {
        return maze::run(&args[1..]);
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
                        "checks": check_receipts(&projection.checks),
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

    fn tiny() -> Layout {
        Layout {
            width: 4,
            height: 4,
            start: Point { x: 0, y: 0 },
            goal: Point { x: 3, y: 0 },
            walls: vec![
                Point { x: 1, y: 0 },
                Point { x: 1, y: 1 },
                Point { x: 1, y: 2 },
            ],
        }
    }

    #[test]
    fn queries_reject_wall_and_non_cell_references_before_search() {
        let original = tiny().source().unwrap();
        for (edge, rule) in [
            (
                "@illegal { (~ cell_0_0, ~ cell_1_0); }",
                "move_to_blocked_cell",
            ),
            ("@illegal { (~ cell_0_0, ~ grid); }", "move_to_non_cell"),
            ("@illegal { (+ cell_0_0, - cell_0_1); }", "non_neutral_move"),
        ] {
            let world = CompiledWorld::compile(&format!("{original}\n{edge}")).unwrap();
            let checks = world.structure_checks(4, 4).unwrap();
            let check = checks.iter().find(|c| c.label == rule).unwrap();
            assert!(!check.passed, "{rule}");
            assert_eq!(check.actual, 1);
            assert_eq!(check.witnesses.len(), 1);
            assert!(world.project().is_err());
        }
    }

    #[test]
    fn queries_count_cells_and_reject_invalid_bounds_and_missing_obstacles() {
        let original = tiny().source().unwrap();
        let world = CompiledWorld::compile(&original).unwrap();
        let checks = world.structure_checks(4, 4).unwrap();
        assert!(checks.iter().all(|c| c.passed));
        assert_eq!(
            checks
                .iter()
                .find(|c| c.label == "cell_count")
                .unwrap()
                .actual,
            16
        );
        let missing = world.structure_checks(5, 4).unwrap();
        assert!(
            !missing
                .iter()
                .find(|c| c.label == "cell_count")
                .unwrap()
                .passed
        );
        let wrong_position = original.replacen("x 0; y 0;", "x 9; y 0;", 1);
        let world = CompiledWorld::compile(&wrong_position).unwrap();
        assert!(
            !world
                .structure_checks(4, 4)
                .unwrap()
                .iter()
                .find(|c| c.label == "invalid_cell_fields")
                .unwrap()
                .passed
        );
        assert!(world.project().is_err());
        let no_obstacles = original.replace(": vocabulary.gazebo_static_box", "");
        let world = CompiledWorld::compile(&no_obstacles).unwrap();
        assert!(
            !world
                .structure_checks(4, 4)
                .unwrap()
                .iter()
                .find(|c| c.label == "obstacle_count")
                .unwrap()
                .passed
        );
    }

    #[test]
    fn seed_replay_variation_and_exact_wall_count() {
        let config = Config::default();
        let mut a = Lcg::new(42);
        let mut b = Lcg::new(42);
        let mut c = Lcg::new(43);
        let first = Layout::random(&config, &mut a);
        assert_eq!(first, Layout::random(&config, &mut b));
        assert_ne!(first, Layout::random(&config, &mut c));
        assert_ne!(first, Layout::random(&config, &mut a));
        assert_eq!(first.walls.len(), 64);
        first.validate().unwrap();
    }

    #[test]
    fn known_maze_has_nine_step_optimal_witness_and_roundtrips() {
        let layout = tiny();
        let (world, source) = CompiledWorld::canonical(&layout.source().unwrap()).unwrap();
        let projection = world.project().unwrap();
        assert_eq!(projection.layout, layout);
        let evaluation = projection.evaluate(&world, false).unwrap();
        assert_eq!(evaluation.shortest_path_length, Some(9));
        assert_eq!(evaluation.path.len(), 10);
        assert_eq!(evaluation.astar.as_ref().unwrap().reopened, 0);
        assert_eq!(
            evaluation.astar_ms, None,
            "correctness test does not time search"
        );
        assert_eq!(
            world.digest(),
            CompiledWorld::compile(&source).unwrap().digest()
        );
        assert_eq!(
            evaluation.astar,
            projection.evaluate(&world, false).unwrap().astar
        );
    }

    #[test]
    fn disconnected_world_is_unscored() {
        let mut layout = tiny();
        layout.walls.push(Point { x: 1, y: 3 });
        let world = CompiledWorld::compile(&layout.source().unwrap()).unwrap();
        let evaluation = world.project().unwrap().evaluate(&world, false).unwrap();
        assert!(!evaluation.feasible);
        assert!(
            evaluation.astar.is_none() && evaluation.zero.is_none() && evaluation.path.is_empty()
        );
    }

    #[test]
    fn invalid_endpoints_duplicates_and_missing_fields_fail() {
        let mut layout = tiny();
        layout.walls.push(layout.start);
        assert!(layout.source().is_err());
        let mut layout = tiny();
        layout.walls.push(layout.walls[0]);
        assert!(layout.source().is_err());
        let source = tiny().source().unwrap().replace("blocked 0;", "blocked 2;");
        assert!(CompiledWorld::compile(&source).unwrap().project().is_err());
        assert!(CompiledWorld::compile("broken {").is_err());
    }

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
    fn canonical_obstacles_reach_existing_gazebo_template() {
        let layout = tiny();
        let (world, _) = CompiledWorld::canonical(&layout.source().unwrap()).unwrap();
        let sdf = world.world_sdf().unwrap();
        for wall in &layout.walls {
            assert!(sdf.contains(&format!("<model name=\"wall_{}_{}\">", wall.x, wall.y)));
        }
        assert!(sdf.contains("<size>1 1 1</size>") || sdf.contains("<size>1.0 1.0 1.0</size>"));
    }
}
