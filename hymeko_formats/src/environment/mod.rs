//! Validated environment projections and source conversion over Core snapshots.
//! Graph owns path and spanning-basis algorithms; Geometry owns collision and FK.
//! Campaign policy and command-line execution live in the application binary.
pub mod locomotion;
pub mod maze;
pub mod maze_physics;
mod physical;
pub mod team;
pub mod terrain;
/// Closed roads read from IR into kawakern's `track`.
pub mod track;
mod visual;

use hymeko::{
    common::ids::{DeclId, NodeId},
    ir::{
        canonical_hash::Canon,
        incidence::build_incidence_index,
        ir::{Ir, ValueR},
    },
    module_store::{execution::CompiledModel, source_provider::MemProvider},
    util::real_parser::RealParser,
};
use hymeko_graph::direct_ir::{
    BergeHeuristic, HyperTraversal, InformedSearchWorkspace, SearchStats, TraversalWorkspace,
    Undirected, UnitCost, astar_berge_path, dijkstra_berge_path,
};
use hymeko_query::{
    MatchExpectation, NamedQuery, Predicate, QueryCheck, QueryEngine, ValuePredicate,
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{error::Error, fmt::Write as _, fs, path::Path, time::Instant};

pub type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Point {
    pub x: usize,
    pub y: usize,
}

/// A proposal/observation DTO. The canonical compiled IR owns all scored connectivity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Layout {
    pub width: usize,
    pub height: usize,
    pub start: Point,
    pub goal: Point,
    pub walls: Vec<Point>,
}

impl Layout {
    fn blocked(&self, point: Point) -> bool {
        self.walls.contains(&point)
    }

    pub fn validate(&self) -> Result<()> {
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

    pub fn source(&self) -> Result<String> {
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

pub struct CompiledWorld {
    model: CompiledModel,
}

impl CompiledWorld {
    /// Compile using Core's owning facade; retain it for all downstream queries.
    pub fn compile(source: &str) -> Result<Self> {
        let model = CompiledModel::compile(
            MemProvider::default().with_file("/grid.hymeko", source),
            RealParser,
            Path::new("/grid.hymeko"),
        )
        .map_err(|error| format!("grid compilation: {error:?}"))?;
        Ok(Self { model })
    }

    pub fn digest(&self) -> String {
        Canon::new(&self.model.program().ir, self.model.interner())
            .hash()
            .0
            .0
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect()
    }

    pub fn canonical(source: &str) -> Result<(Self, String)> {
        let original = Self::compile(source)?;
        let (recompiled, emitted) = original.reemit("adversarial_grid")?;
        if original.project()?.layout != recompiled.project()?.layout {
            return Err("canonical source round trip changed the projected layout".into());
        }
        Ok((recompiled, emitted))
    }

    pub fn reemit(&self, header: &str) -> Result<(Self, String)> {
        let emitted =
            hymeko_emitter::emit_hymeko(&self.model.program().ir, self.model.interner(), header);
        let recompiled = Self::compile(&emitted)?;
        if self.digest() != recompiled.digest() {
            return Err("canonical source round trip changed the semantic digest".into());
        }
        Ok((recompiled, emitted))
    }

    fn scene_scalar(&self, decl: DeclId, key: &str) -> Result<f64> {
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        match engine.effective_value(decl, key) {
            Some(ValueR::Num(n)) if n.is_finite() && n.abs() <= 1000.0 => Ok(*n),
            _ => Err(format!("invalid scene field {key}").into()),
        }
    }

    pub fn project(&self) -> Result<Projection<'_>> {
        let ir = &self.model.program().ir;
        let engine = QueryEngine::new(ir, self.model.interner());
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
            world: self,
            layout,
            nodes,
            positions,
            checks,
        })
    }

    /// Reusable query assertions over canonical declarations, before search or export.
    pub fn structure_checks(&self, width: usize, height: usize) -> Result<Vec<QueryCheck>> {
        let engine = QueryEngine::new(&self.model.program().ir, self.model.interner());
        let patterns = hymeko_query::interpret::interpret_query_source(include_str!(
            "../../data/adversarial_grid_queries.hymeko"
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

    pub fn world_sdf(&self) -> Result<String> {
        let registry = crate::default_registry();
        let config = crate::plugin::TransformConfig::default()
            .with_name("adversarial_grid")
            .with_option("world_name", "adversarial_grid");
        registry
            .render(
                "gazebo",
                &QueryEngine::new(&self.model.program().ir, self.model.interner()),
                &config,
                &crate::gazebo::default_transforms_root(),
            )
            .ok_or("gazebo template is not registered")?
            .map_err(Into::into)
    }
}

/// Borrowed projection: its local indices always refer to this Core snapshot.
pub struct Projection<'a> {
    world: &'a CompiledWorld,
    layout: Layout,
    nodes: Vec<NodeId>,
    positions: Vec<Option<Point>>,
    checks: Vec<QueryCheck>,
}

pub fn check_receipts(checks: &[QueryCheck]) -> Vec<Value> {
    checks.iter().map(|c| json!({
        "rule": c.label, "expected": format!("{:?}", c.expected), "matches": c.actual,
        "passed": c.passed, "witness_decl_ids": c.witnesses.iter().map(|id| id.raw()).collect::<Vec<_>>(),
        "witnesses_truncated": c.witnesses_truncated,
    })).collect()
}

impl Projection<'_> {
    pub fn layout(&self) -> &Layout {
        &self.layout
    }
    pub fn checks(&self) -> &[QueryCheck] {
        &self.checks
    }

    fn node(&self, point: Point) -> NodeId {
        self.nodes[point.y * self.layout.width + point.x]
    }

    fn point(&self, ir: &Ir, node: NodeId) -> Result<Point> {
        self.positions[ir.nodes[node.raw()].decl.raw()]
            .ok_or_else(|| "path leaves the declared grid cells".into())
    }

    pub fn evaluate(&self, measure: bool) -> Result<Evaluation> {
        let ir = &self.world.model.program().ir;
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
pub struct Counters {
    pub expanded_nodes: usize,
    pub expanded_arcs: usize,
    pub resolved_incidences: usize,
    pub pushes: usize,
    pub pops: usize,
    pub stale_pops: usize,
    pub heuristic_evals: usize,
    pub reopened: usize,
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
pub struct Evaluation {
    pub feasible: bool,
    pub shortest_path_length: Option<usize>,
    pub path: Vec<Point>,
    pub astar: Option<Counters>,
    pub zero: Option<Counters>,
    pub bfs_ms: Option<f64>,
    pub astar_ms: Option<f64>,
    pub zero_ms: Option<f64>,
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
    fn known_maze_has_nine_step_optimal_witness_and_roundtrips() {
        let layout = tiny();
        let (world, source) = CompiledWorld::canonical(&layout.source().unwrap()).unwrap();
        let projection = world.project().unwrap();
        assert_eq!(projection.layout, layout);
        let evaluation = projection.evaluate(false).unwrap();
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
        assert_eq!(evaluation.astar, projection.evaluate(false).unwrap().astar);
    }

    #[test]
    fn disconnected_world_is_unscored() {
        let mut layout = tiny();
        layout.walls.push(Point { x: 1, y: 3 });
        let world = CompiledWorld::compile(&layout.source().unwrap()).unwrap();
        let evaluation = world.project().unwrap().evaluate(false).unwrap();
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
