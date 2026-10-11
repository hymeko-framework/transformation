//! Admitted maze navigation over the retained Core model and existing Graph kernels.
use super::*;
use hymeko_graph::direct_ir::{MetricHeuristic, PositionProvider};

/// Read-only domain projection. No second graph or search implementation is built.
pub struct MazeNavigation<'a> {
    world: &'a CompiledWorld,
    geometry: MazeGeometry,
    nodes: Vec<NodeId>,
    positions: Vec<Option<[f64; 2]>>,
}

#[derive(Clone, Debug, Serialize)]
pub struct SearchRun {
    pub algorithm: &'static str,
    pub cost: usize,
    pub route_cells: Vec<usize>,
    pub route_points_m: Vec<[f64; 2]>,
    /// Warm search including returned-path construction, excluding admission and validation.
    pub samples_ns: Vec<u64>,
    /// BFS does not expose counters through its public API.
    pub counters: Option<Counters>,
}

impl PositionProvider for MazeNavigation<'_> {
    fn position(&self, node: DeclId) -> Option<&[f64]> {
        self.positions
            .get(node.raw())?
            .as_ref()
            .map(|p| p.as_slice())
    }
}

impl<'a> MazeNavigation<'a> {
    pub fn new(world: &'a CompiledWorld) -> Result<Self> {
        let (geometry, _) = Topology::project(world)?;
        let ir = &world.model.program().ir;
        let mut nodes = vec![NodeId::new(0); geometry.cells.len()];
        let mut positions = vec![None; ir.decl_nodes.len()];
        for (id, index) in Topology::cell_ids(world)? {
            nodes[index] = ir.as_node(id).ok_or("maze cell is not a node")?;
            positions[id.raw()] = Some(geometry.cells[index].centre_m);
        }
        let max_step = geometry
            .passages
            .iter()
            .map(|[a, b]| {
                let (a, b) = (geometry.cells[*a].centre_m, geometry.cells[*b].centre_m);
                (a[0] - b[0]).hypot(a[1] - b[1])
            })
            .fold(0.0_f64, f64::max);
        if !max_step.is_finite() || max_step <= 0.0 {
            return Err("invalid maze edge geometry".into());
        }
        // Each scaled passage chord is <= 1. Graph's metric heuristic is therefore
        // consistent with UnitCost too, by the triangle inequality. Round upward
        // slightly to avoid overstating a unit step through f64 rounding.
        for position in positions.iter_mut().flatten() {
            *position = position.map(|v| v / (max_step * (1.0 + 1e-12)));
        }
        Ok(Self {
            world,
            geometry,
            nodes,
            positions,
        })
    }

    pub fn geometry(&self) -> &MazeGeometry {
        &self.geometry
    }

    /// All methods minimize passage count. Zero repetitions disables clocks entirely.
    /// Positive repetitions run one warmup then bounded, rotated measurements.
    pub fn evaluate(
        &self,
        start: usize,
        goal: usize,
        repetitions: usize,
    ) -> Result<Vec<SearchRun>> {
        if start >= self.nodes.len() || goal >= self.nodes.len() || repetitions > 1000 {
            return Err("maze endpoint outside map or repetitions exceed 1000".into());
        }
        let ir = &self.world.model.program().ir;
        let index = build_incidence_index(ir)?;
        let traversal = HyperTraversal::new(&index);
        let mut bfs_ws = TraversalWorkspace::new();
        let mut informed = [
            InformedSearchWorkspace::new(),
            InformedSearchWorkspace::new(),
        ];
        let heuristic = MetricHeuristic::new(self, ir.nodes[self.nodes[goal].raw()].decl);
        let ids = Topology::cell_ids(self.world)?;
        let mut runs: Vec<SearchRun> = ["bfs", "dijkstra", "astar"]
            .into_iter()
            .map(|algorithm| SearchRun {
                algorithm,
                cost: 0,
                route_cells: vec![],
                route_points_m: vec![],
                samples_ns: vec![],
                counters: None,
            })
            .collect();
        for iteration in 0..=repetitions {
            for offset in 0..3 {
                let method = (start + goal + iteration + offset) % 3;
                let clock = (iteration > 0).then(Instant::now);
                let (nodes, cost, counters) = if method == 0 {
                    let path = traversal
                        .berge_shortest_path(&mut bfs_ws, self.nodes[start], self.nodes[goal])?
                        .ok_or("admitted maze has no BFS route")?;
                    let elapsed = clock.map(|c| c.elapsed().as_nanos() as u64);
                    if let Some(ns) = elapsed {
                        runs[method].samples_ns.push(ns);
                    }
                    (path.nodes().collect::<Vec<_>>(), path.hyperedge_len(), None)
                } else {
                    let workspace = &mut informed[method - 1];
                    let path = if method == 1 {
                        dijkstra_berge_path(
                            &traversal,
                            self.nodes[start],
                            self.nodes[goal],
                            &UnitCost,
                            &Undirected,
                            workspace,
                        )?
                    } else {
                        astar_berge_path(
                            &traversal,
                            self.nodes[start],
                            self.nodes[goal],
                            &UnitCost,
                            &heuristic,
                            &Undirected,
                            workspace,
                        )?
                    }
                    .ok_or("admitted maze has no informed route")?;
                    let elapsed = clock.map(|c| c.elapsed().as_nanos() as u64);
                    if let Some(ns) = elapsed {
                        runs[method].samples_ns.push(ns);
                    }
                    if path.total_cost != path.hyperedge_len() as f64 {
                        return Err("non-unit path cost".into());
                    }
                    (
                        path.nodes().collect(),
                        path.hyperedge_len(),
                        Some(Counters::from(workspace.stats())),
                    )
                };
                let route: Vec<_> = nodes.iter().map(|n| ids[&ir.nodes[n.raw()].decl]).collect();
                if iteration == 0 {
                    runs[method].cost = cost;
                    runs[method].route_cells = route;
                    runs[method].counters = counters;
                } else if runs[method].cost != cost
                    || runs[method].route_cells != route
                    || runs[method].counters != counters
                {
                    return Err("search replay changed witness or counters".into());
                }
            }
        }
        for run in &runs {
            if run.cost != runs[0].cost
                || run.route_cells.first() != Some(&start)
                || run.route_cells.last() != Some(&goal)
                || run.route_cells.len() != run.cost + 1
                || run.route_cells.windows(2).any(|p| {
                    !self
                        .geometry
                        .passages
                        .contains(&[p[0].min(p[1]), p[0].max(p[1])])
                })
            {
                return Err("graph search cost or passage witness disagreement".into());
            }
        }
        let topology = Topology::new(Profile::parse(&self.geometry.profile)?);
        for run in &mut runs {
            run.route_points_m = topology.route_points(&run.route_cells);
            let mut witness = self.geometry.clone();
            witness.route_points_m = run.route_points_m.clone();
            witness.check_clearance(CLEARANCE)?;
        }
        Ok(runs)
    }
}
