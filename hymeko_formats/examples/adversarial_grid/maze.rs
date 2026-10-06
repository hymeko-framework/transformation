//! Small polar/hex maze profiles: canonical IR owns passages, Graph owns selection/search.
use super::*;
use hymeko::ir::ir::SignedRefR;
use hymeko_geometry::collide::{Convex, Placement, intersects};
use hymeko_graph::{
    GraphicMatroidWorkspace, MatroidRef, rank2_spanning_forest, rank2_weighted_basis,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    f64::consts::PI,
};

const WALL_WIDTH: f64 = 0.12;
const CLEARANCE: f64 = 0.85;

#[derive(Clone, Debug, Serialize, PartialEq)]
pub(super) struct MazeCell {
    pub(super) index: usize,
    pub(super) centre_m: [f64; 2],
}

#[derive(Clone, Debug, Serialize, PartialEq)]
pub(super) struct MazeWall {
    pub(super) index: usize,
    pub(super) centre_m: [f64; 3],
    pub(super) full_size_m: [f64; 3],
    pub(super) yaw_rad: f64,
}

/// Export projection only. The retained compiled source is the topology authority.
#[derive(Clone, Debug, Serialize)]
pub(super) struct MazeGeometry {
    pub(super) profile: String,
    pub(super) seed: u64,
    pub(super) cells: Vec<MazeCell>,
    pub(super) passages: Vec<[usize; 2]>,
    pub(super) walls: Vec<MazeWall>,
    pub(super) start: usize,
    pub(super) goal: usize,
    pub(super) route_cell_indices: Vec<usize>,
    pub(super) route_points_m: Vec<[f64; 2]>,
    pub(super) clearance_radius_m: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Profile {
    Hex,
    Polar,
}

impl Profile {
    fn parse(name: &str) -> Result<Self> {
        match name {
            "hex" => Ok(Self::Hex),
            "polar" => Ok(Self::Polar),
            _ => Err("maze profile must be hex or polar".into()),
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Hex => "hex",
            Self::Polar => "polar",
        }
    }
}

struct Boundary {
    a: usize,
    b: Option<usize>,
    points: Vec<[f64; 2]>,
}

impl Boundary {
    fn pair(&self) -> Option<[usize; 2]> {
        self.b.map(|b| [self.a.min(b), self.a.max(b)])
    }
}

struct Topology {
    profile: Profile,
    cells: Vec<MazeCell>,
    boundaries: Vec<Boundary>,
}

impl Topology {
    fn new(profile: Profile) -> Self {
        match profile {
            Profile::Hex => Self::hex(2),
            Profile::Polar => Self::polar(),
        }
    }

    fn hex(radius: i32) -> Self {
        let side = 2.0;
        let centre = |q: i32, r: i32| {
            [
                3.0_f64.sqrt() * side * (q as f64 + r as f64 * 0.5),
                1.5 * side * r as f64,
            ]
        };
        let mut coordinates = BTreeMap::new();
        let mut cells = Vec::new();
        for q in -radius..=radius {
            for r in -radius..=radius {
                if (q + r).abs() <= radius {
                    coordinates.insert((q, r), cells.len());
                    cells.push(MazeCell {
                        index: cells.len(),
                        centre_m: centre(q, r),
                    });
                }
            }
        }
        let mut boundaries = Vec::new();
        for (&(q, r), &a) in &coordinates {
            let p = cells[a].centre_m;
            for (dq, dr) in [(1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1)] {
                let b = coordinates.get(&(q + dq, r + dr)).copied();
                if b.is_some_and(|b| a > b) {
                    continue;
                }
                let n = centre(q + dq, r + dr);
                let length = (n[0] - p[0]).hypot(n[1] - p[1]);
                let tangent = [
                    -(n[1] - p[1]) / length * side * 0.5,
                    (n[0] - p[0]) / length * side * 0.5,
                ];
                let middle = [(p[0] + n[0]) * 0.5, (p[1] + n[1]) * 0.5];
                boundaries.push(Boundary {
                    a,
                    b,
                    points: vec![
                        [middle[0] - tangent[0], middle[1] - tangent[1]],
                        [middle[0] + tangent[0], middle[1] + tangent[1]],
                    ],
                });
            }
        }
        Self {
            profile: Profile::Hex,
            cells,
            boundaries,
        }
    }

    fn polar() -> Self {
        let mut cells = Vec::new();
        let mut boundaries = Vec::new();
        let point = |radius: f64, angle: f64| [radius * angle.cos(), radius * angle.sin()];
        for ring in 0..2 {
            for sector in 0..6 {
                let a = ring * 6 + sector;
                let inner = 2.6 + ring as f64 * 3.0;
                let lower = sector as f64 * PI / 3.0;
                let upper = (sector + 1) as f64 * PI / 3.0;
                cells.push(MazeCell {
                    index: a,
                    centre_m: point(inner + 1.5, (lower + upper) * 0.5),
                });
                boundaries.push(Boundary {
                    a,
                    b: Some(ring * 6 + (sector + 1) % 6),
                    points: vec![point(inner, upper), point(inner + 3.0, upper)],
                });
                boundaries.push(Boundary {
                    a,
                    b: (ring > 0).then(|| a - 6),
                    points: (0..=3)
                        .map(|i| point(inner, lower + (upper - lower) * i as f64 / 3.0))
                        .collect(),
                });
                if ring == 1 {
                    boundaries.push(Boundary {
                        a,
                        b: None,
                        points: (0..=3)
                            .map(|i| point(inner + 3.0, lower + (upper - lower) * i as f64 / 3.0))
                            .collect(),
                    });
                }
            }
        }
        Self {
            profile: Profile::Polar,
            cells,
            boundaries,
        }
    }

    fn walls(&self, passages: &BTreeSet<[usize; 2]>) -> Vec<MazeWall> {
        let mut walls = Vec::new();
        for boundary in &self.boundaries {
            if boundary.pair().is_some_and(|p| passages.contains(&p)) {
                continue;
            }
            for pair in boundary.points.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                walls.push(MazeWall {
                    index: walls.len(),
                    centre_m: [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, 0.5],
                    // Extend joins by one half-width at each end to close faceted corners.
                    full_size_m: [
                        (b[0] - a[0]).hypot(b[1] - a[1]) + WALL_WIDTH,
                        WALL_WIDTH,
                        1.0,
                    ],
                    yaw_rad: (b[1] - a[1]).atan2(b[0] - a[0]),
                });
            }
        }
        walls
    }

    fn source(
        &self,
        seed: u64,
        passages: &BTreeSet<[usize; 2]>,
        weights: Option<&BTreeMap<[usize; 2], u64>>,
    ) -> Result<String> {
        let walls = if weights.is_some() {
            vec![]
        } else {
            self.walls(passages)
        };
        let mut source = format!(
            "generated_maze {{}}\nvocabulary {{ maze_cell {{}} maze_wall {{}} }}\nmaze {{ profile \"{}\"; seed \"{seed}\"; cell_count {}; wall_count {}; start_index 0; goal_index {}; }}\n",
            self.profile.name(),
            self.cells.len(),
            walls.len(),
            self.cells.len() - 1
        );
        for cell in &self.cells {
            writeln!(
                source,
                "cell_{}: vocabulary.maze_cell {{ index {}; x {}; y {}; }}",
                cell.index, cell.index, cell.centre_m[0], cell.centre_m[1]
            )?;
        }
        for [a, b] in passages {
            let weight = weights
                .and_then(|w| w.get(&[*a, *b]))
                .map_or(String::new(), |w| format!(" [{w}]"));
            writeln!(
                source,
                "@passage_{a}_{b} {{ (~ cell_{a}{weight}, ~ cell_{b}); }}"
            )?;
        }
        for wall in walls {
            writeln!(
                source,
                "wall_{}: vocabulary.maze_wall {{ index {}; x {}; y {}; z {}; size_x {}; size_y {}; size_z {}; yaw {}; }}",
                wall.index,
                wall.index,
                wall.centre_m[0],
                wall.centre_m[1],
                wall.centre_m[2],
                wall.full_size_m[0],
                wall.full_size_m[1],
                wall.full_size_m[2],
                wall.yaw_rad
            )?;
        }
        Ok(source)
    }

    fn generate(&self, seed: u64) -> Result<(CompiledWorld, String, MazeGeometry, Vec<Value>)> {
        let mut rng = Lcg::new(seed);
        let weights: BTreeMap<_, _> = self
            .boundaries
            .iter()
            .filter_map(Boundary::pair)
            .map(|p| (p, rng.next_u64() >> 11))
            .collect();
        let all = weights.keys().copied().collect();
        let proposed = CompiledWorld::compile(&self.source(seed, &all, Some(&weights))?)?;
        let index = build_incidence_index(&proposed.program.ir)?;
        let mut basis = Vec::new();
        rank2_weighted_basis(
            MatroidRef::new(&index),
            true,
            &mut GraphicMatroidWorkspace::new(),
            &mut basis,
        )?;
        let ids = Self::cell_ids(&proposed)?;
        let mut passages = BTreeSet::new();
        for arc in basis {
            passages.insert(Self::arc_pair(&proposed.program.ir.arcs[arc.raw()], &ids)?);
        }
        if passages.len() + 1 != self.cells.len() {
            return Err("candidate graph is disconnected".into());
        }
        let original = CompiledWorld::compile(&self.source(seed, &passages, None)?)?;
        let (world, source) = original.reemit("generated_maze")?;
        let (geometry, checks) = Self::project(&world)?;
        Ok((world, source, geometry, checks))
    }

    fn cell_ids(world: &CompiledWorld) -> Result<BTreeMap<DeclId, usize>> {
        let query = QueryEngine::new(&world.program.ir, &world.store.it);
        query
            .query(&Predicate::node().and(Predicate::inherits("maze_cell")))
            .into_iter()
            .map(|m| match query.effective_value(m.id, "index") {
                Some(ValueR::Num(n))
                    if n.is_finite() && (0.0..19.0).contains(n) && n.fract() == 0.0 =>
                {
                    Ok((m.id, *n as usize))
                }
                _ => Err("invalid maze cell index".into()),
            })
            .collect()
    }

    fn arc_pair(arc: &hymeko::ir::ir::ArcRec, ids: &BTreeMap<DeclId, usize>) -> Result<[usize; 2]> {
        if arc.refs.len() != 2
            || arc
                .refs
                .iter()
                .any(|r| !matches!(r, SignedRefR::Neutral(_)))
        {
            return Err("maze passage must have two neutral cell incidences".into());
        }
        let a = *ids
            .get(&arc.refs[0].target())
            .ok_or("passage endpoint is not a maze cell")?;
        let b = *ids
            .get(&arc.refs[1].target())
            .ok_or("passage endpoint is not a maze cell")?;
        if a == b {
            return Err("self passage".into());
        }
        Ok([a.min(b), a.max(b)])
    }

    fn project(world: &CompiledWorld) -> Result<(MazeGeometry, Vec<Value>)> {
        let ir = &world.program.ir;
        let query = QueryEngine::new(ir, &world.store.it);
        let meta = query
            .query_first(&Predicate::named("maze"))
            .ok_or("missing maze record")?
            .id;
        let scalar = |id, key| world.scene_scalar(id, key);
        let string = |key| -> Result<&str> {
            match query.effective_value(meta, key) {
                Some(ValueR::Str(s)) => Ok(world.store.it.resolve(*s)),
                _ => Err("missing maze profile/seed".into()),
            }
        };
        let profile = Profile::parse(string("profile")?)?;
        let seed = string("seed")?.parse()?;
        let expected = Self::new(profile);
        let cell_count = expected.cells.len();
        if scalar(meta, "cell_count")? != cell_count as f64
            || scalar(meta, "start_index")? != 0.0
            || scalar(meta, "goal_index")? != (cell_count - 1) as f64
        {
            return Err("maze profile endpoints/count mismatch".into());
        }
        let ids = Self::cell_ids(world)?;
        let mut cells = Vec::new();
        for (&id, &index) in &ids {
            cells.push(MazeCell {
                index,
                centre_m: [scalar(id, "x")?, scalar(id, "y")?],
            });
        }
        cells.sort_by_key(|c| c.index);
        if cells.len() != cell_count
            || cells.iter().zip(&expected.cells).any(|(a, b)| {
                a.index != b.index
                    || a.centre_m
                        .iter()
                        .zip(b.centre_m)
                        .any(|(a, b)| (a - b).abs() > 1e-9)
            })
        {
            return Err("cell coordinates disagree with declared topology".into());
        }
        let allowed: BTreeSet<_> = expected
            .boundaries
            .iter()
            .filter_map(Boundary::pair)
            .collect();
        let mut passages = BTreeSet::new();
        for arc in &ir.arcs {
            let pair = Self::arc_pair(arc, &ids)?;
            if !allowed.contains(&pair) || !passages.insert(pair) {
                return Err("duplicate or non-neighbor passage".into());
            }
        }
        let mut walls = Vec::new();
        for m in query.query(&Predicate::node().and(Predicate::inherits("maze_wall"))) {
            let number = scalar(m.id, "index")?;
            if number < 0.0 || number.fract() != 0.0 {
                return Err("invalid wall index".into());
            }
            walls.push(MazeWall {
                index: number as usize,
                centre_m: [scalar(m.id, "x")?, scalar(m.id, "y")?, scalar(m.id, "z")?],
                full_size_m: [
                    scalar(m.id, "size_x")?,
                    scalar(m.id, "size_y")?,
                    scalar(m.id, "size_z")?,
                ],
                yaw_rad: scalar(m.id, "yaw")?,
            });
        }
        walls.sort_by_key(|w| w.index);
        let expected_walls = expected.walls(&passages);
        if walls.len() != expected_walls.len()
            || scalar(meta, "wall_count")? != walls.len() as f64
            || walls.iter().zip(&expected_walls).any(|(a, b)| {
                a.index != b.index
                    || a.centre_m
                        .iter()
                        .chain(&a.full_size_m)
                        .chain([&a.yaw_rad])
                        .zip(b.centre_m.iter().chain(&b.full_size_m).chain([&b.yaw_rad]))
                        .any(|(a, b)| (a - b).abs() > 1e-9)
            })
        {
            return Err("walls disagree with closed maze boundaries".into());
        }
        let mut basis = Vec::new();
        let index = build_incidence_index(ir)?;
        rank2_spanning_forest(
            MatroidRef::new(&index),
            &mut GraphicMatroidWorkspace::new(),
            &mut basis,
        )?;
        if basis.len() + 1 != cell_count || basis.len() != passages.len() {
            return Err("maze passages are not a spanning tree".into());
        }
        let nodes: BTreeMap<_, _> = ids
            .iter()
            .map(|(&id, &i)| (i, ir.as_node(id).expect("queried cell node")))
            .collect();
        let traversal = HyperTraversal::new(&index);
        let bfs = traversal
            .berge_shortest_path(
                &mut TraversalWorkspace::new(),
                nodes[&0],
                nodes[&(cell_count - 1)],
            )?
            .ok_or("maze endpoints disconnected")?;
        let path = dijkstra_berge_path(
            &traversal,
            nodes[&0],
            nodes[&(cell_count - 1)],
            &UnitCost,
            &Undirected,
            &mut InformedSearchWorkspace::new(),
        )?
        .ok_or("maze Dijkstra path missing")?;
        if path.total_cost != bfs.hyperedge_len() as f64 {
            return Err("maze BFS/Dijkstra cost disagreement".into());
        }
        let route_cell_indices: Vec<_> =
            path.nodes().map(|n| ids[&ir.nodes[n.raw()].decl]).collect();
        let route_points_m = expected.route_points(&route_cell_indices);
        let geometry = MazeGeometry {
            profile: profile.name().into(),
            seed,
            cells,
            passages: passages.into_iter().collect(),
            walls,
            start: 0,
            goal: cell_count - 1,
            route_cell_indices,
            route_points_m,
            clearance_radius_m: CLEARANCE,
        };
        geometry.check_clearance(CLEARANCE)?;
        let mut checks = Vec::new();
        for (label, predicate, count) in [
            (
                "one_maze",
                Predicate::node().and(Predicate::named("maze")),
                1,
            ),
            (
                "maze_cells",
                Predicate::node().and(Predicate::inherits("maze_cell")),
                cell_count,
            ),
            (
                "closed_boundary_boxes",
                Predicate::node().and(Predicate::inherits("maze_wall")),
                geometry.walls.len(),
            ),
            ("tree_passages", Predicate::edge(), cell_count - 1),
        ] {
            let check = query.check_query(
                &NamedQuery {
                    label: label.into(),
                    predicate,
                },
                MatchExpectation::Exactly(count),
                8,
            );
            if !check.passed {
                return Err(format!("maze query check {label} failed").into());
            }
            checks.extend(check_receipts(&[check]));
        }
        Ok((geometry, checks))
    }

    fn route_points(&self, route: &[usize]) -> Vec<[f64; 2]> {
        let mut points = vec![self.cells[route[0]].centre_m];
        for pair in route.windows(2) {
            let (a, b) = (self.cells[pair[0]].centre_m, self.cells[pair[1]].centre_m);
            if self.profile == Profile::Polar && pair[0] / 6 == pair[1] / 6 {
                let start = a[1].atan2(a[0]);
                let delta = (b[1].atan2(b[0]) - start + PI).rem_euclid(2.0 * PI) - PI;
                let angle = start + delta * 0.5;
                let radius = a[0].hypot(a[1]);
                points.push([radius * angle.cos(), radius * angle.sin()]);
            }
            points.push(b);
        }
        points
    }
}

impl MazeGeometry {
    fn check_clearance(&self, radius: f64) -> Result<()> {
        if !radius.is_finite() || radius <= 0.0 {
            return Err("clearance radius must be finite and positive".into());
        }
        for (segment, pair) in self.route_points_m.windows(2).enumerate() {
            let dx = pair[1][0] - pair[0][0];
            let dy = pair[1][1] - pair[0][1];
            let length = dx.hypot(dy);
            if !length.is_finite() || length <= 0.0 {
                return Err("invalid route segment".into());
            }
            let (x, y) = (dx / length, dy / length);
            let sweep = Convex::Capsule {
                radius,
                half_height: length * 0.5,
            };
            let placement = Placement {
                rotation: [-y, 0., x, x, 0., y, 0., 1., 0.],
                centre: [
                    (pair[0][0] + pair[1][0]) * 0.5,
                    (pair[0][1] + pair[1][1]) * 0.5,
                    0.5,
                ],
            };
            for wall in &self.walls {
                let (s, c) = wall.yaw_rad.sin_cos();
                let box_shape = Convex::Cuboid {
                    half: wall.full_size_m.map(|v| v * 0.5),
                };
                let wall_pose = Placement {
                    rotation: [c, -s, 0., s, c, 0., 0., 0., 1.],
                    centre: wall.centre_m,
                };
                if intersects(&sweep, &placement, &box_shape, &wall_pose) {
                    return Err(format!(
                        "route segment {segment} violates clearance at wall {}",
                        wall.index
                    )
                    .into());
                }
            }
        }
        Ok(())
    }

    fn visual_source(&self) -> Result<String> {
        let mut source = String::from("maze_view {}\n");
        for wall in &self.walls {
            source.push_str(&super::visual::node(
                &format!("wall_{}", wall.index),
                "Box",
                "obstacle",
                wall.centre_m,
                wall.full_size_m.map(|v| v * 0.5),
                [0.25, 0.35, 0.45],
                wall.yaw_rad,
            ));
        }
        source.push_str(&super::visual::node(
            "ground",
            "Box",
            "ground",
            [0., 0., -0.08],
            [10., 10., 0.08],
            [0.72, 0.75, 0.79],
            0.,
        ));
        for (i, pair) in self.route_points_m.windows(2).enumerate() {
            let (a, b) = (pair[0], pair[1]);
            source.push_str(&super::visual::node(
                &format!("route_{i}"),
                "Box",
                "path_witness",
                [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, 0.03],
                [(b[0] - a[0]).hypot(b[1] - a[1]) * 0.5, 0.04, 0.025],
                [1., 0.55, 0.03],
                (b[1] - a[1]).atan2(b[0] - a[0]),
            ));
        }
        for (name, i, color) in [
            ("start", self.start, [0.07, 0.7, 0.3]),
            ("goal", self.goal, [0.9, 0.1, 0.12]),
        ] {
            let p = self.cells[i].centre_m;
            source.push_str(&super::visual::node(
                name,
                "Sphere",
                name,
                [p[0], p[1], 0.22],
                [0.22, 0., 0.],
                color,
                0.,
            ));
        }
        let (_, emitted) = CompiledWorld::compile(&source)?.reemit("maze_view")?;
        Ok(emitted)
    }
}

pub(super) fn run(args: &[String]) -> Result<()> {
    if !(3..=4).contains(&args.len()) {
        return Err(
            "usage: adversarial_grid --generate-maze hex|polar SEED NEW_DIRECTORY [ACTOR.hymeko]"
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
    let (world, source, geometry, checks) = Topology::new(profile).generate(seed)?;
    let view = geometry.visual_source()?;
    fs::create_dir_all(output)?;
    fs::write(output.join("source.hymeko"), source)?;
    fs::write(output.join("view.hymeko"), view)?;
    fs::write(
        output.join("geometry.json"),
        serde_json::to_vec_pretty(&geometry)?,
    )?;
    let receipt = json!({"schema_version":1,"semantic_digest":world.digest(),"query_checks":checks,"cell_count":geometry.cells.len(),"passage_count":geometry.passages.len(),"wall_count":geometry.walls.len(),"route_cells":geometry.route_cell_indices,"route_points_m":geometry.route_points_m,"clearance_radius_m":CLEARANCE,"clearance_owner":"hymeko_geometry::collide::intersects; swept capsule against actual yaw boxes","scope":"seeded rank-2 spanning-tree maze; geometric route witness, not controller success or uniformly sampled trees"});
    fs::write(
        output.join("validation.json"),
        serde_json::to_vec_pretty(&receipt)?,
    )?;
    super::maze_physics::export(&geometry, output, args.get(3).map(Path::new))?;
    println!("{}", serde_json::to_string(&receipt)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_have_real_polar_and_six_neighbor_hex_topology() {
        let hex = Topology::new(Profile::Hex);
        assert_eq!(hex.cells.len(), 19);
        assert_eq!(Topology::hex(1).cells.len(), 7);
        let centre = hex
            .cells
            .iter()
            .find(|c| c.centre_m == [0., 0.])
            .unwrap()
            .index;
        assert_eq!(
            hex.boundaries
                .iter()
                .filter(|b| b.pair().is_some_and(|p| p.contains(&centre)))
                .count(),
            6
        );
        let polar = Topology::new(Profile::Polar);
        assert_eq!(polar.cells.len(), 12);
        assert!(polar.boundaries.iter().any(|b| b.pair() == Some([0, 6])));
        assert!(polar.boundaries.iter().any(|b| b.pair() == Some([0, 5])));
        assert!(polar.boundaries.iter().any(|b| b.points.len() == 4));
    }

    #[test]
    fn seeded_mazes_roundtrip_are_connected_and_change_passages() {
        for profile in [Profile::Hex, Profile::Polar] {
            let topology = Topology::new(profile);
            let (world, source, first, checks) = topology.generate(0).unwrap();
            let (repeat, again, _, _) = topology.generate(0).unwrap();
            let (_, _, second, _) = topology.generate(1).unwrap();
            assert_eq!(source, again);
            assert_eq!(world.digest(), repeat.digest());
            assert_ne!(first.passages, second.passages);
            assert_eq!(first.passages.len() + 1, first.cells.len());
            assert!(checks.iter().all(|c| c["passed"] == true));
            assert!(first.walls.len() <= 64 && first.route_points_m.len() <= 32);
            assert_eq!(first.route_cell_indices.first(), Some(&first.start));
            assert_eq!(first.route_cell_indices.last(), Some(&first.goal));
            let view = CompiledWorld::compile(&first.visual_source().unwrap()).unwrap();
            let query = QueryEngine::new(&view.program.ir, &view.store.it);
            let wall = query.query_first(&Predicate::named("wall_0")).unwrap().id;
            for (field, value) in [
                ("qw", (first.walls[0].yaw_rad * 0.5).cos()),
                ("qz", (first.walls[0].yaw_rad * 0.5).sin()),
            ] {
                assert!(
                    matches!(query.effective_value(wall,field), Some(ValueR::Num(n)) if (n-value).abs()<1e-12)
                );
            }
        }
    }

    #[test]
    fn canonical_queries_refuse_changed_geometry_and_illegal_passages() {
        let topology = Topology::new(Profile::Hex);
        let (_, source, _, _) = topology.generate(0).unwrap();
        for mutated in [
            source.replace("size_z 1.0;", "size_z 2.0;"),
            source.replacen("index 0.0;", "index 1.0;", 1),
            source.replacen("~ cell_0", "+ cell_0", 1),
            source.replace("wall_0: vocabulary.maze_wall", "wall_0"),
            format!("{source}\n@illegal {{ (~ cell_0, ~ maze); }}"),
        ] {
            assert_ne!(mutated, source);
            assert!(Topology::project(&CompiledWorld::compile(&mutated).unwrap()).is_err());
        }
        assert!(Profile::parse("square").is_err());
    }

    #[test]
    fn polar_arc_points_and_capsule_guard_prevent_corner_cutting() {
        let polar = Topology::new(Profile::Polar);
        let points = polar.route_points(&[6, 7]);
        assert_eq!(points.len(), 3);
        assert!((points[1][0].hypot(points[1][1]) - 7.1).abs() < 1e-10);
        let (_, _, mut geometry, _) = polar.generate(0).unwrap();
        geometry.route_points_m = vec![polar.cells[6].centre_m, polar.cells[7].centre_m];
        // The direct chord fails the inner-radius bound even when its sector portal is open.
        let open = BTreeSet::from([[6, 7]]);
        geometry.walls = polar.walls(&open);
        assert!(geometry.check_clearance(CLEARANCE).is_err());
        geometry.route_points_m = points;
        geometry.check_clearance(CLEARANCE).unwrap();
        assert!(geometry.check_clearance(f64::NAN).is_err());
    }

    #[test]
    fn every_profile_portal_has_the_declared_swept_clearance() {
        for profile in [Profile::Hex, Profile::Polar] {
            let topology = Topology::new(profile);
            let (_, _, mut geometry, _) = topology.generate(0).unwrap();
            for passage in topology.boundaries.iter().filter_map(Boundary::pair) {
                geometry.walls = topology.walls(&BTreeSet::from([passage]));
                geometry.route_points_m = topology.route_points(&passage);
                geometry.check_clearance(CLEARANCE).unwrap();
            }
        }
    }
}
