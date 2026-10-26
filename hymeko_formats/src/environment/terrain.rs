//! Robot-bound terrain over a validated maze route.
//!
//! The maze topology, route and swept wall clearance stay owned by `maze`; this
//! layer places at most `FEATURE_SLOTS` terrain features on route segments, with
//! every magnitude expressed relative to the robot's `TerrainBounds`. Intensities
//! above one exceed the declared capability, so the generator can also produce
//! worlds that the geometric level classifies as infeasible rather than invalid.
//!
//! Three verification levels stay separate: `topology` (Graph route, Query
//! counts), `geometry` (feature bounds, placement, clearance) and `dynamics`,
//! which only a physical rollout in the simulator can establish.
//!
//! Frames and units: world Z up, origin at the maze centre, the floor top at
//! z = 0; metres, seconds and radians in JSON. HyMeKo mounts keep degrees.
use super::locomotion::LeggedRobot;
use super::maze::{MazeGeometry, Profile, Scale, Topology};
use super::maze_physics::{Body, Composition, Motion, compose};
use super::team::{self, TEAM_ROBOTS, TEAM_TARGETS, TeamPlan};
use super::*;
use std::collections::BTreeMap;
use hymeko_graph::rand_lcg::Lcg;

pub const FEATURE_SLOTS: usize = 3;
pub const GENES_PER_FEATURE: usize = 3;
/// Gene 1 of a slot maps to [0, MAX_INTENSITY]; one is the robot's declared bound.
pub const MAX_INTENSITY: f64 = 1.25;
/// Longest declared mover period the simulator's terrain adapter admits (`configure_terrain`).
pub const MAX_MOVER_PERIOD_S: f64 = 600.0;
const SLAB: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FeatureKind {
    Ramp,
    Steps,
    Rough,
    Stones,
    Friction,
    Narrow,
    Hurdle,
    Moving,
}

impl FeatureKind {
    const ALL: [FeatureKind; 8] = [
        Self::Ramp,
        Self::Steps,
        Self::Rough,
        Self::Stones,
        Self::Friction,
        Self::Narrow,
        Self::Hurdle,
        Self::Moving,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::Ramp => "ramp",
            Self::Steps => "steps",
            Self::Rough => "rough",
            Self::Stones => "stones",
            Self::Friction => "friction",
            Self::Narrow => "narrow",
            Self::Hurdle => "hurdle",
            Self::Moving => "moving",
        }
    }

    fn parse(name: &str) -> Result<Self> {
        Self::ALL
            .into_iter()
            .find(|k| k.name() == name)
            .ok_or_else(|| format!("unknown terrain feature {name}").into())
    }

    /// Gene 0 selects nothing or one of the eight kinds with equal width.
    fn from_gene(gene: f64) -> Option<Self> {
        let index = (gene * 9.0).floor().min(8.0) as usize;
        index.checked_sub(1).map(|i| Self::ALL[i])
    }
}

/// A placed feature. `value` is compared with `bound` in the feature's own sense:
/// at most for heights/slopes/pitch, at least for friction, width and free window.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct Feature {
    pub slot: usize,
    pub kind: FeatureKind,
    pub segment: usize,
    pub centre_m: [f64; 2],
    pub yaw_rad: f64,
    pub length_m: f64,
    pub width_m: f64,
    pub intensity: f64,
    pub value: f64,
    pub bound: f64,
    pub unit: &'static str,
    pub at_least: bool,
    pub feasible: bool,
}

/// The robot task over the world: reach the goal cell, cover every cell of the maze, or let
/// two robots of the profile visit `TEAM_TARGETS` generated target cells together (`team`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TaskKind {
    Reach,
    Coverage,
    Team,
}

impl TaskKind {
    pub fn parse(name: &str) -> Result<Self> {
        match name {
            "reach" => Ok(Self::Reach),
            "coverage" => Ok(Self::Coverage),
            "team" => Ok(Self::Team),
            _ => Err("task must be reach, coverage or team".into()),
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Reach => "reach",
            Self::Coverage => "coverage",
            Self::Team => "team",
        }
    }
    /// Genes after the feature slots: the team's target choices.
    fn target_genes(self) -> usize {
        if self == Self::Team { TEAM_TARGETS } else { 0 }
    }
}

/// The two-robot task: target cells, the fixed plan (absent when no assignment and order has
/// a conflict-free schedule) and each robot's waypoints with the cell each one centres on.
#[derive(Clone, Debug, Serialize)]
pub struct TeamTask {
    pub starts: [usize; TEAM_ROBOTS],
    pub targets: Vec<usize>,
    pub plan: Option<TeamPlan>,
    pub waypoints: Vec<Vec<([f64; 2], Option<usize>)>>,
}

/// Depth-first walk over the open passages from the start cell that visits every cell;
/// backtracking steps are part of the walk. Neighbours are taken in ascending index order.
fn coverage_walk(cells: usize, passages: &[[usize; 2]]) -> Vec<usize> {
    let mut walk = vec![0];
    let mut visited = vec![false; cells];
    visited[0] = true;
    let mut stack = vec![0];
    while let Some(&cell) = stack.last() {
        if visited.iter().all(|v| *v) {
            break;
        }
        let next = passages
            .iter()
            .filter_map(|&[a, b]| if a == cell { Some(b) } else if b == cell { Some(a) } else { None })
            .filter(|n| !visited[*n])
            .min();
        match next {
            Some(n) => {
                visited[n] = true;
                stack.push(n);
                walk.push(n);
            }
            None => {
                stack.pop();
                if let Some(&back) = stack.last() {
                    walk.push(back);
                }
            }
        }
    }
    walk
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Category {
    Valid,
    InvalidWorld,
    InfeasibleTask,
}

/// Robot-scaled maze extents. Cells hold the robot's passage width and body length with room
/// for a feature; the slower biped gets fewer cells so a route fits its physical horizon.
pub fn scale_for(profile: Profile, robot: &LeggedRobot) -> Scale {
    let width = robot.bounds.min_passage_width_m;
    let biped = robot.profile.morphology == "biped";
    let (size, factor) = match profile {
        Profile::Square => (if biped { 2 } else { 3 }, 2.2),
        Profile::Hex => (1, 3.0),
        Profile::Polar => (if biped { 1 } else { 2 }, 2.2),
    };
    Scale {
        size,
        cell_m: (factor * width)
            .max(1.6 * robot.measured.body_length_m + 0.3)
            .clamp(0.6, 6.0),
        wall_height_m: (0.6 * robot.measured.body_height_m).clamp(0.3, 1.0),
        clearance_m: 0.5 * width,
    }
}

/// The team task needs two start cells and `TEAM_TARGETS` further cells: the biped's 2×2
/// square grows to 3×3; hex (7 cells) and polar (6 per ring) already hold them.
pub fn scale_for_task(profile: Profile, robot: &LeggedRobot, task: TaskKind) -> Scale {
    let mut scale = scale_for(profile, robot);
    if task == TaskKind::Team && profile == Profile::Square {
        scale.size = scale.size.max(3);
    }
    scale
}

/// Fraction of the horizon the route may need at the declared nominal speed.
pub const TEMPORAL_FRACTION: f64 = 0.8;

#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct TemporalCheck {
    pub route_m: f64,
    pub required_s: f64,
    pub available_s: f64,
    pub feasible: bool,
}

pub struct TerrainWorld {
    pub world: CompiledWorld,
    pub source: String,
    pub maze: MazeGeometry,
    pub features: Vec<Feature>,
    pub category: Category,
    pub refusal: Option<String>,
    pub horizon_s: f64,
    pub temporal: TemporalCheck,
    pub checks: Vec<Value>,
    pub priority_pairs: Vec<[usize; 2]>,
    pub task: TaskKind,
    pub route_points: Vec<[f64; 2]>,
    pub waypoint_cells: Vec<Option<usize>>,
    pub team: Option<TeamTask>,
    bodies: Vec<Body>,
    footholds: Vec<[f64; 3]>,
}

struct Placement {
    start: [f64; 2],
    direction: [f64; 2],
    length: f64,
}

impl Placement {
    fn at(&self, along: f64, across: f64) -> [f64; 2] {
        let [dx, dy] = self.direction;
        [
            self.start[0] + along * dx - across * dy,
            self.start[1] + along * dy + across * dx,
        ]
    }
    fn yaw(&self) -> f64 {
        self.direction[1].atan2(self.direction[0])
    }
}

fn unit_genes(seed: u64, count: usize) -> Vec<f64> {
    let mut rng = Lcg::new(seed ^ 0x7e44_a1d3_5c2b_9f01);
    (0..count)
        .map(|_| (rng.next_u64() >> 11) as f64 / (1_u64 << 53) as f64)
        .collect()
}

impl TerrainWorld {
    /// Number of genes for a world: maze passage priorities followed by feature slots.
    pub fn dimension(profile: Profile, robot: &LeggedRobot) -> Result<usize> {
        Self::dimension_for(profile, robot, TaskKind::Reach)
    }

    /// Genes of a task's world: priorities, feature slots, then the team's target choices.
    pub fn dimension_for(profile: Profile, robot: &LeggedRobot, task: TaskKind) -> Result<usize> {
        let scale = scale_for_task(profile, robot, task);
        Ok(Topology::scaled(profile, scale)?.priority_pairs().len()
            + FEATURE_SLOTS * GENES_PER_FEATURE
            + task.target_genes())
    }

    pub fn generate(
        profile: Profile,
        seed: u64,
        robot: &LeggedRobot,
        genes: Option<&[f64]>,
        horizon_s: Option<f64>,
    ) -> Result<Self> {
        Self::generate_task(profile, seed, robot, genes, horizon_s, TaskKind::Reach)
    }

    pub fn generate_task(
        profile: Profile,
        seed: u64,
        robot: &LeggedRobot,
        genes: Option<&[f64]>,
        horizon_s: Option<f64>,
        task: TaskKind,
    ) -> Result<Self> {
        let scale = scale_for_task(profile, robot, task);
        let topology = Topology::scaled(profile, scale)?;
        let pairs = topology.priority_pairs();
        let feature_genes = FEATURE_SLOTS * GENES_PER_FEATURE;
        let (maze_world, maze_source, maze, checks, slots, target_genes) = match genes {
            Some(genes) => {
                if genes.len() != pairs.len() + feature_genes + task.target_genes()
                    || genes.iter().any(|g| !g.is_finite() || !(0.0..=1.0).contains(g))
                {
                    return Err("terrain genes must be finite [0,1] priorities, slots and targets".into());
                }
                let (w, s, g, c) = topology.generate_priorities(seed, &genes[..pairs.len()])?;
                let tail = &genes[pairs.len()..];
                (w, s, g, c, tail[..feature_genes].to_vec(), tail[feature_genes..].to_vec())
            }
            None => {
                let (w, s, g, c) = topology.generate(seed)?;
                // Target genes use their own stream so the feature slots of a seed stay unchanged.
                let targets = unit_genes(seed ^ 0x51ed_27a0_c3b4_e981, task.target_genes());
                (w, s, g, c, unit_genes(seed, feature_genes), targets)
            }
        };
        // The task routes: the validated start-goal route, a full coverage walk, or the two
        // robots' routes of the fixed team plan.
        let team = match task {
            TaskKind::Team => Some(Self::team(&topology, &maze_world, &maze, &target_genes)?),
            _ => None,
        };
        let routes: Vec<Vec<([f64; 2], Option<usize>)>> = match (task, &team) {
            (TaskKind::Reach, _) => vec![maze.route_points_m().iter().map(|p| (*p, None)).collect()],
            (TaskKind::Coverage, _) => {
                vec![topology.route_waypoints(&coverage_walk(maze.cells().len(), maze.passages()))]
            }
            (TaskKind::Team, Some(TeamTask { plan: Some(_), waypoints, .. })) => waypoints.clone(),
            // Without a schedule the world keeps the start-goal route for its features.
            (TaskKind::Team, _) => vec![maze.route_points_m().iter().map(|p| (*p, None)).collect()],
        };
        let route_points: Vec<[f64; 2]> = routes[0].iter().map(|(p, _)| *p).collect();
        let waypoint_cells: Vec<Option<usize>> = routes[0].iter().map(|(_, c)| *c).collect();
        let segments: Vec<[[f64; 2]; 2]> = routes
            .iter()
            .flat_map(|r| r.windows(2).map(|w| [w[0].0, w[1].0]))
            .collect();
        let mut features = Vec::new();
        let mut bodies = Vec::new();
        let mut footholds = Vec::new();
        let refusal = Self::place(
            seed,
            &segments,
            robot,
            scale,
            &slots,
            &mut features,
            &mut bodies,
            &mut footholds,
        )
        .err()
        .map(|e| e.to_string());
        let length = |r: &[([f64; 2], Option<usize>)]| -> f64 {
            r.windows(2).map(|p| (p[1].0[0] - p[0].0[0]).hypot(p[1].0[1] - p[0].0[1])).sum()
        };
        // A team robot's planned waits count as moves of the mean move length.
        let route = match &team {
            Some(TeamTask { plan: Some(plan), waypoints, .. }) => (0..TEAM_ROBOTS)
                .map(|r| {
                    let moves = plan.robots[r].cells.len() - 1;
                    let waits = plan.robots[r].arrival.last().copied().unwrap_or(0) - moves;
                    let metres = length(&waypoints[r]);
                    metres + waits as f64 * metres / moves.max(1) as f64
                })
                .fold(0.0, f64::max),
            _ => length(&routes[0]),
        };
        let horizon_s = horizon_s.unwrap_or_else(|| {
            (2.5 * route / robot.bounds.nominal_speed_m_s + 20.0).min(180.0)
        });
        if !horizon_s.is_finite() || !(5.0..=600.0).contains(&horizon_s) {
            return Err("terrain horizon must be in [5,600] s".into());
        }
        // Temporal level: at the declared speed the route must fit in 80 % of the horizon,
        // leaving the rest for turns and the goal dwell.
        let required_s = route / robot.bounds.nominal_speed_m_s;
        let temporal = TemporalCheck {
            route_m: route,
            required_s,
            available_s: TEMPORAL_FRACTION * horizon_s,
            feasible: required_s <= TEMPORAL_FRACTION * horizon_s,
        };
        let unscheduled = team.as_ref().is_some_and(|t| t.plan.is_none());
        let category = if refusal.is_some() {
            Category::InvalidWorld
        } else if features.iter().any(|f| !f.feasible) || !temporal.feasible || unscheduled {
            Category::InfeasibleTask
        } else {
            Category::Valid
        };
        let mut source = Self::source(
            &maze_source,
            robot,
            &features,
            category,
            horizon_s,
            maze.cells().len(),
            task,
        )?;
        if let Some(team) = &team {
            Self::team_source(&mut source, team)?;
        }
        let (world, source) = CompiledWorld::compile(&source)?.reemit("generated_maze")?;
        let mut checks = checks;
        checks.extend(Self::query_checks(&world, features.len())?);
        if let Some(team) = &team {
            checks.extend(Self::team_checks(&world, team, maze.passages().len())?);
        }
        Ok(Self {
            world,
            source,
            maze,
            features,
            category,
            refusal,
            horizon_s,
            temporal,
            checks,
            priority_pairs: pairs,
            task,
            route_points,
            waypoint_cells,
            team,
            bodies,
            footholds,
        })
    }

    /// Targets from their genes and the fixed plan over Graph's cell routes.
    fn team(topology: &Topology, maze_world: &CompiledWorld, maze: &MazeGeometry, genes: &[f64]) -> Result<TeamTask> {
        let cells = maze.cells().len();
        let targets = team::select_targets(cells, genes)?;
        let ends: Vec<usize> = team::starts(cells).into_iter().chain(targets.iter().copied()).collect();
        let pairs: Vec<[usize; 2]> = ends.iter().flat_map(|&a| ends.iter().map(move |&b| [a, b])).collect();
        let routes: BTreeMap<[usize; 2], Vec<usize>> =
            pairs.iter().copied().zip(Topology::cell_routes(maze_world, &pairs)?).collect();
        let plan = team::plan(cells, &targets, |a, b| routes[&[a, b]].clone());
        if let Some(plan) = &plan {
            team::verify(plan)?;
        }
        let waypoints = plan
            .as_ref()
            .map(|p| p.robots.iter().map(|r| topology.route_waypoints(&r.cells)).collect())
            .unwrap_or_default();
        Ok(TeamTask { starts: team::starts(cells), targets, plan, waypoints })
    }

    #[allow(clippy::too_many_arguments)]
    fn place(
        seed: u64,
        points: &[[[f64; 2]; 2]],
        robot: &LeggedRobot,
        scale: Scale,
        slots: &[f64],
        features: &mut Vec<Feature>,
        bodies: &mut Vec<Body>,
        footholds: &mut Vec<[f64; 3]>,
    ) -> Result<()> {
        let bounds = &robot.bounds;
        let body = &robot.measured;
        // Keep start and turning areas around route points free: the robot's footprint radius
        // (half the body length, or the farthest foot contact plus half a foot) plus a margin.
        // A biped's 0.3 m feet reach beyond half its trunk depth; the quadruped's paws do not.
        let feet = body
            .foot_contact_m
            .iter()
            .map(|p| p[0].hypot(p[1]) + 0.5 * body.foot_length_m)
            .fold(0.0, f64::max);
        let margin = (0.5 * body.body_length_m).max(feet) + 0.05;
        let segments: Vec<Placement> = points
            .iter()
            .map(|p| {
                let (dx, dy) = (p[1][0] - p[0][0], p[1][1] - p[0][1]);
                let length = dx.hypot(dy);
                Placement {
                    start: p[0],
                    direction: [dx / length, dy / length],
                    length,
                }
            })
            .collect();
        let width = 2.4 * scale.clearance_m;
        let mut used = Vec::new();
        for slot in 0..FEATURE_SLOTS {
            let genes = &slots[slot * GENES_PER_FEATURE..(slot + 1) * GENES_PER_FEATURE];
            let Some(kind) = FeatureKind::from_gene(genes[0]) else {
                continue;
            };
            let intensity = genes[1] * MAX_INTENSITY;
            // A taken segment passes the slot on to the next free one, cyclically; with every
            // segment taken the slot stays unplaced (its genes are then neutral).
            let first = ((genes[2] * segments.len() as f64).floor() as usize).min(segments.len() - 1);
            // A corridor walked twice (coverage backtracking) is one placement site.
            let site = |s: usize| {
                let [a, b] = points[s];
                let key = |p: [f64; 2]| [(p[0] * 1e6).round() as i64, (p[1] * 1e6).round() as i64];
                let (ka, kb) = (key(a), key(b));
                if ka <= kb { (ka, kb) } else { (kb, ka) }
            };
            let Some(segment) = (0..segments.len())
                .map(|k| (first + k) % segments.len())
                .find(|s| !used.contains(&site(*s)))
            else {
                continue;
            };
            used.push(site(segment));
            let seg = &segments[segment];
            let available = seg.length - 2.0 * margin;
            let mid = 0.5 * seg.length;
            let yaw = seg.yaw();
            let at = |name: String, along: f64, across: f64, z: f64, size: [f64; 3], pitch: f64, color: [f64; 4]| {
                let [x, y] = seg.at(along, across);
                Body {
                    pitch,
                    surface: Some("ground"),
                    ..Body::fixed(name, [x, y, z], size, yaw, color)
                }
            };
            let mut emit = |body: Body| bodies.push(body);
            let tag = format!("terrain_{slot}");
            let (length, value, bound, unit, at_least) = match kind {
                FeatureKind::Ramp => {
                    let slope = intensity * bounds.max_slope_rad;
                    let plateau = bounds.foothold_length_m.max(0.1);
                    let run = ((available - plateau) * 0.5).min(1.2);
                    if run < bounds.foothold_length_m.max(0.12) {
                        return Err(format!("segment {segment} too short for a ramp").into());
                    }
                    let height = run * slope.tan();
                    let color = [0.62, 0.52, 0.38, 1.];
                    if height > 1e-4 {
                        emit(at(format!("{tag}_plateau"), mid, 0., height * 0.5, [plateau, width, height], 0., color));
                        let slant = run / slope.cos();
                        for (side, sign) in [("up", -1.0), ("down", 1.0)] {
                            // Top surface spans floor to plateau; the slab hangs below it.
                            let along = mid + sign * (0.5 * plateau + 0.5 * run)
                                + sign * 0.5 * SLAB * slope.sin();
                            let z = 0.5 * height - 0.5 * SLAB * slope.cos();
                            emit(at(format!("{tag}_{side}"), along, 0., z, [slant, width, SLAB], -sign * slope, color));
                        }
                    }
                    (plateau + 2.0 * run, slope, bounds.max_slope_rad, "rad", false)
                }
                FeatureKind::Steps => {
                    let rise = intensity * bounds.max_step_up_m;
                    let tread = (2.5 * bounds.foothold_length_m).max(0.4).min(available);
                    if tread < 2.0 * bounds.foothold_length_m {
                        return Err(format!("segment {segment} too short for a step").into());
                    }
                    if rise > 1e-4 {
                        emit(at(format!("{tag}_step"), mid, 0., rise * 0.5, [tread, width, rise], 0., [0.55, 0.55, 0.6, 1.]));
                    }
                    (tread, rise, bounds.max_step_up_m, "m", false)
                }
                FeatureKind::Rough => {
                    let tile = bounds.foothold_length_m.max(0.06);
                    let along_tiles = ((available.min(1.2)) / tile).floor().max(1.0) as usize;
                    let across_tiles = (width / tile).floor().max(1.0) as usize;
                    let mut rng = Lcg::new(seed ^ (0x9e37_79b9 + slot as u64));
                    let mut highest: f64 = 0.0;
                    for i in 0..along_tiles {
                        for j in 0..across_tiles {
                            let u = (rng.next_u64() >> 11) as f64 / (1_u64 << 53) as f64;
                            let h = (u * intensity * bounds.roughness_m).max(0.002);
                            highest = highest.max(h);
                            let along = mid + (i as f64 - (along_tiles as f64 - 1.0) * 0.5) * tile;
                            let across = (j as f64 - (across_tiles as f64 - 1.0) * 0.5) * tile;
                            emit(at(format!("{tag}_tile_{i}_{j}"), along, across, h * 0.5, [tile, tile, h], 0., [0.5, 0.45, 0.4, 1.]));
                        }
                    }
                    (along_tiles as f64 * tile, highest, bounds.roughness_m, "m", false)
                }
                FeatureKind::Stones => {
                    let rise = intensity * bounds.max_step_up_m;
                    let pitch = robot.profile.step_length_m.max(bounds.foothold_length_m);
                    let count = ((available / pitch).floor() as usize).min(6);
                    if count < 2 {
                        return Err(format!("segment {segment} too short for footholds").into());
                    }
                    let lateral = if robot.profile.morphology == "biped" {
                        body.foot_contact_m.iter().map(|p| p[1].abs()).fold(0.0, f64::max)
                    } else {
                        0.0
                    };
                    for k in 0..count {
                        let along = mid + (k as f64 - (count as f64 - 1.0) * 0.5) * pitch;
                        let across = if lateral > 0.0 {
                            if k % 2 == 0 { lateral } else { -lateral }
                        } else {
                            0.0
                        };
                        let pad = [bounds.foothold_length_m, if lateral > 0.0 { bounds.foothold_width_m } else { width }, rise.max(0.004)];
                        emit(at(format!("{tag}_pad_{k}"), along, across, pad[2] * 0.5, pad, 0., [0.35, 0.5, 0.35, 1.]));
                        let [x, y] = seg.at(along, across);
                        footholds.push([x, y, pad[2]]);
                    }
                    (count as f64 * pitch, rise, bounds.max_step_up_m, "m", false)
                }
                FeatureKind::Friction => {
                    let mu = (1.0 - intensity * (1.0 - bounds.min_friction)).max(0.02);
                    let length = available.min(1.5);
                    emit(Body {
                        friction: Some(mu),
                        ..at(format!("{tag}_patch"), mid, 0., 0.002, [length, width, 0.004], 0., [0.75, 0.85, 0.95, 1.])
                    });
                    (length, mu, bounds.min_friction, "friction coefficient", true)
                }
                FeatureKind::Narrow => {
                    let gap = bounds.min_passage_width_m * (1.5 - 0.5 * intensity);
                    let post = 0.08;
                    for (side, sign) in [("left", 1.0), ("right", -1.0)] {
                        emit(Body {
                            surface: Some("obstacle"),
                            ..at(format!("{tag}_{side}"), mid, sign * (0.5 * gap + 0.5 * post), 0.5 * scale.wall_height_m, [post, post, scale.wall_height_m], 0., [0.4, 0.3, 0.3, 1.])
                        });
                    }
                    (post, gap, bounds.min_passage_width_m, "m", true)
                }
                FeatureKind::Hurdle => {
                    let height = intensity * bounds.hurdle_height_m;
                    if height > 1e-4 {
                        emit(Body {
                            surface: Some("hurdle"),
                            ..at(format!("{tag}_bar"), mid, 0., 0.5 * height, [0.04, width, height], 0., [0.85, 0.6, 0.1, 1.])
                        });
                    }
                    (0.04, height, bounds.hurdle_height_m, "m", false)
                }
                FeatureKind::Moving => {
                    let size = (0.5 * bounds.min_passage_width_m).max(0.1);
                    let amplitude = 0.5 * width + 0.5 * size;
                    let blocked = 0.5 * size + 0.5 * body.body_width_m + 0.05;
                    let crossing = (size + body.body_length_m + 0.1) / bounds.nominal_speed_m_s;
                    let required = 1.2 * crossing;
                    let ratio = (blocked / amplitude).clamp(0.0, 1.0);
                    let free_fraction = 0.5 - ratio.asin() / std::f64::consts::PI;
                    if free_fraction <= 0.0 {
                        return Err("moving obstacle leaves no free window".into());
                    }
                    let base = required / free_fraction;
                    let period = base / intensity.max(0.1);
                    // The physics adapter admits declared motions up to 600 s; a slow robot and a
                    // low intensity can ask for more, which no simulator run could represent.
                    if period > MAX_MOVER_PERIOD_S {
                        return Err(format!("moving obstacle period {period:.0} s exceeds {MAX_MOVER_PERIOD_S} s").into());
                    }
                    let window = period * free_fraction;
                    let phase = (seed % 628) as f64 * 0.01;
                    let height = (0.5 * body.body_height_m).clamp(0.15, 0.8);
                    emit(Body {
                        surface: Some("mover"),
                        motion: Some(Motion {
                            axis: [-seg.direction[1], seg.direction[0]],
                            amplitude_m: amplitude,
                            period_s: period,
                            phase_rad: phase,
                        }),
                        ..at(format!("{tag}_mover"), mid, 0., 0.5 * height + 0.005, [size, size, height], 0., [0.85, 0.15, 0.15, 1.])
                    });
                    (size, window, required, "s", true)
                }
            };
            let feasible = if at_least { value >= bound } else { value <= bound };
            features.push(Feature {
                slot,
                kind,
                segment,
                centre_m: seg.at(mid, 0.),
                yaw_rad: yaw,
                length_m: length,
                width_m: width,
                intensity,
                value,
                bound,
                unit,
                at_least,
                feasible,
            });
        }
        Ok(())
    }

    fn source(
        maze_source: &str,
        robot: &LeggedRobot,
        features: &[Feature],
        category: Category,
        horizon_s: f64,
        cells: usize,
        task: TaskKind,
    ) -> Result<String> {
        let mut source = maze_source.to_owned();
        let category = match category {
            Category::Valid => "valid",
            Category::InvalidWorld => "invalid_world",
            Category::InfeasibleTask => "infeasible_task",
        };
        writeln!(
            source,
            "terrain_vocabulary {{ terrain_feature {{}} }}\nterrain {{ robot \"{}\"; morphology \"{}\"; controller \"{}\"; robot_digest \"{}\"; profile_digest \"{}\"; category \"{category}\"; feature_count {}; }}\ntask {{ kind \"{}\"; start_index 0; goal_index {}; horizon_s {horizon_s}; goal_tolerance_m {}; dwell_s 2; }}",
            robot.profile.robot_root,
            robot.profile.morphology,
            robot.profile.controller,
            robot.robot_digest,
            robot.profile_digest,
            features.len(),
            task.name(),
            cells - 1,
            (0.5 * robot.measured.body_length_m).max(0.15),
        )?;
        for f in features {
            writeln!(
                source,
                "feature_{}: terrain_vocabulary.terrain_feature {{ slot {}; kind \"{}\"; segment {}; x {}; y {}; yaw {}; length {}; width {}; intensity {}; value {}; bound {}; at_least {}; feasible {}; }}",
                f.slot,
                f.slot,
                f.kind.name(),
                f.segment,
                f.centre_m[0],
                f.centre_m[1],
                f.yaw_rad,
                f.length_m,
                f.width_m,
                f.intensity,
                f.value,
                f.bound,
                u8::from(f.at_least),
                u8::from(f.feasible),
            )?;
        }
        Ok(source)
    }

    /// Team records: the plan summary, one node per robot and target, and one hyperedge per
    /// assignment (robot, target). The routes and gates stay in the receipt and the task.
    fn team_source(source: &mut String, team: &TeamTask) -> Result<()> {
        let (feasible, priority, makespan, moves, waits, gates) = match &team.plan {
            Some(p) => (1, p.priority, p.makespan_ticks, p.moves, p.waits,
                        p.robots.iter().map(|r| r.gates.len()).sum::<usize>()),
            None => (0, 0, 0, 0, 0, 0),
        };
        writeln!(
            source,
            "team_vocabulary {{ team_robot {{}} team_target {{}} }}\nteam {{ robots {TEAM_ROBOTS}; targets {}; planner \"prioritized-cell-reservation\"; scheduled {feasible}; priority {priority}; makespan_ticks {makespan}; moves {moves}; waits {waits}; gates {gates}; }}",
            team.targets.len(),
        )?;
        for (r, start) in team.starts.iter().enumerate() {
            let plan = team.plan.as_ref().map(|p| &p.robots[r]);
            writeln!(
                source,
                "team_robot_{r}: team_vocabulary.team_robot {{ index {r}; start_cell {start}; route_cells {}; finish_tick {}; }}",
                plan.map_or(0, |p| p.cells.len()),
                plan.map_or(0, |p| p.arrival.last().copied().unwrap_or(0)),
            )?;
        }
        // Assigned targets carry their robot and visiting order; the assignment is a hyperedge.
        let assigned: BTreeMap<usize, (usize, usize)> = team
            .plan
            .iter()
            .flat_map(|p| p.robots.iter().enumerate())
            .flat_map(|(r, robot)| robot.order.iter().enumerate().map(move |(o, &k)| (k, (r, o))))
            .collect();
        for (k, cell) in team.targets.iter().enumerate() {
            let extra = assigned.get(&k).map_or(String::new(), |(r, o)| format!(" robot {r}; order {o};"));
            writeln!(source, "team_target_{k}: team_vocabulary.team_target {{ index {k}; cell {cell};{extra} }}")?;
        }
        for (k, (r, _)) in &assigned {
            writeln!(source, "@assign_{k} {{ (~ team_robot_{r}, ~ team_target_{k}); }}")?;
        }
        Ok(())
    }

    fn team_checks(world: &CompiledWorld, team: &TeamTask, passages: usize) -> Result<Vec<Value>> {
        let query = QueryEngine::new(&world.model.program().ir, world.model.interner());
        let assigned = if team.plan.is_some() { team.targets.len() } else { 0 };
        let mut checks = Vec::new();
        for (label, predicate, count) in [
            ("one_team", Predicate::node().and(Predicate::named("team")), 1),
            ("team_robots", Predicate::node().and(Predicate::inherits("team_robot")), TEAM_ROBOTS),
            ("team_targets", Predicate::node().and(Predicate::inherits("team_target")), team.targets.len()),
            ("passages_and_assignments", Predicate::edge(), passages + assigned),
        ] {
            let check = query.check_query(&NamedQuery { label: label.into(), predicate }, MatchExpectation::Exactly(count), 8);
            if !check.passed {
                return Err(format!("team query check {label} failed").into());
            }
            checks.extend(check_receipts(&[check]));
        }
        Ok(checks)
    }

    fn query_checks(world: &CompiledWorld, features: usize) -> Result<Vec<Value>> {
        let query = QueryEngine::new(&world.model.program().ir, world.model.interner());
        let mut checks = Vec::new();
        for (label, predicate, count) in [
            ("one_terrain", Predicate::node().and(Predicate::named("terrain")), 1),
            ("one_task", Predicate::node().and(Predicate::named("task")), 1),
            (
                "terrain_features",
                Predicate::node().and(Predicate::inherits("terrain_feature")),
                features,
            ),
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
                return Err(format!("terrain query check {label} failed").into());
            }
            checks.extend(check_receipts(&[check]));
        }
        Ok(checks)
    }

    /// Read the feature records back from the canonical source (round-trip witness).
    pub fn read_features(world: &CompiledWorld) -> Result<Vec<(usize, FeatureKind, usize, f64, bool)>> {
        let query = QueryEngine::new(&world.model.program().ir, world.model.interner());
        let mut rows = query
            .query(&Predicate::node().and(Predicate::inherits("terrain_feature")))
            .into_iter()
            .map(|m| -> Result<_> {
                let kind = match query.effective_value(m.id, "kind") {
                    Some(ValueR::Str(s)) => FeatureKind::parse(world.model.interner().resolve(*s))?,
                    _ => return Err("feature without kind".into()),
                };
                Ok((
                    world.scene_scalar(m.id, "slot")? as usize,
                    kind,
                    world.scene_scalar(m.id, "segment")? as usize,
                    world.scene_scalar(m.id, "value")?,
                    world.scene_scalar(m.id, "feasible")? == 1.0,
                ))
            })
            .collect::<Result<Vec<_>>>()?;
        rows.sort_by_key(|r| r.0);
        Ok(rows)
    }

    /// Validation receipt: the three verification levels and every feature against its bound.
    pub fn receipt(&self) -> Value {
        json!({
            "schema": "hymeko-terrain-validation",
            "semantic_digest": self.world.digest(),
            "category": self.category,
            "refusal": self.refusal,
            "levels": {
                "topology": {"passed": true, "route_cells": self.maze.route_cell_indices(),
                    "owner": "Graph spanning basis and Berge BFS/Dijkstra; Query counts"},
                "temporal": self.temporal,
                "geometry": {"passed": self.refusal.is_none() && self.features.iter().all(|f| f.feasible),
                    "clearance_radius_m": self.maze.clearance_radius_m(),
                    "features": self.features,
                    "owner": "robot-bound feature limits; swept capsule wall clearance in maze"},
                "dynamics": {"passed": null, "owner": "simulator physical rollout"},
            },
            "query_checks": self.checks,
            "route_points_m": self.route_points,
            "task": self.task,
            "team": self.team,
            "priority_pairs": self.priority_pairs,
            "horizon_s": self.horizon_s,
            "feature_slots": FEATURE_SLOTS,
            "genes_per_feature": GENES_PER_FEATURE,
            "max_intensity": MAX_INTENSITY,
            "frames": "world Z up, floor top z=0, metres/seconds/radians",
        })
    }

    /// Owner-emitted physical model, scene and manifest; skipped for invalid worlds.
    pub fn export(&self, robot: &LeggedRobot, output: &Path) -> Result<()> {
        let route = &self.route_points;
        let mut task = task(robot, self.task.name(), route, self.horizon_s);
        if self.task == TaskKind::Coverage {
            task["waypoint_cells"] = json!(self.waypoint_cells);
            task["cells_total"] = json!(self.maze.cells().len());
        }
        if let Some(TeamTask { plan: Some(plan), waypoints, targets, .. }) = &self.team {
            task = self.team_task(robot, plan, waypoints, targets);
        }
        export_scene(self.maze.walls(), &self.bodies, self.horizon_s, robot, output, json!({
            "profile": self.maze.profile,
            "seed": self.maze.seed,
            "world_digest": self.world.digest(),
            "route_points_m": route,
            "footholds_m": self.footholds,
            "features": self.features,
            "category": self.category,
            "task": task,
        }))
    }
}

impl TerrainWorld {
    /// The team block the simulator executes: one single-robot task per member (start, route,
    /// tolerance, dwell, horizon) plus the cell each waypoint centres on, the route index of every
    /// route cell's waypoint, the assigned targets and the precedence gates against the other robot.
    fn team_task(&self, robot: &LeggedRobot, plan: &TeamPlan, waypoints: &[Vec<([f64; 2], Option<usize>)>],
                 targets: &[usize]) -> Value {
        let members: Vec<Value> = plan
            .robots
            .iter()
            .zip(waypoints)
            .enumerate()
            .map(|(r, (member, points))| {
                let route: Vec<[f64; 2]> = points.iter().map(|(p, _)| *p).collect();
                // Waypoint index of route cell k: cell waypoints in order, arc midpoints skipped.
                let cell_waypoints: Vec<usize> =
                    points.iter().enumerate().filter(|(_, (_, c))| c.is_some()).map(|(w, _)| w).collect();
                let mut value = task(robot, "team_member", &route, self.horizon_s);
                value["index"] = json!(r);
                value["waypoint_cells"] = json!(points.iter().map(|(_, c)| *c).collect::<Vec<_>>());
                value["cells"] = json!(member.cells);
                value["cell_waypoints"] = json!(cell_waypoints);
                value["targets"] = json!(member.order.iter().zip(&member.target_index)
                    .map(|(&t, &k)| json!({"target": t, "cell": targets[t], "route_index": k}))
                    .collect::<Vec<_>>());
                value["gates"] = json!(member.gates.iter()
                    .map(|g| json!({"enter": g.enter, "other": 1 - r, "reached": g.reached}))
                    .collect::<Vec<_>>());
                value["planned_arrival_ticks"] = json!(member.arrival);
                value
            })
            .collect();
        json!({
            "kind": "team",
            "robots": members,
            "targets": targets,
            "planner": "prioritized-cell-reservation",
            "priority": plan.priority,
            "makespan_ticks": plan.makespan_ticks,
            "goal_tolerance_m": (0.5 * robot.measured.body_length_m).max(0.15),
            "dwell_s": 2.0,
            "horizon_s": self.horizon_s,
            "route_points_m": self.route_points,
        })
    }
}

/// A flat floor with no maze: the physical pretest scene for one robot.
pub fn export_flat(robot: &LeggedRobot, horizon_s: f64, output: &Path) -> Result<()> {
    let route = [[0.0, 0.0], [1.0, 0.0]];
    export_scene(&[], &[], horizon_s, robot, output, json!({
        "profile": "flat", "seed": 0, "route_points_m": route, "footholds_m": [], "features": [],
        "category": Category::Valid,
        "task": task(robot, "pretest", &route, horizon_s),
    }))
}

/// The task block the simulator executes: start pose on the first route point facing the
/// second, the route, goal tolerance, dwell and horizon (same values as the source record).
fn task(robot: &LeggedRobot, kind: &str, route: &[[f64; 2]], horizon_s: f64) -> Value {
    let yaw = match route {
        [a, b, ..] => (b[1] - a[1]).atan2(b[0] - a[0]),
        _ => 0.0,
    };
    json!({
        "kind": kind,
        "start": {"position_m": [route[0][0], route[0][1], robot.profile.stance_height_m], "yaw_rad": yaw},
        "route_points_m": route,
        "goal_tolerance_m": (0.5 * robot.measured.body_length_m).max(0.15),
        "dwell_s": 2.0,
        "horizon_s": horizon_s,
    })
}

fn export_scene(
    walls: &[super::maze::MazeWall],
    terrain: &[Body],
    horizon_s: f64,
    robot: &LeggedRobot,
    output: &Path,
    extra: Value,
) -> Result<()> {
    for name in ["model.hymeko", "scene.xml", "physics.json", "terrain.json"] {
        if output.join(name).exists() {
            return Err(format!("refusing to replace {name}").into());
        }
    }
    let mut bodies: Vec<Body> = walls
        .iter()
        .map(|w| Body {
            surface: Some("wall"),
            ..Body::wall(w)
        })
        .collect();
    let extent = walls
        .iter()
        .flat_map(|w| w.centre_m[..2].iter())
        .map(|v| v.abs())
        .fold(2.0, f64::max)
        + 2.0;
    bodies.push(Body {
        surface: Some("ground"),
        ..Body::fixed(
            "floor".into(),
            [0., 0., -0.05],
            [2. * extent, 2. * extent, 0.1],
            0.,
            [0.72, 0.72, 0.66, 1.],
        )
    });
    bodies.extend(terrain.iter().cloned());
    let record = format!(
        "terrain_physics {{ timestep_s 0.002; horizon_s {horizon_s}; sample_every 25; contact_time_constant_s 0.008; contact_damping_ratio 1; sliding_friction 0.9; world: meta_kinematics.kinematics.elements.link {{}}"
    );
    let Composition {
        digest,
        objects,
        numbers,
    } = compose(
        output,
        "terrain_physics_description",
        &record,
        "terrain_physics",
        &bodies,
        &[
            "timestep_s",
            "horizon_s",
            "sample_every",
            "contact_time_constant_s",
            "contact_damping_ratio",
            "sliding_friction",
        ],
    )?;
    let mut manifest = json!({
        "schema": "hymeko-terrain-physics",
        "canonical_digest": digest,
        "objects": objects,
        "mjcf": "scene.xml",
        "source": "model.hymeko",
        "backend": "mujoco",
        "mobility_boundary": "dynamics_role 2 selects a MuJoCo mocap body driven by its declared motion",
        "robot_digest": robot.robot_digest,
        "profile_digest": robot.profile_digest,
    });
    for (key, value) in numbers {
        manifest[key] = json!(value);
    }
    fs::write(output.join("physics.json"), serde_json::to_vec_pretty(&manifest)?)?;
    fs::write(output.join("terrain.json"), serde_json::to_vec_pretty(&extra)?)?;
    robot.write(output)
}

#[cfg(test)]
mod tests {
    use super::super::locomotion::TerrainBounds;
    use super::*;

    fn bounds() -> TerrainBounds {
        TerrainBounds {
            max_step_up_m: 0.04,
            max_slope_rad: 0.1,
            min_friction: 0.5,
            hurdle_height_m: 0.04,
            foothold_length_m: 0.3,
            foothold_width_m: 0.14,
            max_gap_m: 0.0,
            min_passage_width_m: 0.8,
            roughness_m: 0.016,
            nominal_speed_m_s: 0.1,
        }
    }

    #[test]
    fn gene_kind_mapping_covers_none_and_all_kinds() {
        assert_eq!(FeatureKind::from_gene(0.0), None);
        assert_eq!(FeatureKind::from_gene(0.99), Some(FeatureKind::Moving));
        assert_eq!(FeatureKind::from_gene(1.0), Some(FeatureKind::Moving));
        let seen: std::collections::BTreeSet<_> = (0..90)
            .filter_map(|i| FeatureKind::from_gene(i as f64 / 90.0))
            .map(|k| k.name())
            .collect();
        assert_eq!(seen.len(), 8);
        for kind in FeatureKind::ALL {
            assert_eq!(FeatureKind::parse(kind.name()).unwrap(), kind);
        }
    }

    #[test]
    fn robot_scale_admits_every_profile_and_keeps_clearance_below_half_a_cell() {
        let b = bounds();
        for profile in [Profile::Square, Profile::Hex, Profile::Polar] {
            let scale = Scale {
                size: if profile == Profile::Square { 2 } else { 1 },
                cell_m: (2.2 * b.min_passage_width_m).max(1.6 * 0.355 + 0.3),
                wall_height_m: 1.0,
                clearance_m: 0.5 * b.min_passage_width_m,
            };
            let topology = Topology::scaled(profile, scale).unwrap();
            let (world, source, geometry, checks) = topology.generate(3).unwrap();
            assert!(source.contains("maze_scale"));
            assert!(checks.iter().all(|c| c["passed"] == true));
            assert_eq!(geometry.clearance_radius_m(), scale.clearance_m);
            let (again, _, _, _) = topology.generate(3).unwrap();
            assert_eq!(world.digest(), again.digest());
        }
    }

    fn genes(robot: &LeggedRobot, profile: Profile, slots: [[f64; 3]; FEATURE_SLOTS]) -> Vec<f64> {
        let n = TerrainWorld::dimension(profile, robot).unwrap() - FEATURE_SLOTS * GENES_PER_FEATURE;
        let mut g = vec![0.5; n];
        g.extend(slots.iter().flatten());
        g
    }

    /// Kind gene for `FeatureKind::ALL[index]`, centred in its band.
    fn kind(index: usize) -> f64 {
        (index as f64 + 1.5) / 9.0
    }

    #[test]
    fn worlds_are_reproducible_and_seed_dependent() {
        let robot = LeggedRobot::fixture("quadruped");
        let a = TerrainWorld::generate(Profile::Square, 3, &robot, None, Some(90.0)).unwrap();
        let b = TerrainWorld::generate(Profile::Square, 3, &robot, None, Some(90.0)).unwrap();
        let c = TerrainWorld::generate(Profile::Square, 4, &robot, None, Some(90.0)).unwrap();
        assert_eq!(a.source, b.source);
        assert_eq!(a.world.digest(), b.world.digest());
        assert_ne!(a.world.digest(), c.world.digest());
        assert!(a.checks.iter().all(|c| c["passed"] == true));
    }

    #[test]
    fn every_kind_is_placed_within_bounds_and_beyond_them_is_infeasible() {
        for morphology in ["quadruped", "biped"] {
            let robot = LeggedRobot::fixture(morphology);
            let profile = Profile::Square;
            for (index, k) in FeatureKind::ALL.iter().enumerate() {
                let within = genes(&robot, profile, [[kind(index), 0.7, 0.0], [0.0; 3], [0.0; 3]]);
                let world = TerrainWorld::generate(profile, 0, &robot, Some(&within), Some(200.0)).unwrap();
                if world.category == Category::InvalidWorld {
                    // A feature that cannot fit is an invalid world, never silently dropped.
                    assert!(world.refusal.as_deref().unwrap().contains("too short"), "{morphology} {k:?}");
                    continue;
                }
                assert_eq!(world.features.len(), 1, "{morphology} {k:?}");
                assert_eq!(world.features[0].kind, *k);
                assert!(world.features[0].feasible, "{morphology} {k:?} {:?}", world.features[0]);
                assert_eq!(world.category, Category::Valid);
                let beyond = genes(&robot, profile, [[kind(index), 1.0, 0.0], [0.0; 3], [0.0; 3]]);
                let world = TerrainWorld::generate(profile, 0, &robot, Some(&beyond), Some(200.0)).unwrap();
                assert_eq!(world.category, Category::InfeasibleTask, "{morphology} {k:?}");
                assert!(!world.features[0].feasible);
            }
        }
    }

    #[test]
    fn feature_records_round_trip_through_the_canonical_source() {
        let robot = LeggedRobot::fixture("quadruped");
        let g = genes(&robot, Profile::Square, [[kind(1), 0.5, 0.1], [kind(6), 0.6, 0.5], [kind(7), 0.4, 0.9]]);
        let world = TerrainWorld::generate(Profile::Square, 1, &robot, Some(&g), Some(90.0)).unwrap();
        let read = TerrainWorld::read_features(&world.world).unwrap();
        assert_eq!(read.len(), world.features.len());
        for (row, f) in read.iter().zip(&world.features) {
            assert_eq!((row.0, row.1, row.2, row.4), (f.slot, f.kind, f.segment, f.feasible));
            assert!((row.3 - f.value).abs() < 1e-9);
        }
        let reparsed = CompiledWorld::compile(&world.source).unwrap();
        assert_eq!(reparsed.digest(), world.world.digest());
    }

    #[test]
    fn a_route_longer_than_the_horizon_allows_is_infeasible() {
        let robot = LeggedRobot::fixture("biped");
        let world = TerrainWorld::generate(Profile::Square, 0, &robot, None, Some(20.0)).unwrap();
        assert!(!world.temporal.feasible);
        assert_eq!(world.category, Category::InfeasibleTask);
        let world = TerrainWorld::generate(Profile::Square, 0, &robot, None, Some(200.0)).unwrap();
        assert!(world.temporal.feasible);
    }

    #[test]
    fn malformed_genes_are_rejected() {
        let robot = LeggedRobot::fixture("quadruped");
        let mut g = genes(&robot, Profile::Hex, [[0.0; 3]; FEATURE_SLOTS]);
        assert!(TerrainWorld::generate(Profile::Hex, 0, &robot, Some(&g), None).is_ok());
        g[0] = 1.5;
        assert!(TerrainWorld::generate(Profile::Hex, 0, &robot, Some(&g), None).is_err());
        g.pop();
        assert!(TerrainWorld::generate(Profile::Hex, 0, &robot, Some(&g), None).is_err());
    }

    #[test]
    fn moving_friction_and_hurdle_features_reach_the_manifest_with_roles() {
        let robot = LeggedRobot::fixture("quadruped");
        let g = genes(&robot, Profile::Square, [[kind(7), 0.5, 0.0], [kind(4), 0.5, 0.4], [kind(6), 0.5, 0.8]]);
        let world = TerrainWorld::generate(Profile::Square, 2, &robot, Some(&g), Some(90.0)).unwrap();
        assert_eq!(world.category, Category::Valid, "{:?}", world.refusal);
        let output = std::env::temp_dir().join(format!("hymeko-terrain-{}", std::process::id()));
        fs::create_dir(&output).unwrap();
        // The fixture has no projection, so the final robot write fails after the scene files exist.
        assert!(export_scene(world.maze.walls(), &world.bodies, 90.0, &robot, &output, json!({})).is_ok()
            || output.join("physics.json").exists());
        let manifest: Value = serde_json::from_slice(&fs::read(output.join("physics.json")).unwrap()).unwrap();
        let objects = manifest["objects"].as_array().unwrap();
        let mover = objects.iter().find(|o| o["kinematic"] == true).expect("kinematic mover");
        assert_eq!(mover["surface"], "mover");
        assert!(mover["motion"]["period_s"].as_f64().unwrap() > 0.0);
        assert!(objects.iter().any(|o| o["friction"].is_number()));
        assert!(objects.iter().any(|o| o["surface"] == "hurdle"));
        assert!(objects.iter().any(|o| o["surface"] == "wall"));
        assert!(fs::read_to_string(output.join("scene.xml")).unwrap().contains("<mujoco"));
        fs::remove_dir_all(output).unwrap();
    }

    #[test]
    fn coverage_walks_visit_every_cell_through_open_passages() {
        let walk = coverage_walk(4, &[[0, 1], [1, 2], [1, 3]]);
        assert_eq!(walk, vec![0, 1, 2, 1, 3]);
        for profile in [Profile::Square, Profile::Hex, Profile::Polar] {
            let robot = LeggedRobot::fixture("quadruped");
            let world = TerrainWorld::generate_task(profile, 1, &robot, None, Some(300.0), TaskKind::Coverage).unwrap();
            let cells: std::collections::BTreeSet<_> = world.waypoint_cells.iter().flatten().collect();
            assert_eq!(cells.len(), world.maze.cells().len(), "{profile:?}");
            assert!(world.route_points.len() >= world.maze.cells().len());
            assert!(world.source.contains("kind \"coverage\""));
            let reach = TerrainWorld::generate(profile, 1, &robot, None, Some(300.0)).unwrap();
            assert!(world.temporal.route_m >= reach.temporal.route_m);
        }
    }

    #[test]
    fn team_worlds_carry_a_verified_plan_records_and_a_member_task_per_robot() {
        let mut scheduled = 0;
        for morphology in ["quadruped", "biped"] {
            let robot = LeggedRobot::fixture(morphology);
            for profile in [Profile::Square, Profile::Hex, Profile::Polar] {
                for seed in 0..3 {
                    let world = TerrainWorld::generate_task(profile, seed, &robot, None, Some(400.0), TaskKind::Team).unwrap();
                    let again = TerrainWorld::generate_task(profile, seed, &robot, None, Some(400.0), TaskKind::Team).unwrap();
                    assert_eq!(world.world.digest(), again.world.digest());
                    assert!(world.checks.iter().all(|c| c["passed"] == true));
                    let team = world.team.as_ref().unwrap();
                    assert_eq!(team.targets.len(), TEAM_TARGETS);
                    assert!(world.source.contains("planner \"prioritized-cell-reservation\""));
                    assert_eq!(CompiledWorld::compile(&world.source).unwrap().digest(), world.world.digest());
                    let Some(plan) = &team.plan else {
                        assert_eq!(world.category, Category::InfeasibleTask);
                        continue;
                    };
                    scheduled += 1;
                    team::verify(plan).unwrap();
                    assert_eq!(world.source.matches("@assign_").count(), TEAM_TARGETS);
                    let task = world.team_task(&robot, plan, &team.waypoints, &team.targets);
                    for (r, member) in task["robots"].as_array().unwrap().iter().enumerate() {
                        let cells = member["cells"].as_array().unwrap().len();
                        let waypoints = member["cell_waypoints"].as_array().unwrap();
                        assert_eq!(waypoints.len(), cells, "{profile:?} {morphology}");
                        let points = member["route_points_m"].as_array().unwrap();
                        assert_eq!(member["waypoint_cells"].as_array().unwrap().len(), points.len());
                        // The start pose stands on the robot's own start cell.
                        let start = &member["start"]["position_m"];
                        assert_eq!(start[0], points[0][0]);
                        assert_eq!(member["cells"][0], json!(team.starts[r]));
                        for gate in member["gates"].as_array().unwrap() {
                            assert!(gate["enter"].as_u64().unwrap() >= 1);
                            assert_eq!(gate["other"], json!(1 - r));
                        }
                    }
                }
            }
        }
        assert!(scheduled > 0, "no team world was scheduled");
    }

    #[test]
    fn team_genes_choose_targets_after_the_feature_slots() {
        let robot = LeggedRobot::fixture("quadruped");
        let profile = Profile::Square;
        let reach = TerrainWorld::dimension(profile, &robot).unwrap();
        let n = TerrainWorld::dimension_for(profile, &robot, TaskKind::Team).unwrap();
        assert_eq!(n, reach + TEAM_TARGETS);
        let mut g = vec![0.5; n];
        for slot in 0..FEATURE_SLOTS {
            g[reach - FEATURE_SLOTS * GENES_PER_FEATURE + slot * GENES_PER_FEATURE] = 0.0;
        }
        g[reach..].copy_from_slice(&[0.0, 0.0, 0.0, 0.0]);
        let world = TerrainWorld::generate_task(profile, 0, &robot, Some(&g), Some(400.0), TaskKind::Team).unwrap();
        assert_eq!(world.team.as_ref().unwrap().targets, vec![1, 2, 3, 4]);
        assert!(world.features.is_empty());
        g.pop();
        assert!(TerrainWorld::generate_task(profile, 0, &robot, Some(&g), Some(400.0), TaskKind::Team).is_err());
        // A biped's square grows to 3x3 for the team, nowhere else.
        let biped = LeggedRobot::fixture("biped");
        assert_eq!(scale_for_task(Profile::Square, &biped, TaskKind::Team).size, 3);
        assert_eq!(scale_for_task(Profile::Square, &biped, TaskKind::Reach).size, 2);
    }

    #[test]
    fn a_mover_slower_than_the_adapter_admits_is_an_invalid_world() {
        let robot = LeggedRobot::fixture("biped");
        let kind_moving = kind(7);
        // A near-zero intensity asks for the longest period; the slow biped exceeds 600 s.
        let g = genes(&robot, Profile::Hex, [[kind_moving, 0.0, 0.0], [0.0; 3], [0.0; 3]]);
        let world = TerrainWorld::generate(Profile::Hex, 0, &robot, Some(&g), Some(200.0)).unwrap();
        assert_eq!(world.category, Category::InvalidWorld, "{:?}", world.features);
        assert!(world.refusal.as_deref().unwrap().contains("period"));
        // The quadruped's movers stay inside the domain at any intensity.
        let quadruped = LeggedRobot::fixture("quadruped");
        let g = genes(&quadruped, Profile::Square, [[kind_moving, 0.0, 0.0], [0.0; 3], [0.0; 3]]);
        let world = TerrainWorld::generate(Profile::Square, 0, &quadruped, Some(&g), Some(200.0)).unwrap();
        assert_ne!(world.category, Category::InvalidWorld, "{:?}", world.refusal);
    }

    #[test]
    fn unit_genes_are_seeded_and_bounded() {
        let a = unit_genes(4, 9);
        assert_eq!(a, unit_genes(4, 9));
        assert_ne!(a, unit_genes(5, 9));
        assert!(a.iter().all(|g| (0.0..1.0).contains(g)));
    }
}
