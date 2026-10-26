//! Two cooperating robots on one generated world: target assignment, visiting order and
//! collision avoidance on a shared clock.
//!
//! The plan is topological and fixed; it belongs to the task projection, not to the searched
//! generator or the controller. Every assignment of the targets to the two robots and every
//! visiting order is scheduled on the maze's cell graph by prioritized cell reservation; the
//! schedule with the smallest makespan wins (ties: fewer moves, fewer waits, enumeration order).
//! Cell routes come from Graph's Berge Dijkstra over the compiled passages.
//!
//! Occupancy model, in ticks of one cell move: a robot holds its current cell, a moving robot
//! holds both cells, and a robot that has finished holds its last cell for good. Two robots
//! never hold one cell in the same tick. Execution does not follow the ticks: the schedule fixes,
//! per cell, the order in which the robots pass it, and a robot may enter a cell only after the
//! other robot has arrived beyond it for every earlier passage (`Gate`). Derived from one
//! timeline, these precedences are acyclic, so execution cannot deadlock and keeps cells
//! exclusive at any speed, provided each robot stays inside the cells it holds.
use super::*;

pub const TEAM_ROBOTS: usize = 2;
pub const TEAM_TARGETS: usize = 4;

/// Robot `r` may start moving towards its route cell `enter` only once the other robot has
/// arrived at its route cell `reached`.
#[derive(Clone, Copy, Debug, Serialize, PartialEq, Eq)]
pub struct Gate {
    pub enter: usize,
    pub reached: usize,
}

#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct RobotPlan {
    pub start: usize,
    /// Indices into `TeamPlan::targets`, in visiting order.
    pub order: Vec<usize>,
    /// Cell route from the start cell through every assigned target.
    pub cells: Vec<usize>,
    /// Route index at which each assigned target (in `order`) is first reached.
    pub target_index: Vec<usize>,
    /// Planned tick of arrival at each route cell (the start is held from tick 0).
    pub arrival: Vec<usize>,
    pub gates: Vec<Gate>,
}

#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct TeamPlan {
    pub targets: Vec<usize>,
    pub robots: Vec<RobotPlan>,
    /// The robot scheduled first, without waiting.
    pub priority: usize,
    pub makespan_ticks: usize,
    pub moves: usize,
    pub waits: usize,
    /// Assignment x order x priority combinations examined, and how many had a schedule.
    pub candidates: usize,
    pub feasible_candidates: usize,
}

/// Robot 0 starts in the maze's first cell, robot 1 in its last.
pub fn starts(cell_count: usize) -> [usize; TEAM_ROBOTS] {
    [0, cell_count - 1]
}

/// Target cells from genes: each gene picks, without replacement, among the cells that are no
/// start (ascending index order, equal-width bands).
pub fn select_targets(cell_count: usize, genes: &[f64]) -> Result<Vec<usize>> {
    let starts = starts(cell_count);
    let mut free: Vec<usize> = (0..cell_count).filter(|c| !starts.contains(c)).collect();
    if genes.len() != TEAM_TARGETS || free.len() < TEAM_TARGETS {
        return Err(format!("a team task needs {TEAM_TARGETS} target genes and as many free cells").into());
    }
    if genes.iter().any(|g| !g.is_finite() || !(0.0..=1.0).contains(g)) {
        return Err("target genes must be finite values in [0,1]".into());
    }
    Ok(genes
        .iter()
        .map(|g| {
            let i = ((g * free.len() as f64).floor() as usize).min(free.len() - 1);
            free.remove(i)
        })
        .collect())
}

fn permutations(items: &[usize]) -> Vec<Vec<usize>> {
    if items.len() <= 1 {
        return vec![items.to_vec()];
    }
    let mut out = Vec::new();
    for (i, &first) in items.iter().enumerate() {
        let rest: Vec<usize> = items.iter().enumerate().filter(|(j, _)| *j != i).map(|(_, &v)| v).collect();
        for mut tail in permutations(&rest) {
            tail.insert(0, first);
            out.push(tail);
        }
    }
    out
}

/// Cells held by a robot following `cells` without waiting, during tick `t`.
fn held(cells: &[usize], t: usize) -> [usize; 2] {
    if t + 1 < cells.len() {
        [cells[t], cells[t + 1]]
    } else {
        let last = cells[cells.len() - 1];
        [last, last]
    }
}

fn overlaps(a: [usize; 2], b: [usize; 2]) -> bool {
    a.iter().any(|x| b.contains(x))
}

/// Earliest-arrival schedule of `cells` (waiting allowed) around a robot that follows `first`
/// without waiting and then stays on its last cell. `None` when no schedule exists.
fn schedule_second(first: &[usize], cells: &[usize]) -> Option<Vec<usize>> {
    let limit = first.len() + 2 * cells.len() + 2;
    // parent[t][i]: the route index held at tick t-1 that led to index i at tick t.
    let mut parent: Vec<Vec<Option<usize>>> = vec![vec![None; cells.len()]; limit + 1];
    let mut reachable = vec![false; cells.len()];
    reachable[0] = true;
    let last = cells.len() - 1;
    let parks = |t: usize| (t..first.len().max(t + 1)).all(|u| !held(first, u).contains(&cells[last]));
    for t in 0..limit {
        if reachable[last] && parks(t) {
            let mut arrival = vec![0; cells.len()];
            let (mut i, mut u) = (last, t);
            while u > 0 {
                let previous = parent[u][i].expect("reachable state has a parent");
                if previous != i {
                    arrival[i] = u;
                }
                i = previous;
                u -= 1;
            }
            return Some(arrival);
        }
        let mut next = vec![false; cells.len()];
        for i in (0..cells.len()).filter(|&i| reachable[i]) {
            let other = held(first, t);
            // Moving on is tried before waiting; the first parent found is kept.
            if i < last && !overlaps([cells[i], cells[i + 1]], other) && !next[i + 1] {
                next[i + 1] = true;
                parent[t + 1][i + 1] = Some(i);
            }
            if !overlaps([cells[i], cells[i]], other) && !next[i] {
                next[i] = true;
                parent[t + 1][i] = Some(i);
            }
        }
        reachable = next;
        if !reachable.iter().any(|r| *r) {
            return None;
        }
    }
    None
}

/// Visit intervals [enter, leave) in ticks of every route cell; the last one never ends.
fn intervals(arrival: &[usize]) -> Vec<(usize, usize)> {
    (0..arrival.len())
        .map(|k| {
            let enter = if k == 0 { 0 } else { arrival[k] - 1 };
            let leave = if k + 1 < arrival.len() { arrival[k + 1] } else { usize::MAX };
            (enter, leave)
        })
        .collect()
}

/// Precedence gates of robot `me` against `other` from the two schedules.
fn gates(me: &RobotPlan, other: &RobotPlan) -> Vec<Gate> {
    let mine = intervals(&me.arrival);
    let theirs = intervals(&other.arrival);
    let mut out = Vec::new();
    for k in 1..me.cells.len() {
        let (enter, _) = mine[k];
        // The latest earlier passage of the other robot through this cell.
        let earlier = (0..other.cells.len())
            .filter(|&j| other.cells[j] == me.cells[k] && theirs[j].1 <= enter)
            .max_by_key(|&j| theirs[j].1);
        if let Some(j) = earlier {
            out.push(Gate { enter: k, reached: j + 1 });
        }
    }
    out
}

fn route_through(start: usize, order: &[usize], targets: &[usize], route: &impl Fn(usize, usize) -> Vec<usize>) -> (Vec<usize>, Vec<usize>) {
    let mut cells = vec![start];
    let mut index = Vec::new();
    for &t in order {
        let leg = route(*cells.last().expect("non-empty route"), targets[t]);
        cells.extend_from_slice(&leg[1..]);
        index.push(cells.len() - 1);
    }
    (cells, index)
}

/// The fixed team plan, or `None` when no assignment and order has a conflict-free schedule.
/// `route(a, b)` is the cell route from cell `a` to cell `b`, both included.
pub fn plan(cell_count: usize, targets: &[usize], route: impl Fn(usize, usize) -> Vec<usize>) -> Option<TeamPlan> {
    let starts = starts(cell_count);
    let mut best: Option<((usize, usize, usize), TeamPlan)> = None;
    let (mut candidates, mut feasible) = (0, 0);
    for mask in 0..(1_usize << targets.len()) {
        let sets: [Vec<usize>; 2] = [
            (0..targets.len()).filter(|k| mask >> k & 1 == 1).collect(),
            (0..targets.len()).filter(|k| mask >> k & 1 == 0).collect(),
        ];
        for order0 in permutations(&sets[0]) {
            for order1 in permutations(&sets[1]) {
                let orders = [order0.clone(), order1];
                let routes: Vec<_> = (0..TEAM_ROBOTS)
                    .map(|r| route_through(starts[r], &orders[r], targets, &route))
                    .collect();
                for priority in 0..TEAM_ROBOTS {
                    candidates += 1;
                    let second = 1 - priority;
                    let Some(late) = schedule_second(&routes[priority].0, &routes[second].0) else {
                        continue;
                    };
                    feasible += 1;
                    let mut arrival = vec![Vec::new(), Vec::new()];
                    arrival[priority] = (0..routes[priority].0.len()).collect();
                    arrival[second] = late;
                    let finish = |r: usize| *arrival[r].last().expect("non-empty");
                    let makespan = finish(0).max(finish(1));
                    let moves = routes[0].0.len() + routes[1].0.len() - 2;
                    let waits = finish(second) - (routes[second].0.len() - 1);
                    let key = (makespan, moves, waits);
                    if best.as_ref().is_some_and(|(k, _)| *k <= key) {
                        continue;
                    }
                    let mut robots: Vec<RobotPlan> = (0..TEAM_ROBOTS)
                        .map(|r| RobotPlan {
                            start: starts[r],
                            order: orders[r].clone(),
                            cells: routes[r].0.clone(),
                            target_index: routes[r].1.clone(),
                            arrival: arrival[r].clone(),
                            gates: Vec::new(),
                        })
                        .collect();
                    for r in 0..TEAM_ROBOTS {
                        robots[r].gates = gates(&robots[r], &robots[1 - r]);
                    }
                    best = Some((key, TeamPlan {
                        targets: targets.to_vec(),
                        robots,
                        priority,
                        makespan_ticks: makespan,
                        moves,
                        waits,
                        candidates: 0,
                        feasible_candidates: 0,
                    }));
                }
            }
        }
    }
    best.map(|(_, mut plan)| {
        plan.candidates = candidates;
        plan.feasible_candidates = feasible;
        plan
    })
}

/// Independent check of a plan: every held cell is exclusive in every tick, every target is
/// on its robot's route, and the gates name earlier passages only.
pub fn verify(plan: &TeamPlan) -> Result<()> {
    let held_at = |r: &RobotPlan, t: usize| -> Vec<usize> {
        let k = r.arrival.iter().rposition(|&a| a <= t).expect("start is held from tick 0");
        if k + 1 < r.cells.len() && r.arrival[k + 1] == t + 1 {
            vec![r.cells[k], r.cells[k + 1]]
        } else {
            vec![r.cells[k]]
        }
    };
    let horizon = plan.makespan_ticks + 2;
    for t in 0..horizon {
        let (a, b) = (held_at(&plan.robots[0], t), held_at(&plan.robots[1], t));
        if a.iter().any(|c| b.contains(c)) {
            return Err(format!("robots share a cell in tick {t}").into());
        }
    }
    let mut seen = vec![false; plan.targets.len()];
    for r in &plan.robots {
        for (&t, &k) in r.order.iter().zip(&r.target_index) {
            if r.cells[k] != plan.targets[t] || std::mem::replace(&mut seen[t], true) {
                return Err("target not on its robot's route or assigned twice".into());
            }
        }
        if r.arrival.windows(2).any(|w| w[1] <= w[0]) || r.arrival[0] != 0 {
            return Err("arrival ticks must start at 0 and increase".into());
        }
    }
    if seen.iter().any(|s| !s) {
        return Err("unassigned target".into());
    }
    for (i, r) in plan.robots.iter().enumerate() {
        let other = &plan.robots[1 - i];
        for g in &r.gates {
            if g.enter == 0 || g.enter >= r.cells.len() || g.reached == 0 || g.reached >= other.cells.len()
                || other.arrival[g.reached] > r.arrival[g.enter] - 1
            {
                return Err("gate does not name an earlier passage".into());
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A path graph 0-1-2-...-n-1: the route between two cells is the index range.
    fn line(a: usize, b: usize) -> Vec<usize> {
        if a <= b { (a..=b).collect() } else { (b..=a).rev().collect() }
    }

    /// A hub tree: both starts (0 and 5) hang on hub 1, which leads to the branches 1-2-3 and 1-4.
    fn hub(a: usize, b: usize) -> Vec<usize> {
        let parent = [1, 1, 1, 2, 1, 1]; // the hub is its own parent
        let up = |mut c: usize| {
            let mut path = vec![c];
            while c != 1 {
                c = parent[c];
                path.push(c);
            }
            path
        };
        let (mut from, to) = (up(a), up(b));
        // Through the hub, then down; drop the duplicated common suffix.
        let common = from.iter().rev().zip(to.iter().rev()).take_while(|(x, y)| x == y).count();
        from.truncate(from.len() - common + 1);
        from.extend(to[..to.len() - common].iter().rev());
        from
    }

    #[test]
    fn targets_are_distinct_free_cells_chosen_by_genes() {
        let t = select_targets(9, &[0.0, 0.0, 0.99, 0.5]).unwrap();
        assert_eq!(t[0], 1);
        assert_eq!(t[1], 2);
        assert!(t.iter().all(|c| ![0, 8].contains(c)));
        let mut sorted = t.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), TEAM_TARGETS);
        assert!(select_targets(5, &[0.1; 4]).is_err());
        assert!(select_targets(9, &[1.5, 0.0, 0.0, 0.0]).is_err());
        assert_eq!(select_targets(9, &[0.3; 4]).unwrap(), select_targets(9, &[0.3; 4]).unwrap());
    }

    #[test]
    fn on_a_line_each_robot_takes_its_own_side() {
        let plan = plan(6, &[1, 2, 3, 4], line).unwrap();
        verify(&plan).unwrap();
        // Crossing would need passing in a corridor; the best plan splits the line.
        assert_eq!(plan.robots[0].cells, vec![0, 1, 2]);
        assert_eq!(plan.robots[1].cells, vec![5, 4, 3]);
        assert_eq!(plan.makespan_ticks, 2);
        assert!(plan.robots.iter().all(|r| r.gates.is_empty()));
    }

    #[test]
    fn a_shared_hub_is_passed_in_turn_with_gates() {
        assert_eq!(hub(0, 3), vec![0, 1, 2, 3]);
        assert_eq!(hub(3, 4), vec![3, 2, 1, 4]);
        assert_eq!(hub(2, 2), vec![2]);
        // One robot alone needs 6 moves; split, both pass the hub and one waits for the other.
        let plan = plan(6, &[1, 2, 3, 4], hub).unwrap();
        verify(&plan).unwrap();
        assert_eq!(plan.makespan_ticks, 4, "{plan:?}");
        assert!(plan.waits > 0);
        let gated: usize = plan.robots.iter().map(|r| r.gates.len()).sum();
        assert!(gated > 0, "the hub must be shared in turn: {plan:?}");
        // Deterministic: the same inputs give the same plan.
        assert_eq!(plan, super::plan(6, &[1, 2, 3, 4], hub).unwrap());
    }

    #[test]
    fn a_blocked_corridor_has_no_schedule() {
        // Robot 0 parks at its start in the middle of a line; robot 1 must pass it.
        let first = vec![2];
        assert!(schedule_second(&first, &[4, 3, 2, 1]).is_none());
        // Without the parked robot in the way the second robot waits for it to pass.
        let arrival = schedule_second(&[0, 1, 2, 3], &[5, 4]).unwrap();
        assert_eq!(arrival, vec![0, 1]);
        let arrival = schedule_second(&[0, 1, 2, 3, 4], &[6, 5, 4]);
        assert!(arrival.is_none(), "robot 0 parks on the second robot's goal");
    }

    #[test]
    fn verification_rejects_shared_cells_and_misassigned_targets() {
        let mut plan = plan(6, &[1, 2, 3, 4], line).unwrap();
        verify(&plan).unwrap();
        let mut broken = plan.clone();
        broken.robots[1].cells = vec![5, 4, 3, 2];
        broken.robots[1].arrival = vec![0, 1, 2, 3];
        broken.makespan_ticks = 3;
        assert!(verify(&broken).is_err());
        plan.robots[0].order.swap(0, 1);
        assert!(verify(&plan).is_err());
    }
}
