//! Where the circuit goes when the model does not say.
//!
//! A terminal may declare `at [x, y, z]`. When it does not, the position is DERIVED — and derived
//! from the same place the dynamics is: the incidence between terminals and the conductors between
//! them. Hall's spectral layout minimises the total squared edge length under a unit-spread
//! constraint, and its solution is the two eigenvectors belonging to the smallest non-zero
//! eigenvalues of the graph Laplacian. Terminals that are strongly connected land near each other
//! because that is what the objective says, not because a force simulation was run until it looked
//! settled — so the drawing is reproducible, and the same model always draws the same way.
//!
//! Declared positions are PINNED. The derived drawing is fitted onto them by the best similarity
//! transform — rotation, uniform scale and translation, in closed form — and the pinned terminals
//! are then placed exactly where the model put them. A model may therefore fix its rails and let
//! everything else arrange itself, which is the useful middle between "every coordinate by hand"
//! and "no say at all".

/// How far a fully derived layout spreads, in scene units. Chosen to sit in the same frame as a
/// hand-drawn one, so a camera aimed at either sees a circuit rather than a dot or a sprawl.
const SPREAD: f64 = 2.2;
/// Two terminals closer than this are pushed apart, because a picture in which two nodes coincide
/// is a picture that cannot be clicked on.
const MIN_GAP: f64 = 0.30;
/// How far apart two components are set down. They share no conductor, so nothing but legibility
/// decides this.
const COMPONENT_GAP: f64 = 2.8;

/// Bounded diagram layout in pixels, following positive-to-negative connections.
/// O(V+E) work and storage; stable input order breaks ties and cyclic components.
/// Cycles may have backward bonds; this is not a topological-order certificate.
/// Returns None above 500 nodes/2000 bonds or for an out-of-range endpoint.
pub fn layered(count: usize, edges: &[(usize, usize)]) -> Option<Vec<[f64; 2]>> {
    if count > 500 || edges.len() > 2000 || edges.iter().any(|&(a, b)| a >= count || b >= count) {
        return None;
    }
    let mut next = vec![Vec::new(); count];
    let mut incoming = vec![0usize; count];
    for &(a, b) in edges {
        next[a].push(b);
        incoming[b] += 1;
    }
    let mut layer = vec![None; count];
    let mut queue = std::collections::VecDeque::new();
    let roots = (0..count).filter(|&i| incoming[i] == 0).chain(0..count);
    for root in roots {
        if layer[root].is_some() {
            continue;
        }
        layer[root] = Some(0usize);
        queue.push_back(root);
        while let Some(a) = queue.pop_front() {
            let depth = layer[a].unwrap_or(0);
            for &b in &next[a] {
                if layer[b].is_none() {
                    layer[b] = Some(depth + 1);
                    queue.push_back(b);
                }
            }
        }
    }
    let mut rows = vec![0usize; count.max(1)];
    Some(
        layer
            .into_iter()
            .map(|depth| {
                let depth = depth.unwrap_or(0);
                let row = rows[depth];
                rows[depth] += 1;
                [40.0 + depth as f64 * 320.0, 40.0 + row as f64 * 190.0]
            })
            .collect(),
    )
}

/// Positions for every terminal: the declared ones exactly, the rest derived from the graph.
///
/// `edges` are terminal index pairs, one per conductor; parallel edges and self-loops are tolerated
/// (a self-loop contributes nothing, which is what it means).
pub fn arrange(
    count: usize,
    edges: &[(usize, usize)],
    pinned: &[Option<[f64; 3]>],
) -> Vec<[f64; 3]> {
    if count == 0 {
        return Vec::new();
    }
    let mut raw = vec![[0.0f64; 2]; count];

    // Each connected component is laid out on its own. A Laplacian's null space has one dimension
    // per component, so running the spectral solve across a disconnected graph returns component
    // INDICATORS — every node of a component stacked on one point — rather than a drawing.
    let mut offset = 0.0;
    for component in components(count, edges) {
        let local = spectral(&component, edges);
        let radius = local
            .iter()
            .fold(0.0f64, |m, p| m.max(p[0].hypot(p[1])))
            .max(f64::MIN_POSITIVE);
        for (slot, &node) in component.iter().enumerate() {
            raw[node] = [local[slot][0] / radius + offset, local[slot][1] / radius];
        }
        offset += COMPONENT_GAP;
    }

    let anchors: Vec<usize> = (0..count).filter(|&i| pinned[i].is_some()).collect();
    let mut out = if anchors.is_empty() {
        let radius = raw
            .iter()
            .fold(0.0f64, |m, p| m.max(p[0].hypot(p[1])))
            .max(f64::MIN_POSITIVE);
        raw.iter()
            .map(|p| [p[0] * SPREAD / radius, p[1] * SPREAD / radius, 0.0])
            .collect()
    } else {
        fit(&raw, pinned, &anchors)
    };

    for &i in &anchors {
        out[i] = pinned[i].expect("anchors are the pinned indices");
    }
    separate(&mut out, pinned);
    out
}

/// Bends for conductors that would otherwise lie exactly on top of each other.
///
/// Two branches between the same pair of terminals are two different elements and have to be two
/// different things to click on. A model may say where a conductor bows with `via`; where it does
/// not, the parallel group is fanned out around the straight line between its endpoints.
pub fn bends(
    positions: &[[f64; 3]],
    edges: &[(usize, usize)],
    declared: &[Option<[f64; 3]>],
) -> Vec<Option<[f64; 3]>> {
    let mut out = declared.to_vec();
    let mut seen: Vec<((usize, usize), Vec<usize>)> = Vec::new();
    for (w, &(a, b)) in edges.iter().enumerate() {
        let key = (a.min(b), a.max(b));
        match seen.iter_mut().find(|(k, _)| *k == key) {
            Some((_, group)) => group.push(w),
            None => seen.push((key, vec![w])),
        }
    }
    for ((a, b), group) in seen {
        if group.len() < 2 || a == b {
            continue;
        }
        let (p, q) = (positions[a], positions[b]);
        let direction = sub(q, p);
        let length = norm(direction);
        if length <= f64::MIN_POSITIVE {
            continue;
        }
        let across = perpendicular(scale(direction, 1.0 / length));
        let middle = scale(add(p, q), 0.5);
        let step = 0.22 * length;
        for (slot, &w) in group.iter().enumerate() {
            if out[w].is_some() {
                continue;
            }
            let lane = slot as f64 - (group.len() - 1) as f64 / 2.0;
            if lane.abs() < 1e-12 {
                continue;
            }
            out[w] = Some(add(middle, scale(across, lane * step)));
        }
    }
    out
}

/// A drawn conductor, as placing a store needs to see it.
pub struct Conductor {
    pub from: usize,
    pub to: usize,
    pub coordinate: usize,
    /// The middle of the route as drawn, bend included.
    pub middle: [f64; 3],
    /// Whether this is the supply rather than a conductor.
    pub supply: bool,
}

/// Where a store's geometry sits, when the model does not place it.
///
/// A region holds one coordinate's energy, and the circuit already says where that energy is. A
/// coordinate a conductor CARRIES is in series with it, so the energy is along that conductor and
/// its axis is the direction of the run. A coordinate no conductor carries is in shunt, and it sits
/// at the node it hangs from — the terminal shared by the conductors it exchanges energy with,
/// counting every terminal except the reference, which is the side every shunt store is tied to and
/// therefore tells them apart from nothing.
///
/// Returns the centre, and the axis where the circuit implies one. A shunt store's orientation is
/// not implied by anything here, and saying nothing is better than picking.
pub fn place_store(
    coordinate: usize,
    coupled: &[usize],
    conductors: &[Conductor],
    terminals: &[[f64; 3]],
    reference: Option<usize>,
) -> Option<([f64; 3], Option<[f64; 3]>)> {
    // A source and the branch it drives are in series and carry the same coordinate. The store is
    // the BRANCH's — a loop's inductance is around the loop, not inside the supply — so a conductor
    // is preferred, and the supply is used only when nothing else carries it, which is the case
    // where the store really is the source's own.
    let carried = |supply: bool| {
        conductors
            .iter()
            .find(|c| c.coordinate == coordinate && c.supply == supply)
    };
    if let Some(run) = carried(false).or_else(|| carried(true)) {
        let along = sub(terminals[run.to], terminals[run.from]);
        let length = norm(along);
        return Some((
            run.middle,
            (length > 1e-9).then(|| scale(along, 1.0 / length)),
        ));
    }

    let mut tally = vec![0usize; terminals.len()];
    for run in conductors
        .iter()
        .filter(|c| coupled.contains(&c.coordinate))
    {
        tally[run.from] += 1;
        tally[run.to] += 1;
    }
    if let Some(ground) = reference {
        tally[ground] = 0;
    }
    let most = tally.iter().copied().max().filter(|n| *n > 0)?;
    let mut winners = tally.iter().enumerate().filter(|(_, n)| **n == most);
    let (node, _) = winners.next()?;
    // Two nodes with an equal claim is an ambiguity, and inventing a tie-break would put the store
    // somewhere the circuit did not say to.
    winners.next().is_none().then(|| (terminals[node], None))
}

// ─── the spectral drawing ───────────────────────────────────────────────────────────────────────

/// Hall's layout of one connected component: the eigenvectors of its Laplacian belonging to the two
/// smallest non-zero eigenvalues.
fn spectral(nodes: &[usize], edges: &[(usize, usize)]) -> Vec<[f64; 2]> {
    let n = nodes.len();
    if n == 1 {
        return vec![[0.0, 0.0]];
    }
    let slot = |node: usize| nodes.iter().position(|&x| x == node);
    let mut laplacian = vec![0.0; n * n];
    for &(a, b) in edges {
        let (Some(i), Some(j)) = (slot(a), slot(b)) else {
            continue;
        };
        if i == j {
            continue;
        }
        laplacian[i * n + j] -= 1.0;
        laplacian[j * n + i] -= 1.0;
        laplacian[i * n + i] += 1.0;
        laplacian[j * n + j] += 1.0;
    }
    let (values, vectors) = jacobi(laplacian, n);

    // Index 0 is the constant vector, whose eigenvalue is zero on a connected component; the drawing
    // is the next two. A component of two nodes has only one non-trivial direction, and a line is
    // the correct picture of it rather than a degenerate one — so a missing direction stays zero
    // instead of being filled with something arbitrary.
    let floor = 1e-9 * values[n - 1].max(1.0);
    let mut out = vec![[0.0f64; 2]; n];
    let mut taken = 0;
    for k in 1..n {
        if values[k] <= floor {
            continue;
        }
        // An eigenvector is only defined up to sign, and a drawing that flips between two runs of
        // the same model is not reproducible. Fixing the largest component positive settles it.
        let lead = (0..n)
            .max_by(|&a, &b| {
                vectors[a * n + k]
                    .abs()
                    .total_cmp(&vectors[b * n + k].abs())
                    .then(b.cmp(&a))
            })
            .expect("a component has at least one node");
        let sign = if vectors[lead * n + k] < 0.0 {
            -1.0
        } else {
            1.0
        };
        for i in 0..n {
            out[i][taken] = sign * vectors[i * n + k];
        }
        taken += 1;
        if taken == 2 {
            break;
        }
    }
    out
}

/// Cyclic Jacobi for a symmetric matrix: eigenvalues ascending, with their eigenvectors.
///
/// Small, dense and symmetric is exactly the case Jacobi is best at, and it is the case here — a
/// terminal graph has as many nodes as somebody drew. `vectors[i * n + k]` is component `i` of
/// eigenvector `k`.
fn jacobi(mut a: Vec<f64>, n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }
    for _ in 0..100 {
        let off = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .map(|(i, j)| a[i * n + j] * a[i * n + j])
            .sum::<f64>()
            .sqrt();
        if off <= 1e-14 * n as f64 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = a[p * n + q];
                if apq.abs() <= 1e-300 {
                    continue;
                }
                let theta = (a[q * n + q] - a[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
                for k in 0..n {
                    let (vkp, vkq) = (v[k * n + p], v[k * n + q]);
                    v[k * n + p] = c * vkp - s * vkq;
                    v[k * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&i, &j| a[i * n + i].total_cmp(&a[j * n + j]));
    let values = order.iter().map(|&k| a[k * n + k]).collect();
    let mut vectors = vec![0.0; n * n];
    for (slot, &k) in order.iter().enumerate() {
        for i in 0..n {
            vectors[i * n + slot] = v[i * n + k];
        }
    }
    (values, vectors)
}

// ─── fitting the drawing onto what the model pinned ─────────────────────────────────────────────

/// The similarity transform — rotation, uniform scale, translation — that best carries the derived
/// points onto the declared ones, in closed form.
///
/// In two dimensions a similarity is multiplication by a complex number, so the least-squares fit is
/// one quotient rather than an iteration: `c = sum(conj(a) b) / sum(|a|^2)`.
fn fit(raw: &[[f64; 2]], pinned: &[Option<[f64; 3]>], anchors: &[usize]) -> Vec<[f64; 3]> {
    let m = anchors.len() as f64;
    let sum_raw = anchors.iter().fold([0.0; 2], |s, &i| add2(s, raw[i]));
    let mean_raw = [sum_raw[0] / m, sum_raw[1] / m];
    let sum_pin = anchors.iter().fold([0.0; 3], |s, &i| {
        add(s, pinned[i].expect("anchors are pinned"))
    });
    let mean_pin = scale(sum_pin, 1.0 / m);

    // One anchor fixes where the drawing sits and nothing about its size or orientation, so the
    // derived spread is kept and only the offset comes from the model. The same is true of several
    // anchors that all sit on one point.
    let (cr, ci) = if anchors.len() < 2 {
        (SPREAD, 0.0)
    } else {
        let (mut re, mut im, mut den) = (0.0, 0.0, 0.0);
        for &i in anchors {
            let (ax, ay) = (raw[i][0] - mean_raw[0], raw[i][1] - mean_raw[1]);
            let p = pinned[i].expect("anchors are pinned");
            let (bx, by) = (p[0] - mean_pin[0], p[1] - mean_pin[1]);
            re += ax * bx + ay * by;
            im += ax * by - ay * bx;
            den += ax * ax + ay * ay;
        }
        if den <= 1e-12 || re.hypot(im) <= 1e-12 * den {
            (SPREAD, 0.0)
        } else {
            (re / den, im / den)
        }
    };

    raw.iter()
        .map(|p| {
            let (ax, ay) = (p[0] - mean_raw[0], p[1] - mean_raw[1]);
            [
                cr * ax - ci * ay + mean_pin[0],
                ci * ax + cr * ay + mean_pin[1],
                mean_pin[2],
            ]
        })
        .collect()
}

/// Push apart terminals that landed on top of each other, moving only the ones the model did not
/// place. Deterministic: where two points coincide exactly the direction comes from their indices.
fn separate(points: &mut [[f64; 3]], pinned: &[Option<[f64; 3]>]) {
    for _ in 0..24 {
        let mut moved = false;
        for i in 0..points.len() {
            for j in i + 1..points.len() {
                let delta = sub(points[j], points[i]);
                let distance = norm(delta);
                if distance >= MIN_GAP {
                    continue;
                }
                let push = if distance > 1e-9 {
                    scale(delta, (MIN_GAP - distance) / distance)
                } else {
                    let angle = (i * 7 + j) as f64;
                    [MIN_GAP * angle.cos(), MIN_GAP * angle.sin(), 0.0]
                };
                match (pinned[i].is_none(), pinned[j].is_none()) {
                    (true, true) => {
                        points[i] = sub(points[i], scale(push, 0.5));
                        points[j] = add(points[j], scale(push, 0.5));
                    }
                    (true, false) => points[i] = sub(points[i], push),
                    (false, true) => points[j] = add(points[j], push),
                    // Both were placed by hand. The model said so, and this is not the place to
                    // overrule it.
                    (false, false) => continue,
                }
                moved = true;
            }
        }
        if !moved {
            break;
        }
    }
}

/// Which connected component each terminal belongs to.
///
/// Two terminals share an id exactly when a run of conductors joins them, which is the condition for
/// a potential difference between them to exist at all.
pub fn component_of(count: usize, edges: &[(usize, usize)]) -> Vec<usize> {
    let mut out = vec![0; count];
    for (id, members) in components(count, edges).into_iter().enumerate() {
        for node in members {
            out[node] = id;
        }
    }
    out
}

/// The connected components of the terminal graph, each as a sorted list of terminal indices.
fn components(count: usize, edges: &[(usize, usize)]) -> Vec<Vec<usize>> {
    let mut parent: Vec<usize> = (0..count).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    for &(a, b) in edges {
        if a >= count || b >= count {
            continue;
        }
        let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
        if ra != rb {
            parent[ra] = rb;
        }
    }
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for i in 0..count {
        let r = root(&mut parent, i);
        match groups.iter_mut().find(|(key, _)| *key == r) {
            Some((_, members)) => members.push(i),
            None => groups.push((r, vec![i])),
        }
    }
    groups.into_iter().map(|(_, members)| members).collect()
}

// ─── three-vector arithmetic, local because it is three lines ───────────────────────────────────

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn add2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}
/// A unit vector across the given direction, in the drawing plane where there is one.
fn perpendicular(d: [f64; 3]) -> [f64; 3] {
    let candidate = [-d[1], d[0], 0.0];
    let length = norm(candidate);
    if length > 1e-9 {
        scale(candidate, 1.0 / length)
    } else {
        [1.0, 0.0, 0.0]
    }
}
