//! MAC geometry and the liquid pressure operator over a validated uniform octree frontier.
//! The iterative solver is Tensor's existing GMRES; this module supplies the fluid stencil.
use super::{Controls, Marker};
use hymeko_graph::octree::Octree;
use hymeko_numerics::diffusion::Diffusion;
use hymeko_numerics::{Operator, gmres};

#[derive(Clone)]
pub(super) struct Face {
    pub slot: usize,
    pub axis: usize,
    pub low: Option<usize>,
    pub high: Option<usize>,
    pub centre: [f64; 3],
}

#[derive(Clone)]
pub(super) struct Grid {
    pub n: usize,
    pub lo: [f64; 3],
    pub hi: [f64; 3],
    pub dx: f64,
    pub faces: Vec<Face>,
    pub outlet_lo: [f64; 3],
    pub outlet_hi: [f64; 3],
}
impl Grid {
    pub fn new(tree: &Octree, outlet_lo: [f64; 3], outlet_hi: [f64; 3]) -> Result<Self, String> {
        let root = &tree.cells[tree.root];
        let leaf = &tree.cells[tree.leaves[0]];
        let dx = leaf.hi[0] - leaf.lo[0];
        let n = ((root.hi[0] - root.lo[0]) / dx).round() as usize;
        if !(2..=8).contains(&n)
            || n.pow(3) != tree.leaves.len()
            || tree
                .leaves
                .iter()
                .any(|&i| (tree.cells[i].hi[0] - tree.cells[i].lo[0] - dx).abs() > dx * 1e-9)
        {
            return Err("fluid requires a uniform 2..8 cubed authored octree frontier".into());
        }
        let mut grid = Self {
            n,
            lo: root.lo,
            hi: root.hi,
            dx,
            faces: Vec::new(),
            outlet_lo,
            outlet_hi,
        };
        let mut seen = vec![false; n.pow(3)];
        for &i in &tree.leaves {
            let index = grid.cell(tree.cells[i].center());
            if seen[index] {
                return Err("octree cells do not form a unique MAC lattice".into());
            }
            seen[index] = true;
        }
        for axis in 0..3 {
            let m = n + 1;
            for flat in 0..m.pow(3) {
                let c = [flat / (m * m), flat / m % m, flat % m];
                if (0..3).any(|a| a != axis && c[a] == n) {
                    continue;
                }
                let mut lower = c;
                lower[axis] = lower[axis].saturating_sub(1);
                grid.faces.push(Face {
                    slot: grid.slot(axis, c),
                    axis,
                    low: (c[axis] > 0).then(|| grid.index(lower)),
                    high: (c[axis] < n).then(|| grid.index(c)),
                    centre: std::array::from_fn(|a| {
                        grid.lo[a] + (c[a] as f64 + if a == axis { 0.0 } else { 0.5 }) * dx
                    }),
                });
            }
        }
        Ok(grid)
    }
    pub fn index(&self, c: [usize; 3]) -> usize {
        (c[0] * self.n + c[1]) * self.n + c[2]
    }
    pub fn cell(&self, p: [f64; 3]) -> usize {
        self.index(std::array::from_fn(|a| {
            (((p[a] - self.lo[a]) / self.dx).floor() as isize).clamp(0, self.n as isize - 1)
                as usize
        }))
    }
    fn slot(&self, axis: usize, c: [usize; 3]) -> usize {
        let m = self.n + 1;
        axis * m.pow(3) + (c[0] * m + c[1]) * m + c[2]
    }
    fn weights(&self, p: [f64; 3], axis: usize) -> [(usize, f64); 8] {
        let q: [f64; 3] = std::array::from_fn(|a| {
            (p[a] - self.lo[a]) / self.dx - if a == axis { 0.0 } else { 0.5 }
        });
        std::array::from_fn(|corner| {
            let c: [isize; 3] =
                std::array::from_fn(|a| q[a].floor() as isize + ((corner >> a) & 1) as isize);
            if (0..3).any(|a| c[a] < 0 || c[a] >= (self.n + usize::from(a == axis)) as isize) {
                return (0, 0.0);
            }
            let w = (0..3)
                .map(|a| {
                    if (corner >> a) & 1 == 1 {
                        q[a] - q[a].floor()
                    } else {
                        1.0 - q[a] + q[a].floor()
                    }
                })
                .product();
            (self.slot(axis, c.map(|v| v as usize)), w)
        })
    }
    pub fn sample(&self, velocity: &[f64], p: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|axis| {
            let weights = self.weights(p, axis);
            let sum: f64 = weights.iter().map(|(_, w)| w).sum();
            weights.iter().map(|&(i, w)| velocity[i] * w).sum::<f64>() / sum.max(1e-12)
        })
    }
    pub fn transfer(&self, particles: &[Marker]) -> (Vec<f64>, Vec<bool>) {
        let mut velocity = vec![0.0; 3 * (self.n + 1).pow(3)];
        let mut weight = velocity.clone();
        let mut liquid = vec![false; self.n.pow(3)];
        for p in particles {
            liquid[self.cell(p.position)] = true;
            for axis in 0..3 {
                for (i, w) in self.weights(p.position, axis) {
                    velocity[i] += w * p.velocity[axis];
                    weight[i] += w;
                }
            }
        }
        for (u, w) in velocity.iter_mut().zip(weight) {
            if w > 0.0 {
                *u /= w;
            }
        }
        (velocity, liquid)
    }
    pub fn is_outlet(&self, p: [f64; 3]) -> bool {
        (1..3).all(|a| p[a] >= self.outlet_lo[a] && p[a] <= self.outlet_hi[a])
    }
    fn opening(&self, face: &Face, valve: f64) -> f64 {
        if face.low.is_some() && face.high.is_some() {
            return 1.0;
        }
        if face.axis == 2 && face.high.is_none() {
            return 1.0;
        }
        if face.axis == 0 && face.high.is_none() && self.is_outlet(face.centre) {
            return valve;
        }
        0.0
    }
    pub fn project(
        &self,
        particles: &[Marker],
        controls: Controls,
        g: f64,
        dt: f64,
        density: f64,
        viscosity: f64,
    ) -> Result<(Vec<f64>, Vec<f64>, f64, f64), String> {
        let (mut velocity, liquid) = self.transfer(particles);
        let viscous_loss = self.diffuse(&mut velocity, &liquid, viscosity, dt, controls.valve)?
            * density
            * self.dx.powi(3);
        let active = |cell: Option<usize>| cell.is_some_and(|i| liquid[i]);
        let mut rhs = vec![0.0; liquid.len()];
        let mut links = Vec::new();
        let mut diagonal = vec![0.0; liquid.len()];
        for f in &self.faces {
            let opening = self.opening(f, controls.valve);
            if opening == 0.0 {
                velocity[f.slot] = 0.0;
                continue;
            }
            if !active(f.low) && !active(f.high) {
                continue;
            }
            if f.axis == 2 {
                velocity[f.slot] -= g * dt;
            }
            if f.low.is_none() || f.high.is_none() {
                velocity[f.slot] *= opening;
            }
            let low = f.low.filter(|&i| liquid[i]);
            let high = f.high.filter(|&i| liquid[i]);
            if let Some(i) = low {
                rhs[i] -= velocity[f.slot];
                diagonal[i] += opening;
            }
            if let Some(i) = high {
                rhs[i] += velocity[f.slot];
                diagonal[i] += opening;
            }
            links.push((f.slot, low, high, opening));
        }
        for (b, d) in rhs.iter_mut().zip(&mut diagonal) {
            if *d == 0.0 {
                *d = 1.0;
            }
            *b /= *d;
        }
        let op = Pressure { diagonal, links };
        let mut pressure = vec![0.0; rhs.len()];
        let report = gmres(&op, 1.0, &rhs, &mut pressure, 1e-7, 32, 160);
        if !report.converged || pressure.iter().any(|p| !p.is_finite()) {
            return Err(format!(
                "fluid pressure solve failed: residual {}",
                report.residual
            ));
        }
        let mut divergence = vec![0.0; rhs.len()];
        for &(slot, low, high, w) in &op.links {
            velocity[slot] -=
                w * (high.map_or(0.0, |i| pressure[i]) - low.map_or(0.0, |i| pressure[i]));
            if let Some(i) = low {
                divergence[i] += velocity[slot] / self.dx;
            }
            if let Some(i) = high {
                divergence[i] -= velocity[slot] / self.dx;
            }
        }
        let residual = divergence.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
        for p in &mut pressure {
            *p *= density * self.dx / dt;
        }
        Ok((velocity, pressure, residual, viscous_loss))
    }

    // Domain projection of nu*Laplacian on MAC dual volumes. Tensor owns both
    // the conservative operator and solve. Missing air neighbours have zero flux;
    // closed normal wall faces use a zero-velocity reservoir (wall reaction).
    pub(super) fn diffuse(
        &self,
        velocity: &mut [f64],
        liquid: &[bool],
        nu: f64,
        dt: f64,
        valve: f64,
    ) -> Result<f64, String> {
        if nu == 0.0 {
            return Ok(0.0);
        }
        let reservoir = velocity.len();
        let mut present = vec![false; reservoir];
        let mut wall = vec![false; reservoir];
        for face in &self.faces {
            present[face.slot] = [face.low, face.high]
                .iter()
                .any(|c| c.is_some_and(|i| liquid[i]));
            wall[face.slot] = self.opening(face, valve) == 0.0;
        }
        let m = self.n + 1;
        let mut links = Vec::new();
        for face in &self.faces {
            if !present[face.slot] {
                continue;
            }
            let flat = face.slot % m.pow(3);
            let c = [flat / (m * m), flat / m % m, flat % m];
            for axis in 0..3 {
                let mut neighbour = c;
                neighbour[axis] += 1;
                if neighbour[axis] >= self.n + usize::from(axis == face.axis) {
                    continue;
                }
                let slot = self.slot(face.axis, neighbour);
                if !present[slot] || (wall[slot] && wall[face.slot]) {
                    continue;
                }
                let a = if wall[face.slot] {
                    reservoir
                } else {
                    face.slot
                };
                let b = if wall[slot] { reservoir } else { slot };
                links.push((a, b, nu / self.dx.powi(2)));
            }
        }
        let op = ViscousBoundary(Diffusion::new(vec![1.0; reservoir + 1], links)?);
        let mut rhs = velocity.to_vec();
        for (i, value) in rhs.iter_mut().enumerate() {
            if wall[i] {
                *value = 0.0;
            }
        }
        rhs.push(0.0);
        let mut next = rhs.clone();
        let report = gmres(&op, dt, &rhs, &mut next, 1e-10, 32, 160);
        if !report.converged || next.iter().any(|v| !v.is_finite()) {
            return Err(format!(
                "fluid viscosity solve failed: residual {}",
                report.residual
            ));
        }
        let loss = 0.5
            * rhs
                .iter()
                .zip(&next)
                .map(|(a, b)| a * a - b * b)
                .sum::<f64>();
        velocity.copy_from_slice(&next[..reservoir]);
        Ok(loss)
    }
}

// The last coordinate is a fixed wall, with zero RHS and zero derivative.
// On the free coordinates this is the symmetric dissipative Dirichlet stencil.
struct ViscousBoundary(Diffusion);
impl Operator for ViscousBoundary {
    fn dim(&self) -> usize {
        self.0.len()
    }
    fn apply(&self, x: &[f64], y: &mut [f64]) {
        self.0.apply(x, y);
        y[self.dim() - 1] = 0.0;
    }
}

struct Pressure {
    diagonal: Vec<f64>,
    links: Vec<(usize, Option<usize>, Option<usize>, f64)>,
}
impl Operator for Pressure {
    fn dim(&self) -> usize {
        self.diagonal.len()
    }
    fn apply(&self, x: &[f64], y: &mut [f64]) {
        // Tensor solves (I-S)x=b. S=I-D^-1 L supplies the Jacobi-scaled pressure stencil.
        y.fill(0.0);
        for &(_, low, high, w) in &self.links {
            let flux = w * (low.map_or(0.0, |i| x[i]) - high.map_or(0.0, |i| x[i]));
            if let Some(i) = low {
                y[i] += flux;
            }
            if let Some(i) = high {
                y[i] -= flux;
            }
        }
        for i in 0..x.len() {
            y[i] = x[i] - y[i] / self.diagonal[i];
        }
    }
}
