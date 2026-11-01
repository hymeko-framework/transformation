//! Hydraulic braided muscles as Kawakern force elements.
//!
//! Each declared `hydraulic_muscle` drive (`kinematics::actuation`, law
//! [`DriveLaw::HydraulicMuscle`]) becomes an element whose path runs through its attachments on
//! the moving links: origin, ordered via points, insertion. Along that path sits the three-element
//! Hill arrangement in hydraulic form: the braided sleeve (contractile element, pulling with
//! `p * f(x)` at sleeve length `x`) in parallel with a passive spring and damper, the pair in series
//! with the tendon spring.
//!
//! State per muscle: the valve pressure `p`, lagging the command `u` by a first-order response
//! (`p += (1 - e^(-h/tau)) (u - p)`, exact for a held command), and the sleeve length `x`. Given
//! the path length `L`, `x` solves the series balance implicitly over the step,
//!
//! `k_t max(0, L - x - s) = p f(x) + k_p max(0, x - x_p) + d (x - x_0) / h`,
//!
//! whose left side falls and right side rises in `x`, so the root is unique and bisection finds it
//! on `[shortest pulling length, braid length]`. The tendon force then pulls every path segment's
//! two ends toward each other, mapped to joint torques by `AbaPlan::constraint_force`; a via point
//! takes the difference of its two segments. The force is held over the step like any applied
//! torque. No pump, line, bulk-modulus or valve-flow model is implied.
use crate::articulated::{AbaPlan, AbaScratch};
use crate::kinematics::actuation::{BraidedSleeve, DriveLaw};
use crate::kinematics::kinematic::KinematicModel;
use hymeko_clifford::Se3;

/// The parameters of one muscle, read from its declaration.
#[derive(Clone, Debug)]
pub struct MuscleLaw {
    pub pressure_pa: f64,
    pub response_s: f64,
    pub sleeve: BraidedSleeve,
    pub tendon_stiffness_n_m: f64,
    pub tendon_slack_m: f64,
    pub parallel_stiffness_n_m: f64,
    pub parallel_rest_m: f64,
    pub damping_ns_m: f64,
}

impl MuscleLaw {
    /// Tendon force at tendon length `length`.
    pub fn tendon_force(&self, length: f64) -> f64 {
        self.tendon_stiffness_n_m * (length - self.tendon_slack_m).max(0.0)
    }

    /// Force of the sleeve and the passive spring at sleeve length `x` and pressure `p`.
    pub fn parallel_force(&self, pressure: f64, x: f64) -> f64 {
        pressure * self.sleeve.force_per_pressure(x) + self.parallel_stiffness_n_m * (x - self.parallel_rest_m).max(0.0)
    }

    /// Sleeve length balancing the series arrangement at path length `path`, pressure `pressure`,
    /// with the damper relaxing from `previous` over `h` (`h = None`: static balance).
    pub fn sleeve_length(&self, path: f64, pressure: f64, previous: f64, h: Option<f64>) -> f64 {
        let lo = self.sleeve.shortest_pulling_length() * 0.5;
        let hi = self.sleeve.braid_length_m;
        let residual = |x: f64| {
            let damping = h.map_or(0.0, |h| self.damping_ns_m * (x - previous) / h);
            self.tendon_force(path - x) - self.parallel_force(pressure, x) - damping
        };
        if residual(hi) >= 0.0 {
            return hi;
        }
        if residual(lo) <= 0.0 {
            return lo;
        }
        let (mut a, mut b) = (lo, hi);
        for _ in 0..60 {
            let mid = 0.5 * (a + b);
            if residual(mid) > 0.0 {
                a = mid;
            } else {
                b = mid;
            }
        }
        0.5 * (a + b)
    }
}

/// One muscle: its name, its path on the links, its law.
#[derive(Clone, Debug)]
pub struct MuscleElement {
    pub name: String,
    /// Link index and the point in that link's frame, origin first.
    pub path: Vec<(usize, [f64; 3])>,
    pub law: MuscleLaw,
}

/// The state a muscle carries between steps, and what it last produced.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MuscleState {
    /// Commanded pressure, Pa, held until changed.
    pub command: f64,
    /// Valve pressure, Pa.
    pub pressure: f64,
    /// Sleeve (contractile) length, m.
    pub sleeve_length: f64,
    /// Path length origin to insertion, m.
    pub path_length: f64,
    /// Tendon force, N (pull only).
    pub force: f64,
}

/// What a renderer needs of one muscle at a pose: the sleeve as a polyline and its diameter, and
/// the tendon polyline that continues it to the insertion.
#[derive(Clone, Debug, PartialEq)]
pub struct MuscleShape {
    pub sleeve: Vec<[f64; 3]>,
    pub diameter: f64,
    pub tendon: Vec<[f64; 3]>,
}

/// Every hydraulic muscle of a model.
#[derive(Clone, Debug, Default)]
pub struct Muscles {
    elements: Vec<MuscleElement>,
    states: Vec<MuscleState>,
    torque: Vec<f64>,
    scratch_tau: Vec<f64>,
}

impl Muscles {
    /// Collect the model's hydraulic muscles; `Ok(None)` when it declares none.
    ///
    /// # Errors
    /// A muscle whose attachment or link is missing from the model.
    pub fn from_model(model: &KinematicModel) -> Result<Option<Self>, String> {
        let mut elements = Vec::new();
        for drive in &model.actuation.drives {
            let DriveLaw::HydraulicMuscle {
                pressure_pa,
                response_s,
                sleeve,
                tendon_stiffness_n_m,
                tendon_slack_m,
                parallel_stiffness_n_m,
                parallel_rest_m,
                damping_ns_m,
            } = drive.law
            else {
                continue;
            };
            let mut path = Vec::new();
            for id in std::iter::once(drive.origin).chain(drive.via.iter().copied()).chain([drive.insertion]) {
                let attachment = model
                    .actuation
                    .attachments
                    .iter()
                    .find(|a| a.did == id)
                    .ok_or_else(|| format!("muscle {}: attachment not in the model", drive.name))?;
                let link = model
                    .links
                    .iter()
                    .position(|l| l.did == attachment.link)
                    .ok_or_else(|| format!("muscle {}: attachment link not in the model", drive.name))?;
                path.push((link, attachment.position));
            }
            elements.push(MuscleElement {
                name: drive.name.clone(),
                path,
                law: MuscleLaw {
                    pressure_pa,
                    response_s,
                    sleeve,
                    tendon_stiffness_n_m,
                    tendon_slack_m,
                    parallel_stiffness_n_m,
                    parallel_rest_m,
                    damping_ns_m,
                },
            });
        }
        if elements.is_empty() {
            return Ok(None);
        }
        let states = vec![MuscleState::default(); elements.len()];
        Ok(Some(Self { elements, states, torque: Vec::new(), scratch_tau: Vec::new() }))
    }

    pub fn elements(&self) -> &[MuscleElement] {
        &self.elements
    }

    pub fn states(&self) -> &[MuscleState] {
        &self.states
    }

    /// Index of the muscle named `name`.
    pub fn index(&self, name: &str) -> Option<usize> {
        self.elements.iter().position(|e| e.name == name)
    }

    /// Command muscle `index` to `pressure` Pa, clamped to `0..=pressure_pa` (a sleeve only pulls).
    pub fn command(&mut self, index: usize, pressure: f64) {
        let limit = self.elements[index].law.pressure_pa;
        self.states[index].command = if pressure.is_finite() { pressure.clamp(0.0, limit) } else { 0.0 };
    }

    /// Joint torques the muscles applied over the last step, by plan slot.
    pub fn torque(&self) -> &[f64] {
        &self.torque
    }

    fn points(element: &MuscleElement, poses: &[Se3]) -> Vec<[f64; 3]> {
        element
            .path
            .iter()
            .map(|(link, local)| {
                let p = poses[*link].apply(local);
                [p[0], p[1], p[2]]
            })
            .collect()
    }

    fn length(points: &[[f64; 3]]) -> f64 {
        points.windows(2).map(|w| distance(w[0], w[1])).sum()
    }

    /// Settle every sleeve at its static balance for the current pose (pressures as they are),
    /// so a run starts without a tendon transient.
    pub fn settle(&mut self, poses: &[Se3]) {
        for (element, state) in self.elements.iter().zip(&mut self.states) {
            let path = Self::length(&Self::points(element, poses));
            state.path_length = path;
            state.sleeve_length = element.law.sleeve_length(path, state.pressure, state.sleeve_length, None);
            state.force = element.law.tendon_force(path - state.sleeve_length);
        }
    }

    /// Advance pressures and sleeves over `h` at the pose `poses`, then write the joint torques of
    /// the resulting tendon forces. `aba`'s scratch must be prepared at this configuration.
    pub fn step(&mut self, aba: &AbaPlan, poses: &[Se3], h: f64, scratch: &mut AbaScratch) {
        let n = aba.slots();
        self.torque.clear();
        self.torque.resize(n, 0.0);
        self.scratch_tau.resize(n, 0.0);
        for (element, state) in self.elements.iter().zip(&mut self.states) {
            let law = &element.law;
            state.pressure += (1.0 - (-h / law.response_s).exp()) * (state.command - state.pressure);
            let points = Self::points(element, poses);
            let path = Self::length(&points);
            state.path_length = path;
            state.sleeve_length = law.sleeve_length(path, state.pressure, state.sleeve_length, Some(h));
            state.force = law.tendon_force(path - state.sleeve_length);
            if state.force <= 0.0 {
                continue;
            }
            for (k, pair) in points.windows(2).enumerate() {
                let span = distance(pair[0], pair[1]);
                if span < 1e-9 {
                    continue;
                }
                let u = [(pair[1][0] - pair[0][0]) / span, (pair[1][1] - pair[0][1]) / span, (pair[1][2] - pair[0][2]) / span];
                let pull = [u[0] * state.force, u[1] * state.force, u[2] * state.force];
                let push = [-pull[0], -pull[1], -pull[2]];
                for (end, direction) in [(k, pull), (k + 1, push)] {
                    let (link, _) = element.path[end];
                    aba.constraint_force(poses, link, points[end], direction, scratch, &mut self.scratch_tau);
                    for (total, part) in self.torque.iter_mut().zip(&self.scratch_tau) {
                        *total += part;
                    }
                }
            }
        }
    }

    /// The sleeve runs from the origin along the path for its current length; the tendon is the
    /// rest of the path to the insertion.
    pub fn shapes(&self, poses: &[Se3]) -> Vec<MuscleShape> {
        self.elements
            .iter()
            .zip(&self.states)
            .map(|(element, state)| {
                let points = Self::points(element, poses);
                let mut remaining = state.sleeve_length;
                let mut sleeve = vec![points[0]];
                let mut tendon = Vec::new();
                for pair in points.windows(2) {
                    let span = distance(pair[0], pair[1]);
                    if remaining >= span || span < 1e-12 {
                        remaining -= span;
                        sleeve.push(pair[1]);
                        continue;
                    }
                    if tendon.is_empty() {
                        let t = (remaining / span).max(0.0);
                        let split = [
                            pair[0][0] + t * (pair[1][0] - pair[0][0]),
                            pair[0][1] + t * (pair[1][1] - pair[0][1]),
                            pair[0][2] + t * (pair[1][2] - pair[0][2]),
                        ];
                        sleeve.push(split);
                        tendon.push(split);
                        remaining = 0.0;
                    }
                    tendon.push(pair[1]);
                }
                MuscleShape { sleeve, diameter: element.law.sleeve.diameter(state.sleeve_length), tendon }
            })
            .collect()
    }
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt()
}
