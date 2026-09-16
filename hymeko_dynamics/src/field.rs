//! Where the fields are, once the model says which mode each coordinate is the amplitude of.
//!
//! # What this is not
//!
//! It is not a field solver. The dynamics is lumped: four coordinates, no space, no `E(x, y, z)`.
//! What a [`Region`] adds is the missing half of a modal expansion — the basis function the
//! coordinate multiplies — so that
//!
//! ```text
//! E(x, t) = shape(x) * effort(t)
//! ```
//!
//! with the TIME factor coming from the simulated trajectory and the SHAPE factor from the
//! declaration. Neither half is invented by a renderer.
//!
//! # Why the geometry is checked and not merely carried
//!
//! A shape implies a capacity from vacuum constants alone: `eps0 eps_r A / s` for a gap,
//! `mu0 N^2 A / l` for a coil. If that disagrees with the coordinate's declared capacity then the
//! drawn field carries a different energy from the simulated coordinate, and the picture is an
//! illustration standing beside the physics rather than a projection of it. So both are computed and
//! [`Region::calibration_error`] reports the difference, which a test holds at zero.
//!
//! The identity that makes it worth doing:
//!
//! ```text
//! integral over the region of the energy density  ==  x^2 / (2 c)  ==  H_k(x)
//! ```
//!
//! The left side is geometry and vacuum constants; the right side is the Hamiltonian term the
//! integrator conserves. They are the same number or the model is inconsistent.

use hymeko::common::ids::DeclId;

/// Vacuum permittivity, F/m. A universal constant, so it lives in the projection rather than in the
/// model — what the model declares is the MATERIAL, `relative_permittivity`.
pub const VACUUM_PERMITTIVITY: f64 = 8.854_187_812_8e-12;
/// Vacuum permeability, H/m.
pub const VACUUM_PERMEABILITY: f64 = 1.256_637_062_12e-6;

/// The spatial mode a coordinate is the amplitude of.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    /// Uniform field between two plates. The conjugate effort is a voltage.
    ParallelPlate {
        area: f64,
        separation: f64,
        relative_permittivity: f64,
    },
    /// Uniform field inside a solenoid. The conjugate effort is a current.
    Solenoid { turns: f64, area: f64, length: f64 },
}

impl Shape {
    /// The volume the field occupies.
    pub fn volume(&self) -> f64 {
        match *self {
            Self::ParallelPlate {
                area, separation, ..
            } => area * separation,
            Self::Solenoid { area, length, .. } => area * length,
        }
    }

    /// The capacity this geometry implies — `eps0 eps_r A / s`, or `mu0 N^2 A / l`.
    ///
    /// This is the number that must equal the coordinate's declared `capacity`.
    pub fn implied_capacity(&self) -> f64 {
        match *self {
            Self::ParallelPlate {
                area,
                separation,
                relative_permittivity,
            } => VACUUM_PERMITTIVITY * relative_permittivity * area / separation,
            Self::Solenoid {
                turns,
                area,
                length,
            } => VACUUM_PERMEABILITY * turns * turns * area / length,
        }
    }

    /// The field magnitude at this effort: volts per metre in a gap, amperes per metre in a coil.
    pub fn field(&self, effort: f64) -> f64 {
        match *self {
            Self::ParallelPlate { separation, .. } => effort / separation,
            Self::Solenoid { turns, length, .. } => turns * effort / length,
        }
    }

    /// The energy density at this effort, in joules per cubic metre.
    pub fn energy_density(&self, effort: f64) -> f64 {
        let field = self.field(effort);
        match *self {
            Self::ParallelPlate {
                relative_permittivity,
                ..
            } => 0.5 * VACUUM_PERMITTIVITY * relative_permittivity * field * field,
            Self::Solenoid { .. } => 0.5 * VACUUM_PERMEABILITY * field * field,
        }
    }

    /// The energy in the whole region. Equals `0.5 * implied_capacity * effort^2` identically.
    pub fn energy(&self, effort: f64) -> f64 {
        self.energy_density(effort) * self.volume()
    }

    /// Is this an electric region? The other kind is magnetic; there are only two.
    pub fn is_electric(&self) -> bool {
        matches!(self, Self::ParallelPlate { .. })
    }
}

/// One coordinate's field, placed for a drawing.
#[derive(Debug, Clone, PartialEq)]
pub struct Region {
    pub decl: DeclId,
    pub name: String,
    /// Which coordinate this region holds the energy of.
    pub coordinate: usize,
    pub shape: Shape,
    /// Where the region sits in a drawing. Presentation: it does not enter the dynamics.
    pub centre: [f64; 3],
    /// The field direction, and the solenoid's axis. Normalised by the projection.
    pub axis: [f64; 3],
}

impl Region {
    /// How far the declared capacity and the geometry disagree, relatively.
    ///
    /// Zero on a consistent model. Anything else means the drawn field and the simulated coordinate
    /// hold different amounts of energy, and the picture is not a projection of the physics.
    pub fn calibration_error(&self, declared_capacity: f64) -> f64 {
        let implied = self.shape.implied_capacity();
        (implied - declared_capacity).abs() / declared_capacity.abs().max(f64::MIN_POSITIVE)
    }

    /// The region's true extent in metres: across the face, and along the axis.
    pub fn span(&self) -> (f64, f64) {
        match self.shape {
            Shape::ParallelPlate {
                area, separation, ..
            } => (area.sqrt(), separation),
            Shape::Solenoid { area, length, .. } => {
                (2.0 * (area / std::f64::consts::PI).sqrt(), length)
            }
        }
    }

    /// A grid of field samples across the region's face, for a renderer to draw.
    ///
    /// One implementation, so a native renderer and a browser one cannot drift apart in what they
    /// think the field is.
    ///
    /// # Why the offsets are dimensionless
    ///
    /// A sample says WHERE on the face it sits, in `[-1, 1]` across each of the face's two
    /// directions, and WHAT the field is there, in the field's own units. It does not say where that
    /// is in a drawing, because it cannot: the region is a fifth of a metre across with a millimetre
    /// between its plates, and no renderer draws that to scale. Mixing the two — offsets in metres
    /// added to a centre in scene units — is a unit error that happens to typecheck, and it was one
    /// here before this comment was.
    ///
    /// [`Self::span`] reports the true extent for anyone who wants the real aspect ratio.
    pub fn sample(&self, effort: f64, across: usize) -> Vec<Sample> {
        let across = across.max(1);
        let magnitude = self.shape.field(effort);
        let axis = normalise(self.axis);
        let (u, v) = frame(axis);
        let round = matches!(self.shape, Shape::Solenoid { .. });

        let mut out = Vec::with_capacity(across * across);
        for i in 0..across {
            for j in 0..across {
                let (a, b) = (span(i, across), span(j, across));
                // A round region drops the corners its cross-section does not have.
                if round && (a * a + b * b).sqrt() > 1.0 {
                    continue;
                }
                out.push(Sample {
                    offset: [
                        u[0] * a + v[0] * b,
                        u[1] * a + v[1] * b,
                        u[2] * a + v[2] * b,
                    ],
                    across: [a, b],
                    direction: axis,
                    magnitude,
                });
            }
        }
        out
    }
}

/// One field sample: where on the face it sits, which way the field points, and how strong it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    /// Displacement from the region's centre across its face, each component in `[-1, 1]`.
    /// Dimensionless: a renderer multiplies by whatever size it draws the region at.
    pub offset: [f64; 3],
    /// The same displacement in the face's own two directions, for a renderer that wants them apart.
    pub across: [f64; 2],
    /// The field direction, unit length. The sign of [`Self::magnitude`] carries the rest.
    pub direction: [f64; 3],
    /// Volts per metre in an electric region, amperes per metre in a magnetic one.
    pub magnitude: f64,
}

/// The Poynting flux between one electric and one magnetic region, in watts per square metre.
///
/// `S = E x H`, and the two are collinear in this reduced model — each region's field lies along its
/// own declared axis — so what is returned is the magnitude with the sign of the product. A renderer
/// draws it along the axis joining the two regions, which is where the curl port moves the energy.
///
/// This is NOT the port's power. The port carries `g * effort_e * effort_h` watts; this is the flux
/// density of the reduced mode, and the two differ by the geometry. Both are useful and they are not
/// the same quantity.
pub fn poynting(
    electric: &Region,
    electric_effort: f64,
    magnetic: &Region,
    magnetic_effort: f64,
) -> f64 {
    electric.shape.field(electric_effort) * magnetic.shape.field(magnetic_effort)
}

fn span(i: usize, n: usize) -> f64 {
    if n == 1 {
        0.0
    } else {
        -1.0 + 2.0 * i as f64 / (n - 1) as f64
    }
}

fn normalise(v: [f64; 3]) -> [f64; 3] {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length <= 0.0 {
        [0.0, 1.0, 0.0]
    } else {
        [v[0] / length, v[1] / length, v[2] / length]
    }
}

/// Two unit vectors perpendicular to `axis` and to each other.
fn frame(axis: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    // Pick the world axis least aligned with this one, so the cross product is well conditioned.
    let helper = if axis[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let u = normalise(cross(axis, helper));
    (u, cross(axis, u))
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
