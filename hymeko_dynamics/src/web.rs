//! The dynamical projection, callable from a browser.
//!
//! The companion to `hymeko_optimize::web`. That one solves a network with Kirchhoff's laws and
//! returns a design; this one takes the same circuit read as a port-Hamiltonian system and returns
//! what it actually does on the way to that design — the modes it rings at, and the trajectory.
//!
//! Neither needs a GPU. A nodal solve is a linear system and a transient is a sequence of them, so
//! both run in a browser exactly as they run on a desktop.

use wasm_bindgen::prelude::*;

use crate::analysis::modes;
use crate::run::{Integrator, run};
use crate::sources::project_sources;
use crate::system::Readout;

/// Compile a declared dynamical system, analyse it, and run it.
///
/// `duration` and `step` are in seconds. When either is not positive they are chosen from the
/// system's own modes: long enough for the slowest to decay, fine enough to resolve the fastest.
/// That is a presentation default, not a physical one, and the caller can always say otherwise.
///
/// Returns JSON: the modes, a decimated trajectory of every declared readout, and the settled value
/// of each — which is the number a nodal solve of the same circuit must agree with.
#[wasm_bindgen]
pub fn transient(
    entry: String,
    paths: Vec<String>,
    texts: Vec<String>,
    duration: f64,
    step: f64,
    samples: usize,
) -> Result<String, JsValue> {
    if paths.len() != texts.len() {
        return Err(JsValue::from_str("paths and texts must be the same length"));
    }
    let files: Vec<(&str, &str)> = paths
        .iter()
        .map(String::as_str)
        .zip(texts.iter().map(String::as_str))
        .collect();
    let (system, _, _) =
        project_sources(&entry, &files).map_err(|e| JsValue::from_str(&format!("{e:?}")))?;

    let found = modes(&system).map_err(|e| JsValue::from_str(&format!("analysis: {e:?}")))?;
    // Forty time constants of the slowest decay leaves under 1e-17 of the transient; thirty samples
    // per cycle of the fastest mode resolves it without arguing about the integrator's phase error.
    let slowest = found
        .iter()
        .map(|m| m.damping_per_s)
        .fold(f64::INFINITY, f64::min);
    let fastest = found.iter().map(|m| m.frequency_hz).fold(0.0, f64::max);
    let duration = if duration > 0.0 {
        duration
    } else if slowest.is_finite() && slowest > 0.0 {
        40.0 / slowest
    } else {
        1.0
    };
    let step = if step > 0.0 {
        step
    } else if fastest > 0.0 {
        1.0 / (fastest * 30.0)
    } else {
        duration / 2000.0
    };

    let steps = (duration / step).max(1.0) as usize;
    let stride = (steps / samples.clamp(50, 4000)).max(1);
    let trace = run(
        &system,
        &vec![0.0; system.dim()],
        duration,
        step,
        stride,
        Integrator::DiscreteGradient,
    );

    let mut out = String::from("{\n");
    out.push_str(&format!(
        "  \"coordinates\": {}, \"steps\": {}, \"duration_s\": {:e}, \"step_s\": {:e}, \"worst_balance\": {:e},\n",
        system.dim(),
        steps,
        duration,
        step,
        trace.worst_balance
    ));
    out.push_str("  \"modes\": [");
    for (i, mode) in found.iter().enumerate() {
        out.push_str(&format!(
            "{}{{ \"frequency_hz\": {:e}, \"damping_per_s\": {:e}, \"quality\": {:e} }}",
            if i > 0 { ", " } else { "" },
            mode.frequency_hz,
            mode.damping_per_s,
            mode.quality
        ));
    }
    out.push_str("],\n  \"time_s\": [");
    out.push_str(
        &trace
            .times
            .iter()
            .map(|t| format!("{t:.12e}"))
            .collect::<Vec<_>>()
            .join(","),
    );
    out.push_str("],\n  \"readouts\": [");
    for (i, observable) in system.observables.iter().enumerate() {
        let column = &trace.columns[i];
        let settled = column.last().copied().unwrap_or(0.0);
        // From the tracked extremes, not from the decimated column: the true peak of the
        // trajectory, independent of how often a row was written down.
        let peak = trace.peak[i];
        out.push_str(&format!(
            "{}\n    {{ \"name\": \"{}\", \"unit\": \"{}\", \"settled\": {:e}, \"peak\": {:e}, \"series\": [{}] }}",
            if i > 0 { "," } else { "" },
            observable.name,
            observable.unit,
            settled,
            peak,
            column
                .iter()
                .map(|v| format!("{v:.6e}"))
                .collect::<Vec<_>>()
                .join(",")
        ));
    }
    out.push_str("\n  ]\n}\n");
    Ok(out)
}

/// The declared mode geometry, and how much energy each region holds over a run.
///
/// # The split this makes, and why
///
/// What comes back is WHERE the field is and HOW MUCH is there: each region's placement, its
/// dimensionless sample offsets across its own face, and per frame the energy it holds with the sign
/// of its effort. Every one of those is the model's — `Region::sample` is the one implementation of
/// the first, and `Shape::energy` of the second.
///
/// What does NOT come back is how big to draw anything. A plate is a fifth of a metre across with a
/// millimetre between its faces and no view draws that to scale, so the size is the viewer's to
/// choose. That is the same split `Region::sample` already makes by returning offsets in `[-1, 1]`
/// rather than in metres — a boundary that exists because mixing the two was a real bug here.
///
/// A viewer therefore draws an arrow of length proportional to `sign * sqrt(energy / energy_max)`,
/// which is a normalisation of numbers it was handed, not a calculation of its own.
///
/// # The dissipation comes too, and it has to
///
/// A region is a STORE, so a picture built from regions alone shows only where energy sits. On a
/// resistor divider that is exactly the wrong half: every store in the model is a parasitic — the
/// node's capacitance, the branches' loop inductance — and the three components the design is
/// actually about are resistors, which store nothing and appear nowhere. A viewer given only the
/// regions draws three coils and a capacitor, and anyone reading it concludes the circuit has three
/// inductors in it.
///
/// So the losses come back as well: which coordinate each sits on, and the power it is burning at
/// each instant. Where energy LEAVES is as much a fact about this circuit as where it is held, and
/// on this one it is the fact the design was optimised for.
#[wasm_bindgen]
pub fn scene(
    entry: String,
    paths: Vec<String>,
    texts: Vec<String>,
    frames: usize,
) -> Result<String, JsValue> {
    if paths.len() != texts.len() {
        return Err(JsValue::from_str("paths and texts must be the same length"));
    }
    let files: Vec<(&str, &str)> = paths
        .iter()
        .map(String::as_str)
        .zip(texts.iter().map(String::as_str))
        .collect();
    let (system, _, _) =
        project_sources(&entry, &files).map_err(|e| JsValue::from_str(&format!("{e:?}")))?;
    if system.regions.is_empty() {
        return Err(JsValue::from_str(
            "this model declares no mode geometry, so it has no field to draw",
        ));
    }

    let found = modes(&system).map_err(|e| JsValue::from_str(&format!("analysis: {e:?}")))?;
    let slowest = found
        .iter()
        .map(|m| m.damping_per_s)
        .fold(f64::INFINITY, f64::min);
    let fastest = found.iter().map(|m| m.frequency_hz).fold(0.0, f64::max);
    let duration = if slowest.is_finite() && slowest > 0.0 {
        12.0 / slowest
    } else {
        1.0
    };
    let step = if fastest > 0.0 {
        1.0 / (fastest * 40.0)
    } else {
        duration / 4000.0
    };
    let steps = (duration / step).max(1.0) as usize;
    let stride = (steps / frames.clamp(30, 900)).max(1);

    // The state itself, sampled: the energies are a function of it and the viewer wants both.
    let n = system.dim();
    let mut x = vec![0.0; n];
    let mut input = vec![0.0; n];
    let mut effort = vec![0.0; n];
    let mut times = Vec::new();
    let mut energies: Vec<Vec<f64>> = Vec::new();
    let mut signs: Vec<Vec<f64>> = Vec::new();
    let mut powers: Vec<Vec<f64>> = Vec::new();
    let mut currents: Vec<Vec<f64>> = Vec::new();
    let mut potentials: Vec<Vec<Option<f64>>> = Vec::new();
    let mut drops: Vec<Vec<f64>> = Vec::new();
    let mut peak = 0.0_f64;
    let mut peak_power = 0.0_f64;
    let mut peak_current = 0.0_f64;
    let mut worst_loop = 0.0_f64;
    let mut peak_drop = f64::MIN_POSITIVE;

    for k in 0..steps {
        let time = k as f64 * step;
        if k % stride == 0 {
            system.effort(&x, &mut effort);
            let mut row = Vec::with_capacity(system.regions.len());
            let mut sign = Vec::with_capacity(system.regions.len());
            for region in &system.regions {
                let e = effort[region.coordinate];
                let held = region.shape.energy(e);
                peak = peak.max(held);
                row.push(held);
                sign.push(if e < 0.0 { -1.0 } else { 1.0 });
            }
            // `R e^2` on the coordinate the loss is attached to: the power that leaves through it.
            let mut burning = Vec::with_capacity(system.blueprint.losses.len());
            for loss in &system.blueprint.losses {
                let e = effort[loss.coordinate];
                let watts = system.port_hamiltonian.dissipation[loss.coordinate] * e * e;
                peak_power = peak_power.max(watts);
                burning.push(watts);
            }
            // A wire's current IS the effort of the coordinate it names. No separate quantity.
            let mut flowing = Vec::with_capacity(system.wires.len());
            for wire in &system.wires {
                let i = effort[wire.coordinate];
                peak_current = peak_current.max(i.abs());
                flowing.push(i);
            }
            // What a probe would read, everywhere one could be put: the potential of every
            // terminal and the drop across every conductor. The viewer then answers "what is the
            // voltage here" for a point nobody declared a readout at, without computing anything
            // of its own — the alternative being a second, weaker circuit model in JavaScript.
            let here = system.branch_drops(&x, time);
            peak_drop = here.iter().fold(peak_drop, |m, v| m.max(v.abs()));
            if let Some(residual) = system.kirchhoff_residual(&x, time) {
                worst_loop = worst_loop.max(residual);
            }
            potentials.push(system.potentials(&x, time));
            drops.push(here);

            times.push(time);
            energies.push(row);
            signs.push(sign);
            powers.push(burning);
            currents.push(flowing);
        }
        system.input_at(time + 0.5 * step, &mut input);
        system.port_hamiltonian.step(&mut x, &input, step, 1e-14);
    }

    let vector = |v: [f64; 3]| format!("[{:.6},{:.6},{:.6}]", v[0], v[1], v[2]);
    let mut out = String::from("{\n  \"regions\": [");
    for (i, region) in system.regions.iter().enumerate() {
        // One call at unit effort gives the placement of every sample; the magnitude is per frame.
        let samples = region.sample(1.0, 3);
        let (width, depth) = region.span();
        out.push_str(&format!(
            "{}\n    {{ \"name\": \"{}\", \"electric\": {}, \"centre\": {}, \"axis\": {}, \
             \"width_m\": {:e}, \"depth_m\": {:e}, \"capacity\": {:e}, \"offsets\": [{}] }}",
            if i > 0 { "," } else { "" },
            region.name,
            region.shape.is_electric(),
            vector(region.centre),
            vector(region.axis),
            width,
            depth,
            system.coordinates[region.coordinate].capacity,
            samples
                .iter()
                .map(|s| vector(s.offset))
                .collect::<Vec<_>>()
                .join(",")
        ));
    }
    out.push_str(&format!(
        "\n  ],\n  \"energy_max\": {peak:e},\n  \"ringing_hz\": {:e},\n  \"time_s\": [{}],\n",
        found.first().map(|m| m.frequency_hz).unwrap_or(0.0),
        times
            .iter()
            .map(|t| format!("{t:.9e}"))
            .collect::<Vec<_>>()
            .join(",")
    ));
    let table = |rows: &Vec<Vec<f64>>| {
        rows.iter()
            .map(|r| {
                format!(
                    "[{}]",
                    r.iter()
                        .map(|v| format!("{v:.5e}"))
                        .collect::<Vec<_>>()
                        .join(",")
                )
            })
            .collect::<Vec<_>>()
            .join(",")
    };
    // The circuit as the model draws it: terminals, the conductors between them, the resistor on
    // each, and which one is a lamp. Without this a viewer sees only where energy is STORED, and on
    // a resistor network every store is a parasitic while the components are invisible.
    out.push_str("  \"terminals\": [");
    for (i, terminal) in system.terminals.iter().enumerate() {
        out.push_str(&format!(
            "{}{{ \"name\": \"{}\", \"pinned\": {}, \"at\": {} }}",
            if i > 0 { ", " } else { "" },
            terminal.name,
            terminal.pinned,
            vector(terminal.at)
        ));
    }
    out.push_str("],\n  \"wires\": [");
    for (i, wire) in system.wires.iter().enumerate() {
        out.push_str(&format!(
            "{}\n    {{ \"name\": \"{}\", \"from\": {}, \"to\": {}, \"via\": {}, \"resistance\": {:e}, \"emits\": {}, \"supply\": {}, \"region\": {} }}",
            if i > 0 { "," } else { "" },
            wire.name,
            wire.from,
            wire.to,
            wire.via.map(vector).unwrap_or_else(|| "null".to_string()),
            system.port_hamiltonian.dissipation[wire.coordinate],
            system.emitters.contains(&wire.coordinate),
            wire.supply,
            system
                .regions
                .iter()
                .position(|r| r.coordinate == wire.coordinate)
                .map(|k| k.to_string())
                .unwrap_or_else(|| "null".to_string())
        ));
    }
    out.push_str("\n  ],\n  \"losses\": [");
    for (i, loss) in system.blueprint.losses.iter().enumerate() {
        out.push_str(&format!(
            "{}{{ \"name\": \"{}\", \"on\": {}, \"resistance\": {:e} }}",
            if i > 0 { ", " } else { "" },
            loss.name,
            system
                .regions
                .iter()
                .position(|r| r.coordinate == loss.coordinate)
                .map(|k| k.to_string())
                .unwrap_or_else(|| "null".to_string()),
            system.port_hamiltonian.dissipation[loss.coordinate]
        ));
    }
    out.push_str(&format!(
        "],\n  \"power_max\": {peak_power:e},\n  \"current_max\": {peak_current:e},\n"
    ));
    out.push_str(&format!("  \"power\": [{}],\n", table(&powers)));
    out.push_str(&format!("  \"current\": [{}],\n", table(&currents)));
    out.push_str(&format!("  \"energy\": [{}],\n", table(&energies)));
    out.push_str(&format!("  \"sign\": [{}],\n", table(&signs)));

    // Every declared probe, by the terminals it is clipped to rather than by its values: the values
    // are `potential[plus] - potential[minus]`, and exporting them again would be a second copy of a
    // number to keep in step with the first.
    out.push_str("  \"probes\": [");
    let mut written = 0;
    for observable in &system.observables {
        let Readout::Across { plus, minus } = observable.readout else {
            continue;
        };
        out.push_str(&format!(
            "{}{{ \"name\": \"{}\", \"unit\": \"{}\", \"plus\": {plus}, \"minus\": {minus} }}",
            if written > 0 { ", " } else { "" },
            observable.name,
            observable.unit
        ));
        written += 1;
    }
    out.push_str(&format!(
        "],\n  \"reference\": {},\n  \"loop_residual\": {worst_loop:e},\n  \"drop_max\": {peak_drop:e},\n",
        system
            .reference
            .map(|i| i.to_string())
            .unwrap_or_else(|| "null".to_string())
    ));
    out.push_str(&format!("  \"drop\": [{}],\n", table(&drops)));

    // A terminal the conductors do not reach has no potential — not a small one. `null` says so; a
    // zero would read as a measurement.
    out.push_str(&format!(
        "  \"potential\": [{}]\n}}\n",
        potentials
            .iter()
            .map(|r| format!(
                "[{}]",
                r.iter()
                    .map(|v| v.map_or_else(|| "null".to_string(), |x| format!("{x:.5e}")))
                    .collect::<Vec<_>>()
                    .join(",")
            ))
            .collect::<Vec<_>>()
            .join(",")
    ));
    Ok(out)
}
