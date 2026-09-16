//! `hymeko-dynamics` — compile a declared port-Hamiltonian system, analyse it, run it, emit it.
//!
//! ```text
//!   hymeko-dynamics <model.hymeko> [--modes] [--sweep] [--trace <seconds> <step>] [--json <file>]
//! ```
//!
//! There is no flag for a coupling, a frequency or a range. What is varied and over what is a
//! DECLARED study — `ph.studies.sweep` in the model — for the same reason the design is: a command
//! line that could override it would be a second place to say what the investigation is.

use std::path::PathBuf;
use std::process::ExitCode;

use hymeko::module_store::module_store::ModuleStore;
use hymeko::module_store::source_provider::StdFsProvider;
use hymeko::util::real_parser::RealParser;
use hymeko_dynamics::System;
use hymeko_dynamics::analysis::{modes, response};
use hymeko_dynamics::run::{Integrator, run};

/// How far outside the modes a response curve is sampled, as a fraction either side.
const MARGIN: (f64, f64) = (0.75, 1.3);
const SAMPLES: usize = 240;

fn main() -> ExitCode {
    let mut arguments = std::env::args().skip(1);
    let mut model: Option<PathBuf> = None;
    let (mut want_modes, mut want_sweep) = (false, false);
    let mut trace: Option<(f64, f64)> = None;
    let mut json: Option<PathBuf> = None;

    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--modes" => want_modes = true,
            "--sweep" => want_sweep = true,
            "--trace" => {
                let seconds = arguments.next().and_then(|v| v.parse().ok());
                let step = arguments.next().and_then(|v| v.parse().ok());
                match (seconds, step) {
                    (Some(s), Some(h)) => trace = Some((s, h)),
                    _ => {
                        eprintln!("--trace takes a duration and a step, both in seconds");
                        return ExitCode::FAILURE;
                    }
                }
            }
            "--json" => json = arguments.next().map(PathBuf::from),
            other if model.is_none() => model = Some(PathBuf::from(other)),
            other => {
                eprintln!("unexpected argument: {other}");
                return ExitCode::FAILURE;
            }
        }
    }
    let Some(model) = model else {
        eprintln!(
            "usage: hymeko-dynamics <model.hymeko> [--modes] [--sweep] [--trace <s> <h>] [--json <file>]"
        );
        return ExitCode::FAILURE;
    };
    if !want_modes && !want_sweep && trace.is_none() && json.is_none() {
        want_modes = true;
    }

    let mut store = ModuleStore::new(StdFsProvider::new(), RealParser);
    let program = match store.compile(&model) {
        Ok(program) => program,
        Err(error) => {
            eprintln!("{}: {error:?}", model.display());
            return ExitCode::FAILURE;
        }
    };
    let system = match hymeko_dynamics::project(&program.ir.clone(), &store.it) {
        Ok(system) => system,
        Err(error) => {
            eprintln!("projection: {error:?}");
            return ExitCode::FAILURE;
        }
    };

    eprintln!(
        "{}: {} coordinates, {} ports, {} losses, {} drives, {} readouts, {} declared studies",
        system.name,
        system.dim(),
        system.blueprint.ports.len(),
        system.blueprint.losses.len(),
        system.drives.len(),
        system.observables.len(),
        system.sweeps.len()
    );

    let declared_modes = match modes(&system) {
        Ok(m) => m,
        Err(error) => {
            eprintln!("analysis: {error:?}");
            return ExitCode::FAILURE;
        }
    };

    if want_modes {
        println!("modes");
        println!(
            "  {:>14}  {:>16}  {:>10}",
            "frequency Hz", "damping 1/s", "Q"
        );
        for mode in &declared_modes {
            println!(
                "  {:>14.3}  {:>16.3}  {:>10.2}",
                mode.frequency_hz, mode.damping_per_s, mode.quality
            );
        }
        println!("coordinates");
        for coordinate in &system.coordinates {
            println!(
                "  {:<40} state in {:<9} capacity {:>12.6e}",
                coordinate.name, coordinate.unit, coordinate.capacity
            );
        }
    }

    if want_sweep {
        for sweep in &system.sweeps {
            println!("sweep {} over {}", sweep.name, sweep.parameter_name);
            println!(
                "  {:>10}  {:>14}  {:>14}  {:>14}",
                "value", "mode 1 Hz", "mode 2 Hz", "product"
            );
            for point in sweep.points() {
                let at = system.at(sweep.parameter, point);
                match modes(&at) {
                    Ok(found) if found.len() >= 2 => println!(
                        "  {point:>10.4}  {:>14.3}  {:>14.3}  {:>14.6e}",
                        found[0].frequency_hz,
                        found[1].frequency_hz,
                        found[0].frequency_hz * found[1].frequency_hz
                    ),
                    Ok(found) => println!("  {point:>10.4}  {} mode(s)", found.len()),
                    Err(error) => println!("  {point:>10.4}  {error:?}"),
                }
            }
        }
    }

    if let Some((duration, step)) = trace {
        let stride = ((duration / step) as usize / 2000).max(1);
        let recorded = run(
            &system,
            &vec![0.0; system.dim()],
            duration,
            step,
            stride,
            Integrator::DiscreteGradient,
        );
        eprintln!(
            "trace: {} samples, worst balance residual {:e}",
            recorded.times.len(),
            recorded.worst_balance
        );
        print!("time");
        for observable in &system.observables {
            print!(",{}", observable.name);
        }
        println!();
        for k in 0..recorded.times.len() {
            print!("{:e}", recorded.times[k]);
            for column in &recorded.columns {
                print!(",{:e}", column[k]);
            }
            println!();
        }
    }

    if let Some(path) = json {
        let text = dataset(&system, &declared_modes);
        if let Err(error) = std::fs::write(&path, text) {
            eprintln!("{}: {error}", path.display());
            return ExitCode::FAILURE;
        }
        eprintln!("dataset written to {}", path.display());
    }
    ExitCode::SUCCESS
}

/// Everything a picture of this system needs, as JSON.
///
/// The response surface is the interesting one: for every value of the declared sweep, the
/// steady-state amplitude of one readout across a band. Two ridges that repel rather than cross is
/// what a coupled pair does, and the analysis produces it without running a single trajectory.
fn dataset(system: &System, declared: &[hymeko_dynamics::analysis::Mode]) -> String {
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(&format!("  \"system\": \"{}\",\n", system.name));

    out.push_str("  \"coordinates\": [");
    for (i, coordinate) in system.coordinates.iter().enumerate() {
        out.push_str(&format!(
            "{}\n    {{ \"decl\": {}, \"name\": \"{}\", \"unit\": \"{}\", \"capacity\": {:e} }}",
            if i > 0 { "," } else { "" },
            coordinate.decl.raw(),
            coordinate.name,
            coordinate.unit,
            coordinate.capacity
        ));
    }
    out.push_str("\n  ],\n  \"ports\": [");
    for (i, port) in system.blueprint.ports.iter().enumerate() {
        let weight = system.port_hamiltonian.interconnection.dense()
            [port.source * system.dim() + port.target];
        out.push_str(&format!(
            "{}\n    {{ \"name\": \"{}\", \"source\": {}, \"target\": {}, \"weight\": {:e} }}",
            if i > 0 { "," } else { "" },
            port.name,
            port.source,
            port.target,
            weight
        ));
    }
    out.push_str("\n  ],\n  \"dissipation\": [");
    out.push_str(
        &system
            .port_hamiltonian
            .dissipation
            .iter()
            .map(|v| format!("{v:e}"))
            .collect::<Vec<_>>()
            .join(", "),
    );
    out.push_str("],\n  \"modes\": [");
    for (i, mode) in declared.iter().enumerate() {
        out.push_str(&format!(
            "{}\n    {{ \"frequency_hz\": {:e}, \"damping_per_s\": {:e}, \"quality\": {:e} }}",
            if i > 0 { "," } else { "" },
            mode.frequency_hz,
            mode.damping_per_s,
            mode.quality
        ));
    }
    out.push_str("\n  ],\n");

    // The frequency grid frames the WHOLE study, not just the declared operating point: the sweep
    // moves the modes, and a grid fixed to where they start would run off the picture by the time
    // the coupling reaches one. So the sweep's modes are collected first and the grid derived from
    // their extremes.
    let mut lowest = f64::INFINITY;
    let mut highest: f64 = 0.0;
    let mut swept: Vec<(f64, Vec<f64>, System)> = Vec::new();
    for sweep in &system.sweeps {
        for point in sweep.points() {
            let at = system.at(sweep.parameter, point);
            let found: Vec<f64> = modes(&at)
                .unwrap_or_default()
                .iter()
                .map(|m| m.frequency_hz)
                .collect();
            for f in &found {
                lowest = lowest.min(*f);
                highest = highest.max(*f);
            }
            swept.push((point, found, at));
        }
    }
    for mode in declared {
        lowest = lowest.min(mode.frequency_hz);
        highest = highest.max(mode.frequency_hz);
    }
    if !lowest.is_finite() {
        lowest = 1000.0;
        highest = 10000.0;
    }
    let (from, to) = (lowest * MARGIN.0, highest * MARGIN.1);
    let grid: Vec<f64> = (0..SAMPLES)
        .map(|i| from + (to - from) * i as f64 / (SAMPLES - 1) as f64)
        .collect();
    out.push_str(&format!(
        "  \"frequencies_hz\": [{}],\n",
        grid.iter()
            .map(|f| format!("{f:.2}"))
            .collect::<Vec<_>>()
            .join(", ")
    ));

    let observable = system.observable_of("capacitor_voltage").unwrap_or(0);
    out.push_str(&format!(
        "  \"observable\": \"{}\",\n",
        system.observables[observable].name
    ));

    // One response curve per sweep point, plus the modes there.
    out.push_str("  \"sweeps\": [");
    for (s, sweep) in system.sweeps.iter().enumerate() {
        out.push_str(&format!(
            "{}\n    {{ \"name\": \"{}\", \"over\": \"{}\", \"points\": [",
            if s > 0 { "," } else { "" },
            sweep.name,
            sweep.parameter_name
        ));
        for (p, point) in sweep.points().iter().enumerate() {
            let at = system.at(sweep.parameter, *point);
            let found = modes(&at).unwrap_or_default();
            let curve: Vec<String> = grid
                .iter()
                .map(|f| {
                    response(&at, 0, observable, std::f64::consts::TAU * f)
                        .map(|c| format!("{:.6e}", c.abs()))
                        .unwrap_or_else(|_| "0".into())
                })
                .collect();
            out.push_str(&format!(
                "{}\n      {{ \"value\": {point:e}, \"modes\": [{}], \"response\": [{}] }}",
                if p > 0 { "," } else { "" },
                found
                    .iter()
                    .map(|m| format!("{:.4}", m.frequency_hz))
                    .collect::<Vec<_>>()
                    .join(", "),
                curve.join(", ")
            ));
        }
        out.push_str("\n    ] }");
    }
    out.push_str("\n  ],\n");

    // A transient at the declared drive, for the trace panel.
    let duration = 60.0 / lowest;
    let step = 1.0 / (highest * 900.0);
    let recorded = run(
        system,
        &vec![0.0; system.dim()],
        duration,
        step,
        ((duration / step) as usize / 1200).max(1),
        Integrator::DiscreteGradient,
    );
    out.push_str(&format!(
        "  \"trace\": {{ \"drive_hz\": {:e}, \"worst_balance\": {:e},\n    \"time\": [{}],\n",
        system.drives.first().map(|d| d.frequency_hz).unwrap_or(0.0),
        recorded.worst_balance,
        recorded
            .times
            .iter()
            .map(|t| format!("{t:.9}"))
            .collect::<Vec<_>>()
            .join(", ")
    ));
    for (i, observable) in system.observables.iter().enumerate() {
        out.push_str(&format!(
            "    \"{}\": [{}]{}\n",
            observable
                .name
                .rsplit('.')
                .next()
                .unwrap_or(&observable.name),
            recorded.columns[i]
                .iter()
                .map(|v| format!("{v:.6e}"))
                .collect::<Vec<_>>()
                .join(", "),
            if i + 1 < system.observables.len() {
                ","
            } else {
                ""
            }
        ));
    }
    out.push_str("  }\n}\n");
    out
}
