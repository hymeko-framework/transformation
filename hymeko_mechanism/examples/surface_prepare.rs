//! Thin JSON authoring adapter for the native surface regularizer.
#[cfg(feature = "authoring")]
#[path = "surface_prepare/pipeline.rs"]
mod pipeline;
use hymeko_mechanism::mesh::{
    TriangleMesh,
    bridge::BoundaryBridge,
    fair::{FairOptions, SurfaceFairing},
    normals::NormalSmoothing,
    regularize::{RegularizeOptions, SurfaceRegularizer},
    solid::{SurfaceUnion, UnionOptions},
    subdivision::SubdivisionSurface,
};
use serde::Deserialize;
use std::{
    fs::File,
    io::{BufReader, BufWriter, Write},
    path::Path,
};

#[derive(Deserialize)]
struct Request {
    mesh: TriangleMesh,
    #[serde(default)]
    options: RegularizeOptions,
    #[serde(default)]
    control_lod: usize,
    #[serde(default)]
    fair_options: FairOptions,
    #[serde(default)]
    influence: Option<Vec<f64>>,
    #[serde(default)]
    targets: Option<Vec<[f32; 3]>>,
    #[serde(default)]
    fixed_patch_boundary: bool,
    #[serde(default)]
    bridge: Option<BoundaryBridge>,
    #[serde(default)]
    addition: Option<TriangleMesh>,
    #[serde(default)]
    union_options: UnionOptions,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args
        .first()
        .is_some_and(|v| matches!(v.as_str(), "recipe" | "pipeline"))
    {
        #[cfg(feature = "authoring")]
        return pipeline::run(&args);
        #[cfg(not(feature = "authoring"))]
        return Err("recipe execution needs the authoring feature".into());
    }
    if args.len() != 3
        || !matches!(
            args[0].as_str(),
            "quads" | "normals" | "follow" | "fair" | "bridge" | "limit" | "union"
        )
    {
        return Err(
            "usage: surface_prepare quads|normals|follow|fair|bridge|limit|union INPUT.json OUTPUT.json"
                .into(),
        );
    }
    let input = Path::new(&args[1]);
    if input.metadata()?.len() > 64 * 1024 * 1024 {
        return Err("input exceeds64MiB".into());
    }
    let mut request: Request = serde_json::from_reader(BufReader::new(File::open(input)?))?;
    if request.control_lod > 3 {
        return Err("control_lod must be0..3".into());
    }
    if args[0] == "fair" {
        let report = if let Some(targets) = &request.targets {
            SurfaceFairing::fit_targets(
                &mut request.mesh,
                targets,
                &request.fair_options,
                request.influence.as_deref(),
            )?
        } else {
            SurfaceFairing::apply_weighted(
                &mut request.mesh,
                &request.fair_options,
                request.influence.as_deref(),
            )?
        };
        let mut writer = BufWriter::new(File::create(&args[2])?);
        serde_json::to_writer(
            &mut writer,
            &serde_json::json!({"mesh":request.mesh,"fairing":report}),
        )?;
        writer.flush()?;
        return Ok(());
    }
    request.mesh.validate()?;
    if args[0] == "union" {
        let addition = request.addition.ok_or("convex union addition required")?;
        eprintln!("surface union: validating and intersecting two closed boundaries");
        let result = SurfaceUnion::with_convex(&request.mesh, &addition, &request.union_options)?;
        eprintln!(
            "surface union: {} intersection segments, {} output triangles",
            result.intersection_segments,
            result.faces.len()
        );
        let mut writer = BufWriter::new(File::create(&args[2])?);
        serde_json::to_writer(&mut writer, &result)?;
        writer.flush()?;
        return Ok(());
    }
    if args[0] == "bridge" {
        let indices = request
            .bridge
            .ok_or("bridge settings required")?
            .indices(&request.mesh)?;
        let mut writer = BufWriter::new(File::create(&args[2])?);
        serde_json::to_writer(&mut writer, &serde_json::json!({"indices":indices}))?;
        writer.flush()?;
        return Ok(());
    }
    if matches!(args[0].as_str(), "normals" | "follow") {
        if args[0] == "follow" {
            let reference = request.mesh.clone();
            request.mesh.positions = request
                .targets
                .ok_or("normal following requires target positions")?;
            if request.mesh.positions != reference.positions {
                request.mesh.lods.truncate(1);
                request.mesh.tangents.clear();
            }
            NormalSmoothing::follow_geometry(
                &reference,
                &mut request.mesh,
                request.options.normal_steps,
                request.options.crease_degrees,
            )?;
        } else {
            NormalSmoothing::apply(
                &mut request.mesh,
                request.options.normal_steps,
                request.options.crease_degrees,
            )?;
        }
        let output = serde_json::json!({"mesh":request.mesh});
        let mut writer = BufWriter::new(File::create(&args[2])?);
        serde_json::to_writer(&mut writer, &output)?;
        writer.flush()?;
        eprintln!("surface normal field updated");
        return Ok(());
    }
    let mut control = request.mesh.clone();
    if request.control_lod > 0 {
        #[cfg(feature = "mesh-optimization")]
        control.optimize(request.control_lod + 1, 0.005)?;
        #[cfg(not(feature = "mesh-optimization"))]
        return Err("control reduction requires the existing mesh-optimization feature".into());
    }
    let indices = &control
        .lods
        .last()
        .ok_or("missing control surface")?
        .indices;
    eprintln!(
        "surface preparation: {} source triangles, {} control triangles",
        request.mesh.lods[0].indices.len() / 3,
        indices.len() / 3
    );
    let output = SurfaceRegularizer::new(&request.mesh)?.prepare(indices, &request.options)?;
    eprintln!(
        "surface preparation: {} quads, {} pinned vertices, maximum relaxation {}",
        output.quads.len(),
        output.pinned_vertices,
        output.maximum_relaxation
    );
    let mut writer = BufWriter::new(File::create(&args[2])?);
    if args[0] == "limit" {
        let surface = SubdivisionSurface::new(&output.positions, &output.quads)?;
        let positions = if request.fixed_patch_boundary {
            surface.limit_positions_with_fixed_boundary()?
        } else {
            surface.limit_positions()?
        };
        let targets: Vec<_> = output
            .source_vertices
            .iter()
            .enumerate()
            .map(|(i, v)| v.map_or(request.mesh.positions[i], |v| positions[v as usize]))
            .collect();
        let fitting = SurfaceFairing::fit_targets(
            &mut request.mesh,
            &targets,
            &request.fair_options,
            request.influence.as_deref(),
        )?;
        serde_json::to_writer(
            &mut writer,
            &serde_json::json!({"control":output,"limit_positions":positions,"mesh":request.mesh,"fitting":fitting}),
        )?;
    } else {
        serde_json::to_writer(&mut writer, &output)?;
    }
    writer.flush()?;
    Ok(())
}
