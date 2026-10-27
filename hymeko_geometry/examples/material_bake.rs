//! Offline export of the same library/recipes consumed by renderer and simulation hosts.
use hymeko_geometry::material::library::MaterialLibrary;
use std::{io::Write, path::PathBuf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let directory = PathBuf::from(
        args.next()
            .ok_or("usage: material_bake DIRECTORY [SIZE=512]")?,
    );
    let size: u32 = args.next().unwrap_or_else(|| "512".into()).parse()?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    std::fs::create_dir_all(&directory)?;
    let library = MaterialLibrary::builtin();
    for (id, material) in &library.materials {
        let Some(recipe) = &material.procedural else {
            continue;
        };
        let baked = recipe.bake(
            size,
            material.tile_m,
            material.appearance.roughness,
            material.appearance.metallic,
        )?;
        for (suffix, image) in [
            ("color", &baked.color),
            ("roughness", &baked.metal_rough),
            ("normal", &baked.normal),
        ] {
            image::save_buffer(
                directory.join(format!("{id}-{suffix}.png")),
                &image.pixels,
                size,
                size,
                image::ColorType::Rgba8,
            )?;
        }
        // Raw little-endian f32 metres retain signed relief without display normalization.
        let mut height = std::io::BufWriter::new(std::fs::File::create(
            directory.join(format!("{id}-height.f32le")),
        )?);
        for value in &baked.height_m {
            height.write_all(&value.to_le_bytes())?;
        }
        height.flush()?;
    }
    let manifest = serde_json::json!({
        "library": library, "size": size,
        "height": "row-major little-endian float32 metres, texel centres, positive V follows row index",
        "color": "sRGB RGBA8", "roughness": "linear RGBA8: R=1, G=roughness, B=metallic",
        "normal": "linear tangent-space RGBA8, +Y follows increasing V"
    });
    std::fs::write(
        directory.join("materials-baked.json"),
        serde_json::to_vec_pretty(&manifest)?,
    )?;
    println!(
        "Exported {} procedural materials at {size}x{size} to {}",
        library
            .materials
            .values()
            .filter(|m| m.procedural.is_some())
            .count(),
        directory.display()
    );
    Ok(())
}
