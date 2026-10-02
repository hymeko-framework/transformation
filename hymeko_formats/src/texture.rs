//! Bounded glTF embedded-image and Radiance decoding via the image crate.
use hymeko_geometry::material::{PbrMaterial, RadianceImage, RgbaImage, TextureMap, TextureWrap};
use std::{collections::HashMap, io::Cursor, sync::Arc};

fn bounded_reader(
    bytes: &[u8],
    format: image::ImageFormat,
    max_height: u32,
) -> Result<image::ImageReader<Cursor<&[u8]>>, String> {
    if bytes.len() > 32 * 1024 * 1024 {
        return Err("image exceeds 32 MiB".into());
    }
    let mut reader = image::ImageReader::with_format(Cursor::new(bytes), format);
    let mut limits = image::Limits::default();
    limits.max_image_width = Some(2048);
    limits.max_image_height = Some(max_height);
    limits.max_alloc = Some(128 * 1024 * 1024);
    reader.limits(limits);
    Ok(reader)
}

/// Decode a bounded Radiance HDR into linear radiance; never clamps away highlights.
pub fn radiance(bytes: &[u8]) -> Result<RadianceImage, String> {
    let image = bounded_reader(bytes, image::ImageFormat::Hdr, 1024)?
        .decode()
        .map_err(|e| e.to_string())?
        .into_rgba32f();
    let result = RadianceImage {
        width: image.width(),
        height: image.height(),
        pixels: image.pixels().map(|p| p.0).collect(),
    };
    result.validate()?;
    Ok(result)
}

#[derive(Default)]
pub(crate) struct MaterialImport {
    images: HashMap<usize, Arc<RgbaImage>>,
    materials: HashMap<Option<usize>, Arc<PbrMaterial>>,
    texels: usize,
}

impl MaterialImport {
    pub(crate) fn read(
        &mut self,
        material: gltf::Material<'_>,
        blob: &[u8],
    ) -> Result<Arc<PbrMaterial>, String> {
        if let Some(value) = self.materials.get(&material.index()) {
            return Ok(value.clone());
        }
        if material.alpha_mode() != gltf::material::AlphaMode::Opaque {
            return Err("GLB PBR currently supports opaque materials only".into());
        }
        let mr = material.pbr_metallic_roughness();
        // KHR_materials_clearcoat and KHR_materials_sheen: their factors, with glTF's defaults
        // for absent keys. Their textures are not read; a textured coat uses its factor alone.
        let factor = |extension: &str, key: &str| {
            material
                .extension_value(extension)
                .and_then(|value| value.get(key))
                .and_then(serde_json::Value::as_f64)
                .map_or(0.0, |value| value as f32)
        };
        let sheen_color = material
            .extension_value("KHR_materials_sheen")
            .and_then(|value| value.get("sheenColorFactor"))
            .and_then(|value| serde_json::from_value::<[f32; 3]>(value.clone()).ok())
            .unwrap_or_default();
        let mut result = PbrMaterial {
            metallic: mr.metallic_factor(),
            roughness: mr.roughness_factor(),
            emissive: material.emissive_factor(),
            double_sided: material.double_sided(),
            clearcoat: factor("KHR_materials_clearcoat", "clearcoatFactor"),
            clearcoat_roughness: factor("KHR_materials_clearcoat", "clearcoatRoughnessFactor"),
            sheen_color,
            sheen_roughness: factor("KHR_materials_sheen", "sheenRoughnessFactor"),
            ..Default::default()
        };
        let slots = [
            mr.base_color_texture()
                .map(|i| (i.texture(), i.tex_coord())),
            mr.metallic_roughness_texture()
                .map(|i| (i.texture(), i.tex_coord())),
            material.normal_texture().map(|i| {
                result.normal_scale = i.scale();
                (i.texture(), i.tex_coord())
            }),
            material.occlusion_texture().map(|i| {
                result.occlusion_strength = i.strength();
                (i.texture(), i.tex_coord())
            }),
            material
                .emissive_texture()
                .map(|i| (i.texture(), i.tex_coord())),
        ];
        for (slot, value) in slots.into_iter().enumerate() {
            if let Some((texture, uv)) = value {
                if uv != 0 {
                    return Err("only TEXCOORD_0 is supported".into());
                }
                result.textures[slot] = Some(self.texture(texture, blob)?);
            }
        }
        result.validate()?;
        let result = Arc::new(result);
        self.materials.insert(material.index(), result.clone());
        Ok(result)
    }

    fn texture(&mut self, texture: gltf::Texture<'_>, blob: &[u8]) -> Result<TextureMap, String> {
        use gltf::texture::{MagFilter, MinFilter, WrappingMode};
        let source = texture.source();
        if !self.images.contains_key(&source.index()) {
            if self.images.len() >= 32 {
                return Err("GLB exceeds 32 decoded images".into());
            }
            let gltf::image::Source::View { view, mime_type } = source.source() else {
                return Err("GLB textures must be embedded buffer views".into());
            };
            let format = match mime_type {
                "image/png" => image::ImageFormat::Png,
                "image/jpeg" => image::ImageFormat::Jpeg,
                _ => return Err("only PNG/JPEG textures are supported".into()),
            };
            let end = view
                .offset()
                .checked_add(view.length())
                .ok_or("image range overflow")?;
            let bytes = blob
                .get(view.offset()..end)
                .ok_or("image buffer outside GLB")?;
            // Check dimensions before decoding so the cumulative budget also precedes allocation.
            let dimensions = bounded_reader(bytes, format, 2048)?
                .into_dimensions()
                .map_err(|e| e.to_string())?;
            let texels = (dimensions.0 as u64) * (dimensions.1 as u64);
            if texels > (16_777_216 - self.texels) as u64 {
                return Err("GLB exceeds decoded texture budget".into());
            }
            let image = bounded_reader(bytes, format, 2048)?
                .decode()
                .map_err(|e| e.to_string())?
                .into_rgba8();
            self.texels += texels as usize;
            self.images.insert(
                source.index(),
                Arc::new(RgbaImage {
                    width: image.width(),
                    height: image.height(),
                    pixels: image.into_raw(),
                }),
            );
        }
        let sampler = texture.sampler();
        let wrap = |mode| match mode {
            WrappingMode::ClampToEdge => TextureWrap::Clamp,
            WrappingMode::MirroredRepeat => TextureWrap::Mirror,
            WrappingMode::Repeat => TextureWrap::Repeat,
        };
        let min = sampler
            .min_filter()
            .unwrap_or(MinFilter::LinearMipmapLinear);
        Ok(TextureMap {
            image: self.images[&source.index()].clone(),
            wrap: [wrap(sampler.wrap_s()), wrap(sampler.wrap_t())],
            mag_linear: sampler.mag_filter() != Some(MagFilter::Nearest),
            min_linear: matches!(
                min,
                MinFilter::Linear | MinFilter::LinearMipmapNearest | MinFilter::LinearMipmapLinear
            ),
            mip_linear: matches!(
                min,
                MinFilter::NearestMipmapLinear | MinFilter::LinearMipmapLinear
            ),
            mipmapped: !matches!(min, MinFilter::Nearest | MinFilter::Linear),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::ImageEncoder;
    fn fixture() -> (gltf::Document, Vec<u8>) {
        let mut bytes = Vec::new();
        image::codecs::png::PngEncoder::new(&mut bytes)
            .write_image(&[31, 127, 239, 255], 1, 1, image::ExtendedColorType::Rgba8)
            .unwrap();
        let document = serde_json::json!({
            "asset":{"version":"2.0"}, "buffers":[{"byteLength":bytes.len()}],
            "bufferViews":[{"buffer":0,"byteLength":bytes.len()}],
            "images":[{"bufferView":0,"mimeType":"image/png"}],
            "samplers":[{"wrapS":33071,"wrapT":33648,"magFilter":9728,"minFilter":9986}],
            "textures":[{"source":0,"sampler":0}],
            "materials":[{"pbrMetallicRoughness":{"metallicFactor":0.25,"roughnessFactor":0.75,"baseColorTexture":{"index":0},"metallicRoughnessTexture":{"index":0}},
                "normalTexture":{"index":0,"scale":0.5},"occlusionTexture":{"index":0,"strength":0.4},"emissiveTexture":{"index":0},"emissiveFactor":[0.1,0.2,0.3],"doubleSided":true}]
        });
        (
            gltf::Gltf::from_slice(&serde_json::to_vec(&document).unwrap())
                .unwrap()
                .document,
            bytes,
        )
    }
    #[test]
    fn channels_factors_samplers_and_shared_image_identity_survive_import() {
        let (document, bytes) = fixture();
        let mut importer = MaterialImport::default();
        let material = importer
            .read(document.materials().next().unwrap(), &bytes)
            .unwrap();
        assert_eq!(
            (
                material.metallic,
                material.roughness,
                material.normal_scale,
                material.occlusion_strength
            ),
            (0.25, 0.75, 0.5, 0.4)
        );
        assert_eq!(material.emissive, [0.1, 0.2, 0.3]);
        assert!(material.double_sided);
        let first = material.textures[0].as_ref().unwrap();
        assert_eq!(first.image.pixels, [31, 127, 239, 255]);
        assert!(matches!(
            first.wrap,
            [TextureWrap::Clamp, TextureWrap::Mirror]
        ));
        assert!(!first.mag_linear && !first.min_linear && first.mip_linear && first.mipmapped);
        for slot in material.textures.iter().flatten() {
            assert!(Arc::ptr_eq(&first.image, &slot.image));
        }
        assert!(Arc::ptr_eq(
            &material,
            &importer
                .read(document.materials().next().unwrap(), &bytes)
                .unwrap()
        ));
        assert!(
            MaterialImport::default()
                .read(document.materials().next().unwrap(), &bytes[..3])
                .is_err()
        );
    }
    #[test]
    fn clearcoat_and_sheen_factors_are_read_and_absent_ones_stay_zero() {
        let document = serde_json::json!({
            "asset": {"version": "2.0"},
            "extensionsUsed": ["KHR_materials_clearcoat", "KHR_materials_sheen"],
            "materials": [
                {"extensions": {
                    "KHR_materials_clearcoat": {"clearcoatFactor": 0.8, "clearcoatRoughnessFactor": 0.15},
                    "KHR_materials_sheen": {"sheenColorFactor": [0.6, 0.5, 0.4], "sheenRoughnessFactor": 0.7}}},
                {"extensions": {"KHR_materials_clearcoat": {}}},
                {}
            ]
        });
        let document = gltf::Gltf::from_slice(&serde_json::to_vec(&document).unwrap())
            .unwrap()
            .document;
        let mut importer = MaterialImport::default();
        let read: Vec<_> = (document.materials())
            .map(|m| importer.read(m, &[]).unwrap())
            .collect();
        let lobes = |m: &PbrMaterial| {
            (m.clearcoat, m.clearcoat_roughness, m.sheen_color, m.sheen_roughness)
        };
        assert_eq!(lobes(&read[0]), (0.8, 0.15, [0.6, 0.5, 0.4], 0.7));
        assert_eq!(lobes(&read[1]), (0.0, 0.0, [0.0; 3], 0.0));
        assert_eq!(lobes(&read[2]), (0.0, 0.0, [0.0; 3], 0.0));
    }

    #[test]
    fn hdr_preserves_radiance_above_one_and_rejects_truncated_payload() {
        let mut bytes = b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 1 +X 1\n".to_vec();
        bytes.extend([128, 64, 32, 131]);
        let hdr = radiance(&bytes).unwrap();
        assert_eq!(hdr.pixels[0], [4., 2., 1., 1.]);
        bytes.pop();
        assert!(radiance(&bytes).is_err());
        assert!(radiance(b"not an HDR").is_err());
    }
}
