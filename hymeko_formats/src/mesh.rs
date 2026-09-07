//! Bounded self-contained GLB import through gltf-rs. No language parser here.
use hymeko_geometry::mesh::{MeshLod, TriangleMesh};
use std::{collections::HashMap, sync::Arc};

pub struct MeshPart {
    pub name: String,
    pub mesh: Arc<TriangleMesh>,
    pub transforms: Vec<[[f32; 4]; 4]>,
    pub color: [f32; 4],
    pub material: Arc<hymeko_geometry::material::PbrMaterial>,
}

pub struct MeshImport {
    pub parts: Vec<MeshPart>,
    /// Skinning/animation is intentionally not evaluated: the imported default geometry is static.
    pub has_animation: bool,
}

struct ImportGeometry<'a> {
    blob: &'a [u8],
    levels: usize,
    error: f32,
    meshes: HashMap<(usize, usize), Arc<TriangleMesh>>,
    budget: [usize; 2],
}

impl ImportGeometry<'_> {
    fn get(
        &mut self,
        key: (usize, usize),
        p: &gltf::Primitive<'_>,
    ) -> Result<Arc<TriangleMesh>, String> {
        if let Some(mesh) = self.meshes.get(&key) {
            return Ok(mesh.clone());
        }
        let vertices = p
            .get(&gltf::Semantic::Positions)
            .ok_or("missing positions")?
            .count();
        let indices = p.indices().map_or(vertices, |a| a.count());
        for (used, (count, limit)) in self
            .budget
            .iter_mut()
            .zip([(vertices, 1_000_000), (indices, 6_000_000)])
        {
            if count > limit - *used {
                return Err("GLB exceeds scene geometry budget".into());
            }
            *used += count;
        }
        let mesh = Arc::new(MeshImport::primitive(
            p,
            self.blob,
            self.levels,
            self.error,
        )?);
        self.meshes.insert(key, mesh.clone());
        Ok(mesh)
    }
}

impl MeshImport {
    pub fn glb(bytes: &[u8], levels: usize, error: f32) -> Result<Self, String> {
        if bytes.len() > 64 * 1024 * 1024 {
            return Err("GLB exceeds 64 MiB".into());
        }
        let file = gltf::Gltf::from_slice(bytes).map_err(|e| e.to_string())?;
        if file.nodes().len() > 4096 {
            return Err("GLB exceeds node budget".into());
        }
        if file.document.extensions_required().next().is_some() {
            return Err("required glTF extensions are not supported".into());
        }
        let blob = file.blob.as_deref().ok_or("expected a binary GLB buffer")?;
        if file
            .buffers()
            .any(|b| !matches!(b.source(), gltf::buffer::Source::Bin) || b.length() > blob.len())
        {
            return Err("GLB must contain its complete geometry buffer".into());
        }
        let scene = file
            .default_scene()
            .or_else(|| file.scenes().next())
            .ok_or("GLB has no scene")?;
        let mut out = Self {
            parts: Vec::new(),
            has_animation: file.animations().next().is_some() || file.skins().next().is_some(),
        };
        let mut cache = ImportGeometry {
            blob,
            levels,
            error,
            meshes: HashMap::new(),
            budget: [0; 2],
        };
        let mut visited = 0;
        let mut materials = crate::texture::MaterialImport::default();
        let mut stack: Vec<_> = scene.nodes().map(|node| (node, Vec::new())).collect();
        while let Some((node, mut transforms)) = stack.pop() {
            visited += 1;
            if visited > 4096 || transforms.len() >= 64 || out.parts.len() >= 1024 {
                return Err("GLB scene exceeds node/depth budget".into());
            }
            transforms.push(node.transform().matrix());
            for child in node.children() {
                stack.push((child, transforms.clone()));
            }
            if let Some(mesh) = node.mesh() {
                for primitive in mesh.primitives() {
                    if out.parts.len() >= 1024 {
                        return Err("GLB exceeds 1024 mesh instances".into());
                    }
                    let key = (mesh.index(), primitive.index());
                    let data = cache.get(key, &primitive)?;
                    let material = materials.read(primitive.material(), blob)?;
                    if material.textures.iter().any(Option::is_some) && data.uv.is_empty() {
                        return Err("textured material requires TEXCOORD_0".into());
                    }
                    out.parts.push(MeshPart {
                        material,
                        name: node.name().unwrap_or("mesh").to_owned(),
                        mesh: data,
                        transforms: transforms.clone(),
                        color: primitive
                            .material()
                            .pbr_metallic_roughness()
                            .base_color_factor(),
                    });
                }
            }
        }
        if out.parts.is_empty() {
            return Err("GLB contains no triangle geometry".into());
        }
        Ok(out)
    }

    fn primitive(
        p: &gltf::Primitive<'_>,
        blob: &[u8],
        levels: usize,
        error: f32,
    ) -> Result<TriangleMesh, String> {
        if p.mode() != gltf::mesh::Mode::Triangles {
            return Err("only triangle primitives are supported".into());
        }
        let count = p
            .get(&gltf::Semantic::Positions)
            .ok_or("missing positions")?
            .count();
        if p.get(&gltf::Semantic::Normals)
            .ok_or("missing normals")?
            .count()
            != count
        {
            return Err("normal and position counts differ".into());
        }
        if count > 1_000_000 || p.indices().is_some_and(|a| a.count() > 6_000_000) {
            return Err("primitive exceeds geometry budget".into());
        }
        let reader = p.reader(|_| Some(blob));
        let positions: Vec<_> = reader
            .read_positions()
            .ok_or("missing positions")?
            .collect();
        let normals: Vec<_> = reader
            .read_normals()
            .ok_or("GLB must include vertex normals")?
            .collect();
        let indices = reader
            .read_indices()
            .map(|r| r.into_u32().collect())
            .unwrap_or_else(|| (0..positions.len() as u32).collect());
        let mut mesh = TriangleMesh {
            uv: reader
                .read_tex_coords(0)
                .map(|r| r.into_f32().collect())
                .unwrap_or_default(),
            tangents: reader
                .read_tangents()
                .map(Iterator::collect)
                .unwrap_or_default(),
            positions,
            normals,
            lods: vec![MeshLod {
                indices,
                error: 0.0,
            }],
        };
        mesh.optimize(levels, error)?;
        Ok(mesh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(uri: Option<&str>) -> Vec<u8> {
        let mut document = serde_json::json!({
            "asset":{"version":"2.0"},"scene":0,"scenes":[{"nodes":[0,1]}],
            "nodes":[{"mesh":0,"translation":[1,2,3]},{"mesh":0,"scale":[-2,1,1]}],
            "meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1}}]}],
            "buffers":[{"byteLength":72}],
            "bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":36}],
            "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]},
                {"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"}]
        });
        if let Some(uri) = uri {
            document["buffers"][0]["uri"] = uri.into();
        }
        let mut json = serde_json::to_vec(&document).unwrap();
        while !json.len().is_multiple_of(4) {
            json.push(b' ');
        }
        let bin: Vec<u8> = [
            0f32, 0., 0., 1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 1., 0., 0., 1.,
        ]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect();
        let mut bytes = Vec::new();
        for value in [
            0x46546c67,
            2,
            (12 + 8 + json.len() + 8 + bin.len()) as u32,
            json.len() as u32,
            0x4e4f534a,
        ] {
            bytes.extend(value.to_le_bytes());
        }
        bytes.extend(json);
        bytes.extend((bin.len() as u32).to_le_bytes());
        bytes.extend(0x004e4942u32.to_le_bytes());
        bytes.extend(bin);
        bytes
    }

    #[test]
    fn glb_uses_shared_validated_geometry_and_preserves_node_transforms() {
        let imported = MeshImport::glb(&fixture(None), 4, 0.02).unwrap();
        assert_eq!(imported.parts.len(), 2);
        assert!(Arc::ptr_eq(
            &imported.parts[0].mesh,
            &imported.parts[1].mesh
        ));
        assert_eq!(imported.parts[0].mesh.positions.len(), 3);
        assert_ne!(imported.parts[0].transforms, imported.parts[1].transforms);
        assert!(!imported.has_animation);
        assert!(MeshImport::glb(&fixture(Some("missing.bin")), 4, 0.02).is_err());
        assert!(MeshImport::glb(b"not GLB", 4, 0.02).is_err());
        assert!(MeshImport::glb(&fixture(None), 0, 0.02).is_err());
    }
}
