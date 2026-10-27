HYMEKO SHARED MATERIAL LIBRARY

Canonical source: materials.json, schema_version 1.
Owner: hymeko_geometry::material::{library,procedural}.
Available without simulation/default features; no renderer or Core dependency.

Eight entries: seven procedural surfaces (oak, plaster, rug, metal, brass,
ceramic, shade) and the accepted avatar-atlas fallback PBR preset. These are
authored visual approximations, not measured physical specimens. The avatar's
accepted albedo/normal/MR images remain in its asset pack; no regeneration is
required. Hair retains its existing specialized optical model.

Use
  let library = MaterialLibrary::builtin();
  let definition = &library.materials["oak"];
  let appearance = definition.bake(512)?;
Share the resulting PbrMaterial/TextureMap images with Arc at asset load.
Scene finish selection still comes from Core IR via the Query API. This asset
collection does not introduce a scene graph or a second HyMeKo parser.
Applications can decode another collection with MaterialLibrary::from_json.
Identifiers are stable; labels are display metadata. Unknown schema/invalid
factors fail instead of silently applying a fallback.

Extending nonvisual properties
Each surface has optional "physical" entries. Example structure only:
  "thermal.conductivity": {
    "value": 0.12,
    "unit": "W/(m*K)",
    "source": "illustrative assumption; replace with a cited specimen measurement",
    "conditions": {"temperature": "293.15 K", "direction": "along grain"}
  }
That example is NOT included as actual data. Absence means unknown. Geometry
checks finite values and required metadata; the consuming simulation domain
must validate units/dimensions, admissible ranges and experimental conditions.
Contact friction depends on the mating material and conditions; visual
roughness never supplies it. Solver-specific physical laws remain with their
existing domain owner. Scalar properties are supported now; tensors, curves
and measured temperature-dependent laws need a versioned domain extension.

Procedural generation
SurfaceRecipe combines seeded smooth periodic value noise with wood grooves,
weave or brushed patterns. The seed, colour, height_m, cycles, roughness variation
and tile_m are explicit. Wood tile is four staggered boards. Plaster cycles
sets integer noise-tile frequency (cycles / 16, minimum one). Integer spatial
frequencies keep seams periodic. High-frequency pattern amplitude fades near
Nyquist; 512 is the room default, lower resolutions are previews.

The same UV/seed field yields sRGB albedo, signed float32 metre height, linear
MR (R=1, G=roughness, B=metallic), and tangent-space normals. Wrapped central
differences use size/(2*tile_m); doubling tile_m halves the physical slope.
Normal encoding: [-dh/du, -dh/dv, 1], normalized, mapped to [0,1]; +Y follows
increasing V/row index. All maps sample texel centres. The normal map changes
lighting, not silhouette, colliders or actual surface geometry.

PbrMaterial factors are set to one for metallic/roughness after baking because
the generated MR map stores the absolute factors. Coat/sheen remain separate.
Output maps have repeat wrapping and mipmaps. Work runs once at load or offline,
not each frame; the existing renderer owns upload/caching/filtering.

Bounds
JSON <=4 MiB, 1..256 entries, bake power-of-two 32..1024, tile 0.001..100 m,
height 0..0.01 m, cycles 1..128, roughness variation 0..0.5. A 1024 bake retains
16 MiB of pixel/height data before caller copies; each 512 surface is 3 MiB of
RGBA maps plus 1 MiB optional height, excluding renderer mips. Invalid numerical
inputs fail before allocation. Results are deterministic on the same numeric
implementation; cross-platform trigonometric byte identity is not promised.

Focused checks
  cargo test --locked --offline -p hymeko_geometry --no-default-features --test material_library
  cargo test --locked --offline -p hymeko_geometry --no-default-features --lib material::
