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

Offline PNG + height export, from Transformation root:
  cargo run --locked --offline -p hymeko_geometry --no-default-features --example material_bake -- OUTPUT_DIRECTORY 512
Height files: row-major .f32le metres, no display normalization. Export metadata
contains the exact library and conventions. The crate itself has no image codec
runtime dependency: image is used only by the exporter example.

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

Geometry recipes
----------------
pipeline::GeometryRecipe is a derived execution description over Core IR. Enable
the optional authoring feature to use RecipeProject::compile or project a compiled
model. Core handles source/imports/digests; Query handles effective children and
inherited fields. Geometry executes the existing mesh operators. No simulation
feature is required. Recipes contain schema_version=1, length_unit="m", up_axis,
workflow, settings and named stages with unique contiguous order values from zero.
up_axis is explicit convention metadata, not an implicit coordinate transform.

surface-smooth.hymeko: topology-preserving fairing and normal filtering.
surface-refine.hymeko: fairing, constrained quad reconstruction, normal filtering.
The character workflow is projected here but executed by the consuming asset
owner; arbitrary command/executable stages are not admitted.

  cargo build --locked --offline -p hymeko_geometry --no-default-features --features authoring,mesh-optimization --example surface_prepare
  target/debug/examples/surface_prepare recipe hymeko_geometry/data/surface-smooth.hymeko geometry RECIPE.json
  target/debug/examples/surface_prepare pipeline hymeko_geometry/data/surface-refine.hymeko geometry INPUT_MESH.json OUTPUT.json
On Windows use .exe. Output files must be new. A JSON source argument is the
captured {entry,files} project instead of a disk .hymeko entry. pipeline accepts
TriangleMesh or {mesh: TriangleMesh}, and returns source_digest, recipe and result
(mesh, optional quad_surface, per-stage counts/displacement). It does not transfer
skinning weights or other parallel character data; character export owns those.
Existing quads/normals CLI operations remain compatible. fair accepts {mesh,
fair_options,influence?} and returns {mesh,fairing} with admitted step fraction.

SurfaceFairing uses the shared topology, forward/reverse diffusion, optional
per-source-vertex influence and smooth bounded regions. Position aliases share
the maximum influence. Default pin mode protects boundaries, UV seams and creases;
free mode admits their movement together. Source UV/indices are unchanged. A
serialized f32 orientation check backtracks the proposed displacement before
commit; invalid input or cancellation leaves the caller's input unchanged.
Displacement is bounded in metres, including f32 conversion. Moved meshes discard
stale tangents and derived LODs. Normal rebuilding is explicit. Normal smoothing
does not move positions. Zero-area work faces are skipped but retained in output;
strict regularization still rejects degenerate/nonmanifold reconstruction input.

Recipe limits: 4 MiB decoded source/256 relative files, depth 12/4096 projected
values, 32 stages; mesh CLI input 64 MiB. Fairing caps 250000 vertices/500000
triangles, 96 iterations and 50 mm displacement. Existing regularization/LOD
operator limits still apply. Run callbacks provide stage progress/cancellation;
the CLI checks HYMEKO_GEOMETRY_CANCEL_FILE between stages. No operator runs per
frame. These limits are admission limits, not measured performance guarantees.

  cargo test --locked --offline -p hymeko_geometry --no-default-features --features authoring,mesh-optimization --test geometry_pipeline --test mesh_regularization
