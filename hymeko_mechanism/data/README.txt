SURFACE MATERIALS AND MESHES

The shared material library (materials.json) and the triangle-mesh operators
are owned by the Geometry repository's IR-free hymeko_mesh crate, with their
documentation, material_bake example and focused tests in hymeko_mesh/data and
hymeko_mesh/tests. hymeko_mechanism re-exports hymeko_mesh::{mesh, material}, so
hymeko_mechanism::mesh and hymeko_mechanism::material paths name the same types.

Geometry recipes
----------------
hymeko_mesh::recipe::GeometryRecipe (Geometry repository) is the derived execution
description; this crate reads it out of Core IR. Enable the optional authoring feature to use
recipe::RecipeProject::compile or project a compiled model. Core handles source/imports/digests; Query handles effective children and
inherited fields. Geometry executes the existing mesh operators. No simulation
feature is required. Recipes contain schema_version=1, length_unit="m", up_axis,
workflow, settings and named stages with unique contiguous order values from zero.
up_axis is explicit convention metadata, not an implicit coordinate transform.

surface-smooth.hymeko: topology-preserving fairing and normal filtering.
surface-refine.hymeko: fairing, constrained quad reconstruction, normal filtering.
The character workflow is projected here but executed by the consuming asset
owner; arbitrary command/executable stages are not admitted.

  cargo build --locked --offline -p hymeko_mechanism --no-default-features --features authoring,mesh-optimization --example surface_prepare
  target/debug/examples/surface_prepare recipe hymeko_mechanism/data/surface-smooth.hymeko geometry RECIPE.json
  target/debug/examples/surface_prepare pipeline hymeko_mechanism/data/surface-refine.hymeko geometry INPUT_MESH.json OUTPUT.json
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

  cargo test --locked --offline -p hymeko_mechanism --no-default-features --features authoring,mesh-optimization --test geometry_pipeline
