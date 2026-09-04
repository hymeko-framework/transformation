# WebXR semantic model (HyMeKo)

A WebXR scene is authored entirely in HyMeKo `.hymeko` source, lowers into the **one** canonical
`hymeko_core::ir::Ir`, and is validated by `hymeko_webxr::validate(&Ir, &Interner)`. This is a
semantic model, not a renderer: no runtime XR handle, no scene DTO, no second scene graph. The vocabulary
splits into `meta_spatial` (general, renderer-independent) and `meta_webxr` (WebXR specialisation that
imports it). Enum strings are the WebXR Device API (W3C Candidate Recommendation) values; `anchors` and
`layers` are Immersive-Web editor drafts.

## Meta-elements

| element | layer | isa | meaning |
|---|---|---|---|
| `scene` | meta_spatial | spatial_thing | the authored scene root |
| `spatial_frame` | meta_spatial | spatial_thing | a coordinate frame to place in / parent to |
| `spatial_entity` | meta_spatial | spatial_thing | a placeable thing with identity |
| `visual_asset` | meta_spatial | spatial_thing | a referenced asset — `uri` / `media_type` / `role` only (no mesh buffers) |
| `appearance` | meta_spatial | spatial_thing | material / appearance hint (`color [r,g,b,a]`) |
| `semantic_anchor` | meta_spatial | spatial_thing | a durable spatial attachment point (semantic, not an `XRAnchor`) |
| `affordance` | meta_spatial | spatial_thing | what an entity offers (selectable, grabbable, …) |
| `action` | meta_spatial | spatial_thing | a hardware-independent interaction verb |
| `input_role` | meta_spatial | spatial_thing | an abstract input source role |
| session modes | meta_webxr | — | `inline`, `immersive_vr`, `immersive_ar` (stable) |
| reference spaces | meta_webxr | — | `viewer`, `local`, `local_floor`, `bounded_floor`, `unbounded` (stable) |
| capabilities | meta_webxr | — | `hit_test`, `hand_tracking`, `dom_overlay` (stable); `anchors`, `layers` (draft) |
| input roles | meta_webxr | input_role | `gaze`, `screen_pointer`, `transient_pointer`, `controller`, `left_hand`, `right_hand`, `any_hand` |
| actions | meta_webxr | action | `select`, `grab`, `release`, `activate`, `move`, `rotate`, `inspect`, `place` |

## Relations (multiway `@`-hyperedges) and role cardinality

Relations are HyMeKo hyperedges with signed incidences: `+` subject, `-` object, `~` peer.

| relation | signature | validator cardinality |
|---|---|---|
| `@placement` | `(+ spatial_entity, - spatial_frame)` + pose | exactly one `+` (a spatial_entity); every `-` is a spatial_frame |
| `@parent_frame` | `(+ child_frame, - parent_frame)` | acyclic (frame-cycle check) |
| `@anchoring` | `(+ spatial_entity, - spatial_frame, ~ semantic_anchor)` | `+` is a spatial_entity; a `-` frame and a `~` anchor are present |
| `@interaction` | `(+ input_role, ~ action, - target[, - context])` | exactly one `+`; ≥1 `-` target; each `~` is a known action |
| `@session_config` | `(+ session_mode, - reference_space, ~ fallback_mode)` | exactly one primary mode and one primary reference space per scene; `~` fallback optional |
| `@capability_request` | `(+ scene, - capability)` + `requirement` | `-` is a known capability; `requirement ∈ {required, optional}` |
| `@effect` | `(+ action, - affected …)` | (not role-validated; carries the semantic effect, e.g. highlight connected component) |
| `@appearance_binding` | `(+ spatial_entity, - appearance \| visual_asset)` | (binding; asset `uri` is validated to be a string) |

## Coordinate / units contract

- Length unit: **metre**. Right-handed; **+X** right, **+Y** up, **−Z** forward.
- Orientation: **unit quaternion, component order x, y, z, w**. Non-zero and normalizable within tolerance
  (`|q|² ≥ 1e-6`).
- Scale is a **separate** factor (three finite positive numbers), NOT part of the rigid transform.
- A pose (`position [x,y,z]`, `orientation [x,y,z,w]`, `scale [sx,sy,sz]`) is always relative to a named
  `spatial_frame`, carried on the `@placement` relation.

This is the WebXR / glTF convention, deliberately distinct from `meta_kinematics` (URDF: degrees, RPY
Euler, Z-up). They are different domains; `meta_kinematics.frame` may later `<isa>` `spatial_frame` under
a separate change, but is not imported here.

## Canonical Ir vs. runtime state

| stays in the canonical Ir | never in the Ir (runtime) |
|---|---|
| scene / entity / frame / action **identity** | `XRSession`, `XRFrame`, `XRSpace`, `XRAnchor` objects |
| **authoring** pose (position/orientation/scale) | per-frame viewer / hand / controller pose |
| placement, parent-frame, anchoring, interaction | WebGL/WGPU resources, DOM nodes |
| affordances | controller button indices |
| required / optional capability **intent** | runtime-reported capability, browser permission state |
| asset references (URI / media-type / role) | runtime tracking confidence |

The `semantic_anchor`'s canonical `DeclId` and a runtime `XRAnchor` handle are **separate identities**;
binding one to the other is a runtime concern. `hymeko_webxr::validate` enforces the
"no runtime-handle field"
half of this boundary: a source declaration whose attribute name is reserved for a runtime handle
(`xrsession`, `xrframe`, `xrspace`, `xranchor`, `button_index`, `viewer_pose`, `permission_state`,
`tracking_confidence`, GL/DOM resources, …) is rejected.

## Capability negotiation

A scene declares **intent**, never device support. Each requested capability is a `@capability_request`
carrying `requirement "required"` or `"optional"`; a capability may not be both. `required` and `optional`
describe what the scene asks a browser for (mapping to WebXR `requiredFeatures` / `optionalFeatures`);
`runtime-reported` support is asserted only by the runtime, never by source. `stability "standard" |
"draft"` on each capability records standardisation maturity so a binding can gate drafts behind a flag.

## Future: endpoint binding and dynamic re-sync (NOT implemented — PoW-1)

The later binding path keeps the Ir canonical and read-only at the boundary:

```
canonical Ir
  -> borrowed semantic queries (this module's shape)
  -> browser-side WebXR binding
  -> runtime handles keyed by canonical IDs
```

The return path is durable, not per-frame:

```
select / grab / move transient events   (runtime transient state)
  -> a durable release / place decision
  -> a future IRDelta transaction        (NOT this phase)
  -> a versioned Ir snapshot
```

There is **no per-frame IR commit**. High-frequency pose is runtime transient state; only a durable
place/grab-release outcome becomes an IRDelta. This section is a documented contract, not code.
