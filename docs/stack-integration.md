# Kawamirai / Kawakern local integration

Updated 2026-09-17. These are local development checkouts, not a published release.

| Role | Checkout / branch | Integration |
| --- | --- | --- |
| Geometry / Kawakern | `hymeko_framework/transformation`, `feat/robotics-authoring-integration` | DH chain, Formula track/vehicle, inherited physical fields, contact/joint-stop fixes and opt-in contact replay |
| Renderer | `hymeko_framework/kawamirai`, `feat/xr-passthrough-rendering` | Published `5d022ba`, local Geometry override |
| Simulator | `hymeko_simulator-xr-publish`, `feat/quest-immersive-ui` | Physical grab, pulse integration, Cyberdisc drive, Formula cockpit and shared UI |
| Quest / WebXR | `hymeko_framework/xr`, `feat/native-quest-workbench` | Shared Simulator and renderer, local Geometry/Query overrides |
| XRobotAssist adapter | `xrobotassist_workspace/middleware_hymeko`, `feat/hymeko-ir-bridge` | Retained-IR DH avatar; explicitly enables Geometry `simulation` |

Paths are relative to `D:/hakiko_ai_ws/03_implementation`. Simulator, XR and
renderer Cargo roots explicitly select the same Geometry checkout. The bridge
uses its existing `tools/use-local-geometry.ps1` override. Cargo ignores a
dependency's own patch table: the consuming workspace must keep its override.
The standalone Workbench published dependencies are unchanged; when embedded in
Quest its dependencies resolve under the XR root's patch table.

Core remains at the published `2c4e072` API, Graph/SBSH at `be8da9c`, Clifford at
`14c1d23` and Spline at `eb65203`. The local Core `10138bf` adds documentation only.
Kawakern is the existing Geometry/Dynamics engine family over Tensor and Graph.
The simulator supplies application inputs; it does not reimplement integration,
contact resolution, forward kinematics or the canonical IR.

`hymeko_geometry` enables `simulation` by default. Pure rendering consumers can
disable default features for mesh/material types. Kinematics consumers that
disable defaults must explicitly enable `simulation`.

The source checkouts `transformation-formula`, `hymeko_simulator-publish` and
`hymeko_simulator` are preserved. Their local physics changes were reconciled by
content into the destinations above; their branches were not rebased or merged
in Git. Keep new work on the integrated destinations to avoid divergence.

Cyberdisc obstacle intersection currently stops at the last safe pose. It is
not a general rigid-body obstacle impulse solver. Contact replay is a separate
diagnostic API; importing it does not replace the production contact solver.
Ignored experiment and external-checkout tests remain opt-in.

Before publication, commit and publish the owner changes, align downstream
revision pins, remove temporary path overrides and verify a clean clone. An
Android compile check does not update the installed APK or validate a headset.

Reconciliation hashes, before/incoming snapshots and checks are stored locally
in `reports/2026-09-17-stack-refresh/`. The task report is
`reports/2026-09-17-stack-refresh.md`; task-close archival uses the shared
`\\192.168.1.228\hymeko\reports\by-repository\transformation` destination.
