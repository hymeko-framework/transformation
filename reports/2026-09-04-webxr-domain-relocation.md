# WebXR domain relocation

Date: 2026-09-04 CEST

Branch: `feat/webxr-domain-projection`

Core authorization: `APPROVED-CORE-EDIT: relocate-webxr-to-transformation`

## Outcome

WebXR and renderer-independent spatial semantics now live in the dedicated
`hymeko_webxr` transformation crate. The crate consumes the canonical
`hymeko_core::ir::Ir` and `Interner`; it does not introduce another IR, a
scene DTO, runtime XR handles, or rendering state.

The migration was taken from the latest core working tree, not merely its
committed WebXR module. It therefore preserves the current camera/fiducial
vocabulary, borrowed spatial query API, query tests, and the
`maxwell_fiducial_demo.hymeko` integration fixture.

The workspace is pinned to paired core cleanup commit
`f5181de6735380abb1efc060bebad548ccf459aa`; the resolved core dependency
therefore contains no WebXR module.

The crate-local `data/` tree contains the WebXR vocabulary and the exact
dynamics/optimization fixtures needed to compile that integration fixture.
The supporting dynamics/optimization sources remain in core for now: removing
all non-WebXR domain fixtures is a wider ownership migration than the approved
WebXR edit and needs its own atomic change.

## Verification

- `cargo fmt --package hymeko_webxr -- --check`: pass
- `cargo test -p hymeko_webxr --all-features --no-fail-fast`: 29 run,
  28 passed, 1 measurement benchmark ignored
- `cargo clippy -p hymeko_webxr --all-targets --all-features -- -D warnings`:
  pass
- `cargo test --workspace --all-features --no-fail-fast`, after pinning the
  WebXR-free core commit: pass, zero failures

The full-workspace authority tests require the repository to be adjacent to a
`core` checkout because an existing test resolves `../../core/data`. The first
isolated worktree run exposed that location assumption; rerunning from the
normal sibling-repository layout passed all 10 authority tests without a code
change.
