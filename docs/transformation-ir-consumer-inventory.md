# Transformation → canonical IR consumer inventory (M2b pre-migration audit)

**Generated:** 2026-08-30 03:22 CEDT · **Base HEAD:** f67b723e4d1bcc6a10404afaf63fbd8ffb349e89 · **core pin:** `15223bc`
**Authorisation:** `APPROVED-TRANSFORMATION-TEST-EDIT: unified-ir-m2b-audit-oracles`

Every raw canonical-IR access site in the three consumer crates, classified by
access pattern. "Raw arena index" = a direct `ir.<arena>[id.0]` projection that
the accessor API (`decls`, `kind_of`, `decl_node`, `as_node`/`node`,
`as_edge`/`edge`, `arc`, `parent_of`, `node_for_decl`/`edge_for_decl`) is meant
to replace. Generated mechanically (`grep`); do not hand-edit — regenerate.

## Totals

| Crate | raw arena-index sites | accessor sites already present |
|---|---|---|
| `hymeko_emitter` | 24 | 3 |
| `hymeko_query` | 111 | 25 |
| `hymeko_formats` | 14 | 5 |

## Per-arena breakdown (raw index sites, src only, production + test)

| Arena projection | emitter | query | formats | accessor replacement |
|---|---|---|---|---|
| `.decl_nodes[…]` | 13 | 63 | 7 | `decl_node(did)` / `kind_of(did)` |
| `.nodes[…]` | 2 | 9 | 2 | `as_node(did)` then `node(nid)` |
| `.edges[…]` | 2 | 20 | 3 | `as_edge(did)` then `edge(eid)` |
| `.arcs[…]` | 4 | 8 | 2 | `arc(aid)` |
| `.decl_to_node[…]` | 1 | 3 | 0 | `as_node(did)` / `node_for_decl(did)` |
| `.decl_to_edge[…]` | 1 | 4 | 0 | `as_edge(did)` / `edge_for_decl(did)` |
| `.decl_to_arc[…]` | 1 | 4 | 0 | `as_arc(did)` |
| `.decl_hash[…]` | 0 | 0 | 0 | (hash cache — core-internal) |
