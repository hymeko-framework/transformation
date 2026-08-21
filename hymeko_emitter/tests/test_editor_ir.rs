//! `editor_ir::HyMeKoEditorIR::apply` smoke + property tests.

use hymeko_emitter::editor_ir::{
    Attribute, AttributeValue, HyMeKoEditorIR, HyperEdge, IRDelta, IRError, Position, Sign, Vertex,
};

fn v(name: &str, level: i8) -> Vertex {
    Vertex {
        name: name.to_string(),
        level,
        attributes: Vec::new(),
        position: None,
    }
}

#[test]
fn apply_add_vertex_increments_slotmap() {
    let mut ir = HyMeKoEditorIR::new();
    assert_eq!(ir.vertices.len(), 0);

    ir.apply(IRDelta::AddVertex {
        data: v("base_link", 0),
    })
    .unwrap();
    assert_eq!(ir.vertices.len(), 1);

    ir.apply(IRDelta::AddVertex {
        data: v("spinner", 0),
    })
    .unwrap();
    assert_eq!(ir.vertices.len(), 2);
}

#[test]
fn apply_remove_vertex_also_prunes_incident_edges() {
    let mut ir = HyMeKoEditorIR::new();
    let a = ir.vertices.insert(v("a", 0));
    let b = ir.vertices.insert(v("b", 0));
    let e = ir.hyperedges.insert(HyperEdge {
        name: "ab".into(),
        incident: vec![(a, Sign::Plus), (b, Sign::Minus)],
        weight: 1.0,
        patch_id: None,
    });

    ir.apply(IRDelta::RemoveVertex { key: a }).unwrap();
    assert_eq!(ir.vertices.len(), 1);
    // The incident list on edge `e` should have dropped the reference to `a`.
    let remaining = &ir.hyperedges[e].incident;
    assert_eq!(remaining.len(), 1);
    assert_eq!(remaining[0].0, b);
}

#[test]
fn apply_move_vertex_updates_position() {
    let mut ir = HyMeKoEditorIR::new();
    let k = ir.vertices.insert(v("x", 0));
    ir.apply(IRDelta::MoveVertex {
        key: k,
        position: Position { x: 3.0, y: 4.0 },
    })
    .unwrap();
    assert_eq!(ir.vertices[k].position, Some(Position { x: 3.0, y: 4.0 }));
}

#[test]
fn apply_update_weight_updates_hyperedge_weight() {
    let mut ir = HyMeKoEditorIR::new();
    let e = ir.hyperedges.insert(HyperEdge {
        name: "edge".into(),
        incident: Vec::new(),
        weight: 0.5,
        patch_id: None,
    });
    ir.apply(IRDelta::UpdateWeight { key: e, weight: 2.5 }).unwrap();
    assert!((ir.hyperedges[e].weight - 2.5).abs() < 1e-12);
}

#[test]
fn apply_update_sign_flips_arc_sign() {
    let mut ir = HyMeKoEditorIR::new();
    let a = ir.vertices.insert(v("a", 0));
    let b = ir.vertices.insert(v("b", 0));
    let e = ir.hyperedges.insert(HyperEdge {
        name: "ab".into(),
        incident: vec![(a, Sign::Plus), (b, Sign::Minus)],
        weight: 1.0,
        patch_id: None,
    });
    ir.apply(IRDelta::UpdateSign {
        key: e,
        arc_index: 0,
        sign: Sign::Neutral,
    })
    .unwrap();
    assert_eq!(ir.hyperedges[e].incident[0].1, Sign::Neutral);
}

#[test]
fn apply_update_sign_out_of_range_errors() {
    let mut ir = HyMeKoEditorIR::new();
    let e = ir.hyperedges.insert(HyperEdge {
        name: "edge".into(),
        incident: Vec::new(),
        weight: 1.0,
        patch_id: None,
    });
    let err = ir
        .apply(IRDelta::UpdateSign {
            key: e,
            arc_index: 5,
            sign: Sign::Plus,
        })
        .unwrap_err();
    assert!(matches!(err, IRError::ArcIndexOutOfRange(5, 0)));
}

#[test]
fn apply_attach_detach_attribute_roundtrip() {
    let mut ir = HyMeKoEditorIR::new();
    let k = ir.vertices.insert(v("link", 0));
    ir.apply(IRDelta::AttachAttribute {
        key: k,
        attr: Attribute {
            key: "mass".into(),
            value: AttributeValue::Float(5.0),
        },
    })
    .unwrap();
    assert_eq!(ir.vertices[k].attributes.len(), 1);

    ir.apply(IRDelta::DetachAttribute {
        key: k,
        name: "mass".into(),
    })
    .unwrap();
    assert_eq!(ir.vertices[k].attributes.len(), 0);
}

#[test]
fn apply_detach_missing_attribute_errors() {
    let mut ir = HyMeKoEditorIR::new();
    let k = ir.vertices.insert(v("link", 0));
    let err = ir
        .apply(IRDelta::DetachAttribute {
            key: k,
            name: "nope".into(),
        })
        .unwrap_err();
    assert!(matches!(err, IRError::AttributeNotFound(s) if s == "nope"));
}

#[test]
fn apply_batch_runs_deltas_in_order() {
    let mut ir = HyMeKoEditorIR::new();
    ir.apply(IRDelta::Batch {
        deltas: vec![
            IRDelta::AddVertex { data: v("a", 0) },
            IRDelta::AddVertex { data: v("b", 0) },
            IRDelta::AddVertex { data: v("c", 0) },
        ],
    })
    .unwrap();
    let names: Vec<&str> = ir.vertices.values().map(|v| v.name.as_str()).collect();
    assert_eq!(names.len(), 3);
    assert!(names.contains(&"a") && names.contains(&"b") && names.contains(&"c"));
}

#[test]
fn apply_batch_short_circuits_on_first_error() {
    let mut ir = HyMeKoEditorIR::new();
    // Second delta targets a non-existent key, so Batch must return Err
    // without applying the third. We rely on the fact that the first one
    // succeeds — so the vertex count reflects a partial apply.
    let bogus = slotmap::KeyData::from_ffi(0xDEADBEEF_u64).into();
    let err = ir
        .apply(IRDelta::Batch {
            deltas: vec![
                IRDelta::AddVertex { data: v("ok", 0) },
                IRDelta::RemoveVertex { key: bogus },
                IRDelta::AddVertex { data: v("never", 0) },
            ],
        })
        .unwrap_err();
    assert!(matches!(err, IRError::NotFound));
    assert_eq!(ir.vertices.len(), 1);
}
