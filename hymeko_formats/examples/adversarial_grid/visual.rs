//! Domain presentation export; the input world remains the semantic authority.
//! The output uses Kawamirai's existing generic primitive vocabulary, not a new scene schema.
use super::*;

/// Author one primitive in the existing scene vocabulary. Box sizes are half extents.
pub(super) fn node(
    name: &str,
    kind: &str,
    role: &str,
    p: [f64; 3],
    size: [f64; 3],
    rgb: [f64; 3],
    yaw: f64,
) -> String {
    format!(
        "{name} {{ kind \"{kind}\"; role \"{role}\"; px {}; py {}; pz {}; sx {}; sy {}; sz {}; r {}; g {}; b {}; a 1; qw {}; qx 0; qy 0; qz {}; }}\n",
        p[0],
        p[1],
        p[2],
        size[0],
        size[1],
        size[2],
        rgb[0],
        rgb[1],
        rgb[2],
        (yaw * 0.5).cos(),
        (yaw * 0.5).sin()
    )
}

impl CompiledWorld {
    pub(super) fn visual_source(&self) -> Result<(String, Value)> {
        let projection = self.project()?; // All seven query assertions precede export.
        let engine = QueryEngine::new(&self.program.ir, &self.store.it);
        let mut source = String::from("adversarial_grid_view {}\n");
        let mut obstacles = Vec::new();
        let scalar = |decl, key| self.scene_scalar(decl, key);
        for wall in engine.query(&Predicate::node().and(Predicate::inherits("gazebo_static_box"))) {
            let position = [
                scalar(wall.id, "x")?,
                scalar(wall.id, "y")?,
                scalar(wall.id, "z")?,
            ];
            let size = [
                scalar(wall.id, "size_x")?,
                scalar(wall.id, "size_y")?,
                scalar(wall.id, "size_z")?,
            ];
            if size.iter().any(|n| *n <= 0.) {
                return Err("obstacle sizes must be positive".into());
            }
            let name = format!("obstacle_{}", wall.id.raw());
            source.push_str(&node(
                &name,
                "Box",
                "obstacle",
                position,
                size.map(|n| n * 0.5),
                [0.25, 0.35, 0.45],
                0.0,
            ));
            obstacles.push(
                json!({"source_decl_id":wall.id.raw(),"source_name":wall.name,
                "visual_name":name,"position":position,"full_size":size}),
            );
        }
        let layout = &projection.layout;
        source.push_str(&node(
            "ground",
            "Box",
            "ground",
            [
                (layout.width as f64 - 1.) * 0.5,
                (layout.height as f64 - 1.) * 0.5,
                -0.05,
            ],
            [layout.width as f64 * 0.5, layout.height as f64 * 0.5, 0.05],
            [0.72, 0.75, 0.79],
            0.0,
        ));
        for (name, point, color) in [
            ("start", layout.start, [0.07, 0.7, 0.3]),
            ("goal", layout.goal, [0.9, 0.1, 0.12]),
        ] {
            source.push_str(&node(
                name,
                "Sphere",
                name,
                [point.x as f64, point.y as f64, 0.2],
                [0.22, 0., 0.],
                color,
                0.0,
            ));
        }
        let evaluation = projection.evaluate(self, false)?;
        for (i, pair) in evaluation.path.windows(2).enumerate() {
            let (a, b) = (pair[0], pair[1]);
            let position = [(a.x + b.x) as f64 * 0.5, (a.y + b.y) as f64 * 0.5, 0.025];
            let size = [
                a.x.abs_diff(b.x) as f64 * 0.5 + 0.035,
                a.y.abs_diff(b.y) as f64 * 0.5 + 0.035,
                0.012,
            ];
            source.push_str(&node(
                &format!("path_{i}"),
                "Box",
                "path_witness",
                position,
                size,
                [1., 0.55, 0.03],
                0.0,
            ));
        }
        let view = Self::compile(&source)?;
        let (recompiled, emitted) = view.reemit("adversarial_grid_view")?;
        let receipt = json!({"schema_version":1,"source_semantic_digest":self.digest(),
            "visual_semantic_digest":recompiled.digest(),"query_checks":check_receipts(&projection.checks),
            "obstacles":obstacles,"layout":layout,"path":evaluation.path,
            "scope":"derived visual document; original world remains authoritative; path is a search witness, not robot motion"});
        Ok((emitted, receipt))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn world() -> CompiledWorld {
        let layout = Layout {
            width: 3,
            height: 3,
            start: Point { x: 0, y: 0 },
            goal: Point { x: 2, y: 2 },
            walls: vec![Point { x: 1, y: 0 }],
        };
        CompiledWorld::compile(&layout.source().unwrap()).unwrap()
    }

    #[test]
    fn visual_export_keeps_obstacle_positions_sizes_and_source_identity() {
        let world = world();
        let (source, receipt) = world.visual_source().unwrap();
        assert_eq!(receipt["source_semantic_digest"], world.digest());
        assert_eq!(receipt["obstacles"].as_array().unwrap().len(), 1);
        assert_eq!(receipt["obstacles"][0]["position"], json!([1., 0., 0.5]));
        let compiled = CompiledWorld::compile(&source).unwrap();
        let engine = QueryEngine::new(&compiled.program.ir, &compiled.store.it);
        let matches = engine.query(&Predicate::node().and(Predicate::ChildValue(
            "role".into(),
            ValuePredicate::StrEq("obstacle".into()),
        )));
        assert_eq!(matches.len(), 1);
        for field in ["sx", "sy", "sz"] {
            assert!(
                matches!(engine.effective_value(matches[0].id,field),Some(ValueR::Num(n)) if *n==0.5)
            );
        }
        assert_eq!(compiled.digest(), receipt["visual_semantic_digest"]);
    }

    #[test]
    fn malformed_world_and_nonpositive_geometry_cannot_be_exported() {
        let world = world();
        let original =
            hymeko_emitter::emit_hymeko(&world.program.ir, &world.store.it, "adversarial_grid");
        let invalid =
            CompiledWorld::compile(&(original.clone() + "\n@bad { (~ cell_0_0, ~ grid); }\n"))
                .unwrap();
        assert!(invalid.visual_source().is_err());
        let invalid_source = original.replace("size_x 1.0;", "size_x 0.0;");
        assert_ne!(
            invalid_source, original,
            "the mutation must change the geometry"
        );
        let invalid = CompiledWorld::compile(&invalid_source).unwrap();
        assert!(invalid.visual_source().is_err());
    }
}
