//! Tests for the `SysmlTransform` — renders a kinematic chain as a
//! SysML 2 textual `package`. Targets the textual concrete syntax
//! consumed by Eclipse Papyrus / Modelix / the OMG SysML v2 Playground.

#[cfg(test)]
mod test_sysml_emit {
    use hymeko_formats::transforms::SysmlTransform;
    use hymeko_query::transforms::{DomainTransform, ModelKind, TransformConfig};

    use crate::test_helpers::load_and_lower;

    const MINI_ARM: &str = "../data/robotics/mini_arm.hymeko";
    const WAM: &str = "../data/robotics_imported/wam/wam.hymeko";

    fn render_sysml(src: &str, name: &str) -> String {
        let (store, compiled) = load_and_lower(src).unwrap();
        let config = TransformConfig::default().with_name(name);
        let model = hymeko_query::transforms::extract(
            &compiled.ir,
            &store.it,
            &config.robot_name,
            ModelKind::Kinematic,
        );
        let t = SysmlTransform;
        t.emit(&model, &config).expect("sysml emit returned None")
    }

    #[test]
    fn mini_arm_emits_package_with_links_and_connection() {
        let out = render_sysml(MINI_ARM, "mini_arm");
        assert!(
            out.contains("package mini_arm {"),
            "expected `package mini_arm {{` header, got:\n{out}"
        );
        assert!(
            out.contains("part def Link {"),
            "expected `part def Link` declaration"
        );
        assert!(
            out.contains("part base_link : Link"),
            "expected base_link instance"
        );
        assert!(
            out.contains("part spinner : Link"),
            "expected spinner instance"
        );
        assert!(
            out.contains("connection spin_joint : ContinuousJoint"),
            "expected spin_joint connection"
        );
        assert!(
            out.ends_with("}\n"),
            "expected trailing `}}` for the package"
        );
    }

    #[test]
    fn mini_arm_links_carry_mass() {
        let out = render_sysml(MINI_ARM, "mini_arm");
        // base_link has mass 5.0, spinner has mass 1.0 in mini_arm.hymeko.
        // The Display impl on f64 elides the trailing `.0` for round numbers,
        // so `5` is the expected serialisation.
        assert!(
            out.contains(":>> mass = 5"),
            "expected base_link's mass redefinition: {out}"
        );
        assert!(
            out.contains(":>> mass = 1"),
            "expected spinner's mass redefinition"
        );
    }

    #[test]
    fn wam_emits_seven_revolute_joints() {
        let out = render_sysml(WAM, "wam7");
        let n_revolute = out.matches(": RevoluteJoint").count();
        // 7 connection instances; the part def line uses `def RevoluteJoint`
        // (no leading colon-space) so it's not counted by this match. Allow a
        // little slack for future template tweaks.
        assert!(
            n_revolute >= 7,
            "expected >=7 RevoluteJoint connection instances, got {n_revolute}"
        );
        assert!(out.contains("package wam7 {"));
    }

    #[test]
    fn output_is_deterministic_across_runs() {
        let a = render_sysml(MINI_ARM, "mini_arm");
        let b = render_sysml(MINI_ARM, "mini_arm");
        assert_eq!(a, b, "sysml emit should be deterministic");
    }

    #[test]
    fn ids_are_sysml_safe() {
        // sysml_id replaces non-alnum-or-underscore with `_`. mini_arm and wam
        // already use legal identifiers, so this just sanity-checks no
        // pathological bytes leak into `part` / `connection` declarations.
        let out = render_sysml(MINI_ARM, "mini_arm");
        for line in out.lines() {
            if let Some(after) = line.trim_start().strip_prefix("part ") {
                let id = after.split_whitespace().next().unwrap_or("");
                if id == "def" {
                    continue;
                } // skip `part def Link {`
                assert!(
                    id.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'),
                    "invalid sysml id `{id}` in line: {line}"
                );
            }
        }
    }
}
