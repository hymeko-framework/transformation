#[cfg(test)]
mod test_urdf {
    use crate::test_helpers::load_and_lower;
    use hymeko_formats::urdf::generate_urdf;

    const ROBOT: &str = "../data/robotics/robot_4wh.hymeko";

    #[test]
    fn generate_urdf_structure() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "diff_robot_4wh");
        println!("--- Generated URDF ---\n{}", urdf);

        assert!(urdf.contains("<robot name=\"diff_robot_4wh\""));
        assert!(urdf.ends_with("</robot>\n"));
        assert!(urdf.contains("<link name=\"base_link\""));
        assert!(urdf.contains("<link name=\"wheel_fr\""));
        assert!(urdf.contains("<link name=\"camera_link\""));
    }

    #[test]
    fn joint_types_correct() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        assert!(urdf.contains("type=\"continuous\""));
        assert!(urdf.contains("type=\"fixed\""));
    }

    #[test]
    fn degree_to_radian_conversion() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        // -90° = -1.5708 rad
        assert!(urdf.contains("-1.5708"), "Expected -90° → -1.5708 radians");
    }

    #[test]
    fn axis_on_continuous_not_fixed() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        let axis_count = urdf.matches("<axis xyz=").count();
        assert!(axis_count >= 4, "At least 4 axes for continuous joints");
    }

    #[test]
    fn geometry_types() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        assert!(urdf.contains("<box size=\"0.7 0.5 0.2\""));
        assert!(urdf.contains("<cylinder"));
    }

    #[test]
    fn link_and_joint_counts() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        let link_count = urdf.matches("<link name=").count();
        let joint_count = urdf.matches("<joint name=").count();
        assert!(link_count >= 6, "Got {} links", link_count);
        assert!(joint_count >= 5, "Got {} joints", joint_count);
    }

    #[test]
    fn xml_well_formed() {
        let (store, compiled) = load_and_lower(ROBOT).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, "test");
        assert!(urdf.starts_with("<?xml"));
        assert!(urdf.ends_with("</robot>\n"));
    }
}
