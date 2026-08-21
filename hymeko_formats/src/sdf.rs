//! SDF (Simulation Description Format) XML generation.
//! SDF 1.7 format for Gazebo compatibility.

use hymeko::ir::ir::Ir;
use hymeko_query::QueryEngine;
use hymeko_query::kinematics::joints::JointType;
use hymeko_query::kinematics::kinematic::{extract_kinematic_model, GeometryInfo, GeometryShape, KinematicModel};
use hymeko_query::traits::NameResolver;

/// Generate SDF XML string from a compiled IR.
///
/// EmissionPipeline: lift IR→KinematicModel via `extract_kinematic_model`,
/// then delegate to `generate_sdf_from_model`. Both entry points produce
/// identical output for the same model name; the model-taking variant is
/// the single source of truth for the emission step.
pub fn generate_sdf<R: NameResolver>(ir: &Ir, resolver: &R, model_name: &str) -> String {
    let engine = QueryEngine::new(ir, resolver);
    let model = extract_kinematic_model(&engine, model_name);
    generate_sdf_from_model(&model)
}

pub fn generate_sdf_from_model(model: &KinematicModel) -> String {
    let mut out = String::with_capacity(4096);
    out.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    out.push_str("<sdf version=\"1.7\">\n");
    out.push_str(&format!("  <model name=\"{}\">\n", model.name));
    out.push_str("    <static>false</static>\n\n");

    for link in &model.links {
        out.push_str(&format!("    <link name=\"{}\">\n", link.name));

        if let Some(mass) = link.mass {
            out.push_str("      <inertial>\n");
            out.push_str(&format!("        <mass>{}</mass>\n", mass));
            let ixx = mass * 0.01; // diagonal approximation
            out.push_str(&format!(
                "        <inertia>\n          <ixx>{ixx}</ixx><iyy>{ixx}</iyy><izz>{ixx}</izz>\n          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>\n        </inertia>\n"
            ));
            out.push_str("      </inertial>\n");
        }

        if let Some(ref geom) = link.geometry {
            for tag in &["visual", "collision"] {
                out.push_str(&format!("      <{} name=\"{}_{}\">\n", tag, link.name, tag));
                out.push_str("        <geometry>\n");
                emit_sdf_geometry(&mut out, geom, 10);
                out.push_str("        </geometry>\n");
                out.push_str(&format!("      </{}>\n", tag));
            }
        }

        out.push_str("    </link>\n\n");
    }

    for joint in &model.joints {
        out.push_str(&format!(
            "    <joint name=\"{}\" type=\"{}\">\n",
            joint.name, joint.joint_type.sdf_str()
        ));
        out.push_str(&format!("      <parent>{}</parent>\n", joint.parent_link));
        out.push_str(&format!("      <child>{}</child>\n", joint.child_link));

        // Skip the joint `<pose>` when the parent is `world` — the
        // `world` identifier inside an SDF `<model>` is not a valid
        // `relative_to` target (Gazebo error 26: "relative_to name
        // [world] does not match a nested model, link, joint, or
        // frame name"). The model-to-world pose is established by
        // the `<include>` directive that loads the model into a
        // world file. The HymeKo convention `(+ world, - base_link)`
        // for the j_fix joint maps to "bolt the model frame to the
        // world frame at the include pose" and emits no model-local
        // `<pose>`.
        if let Some(xyz) = joint.origin_xyz {
            if joint.parent_link != "world" {
                let rpy = joint.origin_rpy_rad().unwrap_or([0.0; 3]);
                out.push_str(&format!(
                    "      <pose relative_to=\"{}\">{} {} {} {:.4} {:.4} {:.4}</pose>\n",
                    joint.parent_link, xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]
                ));
            }
        }

        if joint.joint_type != JointType::Fixed {
            if let Some(ax) = joint.axis {
                out.push_str("      <axis>\n");
                out.push_str(&format!(
                    "        <xyz>{} {} {}</xyz>\n",
                    ax[0] as i32, ax[1] as i32, ax[2] as i32
                ));
                // Emit `<limit>` for non-fixed joints. For
                // Continuous joints we keep the SDF idiom of very
                // large bounds (preserves pre-2026-05-16
                // behaviour). For Revolute / Prismatic we emit
                // the limits from the IR when present — mirrors
                // the URDF emitter's handling.
                //
                // NOTE: SDF / URDF expect rotational limits in
                // radians; the HymeKo source convention is
                // degrees. The pass-through here matches the
                // existing URDF emitter; a unit-conversion fix
                // applies to both and is an open follow-up.
                match joint.joint_type {
                    JointType::Continuous => {
                        out.push_str("        <limit>\n");
                        out.push_str("          <lower>-1e16</lower>\n");
                        out.push_str("          <upper>1e16</upper>\n");
                        out.push_str("        </limit>\n");
                    }
                    JointType::Revolute | JointType::Prismatic => {
                        if let Some(ref lim) = joint.limits {
                            out.push_str("        <limit>\n");
                            out.push_str(&format!(
                                "          <lower>{}</lower>\n", lim.lower,
                            ));
                            out.push_str(&format!(
                                "          <upper>{}</upper>\n", lim.upper,
                            ));
                            out.push_str(&format!(
                                "          <effort>{}</effort>\n", lim.effort,
                            ));
                            out.push_str(&format!(
                                "          <velocity>{}</velocity>\n", lim.velocity,
                            ));
                            out.push_str("        </limit>\n");
                        }
                    }
                    JointType::Fixed => {}
                }
                out.push_str("      </axis>\n");
            }
        }

        out.push_str("    </joint>\n\n");
    }

    out.push_str("  </model>\n");
    out.push_str("</sdf>\n");
    out
}

fn emit_sdf_geometry(out: &mut String, geom: &GeometryInfo, indent: usize) {
    let pad: String = " ".repeat(indent);
    match geom.shape {
        GeometryShape::Box => {
            let d = &geom.dimensions;
            if d.len() >= 3 {
                out.push_str(&format!(
                    "{pad}<box>\n{pad}  <size>{} {} {}</size>\n{pad}</box>\n",
                    d[0], d[1], d[2]
                ));
            }
        }
        GeometryShape::Cylinder => {
            let d = &geom.dimensions;
            if d.len() >= 2 {
                out.push_str(&format!(
                    "{pad}<cylinder>\n{pad}  <radius>{}</radius>\n{pad}  <length>{}</length>\n{pad}</cylinder>\n",
                    d[0], d[1]
                ));
            }
        }
        GeometryShape::Sphere => {
            let d = &geom.dimensions;
            if !d.is_empty() {
                out.push_str(&format!(
                    "{pad}<sphere>\n{pad}  <radius>{}</radius>\n{pad}</sphere>\n",
                    d[0]
                ));
            }
        }
        GeometryShape::Capsule => {
            // Capsule dims = [length, radius]; SDF has a native <capsule>.
            let d = &geom.dimensions;
            if d.len() >= 2 {
                out.push_str(&format!(
                    "{pad}<capsule>\n{pad}  <radius>{}</radius>\n{pad}  <length>{}</length>\n{pad}</capsule>\n",
                    d[1], d[0]
                ));
            }
        }
    }
}
