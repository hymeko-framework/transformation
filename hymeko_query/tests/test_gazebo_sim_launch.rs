//! End-to-end Gazebo launch bundle test.
//!
//! Generates the minimum set of files needed to launch
//! `data/robotics/anthropomorphic_arm.hymeko` in the **new Gazebo** (`gz
//! sim`, a.k.a. Ignition / Gazebo Harmonic / Ionic — not the classic
//! `gazebo-classic` stack):
//!
//!   - `moveo.urdf`             — robot description, produced by
//!                                 `hymeko_formats::urdf::generate_urdf`
//!   - `moveo.world.sdf`        — minimal SDF 1.8 world with ground plane
//!                                 and the standard gz-sim plugin triple
//!                                 (physics / user-commands / scene-broadcaster)
//!   - `gz_sim.launch.py`       — ROS 2 Python launch that (a) starts `gz
//!                                 sim` on the world, (b) runs
//!                                 `robot_state_publisher` with the URDF,
//!                                 (c) spawns the robot via
//!                                 `ros_gz_sim::create`, and (d) brings up
//!                                 `ros_gz_bridge::parameter_bridge` for
//!                                 clock + joint_states topic remapping.
//!
//! The test does **not** invoke `gz sim` — that requires Gazebo installed
//! locally. Instead it writes the bundle to the workspace-level directory
//! `<workspace>/generated/gazebo_launch/<robot>/` (outside `target/`, so
//! `cargo clean` does not delete it) and asserts the structural tokens
//! that make the bundle launchable. Each test run refreshes the directory
//! and deposits a `README.md` with the exact manual-launch commands. The
//! path is logged at INFO level so `RUST_LOG=info cargo test ...` surfaces
//! it; the location is also the same one the user can `cd` into to run
//! `ros2 launch gz_sim.launch.py` against a local `gz sim` install.

#[cfg(test)]
mod test_gazebo_sim_launch {
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::Instant;

    use log::info;

    use hymeko_formats::urdf::generate_urdf;

    use crate::test_helpers::{load_and_lower, log_test_footer, log_test_header};

    const MOVEO: &str = "../data/robotics/anthropomorphic_arm.hymeko";
    const ROBOT_NAME: &str = "moveo";
    const WORLD_NAME: &str = "empty";

    /// Resolve the bundle output directory as
    /// `<workspace-root>/generated/gazebo_launch/<robot>/`.
    ///
    /// Using a workspace-root folder (not `target/`) is deliberate — the
    /// user needs to `cd` here and run `ros2 launch` against a local
    /// `gz sim` install, so the bundle must survive `cargo clean` and be
    /// trivially discoverable. `CARGO_MANIFEST_DIR` resolves at compile
    /// time to the package directory of the test crate (here
    /// `hymeko_query/`), so the workspace root is one level up.
    fn out_dir() -> PathBuf {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        // hymeko_query/.. == workspace root
        let workspace_root = manifest_dir
            .parent()
            .expect("package dir has a parent (workspace root)")
            .to_path_buf();
        workspace_root
            .join("generated")
            .join("gazebo_launch")
            .join(ROBOT_NAME)
    }

    /// Human-readable README written alongside each bundle so the folder
    /// is self-documenting even if someone stumbles into it without
    /// context.
    fn make_readme(
        robot_name: &str,
        urdf_file: &str,
        world_file: &str,
        launch_file: &str,
    ) -> String {
        format!(
            r#"# HyMeKo → new-Gazebo launch bundle — `{robot_name}`

This directory is **generated** by
`hymeko_query/tests/test_gazebo_sim_launch.rs`. It contains everything you
need to spawn the `{robot_name}` robot in the new Gazebo (`gz sim`, not the
old `gazebo-classic`).

## Contents

- `{urdf_file}`  — URDF robot description produced by
  `hymeko_formats::urdf::generate_urdf` from
  `data/robotics/anthropomorphic_arm.hymeko`.
- `{world_file}`  — Minimal SDF 1.8 world with a ground plane and the
  standard `gz-sim-physics-system` / `-user-commands-system` /
  `-scene-broadcaster-system` plugin triple.
- `{launch_file}`  — ROS 2 Python launch file that starts `gz sim`,
  publishes the URDF via `robot_state_publisher`, spawns the robot via
  `ros_gz_sim::create`, and bridges `/clock` + joint-state topics through
  `ros_gz_bridge::parameter_bridge`.

## Regenerate

```bash
cargo test -p hymeko_query --test integration test_gazebo_sim_launch
```

(or with live logging to see the summary:
`RUST_LOG=info cargo test -p hymeko_query --test integration test_gazebo_sim_launch -- --nocapture`)

## Launch in Gazebo

Prerequisites (Ubuntu 24.04 + ROS 2 Jazzy example):

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge ros-jazzy-robot-state-publisher
```

Then:

```bash
cd $(pwd)
ros2 launch {launch_file}
```

You should see `gz sim` start up with an empty world, the `{robot_name}`
URDF spawned at the origin, and joint-state / clock topics bridged into
ROS 2 (visible via `ros2 topic list`).
"#
        )
    }

    /// Build a ROS 2 Python launch file that starts `gz sim` on the world
    /// and spawns the robot via `ros_gz_sim::create`.
    fn make_gz_launch_py(robot_name: &str, urdf_file: &str, world_file: &str) -> String {
        format!(
            r#"# Generated by HyMeKo — do not edit by hand.
# Launches `{robot_name}` in the new Gazebo (`gz sim`, not gazebo-classic).
#
# Run:
#     ros2 launch gz_sim.launch.py
# (ensure ros_gz_sim + ros_gz_bridge are installed for your ROS 2 distro.)

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(pkg_dir, '{urdf_file}')
    world_path = os.path.join(pkg_dir, '{world_file}')

    with open(urdf_path, 'r', encoding='utf-8') as fh:
        robot_description = fh.read()

    ld = LaunchDescription()

    # 1. Start the new Gazebo (`gz sim`) on the generated world.
    ld.add_action(ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r', '--verbose', '3'],
        output='screen',
    ))

    # 2. Publish the URDF as a topic so `ros_gz_sim::create` can consume it.
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='{robot_name}_state_publisher',
        parameters=[{{'robot_description': robot_description}}],
        output='screen',
    ))

    # 3. Spawn the robot into the running `gz sim` instance.
    ld.add_action(Node(
        package='ros_gz_sim',
        executable='create',
        name='{robot_name}_spawner',
        arguments=[
            '-name', '{robot_name}',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    ))

    # 4. Bridge /clock + joint-state topics between gz and ROS 2.
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='{robot_name}_gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/{robot_name}/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    ))

    return ld
"#
        )
    }

    #[test]
    fn generate_gz_sim_launch_bundle_for_moveo() {
        let title = "generate_gz_sim_launch_bundle_for_moveo";
        log_test_header(
            title,
            "Emits URDF + world.sdf + gz_sim.launch.py bundle ready for `gz sim`.",
        );
        let start = Instant::now();

        // --- Load and generate URDF via the full production path ---------
        let (store, compiled) = load_and_lower(MOVEO).expect("moveo should compile");
        let urdf = generate_urdf(&compiled.ir, &store.it, ROBOT_NAME);
        info!("generated URDF: {} bytes", urdf.len());

        // --- Generate the Gazebo world via the real T11 emitter ----------
        // Prior versions of this test used a hand-templated world; since
        // `hymeko_formats::gazebo::generate_gazebo_world` landed
        // on 2026-04-19 we route through it so the bundle's plugin
        // section reflects the fixture's `sim_plugin` / `control_plugin`
        // declarations.
        let world = hymeko_formats::gazebo::generate_gazebo_world(
            &compiled.ir,
            &store.it,
            ROBOT_NAME,
            WORLD_NAME,
        );
        info!("generated world.sdf: {} bytes", world.len());

        // --- Prepare output directory ------------------------------------
        // `fs::write` truncates on open, so idempotent `create_dir_all`
        // is enough — no need to remove first (and the two tests in this
        // file can race on the same directory if we did).
        let dir = out_dir();
        fs::create_dir_all(&dir).expect("create bundle dir");
        info!("writing bundle to: {}", dir.display());

        // --- Write the four artefacts -----------------------------------
        let urdf_file = format!("{ROBOT_NAME}.urdf");
        let world_file = format!("{ROBOT_NAME}.world.sdf");
        let launch_file = "gz_sim.launch.py".to_string();
        let readme_file = "README.md".to_string();

        let urdf_path = dir.join(&urdf_file);
        let world_path = dir.join(&world_file);
        let launch_path = dir.join(&launch_file);
        let readme_path = dir.join(&readme_file);

        fs::write(&urdf_path, &urdf).expect("write URDF");
        fs::write(&world_path, &world).expect("write world.sdf");
        fs::write(
            &launch_path,
            make_gz_launch_py(ROBOT_NAME, &urdf_file, &world_file),
        )
        .expect("write launch.py");
        fs::write(
            &readme_path,
            make_readme(ROBOT_NAME, &urdf_file, &world_file, &launch_file),
        )
        .expect("write README");

        // --- Assert structural content for launchability -----------------
        let urdf_read = fs::read_to_string(&urdf_path).unwrap();
        assert!(urdf_read.starts_with("<?xml"), "URDF must be XML");
        assert!(
            urdf_read.contains("<robot name=\"moveo\""),
            "URDF must carry the robot name"
        );
        for joint in ["j_fix", "j0", "j1", "j2", "j3", "j4", "jtool"] {
            assert!(
                urdf_read.contains(&format!("<joint name=\"{joint}\"")),
                "URDF must include joint `{joint}`"
            );
        }
        info!(
            "URDF content: {} joint tags, {} link tags",
            urdf_read.matches("<joint name=").count(),
            urdf_read.matches("<link name=").count()
        );

        let world_read = fs::read_to_string(&world_path).unwrap();
        assert!(world_read.starts_with("<?xml"), "world must be XML");
        assert!(
            world_read.contains("<sdf version=\"1.8\">"),
            "world must declare SDF 1.8 (gz sim native)"
        );
        for plugin in [
            "gz-sim-physics-system",
            "gz-sim-user-commands-system",
            "gz-sim-scene-broadcaster-system",
        ] {
            assert!(
                world_read.contains(plugin),
                "world must register the `{plugin}` plugin"
            );
        }
        assert!(
            world_read.contains("<model name=\"ground_plane\">"),
            "world must contain a ground_plane"
        );
        info!(
            "world.sdf content: {} plugin tags, ground_plane={}",
            world_read.matches("<plugin ").count(),
            world_read.contains("ground_plane")
        );

        let launch_read = fs::read_to_string(&launch_path).unwrap();
        assert!(
            launch_read.contains("generate_launch_description"),
            "launch file must define generate_launch_description()"
        );
        // gz sim (new Gazebo) invocation — NOT `gazebo` (classic).
        assert!(
            launch_read.contains("'gz', 'sim'"),
            "launch file must start `gz sim`, not classic `gazebo`"
        );
        assert!(
            !launch_read.contains("gazebo_ros"),
            "launch file must not reference the classic `gazebo_ros` stack"
        );
        for pkg in ["ros_gz_sim", "ros_gz_bridge", "robot_state_publisher"] {
            assert!(
                launch_read.contains(pkg),
                "launch file must load `{pkg}` nodes"
            );
        }
        assert!(
            launch_read.contains("'-name', 'moveo'"),
            "launch file must pass the robot name to ros_gz_sim::create"
        );
        info!("launch.py content: {} bytes", launch_read.len());

        // --- Sanity: the four files exist and are non-empty -------------
        for (role, path) in [
            ("urdf", &urdf_path),
            ("world", &world_path),
            ("launch", &launch_path),
            ("readme", &readme_path),
        ] {
            let meta = fs::metadata(path).expect("bundle file must exist");
            info!("  {role:<6} -> {}  ({} bytes)", path.display(), meta.len());
            assert!(meta.len() > 0, "{role} bundle file is empty");
        }

        info!("bundle ready. To launch manually:");
        info!("  cd {}", dir.display());
        info!("  ros2 launch gz_sim.launch.py");
        info!("(see README.md in that directory for prerequisites)");

        log_test_footer(
            title,
            Some(start.elapsed()),
            "bundle written and structurally valid for `gz sim`.",
        );
    }

    #[test]
    fn bundle_files_are_referentially_consistent() {
        // The launch file references the URDF and world file by name; this
        // test generates the bundle (via the primary test's path) and then
        // parses the launch file to confirm the referenced filenames
        // actually exist in the bundle directory.
        let title = "bundle_files_are_referentially_consistent";
        log_test_header(
            title,
            "Every filename the launch.py references must exist next to it.",
        );
        let start = Instant::now();

        // Write into our OWN subdirectory under the bundle root so we can't
        // race with `generate_gz_sim_launch_bundle_for_moveo` under cargo's
        // default parallel runner — `fs::write` truncates the target file
        // before writing, and any stat/read in the other test's window would
        // see a zero-length file. Local runs are fast enough to hide this,
        // but CI under load surfaces it as a flaky failure on the primary
        // test (observed 2026-05-23 on workspace-tests).
        let dir = out_dir().join("_consistency");
        let (store, compiled) = load_and_lower(MOVEO).unwrap();
        let urdf = generate_urdf(&compiled.ir, &store.it, ROBOT_NAME);
        let world = hymeko_formats::gazebo::generate_gazebo_world(
            &compiled.ir,
            &store.it,
            ROBOT_NAME,
            WORLD_NAME,
        );

        fs::create_dir_all(&dir).unwrap();

        let urdf_file = format!("{ROBOT_NAME}.urdf");
        let world_file = format!("{ROBOT_NAME}.world.sdf");
        let launch_file = "gz_sim.launch.py";

        fs::write(dir.join(&urdf_file), &urdf).unwrap();
        fs::write(dir.join(&world_file), &world).unwrap();
        fs::write(
            dir.join(launch_file),
            make_gz_launch_py(ROBOT_NAME, &urdf_file, &world_file),
        )
        .unwrap();
        fs::write(
            dir.join("README.md"),
            make_readme(ROBOT_NAME, &urdf_file, &world_file, launch_file),
        )
        .unwrap();

        let launch = fs::read_to_string(dir.join(launch_file)).unwrap();
        let checks: &[(&str, &Path)] = &[
            (urdf_file.as_str(), Path::new(&urdf_file)),
            (world_file.as_str(), Path::new(&world_file)),
        ];
        for (referenced_name, _) in checks {
            assert!(
                launch.contains(referenced_name),
                "launch file does not reference `{referenced_name}`"
            );
            assert!(
                dir.join(referenced_name).exists(),
                "referenced file `{referenced_name}` missing from bundle"
            );
            info!(
                "verified reference: launch.py -> {}/{}",
                dir.display(),
                referenced_name
            );
        }

        log_test_footer(
            title,
            Some(start.elapsed()),
            "all launch-file references resolve inside the bundle.",
        );
    }

    #[test]
    fn launch_targets_new_gazebo_not_classic() {
        // Guard specifically against regressions that would re-introduce
        // the classic `gazebo_ros` dependency. The new-Gazebo stack is a
        // project requirement (user instruction 2026-04-18).
        let title = "launch_targets_new_gazebo_not_classic";
        log_test_header(
            title,
            "launch.py must use `gz sim` + ros_gz_* packages exclusively.",
        );
        let start = Instant::now();

        let launch = make_gz_launch_py(ROBOT_NAME, "moveo.urdf", "moveo.world.sdf");
        info!("launch template size: {} bytes", launch.len());

        // Must-have — the new stack.
        for good in ["gz", "ros_gz_sim", "ros_gz_bridge"] {
            assert!(launch.contains(good), "missing `{good}` in launch.py");
        }
        // Must-NOT-have — the classic stack.
        for bad in ["gazebo_ros", "gazebo_ros_pkgs", "libgazebo_ros"] {
            assert!(
                !launch.contains(bad),
                "launch.py must not reference classic `{bad}`"
            );
        }

        log_test_footer(
            title,
            Some(start.elapsed()),
            "launch file exclusively targets the new Gazebo.",
        );
    }
}
