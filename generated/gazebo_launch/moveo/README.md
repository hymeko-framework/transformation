# HyMeKo → new-Gazebo launch bundle — `moveo`

This directory is **generated** by
`hymeko_query/tests/test_gazebo_sim_launch.rs`. It contains everything you
need to spawn the `moveo` robot in the new Gazebo (`gz sim`, not the
old `gazebo-classic`).

## Contents

- `moveo.urdf`  — URDF robot description produced by
  `hymeko_formats::urdf::generate_urdf` from
  `data/robotics/anthropomorphic_arm.hymeko`.
- `moveo.world.sdf`  — Minimal SDF 1.8 world with a ground plane and the
  standard `gz-sim-physics-system` / `-user-commands-system` /
  `-scene-broadcaster-system` plugin triple.
- `gz_sim.launch.py`  — ROS 2 Python launch file that starts `gz sim`,
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
ros2 launch gz_sim.launch.py
```

You should see `gz sim` start up with an empty world, the `moveo`
URDF spawned at the origin, and joint-state / clock topics bridged into
ROS 2 (visible via `ros2 topic list`).
