# AMR Project — Context Index

This folder exists so a new agent (or a human) picking up this workspace
cold can get oriented without re-deriving hours of hard-won debugging.
Read these in order the first time; after that, use them as reference.

1. **architecture.md** — packages, TF tree, node/data flow, topic map.
2. **decisions-and-gotchas.md** — the non-obvious choices and bugs that
   will bite you again if you don't know about them. Read this before
   touching `controllers.yaml`, `ekf.yaml`, `amcl.yaml`, or any sensor
   xacro.
3. **proto-regeneration.md** — the manual pipeline that turns the URDF
   into the Webots PROTO. This is NOT part of `colcon build` — if you
   change any xacro file under `amr_description/urdf/`, you must run this
   pipeline by hand or your changes will never reach Webots.
4. **launch-reference.md** — what each launch file starts, in what order,
   and which ones depend on which.

## What this project is

A 4-wheel mecanum AMR, structurally cloned from
`automaticaddison/yahboom_rosmaster` (ROSMASTER-X3 dimensions), simulated
in Webots R2025a on ROS 2 Humble. Three packages:

- **`amr_description`** — URDF/xacro, meshes, RViz config. Single source
  of truth for the robot's physical model. Builds for two targets from
  the same xacro tree via the `use_webots` arg (see architecture.md).
- **`amr_simulation`** — Webots world/PROTO, `ros2_control` bring-up, EKF,
  SLAM, teleop. This is the "make it move in Webots" package.
- **`amr_navigation`** — AMCL localization against a saved static map.
  This is the "make it know where it is" package. (`fleet_control.launch.py`
  and `slam.launch.py` here are currently empty placeholders for future
  work, not implemented yet — don't assume they do anything.)

## Current status (as of this writing)

Working: gravity/physics, mecanum driving, Lidar/IMU/camera sensors, TF
tree via EKF, SLAM mapping, AMCL localization with auto-initialized pose.

Known-open items — check these before assuming they're fixed:
- `predict_to_current_time` in `ekf.yaml` is disabled (commented out) —
  it reproducibly crashed `ekf_node` (SIGSEGV) under live sensor data on
  the installed `robot_localization` 3.5.4-1jammy. See
  decisions-and-gotchas.md.
- RViz's `Map` display crashes with the default `map` color scheme on
  this machine's Mesa Intel driver (GLSL shader link failure). Must use
  `costmap` color scheme instead. Unconfirmed whether this is
  machine-specific or a wider Mesa/Ogre bug.
