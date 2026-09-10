# Decisions and Gotchas

Each entry: what was done, why, and where it will bite you if forgotten.
All of these were verified against real installed source (`strings` on
compiled `.so`/binaries, real header files, or direct empirical testing
against the real Webots node schemas) — not assumed from tutorials, which
is exactly why several of them contradict what a generic ROS2/Nav2
tutorial would tell you to do.

## Controller stack

**`mecanum_drive_controller` is a ChainableController, not the classic
tutorial-era controller.** The installed version
(`ros-humble-mecanum-drive-controller` 2.53.3) has a completely different
parameter schema than most online examples: `front_left_wheel_command_joint_name`
/ `..._state_joint_name` per wheel, `kinematics.wheels_radius`,
`kinematics.sum_of_robot_center_projection_on_X_Y_axis` (= lx + ly from the
wheel geometry), not the old flat `wheel_base`/`wheel_separation`/`wheel_radius`.
Get the real schema from `/opt/ros/humble/include/mecanum_drive_controller/mecanum_drive_controller_parameters.hpp`
if it ever needs revisiting.

**It does not subscribe to `/cmd_vel`.** Commands go to
`<controller_name>/reference_unstamped` (`geometry_msgs/Twist`, since
`use_stamped_vel: false`) or `.../reference` (`TwistStamped`) if that flag
is `true`. `teleop.launch.py` and the navigation stack's velocity output
must target `/mecanum_drive_controller/reference_unstamped`.

**It cannot publish TF**, regardless of `enable_odom_tf`. See
architecture.md's TF section. `enable_odom_tf: false` is deliberate, not
an oversight.

**Controller update-rate/Webots timestep mismatch is a real but harmless
warning.** `update_rate: 50` (20ms) vs. Webots' 32ms `basicTimeStep`
triggers `Ros2Control.cpp`'s own `RCLCPP_WARN_STREAM` every launch. It's
cosmetic — confirmed in source it never blocks or delays anything.

**Mecanum physics requires mirrored asymmetric friction.** 
Applying a single `<contactMaterial>` with a 45-degree rotation to all four wheels collapses the mecanum force decomposition (all diagonal slip vectors point the same way), causing the robot to "ice skate" diagonally instead of rotating in place.
Fixed by creating two Webots `ContactProperties` blocks (`wheel_pos_45` at `+0.785398` rad, and `wheel_neg_45` at `-0.785398` rad) and mapping FL/RR to the positive material and FR/RL to the negative material. This relies on `fix_proto.py` to inject the `contactMaterial` fields into the raw PROTO file, because `urdf2webots` ignores them.

**Starting torque stiction stalls.**
High static URDF `<dynamics damping="0.1" friction="0.05"/>` stalled the simulated motors. Reduced to `0.01` and added explicit `<limit effort="30.0" velocity="15.0"/>` to `wheel.urdf.xacro` to un-stall the PID. Reduced `coulombFriction` in `simulation.wbt` from 3.5 to 0.85 to stop wheel dragging.

**Twist Multiplexing.**
Multiple nodes (`teleop`, `nav2`, `lidar_docker.py`) command velocity. They all map to `/cmd_vel_<source>` which `twist_mux` prioritizes and forwards to `/mecanum_drive_controller/reference_unstamped`. Do NOT command the controller topic directly anymore.

## EKF (robot_localization)

**`ekf_node` is the sole `odom -> base_link` TF publisher** (see
architecture.md). `publish_tf: true` here matters a lot.

**`predict_to_current_time: true` crashes `ekf_node` (SIGSEGV) under live
sensor data**, on the installed version (robot_localization 3.5.4-1jammy).
Confirmed it does NOT crash standalone with zero sensor input (ran fine
15s under `gdb` with no publishers), so it's specifically the live
prediction path that's broken. Currently commented out in `ekf.yaml`. If
you want this feature, test it in isolation first, ideally after an apt
upgrade to see if a newer point release fixes it. A later task asked to
re-enable it (to fix a supposed TF extrapolation race with AMCL on boot)
and was declined for this reason — re-verified against this same evidence
rather than re-tested live, since the crash was already reproduced once.

**`transform_time_offset` is not a real settable parameter on this
installed version.** A task asked to add `transform_time_offset: 0.1` to
future-date the published TF. Checked `ros_filter.hpp`: there's an
internal `tf_time_offset_` member, but it's computed internally, not
exposed as a ROS parameter — confirmed by `strings` on the actual
`ekf_node` binary, which contains zero occurrences of `offset` anywhere.
Setting it in `ekf.yaml` would not error, it would just be silently
ignored (undeclared parameter override), so it can't provide the effect
it was requested for. Not applied.

## AMCL

**`base_frame_id` must be `base_link`**, not the tutorial-default
`base_footprint` (see architecture.md).

**`robot_model_type` must be explicitly set to `"nav2_amcl::OmniMotionModel"`.**
The default is `nav2_amcl::DifferentialMotionModel`, which assumes
forward/rotate-only motion and cannot model this robot's mecanum strafing
at all. Confirmed `OmniMotionModel` is a real registered plugin via
`/opt/ros/humble/share/nav2_amcl/plugins.xml`.

**`set_initial_pose: true` is REQUIRED alongside the `initial_pose:` block**
for AMCL to actually use it on startup. This is easy to miss: AMCL's own
parameter description string (visible via `strings` on `libamcl_core.so`)
says the `initial_pose*` parameters only take effect "with parameter
`set_initial_pose: true`" — without that flag they're silently ignored and
AMCL still waits for a manual "2D Pose Estimate" in RViz.

**`initial_pose` is expressed in the MAP frame, not Webots' world frame —
these are two different coordinate systems and are not expected to
numerically match.** `simulation.wbt`'s `amr_robot` spawn `translation`/
`rotation` (`1.27 0.38 0.104`, yaw `3.14159`) is where the robot boots up
in Webots' world coordinates; `amcl.yaml`'s `initial_pose`
(`x: -0.030, y: 0.357, z: 0.0, yaw: -0.027`) is the verified pose of that
same boot-time position relative to `room_map`'s origin. Don't try to make
the two match numerically — if the robot's real position relative to the
saved map ever changes (new spawn point, new map, hand-moved robot before
boot), re-verify and update `initial_pose` independently of
`simulation.wbt`.

**`transform_tolerance` raised from `1.0` to `1.5`** to widen AMCL's
allowed TF lookup delay window on boot, as part of the same "TF
extrapolation race with EKF on boot" task above (the `ekf.yaml` half of
that task was declined — see EKF section — but this half is a real,
already-used AMCL parameter and a safe, low-risk widening on its own).

## Nav2 stack (Milestone 3: costmaps, planner, controller, behaviors, BT)

**Several plugin/parameter names in the original task spec were wrong for
this installed version (Nav2 1.1.20, Humble) — all verified by `strings`
on the real compiled `.so`/binaries and the real `plugins.xml` files
before writing `nav2_params.yaml`, not assumed:**

- `nav2_controller`'s progress/goal checkers register only a `type=`
  (C++ namespace form, `nav2_controller::SimpleProgressChecker` /
  `nav2_controller::SimpleGoalChecker`) — no `name=` alias. The slash form
  (`nav2_controller/SimpleProgressChecker`) does not exist and fails to load.
- DWB's `StandardTrajectoryGenerator` sample-count parameters are
  `vx_samples` / `vy_samples` / `vtheta_samples` (confirmed via `strings`
  on `libstandard_traj_generator.so`) — **not** `vel_xs_samples` /
  `vel_ys_samples` / `vel_thetas_samples`. The wrong names would have been
  silently ignored (undeclared parameter override, no error), leaving DWB
  on its own default sample counts and never actually widening Y-velocity
  sampling — directly defeating the point of tuning DWB for mecanum strafing.
- `nav2_behaviors`'s registered plugin name is `nav2_behaviors/BackUp`
  (capital U — confirmed via `behavior_plugin.xml`), not `.../Backup`.
- `behavior_server` has no `local_frame` parameter at all (confirmed via
  `strings` on `libbehavior_server_core.so` — only `global_frame` exists).
  It was also set to `map` in the task spec; changed to `odom`, since
  Spin/BackUp execute short safety-critical motions against the local
  costmap/odometry (the same frame `local_costmap` itself operates in),
  not the slower map-frame AMCL estimate.
- `bt_navigator` has no `odom_frame` parameter (confirmed via `strings` on
  `libbt_navigator_core.so` — only `odom_topic`, a topic name not a TF
  frame, exists). Set to `/odometry/filtered` (the EKF-fused output — see
  architecture.md's node/data flow), not the raw
  `/mecanum_drive_controller/odometry`.
- `bt_navigator`'s real parameter for the default behavior tree file is
  `default_nav_to_pose_bt_xml`, not `default_bt_xml_filename` (also
  confirmed via `strings`). Left unset entirely in `nav2_params.yaml`:
  the requested file (`navigate_to_pose_w_replanning_and_recovery.xml`)
  is already `bt_navigator`'s own built-in default when this parameter is
  absent, so setting a wrong-named key would have been a silent no-op
  anyway rather than an active fix.

**`nav2_params.yaml`'s `amcl:` block intentionally duplicates the
standalone `amcl.yaml`, in full** — the task that created this file
listed only `set_initial_pose`/`initial_pose`/`transform_tolerance` there,
which (if used as written) would have silently dropped
`robot_model_type: nav2_amcl::OmniMotionModel` and reverted AMCL to the
default `DifferentialMotionModel` for the whole Nav2-stack launch path,
reintroducing the exact bug fixed in Milestone 2. If AMCL tuning changes,
update both `amcl.yaml` and `nav2_params.yaml`'s `amcl:` block.

**`controller_server` always publishes on `cmd_vel`, remapped in
`navigation.launch.py`** to `/mecanum_drive_controller/reference_unstamped`
— the same remap teleop needs, for the same reason (see Controller stack
section above). Without it Nav2 computes velocities that never reach the
wheels.

**`local_costmap`'s `width`/`height` must be plain integers, not
decimals.** `Costmap2DROS` pre-declares these as int-typed parameters in
its C++ constructor; ROS 2's strict parameter typing then rejects
overwriting an already-declared `int` with a `double` at load time.
Reproduced live: `width: 2.5` / `height: 2.5` crashed `controller_server`
on launch (`InvalidParameterTypeException`, "is of type {integer}, setting
it to {double} is not allowed") — the exact same bug class as the earlier
AMCL `initial_pose` crash, just inverted (there the fix was decimals;
here it's whole numbers). Confirmed against the real
`webots_ros2_turtlebot` reference config, which also uses plain integers
for these two fields. Fixed to `width: 3` / `height: 3` — still divided
internally by `resolution` to get cell counts, so nothing is lost.

**`behavior_server` needs a `"wait"` plugin even if nothing calls it
directly.** `bt_navigator` validates BOTH its default behavior trees on
activation - `navigate_to_pose` AND `navigate_through_poses` - not just
the one actual Nav2 goals use. The through-poses default tree contains a
`Wait` BT node, which needs the `wait` action server from
`behavior_server`. `behavior_plugins: ["spin", "backup"]` (no `wait`) left
that action server missing entirely, which failed BT validation and
aborted the whole lifecycle bringup - reproduced live: `"wait" action
server not available after waiting for 1.00s`, then
`lifecycle_manager_navigation` fails to activate `bt_navigator` and aborts
bringup for the whole stack. Fixed by adding `wait: {plugin:
"nav2_behaviors/Wait"}` to `behavior_plugins`. Matches the real
`webots_ros2_turtlebot` reference config, which also includes `wait` for
this exact reason.

**Both costmaps use an exact rectangular `footprint`, not `robot_radius`**
(for navigating tight 353mm doorways). Corner points are this robot's own
real chassis bounding box measured from `chassis.stl` (X: -0.1759..0.1165,
Y: -0.0824..0.0826 - see `base.urdf.xacro`'s mesh-vertex comment, rounded
to ±0.0825 for symmetry), not a generic vendor spec despite an unrelated
task description calling it a "RoboMaster X3" footprint - the numbers
matched our own robot exactly, so applied as given. `footprint` is parsed
as a STRING by `nav2_costmap_2d` (confirmed via `strings` on
`libnav2_costmap_2d_core.so` - `makeFootprintFromString` takes a
`std::string`), so the quoted `"[ [x,y], ... ]"` syntax is correct, not a
mistaken YAML sequence. Setting `footprint` supersedes `robot_radius`,
which was removed from both costmaps rather than left stale alongside it.

**Inflation tightened for the same reason: `cost_scaling_factor: 3.0 →
15.0`, `inflation_radius: 0.45 → 0.20`.** A 353mm doorway against this
robot's ~165mm Y-width leaves only ~94mm clearance per side - less than
the old 0.45m inflation radius, which would have saturated the entire
doorway with near-maximum cost from both jambs simultaneously. The
steeper `cost_scaling_factor` makes cost decay fast enough that the
doorway center should stay well under lethal, but this is a tuning
prediction, not something verified by an actual live traversal - watch
for the robot refusing to path through narrow gaps and reconsider these
two values first if it does.

**The `RotateToGoal` and `GoalAlign` DWB critics are differential-drive assumptions** (force
rotate-in-place toward the goal heading before approaching). Removed them because they fight 
this robot's ability to strafe directly at a goal. Only `Oscillation`, `BaseObstacle`, `PathDist`, 
`GoalDist`, and `PathAlign` are used. `PathAlign` specifically prevents crab-walking by keeping 
the robot body tangent to the planned path.

**Stop-Turn-Drive elimination.**
`smooth_path: true` is set in the `GridBased` global planner to generate fluid approach curves 
instead of sharp, non-holonomic pivot corners. DWB's `vy_samples` is raised to `20` and 
`min_vel_y`/`max_vel_y` are clamped to `±0.35` for dense, stable lateral motion sampling.

**Controller noise floor and goal jitter.**
`min_x_velocity_threshold`, `min_y_velocity_threshold`, and `min_theta_velocity_threshold` 
are raised from `0.001` to `0.01` to suppress actuator chatter. `SimpleGoalChecker` is tuned 
tight (`0.08`m / `0.15`rad) with `stateful: true` to enable clean hysteresis for goal settling 
without wiggling.

## EKF / IMU wiring (live-confirmed bugs, 2026-08-27)

**`Ros2IMU` publishes empty `frame_id` unless `<frameName>` is set.**
`sim_control.urdf.xacro`'s `<plugin type="webots_ros2_driver::Ros2IMU">` block
previously had no `<frameName>` tag. At runtime `header.frame_id` was `''`
(confirmed via `ros2 topic echo /imu/data --once --field header`).
`robot_localization`'s EKF cannot look up a sensor with no frame in the TF tree,
so all IMU measurements were silently dropped — the filter ran on odometry alone
and had no yaw correction, which is why rotation drift was visible.
Fixed: added `<frameName>imu_link</frameName>` to the plugin block.
This must match the URDF link name (`frame_id + "_link"` as expanded by the
`xacro:imu` macro in `amr.urdf.xacro` — currently `imu_link`).

**`linear_acceleration` from `Ros2IMU` is `NaN` in this Webots configuration.**
Live inspection (`ros2 topic echo /imu/data --once`) showed
`linear_acceleration.x: NaN`, `linear_acceleration.y: NaN`.
The Webots Accelerometer device is not delivering valid data to `Ros2IMU`
(likely because its sampling enable is tied to the Webots simulation step in
a way that isn't active during this launch configuration).
Fusing NaN into the EKF's state vector corrupts the entire covariance matrix and
produces erratic or locked `odom → base_link` output — the filter appeared to run
(no crash, no error) but its `x` and `y` estimates were frozen.
Fixed: disabled the `ax`/`ay` rows in `ekf.yaml`'s `imu0_config` (set both to
`false`). Only orientation (yaw) and angular velocity (vyaw) from the
InertialUnit and Gyro are fused — both were confirmed valid. If the accelerometer
is ever brought up properly, re-enable the last row of `imu0_config`.

**Odometry covariance was all zeros — EKF numerical degeneracy.**
`controllers.yaml`'s `twist_covariance_diagonal` and `pose_covariance_diagonal`
were `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`. `robot_localization` interprets a zero
diagonal as infinite certainty (zero uncertainty), which causes the Kalman gain
to degenerate — the filter either locks on the first measurement and never
updates, or produces tiny covariance values that cause numerical instability.
Fixed: set non-zero diagonals `[0.001, 0.001, 0.0, 0.0, 0.0, 0.005]` (x, y
position ~1mm uncertainty, yaw ~0.005 rad). Z/roll/pitch are left 0.0 because
`two_d_mode: true` ignores those axes anyway.

**RF2O Direct TF Publishing Architecture (2026-08-28).**
To eliminate IMU frame/covariance singularities and simplify dead-reckoning:
- EKF (`ekf_node`) disabled in `sim.launch.py`.
- `rf2o_laser_odometry` configured with `publish_tf: true` in `rf2o_params.yaml`, making RF2O the direct publisher of `odom -> base_link` TF transforms.
- Nav2 stack (`bt_navigator`, AMCL) configured to use `/rf2o/odom` and `odom -> base_link` transform tree directly.

**SmacPlanner2D & Webots Friction ContactProperties (2026-08-28).**
- `planner_server` configured with `nav2_smac_planner/SmacPlanner2D` and `smooth_path: false` to force straight-line global path planning and sharp grid-aligned turns.
- Webots `simulation.wbt` `WorldInfo` updated with `contactProperties` (material `rubber` on `default`, `coulombFriction: 1.5`) for physical traction.

**Instant Wiggle-Free SimpleGoalChecker (2026-08-28).**
- Configured `general_goal_checker` to `nav2_controller::SimpleGoalChecker` with `xy_goal_tolerance: 0.05` (5 cm spatial radius) and `yaw_goal_tolerance: 3.14` (180 deg). This terminates the path instantly upon entering the spatial radius without rotational micro-adjustments or wiggling.

## Lidar

**Two independent self-occlusion bugs, both had to be fixed, neither
alone was sufficient:**
1. Mount height: `laser_frame`'s xacro offset (`amr.urdf.xacro`) was
   originally `0.0825`, inherited from the source repo without
   re-measuring against *our* actual `chassis.stl` (whose real top is
   `z=0.1905`, measured directly from the mesh triangles). The sensor sat
   embedded inside the chassis body. Raised to `0.22`.
2. Even after that fix, all 360 rays still returned `.inf`. Root cause:
   `lidar.stl`'s own bounding box straddles zero on all 3 axes — the
   visualization dome mesh fully encloses its own local origin. Fixed by
   giving the `Lidar` node itself (not the mesh) a further
   `translation 0 0 0.03` inside `fix_proto.py`, so its ray-origin sits
   above the dome's own solid body.

**`type` must be `"rotating"`, not the schema default `"fixed"`.** Matches
the real RPLidar S2 hardware and Webots' own official `lidar.wbt` sample.
(An earlier theory that `"fixed"` silently clamps `fieldOfView` turned out
to be wrong — see the `--full-length` note below — but `"rotating"` is
still the technically correct choice for this hardware regardless.)

**`ros2 topic echo` truncates arrays to 128 elements by default.** Burned
us once: a "the Lidar only returns 129 samples out of 360" diagnosis was
actually just this truncation (`ros2 topic echo /scan --once` vs. the
correct `--full-length` or `--truncate-length N`). Always use
`--full-length` when inspecting `/scan` ranges.

**Lidar `frameName` must be set explicitly.** `webots_ros2_driver`'s
`Ros2SensorPlugin::init()` defaults a device's ROS `frame_id` to the
Webots device's own internal name (`"Lidar"`) unless `<frameName>` is
given in the `<device>` block's `<ros>` config
(`sim_control.urdf.xacro`). Since our TF frame is `laser_frame`, not
`Lidar`, omitting this means the LaserScan's `header.frame_id` can never
resolve a transform — `tf2`'s `MessageFilter` queues messages forever and
eventually crashes the subscriber (RViz or slam_toolbox) under sustained
backpressure ("Message Filter dropping message ... queue is full", then a
segfault). If you see that exact log pattern again on a *different*
topic/frame, check for this same missing-`frameName` class of bug first.

## RViz

**Never point a second display at `/scan` with a different message
type.** `/scan` is `LaserScan`. The point cloud is a genuinely separate
topic, `/scan/point_cloud` (`PointCloud2`) — confirmed in `Ros2Lidar.cpp`,
it publishes both on different topics, never the same one twice. Adding a
`PointCloud2` display pointed at `/scan` by mistake corrupts RViz's DDS
subscription state and crashes it (`invalid allocator`, SIGABRT). This
has happened more than once in this project — the saved
`viewport_config.rviz` is correct (one `LaserScan` display on `/scan`);
the crash comes from ad-hoc displays added live in the GUI.

**`Map` display: the default `map` color scheme crashes RViz (SIGKILL)**
on this machine's Mesa Intel driver — a GLSL shader link failure
(`indexed_8bit_image.frag`, "active samplers with a different type refer
to the same texture image unit"). Confirmed hardware-accelerated
(Mesa 23.2.1, Intel Xe, direct rendering — not a software-render
fallback issue). Workaround: set the `Map` display's `Color Scheme`
property to `costmap` instead of `map`. Unconfirmed whether this is
specific to this machine's Mesa version or a wider bug.

**`Fixed Frame` in `viewport_config.rviz` must be `base_link`**, not
`base_footprint` (see architecture.md) — it was originally
`base_footprint` and silently broke everything under `use_webots:=true`.

## Teleop

**`teleop_twist_keyboard` cannot run via `ros2 launch`.** It calls
`termios.tcgetattr(sys.stdin)` to enter raw keyboard mode, which requires
a real TTY. `ros2 launch`'s `Node` action pipes the child's stdin through
something that isn't a TTY (confirmed: `errno 25, ENOTTY`). The official
`turtlebot3_teleop` package ships no launch file at all for this exact
reason. Run it directly:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mecanum_drive_controller/reference_unstamped
```
`teleop.launch.py` still exists to document the correct remap, but will
always fail the same way if launched via `ros2 launch`.

**This robot is holonomic (mecanum)** — the Shift-held key variants in
`teleop_twist_keyboard` (`J`/`L`/`I`/`K` etc.) produce genuine sideways
strafing, which a differential-drive robot can't do. Worth knowing when
driving it for SLAM coverage.

## Process hygiene (testing discipline, not a code bug)

**`timeout N ros2 launch amr_simulation sim.launch.py` for automated
testing leaves orphaned processes.** `timeout` only signals the `ros2
launch` Python process; the real Webots app (`webots-bin`), its
`ros2_supervisor.py`, and the `webots_ros2_driver` process don't
reliably die with it, and pile up fighting over the same IPC port
(1234) across repeated test runs, causing spurious-looking failures
that have nothing to do with the code under test. If you must
script-test a launch, clean up explicitly afterward:
```
pkill -9 -f "webots-bin"; pkill -9 -f "/usr/local/webots/webots"
pkill -9 -f "ros2_supervisor.py"; pkill -9 -f "webots_ros2_driver"
pkill -9 -f "controller_manager/spawner"
rm -rf /tmp/webots/<username>
```
Prefer asking a human to run and observe a live launch over repeated
automated `timeout`-wrapped test cycles — it's disruptive to a real
GUI session and easy to leave in a broken shared state.

## Map file location

Saved maps go to `~/ros/maps/` (e.g. `~/ros/maps/room_map.yaml` +
`.pgm`), via `ros2 run nav2_map_server map_saver_cli -f ~/ros/maps/room_map`.
This is a project-specific choice, not a Nav2 default — some
generic instructions elsewhere may say `~/map_files/`; that directory
does not exist in this project.
