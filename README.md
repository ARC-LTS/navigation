# navigation (`clearpath_nav2_demos`) — Nav2 / SLAM config for RaMonES

This package provides the **Nav2 navigation stack configuration, SLAM, localization,
and maps** for the robot platform used by RaMonES. It is a fork of Clearpath Robotics'
`clearpath_nav2_demos` package, carrying per-platform Nav2 parameter sets plus
RaMonES-specific tuning and a custom 3D-LiDAR SLAM launch.

Role in the project: RaMonES is a closed-loop radiation-aware explorer (dissertation
Chapter 5). The `explore_lite` node (see the `explore-ros2` package) issues
`navigate_to_pose` goals; **this package configures the Nav2 planner/controller/costmap
servers that execute those goals**, and the SLAM/localization that produces the map and
`map → odom → base_link` TF the explorer and costmaps run on. Crucially, the **global
costmap here carries the radiation costmap layer** that turns the GP radiation field
into navigation cost at the planning level.

`package.xml` name: **`clearpath_nav2_demos`**. It is a pure
config/launch/maps package (`ament_cmake`, `CMakeLists.txt` only installs
`config/`, `launch/`, `maps/` to `share/`; no compiled nodes). Runtime dependencies:
`clearpath_config`, `nav2_bringup`, `rtabmap_odom`, `rtabmap_slam`, `tf2_ros`.

---

## 1. Layout

```
config/<platform>/{nav2,localization}.yaml        # per Clearpath platform
launch/{nav2,slam,localization,rtabmap}.launch.py
maps/warehouse.{pgm,yaml}
```

Per-platform config directories exist for the Clearpath line:
`a200` (Husky), `dd100/dd150` (Dingo-D), `do100/do150` (Dingo-O),
`j100` (Jackal), `r100` (Ridgeback-like), `w200` (Warthog).
**RaMonES runs on the Husky A200, so `config/a200/` is the active configuration.**

The launch files select the platform directory **dynamically**: `nav2.launch.py` and
`localization.launch.py` read the Clearpath `robot.yaml` (via `clearpath_config`) to
discover the platform model, then load `config/<model>/nav2.yaml` (or
`localization.yaml`) and include the corresponding `nav2_bringup` launch. So the same
launch file serves every platform; the YAML that actually takes effect is the one for
the model declared in `robot.yaml`.

---

## 2. Launch files

| File | What it starts | Key args (defaults) |
|---|---|---|
| `nav2.launch.py` | Includes `nav2_bringup/navigation_launch.py` with `config/<model>/nav2.yaml` (planner, controller, costmaps, BT navigator, recoveries, waypoint follower) | `use_sim_time=true`; `setup_path=$RAMONES_SETUP_PATH` or `~/clearpath/` (directory holding `robot.yaml`) |
| `slam.launch.py` | **Custom RTAB-Map 3D-LiDAR SLAM**: `rtabmap_odom/icp_odometry` + `rtabmap_slam/rtabmap` on `/sensors/lidar3d_0/points`, started after a 5 s `TimerAction`. Renames an existing `/root/.ros/rtabmap.db` on launch (a no-op unless running as root; RTAB-Map's `-d` flag discards the previous database regardless) | `use_sim_time=true`; `output=log` |
| `rtabmap.launch.py` | Earlier/alternate RTAB-Map config (looser ICP tolerances, `expected_update_rate=60`, publishes a VLP16 `base_link→velodyne` static TF, `odom` remapped to `/icp/odom`) | `use_sim_time=true`; `deskewing=true` |
| `localization.launch.py` | Includes `nav2_bringup/localization_launch.py` with `config/<model>/localization.yaml` (AMCL + map_server) against a prebuilt map | `use_sim_time=false`; `setup_path=$RAMONES_SETUP_PATH` or `~/clearpath/`; `map=maps/warehouse.yaml` |

> **Note on SLAM:** the custom `slam.launch.py` runs **RTAB-Map on the 3D LiDAR**
> (`/sensors/lidar3d_0/points`), not `slam_toolbox`. The upstream `slam_toolbox`
> `config/<model>/slam.yaml` files were never referenced by any launch file and have been
> removed. The RTAB-Map parameters live inline in the launch files, not in YAML.
> `icp_odometry` is the **sole** publisher of `odom → base_link`: the Clearpath setup
> directory must have `enable_ekf: false` in its generated `platform-service.launch.py`
> (the frozen campaign config does). A second publisher such as the platform EKF fusing
> skid-steer wheel odometry corrupts the map whenever the robot rotates in place. `slam.launch.py` and `rtabmap.launch.py`
> both start RTAB-Map; `slam.launch.py` is the current, more conservatively tuned one
> (tighter ICP correspondence, `Grid/RangeMax 8.0`, longer sim-time TF waits).

`setup_path` (the Clearpath setup directory containing `robot.yaml`) resolves, in
order: an explicit `setup_path:=` launch argument, then the `RAMONES_SETUP_PATH`
environment variable, then `~/clearpath/`. For RaMonES runs, point
`RAMONES_SETUP_PATH` at the frozen campaign config that the `ramones_ws` meta-repo
provides, so that Nav2, the simulator and the robot description all read the same
`robot.yaml`.

---

## 3. Active Nav2 configuration — `config/a200/nav2.yaml`

Values below are extracted verbatim from the file. RaMonES-relevant settings are
flagged.

### Costmaps (the RaMonES-critical part)

Shared footprint on both costmaps: `[[0.55,0.45],[0.55,-0.45],[-0.55,-0.45],[-0.55,0.45]]`
(≈1.1 m × 0.9 m Husky), resolution `0.05 m`. Obstacle source is the 3D LiDAR
`/sensors/lidar3d_0/points` (`PointCloud2`, heights 0.1–0.8 m, raytrace/obstacle max
range 8 m).

**Local costmap** (rolling 8×8 m window, `update_frequency 5.0`):
* plugins: `static_layer, obstacle_layer, inflation_layer, denoise_layer`
* `inflation_layer`: `cost_scaling_factor 5.0`, **`inflation_radius 0.4`** (m)
* **The `radiation_layer` is deliberately ABSENT from the local costmap.** Having it
  there made DWB treat the diffuse radiation field as physical obstacles, leaving no
  valid trajectories and freezing the robot. Radiation cost is applied **only at the
  global planning level.**

**Global costmap** (non-rolling, `track_unknown_space: true`, `update_frequency 1.0`,
`footprint_padding 0.1`):
* plugins: `static_layer, obstacle_layer, inflation_layer, radiation_layer, denoise_layer`
* `inflation_layer`: `cost_scaling_factor 5.0`, **`inflation_radius 0.5`** (m)
* **`radiation_layer`** (`rad_costmap_plugin::RadiationLayer`, an external RaMonES
  costmap plugin):
  * `heatmap_topic: /rad_heatmap/gp_mean_map` (GP mean field)
  * `warning_threshold_mrem_hr: 1.5` (above the ~1.0 background max, so background noise
    doesn't inflate every cell)
  * `evacuation_threshold_mrem_hr: 8.0` (aligned with the `rad_safety` emergency
    threshold so costmap and safety system agree on "dangerous")

The **~0.4 m local / 0.5 m global inflation radius** is a RaMonES tuning ("navA"
adoption): the previous values (0.75 local / 1.0 global) kept 1.25 m corridors fully
inside the inflated band; dropping to 0.4/0.5 broke the coverage ceiling with ~5× fewer
stuck triggers. The pre-navA file is preserved in git history as
`config/a200/nav2.yaml.pre_navA_backup` on the `handoff` branch.

### Controller — `controller_server`

* `controller_frequency: 15.0`, `FollowPath` plugin = **`dwb_core::DWBLocalPlanner`** (DWB).
* Velocity limits: `max_vel_x 1.56`, `max_speed_xy 1.2`, `max_vel_theta 1.0`,
  `acc_lim_x 3.0`, `acc_lim_theta 3.2`; non-holonomic (`vy_samples 1`, lateral motion
  disabled).
* `general_goal_checker` (`SimpleGoalChecker`): `xy_goal_tolerance 0.35`,
  `yaw_goal_tolerance 0.5`, stateful.
* Progress checker: `SimpleProgressChecker`, `required_movement_radius 0.1`,
  `movement_time_allowance 12.0`. `failure_tolerance: 1.0`.
* DWB critics: `RotateToGoal, BaseObstacle, ObstacleFootprint, GoalAlign, PathAlign,
  PathDist, GoalDist`. **RaMonES tuning:** `BaseObstacle.scale` and
  `ObstacleFootprint.scale` were reduced from `500.0` to **`25.0`** (same order as the
  path-following critics `PathAlign 50 / PathDist 32 / GoalDist 24 / GoalAlign 12`) so
  DWB stops over-penalizing trajectories brushing the inflation gradient and will take
  tight-but-passable aisles.

### Planner — `planner_server`

* `GridBased` = **`nav2_smac_planner/SmacPlanner2D`**, `tolerance 0.5`,
  `allow_unknown: true`, `downsample_costmap: false`, `max_iterations 1000000`,
  `max_planning_time 5.0`, `cost_penalty 50.0`, `retrospective_penalty 0.015`.
  (A commented-out `SmacPlannerHybrid` block is kept as an alternative.)

### Other servers

* `bt_navigator`: navigators `navigate_to_pose` + `navigate_through_poses`,
  `global_frame map`, `robot_base_frame base_link`, `odom_topic /odometry`,
  Groot monitoring enabled (ports 1666/1667).
* `recoveries_server`: `spin`, `backup`, `wait` (`max_rotational_vel 1.0`).
* `waypoint_follower`, `map_server`/`map_saver` (occupied 0.65 / free 0.25 defaults).
* `use_sim_time: True` throughout.

---

## 4. SLAM / localization config (a200)

* SLAM is RTAB-Map, configured inline in `launch/slam.launch.py` (see §2).
* `config/a200/localization.yaml` — AMCL (`DifferentialMotionModel`, 500–2000 particles,
  `scan_topic sensors/lidar2d_0/scan`, `laser_max_range 100`) + `map_server`/`map_saver`,
  used by `localization.launch.py` against a prebuilt map.

---

## 5. Map

`maps/warehouse.{pgm,yaml}` — the prebuilt occupancy map used by
`localization.launch.py` (default `map` arg). `warehouse.yaml`: `resolution 0.03`,
`origin [-15.1, -25, 0]`, `occupied_thresh 0.65`, `free_thresh 0.1`, mode `trinary`.

---

## 6. How the stack consumes this package & how to run

Within RaMonES, the global costmap configured here subscribes to the GP mean field
(`/rad_heatmap/gp_mean_map`) through the `radiation_layer`, so the Smac global planner
routes around high-dose regions, while `explore_lite` separately reads the GP mean/std
*images* for frontier costing. Nav2 executes the explorer's `navigate_to_pose` goals;
SLAM (RTAB-Map) supplies `map`/TF.

Build & run:

```bash
cd <ros_ws>
colcon build --packages-select clearpath_nav2_demos
source install/setup.bash

# SLAM (RTAB-Map on 3D LiDAR) + Nav2; override setup_path to your robot.yaml dir
ros2 launch clearpath_nav2_demos slam.launch.py
ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=/path/to/clearpath/

# or localization against the prebuilt map instead of SLAM
ros2 launch clearpath_nav2_demos localization.launch.py \
     map:=<ws>/src/navigation/maps/warehouse.yaml setup_path:=/path/to/clearpath/
```

In practice the full RaMonES simulation/orchestration launch starts SLAM, Nav2, the GP
heatmap pipeline, and the explorer in the correct order; these launch files are the Nav2
layer of that sequence.

---

## License / attribution

BSD. Upstream `clearpath_nav2_demos` by Clearpath Robotics (Roni Kreinin et al.);
radiation-costmap integration and navigation tuning by the RaMonES project.
