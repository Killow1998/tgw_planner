# tgw_planner

`tgw_planner` (`tgw_planner`) is a ROS 2 Humble MVP for Unitree Go2-W
multi-floor navigation from either realtime raycast mapping or a static PCD
debug map.

The package keeps the planner core ROS-free under `include/tgw_planner/core` and `src/core`. The Humble wrapper nodes live in `src/humble`.
Static PCD import is handled by `tgw_pcd_import_node`; the legacy
`pcd_to_path_mvp.launch.py` launch file keeps the ROS node name
`tgw_planner_node` for compatibility.

## Build

```bash
cd $ROS_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
```

## Realtime Raycast Mode

Realtime raycast mapping is the recommended path for live or dirty maps because
it uses sensor pose, ray clearing, and temporal evidence instead of treating
every final PCD point as permanent structure.

```bash
source $ROS_WS/install/setup.bash
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/tgw_mapping/points \
  use_tf:=true \
  map_frame:=map \
  sensor_frame:=livox_frame
```

Main debug topics:

- `/tgw_map/occupied_cloud`
- `/tgw_map/free_cloud`
- `/tgw_map/dynamic_suspect_cloud`
- `/tgw_map/static_candidate_cloud`
- `/tgw_map/surface_cloud`
- `/tgw_map/traversable_cloud`
- `/tgw_map/boundary_cloud`
- `/tgw_map/dropoff_boundary_cloud`
- `/tgw_map/wall_boundary_cloud`
- `/tgw_map/clearance_cloud`
- `/tgw_map/medial_axis_cloud`
- `/tgw_map/blocked_cloud`
- `/tgw_map/forbidden_cloud`
- `/planned_path`
- `/planned_path_marker`

Main services:

- `/tgw_mapping/start`
- `/tgw_mapping/stop`
- `/tgw_mapping/pause`
- `/tgw_mapping/clear`
- `/tgw_mapping/save_map`
- `/tgw_mapping/load_map`
- `/tgw_mapping/export_static_pcd`
- `/tgw_mapping/get_snapshot`
- `/tgw_map/plan_path`
- `/tgw_map/set_blocked_region`

When `planner_require_footprint:=true` or
`validation_require_footprint:=true`, `/tgw_map/plan_path` snaps requested
start and goal poses to nearby cells that are both traversable and
footprint-supported before running A*. The realtime node limits this endpoint
projection with `planner_max_snap_distance_m` and defaults to 0.75 m, so an
imprecise click can land on the nearest surface without silently moving to a
distant map layer.

`/tgw_mapping/save_map` writes a realtime map package with per-layer PCD files,
`voxel_evidence.csv`, `blocked_regions.yaml`, `metadata.yaml`, and `stats.json`.
`voxel_evidence.csv` preserves log-odds, hit/miss/ray-pass counts, view counts,
timing, and static/dynamic classifications. `load_map` validates the saved
`resolution_m` before loading so a package is not silently re-quantized with a
different node resolution.

## PCD Debug Mode

```bash
source $ROS_WS/install/setup.bash
ros2 launch tgw_planner pcd_to_path_mvp.launch.py \
  pcd_file:=/absolute/path/to/global_map.pcd \
  robot_radius_m:=0.35 \
  robot_length_m:=0.70 \
  robot_width_m:=0.43 \
  robot_height_m:=0.50 \
  base_to_front_m:=0.20 \
  map_resolution_m:=0.20
```

The planner treats path poses as the odom/base reference point. By default that
reference is 0.20 m behind the front edge of a 0.70 m x 0.43 m x 0.50 m
footprint. A* neighbor expansion checks the rectangular footprint in the local
movement direction instead of accepting a point-mass path.

PCD mode publishes map-build diagnostics on `/map_build_stats` and
`/map_build_stats_json`, including `pcd_artifact_warning=true`. Realtime raycast
mapping is the recommended deployment path for dirty or live maps.

To save a lightweight map package during startup, pass `save_map_dir:=/path/to/map_package`.
The package contains `map.bt`, `metadata.yaml`, `blocked_regions.yaml`, and `README.generated.txt`.

Use RViz2 Publish Point with:

```bash
ros2 service call /rviz_click_router/set_mode tgw_planner/srv/SetClickMode "{mode: start}"
ros2 service call /rviz_click_router/set_mode tgw_planner/srv/SetClickMode "{mode: goal}"
```

RViz includes `Tgw 3D Start` and `Tgw 3D Goal` tools. Select one of those tools,
use the mouse wheel to set the z height, then left-click and drag like the normal
2D pose tools to publish `/start_pose` or `/goal_pose`. The active height is shown
in the RViz status bar and in the tool property named `Z Height`. After publishing,
the planner snaps the pose to the nearest traversable map layer near that XY
location, so the final start/goal marker should sit on the map instead of floating.

Blocked region API:

```bash
ros2 service call /nav_map/set_blocked_region tgw_planner/srv/SetBlockedRegion "{
  operation: 'add',
  min: {x: 1.0, y: 2.0, z: 0.0},
  max: {x: 2.0, y: 3.0, z: 1.0},
  reason: 'manual_forbidden'
}"
```
