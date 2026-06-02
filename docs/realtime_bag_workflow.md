# Realtime Bag Mapping Workflow

This workflow is for validating `tgw_planner` realtime raycast mapping with a
recorded Go2-W bag. Source the workspace overlays needed by `fast_lio`,
`n3mapping`, and `tgw_planner` before launching.

Example startup sequence:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch n3mapping mapping.launch.py
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/tgw_mapping/points \
  map_frame:=map \
  use_tf:=true
ros2 bag play f7tof9_g2w_ros2/
```

The exact `points_topic` should match the point cloud topic produced by the
active FAST-LIO / n3mapping pipeline. If the point cloud is already published in
the map frame and no sensor TF is available, use:

```bash
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=<cloud_topic> \
  map_frame:=map \
  use_tf:=false \
  assume_cloud_in_map_frame:=true
```

Realtime surface extraction defaults to
`surface_require_observed_free_space:=true` and
`surface_allow_observed_free_bridge:=true`. The extractor builds static-support
surface candidates with head clearance, then accepts only candidate components
that connect to an observed-free or observed-clearance anchor. This allows
continuous floors and stairs to survive sparse ray-clearing evidence while still
rejecting candidate components that have no observed free-space support. Use
`surface_require_observed_free_space:=false` only for clean static PCD-style
smoke tests that do not contain free-space evidence.

Synthetic realtime regression:

```bash
src/tgw_planner/scripts/run_realtime_mapping_sim_tests.sh
```

This regression also exercises `/tgw_mapping/start`, `/tgw_mapping/stop`,
`/tgw_mapping/pause`, and `/tgw_mapping/clear`: it launches with
`start_enabled:=false`, verifies that point clouds are not integrated while
stopped, verifies that `/start` resumes integration, verifies that `/pause` plus
`/clear` prevents fresh integration, then verifies the same for `/stop` plus
`/clear`.

Dirty-map dynamic artifact regression:

```bash
src/tgw_planner/scripts/run_dirty_map_tests.sh
```

Realtime bag plan probe:

```bash
TGW_BAG_PLAY_SECONDS=75 \
TGW_BAG_PROBE_LOG_DIR=/tmp/tgw_real_bag_plan_probe \
src/tgw_planner/scripts/run_realtime_bag_plan_probe.sh \
  /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

The probe launches FAST-LIO, n3mapping, and `tgw_realtime_mapping_node`, waits
for `/tgw_realtime_mapping_node`, `/tgw_mapping/get_snapshot`, and
`/tgw_map/plan_path`, plays the bag, selects two points from the largest
traversable surface component, and calls `/tgw_map/plan_path`. On failure it
prints tails from the FAST-LIO, n3mapping, tgw, bag, probe, and snapshot logs.
Use `TGW_BAG_PLAY_SECONDS=5` for a startup/diagnostic smoke only; it is not a
full map quality validation.

Endpoint snapping is capped by `planner_max_snap_distance_m`, exposed in the
probe as `TGW_PLANNER_MAX_SNAP_DISTANCE_M` and defaulting to 0.75 m. Increase it
only for explicit diagnostics; otherwise a requested start or goal that is too
far from any footprint-supported traversable surface should fail instead of
being silently projected to a distant layer.

The automatic start/goal selector can be tightened for stronger probes:

```bash
TGW_PROBE_MIN_DXY=4.0 TGW_PROBE_MAX_DXY=8.0 \
TGW_PROBE_MIN_ABS_DZ=0.0 TGW_PROBE_MAX_ABS_DZ=0.20 \
TGW_PROBE_PLAN_TIMEOUT=45 \
src/tgw_planner/scripts/run_realtime_bag_plan_probe.sh \
  /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

For cross-height validation, raise `TGW_PROBE_MIN_ABS_DZ` and
`TGW_PROBE_MAX_ABS_DZ`. If no pair is found, the current bag segment does not
contain a connected traversable component matching that geometry.

For A/B diagnosis only, the probe can override realtime surface extraction
without changing launch defaults:

```bash
TGW_SURFACE_REQUIRE_OBSERVED_FREE_SPACE=false \
TGW_PLANNER_ENABLE_SHORTCUT=false \
TGW_PROBE_MIN_ABS_DZ=0.50 TGW_PROBE_MAX_ABS_DZ=3.00 \
src/tgw_planner/scripts/run_realtime_bag_plan_probe.sh \
  /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

If this succeeds while the default observed-free run has no cross-height pair,
the remaining blocker is free-space evidence continuity, not surface A* search.

The current default cross-height probe should succeed on the reference real bag
without disabling observed-free:

```bash
TGW_PROBE_MIN_DXY=1.0 TGW_PROBE_MAX_DXY=8.0 \
TGW_PROBE_MIN_ABS_DZ=0.50 TGW_PROBE_MAX_ABS_DZ=3.00 \
TGW_PROBE_PLAN_TIMEOUT=60 TGW_PROBE_TOP_COMPONENTS=12 \
src/tgw_planner/scripts/run_realtime_bag_plan_probe.sh \
  /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

Realtime debug topics:

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
- `/tgw_map/risk_cloud`
- `/tgw_map/blocked_cloud`
- `/tgw_map/forbidden_cloud`
- `/tgw_map/planned_path`
- `/planned_path` compatibility alias
- `/planned_path_marker`
- `/start_marker`
- `/goal_marker`
- `/planner_stats`
- `/planner_stats_json`
- `/tgw_mapping/stats`
- `/tgw_map/stats_json`

Control services:

- `/tgw_mapping/start`
- `/tgw_mapping/stop`
- `/tgw_mapping/pause`
- `/tgw_mapping/clear`
- `/tgw_mapping/save_map` (`tgw_planner/srv/SaveMap`)
- `/tgw_mapping/load_map` (`tgw_planner/srv/LoadMap`)
- `/tgw_mapping/export_static_pcd` (`tgw_planner/srv/ExportStaticCloud`)
- `/tgw_mapping/get_snapshot` (`tgw_planner/srv/GetSnapshot`)

Planning service:

- `/tgw_map/plan_path` (`tgw_planner/srv/PlanPath`)
- `/tgw_map/set_blocked_region` (`tgw_planner/srv/SetBlockedRegion`)
- Compatibility aliases: `/plan_path`, `/nav_map/set_blocked_region`
- successful responses must pass final path validation before
  `/tgw_map/planned_path` is published

Example:

```bash
ros2 service call /tgw_map/plan_path tgw_planner/srv/PlanPath \
  "{start: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}}},
    goal: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}}}}"
```

Runtime blocked regions use map-frame AABBs. They are applied after surface
extraction, removed from `/tgw_map/traversable_cloud`, inserted into
`/tgw_map/blocked_cloud` and `/tgw_map/forbidden_cloud`, and included in
clearance/risk/path validation:

```bash
ros2 service call /tgw_map/set_blocked_region tgw_planner/srv/SetBlockedRegion \
  "{operation: add,
    min: {x: 1.0, y: -0.5, z: -0.2},
    max: {x: 3.0, y: 0.5, z: 0.5},
    reason: test_block}"
```

`realtime_mapping.launch.py` forwards the core probabilistic mapping,
surface-extraction, and clearance-aware planner parameters to the node. This is
important for bag validation because parameters such as `enable_dynamic_filter`,
`dynamic_clear_ratio_threshold`, `surface_require_static_support`, and planner
weights must be testable from the launch command line.

The launch also forwards `planner_max_snap_distance_m`, which bounds how far
`/tgw_map/plan_path` may project requested start and goal poses onto a nearby
traversable, footprint-supported surface.

The realtime planning response populates:

- `raw_path_waypoints`
- `raw_path_length_m`
- `postprocess_floor_shortcuts`
- `final_path_validated`
- `final_path_fallback_to_raw`
- `final_path_validation_failure`
- `min_path_clearance_m`
- `mean_path_clearance_m`
- `clearance_cost_sum`
- `low_clearance_samples`

With `validation_require_footprint:=true`, the final path is rejected if the
configured rectangular footprint is not fully supported along sampled path
segments. This may reject a point-center path near a surface boundary even if
the A* search itself found connected traversable cells.

With `planner_require_footprint:=true`, the surface A* itself also checks the
configured rectangular footprint at start/goal, at each neighbor cell, and along
sampled swept transitions. This prevents many invalid point-center paths from
being generated in the first place. Final validation remains as a second proof
before publishing `/tgw_map/planned_path`.

With `planner_enable_shortcut:=true`, the realtime surface planner can simplify
raw grid paths, but only when the straight segment remains traversable, passes
footprint checks, avoids forbidden/blocked cells, and does not reduce minimum
clearance below `planner_shortcut_clearance_ratio` of the raw segment or below
`robot_width_m / 2 + planner_shortcut_safety_margin_m`.
If the postprocessed path later fails final validation but the preserved raw
surface A* path validates, `/tgw_map/plan_path` falls back to the raw path and
sets `final_path_fallback_to_raw=true`.

`/tgw_map/medial_axis_cloud` publishes clearance ridges filtered by
`medial_axis_min_clearance_m`. This is a debug layer for checking whether the
clearance map places high-safety cells near corridor or passage centers.

`/tgw_map/risk_cloud` publishes cells with non-zero risk. Risk is currently
derived from boundary, drop-off, wall-adjacent, forbidden-adjacent, and
low-clearance cells. `SurfaceAstarPlanner` adds `planner_w_risk * riskCost`
to edge cost, so this layer is both an RViz diagnostic and part of the path
search objective.

`/tgw_mapping/stats` publishes the same mapping counters as
`/tgw_map/stats_json` in `tgw_planner/msg/MappingStats` form. Use the typed
topic for bagged diagnostics and monitoring; keep the JSON topic for quick CLI
inspection and text logs.

Dynamic filtering runs after each raycast-integrated scan. The latest decay
result is exposed as `last_scan_dynamic_suspect_voxels_after_decay` and
`last_scan_static_candidate_voxels_after_decay` in both stats outputs.

Self filtering is applied before raycasting in the incoming sensor frame. The
default body box is:

```text
self_filter_min_x=-0.60  self_filter_max_x=0.60
self_filter_min_y=-0.40  self_filter_max_y=0.40
self_filter_min_z=-0.35  self_filter_max_z=0.80
```

For rigs where the Livox mount, sensor bracket, or robot-specific structure
appears in the point cloud outside the body box, enable the second configurable
mount box:

```bash
ros2 launch tgw_planner realtime_mapping.launch.py \
  enable_mount_self_filter:=true \
  mount_self_filter_min_x:=<min_x> \
  mount_self_filter_max_x:=<max_x> \
  mount_self_filter_min_y:=<min_y> \
  mount_self_filter_max_y:=<max_y> \
  mount_self_filter_min_z:=<min_z> \
  mount_self_filter_max_z:=<max_z>
```

Both boxes are checked in the sensor frame. Keep them conservative: an overly
large self-filter box can remove real nearby obstacles before raycasting.

Map export:

```bash
ros2 service call /tgw_mapping/export_static_pcd tgw_planner/srv/ExportStaticCloud \
  "{output_pcd: /tmp/tgw_static_candidate_cloud.pcd}"

ros2 service call /tgw_mapping/save_map tgw_planner/srv/SaveMap \
  "{output_dir: /tmp/tgw_realtime_map}"

ros2 service call /tgw_mapping/load_map tgw_planner/srv/LoadMap \
  "{input_dir: /tmp/tgw_realtime_map}"

ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}"
```

`save_map` writes `occupied_cloud.pcd`, `free_cloud.pcd`,
`static_candidate_cloud.pcd`, `dynamic_suspect_cloud.pcd`,
`blocked_cloud.pcd`, `blocked_regions.yaml`, and `stats.json`. The exported PCD
intensity is the current occupancy probability for each voxel center. `load_map`
reconstructs the realtime probabilistic voxel layers from those PCD assets and
restores editable blocked region objects from `blocked_regions.yaml` plus
discrete loaded blocked cells from `blocked_cloud.pcd`.

`get_snapshot` is intentionally lightweight: it returns `MappingStats`, the
same JSON string as `/tgw_map/stats_json`, and per-layer point counts. The
actual clouds remain available through the latched debug topics.

## Verified Smoke

On 2026-06-02, the bag at `/home/user/ros_ws/bagfile/f7tof9_g2w_ros2`
was played for about 45 seconds through:

```bash
ros2 launch fast_lio mapping_mid360.launch.py rviz:=false
ros2 launch n3mapping mapping.launch.py rviz:=false
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/n3mapping/cloud_world \
  use_tf:=false \
  assume_cloud_in_map_frame:=true \
  min_range_m:=0.05 \
  max_range_m:=50.0 \
  publish_period_ms:=1000 \
  max_points_per_scan:=80000 \
  surface_require_static_support:=false
timeout 45 ros2 bag play /home/user/ros_ws/bagfile/f7tof9_g2w_ros2 --clock
```

Observed `/tgw_map/stats_json` after the short run:

```json
{
  "received_clouds": 446,
  "integrated_clouds": 446,
  "dropped_clouds": 0,
  "voxel_count": 462485,
  "occupied_voxels": 21045,
  "free_voxels": 387162,
  "static_candidate_voxels": 18033,
  "dynamic_suspect_voxels": 7799,
  "surface_cells": 8121,
  "traversable_cells": 8121,
  "boundary_cells": 4316
}
```
