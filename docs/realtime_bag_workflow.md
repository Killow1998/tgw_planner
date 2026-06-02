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
- `/tgw_map/forbidden_cloud`
- `/tgw_map/planned_path`
- `/tgw_map/stats_json`

Control services:

- `/tgw_mapping/start`
- `/tgw_mapping/stop`
- `/tgw_mapping/clear`

Planning service:

- `/tgw_map/plan_path` (`tgw_planner/srv/PlanPath`)
- successful responses must pass final path validation before
  `/tgw_map/planned_path` is published

Example:

```bash
ros2 service call /tgw_map/plan_path tgw_planner/srv/PlanPath \
  "{start: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}}},
    goal: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 0.15}, orientation: {w: 1.0}}}}"
```

`realtime_mapping.launch.py` forwards the core probabilistic mapping,
surface-extraction, and clearance-aware planner parameters to the node. This is
important for bag validation because parameters such as `enable_dynamic_filter`,
`surface_require_static_support`, and planner weights must be testable from the
launch command line.

The realtime planning response populates:

- `final_path_validated`
- `final_path_fallback_to_raw`
- `final_path_validation_failure`
- `min_path_clearance_m`
- `mean_path_clearance_m`
- `low_clearance_samples`

With `validation_require_footprint:=true`, the final path is rejected if the
configured rectangular footprint is not fully supported along sampled path
segments. This may reject a point-center path near a surface boundary even if
the A* search itself found connected traversable cells.

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
