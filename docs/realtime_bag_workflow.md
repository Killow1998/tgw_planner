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
- `/tgw_map/stats_json`

Control services:

- `/tgw_mapping/start`
- `/tgw_mapping/stop`
- `/tgw_mapping/clear`
