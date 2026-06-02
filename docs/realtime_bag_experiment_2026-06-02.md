# Realtime Bag Experiment 2026-06-02

## Purpose

Validate `tgw_planner` realtime raycast mapping against the real Go2-W Livox bag
instead of only synthetic point clouds or clean reference PCDs.

## Input

- Bag: `/home/user/ros_ws/bagfile/f7tof9_g2w_ros2`
- Duration: 1072.497 s
- Topics:
  - `/livox/imu` (`sensor_msgs/msg/Imu`)
  - `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`)

## Startup

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/setup.bash

ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch n3mapping mapping.launch.py
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/n3mapping/cloud_world \
  use_tf:=false \
  assume_cloud_in_map_frame:=true
ros2 bag play /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

## Gates

- `tgw_realtime_mapping_node` receives and integrates clouds.
- `/tgw_mapping/get_snapshot` succeeds.
- Snapshot has non-zero occupied/free/static/surface/traversable counts.
- `surface_require_observed_free_space` remains enabled unless the cloud lacks
  ray-cleared free-space evidence.
- Any failed launch or missing topic is recorded here before changing code.

## Result

Partial real-bag validation passed on 2026-06-02.

Command variant:

```bash
timeout 90s ros2 bag play /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

Observed topics included:

- `/n3mapping/cloud_world`
- `/tgw_map/occupied_cloud`
- `/tgw_map/free_cloud`
- `/tgw_map/static_candidate_cloud`
- `/tgw_map/dynamic_suspect_cloud`
- `/tgw_map/surface_cloud`
- `/tgw_map/traversable_cloud`

Snapshot after the 90 s playback:

```text
success: true
received_clouds: 727
integrated_clouds: 727
dropped_clouds: 0
voxel_count: 764676
occupied_voxels: 35921
free_voxels: 611477
static_candidate_voxels: 28694
dynamic_suspect_voxels: 110427
surface_cells: 1434
traversable_cells: 1434
boundary_cells: 1367
forbidden_cells: 34452
clearance_cells: 1434
medial_axis_cells: 3
risk_cells: 1434
map_resolution_m: 0.1
```

Interpretation:

- FAST-LIO and n3mapping produced `/n3mapping/cloud_world`.
- `tgw_realtime_mapping_node` integrated realtime raycast input without drops.
- Free-space, static/dynamic classification, surface extraction, traversability,
  boundary, clearance, and risk layers were all populated.
- This first run was not a full-path navigation validation.

## Plan Probe

`scripts/run_realtime_bag_plan_probe.sh` repeats the real-bag stack, collects
`/tgw_map/traversable_cloud`, selects two points from the largest traversable
surface component, and calls `/tgw_map/plan_path`.

Observed result with a 75 s playback on 2026-06-03:

```text
received_clouds=641
integrated_clouds=641
dynamic_points=19638
surface_points=1205
traversable_points=1205
surface_component_count=491
largest_component_size=381
start=(0.0500000007, 1.1499999762, -0.1500000060)
goal=(-0.75, 0.4499999881, -0.1500000060)
dxy=1.063
success=True
message="path found"
final_path_validated=True
final_path_fallback_to_raw=False
expanded_nodes=117
path_waypoints=13
path_length_m=1.490
min_path_clearance_m=0.000
mean_path_clearance_m=0.097
clearance_cost_sum=121.333
```

This validates the realtime map-to-plan service chain on the real bag, but only
for a short same-component path. Longer cross-floor or cross-component planning
still needs separate point selection and map-quality validation.

The probe script now assigns a temporary `ROS_DOMAIN_ID` when the caller has not
set one. This prevents stale transient-local `/tgw_map/traversable_cloud` data
from an older ROS graph from being mistaken for current realtime mapping output.
The script also fails before planning if `/tgw_mapping/get_snapshot` reports
`integrated_clouds=0`.
