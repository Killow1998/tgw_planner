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

## Parameterized Probe Update 2026-06-03

`scripts/run_realtime_bag_plan_probe.sh` now sources the workspace overlay from
its own path, clears stale probe logs before each run, and supports automatic
start/goal criteria through:

- `TGW_PROBE_MIN_DXY`
- `TGW_PROBE_MAX_DXY`
- `TGW_PROBE_MIN_ABS_DZ`
- `TGW_PROBE_MAX_ABS_DZ`
- `TGW_PROBE_PLAN_TIMEOUT`
- `TGW_PROBE_SAMPLE_LIMIT`

Default same-height probe after this change:

```text
received_clouds=635
integrated_clouds=635
dynamic_points=23072
surface_points=1235
traversable_points=1235
probe_criteria=min_dxy:1.000 max_dxy:2.000 min_abs_dz:0.000 max_abs_dz:0.050 plan_timeout:20.0
surface_component_count=548
largest_component_size=318
success=True
final_path_validated=True
expanded_nodes=79
path_waypoints=12
path_length_m=1.390
mean_path_clearance_m=0.097
```

Longer same-height probe:

```bash
TGW_PROBE_MIN_DXY=3.0 TGW_PROBE_MAX_DXY=6.0 \
TGW_PROBE_MIN_ABS_DZ=0.0 TGW_PROBE_MAX_ABS_DZ=0.20 \
TGW_PROBE_PLAN_TIMEOUT=45 \
scripts/run_realtime_bag_plan_probe.sh /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

Result:

```text
received_clouds=635
integrated_clouds=635
surface_points=1246
traversable_points=1246
surface_component_count=546
largest_component_size=323
dxy=3.061
success=True
final_path_validated=True
expanded_nodes=317
path_waypoints=38
path_length_m=4.363
mean_path_clearance_m=0.066
```

Cross-height probe:

```bash
TGW_PROBE_MIN_DXY=1.0 TGW_PROBE_MAX_DXY=8.0 \
TGW_PROBE_MIN_ABS_DZ=0.50 TGW_PROBE_MAX_ABS_DZ=3.00 \
TGW_PROBE_PLAN_TIMEOUT=60 \
scripts/run_realtime_bag_plan_probe.sh /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

Result:

```text
received_clouds=633
integrated_clouds=633
surface_points=1264
traversable_points=1264
surface_component_count=568
largest_component_size=341
success=False
reason=no_component_pair_matching_probe_criteria
```

Longer 180 s cross-height probe:

```bash
TGW_BAG_PLAY_SECONDS=180 \
TGW_PROBE_MIN_DXY=1.0 TGW_PROBE_MAX_DXY=12.0 \
TGW_PROBE_MIN_ABS_DZ=0.50 TGW_PROBE_MAX_ABS_DZ=4.00 \
TGW_PROBE_PLAN_TIMEOUT=90 \
scripts/run_realtime_bag_plan_probe.sh /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

Result:

```text
received_clouds=736
integrated_clouds=736
surface_points=1622
traversable_points=1622
surface_component_count=811
largest_component_size=328
success=False
reason=no_component_pair_matching_probe_criteria
```

Interpretation:

- The realtime stack can produce validated short and 3 m same-height paths from
  this bag.
- The 75 s and 180 s runs do not produce a connected traversable component
  containing a point pair with `abs(dz) >= 0.5 m`; this is map
  coverage/connectivity evidence, not a failed path search between chosen
  cross-floor endpoints.
- The largest realtime traversable component remains small relative to total
  traversable points, so cross-floor validation now needs either a better bag
  segment, adjusted realtime surface continuity, or explicit inspected
  start/goal points from RViz.
- Mean clearance is low on the real-bag probes, so broader map-quality and
  corridor-quality validation is still required before treating the realtime
  layer as deployment-ready.

## Surface Connectivity A/B 2026-06-03

The probe now prints global traversable bounds and the top traversable
components with XY/Z bounds. This made the remaining real-bag issue explicit:

- Default `surface_require_observed_free_space=true`:
  - `traversable_points=1232`
  - `surface_component_count=539`
  - `largest_component_size=324`
  - top component `z_span=0.000`
  - only one of the top twelve components has `z_span >= 0.5 m`
  - result: `success=False reason=no_component_pair_matching_probe_criteria`

- Diagnostic `surface_require_observed_free_space=false`:
  - `traversable_points=12998`
  - `surface_component_count=1201`
  - `largest_component_size=9975`
  - largest component `z_span=2.100`
  - cross-height path succeeds:

```text
start=(4.4499998093, -2.8499999046, 2.3499999046)
goal=(0.9499999881, 1.8500000238, 2.8499999046)
dxy=5.860
success=True
message="path found"
final_path_validated=True
raw_path_waypoints=48
path_waypoints=48
path_length_m=6.671
mean_path_clearance_m=0.630
```

With shortcuts enabled under the same diagnostic surface mode:

```text
success=True
final_path_validated=True
raw_path_waypoints=55
shortcut_count=43
path_waypoints=12
path_length_m=6.983
mean_path_clearance_m=0.670
```

Interpretation:

- Surface A* and final validation can now handle direct adjacent surface edges
  with vertical change; validation no longer rejects valid stair/slope edges
  because of interpolated samples through empty z layers.
- The remaining default-mode cross-height blocker is the observed-free-space
  requirement fragmenting the realtime surface. Disabling it is useful for
  diagnosis but is not a deployment fix because it can admit ceiling tops and
  unobserved occupied tops.
- The next algorithmic task is to improve free-space evidence continuity or add
  a conservative observed-free relaxation that only bridges locally supported
  stair/slope surface edges, without turning ceiling tops into traversable
  ground.

## Component-Anchored Observed-Free Surface 2026-06-03

The default realtime surface extractor now keeps
`surface_require_observed_free_space=true`, but no longer requires every
standing cell itself to be ray-cleared free. It first builds all static-support
surface candidates that have head clearance, then accepts only candidate
components connected to an observed-free or observed-clearance anchor. Candidate
components with no free-space anchor remain forbidden.

This fixes the real-bag fragmentation without using the unsafe diagnostic mode
that accepts every unobserved candidate:

```text
TGW_SURFACE_REQUIRE_OBSERVED_FREE_SPACE=true
TGW_SURFACE_ALLOW_OBSERVED_FREE_BRIDGE=true
traversable_points=12208
surface_component_count=666
largest_component_size=10033
largest_component_z_span=2.200
success=True
final_path_validated=True
raw_path_waypoints=49
shortcut_count=40
path_waypoints=9
path_length_m=6.485
mean_path_clearance_m=0.674
```

Regression checks after the change:

```text
ctest tgw_phase1_core_smoke: passed
run_synthetic_surface_scene_tests.sh: passed
run_realtime_mapping_sim_tests.sh: passed
  floor_ceiling_free_space: success=True surface_points=55 traversable_points=55 forbidden_points=57
run_reference_pcd_smoke_tests.sh: passed
  surface_pct_building: success=true final_path_validated=true
  surface_pct_spiral: success=true final_path_validated=true
```

Interpretation:

- The original default blocker was over-strict pointwise free-space evidence,
  not A* connectivity or final-path validation.
- The deployment default still rejects surface components with no observed
  robot-clearance anchor, so this is not equivalent to
  `surface_require_observed_free_space=false`.
- Broader dirty-map and multi-scene validation is still needed before freezing
  the realtime global layer.

## Realtime Probe After Validation Diagnostics 2026-06-03

Command:

```bash
src/tgw_planner/scripts/run_realtime_bag_plan_probe.sh /home/user/ros_ws/bagfile/f7tof9_g2w_ros2
```

The probe script now launches FAST-LIO, n3mapping, and `tgw_realtime_mapping_node`
in separate process groups so cleanup removes launch children and RViz children
instead of leaving stale ROS graph state behind. It also prints
`low_clearance_samples` from the final realtime path validation report.

Latest result:

```text
received_clouds=653
integrated_clouds=653
dynamic_points=20371
surface_points=12435
traversable_points=12435
surface_component_count=665
largest_component_size=10102
largest_component_z_span=3.200
start=(4.4499998093, -2.8499999046, 2.3499999046)
goal=(3.75, -3.6500000954, 2.3499999046)
dxy=1.063
success=True
final_path_validated=True
final_path_fallback_to_raw=False
raw_path_waypoints=11
shortcut_count=8
path_waypoints=3
path_length_m=1.223
min_path_clearance_m=0.141
mean_path_clearance_m=0.367
low_clearance_samples=6
```

Interpretation:

- The realtime bag chain still integrates the full bag segment and produces a
  validated short same-height path on the largest traversable component.
- `low_clearance_samples` is now diagnostic-only and independent from the hard
  `validation_min_clearance_m` gate. Low clearance is counted against the
  default 0.30 m reporting threshold, while hard failure remains controlled by
  `validation_min_clearance_m`.
- The path is valid but has nearby boundary/obstacle proximity. This confirms
  that future deployment gates should evaluate corridor quality explicitly
  rather than treating `success=True` alone as sufficient.
