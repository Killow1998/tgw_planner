# TGW PBStream Experience Roadmap

## Direction

`tgw_planner` is a consumer of N3Mapping `n3map.pbstream` navigation resources.
It is not a mapping framework, realtime raycast mapper, terrain semantic
classifier, or PCD traversability fallback system.

Mainline:

```text
n3map.pbstream
  -> N3Mapping lightweight nav resource reader
  -> TGW internal resource adapter
  -> keyframe geometry index
  -> dense trajectory support projection
  -> proven reachable seeds
  -> conservative reachable expansion
  -> clearance / risk / confidence fields
  -> reachable-surface planner and validator
```

## Current Baseline

Branch:

```text
feature/pbstream-experience-reset
```

Archive tag for the removed realtime/PCD/stair experiments:

```text
before-pbstream-experience-reset
```

Baseline commit:

```text
ff58be2 Reset TGW to pbstream experience intake
```

The current default build:

- links `n3mapping::n3map_nav_resource_reader`
- builds `tgw_planner_core`
- builds `tgw_experience_planner_node`
- builds `tgw_clicked_point_router_node`
- builds RViz pose tools
- builds `tgw_experience_core_smoke`

The current node:

- requires `pbstream_path`
- rejects invalid N3 nav resources explicitly
- publishes `/tgw_experience/trajectory_cloud`
- publishes `/tgw_experience/stats_json`
- does not load `global_map.pcd`
- does not run realtime raycast
- does not invoke stair-specific fallback logic

Verified real pbstream:

```text
/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream
keyframes=798
dense_trajectory=10561
map_frame=map
body_frame=body
```

## Hard Contract

TGW must fail explicitly when:

- pbstream cannot be opened
- pbstream cannot be parsed
- dense optimized trajectory is missing
- dense trajectory is from keyframe fallback
- dense trajectory is degraded
- keyframes are missing
- any keyframe cloud is missing or empty
- map/body frame metadata is invalid

Stable error codes in use:

```text
pbstream_open_failed
pbstream_parse_failed
pbstream_missing_dense_trajectory
pbstream_dense_trajectory_from_fallback_not_allowed
pbstream_dense_trajectory_degraded_not_allowed
pbstream_missing_keyframes
pbstream_missing_keyframe_clouds
pbstream_invalid_frame_contract
```

Forbidden mainline behavior:

- no fallback to keyframe-only trajectory
- no fallback to `global_map.pcd`
- no realtime raycast reconstruction
- no probabilistic global-map builder
- no StairFlight / curved stair / spiral stair planner branch
- no source-tree `legacy/` archive; git history is the archive

## Milestones

### M0: PBStream Intake

Status: complete in `ff58be2`.

Acceptance:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
colcon test --packages-select tgw_planner
colcon test-result --verbose
```

Runtime smoke:

```bash
source install/setup.bash
ROS_LOG_DIR=/tmp/tgw_planner_ros_log \
timeout 5s ros2 run tgw_planner tgw_experience_planner_node \
  --ros-args \
  -p pbstream_path:=/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream
```

Expected:

```text
loaded n3map experience resource: keyframes=798 dense_trajectory=10561 frame=map body=body
```

### M1: Support Projection Proof Point

Status: implemented in the working tree after `ff58be2`; pending review and
commit decision.

Goal:

Use keyframe cloud geometry to project dense trajectory samples onto nearby
support surfaces below the lidar/body pose.

Required outputs:

```text
/tgw_experience/trajectory_cloud
/tgw_experience/projected_support_cloud
/tgw_experience/proven_reachable_cloud
/tgw_experience/rejected_projection_cloud
/tgw_experience/stats_json
```

First implementation should be conservative:

- build an XY column geometry index from keyframe clouds transformed by optimized poses
- search only inside a configured vertical band below each dense trajectory pose
- reject samples without support in-band
- reject ambiguous multi-layer support if a simple continuity rule cannot resolve it
- do not search the full column
- do not guess another floor silently

Suggested parameters:

```yaml
support_projection:
  raw_resolution_m: 0.05
  nav_resolution_m: 0.10
  search_below_min_m: 0.10
  search_below_max_m: 1.00
  max_support_jump_m: 0.30
```

Acceptance:

- valid pbstream loads
- projected support count is nonzero
- rejected projection count is reported
- projected support cloud is published in `map_frame`
- proven reachable debug cloud is generated from projected support cells
- no planning service is required yet

Current real-pbstream smoke:

```text
/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream
keyframes=799
dense_trajectory=10649
projected_support=9347
rejected_projection=1302
```

### M2: Proven Reachable Footprint Seeds

Status: implemented in the working tree after `ff58be2`; pending review and
commit decision.

Goal:

Convert projected support trajectory into footprint-swept proven reachable cells.

Rules:

- use robot footprint parameters
- require sufficient support ratio
- tolerate small holes only within configured limits
- reject unsupported samples instead of inventing geometry

Current real-pbstream smoke:

```text
/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream
keyframes=799
dense_trajectory=10649
projected_support=9347
proven_reachable=37058
rejected_projection=1302
footprint_rejected=3
```

Filtered nav-resource smoke:

```text
filter command:
/home/user/ros_ws/to_migrate_ws/install/n3mapping/lib/n3mapping/n3mapping_filter_nav_pbstream \
  --input /home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream \
  --output /tmp/tgw_n3map_nav_filtered.pbstream \
  --debug-dir /tmp/tgw_n3map_nav_filter_debug

filter result:
raw_points=4905826
kept_points=4259965
removed_points=645861
removed_ratio=0.131652
policy=rear_sector_enable=true;rear_sector_center_deg=180;rear_sector_width_deg=45;forward_axis=x;min_range=0;max_range=1e+09;self_filter_enable=false

TGW result:
keyframes=799
dense_trajectory=10649
projected_support=9334
proven_reachable=36994
expanded_reachable=53609
rejected_projection=1315
footprint_rejected=6
nav_cloud_filter_applied=true
```

### M3: Conservative Expansion

Status: implemented in the working tree after `ff58be2`; pending review and
commit decision.

Goal:

Expand proven reachable cells across connected, height-consistent, supported
surfaces.

No semantic terrain classes should be required.

Current behavior:

- expands from proven footprint-swept cells
- requires neighboring keyframe geometry
- limits expansion by grid radius, max steps, z-cell tolerance, and max step height
- publishes `/tgw_experience/expanded_reachable_cloud`

Current real-pbstream smoke:

```text
/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream
keyframes=799
dense_trajectory=10649
projected_support=9347
proven_reachable=37058
expanded_reachable=53801
rejected_projection=1302
footprint_rejected=3
```

Filtered nav-resource smoke:

```text
/tmp/tgw_n3map_nav_filtered.pbstream
keyframes=799
dense_trajectory=10649
projected_support=9334
proven_reachable=36994
expanded_reachable=53609
rejected_projection=1315
footprint_rejected=6
nav_filter_raw_points=4905826
nav_filter_kept_points=4259965
nav_filter_removed_points=645861
```

### M4: Planning On Experience Surface

Goal:

Introduce `ExperienceAstarPlanner` or adapt the retained planner only after the
experience snapshot model is stable.

Allowed fallback:

- path shortcut failed -> raw A* path on the same reachable surface
- only if raw path passes validator

Forbidden fallback:

- no PCD mode
- no realtime raycast rebuild
- no stair rescue planner

## Falsifiers

This direction should be reconsidered if real pbstream evidence shows:

- dense trajectory and keyframe clouds are visibly inconsistent
- support projection jumps floors in normal scenes even with z-band and continuity
- trajectory-supported areas are too sparse to form usable proven seeds
- conservative expansion cannot cover normal corridor or stair width
