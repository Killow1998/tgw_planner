# TGW PBStream Experience Tasklist

## Completed

- [x] Create `feature/pbstream-experience-reset`.
- [x] Tag old default mainline as `before-pbstream-experience-reset`.
- [x] Remove old realtime raycast, PCD, tomogram, and stair-specific code from
  the default build and install path.
- [x] Avoid a source-tree `legacy/` archive.
- [x] Link TGW against N3Mapping's lightweight nav resource reader.
- [x] Convert `n3mapping::N3NavResource` into TGW-owned internal resource types.
- [x] Reject missing, fallback, degraded, or non-native dense trajectory.
- [x] Reject missing keyframes and empty keyframe clouds.
- [x] Add `tgw_experience_planner_node`.
- [x] Publish `/tgw_experience/trajectory_cloud`.
- [x] Publish `/tgw_experience/stats_json`.
- [x] Verify package build and smoke tests.
- [x] Verify real pbstream load on
  `/home/user/ros_ws/to_migrate_ws/src/n3mapping/map/n3map.pbstream`.

## Immediate Next Task: Support Projection Proof Point

Status: implemented in working tree after `ff58be2`; not committed.

Objective:

```text
dense trajectory pose_world_lidar
  + keyframe cloud geometry
  -> projected support samples
  -> proven reachable debug cells
```

Implementation checklist:

- [x] Add a small geometry column/index type for support lookup.
- [x] Transform keyframe cloud points from body frame into world frame.
- [x] Quantize support evidence by XY column and vertical bins.
- [x] Add support projection parameters to `experience_planner_params.yaml`.
- [x] Replace `support_z_offset_m` trajectory projection with in-band support
  search below each dense trajectory sample.
- [x] Track accepted projected support samples.
- [x] Track rejected samples with reason counts.
- [x] Publish `/tgw_experience/projected_support_cloud`.
- [x] Publish `/tgw_experience/proven_reachable_cloud`.
- [x] Publish `/tgw_experience/rejected_projection_cloud`.
- [x] Extend `/tgw_experience/stats_json` with projection counts.
- [x] Add unit coverage for valid support, missing support, and support outside
  the vertical band.
- [x] Build and test.
- [x] Run the node on the real pbstream and record counts.
- [x] Run `n3mapping_filter_nav_pbstream` and verify TGW reads nav filter metadata.

Acceptance commands:

```bash
cd /home/user/ros_ws/to_migrate_ws
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

Expected runtime evidence:

```text
loaded n3map experience resource: keyframes=799 dense_trajectory=10649 projected_support=9347 rejected_projection=1302 frame=map body=body
```

## Later Tasks

- [x] Convert projected support samples into footprint-swept proven reachable
  cells.
- [x] Add support-ratio checks for footprint samples.
- [ ] Add explicit contiguous-hole tolerance checks.
- [x] Implement conservative expansion from proven reachable seeds.
- [x] Add expansion height gate.
- [x] Publish `/tgw_experience/expanded_reachable_cloud`.
- [ ] Add boundary, clearance, risk, and confidence layers to the experience
  snapshot.
- [ ] Decide whether to create `ExperienceAstarPlanner` or adapt retained
  `SurfaceAstarPlanner`.
- [ ] Reintroduce `PlanPath` only after the experience snapshot is grounded in
  support projection and proven reachable cells.

Current M2 runtime evidence:

```text
loaded n3map experience resource: keyframes=799 dense_trajectory=10649 projected_support=9347 proven_reachable=37058 rejected_projection=1302 footprint_rejected=3 frame=map body=body
```

Current M3 runtime evidence:

```text
loaded n3map experience resource: keyframes=799 dense_trajectory=10649 projected_support=9347 proven_reachable=37058 expanded_reachable=53801 rejected_projection=1302 footprint_rejected=3 frame=map body=body
```

Current filtered nav-resource evidence:

```text
filter output: /tmp/tgw_n3map_nav_filtered.pbstream
filter debug: /tmp/tgw_n3map_nav_filter_debug/nav_filter_report.json
raw_points=4905826 kept_points=4259965 removed_points=645861 removed_ratio=0.131652
TGW: keyframes=799 dense_trajectory=10649 projected_support=9334 proven_reachable=36994 expanded_reachable=53609 rejected_projection=1315 footprint_rejected=6
nav_cloud_filter_applied=true
```

## Keep Out

- [ ] Do not restore `global_map.pcd` as an input path.
- [ ] Do not restore realtime raycast as a global map builder.
- [ ] Do not restore PCD smoke tools as default targets.
- [ ] Do not restore StairFlight, curved stair, or spiral stair logic.
- [ ] Do not create `legacy/` in the source tree.
- [ ] Do not make support projection silently search the entire vertical column.
- [ ] Do not implement full planning before support projection is inspectable.
