# Map Input Modes

## Realtime Raycast Mapping Mode

Realtime raycast mapping is the intended deployment path for `tgw_planner`.
It uses timestamped point clouds plus sensor pose in the map frame to update a
probabilistic voxel map with hit and miss observations.

This mode can use ray clearing and temporal consistency to weaken short-lived
artifacts such as moving people, temporary objects, and SLAM ghost points. It
cannot guarantee removal of long-stationary people or long-lived temporary
objects, so deployment maps should still be collected in clean conditions and
manually corrected when needed.

Expected inputs:

- `/tgw_mapping/points` as `sensor_msgs/PointCloud2`
- `/tf`, or an equivalent sensor pose in map frame

Expected map products:

- occupied, free, and unknown voxel layers
- dynamic suspect and static candidate layers
- traversable surface layer
- boundary and clearance fields

## PCD Offline Import Mode

PCD mode is for clean static map import, simulation, and offline debugging. It
loads a final point cloud without scan origins, timestamps, or ray histories.

Because a PCD has no ray clearing information, the planner cannot tell whether
a point came from static structure or from a temporary artifact. If the PCD
contains a human operator, moving-object trail, SLAM ghost, temporary obstacle,
or robot self points, those points can be treated as static occupied structure.

This can corrupt:

- floor extraction
- stair and slope detection
- boundary detection
- clearance field computation
- traversability
- path planning

For deployment, prefer realtime raycast mapping mode or a cleaned static PCD.
When PCD mode is used, `tgw_pcd_import_node` prints a PCD mode warning and
reports `map_input_mode="pcd"` with `pcd_artifact_warning=true` in:

- `/map_build_stats`
- `/map_build_stats_json`
- `/planner_stats_json`

`pcd_to_path_mvp.launch.py` starts `tgw_pcd_import_node` but keeps the ROS node
name `tgw_planner_node` for compatibility with existing scripts.
