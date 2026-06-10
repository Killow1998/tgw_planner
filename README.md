# tgw_planner

`tgw_planner` is now a lightweight experience-based planner entrypoint.

Mainline pipeline:

```text
n3map.pbstream
  -> strict N3 nav resource adapter
  -> dense optimized trajectory debug cloud
  -> nav-resource stats JSON
```

Current scope:

- read `n3map.pbstream` through N3Mapping's nav resource reader
- reject missing, degraded, or keyframe-fallback dense trajectory data
- reject missing keyframes or empty keyframe clouds
- publish `/tgw_experience/trajectory_cloud`
- publish `/tgw_experience/stats_json`

Support projection, proven reachable seed generation, expansion, and A* planning
are the next milestones after pbstream intake is verified.

Out of scope for the main pipeline:

- generic SLAM or map repair
- arbitrary PCD traversability inference
- realtime raycast global reconstruction
- dynamic-object global cleanup
- StairFlight, stair centerline, curved stair, or spiral stair logic
- complex fallback chains
- silent fallback to `global_map.pcd`

Strict failure policy:

- TGW prefers explicit failure over uncertain recovery.
- No silent fallback to `global_map.pcd`.
- No automatic map repair.
- No realtime raycast reconstruction in the main pipeline.
- No terrain semantic planner fallback.
- No StairFlight fallback.
- The only allowed small fallback is path post-processing reverting to the raw
  A* path on the same reachable surface, and only when that raw path passes
  `PathValidator`.

## Current Stage

The default node is `tgw_experience_planner_node`. It uses the real N3Mapping
lightweight nav resource reader. The node fails explicitly
when the pbstream does not satisfy TGW's native dense trajectory and keyframe
cloud contract. It does not fall back to PCD input.

`global_map.pcd` is debug-only and no longer a mainline input.

## Build

```bash
cd $ROS_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
```

## Launch

For RViz kinematic replay without a robot odom source:

```bash
source $ROS_WS/install/setup.bash
ros2 launch tgw_planner experience_planner_sim.launch.py \
  pbstream_path:=/absolute/path/to/n3map.pbstream
```

For real odom / command output wiring:

```bash
source $ROS_WS/install/setup.bash
ros2 launch tgw_planner experience_planner_real.launch.py \
  pbstream_path:=/absolute/path/to/n3map.pbstream
```

The real launch defaults to publishing velocity commands on
`/tgw_experience/cmd_vel`; override `tracking_cmd_vel_topic:=/cmd_vel` only
when the robot-side command path is ready.

Optional RViz support remains available through:

- `tgw_clicked_point_router_node`
- RViz tools `tgw_planner/Tgw3DStartTool` and `tgw_planner/Tgw3DGoalTool`

## Default Build Contents

The default build keeps only:

- retained planning core (`clearance`, `risk`, `footprint`, `surface A*`, `PathValidator`)
- experience planner core (`N3MapReader`, `TrajectoryProjector`,
  `ReachableExpander`, `ExperienceSurfaceBuilder`)
- `tgw_experience_planner_node`
- `tgw_clicked_point_router_node`
- RViz pose tools
- focused core smoke tests

Legacy PCD-only, realtime raycast, and stair-specific paths have been removed
from the default build. The `before-pbstream-experience-reset` tag and git
history are the archive for those removed experiments.
