# tgw_planner

`tgw_planner` is a lightweight N3Mapping pbstream experience planner.

Current mainline:

```text
n3map.pbstream
  -> strict N3 nav resource adapter
  -> shared ExperienceGeometryIndex
  -> dense trajectory support projection
  -> experience reachable surface
  -> layer-safe local surface graph
  -> dense trajectory backbone graph
  -> unified hybrid graph
  -> global path with Surface / Backbone / Portal segment kinds
  -> local route window + smoother + regulated pure pursuit
```

The current status, evidence, and review questions are in:

```text
docs/tgw_current_status_for_review.md
```

The golden regression scenes and commands are in:

```text
docs/exp/golden_scenes.md
```

TGW is not a mapping framework. It does not use PCD fallback, realtime raycast
global reconstruction, StairFlight semantics, or 3D voxel A* in the default
mainline.

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
when the robot-side command path is ready. Real tracking requires explicit
arming through `/tgw_experience/set_tracking_armed`.

Optional RViz support remains available through:

- `tgw_clicked_point_router_node`
- RViz tools `tgw_planner/Tgw3DStartTool` and `tgw_planner/Tgw3DGoalTool`

## Default Build Contents

The default build keeps only:

- retained planning core (`clearance`, `risk`, `footprint`, `PathValidator`)
- experience planner core (`N3MapReader`, `ExperienceGeometryIndex`,
  `TrajectoryProjector`, `ReachableExpander`, `ExperienceSurfaceBuilder`,
  `ExperienceSurfaceGraph`, `ExperienceBackboneGraph`,
  `HybridExperiencePlanner`)
- local tracking core (`RouteProgressTracker`, `LocalPathSmoother`,
  `RegulatedPurePursuitController`, `RollingLocalMap`)
- `tgw_experience_planner_node`
- `tgw_clicked_point_router_node`
- RViz pose tools
- core regression tools (`tgw_experience_global_sweep`,
  `tgw_experience_benchmark`)
- focused core smoke tests

Legacy PCD-only, realtime raycast, and stair-specific paths have been removed
from the default build. The `before-pbstream-experience-reset` tag and git
history are the archive for those removed experiments.
