# tgw_planner

`tgw_planner` is a lightweight N3Mapping pbstream experience planner.

Current stable line:

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

The current pre-HIL stability marker is:

```text
tgw-experience-mvp-pre-hil
```

The golden regression scenes and commands are in:

```text
docs/exp/golden_scenes.md
```

Target edge-device benchmark and robot dry-run plans are in:

```text
docs/exp/edge_benchmark_plan.md
docs/hil_test_plan.md
```

TGW is not a mapping framework. It does not use PCD fallback, realtime raycast
global reconstruction, StairFlight semantics, or 3D voxel A* in the default
mainline.

## Deployment Status

TGW is ready for:

```text
edge-device benchmarking
robot-side dry run without forwarding commands to /cmd_vel
supervised low-speed HIL preparation
```

TGW is not yet a production autonomous safety system. Real robot deployment
still requires:

```text
target-device benchmark evidence
robot-side command mux / e-stop / deadman
odom frame verification against the pbstream map frame
dry-run validation of tracking_status_json and candidate cmd_vel
supervised low-speed single-floor test before any cross-floor run
```

The real launch publishes to `/tgw_experience/cmd_vel` by default and requires
explicit tracking arming. Keep it behind a robot-side safety mux.

## Build

```bash
cd $ROS_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
```

## Offline Regression

Run the core tests:

```bash
cd $ROS_WS
colcon test --packages-select tgw_planner --event-handlers console_direct+
colcon test-result --verbose
```

Run the golden regression:

```bash
cd $ROS_WS
source install/setup.bash
MPLCONFIGDIR=/tmp/matplotlib \
python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene all \
  --sample-pairs 50 \
  --plot-limit 0
```

Current golden coverage:

```text
scene_20260608: stair / cross-floor experience scene
scene_20260610: ramp / grade-change scene

cross_floor global sweep: 50 / 50
same_floor_low global sweep: 50 / 50
same_floor_high global sweep: 50 / 50

cross_floor tracking replay: 50 / 50
same_floor_low tracking replay: 50 / 50
same_floor_high tracking replay: 50 / 50
```

## Benchmark

Benchmark preprocessing and first-query time:

```bash
cd $ROS_WS
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream

build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream
```

Current development CPU baseline:

```text
scene_20260608: preprocess about 4.30s, first query about 0.12ms
scene_20260610: preprocess about 6.15s, first query about 0.26ms
```

Run the same benchmark on target edge hardware before changing architecture or
adding compiled cache.

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

Arm command:

```bash
ros2 service call /tgw_experience/set_tracking_armed \
  std_srvs/srv/SetBool "{data: true}"
```

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

## Branch Policy

The active development branch is:

```text
rebuild/experience-backbone
```

This branch should become the canonical `main` line once the repository owner
decides to promote the pbstream experience planner as the default TGW product.
Old experimental branches should not be deleted until they are either:

```text
tagged as historical checkpoints
confirmed unused by collaborators
not referenced by open review / deployment work
```

Recommended promotion sequence:

```text
1. keep tag tgw-experience-mvp-pre-hil as the stable checkpoint
2. fast-forward main to rebuild/experience-backbone
3. change GitHub default branch to main if it is not already main
4. archive or delete obsolete remote branches only after explicit owner approval
```

Do not preserve old branches as compatibility paths in the default build.
Git history and explicit tags are the archive.
