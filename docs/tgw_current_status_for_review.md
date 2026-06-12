# TGW Current Status For Review

Date: 2026-06-12

## Thesis

TGW is now a pbstream experience planner:

```text
N3Mapping n3map.pbstream
  -> shared geometry index
  -> experience reachable surface
  -> local surface islands
  -> dense trajectory backbone
  -> unified hybrid graph
  -> global path with segment kinds
  -> local route window tracking
```

TGW should not drift back into:

```text
PCD-only traversability
realtime raycast mapping
terrain semantic / StairFlight planning
3D voxel A*
map repair or fallback chains
```

The current remaining work is system hardening, performance, and robot
deployment validation, not another global-planner architecture rewrite.

## Current Pipeline

### 1. Input Contract

TGW consumes N3Mapping pbstream through the N3 nav resource reader.

Required pbstream properties:

```text
native dense optimized trajectory
optimized keyframe poses
non-empty keyframe clouds
map/body frame metadata
no keyframe-fallback trajectory
no degraded dense trajectory
```

TGW fails explicitly if this contract is not met. It does not silently fall
back to `global_map.pcd`.

### 2. Geometry Index

`ExperienceGeometryIndex` is built once from keyframe clouds.

It owns:

```text
raw-resolution evidence
support columns / height runs
nav-resolution support candidates
body obstruction evidence
sampled debug geometry
```

This replaced repeated full keyframe cloud transforms in the projector,
surface builder, and debug publishers.

Relevant files:

```text
include/tgw_planner/core/experience_geometry_index.hpp
src/core/experience_geometry_index.cpp
```

### 3. Support Projection And Reachable Surface

`TrajectoryProjector` projects dense trajectory samples to support surfaces
using the shared geometry index.

`ExperienceSurfaceBuilder` builds:

```text
observed trajectory support
non-expanding virtual bridge cells
support candidates
anchored reachable expansion
hole filling inside anchored components
clearance / risk fields
```

Important semantic boundaries:

```text
raw point != support
bridge != expansion anchor
support candidate != reachable until anchored by experience
```

Relevant files:

```text
include/tgw_planner/core/trajectory_projector.hpp
src/core/trajectory_projector.cpp
include/tgw_planner/core/experience_surface_builder.hpp
src/core/experience_surface_builder.cpp
include/tgw_planner/core/reachable_expander.hpp
src/core/reachable_expander.cpp
```

### 4. Local Surface Graph

`ExperienceSurfaceGraph` is a layer-safe 2D surface graph:

```text
node = reachable surface cell with z as an attribute
edge = valid local movement on a surface island
z is not an independent search dimension
```

This graph does not need to connect all floors. It owns local, same-island
movement and clearance/risk-aware costs.

Relevant files:

```text
include/tgw_planner/core/experience_surface_graph.hpp
src/core/experience_surface_graph.cpp
```

### 5. Dense Trajectory Backbone

`ExperienceBackboneGraph` uses dense trajectory as the owner of global
experience topology. It creates backbone nodes from dense trajectory samples,
backbone edges from consecutive trajectory samples, and portals between nearby
surface nodes and backbone nodes.

This solved the earlier mistake where TGW tried to infer cross-floor topology
purely from sparse surface geometry.

Relevant files:

```text
include/tgw_planner/core/experience_backbone_graph.hpp
src/core/experience_backbone_graph.cpp
```

### 6. Unified Hybrid Graph

`HybridExperienceGraph` contains:

```text
surface nodes / surface edges
backbone nodes / backbone edges
portal edges
```

`HybridExperiencePlanner` searches this unified graph directly. The old
three-stage portal-pair planner was removed as the main route strategy.

Path output preserves segment kinds:

```text
Surface
Backbone
Portal
```

Relevant files:

```text
include/tgw_planner/core/hybrid_experience_planner.hpp
src/core/hybrid_experience_planner.cpp
```

### 7. Local Tracking Core

TGW local execution is core-first:

```text
RouteProgressTracker
  -> LocalPathSmoother
  -> RegulatedPurePursuitController
  -> RollingLocalMap stop layer
```

The global path is a topological guide. The tracker follows a short local
route window, not every jagged global path point.

Current behavior:

```text
RouteProgressTracker:
  projects odom onto path using a local forward window
  fails closed if projection is not found

LocalPathSmoother:
  builds a short Bezier local path
  rejects corner cutting outside the route corridor
  falls back to a route-following local window when Bezier smoothing is too
  curved or outside the corridor

RegulatedPurePursuitController:
  tracks the local path
  applies speed cap, curvature regulation, acceleration limit, and goal
  tolerance

RollingLocalMap:
  provides a 2D inflated stop layer after height filtering against nearby
  local path height
```

Relevant files:

```text
include/tgw_planner/core/route_progress_tracker.hpp
src/core/route_progress_tracker.cpp
include/tgw_planner/core/local_path_smoother.hpp
src/core/local_path_smoother.cpp
include/tgw_planner/core/regulated_pure_pursuit.hpp
src/core/regulated_pure_pursuit.cpp
include/tgw_planner/core/rolling_local_map.hpp
src/core/rolling_local_map.cpp
```

### 8. ROS Wrapper

ROS remains a wrapper around the core algorithms.

Main node:

```text
src/humble/tgw_experience_planner_node.cpp
```

Launch/config split:

```text
launch/experience_planner_sim.launch.py
launch/experience_planner_real.launch.py
config/experience_planner_common.yaml
config/experience_planner_sim.yaml
config/experience_planner_real.yaml
```

Simulation config enables kinematic replay and fake odom. Real config enables
tracking but keeps command output on `/tgw_experience/cmd_vel` and requires
explicit arming.

## Validated Golden Scenes

### Scene 20260608

Input:

```text
docs/data/tgw_n3map_nav_filtered.pbstream
```

Role:

```text
stair / cross-floor experience scene
```

Latest golden regression:

```text
cross_floor global sweep: 50 / 50
same_floor_low global sweep: 50 / 50
same_floor_high global sweep: 50 / 50

cross_floor tracking replay: 50 / 50
same_floor_low tracking replay: 50 / 50
same_floor_high tracking replay: 50 / 50
```

Details:

```text
docs/exp/scene_20260608/golden_regression/README.md
```

### Scene 20260610

Input:

```text
docs/data/tgw_n3map_nav_filtered_20260610.pbstream
```

Role:

```text
ramp / grade-change scene
```

Latest golden regression:

```text
cross_floor global sweep: 50 / 50
same_floor_low global sweep: 50 / 50
same_floor_high global sweep: 50 / 50

cross_floor tracking replay: 50 / 50
same_floor_low tracking replay: 50 / 50
same_floor_high tracking replay: 50 / 50
```

Details:

```text
docs/exp/scene_20260610/golden_regression/README.md
```

## Current Performance Snapshot

Fresh benchmark command:

```bash
cd /home/user/ros_ws/to_migrate_ws
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream
```

Measured on the current development CPU:

| Scene | End-to-end preprocess | First query | Peak RSS | Main hotspots |
| --- | ---: | ---: | ---: | --- |
| 20260608 | about 7.9s | 0.27ms | 725 MB | surface build 4.37s, surface graph 1.43s |
| 20260610 | about 13.8s | 14.5ms | 875 MB | surface build 8.22s, surface graph 2.66s |

Interpretation:

```text
global search is already fast;
startup preprocessing is the remaining performance pressure;
surface_build and surface_graph_build are the next optimization targets.
```

## Current Run Commands

Build and tests:

```bash
cd /home/user/ros_ws/to_migrate_ws
colcon build --packages-select tgw_planner
colcon test --packages-select tgw_planner --event-handlers console_direct+
colcon test-result --verbose
```

Golden regression:

```bash
cd /home/user/ros_ws/to_migrate_ws
source install/setup.bash
MPLCONFIGDIR=/tmp/matplotlib python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene all \
  --sample-pairs 50 \
  --plot-limit 8
```

RViz kinematic simulation:

```bash
cd /home/user/ros_ws/to_migrate_ws
source install/setup.bash
ros2 launch tgw_planner experience_planner_sim.launch.py \
  pbstream_path:=/absolute/path/to/n3map.pbstream
```

Real wrapper dry run:

```bash
cd /home/user/ros_ws/to_migrate_ws
source install/setup.bash
ros2 launch tgw_planner experience_planner_real.launch.py \
  pbstream_path:=/absolute/path/to/n3map.pbstream
```

Real tracking still requires explicit arm:

```bash
ros2 service call /tgw_experience/set_tracking_armed std_srvs/srv/SetBool "{data: true}"
```

## Important Historical Lessons

### Raw geometry is not traversability

Early versions made every keyframe point a support cell. This expanded into
walls, sparse artifacts, and ceiling planes. The current model separates raw
geometry evidence, support candidates, observed reachable seeds, inferred
reachable surface, and final graph nodes.

### Bridge cells are not support anchors

Bridge cells are useful to preserve narrow walked gaps where point clouds are
sparse, but they must not seed lateral expansion or turn nearby surfaces into
reachable areas.

### Surface graph does not own cross-floor topology

Trying to force the surface graph to connect floors created either false layer
jumps or correct-but-disconnected failures. Dense trajectory is the correct
owner of experience topology.

### Portal-pair planning was too greedy

A planner that chooses one start portal and one goal portal can follow a huge
piece of the mapping trajectory unnecessarily. The unified hybrid graph allows
the route to enter and exit the backbone wherever the total graph cost is best.

### Global path is not the tracking path

The global path can be jagged. Local tracking must project current odom into a
route window, smooth the local segment, and follow that segment. If smoothing
would cut a corner, the fallback is a route-following local window, not a
tracking failure.

### `tracking_odom_stale` can be a secondary symptom

In kinematic replay, a local-path failure can stop fake odom publication and
then later appear as stale odom. The node now keeps fake odom alive during
local failure so the original failure reason remains visible.

## Current Known Limits

These are not solved by the current core implementation:

```text
real robot dynamics and foot contact
body attitude limits on steep stairs/ramps
hardware e-stop / deadman / command mux
real odom <-> map frame transform if frames differ
sensor latency and localization jumps
dynamic obstacle avoidance beyond stop-only 2D inflated checks
compiled experience cache for instant startup
```

The current system is suitable for:

```text
core global-path regression
core local-tracking replay
RViz kinematic simulation
low-risk interface testing before robot deployment
```

It is not yet a production robot safety system.

## Suggested Next Review Questions

Ask the next reviewer to focus on:

```text
1. Is the current backbone + surface island + hybrid graph model coherent enough
   to freeze as the global planner architecture?

2. Are the local tracking safety gates sufficient before robot-side integration,
   assuming TGW command output remains behind a safety mux?

3. Is the current stop-only local obstacle layer acceptable for first hardware
   tests, or should a stricter body-volume checker be added before deployment?

4. What is the cleanest way to reduce 20260610 preprocessing below 10s without
   adding a compiled cache yet?

5. Which regression metrics should become hard CI gates, and which should stay
   diagnostic because map fragments and N3 input quality vary?
```

Do not ask the reviewer to solve old PCD, raycast, StairFlight, or bridge-only
topology problems. Those are intentionally out of the current architecture.

