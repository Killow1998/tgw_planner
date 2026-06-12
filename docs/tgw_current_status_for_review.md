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

The default trajectory ROI is currently:

```text
geometry_roi_distance_to_trajectory_m = 1.2
```

This keeps TGW focused on the robot's experienced navigation corridor instead
of rebuilding a full general terrain map. The value is evidence-based for the
current golden scenes: both scenes still pass dominant cross-floor and
same-floor sweeps at 50 / 50.

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
| 20260608 | about 4.30s | 0.12ms | 682 MB | surface build 2.00s, surface graph 0.95s |
| 20260610 | about 6.15s | 0.26ms | 732 MB | surface build 3.35s, surface graph 1.52s |

Interpretation:

```text
global search is already fast;
startup preprocessing now meets the current sub-10s proof point on the
development CPU;
surface expansion and surface_graph_build remain the next optimization targets
before any compiled cache work.
```

Recent 20260610 benchmark breakdown:

```text
read_pbstream: 0.18s
geometry_index_build: 0.77s
trajectory_projection: 0.23s
surface_build: 3.35s
  expansion frontier/wave: about 1.58s / 1.74s
  hole fill: about 0.42s
  clearance: about 0.44s
surface_graph_build: 1.52s
backbone_build: 0.09s
hybrid_graph_build: 0.02s
first_query: 0.26ms
```

The acceleration came from:

```text
1. using the shared ExperienceGeometryIndex as the only keyframe cloud pass;
2. making reachable expansion avoid repeated neighbor offset generation,
   temporary vectors, redundant component lookups, and one unused anchor
   height lookup;
3. making clearance / boundary / risk construction pre-allocate hot data and
   reuse precomputed clearance neighbor offsets;
4. making surface graph node filtering compute endpoint support and
   directional footprint support in one pass, then reject obvious
   cross-component / unsupported-footprint edges before the full edge builder;
5. making hole filling frontier-based after the first iteration, so only new
   filled cells trigger follow-up candidate checks;
6. replacing expansion/component `std::queue` frontiers with compact vectors
   and head indices to avoid deque overhead in hot BFS loops;
7. using one packed XY bucket for surface graph adjacency and snap / portal
   lookup paths, removing the older duplicate GridIndex-keyed XY map;
8. tightening the default trajectory ROI from 1.8m to 1.2m after regression
   evidence showed both golden scenes still pass.
```

The ROI is a bounded experience-planner knob, not the only optimization.
With the current code and a wider diagnostic ROI:

```text
scene_20260610, roi=1.8m:
  preprocess: about 8.64s
  surface_build: about 4.69s
  surface_graph_build: about 2.20s
```

This is still well below the older 13.8s baseline, but the default remains
1.2m because it preserves the current golden routes while reducing startup
load. New scenes should run ROI sweep before changing this default.

### Compact Workspace Falsifier

An attempted `unordered_map` to compact-array rewrite for
`ReachableExpander` was benchmarked and rejected rather than merged.

Tested variants:

```text
1. all-geometry `GridIndex -> state index`
2. anchored-component-only `GridIndex -> state index`
3. sparse fixed-size chunk lookup
4. bounded dense local array over the anchored component bounding box
5. source-owned `SupportCandidateStore` with vector entries + packed-key lookup
```

Result on `scene_20260610`:

```text
baseline after e7ae036: about 6.15s preprocess, about 732MB RSS
all / anchored / chunked side-index variants: slower surface_build
64M-cell dense array variant: about 6.31s preprocess, about 786MB RSS
source-owned vector store variant:
  component labeling improved, but expansion frontier regressed;
  preprocess about 6.9s, peak RSS about 760MB
```

Interpretation:

```text
The current sparse grid still pays either hash lookup cost or dense memory
cost when a compact workspace is bolted onto the side. The clean next target
is not another side index inside ReachableExpander, nor a vector wrapper around
the existing unordered_map. It is to make GeometryIndex build support evidence
directly into a native chunked / tiled representation, so neighbor access is
array-local from the moment points are inserted.
Until that rewrite is justified, the verified baseline should stay in place.
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

4. What is the cleanest next step to reduce surface expansion and graph build
   further without adding compiled cache yet?

5. Which regression metrics should become hard CI gates, and which should stay
   diagnostic because map fragments and N3 input quality vary?
```

Do not ask the reviewer to solve old PCD, raycast, StairFlight, or bridge-only
topology problems. Those are intentionally out of the current architecture.
