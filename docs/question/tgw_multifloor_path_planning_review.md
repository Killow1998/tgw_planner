# TGW Layered Surface Graph Cross-Floor Planning Review

## Prompt For GPT Pro

Please review this as a code/design problem, not as a parameter tuning problem.

TGW is a pbstream experience planner. It consumes N3Mapping dense trajectory and
keyframe clouds, builds an experience reachable surface, then plans on a layered
2D surface graph. The current path planner can still draw invalid vertical
floor-jump paths, or after stricter edge validation, it loses true cross-floor
connectivity.

I need concrete guidance on the right implementation shape:

```text
How should trajectory bridge evidence be represented and connected in the
Layered 2D Surface Graph so that:

1. A path cannot jump vertically between floors.
2. A real walked stair/ramp gap can still be connected.
3. Bridge cells do not become lateral expansion anchors.
4. Graph A* searches only explicit valid surface/bridge edges.
5. The implementation stays clean C++ core + thin ROS wrapper.
```

Please focus on what to change in:

```text
TrajectoryProjector
ExperienceSurfaceBuilder / ExperienceSnapshot
ExperienceSurfaceGraph
SurfaceAstarPlanner
ROS debug topics / stats
tests
```

Do not suggest returning to:

```text
3D voxel A*
realtime raycast mapping
PCD fallback
StairFlight / semantic stair model
global_map.pcd fallback
```

## Current Question

TGW has moved from 3D lattice A* to a layered 2D surface graph:

```text
SurfaceNode = reachable surface cell with x/y and surface height z
SurfaceEdge = local movement proof between two surface nodes
A* expands graph adjacency only, not dz neighbors
```

This is the right high-level direction for a legged robot:

```text
z is a surface attribute, not a commanded motion dimension
```

However, the current graph still does not produce a valid true cross-floor path.

The most recent RViz result showed a path with large vertical white segments:

```text
The path visually jumps between floors.
The start and goal are on reachable surfaces, but the path contains large
layer-to-layer z discontinuities instead of following the stair/ramp surface.
```

The attached user screenshot should be reviewed together with this document.
I cannot embed the chat-uploaded image bytes from the local workspace, so the
important visual symptom is described above.

## Branch And Code State

Repository:

```text
/home/user/ros_ws/to_migrate_ws/src/tgw_planner
```

Branch:

```text
feature/pbstream-experience-reset
```

Latest pushed commit before this diagnostic patch:

```text
08d7673 Plan on layered 2D experience surface graph
```

That commit introduced:

- `ExperienceSurfaceGraph`
- graph-backed `SurfaceAstarPlanner::plan(graph, start, goal)`
- graph-aware start/goal snap in `tgw_experience_planner_node`
- no dz expansion in the new graph A*

Current local diagnostic patch, not yet a final solution:

- adds `SurfaceGraphBuildOptions`
- makes graph edge construction go through `makeContinuousSurfaceEdge()`
- distinguishes normal surface edges from trajectory bridge edges
- rejects adjacent XY graph edges whose true `SurfaceCell.height_m` jump is too large
- adds tests for layer-jump rejection and bounded bridge-height policy

## Verified Build And Test Status

Commands:

```bash
cd /home/user/ros_ws/to_migrate_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
colcon test --packages-select tgw_planner --event-handlers console_direct+
colcon test-result --verbose
```

Result:

```text
Summary: 1 package finished
PASS experience_core_smoke
Summary: 212 tests, 0 errors, 0 failures, 0 skipped
```

## Input Map

The real test input is:

```text
/tmp/tgw_n3map_nav_filtered.pbstream
```

The map has native dense trajectory and filtered nav cloud. TGW still rejects
fallback/degraded dense trajectory and does not use `global_map.pcd`,
realtime raycast, or StairFlight logic.

## Root Cause Found For The RViz Floor Jump

The pushed layered graph code created graph edges using a validator over
`GridIndex{x,y,z}` cells, while the graph path output uses true surface height:

```text
SurfaceNode.z = SurfaceCell.height_m
```

That allowed this bad case:

```text
from.cell.z and to.cell.z are equal or close enough for GridIndex validation
but
abs(from.surface.height_m - to.surface.height_m) is several meters
```

So the graph could contain an edge between two adjacent XY nodes on different
floors. RViz then showed a vertical path segment.

This is not a path-smoothing problem. It is an invalid graph-edge problem.

## Current Local Patch

The current patch changes edge construction to this principle:

```text
Graph edges are not inferred from visual reachability.
Each edge is a local movement proof.
```

Normal surface edge:

```text
both endpoints are non-bridge surface nodes
true surface-height delta <= plan_max_step_height_m
optional slope policy passes
SurfaceTransitionValidator passes footprint / diagonal / swept checks
```

Trajectory bridge edge:

```text
at least one endpoint is a trajectory bridge node
true surface-height delta <= max_trajectory_bridge_height_delta_m
edge is tagged TrajectoryBridge and penalized
```

This removes the illegal multi-meter floor jumps.

## Real Diagnostic After Removing Illegal Jumps

Temporary helper:

```text
/tmp/tgw_high_pair.cpp
```

It builds:

```text
N3 pbstream
  -> ExperienceSurfaceBuilder
  -> ExperienceSurfaceGraph
```

and searches for a graph component with at least 8 m z range.

Output after the local patch:

```text
read_ms 173.569
surface_build_ms 8806.23
build_counts traversable 164899 bridge 508 bridge_z_min 0.55 bridge_z_max 8.35 bridge_z_range 7.8
graph_build_ms 18333.1

component_rank 0 id 0 size 88556 bridge_nodes 16555 z_min -1.09301 z_max 1.11032 z_range 2.20334
component_rank 1 id 4 size 63782 bridge_nodes 11955 z_min 7.23157 z_max 8.35 z_range 1.11843
component_rank 2 id 2 size 574 bridge_nodes 186 z_min 3.51011 z_max 4.26088 z_range 0.750764
component_rank 3 id 1 size 62 bridge_nodes 40 z_min 4.48925 z_max 5.20012 z_range 0.710874
component_rank 4 id 5 size 148 bridge_nodes 61 z_min 5.40037 z_max 5.88121 z_range 0.480843
component_rank 5 id 14 size 16 bridge_nodes 11 z_min 7.42352 z_max 7.84916 z_range 0.425634
component_rank 6 id 6 size 149 bridge_nodes 47 z_min 5.81539 z_max 6.20492 z_range 0.389534
component_rank 7 id 7 size 213 bridge_nodes 76 z_min 2.10698 z_max 2.46132 z_range 0.35434
component_rank 8 id 337 size 3 bridge_nodes 0 z_min 4.85307 z_max 5.19923 z_range 0.346161
component_rank 9 id 568 size 2 bridge_nodes 0 z_min 7.31916 z_max 7.66071 z_range 0.34155

no component with z range >= 8m
```

In this diagnostic:

```text
build_counts bridge = cells with SurfaceLabel::TrajectoryBridge
component_rank bridge_nodes = graph nodes where SurfaceNode.bridge is true,
                              including TrajectoryBridge and LowConfidenceReachable
```

Interpretation:

```text
The previous cross-floor graph connectivity depended on illegal height-jump edges.
After those edges are removed, there is no valid full-height graph component.
```

Important nuance:

```text
bridge cells exist and span z = 0.55m to 8.35m
but they do not form one valid graph chain.
```

So the problem is not simply "no bridge cells". The problem is that bridge cells
are only a set of cells after the builder stage; they do not retain trajectory
order, bridge id, tangent, or endpoint metadata.

## Current Design Gap

The planner now has the right abstraction:

```text
Layered 2D Surface Graph
```

But the graph builder lacks a first-principles representation for trajectory
connectors:

```text
trajectory bridge = ordered connector evidence along dense trajectory
```

Current bridge information:

```text
std::unordered_set<GridIndex> bridge_seed_cells
SurfaceCell.label = TrajectoryBridge
ReachabilityLabel = LowConfidenceReachable
```

Missing bridge information:

```text
bridge_id
bridge_order
bridge_tangent
source projected sample seq/timestamp
entry observed support node
exit observed support node
confidence / gap length
```

Because this metadata is missing, `ExperienceSurfaceGraph` can only connect
bridge cells by local XY adjacency and height thresholds. That is insufficient:

```text
if thresholds are loose:
  graph can jump between layers

if thresholds are strict:
  graph becomes disconnected across missing stair observations
```

## First-Principles Requirement

The correct cross-floor connector should not be:

```text
any nearby low-confidence cells with similar z
```

It should be:

```text
an explicit ordered trajectory bridge edge chain between two observed support
samples that the robot actually traversed
```

Bridge semantics should be:

```text
ObservedTrajectorySupport:
  can anchor normal surface expansion
  can connect to normal graph edges

TrajectoryBridge:
  can preserve path continuity along dense trajectory gaps
  cannot laterally expand
  cannot create arbitrary same-height graph edges
  cannot jump several meters
  must connect in trajectory order
```

## Review Questions

1. Should `TrajectoryProjector` output explicit bridge segments instead of only
   `bridge_seed_cells`?

   Proposed shape:

   ```cpp
   struct TrajectoryBridgeSegment {
     uint32_t bridge_id;
     int order;
     ProjectedSupportSample from;
     ProjectedSupportSample to;
     std::vector<GridIndex> footprint_cells;
     Point3 tangent;
     double gap_length_m;
     double height_delta_m;
   };
   ```

2. Should `ExperienceSnapshot` carry bridge connector metadata into
   `ExperienceSurfaceGraph`?

   Proposed graph rule:

   ```text
   normal edges:
     local XY support graph

   bridge edges:
     explicit edges from bridge segment order
     not inferred from local adjacency
   ```

3. How should bridge edges attach to normal support components?

   Candidate rule:

   ```text
   bridge entry/exit may attach only to observed support endpoint cells or
   planner-valid normal nodes within one footprint radius of the projected
   support endpoint
   ```

4. Should bridge cells remain visible as nodes, or should the planner store
   bridge as edges only?

   Candidate answer:

   ```text
   Use narrow bridge nodes for visualization/path output, but only add bridge
   adjacency according to bridge_id/order, not by free local XY expansion.
   ```

5. Should graph edge validation use `SurfaceCell.height_m` everywhere and stop
   using `GridIndex.z` for movement validation?

   Candidate answer:

   ```text
   Yes for graph planning. GridIndex.z is only a storage key. Real movement
   continuity must use surface height.
   ```

## Current Recommendation

Do not solve this by increasing:

```text
plan_max_step_height_m
max_trajectory_bridge_height_delta_m
plan_max_iterations
```

Those parameters can hide the symptom by recreating invalid layer jumps.

The next clean implementation should be:

```text
TrajectoryProjector emits ordered bridge segments
ExperienceSurfaceBuilder stores bridge connector metadata
ExperienceSurfaceGraph adds:
  normal support edges from local surface continuity
  bridge connector edges from ordered trajectory evidence
A* searches only this graph
RViz publishes bridge connector debug separately
```

Acceptance should include:

```text
1. no path edge has a multi-meter z jump
2. true cross-floor path succeeds only if ordered bridge/stair connector exists
3. if connector metadata is missing, planner fails explicitly instead of drawing
   a vertical path
4. max_path_dz per edge is bounded and reported
5. bridge edge count and largest bridge-connected z range are reported in stats_json
```
