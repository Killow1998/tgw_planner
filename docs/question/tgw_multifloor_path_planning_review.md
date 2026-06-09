# TGW True Multi-Floor Path Planning Review Request

## Summary

TGW is now on the pbstream experience-planner route. The previous ceiling /
upper-plane false expansion issue is mostly fixed, and a planner connectivity
layer now exists. However, true multi-floor path planning is still not solved.

The current key failure is:

```text
RViz / planner component says a cross-floor component exists,
but a real 9 m cross-floor PlanPath request does not return within an
interactive time budget.
```

This is not a mapping-route failure yet. It is now a planner-graph / search
problem.

## Current Code State

Branch:

```text
feature/pbstream-experience-reset
```

Relevant latest commit:

```text
28862a0 Add planner connectivity graph for experience paths
```

Mainline remains strict pbstream experience planning:

- no `global_map.pcd` fallback
- no realtime raycast mapper
- no PCD-only planner
- no StairFlight / curved-stair / spiral-stair branch
- no keyframe fallback dense trajectory

Implemented since the previous review:

1. `SurfaceTransitionValidator`
   - extracted transition rules from `SurfaceAstarPlanner`
   - shared by A* and planner connectivity
   - validates traversable cell, diagonal corner support, swept footprint, and
     footprint support

2. `PlannerConnectivityLayer`
   - builds components using `SurfaceTransitionValidator::validNeighbors()`
   - endpoint cells are cached before BFS to avoid repeated endpoint footprint
     checks
   - exposes component id, component sizes, min/max z, and multi-floor count

3. Planner-aware snap
   - start/goal snap now selects from planner-reachable cells
   - if snapped start and goal are in different planner components, the node
     fails before A*

4. Footprint support ratio
   - `RobotFootprint::supportReport()` exists
   - planner uses configurable `planner_footprint_min_support_ratio`
   - current default: `0.80`

5. Debug outputs
   - `/tgw_experience/planner_component_cloud`
   - `stats_json` planner component metrics
   - RViz config includes planner component cloud

## Build And Unit Verification

The package builds and tests:

```bash
cd /home/user/ros_ws/to_migrate_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
colcon test --packages-select tgw_planner --event-handlers console_direct+
colcon test-result --verbose
```

Result:

```text
Summary: 212 tests, 0 errors, 0 failures, 0 skipped
```

## Runtime Map Evidence

Input pbstream:

```text
/tmp/tgw_n3map_nav_filtered.pbstream
```

Runtime load log:

```text
loaded n3map experience resource:
  keyframes=799
  dense_trajectory=10649
  raw_geometry=836844
  support_candidates=313173
  projected_support=10579
  observed_seed=41542
  bridge_seed=526
  expanded_reachable=164899
  planner_components=82
  largest_planner_component=154006
  planner_multifloor_components=1
  rejected_projection=70
  no_support=70
  ambiguous_multifloor=0
  reanchored_support=2
  retry_support=65
  bridge_used_as_expansion_anchor=0
  support_components=6144
  anchored_components=95
  rejected_unanchored_component=1739
  footprint_rejected=1
  body_obstructed_rejected=154056
  anchor_envelope_rejected=4723
  hole_filled=28771
  frame=map
  body=body
```

Important observations:

- The largest planner component has `154006` cells.
- There is exactly one planner component with multi-floor z range.
- `bridge_used_as_expansion_anchor=0`, so bridge expansion leakage is not the
  current issue.
- The real issue is search/query behavior on this very large component.

## Correction To Previous 50-Query Claim

I previously reported a 50-query pass, but that was not a valid proof of
multi-floor planning.

What actually happened:

1. A seed path was found.
2. 50 short cross-height subqueries were sampled along that seed path.
3. The first successful subquery was:

```text
start = (10.15, -34.15, 0.35)
goal  = (11.55, -34.15, 1.35)
delta_z = 1.0 m
```

This only proves local cross-height pathing on a small segment. It does not
prove true floor-to-floor planning. It should not be used as acceptance
evidence.

The user correctly rejected this as insufficient.

## True 9 m Cross-Floor Test

A temporary diagnostic selected endpoints from the same large planner component
with about 9 m vertical separation.

Diagnostic output:

```text
component 0
size 154006
z_min -1.05
z_max 8.45
delta_z 9

start -39.85 -26.25 -0.85
start clearance 0.95

goal -27.15 -2.25 8.15
goal clearance 1.93995
```

So this is not a tiny local query:

```text
start = (-39.85, -26.25, -0.85)
goal  = (-27.15,  -2.25,  8.15)
z difference = 9.0 m
same planner component = true
```

The ROS service call was:

```bash
ros2 service call /tgw_experience/plan_path tgw_planner/srv/PlanPath \
  "{start: {header: {frame_id: map}, pose: {position: {x: -39.85, y: -26.25, z: -0.85}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: -27.15, y: -2.25, z: 8.15}, orientation: {w: 1.0}}}}"
```

Result:

```text
No response after more than two minutes.
The request was stopped manually.
```

This is the current core failure.

## Current Failure Interpretation

The planner graph is not obviously disconnected anymore. Instead, the search is
too expensive or insufficiently guided for true floor-to-floor queries.

Current likely explanation:

```text
Experience reachable surface:
  large multi-floor component exists

Planner connectivity:
  same component check passes

Bare A*:
  searches directly over a ~154k-cell 3D surface component
  with local footprint/swept transition validation
  and weak Euclidean heuristic for a maze-like multi-floor route

Result:
  long cross-floor query can become non-interactive
```

This means `PlannerConnectivityLayer` solved only:

```text
is there a transition-valid graph component?
```

It did not solve:

```text
how to find a long route through that component quickly?
```

## Suspected Root Causes

### 1. Bare A* Is The Wrong Top-Level Search For A 154k-Cell Multi-Floor Surface

`SurfaceAstarPlanner` still expands local grid neighbors directly.

For a multi-floor building-like graph, Euclidean distance can be a weak
heuristic because the valid route may require:

- following corridors
- reaching stairs
- climbing the staircase
- continuing through another corridor

The straight-line heuristic does not know about stair/connector structure.

### 2. PlannerConnectivityLayer Has No Shortest-Path Or Landmark Information

The connectivity layer only gives component id and z range. It does not provide:

- a compact skeleton
- portals/connectors
- stair/vertical transition corridors
- landmark distances
- component-internal shortest path hints
- trajectory-anchored route priors

So A* still starts cold.

### 3. Trajectory Evidence Is Not Used As A Planning Prior

TGW's strongest evidence is the dense trajectory. The robot has physically
walked the route, and the planner has that path as experience. But the current
A* does not use the trajectory as a coarse guide.

Possible missing idea:

```text
Use dense trajectory / observed support corridor as a topological backbone.
Plan coarse route on a trajectory/skeleton graph first.
Then refine locally on the reachable surface.
```

### 4. The Selected 9 m Endpoints May Be In The Same Component But Far In Graph Distance

The diagnostic selected high-clearance low/high endpoints from the same
component. That proves same component, but not that the path is short.

The endpoint pair:

```text
start = (-39.85, -26.25, -0.85)
goal  = (-27.15,  -2.25,  8.15)
```

may require traversing a long route through corridors and stairs. This is still
a valid planning query, but it is a harder test than local stair traversal.

### 5. Current Debugging Lacks A* Frontier / Closed Set Visualization

For the failing long query, RViz currently shows the map and component, but not:

- how far A* expanded
- whether A* found the stair connector
- whether A* is trapped in a floor region
- whether the heuristic drives it toward a wrong area
- rejected transition reasons during search

## Questions For GPT Pro Review

Please review the current situation from first principles.

1. What should be the next planner architecture?

Candidate options:

```text
A. Keep single-level A*, but improve heuristic / tie breaking / weights.
B. Add weighted A* or anytime weighted A*.
C. Build a coarse graph / skeleton / portal graph from planner component.
D. Use dense trajectory as a topological backbone, then refine locally.
E. Build a hierarchical planner:
     planner component cells
       -> corridor/trajectory/skeleton graph
       -> local surface A*
```

2. Should the dense trajectory become the default global guidance layer?

The robot physically traversed the route, so the dense trajectory is a strong
experience prior. Should the planner:

```text
snap start to nearest trajectory-backed reachable corridor
snap goal to nearest trajectory-backed reachable corridor
search along trajectory / support corridor first
then locally expand/refine on reachable surface
```

3. Should TGW compute a `PlannerSkeletonLayer`?

Possible skeleton sources:

- trajectory support samples
- medial axis / clearance ridge of reachable surface
- support component portals
- vertical transition zones where z changes rapidly
- bridge corridor endpoints

4. Should multi-floor connectors be explicitly detected?

Not semantic stairs, but purely geometric/topological connectors:

```text
cells where connected planner path changes z continuously
within the same component
```

Could this produce:

```text
floor region graph
connector graph
portal nodes
```

without adding StairFlight semantics?

5. Should long `PlanPath` queries have a hard time/iteration budget and return
diagnostics instead of blocking?

Current behavior is bad for ROS interaction:

```text
service call can hang for minutes
```

Suggested requirement:

```text
PlanPath either returns a path or returns a structured failure within a bounded
time, with frontier/closed-set debug output.
```

6. What should the next acceptance test be?

The previous 50 local subqueries were not sufficient. A better target may be:

```text
N automatically generated queries with:
  same planner component
  delta_z >= 8.0 m
  endpoint clearance >= robot_width_m
  bounded runtime per query
```

But if the query is too global, maybe the test should first validate:

```text
trajectory-backed full-route replay succeeds
then arbitrary start/goal-to-trajectory local connections succeed
```

## Desired Acceptance Target

The next implementation should prove true floor-to-floor planning, not local
1 m height changes.

Minimum target:

```text
For /tmp/tgw_n3map_nav_filtered.pbstream:

1. Load pbstream and build experience surface.
2. Build planner graph.
3. Run at least one path query with delta_z >= 8.0 m.
4. Return within an interactive bound, ideally < 5 s, definitely < 15 s.
5. Publish /tgw_experience/path in RViz.
6. Publish useful failure/debug clouds if it cannot find a path.
```

Stronger target:

```text
Run 20-50 generated cross-floor queries:
  delta_z >= 8.0 m
  same planner component
  planner-aware snapped endpoints
  no query blocks indefinitely
  success/failure has explainable diagnostics
```

## Important Constraint

Do not solve this by restoring old TGW concepts:

- no realtime raycast global mapping
- no PCD fallback planner
- no StairFlight / curved stair / spiral stair semantic planner
- no global terrain understanding framework

The fix should stay within:

```text
pbstream experience evidence
planner-valid reachable surface
trajectory/topological guidance
bounded planning search
```

## Current Working Hypothesis

The most promising next direction is probably not more surface expansion.

The surface and planner component are already large and cross-floor. The missing
piece is a topological / hierarchical planner:

```text
ExperienceSnapshot
  -> PlannerConnectivityLayer
  -> Trajectory/Skeleton/Portal guidance graph
  -> local SurfaceAstarPlanner refinement
  -> bounded PlanPath service
```

This should preserve the thesis:

```text
TGW does not understand terrain semantically.
TGW consumes N3Mapping experience and plans on a proven reachable manifold.
```
