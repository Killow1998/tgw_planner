# Stair Flight Segmentation Issues

## Context

`tgw_planner` is currently extracting traversable surfaces and stair flights from a static multi-floor PCD map. The planner now uses a rectangular robot footprint:

- Length: 0.70 m
- Width: 0.43 m
- Height: 0.50 m
- Base reference offset from front edge: 0.20 m

The reference PCT building map is stored locally under `test_data/pcd/` and is ignored by git because it is a large map artifact.

## Current Implementation Summary

The current core pipeline is:

1. Build occupied voxel columns from the input PCD.
2. Extract standing surface candidates from the top of each occupied z-run.
3. Classify candidates into floor, stair, rejected clearance, rejected collision, rejected ceiling, or rejected noise sets.
4. Build local stair segments from stair cells using local z-gradient direction.
5. Fit each stair segment into a `StairFlight` with:
   - uphill axis
   - side axis
   - low and high endpoints
   - width, length, height, slope, and residual checks
   - low/high nearby floor component IDs
   - centerline and safe corridor
6. Run A* with stair-aware constraints:
   - rectangular footprint support checks
   - stair center cost
   - portal constraints for entering and leaving stairs
   - same-flight directional constraints
   - limited cross-flight portal transitions

RViz debug output includes stair flight IDs, stair centerlines, safe corridors, rejected stair noise, accepted floor, accepted stair, and planner start/goal markers.

## Verified State

Using `pct_building2_9.pcd` at 0.10 m resolution, the current build reports:

```text
stair_centerlines: 24
accepted_stair_flight_count: 24
stair_flight_diagnostics:
  raw_segments=2004
  segment_width_rejected=1917
  fit_rejected=63
  accepted_candidates=24
  merged_candidates=24
stair_flight_fit_rejects:
  too_few=2
  no_axis=0
  short_or_low=58
  negative_slope=0
  slope_out=1
  residual=0
  nonmonotonic=0
  narrow=2
  missing_portals=0
  same_floor_ends=0
```

The reference smoke test still succeeds for the broad building path:

```text
pct_building: success=True
```

## Main Remaining Failure

A manually selected point pair around a real stair still fails:

```text
start_raw: [11.650, 3.050, 11.350]
goal_raw:  [8.450, 2.650, 8.650]
start_snapped: [116, 30, 113]
goal_snapped:  [84, 26, 86]
start_cell_type: stair=false floor_or_landing=true flight_id=-1
goal_cell_type:  stair=true floor_or_landing=false flight_id=5
result: A* failed to find a path
```

This means the start and goal both snap to traversable cells, but the stair/landing graph between them is not connected under the current constraints. The likely missing part is the upper continuation between the goal-side stair and the start-side platform, not the endpoint snapping itself.

## Important Observation: Width Is Probably Not The Main Cause

The rejected-fit counters show only two `narrow` rejections. The visually inspected stairs also appear to have consistent width. The dominant failure reason is `short_or_low`, not width.

Therefore, the current problem should not be treated as "stairs are too narrow." Width should remain a footprint feasibility constraint, not the primary stair detection criterion.

## Current Failure Hypothesis

The likely failure is caused by over-fragmentation of stair surfaces:

1. Real stair flights are split into multiple short local segments by gaps, platforms, or sparse stair cells.
2. Some real stair fragments do not meet the minimum flight height or length checks and are rejected as `short_or_low`.
3. Once those fragments are rejected, the remaining accepted flights can look correct in isolation but fail to connect through the whole stair chain.
4. The A* constraints then correctly refuse unsafe direct transitions, but there is no accepted intermediate stair/landing route left.

This is consistent with:

- many raw segments: `raw_segments=2004`
- many early segment prefilter rejections: `segment_width_rejected=1917`
- dominant fitted-flight rejection reason: `short_or_low=58`
- very few narrow rejections: `narrow=2`
- failed path where start is floor/landing and goal is stair, with no valid graph route between them

## Why Manual Threshold Tuning Is Not Enough

Hard-coded global thresholds are fragile here because the map contains:

- long regular stairs
- short stair fragments near landings
- split stair pieces caused by point-cloud sampling
- platform-to-stair transitions
- curved or building-edge structures that can look like short sloped surfaces

Lowering minimum height and length globally can recover real fragments, but it can also admit false stair-like structures. Raising thresholds removes false positives, but it breaks real stair connectivity.

The planner needs data-driven, map-level normalization instead of manual tuning for each map.

## Recommended Next Algorithm Direction

Replace fixed stair-flight acceptance with a two-stage adaptive process:

1. Collect loose stair-fragment candidates.
   - Use permissive geometric bounds.
   - Record per-candidate length, height, slope, residual, width, endpoint proximity, and local support density.

2. Estimate map-level stair statistics.
   - Typical slope range from robust percentiles.
   - Typical width range from robust percentiles.
   - Typical z-step or flight height bands.
   - Typical straight-flight axis clusters.

3. Merge or rescue fragmented candidates before final rejection.
   - Merge fragments with compatible slope, axis, width, and endpoint proximity.
   - Allow short fragments if they bridge two accepted stair/landing structures.
   - Treat fragments near landings differently from isolated noise.

4. Classify candidates by score instead of one hard threshold.
   - Positive evidence: slope consistency, endpoint portals, floor/landing adjacency, width above footprint, monotonic z trend.
   - Negative evidence: high residual, no portals, no adjacent stair/floor, inconsistent slope, isolated short segment.

5. Build an explicit stair-flight graph.
   - Nodes: floor components, landing components, stair flights.
   - Edges: low/high portal connections only.
   - A* should follow this graph for stair transitions instead of relying only on local voxel adjacency.

## Acceptance Targets

For the PCT building map:

- Extract 24 real stair centerlines.
- Avoid creating extra centerlines on curved architectural edges.
- Preserve the successful broad building smoke-test path.
- Make the manually selected stair pair connect through accepted stair/landing transitions.
- Keep `narrow` as a footprint feasibility rejection, not a map-specific tuning knob.

For dirty real-world maps:

- Recover fragmented stair surfaces when enough geometric evidence exists.
- Reject ceiling and human-shadow artifacts using clearance, support, endpoint, and graph consistency.
- Avoid requiring per-map manual retuning of min height, min length, or width thresholds.

## Surface Refactor Probe

The realtime/surface refactor added `tgw_surface_pcd_smoke` to test clean PCDs
through `ProbabilisticVoxelMap -> SurfaceExtractor -> Clearance/Risk ->
SurfaceAstarPlanner`, bypassing the old StairFlight path.

Current reference results:

```text
surface_pct_building:
  success=true
  start_surface_component=0
  goal_surface_component=0
  largest_surface_component_size=82845
  final_path_validated=true

surface_pct_spiral:
  success=false
  reason="surface A* failed to find a path"
  start_surface_component=5
  goal_surface_component=15
  start_surface_component_size=20144
  goal_surface_component_size=16650
  surface_component_count=811
  start_goal_component_gap_m=0.8
  gap_start_cell=[-109,-92,1]
  gap_goal_cell=[-113,-92,1]
  gap_line_cells=5
  gap_line_occupied_cells=2
  gap_line_surface_cells=3
  gap_line_traversable_cells=3
  gap_line_forbidden_cells=2
```

This means the PCT spiral failure is currently a surface connectivity problem:
the chosen start and goal snap to different traversable surface components. It
should not be treated as only a stair centerline smoothing issue. The next useful
work is not to blindly bridge the gap: the closest component gap currently
contains occupied/forbidden cells. The next useful investigation is whether those
occupied cells are true geometry, railing/wall artifacts, or PCD-mode artifacts
that realtime ray clearing would remove.

## Useful Debug Signals To Keep

The following logs are useful and should remain available during development:

```text
stair_flight_diagnostics
stair_flight_fit_rejects
accepted_stair_flight_count
stair_centerlines
per-flight width, length, slope, z range, endpoints, and component IDs
start_cell_type / goal_cell_type with flight IDs
```

## 2026-06-02 Spiral Surface Gap Evidence

`tgw_surface_pcd_smoke` now reports per-cell diagnostics for the closest
surface-component gap. For `spiral0.3_2.pcd` at 0.20 m resolution, the closest
gap between the snapped start and goal components is:

```text
gap_start_cell=[-109,-92,1]
gap_goal_cell=[-113,-92,1]
gap_line_cells=5
gap_line_occupied_cells=2
gap_line_forbidden_cells=2
nearest_clear_component_gap_m=0
```

The line alternates between traversable cells and occupied/forbidden columns:

```text
[-109,-92,1]: occ=0 support_below=1 surface=1 trav=1 forbid=0 head_clear=1 runs=0..0
[-110,-92,1]: occ=1 support_below=1 surface=0 trav=0 forbid=1 head_clear=0 runs=-4..6|8..8
[-111,-92,1]: occ=0 support_below=1 surface=1 trav=1 forbid=0 head_clear=1 runs=0..0
[-112,-92,1]: occ=1 support_below=1 surface=0 trav=0 forbid=1 head_clear=0 runs=-4..6|8..8
[-113,-92,1]: occ=0 support_below=1 surface=1 trav=1 forbid=0 head_clear=1 runs=0..0
```

This is an important distinction from earlier stair-fragment failures: the
spiral map is not merely missing a smoothed centerline or a local A* shortcut.
The current raw voxel surface graph has no occupied/free-clear portal within
2.0 m between the two snapped components. Bridging this directly would mean
planning through columns that the current occupancy model considers occupied
through the robot body height.

## 2026-06-02 Tomogram Probe

`tgw_tomogram_pcd_smoke` was added as a narrow experiment to test the PCT-style
representation:

```text
PCD -> layered ground/ceiling slices -> traversability cost -> gateway-aware A*
```

It intentionally does not replace the ROS wrapper or the realtime map path yet.
The first probe on `spiral0.3_2.pcd` shows that a naive tomogram is still
insufficient:

```text
success=false
tomogram_grid=[419,216,48]
traversable_cells=1767179
start_node=[1,232,136]
goal_node=[1,182,141]
expanded_nodes=19132
```

The useful conclusion is negative but concrete: PCT's result is not reproduced
by only adding slices and same-XY gateway edges. Their planner also uses
layer-simplification, inflated cost fields, traversability gradients, and
unsafe-grid layer correction. The next implementation step should therefore be
either:

1. port the full tomogram/elevation-map semantics into `tgw_planner`, or
2. keep raw PCD spiral unsupported and rely on realtime ray-cleared mapping plus
   explicit future curved-stair support.

What should not be done: lowering local thresholds or adding a direct bridge
across the reported gap. The gap currently contains occupied columns, so that
would be a safety regression rather than a first-principles fix.

The following RViz layers are also useful:

- accepted stair cloud
- accepted floor cloud
- rejected stair noise cloud
- stair flight ID cloud
- stair centerline markers
- stair portal markers
- stair safe corridor markers
