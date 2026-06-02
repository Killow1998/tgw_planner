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

The following RViz layers are also useful:

- accepted stair cloud
- accepted floor cloud
- rejected stair noise cloud
- stair flight ID cloud
- stair centerline markers
- stair portal markers
- stair safe corridor markers

