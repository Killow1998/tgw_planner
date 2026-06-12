# TGW Edge Benchmark Plan

Purpose:

```text
Verify whether the current pbstream experience planner is fast enough on
target edge hardware before doing another performance architecture rewrite.
```

This plan intentionally does not change TGW algorithms. It measures the current
baseline.

## Baseline

Branch:

```text
rebuild/experience-backbone
```

Stable code baseline:

```text
e7ae036 Tighten TGW preprocessing data paths
```

Later commits may add documentation only. If code changes after this baseline,
rerun all benchmark and golden checks.

## Devices

Run this plan on each available target:

```text
Jetson Orin Nano
Jetson Orin NX
RK3588
```

Record CPU governor / power mode if available.

## Inputs

Use the golden pbstreams already tracked in this package:

```text
docs/data/tgw_n3map_nav_filtered.pbstream
docs/data/tgw_n3map_nav_filtered_20260610.pbstream
```

## Build

From the workspace root:

```bash
colcon build --packages-select tgw_planner
```

## Benchmark Commands

```bash
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream

build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream

build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream \
  --roi-distance 1.8
```

## Golden Regression

```bash
MPLCONFIGDIR=/tmp/matplotlib \
python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene all \
  --sample-pairs 50 \
  --plot-limit 0
```

## Metrics To Record

For each device and each pbstream:

```text
preprocess_ms
read_pbstream_ms
geometry_index_build_ms
trajectory_projection_ms
surface_build_ms
surface_expansion_ms
surface_graph_build_ms
backbone_build_ms
hybrid_graph_build_ms
first_query_ms
peak_rss_mb
transformed_points
roi_skipped_points
raw_geometry_cells
support_candidates
traversable_cells
graph_nodes
graph_edges
```

For golden regression:

```text
cross_floor sampled_success / sampled_queries
same_floor_low sampled_success / sampled_queries
same_floor_high sampled_success / sampled_queries
tracking_replay passed / total
max_detour_ratio
max_backbone_ratio
max_portal_switch_count
```

## Decision Gates

Accept current algorithm on edge hardware if:

```text
golden regression passes
preprocess < 30s on target device
first_query is ms to low tens-of-ms scale
peak RSS leaves enough system memory for robot stack
tracking replay remains 50/50 for golden scenes
```

If edge preprocess is between 30s and 60s:

```text
Keep the current algorithm.
Consider compiled experience cache as deployment optimization.
Do not rewrite GeometryIndex yet.
```

If edge preprocess is above 60s, or peak RSS is too close to device limits:

```text
Do not retry side-index wrappers around unordered_map.
Choose one of:
  short-term: compiled experience cache
  long-term: native chunked/tiled GeometryIndex from point insertion onward
```

## What Not To Do

```text
Do not reintroduce PCD fallback.
Do not reintroduce realtime raycast.
Do not reintroduce StairFlight / terrain semantics.
Do not reintroduce 3D voxel A*.
Do not retry SupportCandidateStore side-index refactor.
Do not shrink ROI below the golden-tested value without running ROI sweep.
```
