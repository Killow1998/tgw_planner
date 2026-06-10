# Same-Floor Core Sweep - scene_20260610

Date: 2026-06-10

Input: `docs/data/tgw_n3map_nav_filtered_20260610.pbstream`

Scene note: this scene is a ramp/grade-change scene rather than a stair scene. Cross-height connectivity should therefore mostly come from continuous traversed ramp experience plus the dense trajectory backbone, not from stair-step bridge behavior.

## Commands

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

build/tgw_planner/tgw_experience_global_sweep \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream \
  --mode same-band --same-z-min 2.20 --same-z-max 2.70 \
  --export-jsonl /tmp/tgw_20260610_same_low.jsonl

build/tgw_planner/tgw_experience_global_sweep \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream \
  --mode same-band --same-z-min 3.10 --same-z-max 3.60 \
  --export-jsonl /tmp/tgw_20260610_same_high.jsonl
```

## Results

| Band | Queries | Success | Mean detour | Max detour | Mean backbone ratio | Max portal switches |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Lower ramp band, z [2.20, 2.70] | 50 | 50 | 1.63639 | 2.84363 | 0.03675 | 6 |
| Upper ramp band, z [3.10, 3.60] | 50 | 50 | 1.58171 | 2.24775 | 0.10534 | 8 |

## Sample Plots

- `same_floor_low_query_35_detour_2.8.png`
- `same_floor_high_query_19_detour_2.2.png`

## Interpretation

Same-band routing succeeds in both tested ramp height bands. The ramp scene is passable, but it is more fragmented than `scene_20260608`: even same-band queries sometimes use portal/backbone edges as shortcuts. That is not a path-finding failure, but it means this scene is useful for checking path quality and unnecessary backbone switching before local tracking.
