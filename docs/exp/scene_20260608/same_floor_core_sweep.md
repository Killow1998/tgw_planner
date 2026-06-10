# Same-Floor Core Sweep - scene_20260608

Date: 2026-06-10

Input: `docs/data/tgw_n3map_nav_filtered.pbstream`

Purpose: verify that the hybrid backbone planner still handles same-floor/local routing after cross-floor routing was introduced.

## Commands

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

build/tgw_planner/tgw_experience_global_sweep \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream \
  --mode same-band --same-z-min -0.20 --same-z-max 0.20 \
  --export-jsonl /tmp/tgw_old_same_low.jsonl

build/tgw_planner/tgw_experience_global_sweep \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream \
  --mode same-band --same-z-min 7.80 --same-z-max 8.50 \
  --export-jsonl /tmp/tgw_old_same_high.jsonl
```

## Results

| Band | Queries | Success | Mean detour | Max detour | Mean backbone ratio | Max portal switches |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Low floor, z [-0.20, 0.20] | 50 | 50 | 1.26466 | 1.63575 | 0.00247 | 2 |
| High floor, z [7.80, 8.50] | 50 | 50 | 1.51950 | 2.72907 | 0.00000 | 0 |

## Sample Plots

- `same_floor_low_query_25_detour_1.6.png`
- `same_floor_high_query_12_detour_2.7.png`

## Interpretation

Same-floor routing still works on this scene. The high-floor band does not use the backbone at all; the low-floor band only uses it minimally. This suggests the cross-floor hybrid graph did not break ordinary same-floor planning in this map.
