# TGW Golden Scenes

This document defines the current core-only regression scenes for TGW.

For the full current architecture and review summary, start with:

```text
../tgw_current_status_for_review.md
```

The goal is not to prove hardware readiness. The goal is to keep the global
path and local tracking contract from regressing while TGW moves toward
hardware-in-the-loop testing.

## Scenes

| Scene | Input | Role |
| --- | --- | --- |
| `scene_20260608` | `docs/data/tgw_n3map_nav_filtered.pbstream` | Stair / cross-floor experience scene. Primary test for backbone-assisted multi-level routing. |
| `scene_20260610` | `docs/data/tgw_n3map_nav_filtered_20260610.pbstream` | Ramp / grade-change scene. Primary test for smoother height transitions and same-band routing quality. |

## Regression Command

Run from the workspace root:

```bash
cd /home/user/ros_ws/to_migrate_ws
source install/setup.bash
MPLCONFIGDIR=/tmp/matplotlib python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene all \
  --sample-pairs 50 \
  --plot-limit 8
```

Run only one scene when iterating:

```bash
MPLCONFIGDIR=/tmp/matplotlib python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene 20260610 \
  --sample-pairs 50 \
  --plot-limit 8
```

Outputs:

```text
src/tgw_planner/docs/exp/scene_20260608/golden_regression/
src/tgw_planner/docs/exp/scene_20260610/golden_regression/
```

Each scene output contains:

```text
README.md
cross_floor_paths.jsonl
same_floor_low_paths.jsonl
same_floor_high_paths.jsonl
cross_floor_tracking/
same_floor_low_tracking/
same_floor_high_tracking/
```

## Pass Gate

For each scene:

```text
cross_floor global sweep: 50 / 50 success
same_floor_low global sweep: 50 / 50 success
same_floor_high global sweep: 50 / 50 success

cross_floor tracking replay: 50 / 50 pass
same_floor_low tracking replay: 50 / 50 pass
same_floor_high tracking replay: 50 / 50 pass
```

The gate is intentionally based on the dominant hybrid component. Strict
all-fragment connectivity is a map-quality diagnostic, not the default planner
pass/fail condition.

The runner also enforces configurable core path-quality limits. The current
defaults are scene-tolerant, but they are no longer passive summary numbers:

```text
max_detour: 100.0
max_same_floor_detour: 6.0
max_backbone_ratio: 0.85
max_portal_switch_count: 12
max_path_edge_dz_m: 0.85
max_tracking_final_error_m: 0.40
max_tracking_lateral_error_m: 0.90
max_tracking_z_step_m: 0.85
```

Cross-floor detour remains intentionally loose because XY straight-line
distance is a poor lower bound for stairs and ramps. Same-floor detour is
gated separately because a same-floor path should not depend on backbone
topology to hide poor local routing.

## Production Gap

These golden scenes validate:

```text
N3 pbstream -> reachable surface -> dense trajectory backbone
hybrid global path -> segment-kind path contract
core-only local tracking replay
```

They do not validate:

```text
real odom/map frame alignment
Unitree command chain
hardware command mux / e-stop
sensor latency
body attitude / foot contact dynamics
real dynamic obstacle observations
```

## Hardware TODO

These tasks are intentionally left for the robot deployment environment:

```text
1. Verify odom frame equals map frame, or add a tf2 transform path.
2. Keep /tgw_experience/cmd_vel behind a robot-side safety mux.
3. Add hardware e-stop / deadman supervision outside TGW.
4. Run low-speed armed tracking with no dynamic obstacles.
5. Enable local obstacle stop-only mode after odom and command chain are stable.
6. Record real robot trace and compare against the core replay trace.
```

TGW should not directly own the robot's final hardware safety layer. It should
publish a conservative command candidate with explicit status, and the robot
deployment stack should decide whether to forward it.

## Current Tracking Boundary

The ROS node keeps simulation/replay and real command output separate:

```text
enable_kinematic_replay=true
  publishes fake odom and visualization only
  does not publish cmd_vel when enable_path_tracking=false

enable_path_tracking=true
  may publish /tgw_experience/cmd_vel
  requires arm_tracking when require_tracking_arm=true
  stops on stale odom, frame mismatch, route projection failure, smoothing
  failure, collision arc block, or goal reached
```

Local obstacle avoidance is still a 2D inflated stop layer after height
filtering against the nearest local path point. It is useful for conservative
stop-only replay and early robot tests, but it is not a full 3D body collision
checker.

## Current Benchmark Snapshot

Run from the workspace root:

```bash
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered.pbstream
build/tgw_planner/tgw_experience_benchmark \
  src/tgw_planner/docs/data/tgw_n3map_nav_filtered_20260610.pbstream
```

Recent local CPU measurements:

| Scene | Preprocess | First query | Peak RSS | Primary hotspot |
| --- | ---: | ---: | ---: | --- |
| `scene_20260608` | about 7.9s | 0.27ms | 725 MB | `surface_build` |
| `scene_20260610` | about 13.8s | 14.5ms | 875 MB | `surface_build`, then `surface_graph_build` |

This means the global planner is already fast enough for interactive use after
map load. Startup preprocessing, especially surface expansion / graph build, is
the next performance target if the goal is consistent sub-10s readiness.
