# TGW Local Tracking Review

## Current Thesis

TGW should not trace every point of the global path.

The current target model is:

```text
global hybrid path
  -> route progress projection
  -> local route window / ahead point
  -> local smooth trajectory
  -> regulated pure pursuit tracking
```

The global path is only the topological guide. The local tracker should track a short, smooth, corridor-constrained local trajectory.

## Current Architecture

Core code:

```text
RouteProgressTracker
  - projects current odom pose onto the global path
  - keeps progress monotonic inside a forward projection window
  - extracts a local route window
  - provides projected point, ahead point, remaining distance

LocalPathSmoother
  - builds a short Bezier local trajectory from current pose to the selected local target
  - computes local path smoothness and max turn angle
  - rejects overly curved local paths
  - rejects local paths outside the global-path corridor

RegulatedPurePursuitController
  - tracks the local smooth trajectory
  - applies speed cap, curvature regulation, acceleration limit, goal slowdown
  - checks the rolling local obstacle map collision arc

RollingLocalMap
  - stores recent local obstacle points inside a time window
  - supports path and arc collision checks
```

ROS side is intended to remain a thin wrapper:

```text
tgw_experience_planner_node
  - loads parameters
  - publishes local path, fake odom, sim robot path, markers, status JSON
  - subscribes odom and optional local obstacle cloud
```

## Recent Fixes

### 1. Snap failure text

Previously, snap-too-far failures appeared as:

```text
start_not_on_reachable_surface
```

Now they explicitly report:

```text
start_snap_distance_too_far 1.89m > 1.50m
goal_snap_distance_too_far ...
```

This avoids confusing a bad click with a planner failure.

### 2. Simulated robot trajectory recording

The simulator now publishes:

```text
/tgw_experience/sim_robot_path
```

The status JSON also reports:

```json
{
  "sim_trace_points": 0,
  "sim_trace_length_m": 0.0,
  "sim_trace_smoothness_rad_per_m": 0.0,
  "sim_trace_max_turn_angle_rad": 0.0
}
```

This lets us evaluate the actual replayed trajectory, not only the planned global path.

### 3. Local path smoothness metrics

Local path status now includes:

```json
{
  "local_path_length_m": 0.0,
  "local_path_smoothness_rad_per_m": 0.0,
  "local_path_max_turn_angle_rad": 0.0
}
```

Current default thresholds:

```yaml
local_path_max_smoothness_rad_per_m: 2.50
local_path_max_turn_angle_rad: 1.20
```

### 4. Prevent premature corner cutting

Observed problem:

```text
The local smoothed path could cut across the inside of a global-path corner.
This can cross unknown/empty cells or edge regions even if the global path itself is valid.
```

Fix:

```yaml
local_path_max_route_deviation_m: 0.45
local_path_corner_cut_turn_angle_rad: 0.75
local_path_min_corner_target_distance_m: 0.60
```

Behavior:

```text
1. If the local route window contains a sharp corner, the local target is clipped to that corner instead of jumping past it.
2. After Bezier smoothing, every point must remain inside a corridor around the global local route.
3. If the path leaves the corridor, tracking rejects it with:

   local_path_outside_global_corridor
```

Regression test:

```text
testLocalPathSmootherDoesNotCutAcrossCorner
```

### 5. Tracking status marker

RViz now has:

```text
/tgw_experience/tracking_status_marker
```

Status colors:

```text
gray  = idle / no path
yellow = path ready but not armed
green = tracking or goal reached
red   = real tracking failure
```

Previously, `tracking_path_not_set` showed as red, and simulated goal arrival could flash green and then become red `tracking_odom_stale`.

Fix:

```text
1. no path is TRACK IDLE, not a failure
2. on simulated goal reached, fake odom continues publishing zero velocity
3. goal reached remains green instead of becoming stale
```

### 6. Safety gates before real command output

Implemented after the 2026-06-11 review:

```text
RouteProgressTracker:
  - projection now has an explicit found flag
  - poses outside the allowed lateral projection error fail with:
    route_projection_failed
  - tracking stops instead of treating the lateral error as zero

RegulatedPurePursuitController:
  - goal reached uses goal_tolerance_m
  - min_linear_speed_mps is no longer used as a distance threshold

ROS wrapper:
  - real cmd_vel output is gated by /tgw_experience/set_tracking_armed
  - kinematic replay no longer publishes command twists; it only updates fake odom / replay status
  - require_tracking_arm is true in real config and false in sim config
  - arm requests are rejected unless a path exists and odom is current in the expected frame
  - odom frame must match the map frame by default
  - mismatched odom frame fails with:
    tracking_odom_frame_mismatch
  - odom frame mismatch clears local tracking state, publishes zero velocity,
    and disarms tracking; a later valid odom frame still requires re-arming
  - local obstacle height filtering uses nearest local path height, not one cached surface z
```

Relevant parameters:

```yaml
require_tracking_arm: true
tracking_odom_must_be_in_map_frame: true
tracking_max_projection_lateral_error_m: 2.0
```

Arm command for real tracking:

```bash
ros2 service call /tgw_experience/set_tracking_armed std_srvs/srv/SetBool "{data: true}"
```

Disarm:

```bash
ros2 service call /tgw_experience/set_tracking_armed std_srvs/srv/SetBool "{data: false}"
```

## Current Known Boundary

This is still a kinematic replay, not physics simulation.

It validates:

```text
path tracking state machine
local trajectory generation
RPP command behavior
trajectory smoothness metrics
basic local obstacle collision hooks
```

It does not validate:

```text
foot contact dynamics
Unitree low-level controller response
slippage
body attitude constraints
real sensor latency
real local obstacle updates
```

## Review Questions

Please review whether the local tracking model is now conceptually clean:

```text
global path -> progress/ahead point -> smooth local path -> RPP
```

Specific questions:

1. Is corridor-constrained Bezier local smoothing a reasonable MVP before introducing a heavier local planner?
2. Are the smoothness metrics sufficient for early regression testing?
3. Should the local smoother use a different primitive, such as clothoid / cubic Hermite / timed elastic band, before real-robot tests?
4. Should the RPP speed target ignore global path segment kind entirely and only use speed caps, curvature, acceleration, and local obstacle limits?
5. What additional failure states should be visible in RViz before moving to hardware?

## Suggested Next Proof Point

The next core-only proof point is now captured by:

```text
docs/exp/golden_scenes.md
scripts/run_tgw_golden_regression.py
```

The regression runner supports scene-scoped runs:

```bash
python3 src/tgw_planner/scripts/run_tgw_golden_regression.py --scene 20260610
python3 src/tgw_planner/scripts/run_tgw_golden_regression.py --scene all
```

The regression runner also exposes path-quality gates:

```text
--max-detour
--max-same-floor-detour
--max-backbone-ratio
--max-portal-switch-count
--max-path-edge-dz-m
--max-tracking-final-error-m
--max-tracking-lateral-error-m
--max-tracking-z-step-m
```

Run multiple start/goal pairs in the two existing pbstream scenes and verify:

```text
1. global path is found
2. local_path is smooth and remains inside the global corridor
3. sim_robot_path reaches the goal without odom stale
4. no tracking status red except real failure states
5. sim_trace_smoothness_rad_per_m stays under a documented threshold
```

If this holds, TGW can move from kinematic replay toward a real local obstacle input and then hardware-in-the-loop testing.

## Hardware TODO

The remaining production-facing tasks need to happen in the robot deployment
environment rather than this development workstation:

```text
1. Verify odom/map frame contract against the robot localization stack.
2. Keep TGW command output behind a robot-side mux / safety supervisor.
3. Add e-stop / deadman before forwarding to /cmd_vel.
4. Run low-speed supervised tracking on one known route.
5. Enable local obstacle stop-only mode only after odom and command forwarding are stable.
```

## Cleanup Decision

The old `GlobalPathTracker` is no longer part of the default core build or smoke
test path, and its header/source have been removed. The current mainline tracker
contract is:

```text
RouteProgressTracker
  -> LocalPathSmoother
  -> RegulatedPurePursuitController
```

Do not reintroduce a second tracker lifecycle unless there is a concrete
behavioral gap that cannot be handled by the current progress/smoothing/RPP
pipeline.
