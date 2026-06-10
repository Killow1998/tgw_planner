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

Run multiple start/goal pairs in the two existing pbstream scenes and verify:

```text
1. global path is found
2. local_path is smooth and remains inside the global corridor
3. sim_robot_path reaches the goal without odom stale
4. no tracking status red except real failure states
5. sim_trace_smoothness_rad_per_m stays under a documented threshold
```

If this holds, TGW can move from kinematic replay toward a real local obstacle input and then hardware-in-the-loop testing.
