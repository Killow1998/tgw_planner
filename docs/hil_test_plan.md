# TGW HIL / Supervised Robot Validation Plan

Purpose:

```text
Move TGW from offline golden regression and RViz kinematic simulation into
hardware-in-the-loop validation without treating it as a production safety
system yet.
```

This plan deliberately keeps command output behind a robot-side safety layer.
The first real-robot work is dry-run validation, not autonomous deployment.

## Preconditions

Before any robot-side run:

```text
current branch built on the robot / edge computer
golden regression passed on the same code
edge benchmark recorded for the target pbstream
tracking command topic remains /tgw_experience/cmd_vel
/tgw_experience/cmd_vel is not forwarded to the robot command topic yet
operator e-stop and deadman are ready
robot-side mux behavior is understood
odom frame contract is verified
```

TGW must fail closed for:

```text
tracking not armed
odom stale
odom frame mismatch
route projection failure
local obstacle stop condition
lost operator stop authority
```

## Stage 0: Offline Gate

Run from the workspace root:

```bash
colcon build --packages-select tgw_planner
colcon test --packages-select tgw_planner --event-handlers console_direct+
colcon test-result --verbose

MPLCONFIGDIR=/tmp/matplotlib \
python3 src/tgw_planner/scripts/run_tgw_golden_regression.py \
  --scene all \
  --sample-pairs 50 \
  --plot-limit 0
```

Pass condition:

```text
unit/core tests pass
scene_20260608 global sweeps pass 50 / 50
scene_20260610 global sweeps pass 50 / 50
tracking replay passes 50 / 50 for cross_floor, same_floor_low, same_floor_high
```

## Stage 1: Robot-Side Dry Run

Goal:

```text
Verify real odom, frame contract, plan generation, tracking status, local path,
and candidate cmd_vel without moving the robot from TGW commands.
```

Launch:

```bash
ros2 launch tgw_planner experience_planner_real.launch.py \
  pbstream_path:=/absolute/path/to/n3map.pbstream \
  tracking_cmd_vel_topic:=/tgw_experience/cmd_vel
```

Rules:

```text
do not forward /tgw_experience/cmd_vel to /cmd_vel
do not arm real robot command mux
set start / goal in RViz
inspect plan path, local path, status text, and cmd_vel candidate
record logs and topic snapshots
```

Verify:

```text
odom frame equals the pbstream map frame, or tracking rejects it
plan_path succeeds for known reachable goals
tracking_status_json transitions through ready / active / goal reached clearly
cmd_vel is zero while unarmed
candidate nonzero cmd_vel is smooth and bounded when armed in TGW only
route_projection_failed stops instead of continuing
odom_stale stops instead of continuing
frame_mismatch stops instead of continuing
```

Abort if:

```text
odom frame is unknown or inconsistent
nonzero cmd_vel appears on the real robot command topic
tracking status is ambiguous
local path oscillates wildly
operator cannot stop command forwarding
```

## Stage 2: Low-Speed Single-Floor Armed Route

Goal:

```text
Validate the command chain at low speed in an open, single-floor area.
```

Required setup:

```text
robot-side mux ready
e-stop ready
deadman ready
velocity cap set conservatively
no dynamic obstacles
operator beside robot
```

Recommended first route:

```text
short single-floor path
wide clearance
no stairs / ramps
no portal segment required
```

Pass condition:

```text
robot reaches goal under supervision
tracking_status_json reports goal reached
max lateral error is acceptable for low-speed test
no stale odom or frame mismatch occurs
operator can stop immediately
```

## Stage 3: Low-Speed Cross-Floor Route

Goal:

```text
Validate backbone / portal / surface route execution on the robot at low speed.
```

Required setup:

```text
the same pbstream already passed offline regression
backbone and portal visualization are enabled
single-floor low-speed route already succeeded
stairs / ramp area is supervised and clear
```

Record:

```text
planned global path
local path
tracking_status_json
/tgw_experience/cmd_vel
odom trace
robot-side executed command topic
operator intervention events
```

Compare:

```text
real trace vs RViz kinematic replay
surface / backbone / portal segment transitions
max lateral error near portals
whether route projection remains stable on stairs / ramps
```

## Stage 4: Local Obstacle Stop-Only Check

Goal:

```text
Validate the local obstacle layer as a stop layer, not autonomous obstacle
avoidance.
```

Test:

```text
place a controlled obstacle on the local path
confirm TGW stops before collision
remove obstacle
confirm tracking can resume only under operator-approved conditions
```

Do not test:

```text
dynamic obstacle dodging
unverified local rerouting
high-speed obstacle encounters
```

## Required Logs

Save for every HIL attempt:

```text
git commit / tag
device model and power mode
pbstream path
launch command and parameter overrides
tracking_status_json
planned global path
local path
/tgw_experience/cmd_vel
robot-side forwarded cmd_vel, if any
odom topic and frame_id
local obstacle cloud, if used
operator notes
abort reason, if any
```

## Cache Decision

Do not add compiled experience cache before the first edge and dry-run evidence.

Use cache if:

```text
edge preprocess is acceptable but production startup requires near-instant map load
maps are reused frequently
operator workflow cannot tolerate first-load time
```

Do not use cache to hide:

```text
bad first-build algorithm
excessive RSS
broken edge regression
unsafe tracking status
```

## What Not To Do Yet

```text
do not run high-speed autonomous tests
do not bypass robot-side mux / e-stop / deadman
do not publish directly to /cmd_vel from TGW as the first robot test
do not test in public or dynamic environments
do not introduce PCD fallback, realtime raycast, StairFlight, or 3D voxel A*
do not resume SupportCandidateStore side-index performance work
```
