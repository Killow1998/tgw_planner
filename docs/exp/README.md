# TGW Experiments

This directory contains reusable experiment records for the TGW pbstream
experience planner.

## Current Regression Entry Point

Use [golden_scenes.md](golden_scenes.md) for the canonical commands.

Current golden scenes:

- `scene_20260608`: stair / cross-floor experience scene.
- `scene_20260610`: ramp / grade-change scene.

Each scene may contain:

```text
hybrid_path_quality_samples.md
same_floor_core_sweep.md
golden_regression/
tracking_replay/
regression_50/
```

`golden_regression/` is the current preferred output layout. Older
`tracking_replay/` and `regression_50/` folders are retained as historical
evidence when they explain a design decision.

## Reading The Plots

Hybrid path plots use this legend:

```text
gray  = full dense-trajectory backbone
black = selected global path
green = surface path points
blue  = selected backbone path points
red   = portal points
```

For cross-floor queries, XY detour can look large because horizontal distance
is a bad lower bound when the robot must traverse stairs or ramps. Prefer these
signals:

```text
success rate
max_path_edge_dz_m
backbone_ratio
portal_switch_count
same-floor detour
tracking replay final/lateral error
```

