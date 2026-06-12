# TGW Planner Docs

This directory is the durable review and regression record for the current
TGW pbstream experience planner.

## Start Here

- [tgw_current_status_for_review.md](tgw_current_status_for_review.md)
  is the current high-signal review document. Send this to another reviewer
  first.
- [exp/golden_scenes.md](exp/golden_scenes.md) defines the current golden
  regression scenes and commands.
- [question/README.md](question/README.md) indexes historical GPT Pro review
  prompts and marks which ones are still relevant.

## Directory Map

| Path | Purpose |
| --- | --- |
| `data/` | Golden pbstream and debug data used by core regression. Large map files are intentionally local artifacts. |
| `exp/` | Reproducible experiment summaries, selected plots, and golden regression outputs. |
| `question/` | Historical review prompts, screenshots, and issue writeups used to drive design decisions. |

## Current Golden Inputs

| Scene | File | Role |
| --- | --- | --- |
| `scene_20260608` | `data/tgw_n3map_nav_filtered.pbstream` | Stair / cross-floor experience scene. |
| `scene_20260610` | `data/tgw_n3map_nav_filtered_20260610.pbstream` | Ramp / grade-change scene. |

## What Is Current

The active design is:

```text
N3Mapping pbstream
  -> shared ExperienceGeometryIndex
  -> dense trajectory support projection
  -> experience reachable surface
  -> layer-safe local surface graph
  -> dense trajectory backbone graph
  -> unified hybrid graph
  -> global path with Surface / Backbone / Portal segment kinds
  -> local route window + smoother + regulated pure pursuit
```

This is no longer a PCD-only planner, realtime raycast mapper, or stair
semantic planner.

## What Is Historical

Older documents under `question/` describe failures that have already been
resolved or reframed:

- raw keyframe cloud points were incorrectly treated as support;
- bridge cells were incorrectly allowed to anchor lateral expansion;
- surface graph was incorrectly asked to own global cross-floor topology;
- portal-pair routing was too greedy and was replaced by unified hybrid graph
  search.

Keep those documents as design evidence, but do not treat them as the current
implementation contract.

