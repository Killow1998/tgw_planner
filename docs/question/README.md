# TGW Review Questions

This folder keeps historical review prompts and screenshots used while
converging the current TGW design.

## Current Status

The current implementation is summarized in:

```text
../tgw_current_status_for_review.md
```

Use that document first for new review.

## Historical Documents

| Document | Status |
| --- | --- |
| `tgw_multifloor_path_planning_review.md` | Historical. It records the transition from layer-safe surface graph work to dense-trajectory backbone topology. The original surface-connector problem has been reframed. |
| `tgw_local_tracking_review.md` | Mostly current for local tracking concepts, but the freshest status is in `../tgw_current_status_for_review.md`. |
| `images/` | Historical RViz screenshots that motivated previous fixes. |

## Lessons Preserved Here

- A raw point cloud cell is not automatically a support surface.
- A virtual trajectory bridge is not a lateral expansion anchor.
- Surface graph connectivity should not own global cross-floor topology.
- Dense trajectory is the owner of experience topology.
- A global path is a topological guide; local tracking should follow a short,
  smooth, corridor-constrained route window.

