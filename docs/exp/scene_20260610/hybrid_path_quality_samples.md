# TGW Hybrid Path Quality Samples - Scene 20260610

Source pbstream: `docs/data/tgw_n3map_nav_filtered_20260610.pbstream` (ignored by git).

Sweep command: `tgw_experience_global_sweep docs/data/tgw_n3map_nav_filtered_20260610.pbstream --export-jsonl /tmp/tgw_sweep_paths_20260610_auto.jsonl`.

Legend: gray line = full dense-trajectory backbone, black line = selected path, green points = surface path, blue points = selected backbone path, red points = portal points, green marker = start, red marker = goal.

Summary: auto bands low_z_max=2.54753 high_z_min=3.27478, sampled_queries=50, sampled_success=50, max_detour_ratio=11.8478, mean_detour_ratio=4.75625, suspicious_detour_count=37, max_portal_switch_count=8.

| Query | Detour | XY distance m | Path length m | Backbone length m | Portal switches | Image |
|---:|---:|---:|---:|---:|---:|---|
| 21 | 11.85 | 15.06 | 178.45 | 41.91 | 8 | [hybrid_query_21_detour_11.8.png](hybrid_query_21_detour_11.8.png) |
| 22 | 10.61 | 17.28 | 183.28 | 41.91 | 8 | [hybrid_query_22_detour_10.6.png](hybrid_query_22_detour_10.6.png) |
| 39 | 9.67 | 17.18 | 166.18 | 41.91 | 8 | [hybrid_query_39_detour_9.7.png](hybrid_query_39_detour_9.7.png) |
| 29 | 9.33 | 17.97 | 167.73 | 41.91 | 8 | [hybrid_query_29_detour_9.3.png](hybrid_query_29_detour_9.3.png) |
| 16 | 9.30 | 20.61 | 191.73 | 41.91 | 8 | [hybrid_query_16_detour_9.3.png](hybrid_query_16_detour_9.3.png) |
| 26 | 9.06 | 19.55 | 177.06 | 41.91 | 8 | [hybrid_query_26_detour_9.1.png](hybrid_query_26_detour_9.1.png) |
| 23 | 8.77 | 20.04 | 175.68 | 41.91 | 8 | [hybrid_query_23_detour_8.8.png](hybrid_query_23_detour_8.8.png) |
| 17 | 8.25 | 22.48 | 185.49 | 41.91 | 8 | [hybrid_query_17_detour_8.3.png](hybrid_query_17_detour_8.3.png) |
