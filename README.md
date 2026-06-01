# tgw_planner

`tgw_planner` (`tgw_planner`) is a ROS 2 Humble MVP for Unitree Go2-W multi-floor navigation from a static PCD map.

The package keeps the planner core ROS-free under `include/tgw_planner/core` and `src/core`. The Humble wrapper nodes live in `src/humble`.

## Build

```bash
cd $ROS_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select tgw_planner --symlink-install
```

## Run

```bash
source $ROS_WS/install/setup.bash
ros2 launch tgw_planner pcd_to_path_mvp.launch.py \
  pcd_file:=/absolute/path/to/global_map.pcd \
  robot_radius_m:=0.35 \
  robot_height_m:=0.80 \
  map_resolution_m:=0.20
```

To save a lightweight map package during startup, pass `save_map_dir:=/path/to/map_package`.
The package contains `map.bt`, `metadata.yaml`, `blocked_regions.yaml`, and `README.generated.txt`.

Use RViz2 Publish Point with:

```bash
ros2 service call /rviz_click_router/set_mode tgw_planner/srv/SetClickMode "{mode: start}"
ros2 service call /rviz_click_router/set_mode tgw_planner/srv/SetClickMode "{mode: goal}"
```

RViz includes `Tgw 3D Start` and `Tgw 3D Goal` tools. Select one of those tools,
use the mouse wheel to set the z height, then left-click and drag like the normal
2D pose tools to publish `/start_pose` or `/goal_pose`. The active height is shown
in the RViz status bar and in the tool property named `Z Height`. After publishing,
the planner snaps the pose to the nearest traversable map layer near that XY
location, so the final start/goal marker should sit on the map instead of floating.

Blocked region API:

```bash
ros2 service call /nav_map/set_blocked_region tgw_planner/srv/SetBlockedRegion "{
  operation: 'add',
  min: {x: 1.0, y: 2.0, z: 0.0},
  max: {x: 2.0, y: 3.0, z: 1.0},
  reason: 'manual_forbidden'
}"
```
