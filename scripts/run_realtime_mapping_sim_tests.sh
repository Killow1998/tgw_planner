#!/usr/bin/env bash
set -eo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

launch_pid=""
cleanup()
{
  if [[ -n "${launch_pid}" ]]; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    launch_pid=""
  fi
}
trap cleanup EXIT

existing_nodes="$(ros2 node list 2>/dev/null | grep -E '^/tgw_realtime_mapping_node$' || true)"
if [[ -n "${existing_nodes}" ]]; then
  echo "Refusing to run while realtime mapping nodes already exist:"
  echo "${existing_nodes}"
  exit 1
fi

wait_for_node()
{
  for _ in $(seq 1 80); do
    if ros2 node list 2>/dev/null | grep -qx "/tgw_realtime_mapping_node"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

value_from_snapshot()
{
  local file="$1"
  local key="$2"
  grep -o "${key}=[0-9]*" "${file}" | tail -n 1 | cut -d= -f2
}

publish_floor_ceiling_scene()
{
  python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Publisher(Node):
    def __init__(self):
        super().__init__("tgw_realtime_sim_pub")
        self.pose_pub = self.create_publisher(PoseStamped, "/tgw_sim/pose", 10)
        self.cloud_pub = self.create_publisher(point_cloud2.PointCloud2, "/tgw_sim/points", 10)


rclpy.init()
node = Publisher()
points = []
for ix in range(8):
    for iy in range(-3, 4):
        x = 0.5 + 0.15 * ix
        y = 0.15 * iy
        points.append((x, y, -1.0))  # floor at map z=0 from a 1m-high sensor
        points.append((x, y, 1.0))   # ceiling at map z=2

for _ in range(5):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.z = 1.0
    pose.pose.orientation.w = 1.0
    node.pose_pub.publish(pose)

    header = Header()
    header.frame_id = "sensor"
    header.stamp = node.get_clock().now().to_msg()
    cloud = point_cloud2.create_cloud_xyz32(header, points)
    node.cloud_pub.publish(cloud)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.15)

node.destroy_node()
rclpy.shutdown()
PY
}

publish_dynamic_disappears_scene()
{
  python3 - <<'PY'
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Publisher(Node):
    def __init__(self):
        super().__init__("tgw_dynamic_sim_pub")
        self.pose_pub = self.create_publisher(PoseStamped, "/tgw_sim/pose", 10)
        self.cloud_pub = self.create_publisher(point_cloud2.PointCloud2, "/tgw_sim/points", 10)


def publish_frame(node, points):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.orientation.w = 1.0
    node.pose_pub.publish(pose)

    header = Header()
    header.frame_id = "sensor"
    header.stamp = node.get_clock().now().to_msg()
    cloud = point_cloud2.create_cloud_xyz32(header, points)
    node.cloud_pub.publish(cloud)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.12)


rclpy.init()
node = Publisher()
publish_frame(node, [(1.0, 0.0, 0.0)])
for _ in range(5):
    publish_frame(node, [(2.0, 0.0, 0.0)])

node.destroy_node()
rclpy.shutdown()
PY
}

run_floor_ceiling_case()
{
  cleanup
  local log_file="/tmp/tgw_realtime_floor_ceiling_launch.log"
  local snapshot_file="/tmp/tgw_realtime_floor_ceiling_snapshot.out"
  ros2 launch tgw_planner realtime_mapping.launch.py \
    use_tf:=false \
    assume_cloud_in_map_frame:=false \
    points_topic:=/tgw_sim/points \
    pose_topic:=/tgw_sim/pose \
    publish_period_ms:=300 \
    min_static_hits:=1 \
    min_distinct_views:=1 \
    min_static_lifetime_sec:=0.0 \
    enable_dynamic_filter:=false \
    max_range_m:=10.0 \
    min_range_m:=0.01 \
    surface_require_static_support:=false \
    surface_require_observed_free_space:=true \
    planner_require_footprint:=false \
    validation_require_footprint:=false >"${log_file}" 2>&1 &
  launch_pid="$!"
  wait_for_node
  publish_floor_ceiling_scene
  sleep 1.0
  ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}" \
    >"${snapshot_file}" 2>&1

  local success surface traversable forbidden
  success="$(grep -o "success=True\\|success=False" "${snapshot_file}" | tail -n 1 || true)"
  surface="$(value_from_snapshot "${snapshot_file}" "surface_points")"
  traversable="$(value_from_snapshot "${snapshot_file}" "traversable_points")"
  forbidden="$(value_from_snapshot "${snapshot_file}" "forbidden_points")"
  echo "floor_ceiling_free_space: ${success:-success=unknown} surface_points=${surface:-0} traversable_points=${traversable:-0} forbidden_points=${forbidden:-0}"
  if [[ "${success}" != "success=True" ]]; then
    echo "FAIL floor_ceiling_free_space: snapshot failed"
    return 1
  fi
  if (( ${surface:-0} == 0 || ${traversable:-0} == 0 || ${forbidden:-0} == 0 )); then
    echo "FAIL floor_ceiling_free_space: expected surface, traversable, and forbidden points"
    return 1
  fi
}

run_dynamic_disappears_case()
{
  cleanup
  local log_file="/tmp/tgw_realtime_dynamic_disappears_launch.log"
  local snapshot_file="/tmp/tgw_realtime_dynamic_disappears_snapshot.out"
  ros2 launch tgw_planner realtime_mapping.launch.py \
    use_tf:=false \
    assume_cloud_in_map_frame:=false \
    points_topic:=/tgw_sim/points \
    pose_topic:=/tgw_sim/pose \
    publish_period_ms:=300 \
    min_static_hits:=3 \
    min_distinct_views:=2 \
    min_static_lifetime_sec:=1.0 \
    enable_dynamic_filter:=true \
    dynamic_clear_ratio_threshold:=0.50 \
    enable_self_filter:=false \
    max_range_m:=10.0 \
    min_range_m:=0.01 \
    surface_require_static_support:=true \
    surface_require_observed_free_space:=true \
    planner_require_footprint:=false \
    validation_require_footprint:=false >"${log_file}" 2>&1 &
  launch_pid="$!"
  wait_for_node
  publish_dynamic_disappears_scene
  sleep 1.0
  ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}" \
    >"${snapshot_file}" 2>&1

  local success dynamic static
  success="$(grep -o "success=True\\|success=False" "${snapshot_file}" | tail -n 1 || true)"
  dynamic="$(value_from_snapshot "${snapshot_file}" "dynamic_points")"
  static="$(value_from_snapshot "${snapshot_file}" "static_points")"
  echo "dynamic_disappears: ${success:-success=unknown} dynamic_points=${dynamic:-0} static_points=${static:-0}"
  if [[ "${success}" != "success=True" ]]; then
    echo "FAIL dynamic_disappears: snapshot failed"
    return 1
  fi
  if (( ${dynamic:-0} == 0 )); then
    echo "FAIL dynamic_disappears: expected dynamic suspect points after clearing a transient obstacle"
    return 1
  fi
}

run_floor_ceiling_case
run_dynamic_disappears_case
