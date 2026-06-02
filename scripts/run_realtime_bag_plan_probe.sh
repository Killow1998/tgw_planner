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

bag_path="${1:-/home/user/ros_ws/bagfile/f7tof9_g2w_ros2}"
play_seconds="${TGW_BAG_PLAY_SECONDS:-75}"
log_dir="${TGW_BAG_PROBE_LOG_DIR:-/tmp/tgw_real_bag_plan_probe}"
mkdir -p "${log_dir}"

pids=""
cleanup()
{
  for pid in ${pids}; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in ${pids}; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT

ros2 launch fast_lio mapping_mid360.launch.py >"${log_dir}/fast_lio.log" 2>&1 &
pids="${pids} $!"
sleep 3
ros2 launch n3mapping mapping.launch.py >"${log_dir}/n3mapping.log" 2>&1 &
pids="${pids} $!"
sleep 3
ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/n3mapping/cloud_world \
  use_tf:=false \
  assume_cloud_in_map_frame:=true \
  publish_period_ms:=1000 \
  planner_require_footprint:=false \
  validation_require_footprint:=false >"${log_dir}/tgw.log" 2>&1 &
pids="${pids} $!"
sleep 5

timeout "${play_seconds}s" ros2 bag play "${bag_path}" >"${log_dir}/bag.log" 2>&1 || true
sleep 3

python3 - <<'PY'
import math
import sys
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tgw_planner.srv import PlanPath


class Probe(Node):
    def __init__(self):
        super().__init__("tgw_real_bag_plan_probe")
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cloud = None
        self.create_subscription(PointCloud2, "/tgw_map/traversable_cloud", self.on_cloud, qos)
        self.cli = self.create_client(PlanPath, "/tgw_map/plan_path")

    def on_cloud(self, msg):
        self.cloud = msg


def spin_until(node, predicate, timeout):
    deadline = node.get_clock().now() + Duration(seconds=timeout)
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    node = Probe()
    if not spin_until(node, lambda: node.cloud is not None, 8.0):
        print("success=False reason=no_traversable_cloud")
        return 1

    points = [
        (float(p[0]), float(p[1]), float(p[2]))
        for p in point_cloud2.read_points(
            node.cloud, field_names=("x", "y", "z"), skip_nans=True)
    ]
    print(f"traversable_points={len(points)}")
    if len(points) < 2:
        print("success=False reason=not_enough_points")
        return 1

    resolution = 0.10
    cells = {}
    for p in points:
        cell = tuple(int(math.floor(v / resolution)) for v in p)
        cells[cell] = p
    unvisited = set(cells)
    components = []
    while unvisited:
        seed = unvisited.pop()
        queue = deque([seed])
        component = [seed]
        while queue:
            current = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-3, -2, -1, 0, 1, 2, 3):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                        if neighbor not in unvisited:
                            continue
                        unvisited.remove(neighbor)
                        queue.append(neighbor)
                        component.append(neighbor)
        components.append(component)
    components.sort(key=len, reverse=True)
    print(f"surface_component_count={len(components)} largest_component_size={len(components[0]) if components else 0}")
    largest = components[0] if components else []
    sampled = largest[::max(1, len(largest) // 2000)]
    chosen = None
    for a_cell in sampled:
        a = cells[a_cell]
        for b_cell in sampled:
            b = cells[b_cell]
            dz = abs(a[2] - b[2])
            dxy = math.hypot(a[0] - b[0], a[1] - b[1])
            if dz <= 0.05 and 1.0 <= dxy <= 2.0:
                chosen = (a, b, dxy)
                break
        if chosen is not None:
            break
    if chosen is None and len(largest) >= 2:
        a = cells[largest[0]]
        b = min(
            (cells[cell] for cell in largest[1:]),
            key=lambda p: abs(p[2] - a[2]) + abs(math.hypot(p[0] - a[0], p[1] - a[1]) - 1.0))
        chosen = (a, b, math.hypot(a[0] - b[0], a[1] - b[1]))
    if chosen is None:
        print("success=False reason=no_component_pair")
        return 1

    start, goal, dxy = chosen
    print(f"start={start} goal={goal} dxy={dxy:.3f}")
    if not node.cli.wait_for_service(timeout_sec=5.0):
        print("success=False reason=no_plan_service")
        return 1

    req = PlanPath.Request()
    req.start.header.frame_id = "map"
    req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z = start
    req.start.pose.orientation.w = 1.0
    req.goal.header.frame_id = "map"
    req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z = goal
    req.goal.pose.orientation.w = 1.0
    future = node.cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done() or future.result() is None:
        print("success=False reason=plan_timeout")
        return 1

    response = future.result()
    stats = response.stats
    print(
        f"success={response.success} message=\"{response.message}\" "
        f"final_path_validated={stats.final_path_validated} "
        f"final_path_fallback_to_raw={stats.final_path_fallback_to_raw} "
        f"expanded_nodes={stats.expanded_nodes} path_waypoints={stats.path_waypoints} "
        f"path_length_m={stats.path_length_m:.3f} "
        f"min_path_clearance_m={stats.min_path_clearance_m:.3f} "
        f"mean_path_clearance_m={stats.mean_path_clearance_m:.3f} "
        f"clearance_cost_sum={stats.clearance_cost_sum:.3f}")
    return 0 if response.success and stats.final_path_validated and stats.path_waypoints > 0 else 1


sys.exit(main())
PY

ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}" \
  >"${log_dir}/snapshot.out" 2>&1 || true
grep -o \
  "received_clouds=[0-9]*\\|integrated_clouds=[0-9]*\\|traversable_points=[0-9]*\\|surface_points=[0-9]*\\|dynamic_points=[0-9]*" \
  "${log_dir}/snapshot.out" | tr "\n" " " || true
echo
