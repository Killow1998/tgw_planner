#!/usr/bin/env bash
set -eo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_setup="${script_dir}/../../../install/setup.bash"
if [[ -f "${workspace_setup}" ]]; then
  # shellcheck disable=SC1090
  source "${workspace_setup}"
elif [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
set -u

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=$((20 + RANDOM % 180))
fi

bag_path="${1:-/home/user/ros_ws/bagfile/f7tof9_g2w_ros2}"
play_seconds="${TGW_BAG_PLAY_SECONDS:-75}"
log_dir="${TGW_BAG_PROBE_LOG_DIR:-/tmp/tgw_real_bag_plan_probe}"
surface_require_observed_free_space="${TGW_SURFACE_REQUIRE_OBSERVED_FREE_SPACE:-true}"
surface_allow_observed_free_bridge="${TGW_SURFACE_ALLOW_OBSERVED_FREE_BRIDGE:-true}"
surface_require_static_support="${TGW_SURFACE_REQUIRE_STATIC_SUPPORT:-false}"
enable_dynamic_filter="${TGW_ENABLE_DYNAMIC_FILTER:-true}"
min_static_hits="${TGW_MIN_STATIC_HITS:-3}"
min_distinct_views="${TGW_MIN_DISTINCT_VIEWS:-2}"
min_static_lifetime_sec="${TGW_MIN_STATIC_LIFETIME_SEC:-1.0}"
planner_enable_shortcut="${TGW_PLANNER_ENABLE_SHORTCUT:-true}"
mkdir -p "${log_dir}"
rm -f \
  "${log_dir}/domain.log" \
  "${log_dir}/launch_options.log" \
  "${log_dir}/fast_lio.log" \
  "${log_dir}/n3mapping.log" \
  "${log_dir}/tgw.log" \
  "${log_dir}/bag.log" \
  "${log_dir}/probe.out" \
  "${log_dir}/snapshot.out"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >"${log_dir}/domain.log"
{
  echo "TGW_SURFACE_REQUIRE_OBSERVED_FREE_SPACE=${surface_require_observed_free_space}"
  echo "TGW_SURFACE_ALLOW_OBSERVED_FREE_BRIDGE=${surface_allow_observed_free_bridge}"
  echo "TGW_SURFACE_REQUIRE_STATIC_SUPPORT=${surface_require_static_support}"
  echo "TGW_ENABLE_DYNAMIC_FILTER=${enable_dynamic_filter}"
  echo "TGW_MIN_STATIC_HITS=${min_static_hits}"
  echo "TGW_MIN_DISTINCT_VIEWS=${min_distinct_views}"
  echo "TGW_MIN_STATIC_LIFETIME_SEC=${min_static_lifetime_sec}"
  echo "TGW_PLANNER_ENABLE_SHORTCUT=${planner_enable_shortcut}"
} >"${log_dir}/launch_options.log"

if [[ ! -e "${bag_path}" ]]; then
  echo "FAIL realtime_bag_plan_probe: bag path does not exist: ${bag_path}"
  exit 1
fi

pids=""
cleanup()
{
  for pid in ${pids}; do
    kill -- "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
  done
  for pid in ${pids}; do
    wait "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT

tail_logs()
{
  for log in domain launch_options fast_lio n3mapping tgw bag probe snapshot; do
    local file="${log_dir}/${log}.log"
    if [[ "${log}" == "probe" ]]; then
      file="${log_dir}/probe.out"
    elif [[ "${log}" == "snapshot" ]]; then
      file="${log_dir}/snapshot.out"
    fi
    if [[ -f "${file}" ]]; then
      echo "==== ${file} ===="
      tail -n 80 "${file}" || true
    fi
  done
}

wait_for_node()
{
  local node_name="$1"
  for _ in $(seq 1 80); do
    if ros2 node list 2>/dev/null | grep -qx "${node_name}"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_service()
{
  local service_name="$1"
  local service_type="$2"
  for _ in $(seq 1 80); do
    if ros2 service list -t 2>/dev/null | grep -Fqx "${service_name} [${service_type}]"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

setsid ros2 launch fast_lio mapping_mid360.launch.py >"${log_dir}/fast_lio.log" 2>&1 &
pids="${pids} $!"
sleep 3
setsid ros2 launch n3mapping mapping.launch.py >"${log_dir}/n3mapping.log" 2>&1 &
pids="${pids} $!"
sleep 3
setsid ros2 launch tgw_planner realtime_mapping.launch.py \
  points_topic:=/n3mapping/cloud_world \
  use_tf:=false \
  assume_cloud_in_map_frame:=true \
  publish_period_ms:=1000 \
  surface_require_observed_free_space:="${surface_require_observed_free_space}" \
  surface_allow_observed_free_bridge:="${surface_allow_observed_free_bridge}" \
  surface_require_static_support:="${surface_require_static_support}" \
  enable_dynamic_filter:="${enable_dynamic_filter}" \
  min_static_hits:="${min_static_hits}" \
  min_distinct_views:="${min_distinct_views}" \
  min_static_lifetime_sec:="${min_static_lifetime_sec}" \
  planner_enable_shortcut:="${planner_enable_shortcut}" \
  planner_require_footprint:=false \
  validation_require_footprint:=false >"${log_dir}/tgw.log" 2>&1 &
pids="${pids} $!"
if ! wait_for_node "/tgw_realtime_mapping_node"; then
  echo "FAIL realtime_bag_plan_probe: /tgw_realtime_mapping_node did not start"
  tail_logs
  exit 1
fi
if ! wait_for_service "/tgw_mapping/get_snapshot" "tgw_planner/srv/GetSnapshot"; then
  echo "FAIL realtime_bag_plan_probe: /tgw_mapping/get_snapshot service did not appear"
  tail_logs
  exit 1
fi
if ! wait_for_service "/tgw_map/plan_path" "tgw_planner/srv/PlanPath"; then
  echo "FAIL realtime_bag_plan_probe: /tgw_map/plan_path service did not appear"
  tail_logs
  exit 1
fi

setsid timeout "${play_seconds}s" ros2 bag play "${bag_path}" >"${log_dir}/bag.log" 2>&1 || true
sleep 3

ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}" \
  >"${log_dir}/snapshot.out" 2>&1 || true
snapshot_summary="$(grep -o \
  "received_clouds=[0-9]*\\|integrated_clouds=[0-9]*\\|traversable_points=[0-9]*\\|surface_points=[0-9]*\\|dynamic_points=[0-9]*" \
  "${log_dir}/snapshot.out" | tr "\n" " " || true)"
echo "${snapshot_summary}"
integrated_clouds="$(grep -o "integrated_clouds=[0-9]*" "${log_dir}/snapshot.out" | tail -n 1 | cut -d= -f2 || true)"
if (( ${integrated_clouds:-0} == 0 )); then
  echo "FAIL realtime_bag_plan_probe: realtime node integrated zero clouds"
  tail_logs
  exit 1
fi

set +e
python3 - >"${log_dir}/probe.out" 2>&1 <<'PY'
import math
import os
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
    min_dxy = float(os.environ.get("TGW_PROBE_MIN_DXY", "1.0"))
    max_dxy = float(os.environ.get("TGW_PROBE_MAX_DXY", "2.0"))
    min_abs_dz = float(os.environ.get("TGW_PROBE_MIN_ABS_DZ", "0.0"))
    max_abs_dz = float(os.environ.get("TGW_PROBE_MAX_ABS_DZ", "0.05"))
    plan_timeout = float(os.environ.get("TGW_PROBE_PLAN_TIMEOUT", "20.0"))
    sample_limit = int(os.environ.get("TGW_PROBE_SAMPLE_LIMIT", "2000"))
    top_component_limit = int(os.environ.get("TGW_PROBE_TOP_COMPONENTS", "8"))
    print(
        f"probe_criteria=min_dxy:{min_dxy:.3f} max_dxy:{max_dxy:.3f} "
        f"min_abs_dz:{min_abs_dz:.3f} max_abs_dz:{max_abs_dz:.3f} "
        f"plan_timeout:{plan_timeout:.1f} sample_limit:{sample_limit} "
        f"top_components:{top_component_limit}")

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
    print(
        f"global_bounds=x:[{min(p[0] for p in points):.3f},{max(p[0] for p in points):.3f}] "
        f"y:[{min(p[1] for p in points):.3f},{max(p[1] for p in points):.3f}] "
        f"z:[{min(p[2] for p in points):.3f},{max(p[2] for p in points):.3f}]")

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

    def summarize_component(component):
        pts = [cells[cell] for cell in component]
        return {
            "size": len(component),
            "x_min": min(p[0] for p in pts),
            "x_max": max(p[0] for p in pts),
            "y_min": min(p[1] for p in pts),
            "y_max": max(p[1] for p in pts),
            "z_min": min(p[2] for p in pts),
            "z_max": max(p[2] for p in pts),
        }

    eligible_z_components = 0
    for index, component in enumerate(components[:max(0, top_component_limit)]):
        summary = summarize_component(component)
        z_span = summary["z_max"] - summary["z_min"]
        if z_span >= min_abs_dz:
            eligible_z_components += 1
        print(
            f"component[{index}]=size:{summary['size']} "
            f"x:[{summary['x_min']:.3f},{summary['x_max']:.3f}] "
            f"y:[{summary['y_min']:.3f},{summary['y_max']:.3f}] "
            f"z:[{summary['z_min']:.3f},{summary['z_max']:.3f}] "
            f"z_span:{z_span:.3f}")
    print(f"top_components_with_required_z_span={eligible_z_components}")

    largest = components[0] if components else []
    sampled = largest[::max(1, len(largest) // max(1, sample_limit))]
    chosen = None
    for a_cell in sampled:
        a = cells[a_cell]
        for b_cell in sampled:
            b = cells[b_cell]
            dz = abs(a[2] - b[2])
            dxy = math.hypot(a[0] - b[0], a[1] - b[1])
            if min_abs_dz <= dz <= max_abs_dz and min_dxy <= dxy <= max_dxy:
                chosen = (a, b, dxy)
                break
        if chosen is not None:
            break
    if chosen is None and min_abs_dz <= 0.0 and len(largest) >= 2:
        a = cells[largest[0]]
        b = min(
            (cells[cell] for cell in largest[1:]),
            key=lambda p: abs(p[2] - a[2]) +
            abs(math.hypot(p[0] - a[0], p[1] - a[1]) - min_dxy))
        chosen = (a, b, math.hypot(a[0] - b[0], a[1] - b[1]))
    if chosen is None:
        print("success=False reason=no_component_pair_matching_probe_criteria")
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
    rclpy.spin_until_future_complete(node, future, timeout_sec=plan_timeout)
    if not future.done() or future.result() is None:
        print("success=False reason=plan_timeout")
        return 1

    response = future.result()
    stats = response.stats
    print(
        f"success={response.success} message=\"{response.message}\" "
        f"final_path_validated={stats.final_path_validated} "
        f"final_path_fallback_to_raw={stats.final_path_fallback_to_raw} "
        f"final_path_validation_failure=\"{stats.final_path_validation_failure}\" "
        f"expanded_nodes={stats.expanded_nodes} generated_nodes={stats.generated_nodes} "
        f"raw_path_waypoints={stats.raw_path_waypoints} "
        f"raw_path_length_m={stats.raw_path_length_m:.3f} "
        f"shortcut_count={stats.postprocess_floor_shortcuts} "
        f"path_waypoints={stats.path_waypoints} "
        f"path_length_m={stats.path_length_m:.3f} "
        f"min_path_clearance_m={stats.min_path_clearance_m:.3f} "
        f"mean_path_clearance_m={stats.mean_path_clearance_m:.3f} "
        f"low_clearance_samples={stats.low_clearance_samples} "
        f"clearance_cost_sum={stats.clearance_cost_sum:.3f}")
    return 0 if response.success and stats.final_path_validated and stats.path_waypoints > 0 else 1


sys.exit(main())
PY
probe_rc=$?
set -e
cat "${log_dir}/probe.out"
if [[ ${probe_rc} -ne 0 ]]; then
  echo "FAIL realtime_bag_plan_probe: planner probe failed with rc=${probe_rc}"
  tail_logs
  exit "${probe_rc}"
fi
