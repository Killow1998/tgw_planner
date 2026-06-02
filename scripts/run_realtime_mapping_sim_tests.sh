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

launch_pid=""
tmp_paths=()
tmp_root="$(mktemp -d /tmp/tgw_realtime_sim.XXXXXX)"
cleanup_case()
{
  if [[ -n "${launch_pid}" ]]; then
    kill -- "-${launch_pid}" >/dev/null 2>&1 || kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
    launch_pid=""
  fi
  if (( ${#tmp_paths[@]} > 0 )); then
    rm -rf "${tmp_paths[@]}"
    tmp_paths=()
  fi
}
cleanup()
{
  cleanup_case
  rm -rf "${tmp_root}"
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

wait_for_service()
{
  local service="$1"
  for _ in $(seq 1 80); do
    if ros2 service list 2>/dev/null | grep -qx "${service}"; then
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
time.sleep(0.5)
for _ in range(2):
    publish_frame(node, [(1.0, 0.0, 0.0)])
for _ in range(8):
    publish_frame(node, [(2.0, 0.0, 0.0)])

node.destroy_node()
rclpy.shutdown()
PY
}

publish_single_static_point_scene()
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
        super().__init__("tgw_control_sim_pub")
        self.pose_pub = self.create_publisher(PoseStamped, "/tgw_sim/pose", 10)
        self.cloud_pub = self.create_publisher(point_cloud2.PointCloud2, "/tgw_sim/points", 10)


rclpy.init()
node = Publisher()
points = [(1.0, 0.0, 0.0)]
for _ in range(5):
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

node.destroy_node()
rclpy.shutdown()
PY
}

snapshot_occupied_points()
{
  local output_file="$1"
  ros2 service call /tgw_mapping/get_snapshot tgw_planner/srv/GetSnapshot "{}" \
    >"${output_file}" 2>&1
  value_from_snapshot "${output_file}" "occupied_points"
}

run_floor_ceiling_case()
{
  cleanup_case
  local log_file="${tmp_root}/floor_ceiling_launch.log"
  local snapshot_file="${tmp_root}/floor_ceiling_snapshot.out"
  setsid ros2 launch tgw_planner realtime_mapping.launch.py \
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
  if ! wait_for_node; then
    echo "FAIL floor_ceiling_free_space: /tgw_realtime_mapping_node did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi
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
  cleanup_case
  local log_file="${tmp_root}/dynamic_disappears_launch.log"
  local snapshot_file="${tmp_root}/dynamic_disappears_snapshot.out"
  setsid ros2 launch tgw_planner realtime_mapping.launch.py \
    use_tf:=false \
    assume_cloud_in_map_frame:=false \
    points_topic:=/tgw_sim/points \
    pose_topic:=/tgw_sim/pose \
    publish_period_ms:=300 \
    min_static_hits:=3 \
    min_distinct_views:=2 \
    min_static_lifetime_sec:=10.0 \
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
  if ! wait_for_node; then
    echo "FAIL dynamic_disappears: /tgw_realtime_mapping_node did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi
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

run_mapping_control_case()
{
  cleanup_case
  local log_file="${tmp_root}/mapping_control_launch.log"
  local start_file="${tmp_root}/mapping_control_start.out"
  local restart_file="${tmp_root}/mapping_control_restart.out"
  local pause_file="${tmp_root}/mapping_control_pause.out"
  local stop_file="${tmp_root}/mapping_control_stop.out"
  local clear_file="${tmp_root}/mapping_control_clear.out"
  local snapshot_initial_stopped="${tmp_root}/mapping_control_initial_stopped_snapshot.out"
  local snapshot_started="${tmp_root}/mapping_control_started_snapshot.out"
  local snapshot_paused="${tmp_root}/mapping_control_paused_snapshot.out"
  local snapshot_restarted="${tmp_root}/mapping_control_restarted_snapshot.out"
  local snapshot_final_stopped="${tmp_root}/mapping_control_final_stopped_snapshot.out"
  tmp_paths+=(
    "${pause_file}" "${stop_file}" "${start_file}" "${restart_file}" "${clear_file}"
    "${snapshot_initial_stopped}" "${snapshot_started}" "${snapshot_paused}"
    "${snapshot_restarted}" "${snapshot_final_stopped}")

  setsid ros2 launch tgw_planner realtime_mapping.launch.py \
    start_enabled:=false \
    use_tf:=false \
    assume_cloud_in_map_frame:=false \
    points_topic:=/tgw_sim/points \
    pose_topic:=/tgw_sim/pose \
    publish_period_ms:=300 \
    min_static_hits:=1 \
    min_distinct_views:=1 \
    min_static_lifetime_sec:=0.0 \
    enable_dynamic_filter:=false \
    enable_self_filter:=false \
    max_range_m:=10.0 \
    min_range_m:=0.01 \
    planner_require_footprint:=false \
    validation_require_footprint:=false >"${log_file}" 2>&1 &
  launch_pid="$!"
  if ! wait_for_node; then
    echo "FAIL mapping_control: /tgw_realtime_mapping_node did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi
  for service in /tgw_mapping/stop /tgw_mapping/start /tgw_mapping/pause /tgw_mapping/clear /tgw_mapping/get_snapshot; do
    if ! wait_for_service "${service}"; then
      echo "FAIL mapping_control: ${service} did not appear"
      tail -n 120 "${log_file}" || true
      return 1
    fi
  done

  publish_single_static_point_scene
  sleep 0.5
  local initial_stopped_occupied
  initial_stopped_occupied="$(snapshot_occupied_points "${snapshot_initial_stopped}")"

  ros2 service call /tgw_mapping/start std_srvs/srv/Trigger "{}" >"${start_file}" 2>&1
  publish_single_static_point_scene
  sleep 0.5
  local started_occupied
  started_occupied="$(snapshot_occupied_points "${snapshot_started}")"

  ros2 service call /tgw_mapping/pause std_srvs/srv/Trigger "{}" >"${pause_file}" 2>&1
  ros2 service call /tgw_mapping/clear std_srvs/srv/Trigger "{}" >"${clear_file}" 2>&1
  publish_single_static_point_scene
  sleep 0.5
  local paused_occupied
  paused_occupied="$(snapshot_occupied_points "${snapshot_paused}")"

  ros2 service call /tgw_mapping/start std_srvs/srv/Trigger "{}" >"${restart_file}" 2>&1
  publish_single_static_point_scene
  sleep 0.5
  local restarted_occupied
  restarted_occupied="$(snapshot_occupied_points "${snapshot_restarted}")"

  ros2 service call /tgw_mapping/stop std_srvs/srv/Trigger "{}" >"${stop_file}" 2>&1
  ros2 service call /tgw_mapping/clear std_srvs/srv/Trigger "{}" >"${clear_file}" 2>&1
  publish_single_static_point_scene
  sleep 0.5
  local final_stopped_occupied
  final_stopped_occupied="$(snapshot_occupied_points "${snapshot_final_stopped}")"

  echo "mapping_control: initial_stopped_occupied=${initial_stopped_occupied:-unknown} started_occupied=${started_occupied:-unknown} paused_occupied=${paused_occupied:-unknown} restarted_occupied=${restarted_occupied:-unknown} final_stopped_occupied=${final_stopped_occupied:-unknown}"
  if (( ${initial_stopped_occupied:-999999} != 0 )); then
    echo "FAIL mapping_control: start_enabled=false did not prevent integration"
    cat "${snapshot_initial_stopped}"
    return 1
  fi
  if (( ${started_occupied:-0} == 0 )); then
    echo "FAIL mapping_control: start did not resume integration"
    cat "${start_file}" "${snapshot_started}"
    return 1
  fi
  if (( ${paused_occupied:-999999} != 0 )); then
    echo "FAIL mapping_control: pause+clear did not prevent fresh integration"
    cat "${pause_file}" "${clear_file}" "${snapshot_paused}"
    return 1
  fi
  if (( ${restarted_occupied:-0} == 0 )); then
    echo "FAIL mapping_control: restart after pause did not resume integration"
    cat "${restart_file}" "${snapshot_restarted}"
    return 1
  fi
  if (( ${final_stopped_occupied:-999999} != 0 )); then
    echo "FAIL mapping_control: stop+clear did not prevent fresh integration"
    cat "${stop_file}" "${clear_file}" "${snapshot_final_stopped}"
    return 1
  fi
}

run_blocked_region_persistence_case()
{
  cleanup_case
  local log_file="${tmp_root}/blocked_region_launch.log"
  local output_dir
  output_dir="$(mktemp -d "${tmp_root}/blocked_region_map.XXXXXX")"
  local mismatch_dir
  mismatch_dir="$(mktemp -d "${tmp_root}/blocked_region_mismatch_map.XXXXXX")"
  local bad_format_dir
  bad_format_dir="$(mktemp -d "${tmp_root}/blocked_region_bad_format_map.XXXXXX")"
  local evidence_only_dir
  evidence_only_dir="$(mktemp -d "${tmp_root}/blocked_region_evidence_only_map.XXXXXX")"
  local add_file="${tmp_root}/blocked_region_add.out"
  local save_file="${tmp_root}/blocked_region_save.out"
  local clear_file="${tmp_root}/blocked_region_clear.out"
  local load_file="${tmp_root}/blocked_region_load.out"
  local evidence_only_load_file="${tmp_root}/blocked_region_evidence_only_load.out"
  local mismatch_load_file="${tmp_root}/blocked_region_mismatch_load.out"
  local bad_format_load_file="${tmp_root}/blocked_region_bad_format_load.out"
  local remove_file="${tmp_root}/blocked_region_remove.out"
  tmp_paths+=(
    "${output_dir}" "${mismatch_dir}" "${bad_format_dir}" "${evidence_only_dir}"
    "${add_file}" "${save_file}" "${clear_file}" "${load_file}"
    "${evidence_only_load_file}" "${mismatch_load_file}" "${bad_format_load_file}"
    "${remove_file}")

  setsid ros2 launch tgw_planner realtime_mapping.launch.py \
    publish_period_ms:=300 \
    planner_require_footprint:=false \
    validation_require_footprint:=false >"${log_file}" 2>&1 &
  launch_pid="$!"
  if ! wait_for_node; then
    echo "FAIL blocked_region_persistence: /tgw_realtime_mapping_node did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi
  if ! wait_for_service "/plan_path"; then
    echo "FAIL blocked_region_persistence: /plan_path did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi
  if ! wait_for_service "/nav_map/set_blocked_region"; then
    echo "FAIL blocked_region_persistence: /nav_map/set_blocked_region did not appear"
    tail -n 120 "${log_file}" || true
    return 1
  fi

  ros2 service call /tgw_map/set_blocked_region tgw_planner/srv/SetBlockedRegion \
    "{operation: add, min: {x: 0.0, y: 0.0, z: 0.0}, max: {x: 1.0, y: 1.0, z: 1.0}, reason: regression}" \
    >"${add_file}" 2>&1
  ros2 service call /tgw_mapping/save_map tgw_planner/srv/SaveMap \
    "{output_dir: ${output_dir}}" >"${save_file}" 2>&1
  if ! grep -q "voxel_evidence_csv=" "${save_file}"; then
    echo "FAIL blocked_region_persistence: SaveMap response did not report voxel_evidence_csv"
    cat "${save_file}"
    return 1
  fi
  if [[ ! -f "${output_dir}/metadata.yaml" ]]; then
    echo "FAIL blocked_region_persistence: save_map did not write metadata.yaml"
    cat "${save_file}"
    return 1
  fi
  if ! grep -q "^resolution_m: 0.100000" "${output_dir}/metadata.yaml"; then
    echo "FAIL blocked_region_persistence: metadata.yaml did not record map resolution"
    cat "${output_dir}/metadata.yaml"
    return 1
  fi
  if [[ ! -f "${output_dir}/voxel_evidence.csv" ]]; then
    echo "FAIL blocked_region_persistence: save_map did not write voxel_evidence.csv"
    cat "${save_file}"
    return 1
  fi
  if ! head -n 1 "${output_dir}/voxel_evidence.csv" | grep -q "^x,y,z,log_odds,hit_count"; then
    echo "FAIL blocked_region_persistence: voxel_evidence.csv has an invalid header"
    head -n 5 "${output_dir}/voxel_evidence.csv"
    return 1
  fi
  cp -a "${output_dir}/." "${evidence_only_dir}/"
  rm -f \
    "${evidence_only_dir}/occupied_cloud.pcd" \
    "${evidence_only_dir}/free_cloud.pcd" \
    "${evidence_only_dir}/static_candidate_cloud.pcd" \
    "${evidence_only_dir}/dynamic_suspect_cloud.pcd"
  cp -a "${output_dir}/." "${mismatch_dir}/"
  python3 - "${mismatch_dir}/metadata.yaml" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
path.write_text(text.replace("resolution_m: 0.100000", "resolution_m: 0.200000", 1))
PY
  cp -a "${output_dir}/." "${bad_format_dir}/"
  python3 - "${bad_format_dir}/metadata.yaml" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
path.write_text(text.replace("map_format: tgw_realtime_map_v1", "map_format: wrong_format", 1))
PY
  ros2 service call /tgw_mapping/clear std_srvs/srv/Trigger "{}" >"${clear_file}" 2>&1
  ros2 service call /tgw_mapping/load_map tgw_planner/srv/LoadMap \
    "{input_dir: ${output_dir}}" >"${load_file}" 2>&1
  ros2 service call /tgw_mapping/load_map tgw_planner/srv/LoadMap \
    "{input_dir: ${evidence_only_dir}}" >"${evidence_only_load_file}" 2>&1
  ros2 service call /tgw_mapping/load_map tgw_planner/srv/LoadMap \
    "{input_dir: ${mismatch_dir}}" >"${mismatch_load_file}" 2>&1
  ros2 service call /tgw_mapping/load_map tgw_planner/srv/LoadMap \
    "{input_dir: ${bad_format_dir}}" >"${bad_format_load_file}" 2>&1
  ros2 service call /tgw_map/set_blocked_region tgw_planner/srv/SetBlockedRegion \
    "{operation: remove, min: {x: 0.0, y: 0.0, z: 0.0}, max: {x: 1.0, y: 1.0, z: 1.0}, reason: regression}" \
    >"${remove_file}" 2>&1

  local save_success load_success evidence_only_success loaded_evidence evidence_only_loaded_evidence mismatch_failed bad_format_failed removed_region
  save_success="$(grep -o "success=True\\|success=False" "${save_file}" | tail -n 1 || true)"
  load_success="$(grep -o "success=True\\|success=False" "${load_file}" | tail -n 1 || true)"
  evidence_only_success="$(grep -o "success=True\\|success=False" "${evidence_only_load_file}" | tail -n 1 || true)"
  loaded_evidence="$(grep -o "loaded_voxel_evidence=True" "${load_file}" | tail -n 1 || true)"
  evidence_only_loaded_evidence="$(grep -o "loaded_voxel_evidence=True" "${evidence_only_load_file}" | tail -n 1 || true)"
  mismatch_failed="$(grep -o "map resolution mismatch" "${mismatch_load_file}" | tail -n 1 || true)"
  bad_format_failed="$(grep -o "unsupported realtime map format" "${bad_format_load_file}" | tail -n 1 || true)"
  removed_region="$(grep -o "removed 1 realtime blocked regions" "${remove_file}" | tail -n 1 || true)"
  echo "blocked_region_persistence: ${save_success:-save=unknown} ${load_success:-load=unknown} loaded_evidence=${loaded_evidence:+true} evidence_only_load=${evidence_only_success:-unknown} evidence_only_loaded_evidence=${evidence_only_loaded_evidence:+true} metadata_mismatch_rejected=${mismatch_failed:+true} bad_format_rejected=${bad_format_failed:+true} removed_region=${removed_region:+true}"
  if [[ "${save_success}" != "success=True" || "${load_success}" != "success=True" ]]; then
    echo "FAIL blocked_region_persistence: save/load failed"
    cat "${save_file}" "${load_file}"
    return 1
  fi
  if [[ "${evidence_only_success}" != "success=True" ]]; then
    echo "FAIL blocked_region_persistence: evidence-only map package did not load"
    cat "${evidence_only_load_file}"
    return 1
  fi
  if [[ -z "${loaded_evidence}" || -z "${evidence_only_loaded_evidence}" ]]; then
    echo "FAIL blocked_region_persistence: LoadMap did not report loaded_voxel_evidence=true"
    cat "${load_file}" "${evidence_only_load_file}"
    return 1
  fi
  if [[ -z "${mismatch_failed}" ]]; then
    echo "FAIL blocked_region_persistence: metadata resolution mismatch was not rejected"
    cat "${mismatch_load_file}"
    return 1
  fi
  if [[ -z "${bad_format_failed}" ]]; then
    echo "FAIL blocked_region_persistence: unsupported metadata format was not rejected"
    cat "${bad_format_load_file}"
    return 1
  fi
  if [[ -z "${removed_region}" ]]; then
    echo "FAIL blocked_region_persistence: loaded blocked_regions.yaml did not restore an editable region"
    cat "${remove_file}"
    return 1
  fi
}

run_floor_ceiling_case
run_dynamic_disappears_case
run_mapping_control_case
run_blocked_region_persistence_case
