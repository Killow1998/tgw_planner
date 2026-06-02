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

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=$((20 + RANDOM % 180))
fi

pcd_dir="${PCT_PCD_DIR:-$HOME/robot_nav_refs/PCT_planner/rsc/pcd}"
require_legacy_spiral_pass="${TGW_REQUIRE_LEGACY_SPIRAL_PASS:-0}"
require_surface_spiral_pass="${TGW_REQUIRE_SURFACE_SPIRAL_PASS:-${TGW_REQUIRE_SPIRAL_PASS:-0}}"

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

existing_nodes="$(ros2 node list 2>/dev/null | grep -E '^/tgw_planner_node$|^/tgw_clicked_point_router_node$' || true)"
if [[ -n "${existing_nodes}" ]]; then
  echo "Refusing to run while tgw_planner nodes already exist:"
  echo "${existing_nodes}"
  echo "Stop the existing launch first so /plan_path is not served by the wrong map."
  exit 1
fi

wait_for_node()
{
  for _ in $(seq 1 80); do
    if ros2 node list 2>/dev/null | grep -qx "/tgw_planner_node"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

wait_for_map()
{
  local log_file="$1"
  for _ in $(seq 1 120); do
    if grep -q "\\[NavMapBuilder\\] build_time_ms" "${log_file}" 2>/dev/null; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

metric_value()
{
  local text="$1"
  local key="$2"
  grep -o "${key}=[^ ]*" <<<"${text}" | tail -n 1 | cut -d= -f2
}

require_surface_metrics()
{
  local name="$1"
  local output="$2"
  local final_path_validated path_waypoints expanded_nodes min_clearance mean_clearance
  final_path_validated="$(metric_value "${output}" "final_path_validated")"
  path_waypoints="$(metric_value "${output}" "path_waypoints")"
  expanded_nodes="$(metric_value "${output}" "expanded_nodes")"
  min_clearance="$(metric_value "${output}" "min_path_clearance_m")"
  mean_clearance="$(metric_value "${output}" "mean_path_clearance_m")"
  if [[ "${final_path_validated}" != "true" ]]; then
    echo "FAIL ${name}: final_path_validated is not true"
    return 1
  fi
  if (( ${path_waypoints:-0} == 0 )); then
    echo "FAIL ${name}: path_waypoints is zero"
    return 1
  fi
  if (( ${expanded_nodes:-0} == 0 )); then
    echo "FAIL ${name}: expanded_nodes is zero"
    return 1
  fi
  if [[ -z "${min_clearance}" || -z "${mean_clearance}" ]]; then
    echo "FAIL ${name}: clearance metrics are missing"
    return 1
  fi
}

run_case()
{
  local name="$1"
  local pcd="$2"
  local resolution="$3"
  local start_x="$4"
  local start_y="$5"
  local start_z="$6"
  local goal_x="$7"
  local goal_y="$8"
  local goal_z="$9"
  local expected="${10}"

  if [[ ! -f "${pcd}" ]]; then
    echo "SKIP ${name}: missing ${pcd}"
    return 0
  fi

  cleanup
  local log_file="/tmp/tgw_${name}_launch.log"
  local call_file="/tmp/tgw_${name}_service.out"
  ros2 launch tgw_planner pcd_to_path_mvp.launch.py \
    use_rviz:=false \
    max_marker_cells:=20 \
    map_resolution_m:="${resolution}" \
    pcd_file:="${pcd}" >"${log_file}" 2>&1 &
  launch_pid="$!"
  if ! wait_for_node; then
    echo "FAIL ${name}: /tgw_planner_node did not appear"
    tail -n 80 "${log_file}" || true
    return 1
  fi
  if ! wait_for_map "${log_file}"; then
    echo "FAIL ${name}: map build did not finish"
    tail -n 120 "${log_file}" || true
    return 1
  fi

  local request
  request="{start: {header: {frame_id: map}, pose: {position: {x: ${start_x}, y: ${start_y}, z: ${start_z}}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: ${goal_x}, y: ${goal_y}, z: ${goal_z}}, orientation: {w: 1.0}}}}"
  ros2 service call /plan_path tgw_planner/srv/PlanPath "${request}" >"${call_file}" 2>&1 || true

  local success
  success="$(grep -o "success=True\\|success=False" "${call_file}" | tail -n 1 || true)"
  local summary
  summary="$(grep -o "expanded_nodes=[0-9]*\\| raw_path_waypoints=[0-9]*\\| raw_path_length_m=[0-9.]*\\| postprocess_floor_shortcuts=[0-9]*\\| path_waypoints=[0-9]*\\| path_length_m=[0-9.]*\\| path_vertical_gain_m=[0-9.]*" "${call_file}" | sed 's/^ //' | tr '\n' ' ')"
  echo "${name}: ${success:-success=unknown} ${summary}"

  cleanup
  if [[ "${expected}" == "pass" && "${success}" != "success=True" ]]; then
    echo "FAIL ${name}: expected success"
    return 1
  fi
  if [[ "${expected}" == "spiral" && "${success}" != "success=True" && "${require_legacy_spiral_pass}" == "1" ]]; then
    echo "FAIL ${name}: legacy spiral pass is required"
    return 1
  fi
  return 0
}

run_surface_case()
{
  local name="$1"
  local pcd="$2"
  local resolution="$3"
  local start_x="$4"
  local start_y="$5"
  local start_z="$6"
  local goal_x="$7"
  local goal_y="$8"
  local goal_z="$9"
  local expected="${10}"

  if [[ ! -f "${pcd}" ]]; then
    echo "SKIP ${name}: missing ${pcd}"
    return 0
  fi

  local output
  set +e
  output="$(ros2 run tgw_planner tgw_surface_pcd_smoke \
    "${pcd}" "${resolution}" "${start_x}" "${start_y}" "${start_z}" \
    "${goal_x}" "${goal_y}" "${goal_z}" 0 2>&1)"
  local rc=$?
  set -e
  echo "${name}: ${output}"

  if [[ "${expected}" == "pass" && ${rc} -ne 0 ]]; then
    echo "FAIL ${name}: expected success"
    return 1
  fi
  if [[ "${expected}" == "pass" ]]; then
    require_surface_metrics "${name}" "${output}" || return 1
  fi
  if [[ "${expected}" == "spiral" && ${rc} -ne 0 && "${require_surface_spiral_pass}" == "1" ]]; then
    echo "FAIL ${name}: spiral surface pass is required"
    return 1
  fi
  if [[ "${expected}" == "spiral" && "${require_surface_spiral_pass}" == "1" ]]; then
    require_surface_metrics "${name}" "${output}" || return 1
  fi
  return 0
}

building_pcd="${pcd_dir}/extracted/building2_9.pcd"
if [[ ! -f "${building_pcd}" ]]; then
  building_pcd="${pcd_dir}/building2_9.pcd"
fi
spiral_pcd="${pcd_dir}/spiral0.3_2.pcd"

run_case "pct_building" "${building_pcd}" "0.10" "5.0" "5.0" "0.0" "-6.0" "-1.0" "0.0" "pass"
run_case "pct_spiral" "${spiral_pcd}" "0.20" "-16.0" "-6.0" "0.0" "-26.0" "-5.0" "0.0" "spiral"
run_surface_case "surface_pct_building" "${building_pcd}" "0.10" "5.0" "5.0" "0.0" "-6.0" "-1.0" "0.0" "pass"
run_surface_case "surface_pct_spiral" "${spiral_pcd}" "0.20" "-16.0" "-6.0" "0.0" "-26.0" "-5.0" "0.0" "spiral"
