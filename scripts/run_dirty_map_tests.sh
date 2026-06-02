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

output_file="$(mktemp /tmp/tgw_dirty_map_realtime.XXXXXX.out)"
tmp_root="$(mktemp -d /tmp/tgw_dirty_map_pcd.XXXXXX)"
launch_pid=""
cleanup()
{
  if [[ -n "${launch_pid}" ]]; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
  rm -f "${output_file}"
  rm -rf "${tmp_root}"
}
trap cleanup EXIT

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

wait_for_map_build()
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

write_dirty_pcd()
{
  local output="$1"
  python3 - "${output}" <<'PY'
import sys

output = sys.argv[1]
points = []
res = 0.20


def add_rect(ix0, ix1, iy0, iy1, z):
    for ix in range(ix0, ix1 + 1):
        for iy in range(iy0, iy1 + 1):
            points.append(((ix + 0.1) * res, (iy + 0.1) * res, z, 0.0))


def add_human_like_column(cx, cy):
    for ix in range(cx - 1, cx + 2):
        for iy in range(cy - 1, cy + 2):
            for iz in range(1, 9):
                points.append(((ix + 0.1) * res, (iy + 0.1) * res, (iz + 0.1) * res, 0.0))


add_rect(-18, 18, -10, 10, 0.0)
add_rect(-18, 18, -10, 10, 2.0)
for cx, cy in [(-10, -5), (-4, 4), (0, -1), (6, 5), (11, -4)]:
    add_human_like_column(cx, cy)

with open(output, "w", encoding="ascii") as f:
    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z intensity\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write(f"WIDTH {len(points)}\n")
    f.write("HEIGHT 1\n")
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write(f"POINTS {len(points)}\n")
    f.write("DATA ascii\n")
    for x, y, z, intensity in points:
        f.write(f"{x:.3f} {y:.3f} {z:.3f} {intensity:.1f}\n")
PY
}

run_dirty_pcd_warning_test()
{
  local pcd="${tmp_root}/dirty_human_artifact.pcd"
  local log_file="${tmp_root}/pcd_import.log"
  local stats_file="${tmp_root}/map_build_stats_json.out"

  write_dirty_pcd "${pcd}"
  echo "dirty_pcd_artifact_warning: running PCD import warning regression"
  ros2 launch tgw_planner pcd_to_path_mvp.launch.py \
    use_rviz:=false \
    max_marker_cells:=20 \
    map_resolution_m:=0.20 \
    pcd_file:="${pcd}" >"${log_file}" 2>&1 &
  launch_pid="$!"

  if ! wait_for_node "/tgw_planner_node"; then
    echo "FAIL dirty_pcd_artifact_warning: /tgw_planner_node did not appear"
    tail -n 80 "${log_file}" || true
    return 1
  fi
  if ! wait_for_map_build "${log_file}"; then
    echo "FAIL dirty_pcd_artifact_warning: map build did not finish"
    tail -n 120 "${log_file}" || true
    return 1
  fi
  timeout 8s ros2 topic echo --once /map_build_stats_json std_msgs/msg/String \
    >"${stats_file}" 2>&1 || true

  if ! grep -Fq '"pcd_artifact_warning":true' "${stats_file}"; then
    echo "FAIL dirty_pcd_artifact_warning: missing pcd_artifact_warning=true"
    cat "${stats_file}" || true
    return 1
  fi
  if ! grep -Fq '"possible_artifacts_detected":true' "${stats_file}"; then
    echo "FAIL dirty_pcd_artifact_warning: expected possible_artifacts_detected=true"
    tail -n 120 "${log_file}" || true
    cat "${stats_file}" || true
    return 1
  fi
  if ! grep -q "\\[PCD MODE WARNING\\] possible artifacts detected" "${log_file}"; then
    echo "FAIL dirty_pcd_artifact_warning: missing console artifact warning"
    tail -n 120 "${log_file}" || true
    return 1
  fi
  echo "dirty_pcd_artifact_warning: success=true"

  kill "${launch_pid}" >/dev/null 2>&1 || true
  wait "${launch_pid}" >/dev/null 2>&1 || true
  launch_pid=""
}

echo "dirty_dynamic_artifact: running realtime ray-clearing regression"
"${script_dir}/run_realtime_mapping_sim_tests.sh" | tee "${output_file}"

if ! grep -q "dynamic_disappears: success=True" "${output_file}"; then
  echo "FAIL dirty_dynamic_artifact: dynamic clearing case did not succeed"
  exit 1
fi

if ! grep -Eq "dynamic_disappears: .*dynamic_points=[1-9][0-9]*" "${output_file}"; then
  echo "FAIL dirty_dynamic_artifact: expected non-zero dynamic suspect points"
  exit 1
fi

run_dirty_pcd_warning_test

echo "dirty_map_tests passed"
