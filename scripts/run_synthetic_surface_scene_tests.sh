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

tmp_root="$(mktemp -d /tmp/tgw_synthetic_surface.XXXXXX)"
cleanup()
{
  rm -rf "${tmp_root}"
}
trap cleanup EXIT

write_pcd()
{
  local scene="$1"
  local output="$2"
  python3 - "${scene}" "${output}" <<'PY'
import math
import sys

scene = sys.argv[1]
output = sys.argv[2]
points = []
surface_cells = set()
RES = 0.20


def add_surface_rect(ix0, ix1, iy0, iy1, iz):
    for ix in range(min(ix0, ix1), max(ix0, ix1) + 1):
        for iy in range(min(iy0, iy1), max(iy0, iy1) + 1):
            surface_cells.add((ix, iy, iz))


def add_spiral(cx, cy, z0, turns=1.15, samples=44, radius=2.0, width=1.0, rise_total=2.0):
    for i in range(samples):
        theta0 = turns * 2.0 * math.pi * i / samples
        theta1 = turns * 2.0 * math.pi * (i + 1) / samples
        z = z0 + rise_total * i / max(1, samples - 1)
        for r_i in range(6):
            r = radius - width * 0.5 + width * r_i / 5.0
            for t_i in range(3):
                t = theta0 + (theta1 - theta0) * t_i / 2.0
                ix = int(round((cx + r * math.cos(t)) / RES))
                iy = int(round((cy + r * math.sin(t)) / RES))
                iz = int(round(z / RES)) + 1
                surface_cells.add((ix, iy, iz))


if scene == "straight":
    add_surface_rect(-8, 0, -3, 3, 1)
    for i in range(10):
        add_surface_rect(1 + 2 * i, 2 + 2 * i, -3, 3, 1 + i)
    add_surface_rect(21, 28, -3, 3, 10)
elif scene == "switchback":
    add_surface_rect(-8, 0, -3, 3, 1)
    for i in range(8):
        add_surface_rect(1 + 2 * i, 2 + 2 * i, -3, 3, 1 + i)
    add_surface_rect(17, 23, -3, 5, 8)
    for i in range(8):
        add_surface_rect(17, 23, 6 + 2 * i, 7 + 2 * i, 8 + i)
    add_surface_rect(17, 23, 22, 29, 15)
elif scene == "spiral":
    add_surface_rect(-14, 12, -3, 3, 1)
    add_spiral(0.0, 0.0, 0.0, turns=1.15, samples=96, radius=2.0, width=1.0, rise_total=2.0)
    add_surface_rect(0, 7, 6, 14, 11)
elif scene == "steep_step_chain":
    add_surface_rect(-8, 0, -3, 3, 1)
    for i in range(6):
        add_surface_rect(1 + i, 1 + i, -3, 3, 1 + 2 * (i + 1))
    add_surface_rect(7, 12, -3, 3, 13)
elif scene == "negative_gap":
    add_surface_rect(-10, -3, -3, 3, 1)
    add_surface_rect(4, 10, -3, 3, 1)
elif scene == "negative_railing_bridge":
    add_surface_rect(-10, -4, -4, 4, 1)
    add_surface_rect(4, 10, -4, 4, 1)
    add_surface_rect(-3, 3, 0, 0, 1)
else:
    raise SystemExit(f"unknown scene: {scene}")

for ix, iy, iz in sorted(surface_cells):
    points.append(((ix + 0.1) * RES, (iy + 0.1) * RES, (iz - 1 + 0.1) * RES, 0.0))

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
    for p in points:
        f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} {p[3]:.1f}\n")
PY
}

metric_value()
{
  local text="$1"
  local key="$2"
  grep -o "${key}=[^ ]*" <<<"${text}" | tail -n 1 | cut -d= -f2
}

run_success_case()
{
  local name="$1"
  local start="$2"
  local goal="$3"
  local require_footprint="${4:-0}"
  local pcd="${tmp_root}/${name}.pcd"
  write_pcd "${name}" "${pcd}"
  local output
  set +e
  output="$(ros2 run tgw_planner tgw_surface_pcd_smoke "${pcd}" 0.20 ${start} ${goal} "${require_footprint}" 2>&1)"
  local rc=$?
  set -e
  echo "${name}: ${output}"
  if [[ ${rc} -ne 0 || "$(metric_value "${output}" "success")" != "true" ]]; then
    echo "FAIL ${name}: expected success"
    return 1
  fi
  if [[ "$(metric_value "${output}" "final_path_validated")" != "true" ]]; then
    echo "FAIL ${name}: final path was not validated"
    return 1
  fi
  if (( $(metric_value "${output}" "path_waypoints") == 0 )); then
    echo "FAIL ${name}: path_waypoints is zero"
    return 1
  fi
}

run_failure_case()
{
  local name="$1"
  local start="$2"
  local goal="$3"
  local require_footprint="${4:-0}"
  local pcd="${tmp_root}/${name}.pcd"
  write_pcd "${name}" "${pcd}"
  set +e
  local output
  output="$(ros2 run tgw_planner tgw_surface_pcd_smoke "${pcd}" 0.20 ${start} ${goal} "${require_footprint}" 2>&1)"
  local rc=$?
  set -e
  echo "${name}: ${output}"
  if [[ ${rc} -eq 0 || "$(metric_value "${output}" "success")" == "true" ]]; then
    echo "FAIL ${name}: expected no path"
    return 1
  fi
}

run_success_case "straight" "-1.0 0.0 0.0" "5.0 0.0 1.8"
run_success_case "switchback" "-1.0 0.0 0.0" "3.8 5.2 2.8"
run_success_case "spiral" "-2.0 0.0 0.0" "0.8 2.0 2.0"
run_success_case "steep_step_chain" "-1.0 0.0 0.0" "2.0 0.0 2.4"
run_failure_case "negative_gap" "-1.5 0.0 0.0" "1.5 0.0 0.0"
run_failure_case "negative_railing_bridge" "-1.5 0.0 0.0" "1.5 0.0 0.0" 1

echo "synthetic_surface_scene_tests passed"
