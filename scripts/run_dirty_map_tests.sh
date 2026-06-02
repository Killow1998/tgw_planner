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

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "dirty_dynamic_artifact: running realtime ray-clearing regression"
"${script_dir}/run_realtime_mapping_sim_tests.sh" | tee /tmp/tgw_dirty_map_realtime.out

if ! grep -q "dynamic_disappears: success=True" /tmp/tgw_dirty_map_realtime.out; then
  echo "FAIL dirty_dynamic_artifact: dynamic clearing case did not succeed"
  exit 1
fi

if ! grep -Eq "dynamic_disappears: .*dynamic_points=[1-9][0-9]*" /tmp/tgw_dirty_map_realtime.out; then
  echo "FAIL dirty_dynamic_artifact: expected non-zero dynamic suspect points"
  exit 1
fi

echo "dirty_map_tests passed"
