#include "tgw_planner/core/raycast_integrator.hpp"

#include <algorithm>
#include <cmath>

namespace tgw_planner::core
{

RaycastIntegrator::RaycastIntegrator(MappingOptions options, SelfFilterBox self_filter_box)
: options_(options), self_filter_box_(self_filter_box)
{
  options_.resolution_m = std::max(0.01, options_.resolution_m);
}

RaycastStats RaycastIntegrator::insertScan(
  const ScanInput & scan, ProbabilisticVoxelMap & map) const
{
  RaycastStats stats;
  const GridIndex origin_idx = map.worldToGrid(scan.sensor_pose_map.translation);

  for (const Point3 & point_sensor : scan.points_sensor_frame) {
    ++stats.input_points;
    if (isInvalid(point_sensor)) {
      ++stats.filtered_invalid;
      continue;
    }
    const double range = std::sqrt(
      point_sensor.x * point_sensor.x + point_sensor.y * point_sensor.y +
      point_sensor.z * point_sensor.z);
    if (range < options_.min_range_m || range > options_.max_range_m) {
      ++stats.filtered_range;
      continue;
    }
    if (options_.enable_self_filter && isSelfPoint(point_sensor)) {
      ++stats.filtered_self;
      continue;
    }

    const Point3 endpoint_map = transformPoint(scan.sensor_pose_map, point_sensor);
    const GridIndex endpoint_idx = map.worldToGrid(endpoint_map);
    const std::vector<GridIndex> ray = rayVoxels(origin_idx, endpoint_idx);
    for (std::size_t i = 0; i + 1U < ray.size(); ++i) {
      map.updateMiss(ray[i], scan.stamp_sec, scan.view_id);
      ++stats.miss_updates;
    }
    map.updateHit(endpoint_idx, scan.stamp_sec, scan.view_id);
    ++stats.hit_updates;
    ++stats.inserted_points;
  }

  map.decayDynamic(scan.stamp_sec);
  stats.dynamic_suspect_voxels_after_decay = map.dynamicSuspectVoxels().size();
  stats.static_candidate_voxels_after_decay = map.staticCandidateVoxels().size();
  return stats;
}

bool RaycastIntegrator::isInvalid(const Point3 & point) const
{
  return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
}

bool RaycastIntegrator::isSelfPoint(const Point3 & point_sensor_frame) const
{
  return point_sensor_frame.x >= self_filter_box_.min_x &&
         point_sensor_frame.x <= self_filter_box_.max_x &&
         point_sensor_frame.y >= self_filter_box_.min_y &&
         point_sensor_frame.y <= self_filter_box_.max_y &&
         point_sensor_frame.z >= self_filter_box_.min_z &&
         point_sensor_frame.z <= self_filter_box_.max_z;
}

std::vector<GridIndex> RaycastIntegrator::rayVoxels(
  const GridIndex & origin, const GridIndex & endpoint) const
{
  const int dx = endpoint.x - origin.x;
  const int dy = endpoint.y - origin.y;
  const int dz = endpoint.z - origin.z;
  const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz), 1});

  std::vector<GridIndex> voxels;
  voxels.reserve(static_cast<std::size_t>(steps) + 1U);
  GridIndex last{origin.x - 1, origin.y - 1, origin.z - 1};
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const GridIndex idx{
      static_cast<int>(std::floor(static_cast<double>(origin.x) + t * static_cast<double>(dx) + 0.5)),
      static_cast<int>(std::floor(static_cast<double>(origin.y) + t * static_cast<double>(dy) + 0.5)),
      static_cast<int>(std::floor(static_cast<double>(origin.z) + t * static_cast<double>(dz) + 0.5))};
    if (idx != last) {
      voxels.push_back(idx);
      last = idx;
    }
  }
  if (voxels.empty() || voxels.back() != endpoint) {
    voxels.push_back(endpoint);
  }
  return voxels;
}

}  // namespace tgw_planner::core
