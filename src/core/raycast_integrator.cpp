#include "tgw_planner/core/raycast_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{

RaycastIntegrator::RaycastIntegrator(
  MappingOptions options, SelfFilterBox body_filter_box, SelfFilterBox mount_filter_box)
: options_(options), body_filter_box_(body_filter_box), mount_filter_box_(mount_filter_box)
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
  const auto inside = [&point_sensor_frame](const SelfFilterBox & box) {
      return box.enabled &&
             point_sensor_frame.x >= box.min_x &&
             point_sensor_frame.x <= box.max_x &&
             point_sensor_frame.y >= box.min_y &&
             point_sensor_frame.y <= box.max_y &&
             point_sensor_frame.z >= box.min_z &&
             point_sensor_frame.z <= box.max_z;
    };
  return inside(body_filter_box_) || inside(mount_filter_box_);
}

std::vector<GridIndex> RaycastIntegrator::rayVoxels(
  const GridIndex & origin, const GridIndex & endpoint) const
{
  std::vector<GridIndex> voxels;
  const int dx = endpoint.x - origin.x;
  const int dy = endpoint.y - origin.y;
  const int dz = endpoint.z - origin.z;
  voxels.reserve(static_cast<std::size_t>(std::abs(dx) + std::abs(dy) + std::abs(dz)) + 1U);

  GridIndex current = origin;
  const double start_x = static_cast<double>(origin.x) + 0.5;
  const double start_y = static_cast<double>(origin.y) + 0.5;
  const double start_z = static_cast<double>(origin.z) + 0.5;
  const double end_x = static_cast<double>(endpoint.x) + 0.5;
  const double end_y = static_cast<double>(endpoint.y) + 0.5;
  const double end_z = static_cast<double>(endpoint.z) + 0.5;
  const double ray_dx = end_x - start_x;
  const double ray_dy = end_y - start_y;
  const double ray_dz = end_z - start_z;

  const auto sign = [](double value) {
      return value > 0.0 ? 1 : (value < 0.0 ? -1 : 0);
    };
  const int step_x = sign(ray_dx);
  const int step_y = sign(ray_dy);
  const int step_z = sign(ray_dz);
  const double inf = std::numeric_limits<double>::infinity();
  const double t_delta_x = step_x == 0 ? inf : 1.0 / std::abs(ray_dx);
  const double t_delta_y = step_y == 0 ? inf : 1.0 / std::abs(ray_dy);
  const double t_delta_z = step_z == 0 ? inf : 1.0 / std::abs(ray_dz);
  double t_max_x = step_x == 0 ? inf :
    ((step_x > 0 ? static_cast<double>(origin.x + 1) : static_cast<double>(origin.x)) - start_x) /
    ray_dx;
  double t_max_y = step_y == 0 ? inf :
    ((step_y > 0 ? static_cast<double>(origin.y + 1) : static_cast<double>(origin.y)) - start_y) /
    ray_dy;
  double t_max_z = step_z == 0 ? inf :
    ((step_z > 0 ? static_cast<double>(origin.z + 1) : static_cast<double>(origin.z)) - start_z) /
    ray_dz;

  constexpr double tie_epsilon = 1.0e-12;
  while (true) {
    voxels.push_back(current);
    if (current == endpoint) {
      break;
    }

    const double next_t = std::min({t_max_x, t_max_y, t_max_z});
    if (t_max_x <= next_t + tie_epsilon) {
      current.x += step_x;
      t_max_x += t_delta_x;
    }
    if (t_max_y <= next_t + tie_epsilon) {
      current.y += step_y;
      t_max_y += t_delta_y;
    }
    if (t_max_z <= next_t + tie_epsilon) {
      current.z += step_z;
      t_max_z += t_delta_z;
    }
  }
  return voxels;
}

}  // namespace tgw_planner::core
