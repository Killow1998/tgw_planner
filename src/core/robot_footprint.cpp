#include "tgw_planner/core/robot_footprint.hpp"

#include <algorithm>
#include <cmath>

namespace tgw_planner::core
{

RobotFootprint::RobotFootprint(RobotFootprintOptions options)
: options_(options)
{
  options_.length_m = std::max(0.10, options_.length_m);
  options_.width_m = std::max(0.10, options_.width_m);
  options_.height_m = std::max(0.10, options_.height_m);
  options_.support_height_tolerance_m = std::max(0.0, options_.support_height_tolerance_m);
  options_.min_support_ratio = std::clamp(options_.min_support_ratio, 0.0, 1.0);
  options_.base_to_front_m =
    std::clamp(options_.base_to_front_m, 0.01, options_.length_m - 0.01);
}

std::vector<Point3> RobotFootprint::sampleFootprint(
  const Point3 & center, double yaw_rad, double resolution_m) const
{
  const double step = std::max(0.05, 0.5 * resolution_m);
  const double front = options_.base_to_front_m;
  const double rear = options_.length_m - options_.base_to_front_m;
  const double half_width = 0.5 * options_.width_m;
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);

  std::vector<Point3> out;
  for (double x = -rear; x <= front + 1.0e-9; x += step) {
    for (double y = -half_width; y <= half_width + 1.0e-9; y += step) {
      out.push_back({
        center.x + x * cos_yaw - y * sin_yaw,
        center.y + x * sin_yaw + y * cos_yaw,
        center.z});
    }
  }
  return out;
}

bool RobotFootprint::containsBodyPoint(const Point3 & point_base) const
{
  const double front = options_.base_to_front_m;
  const double rear = options_.length_m - options_.base_to_front_m;
  const double half_width = 0.5 * options_.width_m;
  return point_base.x >= -rear && point_base.x <= front &&
         std::abs(point_base.y) <= half_width &&
         point_base.z >= -0.5 * options_.height_m &&
         point_base.z <= 0.5 * options_.height_m;
}

bool RobotFootprint::isSupported(
  const SurfaceMap & surface, const Point3 & center, double yaw_rad, double resolution_m) const
{
  const FootprintSupportReport report = supportReport(surface, center, yaw_rad, resolution_m);
  return report.reason.empty() && report.support_ratio >= options_.min_support_ratio;
}

FootprintSupportReport RobotFootprint::supportReport(
  const SurfaceMap & surface, const Point3 & center, double yaw_rad, double resolution_m) const
{
  FootprintSupportReport report;
  const int vertical_tolerance_cells = std::max(
    0, static_cast<int>(std::ceil(options_.support_height_tolerance_m / resolution_m)));

  const double step = std::max(0.05, 0.5 * resolution_m);
  const double front = options_.base_to_front_m;
  const double rear = options_.length_m - options_.base_to_front_m;
  const double half_width = 0.5 * options_.width_m;
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);

  for (double x = -rear; x <= front + 1.0e-9; x += step) {
    for (double y = -half_width; y <= half_width + 1.0e-9; y += step) {
      ++report.total_samples;
      const Point3 sample{
        center.x + x * cos_yaw - y * sin_yaw,
        center.y + x * sin_yaw + y * cos_yaw,
        center.z};
      const GridIndex cell = worldToGrid(sample, resolution_m);
      bool supported = false;
      for (int dz = -vertical_tolerance_cells; dz <= vertical_tolerance_cells; ++dz) {
        const GridIndex candidate{cell.x, cell.y, cell.z + dz};
        if (surface.forbidden_cells.find(candidate) != surface.forbidden_cells.end()) {
          report.reason = "footprint overlaps forbidden cell";
          return report;
        }
        if (surface.blocked_cells.find(candidate) != surface.blocked_cells.end()) {
          report.reason = "footprint overlaps blocked cell";
          return report;
        }
        if (surface.traversable_cells.find(candidate) == surface.traversable_cells.end()) {
          continue;
        }
        supported = true;
        break;
      }
      if (supported) {
        ++report.supported_samples;
      }
    }
  }
  if (report.total_samples > 0) {
    report.support_ratio =
      static_cast<double>(report.supported_samples) / static_cast<double>(report.total_samples);
  }
  if (report.support_ratio < options_.min_support_ratio) {
    report.reason = "footprint support ratio below minimum";
  }
  return report;
}

const RobotFootprintOptions & RobotFootprint::options() const
{
  return options_;
}

GridIndex RobotFootprint::worldToGrid(const Point3 & point, double resolution_m) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

}  // namespace tgw_planner::core
