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

const RobotFootprintOptions & RobotFootprint::options() const
{
  return options_;
}

}  // namespace tgw_planner::core
