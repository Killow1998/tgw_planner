#pragma once

#include <array>
#include <cmath>

namespace tgw_planner::core
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PointXYZI
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double intensity{0.0};
};

struct Pose3
{
  Point3 translation;
  std::array<double, 9> rotation{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
};

inline Point3 transformPoint(const Pose3 & pose, const Point3 & point)
{
  return {
    pose.rotation[0] * point.x + pose.rotation[1] * point.y + pose.rotation[2] * point.z +
      pose.translation.x,
    pose.rotation[3] * point.x + pose.rotation[4] * point.y + pose.rotation[5] * point.z +
      pose.translation.y,
    pose.rotation[6] * point.x + pose.rotation[7] * point.y + pose.rotation[8] * point.z +
      pose.translation.z};
}

inline double distance3d(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace tgw_planner::core
