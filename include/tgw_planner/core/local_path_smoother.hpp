#pragma once

#include <string>
#include <vector>

#include "tgw_planner/core/rolling_local_map.hpp"
#include "tgw_planner/core/route_progress_tracker.hpp"

namespace tgw_planner::core
{

struct LocalPathSmootherOptions
{
  double min_point_spacing_m{0.20};
  double max_point_spacing_m{0.20};
  bool enable_collision_check{true};
  double bezier_handle_ratio{0.35};
  double max_smoothness_rad_per_m{2.50};
  double max_turn_angle_rad{1.20};
  double max_route_deviation_m{0.45};
  double corner_cut_turn_angle_rad{0.75};
  double min_corner_target_distance_m{0.60};
};

struct LocalPathResult
{
  bool success{false};
  std::string message;
  std::vector<Point3> path;
  double length_m{0.0};
  double smoothness_rad_per_m{0.0};
  double max_turn_angle_rad{0.0};
};

class LocalPathSmoother
{
public:
  explicit LocalPathSmoother(LocalPathSmootherOptions options = {});

  LocalPathResult build(
    const RoutePose2D & pose,
    const RouteProgressState & route,
    const RollingLocalMap & local_map) const;

private:
  struct LocalTarget
  {
    Point3 point;
    Point3 tangent;
  };

  static double xyDistance(const Point3 & a, const Point3 & b);
  static double distanceToSegment(const Point3 & point, const Point3 & a, const Point3 & b);
  static double distanceToRoute(const Point3 & point, const std::vector<GlobalPathPoint> & route);
  static LocalTarget selectTarget(const RouteProgressState & route, const LocalPathSmootherOptions & options);
  static Point3 bezierPoint(
    const Point3 & p0,
    const Point3 & p1,
    const Point3 & p2,
    const Point3 & p3,
    double t);
  static Point3 pointFromYaw(const RoutePose2D & pose, double distance_m, double z);
  static Point3 tangentBeforeAhead(const RouteProgressState & route);
  static double pathLength(const std::vector<Point3> & path);
  static double wrapAngle(double angle_rad);
  static std::pair<double, double> smoothnessMetrics(const std::vector<Point3> & path);
  static std::vector<Point3> pruneClosePoints(
    const std::vector<Point3> & points, double min_spacing_m);
  static std::vector<Point3> resample(const std::vector<Point3> & points, double max_spacing_m);

  LocalPathSmootherOptions options_;
};

}  // namespace tgw_planner::core
