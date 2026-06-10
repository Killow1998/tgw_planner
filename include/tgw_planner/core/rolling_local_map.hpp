#pragma once

#include <cstddef>
#include <vector>

#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/route_progress_tracker.hpp"

namespace tgw_planner::core
{

struct CollisionCheckResult
{
  bool collision_free{true};
  double collision_distance_m{0.0};
};

struct RollingLocalMapOptions
{
  double robot_radius_m{0.35};
  double inflation_radius_m{0.10};
  double time_window_s{1.50};
  std::size_t max_obstacle_points{50000};
};

class RollingLocalMap
{
public:
  explicit RollingLocalMap(RollingLocalMapOptions options = {});

  void clear();
  void setObstaclePoints(const std::vector<Point3> & points);
  void addObstaclePoints(const std::vector<Point3> & points, double stamp_s);
  void prune(double stamp_s);
  bool empty() const;
  std::size_t obstacleCount() const;
  std::vector<Point3> obstaclePoints() const;

  CollisionCheckResult checkPoint(const Point3 & point) const;
  CollisionCheckResult checkPath(const std::vector<Point3> & path) const;
  CollisionCheckResult checkArc(
    const RoutePose2D & pose,
    double linear_speed_mps,
    double yaw_rate_radps,
    double time_horizon_s,
    double sample_time_s) const;

private:
  struct ObstaclePoint
  {
    Point3 point;
    double stamp_s{0.0};
  };

  RollingLocalMapOptions options_;
  std::vector<ObstaclePoint> obstacle_points_;
};

}  // namespace tgw_planner::core
