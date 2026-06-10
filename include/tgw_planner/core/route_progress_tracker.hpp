#pragma once

#include <string>
#include <vector>

#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{

struct RoutePose2D
{
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

struct RouteProgressTrackerOptions
{
  double projection_window_m{5.0};
  double local_route_length_m{4.0};
  double min_local_route_length_m{0.8};
  double goal_tolerance_m{0.35};
};

struct RouteProgressState
{
  bool valid{false};
  bool goal_reached{false};
  std::string status;
  double progress_m{0.0};
  double remaining_m{0.0};
  double lateral_error_m{0.0};
  GlobalPathPoint projected_point;
  GlobalPathPoint ahead_point;
  std::vector<GlobalPathPoint> local_route;
};

class RouteProgressTracker
{
public:
  explicit RouteProgressTracker(RouteProgressTrackerOptions options = {});

  bool setPath(const std::vector<GlobalPathPoint> & path, std::string * error = nullptr);
  void resetProgress(double progress_m = 0.0);
  bool empty() const;
  double pathLength() const;
  double progress() const;

  RouteProgressState update(const RoutePose2D & pose);

private:
  struct Projection
  {
    double progress_m{0.0};
    double distance_m{0.0};
  };

  static double xyDistance(const Point3 & a, const Point3 & b);
  static double wrapAngle(double angle_rad);
  static bool samePosition(const Point3 & a, const Point3 & b);

  GlobalPathPoint interpolate(double progress_m) const;
  Projection projectPose(const RoutePose2D & pose, double min_progress_m, double max_progress_m) const;
  std::vector<GlobalPathPoint> extractLocalRoute(double start_progress_m, double end_progress_m) const;
  void buildCumulativeLengths();

  RouteProgressTrackerOptions options_;
  std::vector<GlobalPathPoint> path_;
  std::vector<double> cumulative_length_m_;
  double progress_m_{0.0};
};

}  // namespace tgw_planner::core
