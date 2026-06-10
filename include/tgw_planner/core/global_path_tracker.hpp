#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{

struct TrackerPose2D
{
  double x{0.0};
  double y{0.0};
  double yaw_rad{0.0};
};

struct GlobalPathTrackerOptions
{
  double lookahead_m{0.80};
  double projection_window_m{5.0};
  double max_yaw_rate_radps{1.20};
  double goal_tolerance_m{0.35};
  double min_target_speed_mps{0.02};
  double default_surface_speed_mps{0.60};
  double default_backbone_speed_mps{0.30};
  double default_portal_speed_mps{0.15};
};

struct GlobalPathTrackerCommand
{
  bool valid{false};
  bool goal_reached{false};
  std::string status;
  double linear_speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double progress_m{0.0};
  double remaining_m{0.0};
  double lateral_error_m{0.0};
  Point3 lookahead_point;
  Point3 projected_point;
  double expected_surface_z_m{0.0};
  PathPointKind segment_kind{PathPointKind::Unknown};
  double confidence{1.0};
};

class GlobalPathTracker
{
public:
  explicit GlobalPathTracker(GlobalPathTrackerOptions options = {});

  bool setPath(const std::vector<GlobalPathPoint> & path, std::string * error = nullptr);
  void resetProgress(double progress_m = 0.0);
  bool empty() const;
  double pathLength() const;
  double progress() const;

  GlobalPathTrackerCommand computeCommand(const TrackerPose2D & pose, double dt_s);

private:
  struct Projection
  {
    double progress_m{0.0};
    double distance_m{0.0};
  };

  static double xyDistance(const Point3 & a, const Point3 & b);
  static double wrapAngle(double angle_rad);
  static double targetSpeedForPoint(
    const GlobalPathPoint & point, const GlobalPathTrackerOptions & options);

  GlobalPathPoint interpolate(double progress_m) const;
  Projection projectPose(const TrackerPose2D & pose, double min_progress_m, double max_progress_m) const;
  void buildCumulativeLengths();

  GlobalPathTrackerOptions options_;
  std::vector<GlobalPathPoint> path_;
  std::vector<double> cumulative_length_m_;
  double progress_m_{0.0};
};

}  // namespace tgw_planner::core
