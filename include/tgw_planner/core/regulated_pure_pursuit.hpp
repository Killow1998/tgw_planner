#pragma once

#include <string>
#include <vector>

#include "tgw_planner/core/rolling_local_map.hpp"
#include "tgw_planner/core/route_progress_tracker.hpp"

namespace tgw_planner::core
{

struct RegulatedPurePursuitOptions
{
  double desired_linear_speed_mps{0.60};
  double max_linear_speed_mps{0.80};
  double min_linear_speed_mps{0.02};
  double max_angular_speed_radps{1.20};
  double lookahead_m{0.80};
  double max_lateral_accel_mps2{0.80};
  double goal_slowdown_distance_m{1.00};
  double collision_time_horizon_s{1.50};
  double collision_sample_time_s{0.10};
  double max_linear_accel_mps2{0.80};
};

struct RegulatedPurePursuitCommand
{
  bool valid{false};
  bool goal_reached{false};
  std::string status;
  double linear_speed_mps{0.0};
  double yaw_rate_radps{0.0};
  double curvature{0.0};
  double remaining_m{0.0};
  double lookahead_distance_m{0.0};
  double collision_limited_speed_mps{0.0};
  Point3 lookahead_point;
};

class RegulatedPurePursuitController
{
public:
  explicit RegulatedPurePursuitController(RegulatedPurePursuitOptions options = {});

  void reset(double current_linear_speed_mps = 0.0);

  RegulatedPurePursuitCommand computeCommand(
    const RoutePose2D & pose,
    const std::vector<Point3> & local_path,
    double remaining_global_path_m,
    double dt_s,
    const RollingLocalMap & local_map);

private:
  static double xyDistance(const Point3 & a, const Point3 & b);
  static double wrapAngle(double angle_rad);
  static double pathLength(const std::vector<Point3> & path);
  static Point3 interpolatePath(const std::vector<Point3> & path, double distance_m);

  RegulatedPurePursuitOptions options_;
  double previous_linear_speed_mps_{0.0};
};

}  // namespace tgw_planner::core
