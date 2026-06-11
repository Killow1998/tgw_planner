#include "tgw_planner/core/regulated_pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;
}

RegulatedPurePursuitController::RegulatedPurePursuitController(
  RegulatedPurePursuitOptions options)
: options_(options)
{
  options_.desired_linear_speed_mps = std::max(0.0, options_.desired_linear_speed_mps);
  options_.max_linear_speed_mps =
    std::max(options_.min_linear_speed_mps, options_.max_linear_speed_mps);
  options_.min_linear_speed_mps = std::max(0.0, options_.min_linear_speed_mps);
  options_.max_angular_speed_radps = std::max(0.05, options_.max_angular_speed_radps);
  options_.lookahead_m = std::max(0.05, options_.lookahead_m);
  options_.max_lateral_accel_mps2 = std::max(0.05, options_.max_lateral_accel_mps2);
  options_.goal_tolerance_m = std::max(0.01, options_.goal_tolerance_m);
  options_.goal_slowdown_distance_m = std::max(0.05, options_.goal_slowdown_distance_m);
  options_.collision_time_horizon_s = std::max(0.0, options_.collision_time_horizon_s);
  options_.collision_sample_time_s = std::max(0.02, options_.collision_sample_time_s);
  options_.max_linear_accel_mps2 = std::max(0.0, options_.max_linear_accel_mps2);
}

void RegulatedPurePursuitController::reset(double current_linear_speed_mps)
{
  previous_linear_speed_mps_ = std::max(0.0, current_linear_speed_mps);
}

RegulatedPurePursuitCommand RegulatedPurePursuitController::computeCommand(
  const RoutePose2D & pose,
  const std::vector<Point3> & local_path,
  double remaining_global_path_m,
  double dt_s,
  const RollingLocalMap & local_map)
{
  RegulatedPurePursuitCommand command;
  if (local_path.size() < 2U) {
    command.status = "local_path_too_short";
    previous_linear_speed_mps_ = 0.0;
    return command;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_rad)) {
    command.status = "pose_non_finite";
    previous_linear_speed_mps_ = 0.0;
    return command;
  }

  const double local_length = pathLength(local_path);
  const double remaining_m = std::max(0.0, remaining_global_path_m);
  if (remaining_m <= options_.goal_tolerance_m) {
    command.valid = true;
    command.goal_reached = true;
    command.status = "goal_reached";
    previous_linear_speed_mps_ = 0.0;
    return command;
  }

  const double lookahead_distance = std::min(options_.lookahead_m, local_length);
  const Point3 lookahead = interpolatePath(local_path, lookahead_distance);
  const double dx = lookahead.x - pose.x;
  const double dy = lookahead.y - pose.y;
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const double lookahead_sq = local_x * local_x + local_y * local_y;
  if (lookahead_sq <= kEpsilon) {
    command.status = "lookahead_too_close";
    previous_linear_speed_mps_ = 0.0;
    return command;
  }

  const double curvature = 2.0 * local_y / lookahead_sq;
  double linear_speed = std::min(options_.desired_linear_speed_mps, options_.max_linear_speed_mps);
  if (std::abs(curvature) > kEpsilon) {
    const double curvature_limit =
      std::sqrt(options_.max_lateral_accel_mps2 / std::abs(curvature));
    linear_speed = std::min(linear_speed, curvature_limit);
  }
  if (remaining_m < options_.goal_slowdown_distance_m) {
    const double scale = std::clamp(
      remaining_m / options_.goal_slowdown_distance_m, 0.0, 1.0);
    linear_speed = std::min(linear_speed, std::max(0.0, options_.desired_linear_speed_mps * scale));
  }

  double yaw_rate = curvature * linear_speed;
  if (std::abs(yaw_rate) > options_.max_angular_speed_radps) {
    if (std::abs(curvature) > kEpsilon) {
      linear_speed = std::min(linear_speed, options_.max_angular_speed_radps / std::abs(curvature));
    }
    yaw_rate = std::clamp(
      curvature * linear_speed, -options_.max_angular_speed_radps, options_.max_angular_speed_radps);
  }

  const double dt = std::max(1.0e-3, dt_s);
  const double max_delta_v = options_.max_linear_accel_mps2 * dt;
  linear_speed = std::clamp(
    linear_speed,
    std::max(0.0, previous_linear_speed_mps_ - max_delta_v),
    previous_linear_speed_mps_ + max_delta_v);
  if (linear_speed > 0.0) {
    linear_speed = std::max(options_.min_linear_speed_mps, linear_speed);
  }

  const CollisionCheckResult collision = local_map.checkArc(
    pose, linear_speed, yaw_rate, options_.collision_time_horizon_s,
    options_.collision_sample_time_s);
  command.collision_limited_speed_mps = linear_speed;
  if (!collision.collision_free) {
    linear_speed = 0.0;
    yaw_rate = 0.0;
    command.status = "collision_arc_blocked";
  } else {
    command.status = "tracking";
  }

  previous_linear_speed_mps_ = linear_speed;
  command.valid = true;
  command.goal_reached = false;
  command.linear_speed_mps = linear_speed;
  command.yaw_rate_radps = yaw_rate;
  command.curvature = curvature;
  command.remaining_m = remaining_m;
  command.lookahead_distance_m = lookahead_distance;
  command.lookahead_point = lookahead;
  return command;
}

double RegulatedPurePursuitController::xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double RegulatedPurePursuitController::wrapAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double RegulatedPurePursuitController::pathLength(const std::vector<Point3> & path)
{
  double length = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length += xyDistance(path[i - 1U], path[i]);
  }
  return length;
}

Point3 RegulatedPurePursuitController::interpolatePath(
  const std::vector<Point3> & path, double distance_m)
{
  if (path.empty()) {
    return {};
  }
  if (distance_m <= 0.0) {
    return path.front();
  }
  double walked = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const Point3 & a = path[i - 1U];
    const Point3 & b = path[i];
    const double segment = xyDistance(a, b);
    if (segment <= kEpsilon) {
      continue;
    }
    if (walked + segment >= distance_m) {
      const double t = (distance_m - walked) / segment;
      return {
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z)};
    }
    walked += segment;
  }
  return path.back();
}

}  // namespace tgw_planner::core
