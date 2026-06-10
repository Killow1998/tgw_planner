#include "tgw_planner/core/rolling_local_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{

RollingLocalMap::RollingLocalMap(RollingLocalMapOptions options)
: options_(options)
{
  options_.robot_radius_m = std::max(0.0, options_.robot_radius_m);
  options_.inflation_radius_m = std::max(0.0, options_.inflation_radius_m);
  options_.time_window_s = std::max(0.0, options_.time_window_s);
  options_.max_obstacle_points = std::max<std::size_t>(1U, options_.max_obstacle_points);
}

void RollingLocalMap::clear()
{
  obstacle_points_.clear();
}

void RollingLocalMap::setObstaclePoints(const std::vector<Point3> & points)
{
  obstacle_points_.clear();
  addObstaclePoints(points, 0.0);
}

void RollingLocalMap::addObstaclePoints(const std::vector<Point3> & points, double stamp_s)
{
  if (!std::isfinite(stamp_s)) {
    stamp_s = 0.0;
  }
  obstacle_points_.reserve(
    std::min(options_.max_obstacle_points, obstacle_points_.size() + points.size()));
  for (const Point3 & point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      obstacle_points_.push_back({point, stamp_s});
    }
  }
  prune(stamp_s);
  if (obstacle_points_.size() > options_.max_obstacle_points) {
    const std::size_t extra = obstacle_points_.size() - options_.max_obstacle_points;
    obstacle_points_.erase(obstacle_points_.begin(), obstacle_points_.begin() + extra);
  }
}

void RollingLocalMap::prune(double stamp_s)
{
  if (!std::isfinite(stamp_s) || options_.time_window_s <= 0.0) {
    return;
  }
  const double oldest = stamp_s - options_.time_window_s;
  obstacle_points_.erase(
    std::remove_if(
      obstacle_points_.begin(),
      obstacle_points_.end(),
      [oldest](const ObstaclePoint & point) {
        return point.stamp_s < oldest;
      }),
    obstacle_points_.end());
}

bool RollingLocalMap::empty() const
{
  return obstacle_points_.empty();
}

std::size_t RollingLocalMap::obstacleCount() const
{
  return obstacle_points_.size();
}

std::vector<Point3> RollingLocalMap::obstaclePoints() const
{
  std::vector<Point3> points;
  points.reserve(obstacle_points_.size());
  for (const ObstaclePoint & obstacle : obstacle_points_) {
    points.push_back(obstacle.point);
  }
  return points;
}

CollisionCheckResult RollingLocalMap::checkPoint(const Point3 & point) const
{
  CollisionCheckResult result;
  const double clearance_radius = options_.robot_radius_m + options_.inflation_radius_m;
  const double clearance_sq = clearance_radius * clearance_radius;
  for (const ObstaclePoint & obstacle : obstacle_points_) {
    const double dx = point.x - obstacle.point.x;
    const double dy = point.y - obstacle.point.y;
    if (dx * dx + dy * dy <= clearance_sq) {
      result.collision_free = false;
      result.collision_distance_m = std::hypot(dx, dy);
      return result;
    }
  }
  return result;
}

CollisionCheckResult RollingLocalMap::checkPath(const std::vector<Point3> & path) const
{
  for (const Point3 & point : path) {
    const CollisionCheckResult result = checkPoint(point);
    if (!result.collision_free) {
      return result;
    }
  }
  return {};
}

CollisionCheckResult RollingLocalMap::checkArc(
  const RoutePose2D & pose,
  double linear_speed_mps,
  double yaw_rate_radps,
  double time_horizon_s,
  double sample_time_s) const
{
  if (empty()) {
    return {};
  }
  RoutePose2D sample_pose = pose;
  const double dt = std::max(0.02, sample_time_s);
  const int samples = std::max(1, static_cast<int>(std::ceil(std::max(0.0, time_horizon_s) / dt)));
  for (int i = 0; i < samples; ++i) {
    sample_pose.yaw_rad += yaw_rate_radps * dt;
    sample_pose.x += linear_speed_mps * dt * std::cos(sample_pose.yaw_rad);
    sample_pose.y += linear_speed_mps * dt * std::sin(sample_pose.yaw_rad);
    const CollisionCheckResult result = checkPoint({sample_pose.x, sample_pose.y, 0.0});
    if (!result.collision_free) {
      return result;
    }
  }
  return {};
}

}  // namespace tgw_planner::core
