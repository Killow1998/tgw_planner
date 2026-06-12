#include "tgw_planner/core/local_path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tgw_planner::core
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;
}

LocalPathSmoother::LocalPathSmoother(LocalPathSmootherOptions options)
: options_(options)
{
  options_.min_point_spacing_m = std::max(0.01, options_.min_point_spacing_m);
  options_.max_point_spacing_m = std::max(options_.min_point_spacing_m, options_.max_point_spacing_m);
  options_.bezier_handle_ratio = std::clamp(options_.bezier_handle_ratio, 0.05, 0.80);
  options_.max_smoothness_rad_per_m = std::max(0.10, options_.max_smoothness_rad_per_m);
  options_.max_turn_angle_rad = std::max(0.10, options_.max_turn_angle_rad);
  options_.max_route_deviation_m = std::max(0.05, options_.max_route_deviation_m);
  options_.corner_cut_turn_angle_rad = std::clamp(options_.corner_cut_turn_angle_rad, 0.10, kPi);
  options_.min_corner_target_distance_m = std::max(0.05, options_.min_corner_target_distance_m);
}

LocalPathResult LocalPathSmoother::build(
  const RoutePose2D & pose,
  const RouteProgressState & route,
  const RollingLocalMap & local_map) const
{
  LocalPathResult result;
  if (!route.valid || route.local_route.size() < 2U) {
    result.message = "route_window_not_available";
    return result;
  }

  const Point3 start{pose.x, pose.y, route.projected_point.position.z};
  const LocalTarget target = selectTarget(route, options_);
  const Point3 goal = target.point;
  const double direct_distance = xyDistance(start, goal);
  if (direct_distance < options_.min_point_spacing_m) {
    result.message = "ahead_point_too_close";
    return result;
  }

  const double handle = std::max(options_.min_point_spacing_m, direct_distance * options_.bezier_handle_ratio);
  const Point3 start_handle = pointFromYaw(pose, handle, start.z);
  const Point3 ahead_tangent = target.tangent;
  const double tangent_norm = std::hypot(ahead_tangent.x, ahead_tangent.y);
  Point3 end_handle;
  if (tangent_norm > kEpsilon) {
    end_handle = {
      goal.x - handle * ahead_tangent.x / tangent_norm,
      goal.y - handle * ahead_tangent.y / tangent_norm,
      goal.z};
  } else {
    const double dx = goal.x - start.x;
    const double dy = goal.y - start.y;
    const double norm = std::max(kEpsilon, std::hypot(dx, dy));
    end_handle = {
      goal.x - handle * dx / norm,
      goal.y - handle * dy / norm,
      goal.z};
  }

  const int bezier_steps = std::max(6, static_cast<int>(std::ceil(direct_distance / options_.max_point_spacing_m)));
  std::vector<Point3> smoothed;
  smoothed.reserve(static_cast<std::size_t>(bezier_steps + 1));
  for (int i = 0; i <= bezier_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(bezier_steps);
    smoothed.push_back(bezierPoint(start, start_handle, end_handle, goal, t));
  }
  smoothed = pruneClosePoints(smoothed, options_.min_point_spacing_m);
  smoothed = resample(smoothed, options_.max_point_spacing_m);
  if (smoothed.size() < 2U) {
    result.message = "smoothed_path_too_short";
    return result;
  }
  const double allowed_route_deviation =
    std::max(options_.max_route_deviation_m, route.lateral_error_m + 0.20);
  for (const Point3 & point : smoothed) {
    if (distanceToRoute(point, route.local_route) > allowed_route_deviation) {
      result.path = std::move(smoothed);
      result.length_m = pathLength(result.path);
      const auto [smoothness, max_turn] = smoothnessMetrics(result.path);
      result.smoothness_rad_per_m = smoothness;
      result.max_turn_angle_rad = max_turn;
      return buildRouteFollowingFallback(
        route, local_map, options_, "local_path_outside_global_corridor");
    }
  }
  const auto [smoothness, max_turn] = smoothnessMetrics(smoothed);
  if (smoothness > options_.max_smoothness_rad_per_m ||
    max_turn > options_.max_turn_angle_rad)
  {
    result.path = std::move(smoothed);
    result.length_m = pathLength(result.path);
    result.smoothness_rad_per_m = smoothness;
    result.max_turn_angle_rad = max_turn;
    return buildRouteFollowingFallback(route, local_map, options_, "local_path_too_curvy");
  }

  if (options_.enable_collision_check) {
    const CollisionCheckResult collision = local_map.checkPath(smoothed);
    if (!collision.collision_free) {
      result.message = "local_path_collision";
      return result;
    }
  }

  result.success = true;
  result.message = "ok";
  result.path = std::move(smoothed);
  result.length_m = pathLength(result.path);
  result.smoothness_rad_per_m = smoothness;
  result.max_turn_angle_rad = max_turn;
  return result;
}

LocalPathResult LocalPathSmoother::buildRouteFollowingFallback(
  const RouteProgressState & route,
  const RollingLocalMap & local_map,
  const LocalPathSmootherOptions & options,
  const std::string & replaced_failure)
{
  LocalPathResult result;
  if (route.local_route.size() < 2U) {
    result.message = replaced_failure + "_fallback_route_too_short";
    return result;
  }

  std::vector<Point3> fallback;
  fallback.reserve(route.local_route.size());
  for (const GlobalPathPoint & point : route.local_route) {
    fallback.push_back(point.position);
  }
  fallback = pruneClosePoints(fallback, options.min_point_spacing_m);
  fallback = resample(fallback, options.max_point_spacing_m);
  if (fallback.size() < 2U) {
    result.message = replaced_failure + "_fallback_route_too_short";
    return result;
  }

  if (options.enable_collision_check) {
    const CollisionCheckResult collision = local_map.checkPath(fallback);
    if (!collision.collision_free) {
      result.message = replaced_failure + "_fallback_route_collision";
      return result;
    }
  }

  result.success = true;
  result.message = "ok_route_following_fallback_after_" + replaced_failure;
  result.path = std::move(fallback);
  result.length_m = pathLength(result.path);
  const auto [smoothness, max_turn] = smoothnessMetrics(result.path);
  result.smoothness_rad_per_m = smoothness;
  result.max_turn_angle_rad = max_turn;
  return result;
}

double LocalPathSmoother::xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double LocalPathSmoother::distanceToSegment(
  const Point3 & point, const Point3 & a, const Point3 & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double segment_sq = vx * vx + vy * vy;
  if (segment_sq <= kEpsilon) {
    return xyDistance(point, a);
  }
  const double t = std::clamp(
    ((point.x - a.x) * vx + (point.y - a.y) * vy) / segment_sq, 0.0, 1.0);
  const Point3 projected{a.x + t * vx, a.y + t * vy, 0.0};
  return std::hypot(point.x - projected.x, point.y - projected.y);
}

double LocalPathSmoother::distanceToRoute(
  const Point3 & point, const std::vector<GlobalPathPoint> & route)
{
  if (route.empty()) {
    return 0.0;
  }
  if (route.size() == 1U) {
    return xyDistance(point, route.front().position);
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1U; i < route.size(); ++i) {
    best = std::min(best, distanceToSegment(point, route[i - 1U].position, route[i].position));
  }
  return std::isfinite(best) ? best : 0.0;
}

LocalPathSmoother::LocalTarget LocalPathSmoother::selectTarget(
  const RouteProgressState & route,
  const LocalPathSmootherOptions & options)
{
  LocalTarget target;
  target.point = route.ahead_point.position;
  target.tangent = tangentBeforeAhead(route);
  if (route.local_route.size() < 3U) {
    return target;
  }

  double distance_from_start = 0.0;
  for (std::size_t i = 1U; i + 1U < route.local_route.size(); ++i) {
    const Point3 & prev = route.local_route[i - 1U].position;
    const Point3 & corner = route.local_route[i].position;
    const Point3 & next = route.local_route[i + 1U].position;
    distance_from_start += xyDistance(prev, corner);
    if (distance_from_start < options.min_corner_target_distance_m) {
      continue;
    }
    if (xyDistance(prev, corner) <= kEpsilon || xyDistance(corner, next) <= kEpsilon) {
      continue;
    }
    const double heading_in = std::atan2(corner.y - prev.y, corner.x - prev.x);
    const double heading_out = std::atan2(next.y - corner.y, next.x - corner.x);
    const double turn = std::abs(wrapAngle(heading_out - heading_in));
    if (turn >= options.corner_cut_turn_angle_rad) {
      target.point = corner;
      target.tangent = {corner.x - prev.x, corner.y - prev.y, corner.z - prev.z};
      return target;
    }
  }
  return target;
}

Point3 LocalPathSmoother::bezierPoint(
  const Point3 & p0,
  const Point3 & p1,
  const Point3 & p2,
  const Point3 & p3,
  double t)
{
  t = std::clamp(t, 0.0, 1.0);
  const double u = 1.0 - t;
  const double b0 = u * u * u;
  const double b1 = 3.0 * u * u * t;
  const double b2 = 3.0 * u * t * t;
  const double b3 = t * t * t;
  return {
    b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
    b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y,
    b0 * p0.z + b1 * p1.z + b2 * p2.z + b3 * p3.z};
}

Point3 LocalPathSmoother::pointFromYaw(
  const RoutePose2D & pose, double distance_m, double z)
{
  return {
    pose.x + distance_m * std::cos(pose.yaw_rad),
    pose.y + distance_m * std::sin(pose.yaw_rad),
    z};
}

Point3 LocalPathSmoother::tangentBeforeAhead(const RouteProgressState & route)
{
  if (route.local_route.size() >= 2U) {
    const Point3 & last = route.local_route.back().position;
    for (std::size_t offset = 2U; offset <= route.local_route.size(); ++offset) {
      const Point3 & prev = route.local_route[route.local_route.size() - offset].position;
      if (xyDistance(prev, last) > kEpsilon) {
        return {last.x - prev.x, last.y - prev.y, last.z - prev.z};
      }
    }
  }
  return {
    route.ahead_point.position.x - route.projected_point.position.x,
    route.ahead_point.position.y - route.projected_point.position.y,
    route.ahead_point.position.z - route.projected_point.position.z};
}

double LocalPathSmoother::pathLength(const std::vector<Point3> & path)
{
  double length = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length += xyDistance(path[i - 1U], path[i]);
  }
  return length;
}

double LocalPathSmoother::wrapAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

std::pair<double, double> LocalPathSmoother::smoothnessMetrics(
  const std::vector<Point3> & path)
{
  if (path.size() < 3U) {
    return {0.0, 0.0};
  }
  double total_turn = 0.0;
  double max_turn = 0.0;
  for (std::size_t i = 1U; i + 1U < path.size(); ++i) {
    const Point3 & a = path[i - 1U];
    const Point3 & b = path[i];
    const Point3 & c = path[i + 1U];
    if (xyDistance(a, b) <= kEpsilon || xyDistance(b, c) <= kEpsilon) {
      continue;
    }
    const double heading_ab = std::atan2(b.y - a.y, b.x - a.x);
    const double heading_bc = std::atan2(c.y - b.y, c.x - b.x);
    const double turn = std::abs(wrapAngle(heading_bc - heading_ab));
    total_turn += turn;
    max_turn = std::max(max_turn, turn);
  }
  const double length = std::max(kEpsilon, pathLength(path));
  return {total_turn / length, max_turn};
}

std::vector<Point3> LocalPathSmoother::pruneClosePoints(
  const std::vector<Point3> & points, double min_spacing_m)
{
  std::vector<Point3> pruned;
  pruned.reserve(points.size());
  for (const Point3 & point : points) {
    if (pruned.empty() || xyDistance(pruned.back(), point) >= min_spacing_m) {
      pruned.push_back(point);
    } else if (&point == &points.back()) {
      pruned.back() = point;
    }
  }
  if (!points.empty() && (pruned.empty() || xyDistance(pruned.back(), points.back()) > kEpsilon)) {
    pruned.push_back(points.back());
  }
  return pruned;
}

std::vector<Point3> LocalPathSmoother::resample(
  const std::vector<Point3> & points, double max_spacing_m)
{
  if (points.size() < 2U) {
    return points;
  }
  std::vector<Point3> out;
  out.reserve(points.size());
  out.push_back(points.front());
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const Point3 a = out.back();
    const Point3 & b = points[i];
    const double distance = xyDistance(a, b);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / max_spacing_m)));
    for (int step = 1; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      out.push_back({
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z)});
    }
  }
  return out;
}

}  // namespace tgw_planner::core
