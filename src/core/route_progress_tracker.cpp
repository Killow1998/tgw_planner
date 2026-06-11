#include "tgw_planner/core/route_progress_tracker.hpp"

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

RouteProgressTracker::RouteProgressTracker(RouteProgressTrackerOptions options)
: options_(options)
{
  options_.projection_window_m = std::max(0.20, options_.projection_window_m);
  options_.max_projection_lateral_error_m =
    std::max(0.05, options_.max_projection_lateral_error_m);
  options_.local_route_length_m = std::max(0.20, options_.local_route_length_m);
  options_.min_local_route_length_m = std::max(0.05, options_.min_local_route_length_m);
  options_.goal_tolerance_m = std::max(0.01, options_.goal_tolerance_m);
}

bool RouteProgressTracker::setPath(const std::vector<GlobalPathPoint> & path, std::string * error)
{
  path_.clear();
  cumulative_length_m_.clear();
  progress_m_ = 0.0;

  for (const GlobalPathPoint & point : path) {
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
      !std::isfinite(point.position.z))
    {
      if (error) {
        *error = "path_contains_non_finite_point";
      }
      return false;
    }
    if (!path_.empty() && samePosition(path_.back().position, point.position)) {
      if (path_.back().kind != PathPointKind::Portal && point.kind == PathPointKind::Portal) {
        path_.back() = point;
      }
      continue;
    }
    path_.push_back(point);
  }

  if (path_.size() < 2U) {
    if (error) {
      *error = "path_too_short";
    }
    path_.clear();
    return false;
  }

  buildCumulativeLengths();
  if (cumulative_length_m_.empty() || cumulative_length_m_.back() <= kEpsilon) {
    if (error) {
      *error = "path_zero_length";
    }
    path_.clear();
    cumulative_length_m_.clear();
    return false;
  }
  return true;
}

void RouteProgressTracker::resetProgress(double progress_m)
{
  if (cumulative_length_m_.empty()) {
    progress_m_ = 0.0;
    return;
  }
  progress_m_ = std::clamp(progress_m, 0.0, cumulative_length_m_.back());
}

bool RouteProgressTracker::empty() const
{
  return path_.empty();
}

double RouteProgressTracker::pathLength() const
{
  return cumulative_length_m_.empty() ? 0.0 : cumulative_length_m_.back();
}

double RouteProgressTracker::progress() const
{
  return progress_m_;
}

RouteProgressState RouteProgressTracker::update(const RoutePose2D & pose)
{
  RouteProgressState state;
  if (path_.size() < 2U || cumulative_length_m_.size() != path_.size()) {
    state.status = "route_path_not_set";
    return state;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_rad)) {
    state.status = "pose_non_finite";
    return state;
  }

  const double length = pathLength();
  const double max_projection = std::min(length, progress_m_ + options_.projection_window_m);
  const Projection projection = projectPose(pose, progress_m_, max_projection);
  if (!projection.found) {
    state.status = "route_projection_failed";
    state.progress_m = progress_m_;
    state.remaining_m = std::max(0.0, length - progress_m_);
    state.lateral_error_m = std::isfinite(projection.distance_m) ? projection.distance_m : -1.0;
    state.projected_point = interpolate(progress_m_);
    state.ahead_point = state.projected_point;
    return state;
  }
  progress_m_ = std::max(progress_m_, projection.progress_m);

  const double end_progress = std::min(length, progress_m_ + options_.local_route_length_m);
  state.projected_point = interpolate(progress_m_);
  state.ahead_point = interpolate(end_progress);
  state.local_route = extractLocalRoute(progress_m_, end_progress);
  state.progress_m = progress_m_;
  state.remaining_m = std::max(0.0, length - progress_m_);
  state.lateral_error_m = projection.distance_m;

  const double final_error = std::hypot(pose.x - path_.back().position.x, pose.y - path_.back().position.y);
  state.goal_reached = final_error <= options_.goal_tolerance_m &&
    state.remaining_m <= options_.goal_tolerance_m;
  state.valid = !state.local_route.empty();
  state.status = state.goal_reached ? "goal_reached" : "tracking";
  return state;
}

double RouteProgressTracker::xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double RouteProgressTracker::wrapAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

bool RouteProgressTracker::samePosition(const Point3 & a, const Point3 & b)
{
  return xyDistance(a, b) < 1.0e-6 && std::abs(a.z - b.z) < 1.0e-6;
}

GlobalPathPoint RouteProgressTracker::interpolate(double progress_m) const
{
  if (progress_m <= 0.0) {
    return path_.front();
  }
  if (progress_m >= pathLength()) {
    return path_.back();
  }
  const auto upper = std::upper_bound(
    cumulative_length_m_.begin(), cumulative_length_m_.end(), progress_m);
  const std::size_t next_index = static_cast<std::size_t>(
    std::distance(cumulative_length_m_.begin(), upper));
  const std::size_t prev_index = next_index > 0U ? next_index - 1U : 0U;
  if (next_index >= path_.size()) {
    return path_.back();
  }

  const double segment_length =
    cumulative_length_m_[next_index] - cumulative_length_m_[prev_index];
  const double t = segment_length > kEpsilon ?
    (progress_m - cumulative_length_m_[prev_index]) / segment_length : 0.0;
  const GlobalPathPoint & a = path_[prev_index];
  const GlobalPathPoint & b = path_[next_index];

  GlobalPathPoint out;
  out.position = {
    a.position.x + t * (b.position.x - a.position.x),
    a.position.y + t * (b.position.y - a.position.y),
    a.position.z + t * (b.position.z - a.position.z)};
  out.yaw_hint_rad = a.yaw_hint_rad + t * wrapAngle(b.yaw_hint_rad - a.yaw_hint_rad);
  out.kind = t > 0.5 ? b.kind : a.kind;
  out.target_speed_mps = a.target_speed_mps + t * (b.target_speed_mps - a.target_speed_mps);
  out.confidence = a.confidence + t * (b.confidence - a.confidence);
  out.surface_component_id = t > 0.5 ? b.surface_component_id : a.surface_component_id;
  return out;
}

RouteProgressTracker::Projection RouteProgressTracker::projectPose(
  const RoutePose2D & pose, double min_progress_m, double max_progress_m) const
{
  Projection best;
  best.progress_m = min_progress_m;
  best.distance_m = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0U; i + 1U < path_.size(); ++i) {
    if (cumulative_length_m_[i] > max_progress_m ||
      cumulative_length_m_[i + 1U] < min_progress_m)
    {
      continue;
    }
    const Point3 & a = path_[i].position;
    const Point3 & b = path_[i + 1U].position;
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double segment_length_sq = vx * vx + vy * vy;
    if (segment_length_sq <= kEpsilon) {
      continue;
    }
    double t = ((pose.x - a.x) * vx + (pose.y - a.y) * vy) / segment_length_sq;
    t = std::clamp(t, 0.0, 1.0);
    const double progress =
      cumulative_length_m_[i] + t * (cumulative_length_m_[i + 1U] - cumulative_length_m_[i]);
    if (progress + 1.0e-6 < min_progress_m || progress > max_progress_m + 1.0e-6) {
      continue;
    }
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double distance = std::hypot(pose.x - px, pose.y - py);
    if (distance < best.distance_m) {
      best.found = true;
      best.distance_m = distance;
      best.progress_m = progress;
    }
  }

  if (!best.found || best.distance_m > options_.max_projection_lateral_error_m) {
    best.found = false;
  }
  return best;
}

std::vector<GlobalPathPoint> RouteProgressTracker::extractLocalRoute(
  double start_progress_m, double end_progress_m) const
{
  std::vector<GlobalPathPoint> route;
  if (path_.empty()) {
    return route;
  }
  route.push_back(interpolate(start_progress_m));
  for (std::size_t i = 1U; i + 1U < path_.size(); ++i) {
    if (cumulative_length_m_[i] > start_progress_m + 1.0e-6 &&
      cumulative_length_m_[i] < end_progress_m - 1.0e-6)
    {
      route.push_back(path_[i]);
    }
  }
  if (end_progress_m - start_progress_m >= options_.min_local_route_length_m ||
    std::abs(end_progress_m - pathLength()) < 1.0e-6)
  {
    route.push_back(interpolate(end_progress_m));
  }
  return route;
}

void RouteProgressTracker::buildCumulativeLengths()
{
  cumulative_length_m_.assign(path_.size(), 0.0);
  for (std::size_t i = 1U; i < path_.size(); ++i) {
    cumulative_length_m_[i] =
      cumulative_length_m_[i - 1U] + xyDistance(path_[i - 1U].position, path_[i].position);
  }
}

}  // namespace tgw_planner::core
