#include "tgw_planner/core/global_path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{
namespace
{
constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

bool samePosition(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y) < 1.0e-6 && std::abs(a.z - b.z) < 1.0e-6;
}
}  // namespace

GlobalPathTracker::GlobalPathTracker(GlobalPathTrackerOptions options)
: options_(options)
{
  options_.lookahead_m = std::max(0.05, options_.lookahead_m);
  options_.projection_window_m = std::max(options_.lookahead_m, options_.projection_window_m);
  options_.max_yaw_rate_radps = std::max(0.05, options_.max_yaw_rate_radps);
  options_.goal_tolerance_m = std::max(0.01, options_.goal_tolerance_m);
  options_.min_target_speed_mps = std::max(0.0, options_.min_target_speed_mps);
}

bool GlobalPathTracker::setPath(const std::vector<GlobalPathPoint> & path, std::string * error)
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

void GlobalPathTracker::resetProgress(double progress_m)
{
  if (cumulative_length_m_.empty()) {
    progress_m_ = 0.0;
    return;
  }
  progress_m_ = std::clamp(progress_m, 0.0, cumulative_length_m_.back());
}

bool GlobalPathTracker::empty() const
{
  return path_.empty();
}

double GlobalPathTracker::pathLength() const
{
  return cumulative_length_m_.empty() ? 0.0 : cumulative_length_m_.back();
}

double GlobalPathTracker::progress() const
{
  return progress_m_;
}

GlobalPathTrackerCommand GlobalPathTracker::computeCommand(const TrackerPose2D & pose, double dt_s)
{
  GlobalPathTrackerCommand command;
  if (path_.size() < 2U || cumulative_length_m_.size() != path_.size()) {
    command.status = "tracker_path_not_set";
    return command;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw_rad)) {
    command.status = "pose_non_finite";
    return command;
  }

  const double path_length = pathLength();
  const double max_projection_progress =
    std::min(path_length, progress_m_ + options_.projection_window_m);
  const Projection projection = projectPose(pose, progress_m_, max_projection_progress);
  progress_m_ = std::max(progress_m_, projection.progress_m);

  const GlobalPathPoint projected = interpolate(progress_m_);
  const GlobalPathPoint lookahead =
    interpolate(std::min(path_length, progress_m_ + options_.lookahead_m));
  const double dx = lookahead.position.x - pose.x;
  const double dy = lookahead.position.y - pose.y;
  const double desired_yaw = std::hypot(dx, dy) > kEpsilon ?
    std::atan2(dy, dx) : pose.yaw_rad;
  const double yaw_error = wrapAngle(desired_yaw - pose.yaw_rad);
  const double max_yaw_step = options_.max_yaw_rate_radps;
  const double yaw_rate = std::clamp(yaw_error / std::max(dt_s, 1.0e-3), -max_yaw_step, max_yaw_step);
  const double final_error = std::hypot(
    pose.x - path_.back().position.x,
    pose.y - path_.back().position.y);

  command.valid = true;
  command.goal_reached =
    final_error <= options_.goal_tolerance_m &&
    path_length - progress_m_ <= options_.goal_tolerance_m;
  command.status = command.goal_reached ? "goal_reached" : "tracking";
  command.linear_speed_mps =
    command.goal_reached ? 0.0 : targetSpeedForPoint(projected, options_);
  command.yaw_rate_radps = command.goal_reached ? 0.0 : yaw_rate;
  command.progress_m = progress_m_;
  command.remaining_m = std::max(0.0, path_length - progress_m_);
  command.lateral_error_m = projection.distance_m;
  command.lookahead_point = lookahead.position;
  command.projected_point = projected.position;
  command.expected_surface_z_m = projected.position.z;
  command.segment_kind = projected.kind;
  command.confidence = projected.confidence;
  return command;
}

double GlobalPathTracker::xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double GlobalPathTracker::wrapAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double GlobalPathTracker::targetSpeedForPoint(
  const GlobalPathPoint & point, const GlobalPathTrackerOptions & options)
{
  if (point.target_speed_mps > 0.0 && std::isfinite(point.target_speed_mps)) {
    return std::max(options.min_target_speed_mps, point.target_speed_mps);
  }
  switch (point.kind) {
    case PathPointKind::Backbone:
      return std::max(options.min_target_speed_mps, options.default_backbone_speed_mps);
    case PathPointKind::Portal:
      return std::max(options.min_target_speed_mps, options.default_portal_speed_mps);
    case PathPointKind::Surface:
    case PathPointKind::Unknown:
    default:
      return std::max(options.min_target_speed_mps, options.default_surface_speed_mps);
  }
}

GlobalPathPoint GlobalPathTracker::interpolate(double progress_m) const
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

GlobalPathTracker::Projection GlobalPathTracker::projectPose(
  const TrackerPose2D & pose, double min_progress_m, double max_progress_m) const
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
      best.distance_m = distance;
      best.progress_m = progress;
    }
  }

  if (!std::isfinite(best.distance_m)) {
    best.distance_m = 0.0;
  }
  return best;
}

void GlobalPathTracker::buildCumulativeLengths()
{
  cumulative_length_m_.assign(path_.size(), 0.0);
  for (std::size_t i = 1U; i < path_.size(); ++i) {
    cumulative_length_m_[i] =
      cumulative_length_m_[i - 1U] + xyDistance(path_[i - 1U].position, path_[i].position);
  }
}

}  // namespace tgw_planner::core
