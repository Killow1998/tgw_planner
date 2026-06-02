#include "tgw_planner/core/path_validator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tgw_planner::core
{

PathValidator::PathValidator(RobotFootprint footprint, PathValidationOptions options)
: footprint_(std::move(footprint)), options_(options)
{
  options_.sample_step_m = std::max(0.01, options_.sample_step_m);
  options_.min_clearance_m = std::max(0.0, options_.min_clearance_m);
}

PathValidationReport PathValidator::validate(
  const NavigationSnapshot & snapshot, const std::vector<Point3> & path) const
{
  PathValidationReport report;
  if (path.empty()) {
    report.failure_reason = "final path is empty";
    return report;
  }

  double clearance_sum = 0.0;
  report.min_clearance_m = std::numeric_limits<double>::infinity();

  if (path.size() == 1U) {
    if (!validateSample(snapshot, path.front(), 0.0, report, clearance_sum)) {
      return report;
    }
    report.valid = true;
    report.mean_clearance_m = clearance_sum / static_cast<double>(report.checked_samples);
    return report;
  }

  for (std::size_t i = 1; i < path.size(); ++i) {
    const Point3 & from = path[i - 1U];
    const Point3 & to = path[i];
    const double segment_length = distance3d(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / options_.sample_step_m)));
    const double yaw = std::atan2(to.y - from.y, to.x - from.x);
    for (int step = 0; step <= steps; ++step) {
      if (i > 1U && step == 0) {
        continue;
      }
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const Point3 sample{
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t,
        from.z + (to.z - from.z) * t};
      if (!validateSample(snapshot, sample, yaw, report, clearance_sum)) {
        return report;
      }
    }
  }

  report.valid = true;
  if (report.checked_samples > 0U) {
    report.mean_clearance_m = clearance_sum / static_cast<double>(report.checked_samples);
  }
  if (!std::isfinite(report.min_clearance_m)) {
    report.min_clearance_m = 0.0;
  }
  return report;
}

GridIndex PathValidator::worldToGrid(const Point3 & point, double resolution_m) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

bool PathValidator::validateSample(
  const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad,
  PathValidationReport & report, double & clearance_sum) const
{
  const GridIndex cell = worldToGrid(point, snapshot.resolution_m);
  if (snapshot.surface.traversable_cells.find(cell) == snapshot.surface.traversable_cells.end()) {
    report.failure_reason = "final path sample is not traversable";
    return false;
  }
  if (snapshot.surface.forbidden_cells.find(cell) != snapshot.surface.forbidden_cells.end()) {
    report.failure_reason = "final path sample is forbidden";
    return false;
  }
  if (snapshot.surface.blocked_cells.find(cell) != snapshot.surface.blocked_cells.end()) {
    report.failure_reason = "final path sample is blocked";
    return false;
  }
  if (options_.require_footprint_support && !footprintSupported(snapshot, point, yaw_rad)) {
    report.failure_reason = "final path footprint is not fully supported";
    return false;
  }

  const double clearance = snapshot.clearance.clearanceDistance(cell);
  report.min_clearance_m = std::min(report.min_clearance_m, clearance);
  clearance_sum += clearance;
  ++report.checked_samples;
  if (clearance < options_.min_clearance_m) {
    ++report.low_clearance_samples;
    report.failure_reason = "final path clearance below minimum";
    return false;
  }
  return true;
}

bool PathValidator::footprintSupported(
  const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const
{
  for (const Point3 & sample :
    footprint_.sampleFootprint(point, yaw_rad, snapshot.resolution_m))
  {
    const GridIndex cell = worldToGrid(sample, snapshot.resolution_m);
    if (snapshot.surface.traversable_cells.find(cell) == snapshot.surface.traversable_cells.end()) {
      return false;
    }
    if (snapshot.surface.forbidden_cells.find(cell) != snapshot.surface.forbidden_cells.end()) {
      return false;
    }
    if (snapshot.surface.blocked_cells.find(cell) != snapshot.surface.blocked_cells.end()) {
      return false;
    }
  }
  return true;
}

}  // namespace tgw_planner::core
