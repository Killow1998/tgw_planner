#pragma once

#include <string>
#include <vector>

#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/robot_footprint.hpp"

namespace tgw_planner::core
{

struct PathValidationOptions
{
  double sample_step_m{0.05};
  double min_clearance_m{0.0};
  double max_step_height_m{0.30};
  bool require_footprint_support{true};
};

struct PathValidationReport
{
  bool valid{false};
  std::string failure_reason;
  std::uint32_t checked_samples{0};
  double min_clearance_m{0.0};
  double mean_clearance_m{0.0};
  std::uint32_t low_clearance_samples{0};
};

class PathValidator
{
public:
  PathValidator(RobotFootprint footprint, PathValidationOptions options = {});

  PathValidationReport validate(
    const NavigationSnapshot & snapshot, const std::vector<Point3> & path) const;

private:
  GridIndex worldToGrid(const Point3 & point, double resolution_m) const;
  bool isDirectSurfaceNeighbor(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const;
  bool validateSample(
    const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad,
    PathValidationReport & report, double & clearance_sum) const;
  bool footprintSupported(
    const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const;

  RobotFootprint footprint_;
  PathValidationOptions options_;
};

}  // namespace tgw_planner::core
