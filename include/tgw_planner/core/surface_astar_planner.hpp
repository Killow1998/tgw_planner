#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/robot_footprint.hpp"

namespace tgw_planner::core
{

struct SurfacePlannerOptions
{
  double w_clearance{0.8};
  double w_slope{0.3};
  double w_turn{0.1};
  double w_unknown{2.0};
  std::uint32_t max_iterations{250000};
  bool require_footprint_support{true};
  double swept_sample_step_m{0.05};
  RobotFootprintOptions footprint;
};

struct SurfacePlanMetrics
{
  bool success{false};
  std::string failure_reason;
  std::uint32_t expanded_nodes{0};
  std::uint32_t generated_nodes{0};
  double path_length_m{0.0};
  double min_path_clearance_m{0.0};
  double mean_path_clearance_m{0.0};
  double clearance_cost_sum{0.0};
  std::uint32_t low_clearance_samples{0};
};

struct SurfacePlanResult
{
  bool success{false};
  std::string message;
  std::vector<GridIndex> cells;
  std::vector<Point3> path;
  SurfacePlanMetrics metrics;
};

class SurfaceAstarPlanner
{
public:
  explicit SurfaceAstarPlanner(SurfacePlannerOptions options = {});

  SurfacePlanResult plan(
    const NavigationSnapshot & snapshot, const GridIndex & start, const GridIndex & goal) const;

private:
  double transitionCost(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to,
    const GridIndex * previous) const;
  bool isCellTraversable(const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  bool isFootprintSupported(
    const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const;
  bool isTransitionAllowed(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const;
  Point3 cellCenter(const GridIndex & cell, double resolution_m) const;
  GridIndex worldToGrid(const Point3 & point, double resolution_m) const;
  void fillMetrics(const NavigationSnapshot & snapshot, SurfacePlanResult & result) const;

  SurfacePlannerOptions options_;
  RobotFootprint footprint_;
};

}  // namespace tgw_planner::core
