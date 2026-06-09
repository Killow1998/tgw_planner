#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/robot_footprint.hpp"

namespace tgw_planner::core
{

struct SurfacePlannerOptions
{
  double w_clearance{0.8};
  double w_risk{1.5};
  double w_slope{0.3};
  double w_turn{0.1};
  double w_unknown{2.0};
  double w_bridge{2.5};
  std::uint32_t max_iterations{250000};
  bool require_footprint_support{true};
  double max_step_height_m{0.30};
  double swept_sample_step_m{0.05};
  bool enable_shortcut{true};
  double shortcut_sample_step_m{0.05};
  double shortcut_clearance_ratio{0.80};
  double shortcut_safety_margin_m{0.02};
  double final_validation_min_clearance_m{0.0};
  RobotFootprintOptions footprint;
};

struct TransitionReport
{
  bool allowed{false};
  std::string reason;
};

struct SurfacePlanMetrics
{
  bool success{false};
  std::string failure_reason;
  std::uint32_t expanded_nodes{0};
  std::uint32_t generated_nodes{0};
  double path_length_m{0.0};
  double min_path_clearance_m{0.0};
  GridIndex min_path_clearance_cell;
  bool has_min_path_clearance_cell{false};
  double mean_path_clearance_m{0.0};
  double clearance_cost_sum{0.0};
  double unknown_cost_sum{0.0};
  double risk_cost_sum{0.0};
  double max_path_risk{0.0};
  std::uint32_t low_clearance_samples{0};
  std::uint32_t raw_path_waypoints{0};
  double raw_path_length_m{0.0};
  std::uint32_t shortcut_count{0};
  bool final_path_validated{false};
  bool final_path_fallback_to_raw{false};
  std::string final_path_validation_failure;
};

struct SurfacePlanResult
{
  bool success{false};
  std::string message;
  std::vector<GridIndex> raw_cells;
  std::vector<Point3> raw_path;
  std::vector<GridIndex> cells;
  std::vector<Point3> path;
  SurfacePlanMetrics metrics;
};

class SurfaceTransitionValidator
{
public:
  explicit SurfaceTransitionValidator(SurfacePlannerOptions options = {});

  TransitionReport validate(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const;
  std::vector<GridIndex> validNeighbors(
    const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  bool isCellTraversable(const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  bool isEndpointCell(const NavigationSnapshot & snapshot, const GridIndex & cell) const;

private:
  bool isFootprintSupported(
    const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const;
  bool isDirectSurfaceNeighbor(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const;
  bool isDiagonalCornerSupported(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const;
  bool hasTraversableCellAtXY(
    const NavigationSnapshot & snapshot, int x, int y, int min_z, int max_z) const;
  Point3 cellCenter(const GridIndex & cell, double resolution_m) const;
  GridIndex worldToGrid(const Point3 & point, double resolution_m) const;

  SurfacePlannerOptions options_;
  RobotFootprint footprint_;
};

class SurfaceAstarPlanner
{
public:
  explicit SurfaceAstarPlanner(SurfacePlannerOptions options = {});

  SurfacePlanResult plan(
    const NavigationSnapshot & snapshot, const GridIndex & start, const GridIndex & goal) const;
  SurfacePlanResult plan(
    const ExperienceSurfaceGraph & graph, SurfaceNodeId start, SurfaceNodeId goal) const;

private:
  double transitionCost(
    const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to,
    const GridIndex * previous) const;
  double unknownPenalty(const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  Point3 cellCenter(const GridIndex & cell, double resolution_m) const;
  GridIndex worldToGrid(const Point3 & point, double resolution_m) const;
  std::vector<Point3> cellsToPath(
    const std::vector<GridIndex> & cells, double resolution_m) const;
  std::vector<GridIndex> shortcutPath(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells) const;
  bool isShortcutAllowed(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
    std::size_t from_index, std::size_t to_index) const;
  double minRawClearance(
    const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
    std::size_t from_index, std::size_t to_index) const;
  bool validatePath(
    const NavigationSnapshot & snapshot, const std::vector<Point3> & path,
    std::string & failure_reason) const;
  bool hasRequiredFinalClearance(
    const NavigationSnapshot & snapshot, const GridIndex & cell, std::string & failure_reason) const;
  void fillMetrics(const NavigationSnapshot & snapshot, SurfacePlanResult & result) const;

  SurfacePlannerOptions options_;
  SurfaceTransitionValidator transition_validator_;
};

}  // namespace tgw_planner::core
