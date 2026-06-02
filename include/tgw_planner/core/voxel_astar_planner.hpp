#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/navigation_map.hpp"

namespace tgw_planner::core
{

struct PlannerMetrics
{
  bool success{false};
  std::string failure_reason;
  bool final_path_validated{false};
  bool final_path_fallback_to_raw{false};
  std::string final_path_validation_failure;
  double search_time_ms{0.0};
  double total_plan_time_ms{0.0};
  std::uint32_t expanded_nodes{0};
  std::uint32_t generated_nodes{0};
  std::uint32_t reopened_nodes{0};
  std::uint32_t max_open_set_size{0};
  std::uint32_t raw_path_waypoints{0};
  double raw_path_length_m{0.0};
  double raw_path_vertical_gain_m{0.0};
  double raw_path_vertical_loss_m{0.0};
  std::uint32_t postprocess_floor_shortcuts{0};
  std::uint32_t postprocess_stair_centerline_replacements{0};
  std::uint32_t path_waypoints{0};
  double path_length_m{0.0};
  double path_vertical_gain_m{0.0};
  double path_vertical_loss_m{0.0};
  double start_snap_distance_m{0.0};
  double goal_snap_distance_m{0.0};
};

struct PlanResult
{
  bool success{false};
  std::string message;
  bool start_snap_success{false};
  bool goal_snap_success{false};
  bool closest_closed_cell_valid{false};
  GridIndex start_cell;
  GridIndex goal_cell;
  GridIndex closest_closed_cell;
  double closest_closed_distance_m{0.0};
  std::vector<Point3> path;
  PlannerMetrics metrics;
};

struct PathPostprocessStats
{
  std::uint32_t floor_shortcuts{0};
  std::uint32_t stair_centerline_replacements{0};
};

class VoxelAstarPlanner
{
public:
  explicit VoxelAstarPlanner(std::uint32_t max_iterations = 250000);

  PlanResult plan(const NavigationMap & map, const Point3 & start, const Point3 & goal) const;
  bool snapToTraversable(
    const NavigationMap & map, const Point3 & seed, GridIndex & snapped,
    double & snap_distance_m) const;

private:
  std::vector<Point3> postProcessPath(
    const NavigationMap & map, const std::vector<GridIndex> & cells,
    PathPostprocessStats & stats) const;
  std::vector<Point3> simplifyFloorRun(
    const NavigationMap & map, const std::vector<GridIndex> & cells) const;
  std::vector<Point3> centerlineStairRun(
    const NavigationMap & map, int stair_flight_id, const std::vector<GridIndex> & cells) const;
  bool isLineTraversable(
    const NavigationMap & map, const Point3 & from, const Point3 & to, bool require_floor) const;
  std::vector<Point3> rawPathFromCells(
    const NavigationMap & map, const std::vector<GridIndex> & cells) const;
  bool validateFinalPath(
    const NavigationMap & map, const std::vector<Point3> & path, std::string & failure) const;
  std::vector<Point3> reconstructPath(
    const NavigationMap & map,
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    const GridIndex & start, const GridIndex & goal) const;
  void fillPathMetrics(const std::vector<Point3> & path, PlannerMetrics & metrics) const;

  std::uint32_t max_iterations_{250000};
};

}  // namespace tgw_planner::core
