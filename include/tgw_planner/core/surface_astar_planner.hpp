#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
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
  std::uint32_t path_layer_jump_edges{0};
  double max_path_edge_dz_m{0.0};
  std::uint32_t start_portal_candidates{0};
  std::uint32_t goal_portal_candidates{0};
  std::uint32_t evaluated_portal_pairs{0};
  std::uint32_t hybrid_nodes{0};
  std::uint32_t hybrid_surface_edges{0};
  std::uint32_t hybrid_backbone_edges{0};
  std::uint32_t hybrid_portal_edges{0};
  std::uint32_t hybrid_expanded_nodes{0};
  std::uint32_t used_backbone_edges{0};
  std::uint32_t used_portal_edges{0};
  std::uint32_t used_surface_edges{0};
  double backbone_path_length_m{0.0};
  double surface_path_length_m{0.0};
  std::uint32_t portal_switch_count{0};
  std::uint32_t selected_start_portal_id{std::numeric_limits<std::uint32_t>::max()};
  std::uint32_t selected_goal_portal_id{std::numeric_limits<std::uint32_t>::max()};
  std::uint32_t selected_start_backbone_node{std::numeric_limits<std::uint32_t>::max()};
  std::uint32_t selected_goal_backbone_node{std::numeric_limits<std::uint32_t>::max()};
  std::uint32_t selected_backbone_index_delta{0};
  double selected_backbone_length_m{0.0};
  double selected_start_surface_leg_m{0.0};
  double selected_goal_surface_leg_m{0.0};
  double selected_total_hybrid_cost{0.0};
};

enum class PathPointKind
{
  Unknown,
  Surface,
  Backbone,
  Portal
};

struct GlobalPathPoint
{
  Point3 position;
  double yaw_hint_rad{0.0};
  PathPointKind kind{PathPointKind::Unknown};
  double target_speed_mps{0.0};
  double confidence{1.0};
  int surface_component_id{-1};
};

struct SurfacePlanResult
{
  bool success{false};
  std::string message;
  std::vector<GridIndex> raw_cells;
  std::vector<Point3> raw_path;
  std::vector<GridIndex> cells;
  std::vector<Point3> path;
  std::vector<PathPointKind> path_kinds;
  std::vector<GlobalPathPoint> global_path;
  std::vector<Point3> debug_start_portal_candidates;
  std::vector<Point3> debug_goal_portal_candidates;
  std::vector<Point3> debug_selected_start_portal;
  std::vector<Point3> debug_selected_goal_portal;
  std::vector<Point3> debug_selected_backbone_segment;
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
  bool isCellCenterFootprintSupported(
    const NavigationSnapshot & snapshot, const GridIndex & cell, double yaw_rad) const;
  void reserveCellCenterFootprintCache(std::size_t entries) const;
  const SurfacePlannerOptions & options() const;

private:
  struct FootprintCacheKey
  {
    const NavigationSnapshot * snapshot{nullptr};
    GridIndex cell;
    int cos_bucket{0};
    int sin_bucket{0};

    bool operator==(const FootprintCacheKey & other) const
    {
      return snapshot == other.snapshot &&
             cell == other.cell &&
             cos_bucket == other.cos_bucket &&
             sin_bucket == other.sin_bucket;
    }
  };

  struct FootprintCacheKeyHash
  {
    std::size_t operator()(const FootprintCacheKey & key) const
    {
      std::size_t seed = std::hash<const void *>{}(key.snapshot);
      const auto mix = [&seed](std::size_t value) {
        seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
      };
      mix(GridIndexHash{}(key.cell));
      mix(std::hash<int>{}(key.cos_bucket));
      mix(std::hash<int>{}(key.sin_bucket));
      return seed;
    }
  };

  FootprintCacheKey footprintCacheKey(
    const NavigationSnapshot & snapshot, const GridIndex & cell, double yaw_rad) const;
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
  mutable const NavigationSnapshot * cell_center_footprint_cache_snapshot_{nullptr};
  mutable std::unordered_map<FootprintCacheKey, bool, FootprintCacheKeyHash>
  cell_center_footprint_cache_;
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
  bool validateGraphPath(
    const ExperienceSurfaceGraph & graph, const std::vector<SurfaceNodeId> & node_path,
    std::string & failure_reason, SurfacePlanMetrics & metrics) const;
  bool hasRequiredFinalClearance(
    const NavigationSnapshot & snapshot, const GridIndex & cell, std::string & failure_reason) const;
  void fillMetrics(const NavigationSnapshot & snapshot, SurfacePlanResult & result) const;

  SurfacePlannerOptions options_;
  SurfaceTransitionValidator transition_validator_;
};

}  // namespace tgw_planner::core
