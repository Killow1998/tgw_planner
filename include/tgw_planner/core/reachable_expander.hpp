#pragma once

#include <unordered_map>
#include <unordered_set>

#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

struct ReachableExpanderOptions
{
  double resolution_m{0.10};
  int expansion_radius_cells{1};
  int max_expansion_steps{1};
  int vertical_tolerance_cells{1};
  double max_expansion_step_height_m{0.20};
  int body_clearance_cells{7};
  int experience_anchor_radius_cells{24};
  double experience_anchor_height_tolerance_m{0.60};
  int experience_anchor_vertical_tolerance_cells{3};
  bool enable_hole_filling{true};
  int hole_fill_iterations{2};
  int min_hole_fill_neighbors{5};
  double max_hole_fill_height_spread_m{0.12};
};

struct ReachableExpansionResult
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> surface_cells;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells;
  std::unordered_map<GridIndex, ReachabilityLabel, GridIndexHash> reachability;
  std::size_t proven_seed_count{0};
  std::size_t inferred_cell_count{0};
  std::size_t rejected_expansion_count{0};
  std::size_t body_obstructed_rejected_count{0};
  std::size_t anchor_envelope_rejected_count{0};
  std::size_t hole_filled_count{0};
  std::size_t bridge_seed_count{0};
  std::size_t bridge_used_as_expansion_anchor{0};
  std::size_t hole_fill_from_bridge_rejected{0};
  std::size_t support_component_count{0};
  std::size_t anchored_support_component_count{0};
  std::size_t rejected_unanchored_component_cells{0};
  double anchored_component_time_ms{0.0};
  double anchor_envelope_time_ms{0.0};
  double seed_initialization_time_ms{0.0};
  double expansion_frontier_time_ms{0.0};
  double expansion_wave_time_ms{0.0};
  double hole_fill_time_ms{0.0};
  double layer_assignment_time_ms{0.0};
  double bridge_seed_time_ms{0.0};
  double compact_time_ms{0.0};
};

class ReachableExpander
{
public:
  explicit ReachableExpander(ReachableExpanderOptions options = {});

  ReachableExpansionResult expand(
    const std::unordered_set<GridIndex, GridIndexHash> & observed_seed_cells,
    const std::unordered_set<GridIndex, GridIndexHash> & bridge_seed_cells,
    const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const;
  ReachableExpansionResult expand(
    const std::unordered_set<GridIndex, GridIndexHash> & observed_seed_cells,
    const std::unordered_set<GridIndex, GridIndexHash> & bridge_seed_cells,
    const std::unordered_map<GridIndex, BridgeCellMetadata, GridIndexHash> & bridge_metadata,
    const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const;

private:
  ReachableExpanderOptions options_;
};

}  // namespace tgw_planner::core
