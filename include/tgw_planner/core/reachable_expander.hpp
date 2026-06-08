#pragma once

#include <unordered_map>
#include <unordered_set>

#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

struct ReachableExpanderOptions
{
  int expansion_radius_cells{1};
  int max_expansion_steps{1};
  int vertical_tolerance_cells{1};
};

struct ReachableExpansionResult
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> surface_cells;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells;
  std::unordered_map<GridIndex, ReachabilityLabel, GridIndexHash> reachability;
  std::size_t proven_seed_count{0};
  std::size_t inferred_cell_count{0};
};

class ReachableExpander
{
public:
  explicit ReachableExpander(ReachableExpanderOptions options = {});

  ReachableExpansionResult expand(
    const std::unordered_set<GridIndex, GridIndexHash> & proven_seed_cells,
    const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const;

private:
  ReachableExpanderOptions options_;
};

}  // namespace tgw_planner::core
