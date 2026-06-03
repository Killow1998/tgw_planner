#pragma once

#include <cstddef>
#include <unordered_set>

#include "tgw_planner/core/grid_index.hpp"

namespace tgw_planner::core
{

class NavigationMap;

bool possibleVerticalPcdArtifactsDetected(
  const std::unordered_set<GridIndex, GridIndexHash> & occupied_cells, double resolution_m);

bool possiblePcdArtifactsDetected(
  const std::unordered_set<GridIndex, GridIndexHash> & occupied_cells, double resolution_m,
  std::size_t rejected_collision_cells, std::size_t rejected_stair_noise_cells,
  std::size_t rejected_short_low_cells, std::size_t rejected_width_prefilter_cells);

bool possiblePcdArtifactsDetected(const NavigationMap & map);

}  // namespace tgw_planner::core
