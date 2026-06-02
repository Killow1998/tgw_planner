#pragma once

#include <unordered_set>

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/risk_field.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace tgw_planner::core
{

struct NavigationSnapshot
{
  SurfaceMap surface;
  ClearanceField clearance;
  RiskField risk;
  std::unordered_set<GridIndex, GridIndexHash> observed_free_cells;
  double resolution_m{0.10};
};

}  // namespace tgw_planner::core
