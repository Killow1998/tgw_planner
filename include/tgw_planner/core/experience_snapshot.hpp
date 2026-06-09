#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/risk_field.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

struct ExperienceSnapshot
{
  SurfaceMap surface;
  ClearanceField clearance;
  RiskField risk;
  std::unordered_map<GridIndex, ReachabilityLabel, GridIndexHash> reachability;
  std::unordered_set<GridIndex, GridIndexHash> observed_free_cells;
  std::vector<TrajectoryBridgeSegment> bridge_segments;
  std::string map_frame{"map"};
  double resolution_m{0.10};
};

using NavigationSnapshot = ExperienceSnapshot;

}  // namespace tgw_planner::core
