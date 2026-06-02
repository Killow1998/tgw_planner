#pragma once

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace tgw_planner::core
{

struct NavigationSnapshot
{
  SurfaceMap surface;
  ClearanceField clearance;
  double resolution_m{0.10};
};

}  // namespace tgw_planner::core
