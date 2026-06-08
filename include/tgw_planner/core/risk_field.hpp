#pragma once

#include <unordered_map>

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

struct RiskFieldOptions
{
  double boundary_risk{0.5};
  double dropoff_risk{1.5};
  double wall_risk{1.0};
  double forbidden_risk{2.0};
  double low_clearance_risk{1.0};
  double low_clearance_threshold_m{0.35};
};

class RiskField
{
public:
  explicit RiskField(RiskFieldOptions options = {});

  void compute(const SurfaceMap & surface, const ClearanceField & clearance);

  double riskCost(const GridIndex & cell) const;
  const std::unordered_map<GridIndex, double, GridIndexHash> & risks() const;

private:
  RiskFieldOptions options_;
  std::unordered_map<GridIndex, double, GridIndexHash> risk_;
};

}  // namespace tgw_planner::core
