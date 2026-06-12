#include "tgw_planner/core/risk_field.hpp"

#include <algorithm>
#include <cmath>

namespace tgw_planner::core
{

RiskField::RiskField(RiskFieldOptions options)
: options_(options)
{
  options_.low_clearance_threshold_m = std::max(0.0, options_.low_clearance_threshold_m);
}

void RiskField::compute(const SurfaceMap & surface, const ClearanceField & clearance)
{
  risk_.clear();
  risk_.reserve(surface.boundary_cells.size());
  for (const GridIndex & cell : surface.traversable_cells) {
    double risk = 0.0;
    if (surface.boundary_cells.find(cell) != surface.boundary_cells.end()) {
      risk += options_.boundary_risk;
    }
    if (surface.dropoff_boundary_cells.find(cell) != surface.dropoff_boundary_cells.end()) {
      risk += options_.dropoff_risk;
    }
    if (surface.wall_boundary_cells.find(cell) != surface.wall_boundary_cells.end()) {
      risk += options_.wall_risk;
    }
    if (surface.forbidden_boundary_cells.find(cell) != surface.forbidden_boundary_cells.end()) {
      risk += options_.forbidden_risk;
    }

    const double clearance_m = clearance.clearanceDistance(cell);
    if (std::isfinite(clearance_m) && clearance_m < options_.low_clearance_threshold_m) {
      const double normalized =
        options_.low_clearance_threshold_m <= 1.0e-9 ? 1.0 :
        (options_.low_clearance_threshold_m - clearance_m) / options_.low_clearance_threshold_m;
      risk += options_.low_clearance_risk * std::clamp(normalized, 0.0, 1.0);
    }

    if (risk > 0.0) {
      risk_[cell] = risk;
    }
  }
}

double RiskField::riskCost(const GridIndex & cell) const
{
  const auto it = risk_.find(cell);
  return it == risk_.end() ? 0.0 : it->second;
}

const std::unordered_map<GridIndex, double, GridIndexHash> & RiskField::risks() const
{
  return risk_;
}

}  // namespace tgw_planner::core
