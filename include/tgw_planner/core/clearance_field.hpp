#pragma once

#include <unordered_map>
#include <unordered_set>

#include "tgw_planner/core/grid_index.hpp"

namespace tgw_planner::core
{

class ClearanceField
{
public:
  void compute(
    const std::unordered_set<GridIndex, GridIndexHash> & traversable,
    const std::unordered_set<GridIndex, GridIndexHash> & boundary,
    double resolution_m);

  double clearanceDistance(const GridIndex & cell) const;
  double clearancePenalty(const GridIndex & cell) const;

  const std::unordered_map<GridIndex, double, GridIndexHash> & distances() const;

private:
  std::unordered_map<GridIndex, double, GridIndexHash> distance_m_;
};

}  // namespace tgw_planner::core
