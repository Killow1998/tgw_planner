#pragma once

#include <unordered_set>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/n3map_reader.hpp"

namespace tgw_planner::core
{

struct TrajectoryProjectorOptions
{
  double resolution_m{0.10};
  double support_z_offset_m{0.0};
};

struct TrajectoryProjectionResult
{
  std::unordered_set<GridIndex, GridIndexHash> proven_seed_cells;
};

class TrajectoryProjector
{
public:
  explicit TrajectoryProjector(TrajectoryProjectorOptions options = {});

  TrajectoryProjectionResult project(const N3NavResource & resource) const;

private:
  GridIndex worldToGrid(const Point3 & point) const;

  TrajectoryProjectorOptions options_;
};

}  // namespace tgw_planner::core
