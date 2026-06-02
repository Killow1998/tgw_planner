#pragma once

#include "tgw_planner/core/probabilistic_voxel_map.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace tgw_planner::core
{

struct BoundaryExtractionOptions
{
  double max_step_height_m{0.30};
};

class BoundaryExtractor
{
public:
  explicit BoundaryExtractor(BoundaryExtractionOptions options = {});

  void rebuildBoundaryLayer(
    SurfaceMap & surface, const ProbabilisticVoxelMap & occupancy) const;

private:
  BoundaryExtractionOptions options_;
};

}  // namespace tgw_planner::core
