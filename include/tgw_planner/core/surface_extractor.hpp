#pragma once

#include <unordered_map>
#include <unordered_set>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/probabilistic_voxel_map.hpp"

namespace tgw_planner::core
{

enum class SurfaceLabel
{
  Unknown,
  FloorLike,
  SlopeLike,
  StairLike,
  Narrow
};

struct SurfaceCell
{
  GridIndex cell;
  GridIndex support;
  SurfaceLabel label{SurfaceLabel::Unknown};
  double height_m{0.0};
  double slope_m{0.0};
};

struct SurfaceMap
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> surface_cells;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells;
  std::unordered_set<GridIndex, GridIndexHash> blocked_cells;
  std::unordered_set<GridIndex, GridIndexHash> forbidden_cells;
  std::unordered_set<GridIndex, GridIndexHash> boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> dropoff_boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> wall_boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> forbidden_boundary_cells;
};

struct SurfaceExtractionOptions
{
  double robot_height_m{0.50};
  double max_step_height_m{0.30};
  int min_static_hits{1};
  bool require_static_support{false};
  bool require_observed_free_space{true};
  bool treat_hits_as_surface_samples{false};
};

class SurfaceExtractor
{
public:
  explicit SurfaceExtractor(SurfaceExtractionOptions options = {});

  SurfaceMap extract(const ProbabilisticVoxelMap & occupancy) const;
  void rebuildBoundaryLayer(SurfaceMap & surface, const ProbabilisticVoxelMap & occupancy) const;

private:
  bool hasHeadClearance(
    const ProbabilisticVoxelMap & occupancy, const GridIndex & stand, int height_cells) const;
  bool hasObservedFreeSpace(
    const ProbabilisticVoxelMap & occupancy, const GridIndex & stand) const;
  bool supportAccepted(const VoxelState & state) const;
  SurfaceLabel classify(
    const GridIndex & cell,
    const std::unordered_map<GridIndex, int, GridIndexHash> & support_by_cell,
    double resolution_m) const;

  SurfaceExtractionOptions options_;
};

}  // namespace tgw_planner::core
