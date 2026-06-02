#include "tgw_planner/core/boundary_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace tgw_planner::core
{

BoundaryExtractor::BoundaryExtractor(BoundaryExtractionOptions options)
: options_(options)
{
  options_.max_step_height_m = std::max(0.05, options_.max_step_height_m);
}

void BoundaryExtractor::rebuildBoundaryLayer(
  SurfaceMap & surface, const ProbabilisticVoxelMap & occupancy) const
{
  surface.boundary_cells.clear();
  surface.dropoff_boundary_cells.clear();
  surface.wall_boundary_cells.clear();
  surface.forbidden_boundary_cells.clear();
  const int max_step_cells =
    std::max(1, static_cast<int>(std::ceil(options_.max_step_height_m / occupancy.resolution())));

  for (const GridIndex & cell : surface.traversable_cells) {
    bool boundary = false;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        bool found_neighbor = false;
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (surface.traversable_cells.find(neighbor) != surface.traversable_cells.end()) {
            found_neighbor = true;
            break;
          }
        }
        if (!found_neighbor) {
          boundary = true;
          surface.dropoff_boundary_cells.insert(cell);
        }
        const GridIndex same_level{cell.x + dx, cell.y + dy, cell.z};
        if (surface.forbidden_cells.find(same_level) != surface.forbidden_cells.end()) {
          boundary = true;
          surface.forbidden_boundary_cells.insert(cell);
        }
        for (int dz = 1; dz <= max_step_cells + 2; ++dz) {
          if (occupancy.isOccupied({cell.x + dx, cell.y + dy, cell.z + dz})) {
            boundary = true;
            surface.wall_boundary_cells.insert(cell);
            break;
          }
        }
      }
    }
    if (boundary) {
      surface.boundary_cells.insert(cell);
    }
  }
}

}  // namespace tgw_planner::core
