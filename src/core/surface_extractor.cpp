#include "tgw_planner/core/surface_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{
SurfaceExtractor::SurfaceExtractor(SurfaceExtractionOptions options)
: options_(options)
{
  options_.robot_height_m = std::max(0.10, options_.robot_height_m);
  options_.max_step_height_m = std::max(0.05, options_.max_step_height_m);
}

SurfaceMap SurfaceExtractor::extract(const ProbabilisticVoxelMap & occupancy) const
{
  SurfaceMap surface;
  std::unordered_map<GridIndex, int, GridIndexHash> support_by_cell;

  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(options_.robot_height_m / occupancy.resolution())));
  for (const GridIndex & support : occupancy.occupiedVoxels()) {
    const VoxelState * state = occupancy.lookup(support);
    if (state == nullptr || !supportAccepted(*state)) {
      continue;
    }
    const GridIndex stand{support.x, support.y, support.z + 1};
    if ((!options_.treat_hits_as_surface_samples && occupancy.isOccupied(stand)) ||
      !hasHeadClearance(occupancy, stand, height_cells) ||
      !hasObservedFreeSpace(occupancy, stand))
    {
      surface.forbidden_cells.insert(stand);
      continue;
    }
    SurfaceCell cell;
    cell.cell = stand;
    cell.support = support;
    cell.height_m = occupancy.gridToWorld(stand).z;
    surface.surface_cells[stand] = cell;
    surface.traversable_cells.insert(stand);
    support_by_cell[stand] = support.z;
  }

  for (auto & entry : surface.surface_cells) {
    entry.second.label = classify(entry.first, support_by_cell, occupancy.resolution());
  }

  rebuildBoundaryLayer(surface, occupancy);
  return surface;
}

void SurfaceExtractor::rebuildBoundaryLayer(
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

bool SurfaceExtractor::hasHeadClearance(
  const ProbabilisticVoxelMap & occupancy, const GridIndex & stand, int height_cells) const
{
  if (options_.treat_hits_as_surface_samples) {
    return true;
  }
  for (int dz = 0; dz <= height_cells; ++dz) {
    if (occupancy.isOccupied({stand.x, stand.y, stand.z + dz})) {
      return false;
    }
  }
  return true;
}

bool SurfaceExtractor::hasObservedFreeSpace(
  const ProbabilisticVoxelMap & occupancy, const GridIndex & stand) const
{
  if (!options_.require_observed_free_space) {
    return true;
  }
  return occupancy.isFree(stand);
}

bool SurfaceExtractor::supportAccepted(const VoxelState & state) const
{
  if (options_.require_static_support) {
    return state.static_candidate && !state.dynamic_suspect;
  }
  return state.occupied && !state.dynamic_suspect &&
         state.hit_count >= static_cast<std::uint16_t>(std::max(1, options_.min_static_hits));
}

SurfaceLabel SurfaceExtractor::classify(
  const GridIndex & cell,
  const std::unordered_map<GridIndex, int, GridIndexHash> & support_by_cell,
  double resolution_m) const
{
  const auto center_it = support_by_cell.find(cell);
  if (center_it == support_by_cell.end()) {
    return SurfaceLabel::Unknown;
  }

  int max_abs_dz = 0;
  int neighbor_count = 0;
  const int max_step_cells =
    std::max(1, static_cast<int>(std::ceil(options_.max_step_height_m / resolution_m)));
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const auto same_level_it = support_by_cell.find({cell.x + dx, cell.y + dy, cell.z});
      auto neighbor_it = same_level_it;
      int best_abs_dz = std::numeric_limits<int>::max();
      if (same_level_it == support_by_cell.end()) {
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          const auto candidate_it = support_by_cell.find({cell.x + dx, cell.y + dy, cell.z + dz});
          if (candidate_it == support_by_cell.end()) {
            continue;
          }
          const int abs_dz = std::abs(dz);
          if (abs_dz < best_abs_dz) {
            neighbor_it = candidate_it;
            best_abs_dz = abs_dz;
          }
        }
      }
      if (neighbor_it == support_by_cell.end()) {
        continue;
      }
      max_abs_dz = std::max(max_abs_dz, std::abs(neighbor_it->second - center_it->second));
      ++neighbor_count;
    }
  }

  if (neighbor_count <= 2) {
    return SurfaceLabel::Narrow;
  }
  const double max_dz_m = static_cast<double>(max_abs_dz) * resolution_m;
  if (max_dz_m <= 0.5 * resolution_m) {
    return SurfaceLabel::FloorLike;
  }
  if (max_dz_m <= options_.max_step_height_m) {
    return SurfaceLabel::SlopeLike;
  }
  return SurfaceLabel::StairLike;
}

}  // namespace tgw_planner::core
