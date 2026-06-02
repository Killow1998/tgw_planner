#include "tgw_planner/core/surface_extractor.hpp"

#include "tgw_planner/core/boundary_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> candidates;
  std::unordered_map<GridIndex, int, GridIndexHash> candidate_support_by_cell;
  std::unordered_set<GridIndex, GridIndexHash> observed_anchors;

  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(options_.robot_height_m / occupancy.resolution())));
  const int max_step_cells =
    std::max(1, static_cast<int>(std::ceil(options_.max_step_height_m / occupancy.resolution())));
  for (const GridIndex & support : occupancy.occupiedVoxels()) {
    const VoxelState * state = occupancy.lookup(support);
    if (state == nullptr || !supportAccepted(*state)) {
      continue;
    }
    const GridIndex stand{support.x, support.y, support.z + 1};
    if ((!options_.treat_hits_as_surface_samples && occupancy.isOccupied(stand)) ||
      !hasHeadClearance(occupancy, stand, height_cells))
    {
      surface.forbidden_cells.insert(stand);
      continue;
    }
    SurfaceCell cell;
    cell.cell = stand;
    cell.support = support;
    cell.height_m = occupancy.gridToWorld(stand).z;
    const bool observed_anchor =
      hasObservedFreeSpace(occupancy, stand) ||
      hasObservedClearanceEvidence(occupancy, stand, height_cells);
    if (!observed_anchor && options_.require_observed_free_space &&
      !options_.allow_observed_free_bridge)
    {
      surface.forbidden_cells.insert(stand);
      continue;
    }
    candidates[stand] = cell;
    candidate_support_by_cell[stand] = support.z;
    if (observed_anchor) {
      observed_anchors.insert(stand);
    }
  }

  if (!options_.require_observed_free_space || !options_.allow_observed_free_bridge) {
    for (const auto & entry : candidates) {
      surface.surface_cells[entry.first] = entry.second;
      surface.traversable_cells.insert(entry.first);
      support_by_cell[entry.first] = candidate_support_by_cell[entry.first];
    }
  } else {
    std::unordered_set<GridIndex, GridIndexHash> visited;
    std::vector<GridIndex> stack;
    std::vector<GridIndex> component;
    for (const auto & seed_entry : candidates) {
      const GridIndex & seed = seed_entry.first;
      if (visited.find(seed) != visited.end()) {
        continue;
      }
      bool has_anchor = false;
      stack.clear();
      component.clear();
      visited.insert(seed);
      stack.push_back(seed);
      while (!stack.empty()) {
        const GridIndex cell = stack.back();
        stack.pop_back();
        component.push_back(cell);
        has_anchor = has_anchor || observed_anchors.find(cell) != observed_anchors.end();
        const int support_z = candidate_support_by_cell[cell];
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
              const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
              if (visited.find(neighbor) != visited.end()) {
                continue;
              }
              const auto neighbor_it = candidates.find(neighbor);
              if (neighbor_it == candidates.end()) {
                continue;
              }
              const int neighbor_support_z = candidate_support_by_cell[neighbor];
              if (std::abs(neighbor_support_z - support_z) > max_step_cells) {
                continue;
              }
              visited.insert(neighbor);
              stack.push_back(neighbor);
            }
          }
        }
      }

      if (!has_anchor) {
        for (const GridIndex & cell : component) {
          surface.forbidden_cells.insert(cell);
        }
        continue;
      }
      for (const GridIndex & cell : component) {
        surface.surface_cells[cell] = candidates[cell];
        surface.traversable_cells.insert(cell);
        support_by_cell[cell] = candidate_support_by_cell[cell];
      }
    }
  }

  for (const auto & entry : candidates) {
    if (surface.surface_cells.find(entry.first) == surface.surface_cells.end()) {
      surface.forbidden_cells.insert(entry.first);
      continue;
    }
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
  BoundaryExtractor({options_.max_step_height_m}).rebuildBoundaryLayer(surface, occupancy);
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

bool SurfaceExtractor::hasObservedClearanceEvidence(
  const ProbabilisticVoxelMap & occupancy, const GridIndex & stand, int height_cells) const
{
  if (!options_.require_observed_free_space) {
    return true;
  }

  int observed_columns = 0;
  bool same_column_body_free = false;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      bool column_observed = false;
      for (int dz = 0; dz <= height_cells; ++dz) {
        const GridIndex probe{stand.x + dx, stand.y + dy, stand.z + dz};
        if (!occupancy.isFree(probe)) {
          continue;
        }
        column_observed = true;
        if (dx == 0 && dy == 0 && dz > 0) {
          same_column_body_free = true;
        }
      }
      if (column_observed) {
        ++observed_columns;
      }
    }
  }

  return same_column_body_free || observed_columns >= 2;
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
