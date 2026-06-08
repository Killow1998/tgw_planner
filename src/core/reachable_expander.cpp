#include "tgw_planner/core/reachable_expander.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace tgw_planner::core
{
namespace
{
struct QueueItem
{
  GridIndex cell;
  int steps{0};
};

struct AnchorHeightRange
{
  std::vector<int> z_values;
};

using AnchorColumns = std::unordered_map<GridIndex, AnchorHeightRange, GridIndexHash>;

double fallbackHeight(const GridIndex & cell, double resolution_m)
{
  return (static_cast<double>(cell.z) + 0.5) * resolution_m;
}

double cellHeight(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const GridIndex & cell,
  double resolution_m)
{
  const auto it = cells.find(cell);
  if (it == cells.end()) {
    return fallbackHeight(cell, resolution_m);
  }
  return std::isfinite(it->second.height_m) ? it->second.height_m :
         fallbackHeight(cell, resolution_m);
}

bool hasBodyObstruction(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const GridIndex & cell,
  int body_clearance_cells)
{
  for (int dz = 1; dz <= body_clearance_cells; ++dz) {
    if (cells.find({cell.x, cell.y, cell.z + dz}) != cells.end()) {
      return true;
    }
  }
  return false;
}

bool isBodyClear(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const SurfaceCell & candidate,
  int body_clearance_cells)
{
  return !candidate.body_obstructed &&
         !hasBodyObstruction(cells, candidate.cell, body_clearance_cells);
}

bool collectHoleFillSupport(
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & surface_cells,
  const std::unordered_set<GridIndex, GridIndexHash> & traversable_cells,
  const GridIndex & candidate,
  const ReachableExpanderOptions & options,
  double * average_height_m)
{
  std::vector<double> heights;
  heights.reserve(8U);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      for (int dz = -options.vertical_tolerance_cells; dz <= options.vertical_tolerance_cells;
        ++dz)
      {
        const GridIndex neighbor{candidate.x + dx, candidate.y + dy, candidate.z + dz};
        if (traversable_cells.find(neighbor) == traversable_cells.end()) {
          continue;
        }
        heights.push_back(cellHeight(surface_cells, neighbor, options.resolution_m));
      }
    }
  }
  if (static_cast<int>(heights.size()) < options.min_hole_fill_neighbors) {
    return false;
  }

  const auto [min_it, max_it] = std::minmax_element(heights.begin(), heights.end());
  if ((*max_it - *min_it) > options.max_hole_fill_height_spread_m) {
    return false;
  }

  double total = 0.0;
  for (const double height : heights) {
    total += height;
  }
  *average_height_m = total / static_cast<double>(heights.size());
  return true;
}

GridIndex columnKey(const GridIndex & cell)
{
  return {cell.x, cell.y, 0};
}

AnchorColumns buildAnchorColumns(
  const std::unordered_set<GridIndex, GridIndexHash> & anchors,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & cells,
  const ReachableExpanderOptions & options)
{
  AnchorColumns columns;
  const int radius = std::max(0, options.experience_anchor_radius_cells);
  for (const GridIndex & anchor : anchors) {
    const double height = cellHeight(cells, anchor, options.resolution_m);
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }
        (void)height;
        columns[{anchor.x + dx, anchor.y + dy, 0}].z_values.push_back(anchor.z);
      }
    }
  }
  for (auto & entry : columns) {
    auto & z_values = entry.second.z_values;
    std::sort(z_values.begin(), z_values.end());
    z_values.erase(std::unique(z_values.begin(), z_values.end()), z_values.end());
  }
  return columns;
}

bool isInsideAnchorEnvelope(
  const AnchorColumns & anchors,
  const GridIndex & cell,
  double,
  double height_tolerance_m,
  int vertical_tolerance_cells)
{
  (void)height_tolerance_m;
  const auto it = anchors.find(columnKey(cell));
  if (it == anchors.end()) {
    return false;
  }
  for (const int anchor_z : it->second.z_values) {
    if (std::abs(cell.z - anchor_z) <= vertical_tolerance_cells) {
      return true;
    }
  }
  return false;
}
}  // namespace

ReachableExpander::ReachableExpander(ReachableExpanderOptions options)
: options_(options)
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
  options_.expansion_radius_cells = std::max(0, options_.expansion_radius_cells);
  options_.max_expansion_steps = std::max(0, options_.max_expansion_steps);
  options_.vertical_tolerance_cells = std::max(0, options_.vertical_tolerance_cells);
  options_.max_expansion_step_height_m = std::max(0.0, options_.max_expansion_step_height_m);
  options_.body_clearance_cells = std::max(0, options_.body_clearance_cells);
  if (options_.experience_anchor_radius_cells < 0) {
    options_.experience_anchor_radius_cells =
      options_.expansion_radius_cells * options_.max_expansion_steps;
  }
  options_.experience_anchor_height_tolerance_m =
    std::max(0.0, options_.experience_anchor_height_tolerance_m);
  options_.experience_anchor_vertical_tolerance_cells =
    std::max(0, options_.experience_anchor_vertical_tolerance_cells);
  options_.hole_fill_iterations = std::max(0, options_.hole_fill_iterations);
  options_.min_hole_fill_neighbors = std::max(1, options_.min_hole_fill_neighbors);
  options_.max_hole_fill_height_spread_m =
    std::max(0.0, options_.max_hole_fill_height_spread_m);
}

ReachableExpansionResult ReachableExpander::expand(
  const std::unordered_set<GridIndex, GridIndexHash> & proven_seed_cells,
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry_cells) const
{
  ReachableExpansionResult result;
  result.surface_cells = geometry_cells;

  std::queue<QueueItem> queue;
  for (const GridIndex & seed : proven_seed_cells) {
    SurfaceCell & cell = result.surface_cells[seed];
    cell.cell = seed;
    cell.support = {seed.x, seed.y, seed.z - 1};
    cell.label = SurfaceLabel::ReachableSeed;
    cell.reachability = ReachabilityLabel::ProvenReachable;
    cell.confidence = 1.0;
    if (!std::isfinite(cell.height_m)) {
      cell.height_m = fallbackHeight(seed, options_.resolution_m);
    }
    result.traversable_cells.insert(seed);
    result.reachability[seed] = ReachabilityLabel::ProvenReachable;
    queue.push({seed, 0});
  }
  result.proven_seed_count = result.traversable_cells.size();
  const AnchorColumns anchor_columns = buildAnchorColumns(
    result.traversable_cells, result.surface_cells, options_);

  while (!queue.empty()) {
    const QueueItem current = queue.front();
    queue.pop();
    if (current.steps >= options_.max_expansion_steps) {
      continue;
    }

    for (int dx = -options_.expansion_radius_cells; dx <= options_.expansion_radius_cells; ++dx) {
      for (int dy = -options_.expansion_radius_cells; dy <= options_.expansion_radius_cells; ++dy) {
        for (int dz = -options_.vertical_tolerance_cells; dz <= options_.vertical_tolerance_cells;
          ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{
            current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          const auto geometry_it = geometry_cells.find(neighbor);
          if (geometry_it == geometry_cells.end()) {
            continue;
          }
          if (!isInsideAnchorEnvelope(
              anchor_columns, neighbor, geometry_it->second.height_m,
              options_.experience_anchor_height_tolerance_m,
              options_.experience_anchor_vertical_tolerance_cells))
          {
            ++result.anchor_envelope_rejected_count;
            continue;
          }
          if (!isBodyClear(
              result.surface_cells, geometry_it->second, options_.body_clearance_cells))
          {
            ++result.body_obstructed_rejected_count;
            continue;
          }
          const double current_height = cellHeight(
            result.surface_cells, current.cell, options_.resolution_m);
          if (std::abs(geometry_it->second.height_m - current_height) >
            options_.max_expansion_step_height_m)
          {
            ++result.rejected_expansion_count;
            continue;
          }
          if (!result.traversable_cells.insert(neighbor).second) {
            continue;
          }
          SurfaceCell & cell = result.surface_cells[neighbor];
          cell = geometry_it->second;
          cell.label = SurfaceLabel::Expanded;
          cell.reachability = ReachabilityLabel::InferredReachable;
          cell.confidence = std::max(cell.confidence, 0.5);
          result.reachability[neighbor] = ReachabilityLabel::InferredReachable;
          ++result.inferred_cell_count;
          queue.push({neighbor, current.steps + 1});
        }
      }
    }
  }

  if (options_.enable_hole_filling) {
    for (int iteration = 0; iteration < options_.hole_fill_iterations; ++iteration) {
      std::vector<SurfaceCell> additions;
      for (const GridIndex & cell : result.traversable_cells) {
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const GridIndex candidate{cell.x + dx, cell.y + dy, cell.z};
            if (result.traversable_cells.find(candidate) != result.traversable_cells.end()) {
              continue;
            }
            const double candidate_height = fallbackHeight(candidate, options_.resolution_m);
            if (!isInsideAnchorEnvelope(
                anchor_columns, candidate, candidate_height,
                options_.experience_anchor_height_tolerance_m,
                options_.experience_anchor_vertical_tolerance_cells))
            {
              ++result.anchor_envelope_rejected_count;
              continue;
            }
            if (hasBodyObstruction(
                result.surface_cells, candidate, options_.body_clearance_cells))
            {
              ++result.body_obstructed_rejected_count;
              continue;
            }

            double average_height = 0.0;
            if (!collectHoleFillSupport(
                result.surface_cells, result.traversable_cells, candidate, options_,
                &average_height))
            {
              continue;
            }

            const auto geometry_it = geometry_cells.find(candidate);
            if (geometry_it != geometry_cells.end()) {
              if (!isBodyClear(
                  result.surface_cells, geometry_it->second, options_.body_clearance_cells))
              {
                ++result.body_obstructed_rejected_count;
                continue;
              }
              if (std::abs(geometry_it->second.height_m - average_height) >
                options_.max_hole_fill_height_spread_m)
              {
                continue;
              }
            }

            SurfaceCell filled;
            if (geometry_it != geometry_cells.end()) {
              filled = geometry_it->second;
            }
            filled.cell = candidate;
            filled.support = {candidate.x, candidate.y, candidate.z - 1};
            filled.label = SurfaceLabel::Expanded;
            filled.reachability = ReachabilityLabel::LowConfidenceReachable;
            filled.height_m = geometry_it == geometry_cells.end() ?
              average_height : geometry_it->second.height_m;
            filled.confidence = std::max(filled.confidence, 0.35);
            filled.hole_filled = true;
            additions.push_back(filled);
          }
        }
      }

      if (additions.empty()) {
        break;
      }
      for (const SurfaceCell & filled : additions) {
        if (!result.traversable_cells.insert(filled.cell).second) {
          continue;
        }
        result.surface_cells[filled.cell] = filled;
        result.reachability[filled.cell] = filled.reachability;
        ++result.inferred_cell_count;
        ++result.hole_filled_count;
      }
    }
  }

  return result;
}

}  // namespace tgw_planner::core
