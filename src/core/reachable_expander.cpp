#include "tgw_planner/core/reachable_expander.hpp"

#include <algorithm>
#include <queue>

namespace tgw_planner::core
{
namespace
{
struct QueueItem
{
  GridIndex cell;
  int steps{0};
};
}  // namespace

ReachableExpander::ReachableExpander(ReachableExpanderOptions options)
: options_(options)
{
  options_.expansion_radius_cells = std::max(0, options_.expansion_radius_cells);
  options_.max_expansion_steps = std::max(0, options_.max_expansion_steps);
  options_.vertical_tolerance_cells = std::max(0, options_.vertical_tolerance_cells);
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
    result.traversable_cells.insert(seed);
    result.reachability[seed] = ReachabilityLabel::ProvenReachable;
    queue.push({seed, 0});
  }
  result.proven_seed_count = result.traversable_cells.size();

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

  return result;
}

}  // namespace tgw_planner::core
