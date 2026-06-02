#include "tgw_planner/core/surface_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace tgw_planner::core
{
namespace
{
struct QueueNode
{
  GridIndex cell;
  double f{0.0};
  double g{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

double gridDistance(const GridIndex & a, const GridIndex & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

SurfaceAstarPlanner::SurfaceAstarPlanner(SurfacePlannerOptions options)
: options_(options)
{
  options_.max_iterations = std::max<std::uint32_t>(1U, options_.max_iterations);
}

SurfacePlanResult SurfaceAstarPlanner::plan(
  const NavigationSnapshot & snapshot, const GridIndex & start, const GridIndex & goal) const
{
  SurfacePlanResult result;
  if (snapshot.surface.traversable_cells.find(start) == snapshot.surface.traversable_cells.end()) {
    result.message = "start is not traversable";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (snapshot.surface.traversable_cells.find(goal) == snapshot.surface.traversable_cells.end()) {
    result.message = "goal is not traversable";
    result.metrics.failure_reason = result.message;
    return result;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed;

  g_score[start] = 0.0;
  open.push({start, gridDistance(start, goal) * snapshot.resolution_m, 0.0});

  bool found = false;
  while (!open.empty() && result.metrics.expanded_nodes < options_.max_iterations) {
    const QueueNode current = open.top();
    open.pop();
    const auto best = g_score.find(current.cell);
    if (best == g_score.end() || current.g > best->second + 1.0e-9) {
      continue;
    }
    if (current.cell == goal) {
      found = true;
      break;
    }
    if (!closed.insert(current.cell).second) {
      continue;
    }
    ++result.metrics.expanded_nodes;

    const GridIndex * previous = nullptr;
    const auto previous_it = came_from.find(current.cell);
    if (previous_it != came_from.end()) {
      previous = &previous_it->second;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          if (snapshot.surface.traversable_cells.find(neighbor) ==
            snapshot.surface.traversable_cells.end())
          {
            continue;
          }
          if (snapshot.surface.forbidden_cells.find(neighbor) !=
            snapshot.surface.forbidden_cells.end())
          {
            continue;
          }
          const double tentative_g =
            current.g + transitionCost(snapshot, current.cell, neighbor, previous);
          const auto old = g_score.find(neighbor);
          if (old != g_score.end() && tentative_g >= old->second) {
            continue;
          }
          came_from[neighbor] = current.cell;
          g_score[neighbor] = tentative_g;
          const double h = gridDistance(neighbor, goal) * snapshot.resolution_m;
          open.push({neighbor, tentative_g + h, tentative_g});
          ++result.metrics.generated_nodes;
        }
      }
    }
  }

  if (!found) {
    result.message = result.metrics.expanded_nodes >= options_.max_iterations ?
      "surface A* reached max_iterations" : "surface A* failed to find a path";
    result.metrics.failure_reason = result.message;
    return result;
  }

  GridIndex current = goal;
  result.cells.push_back(current);
  while (current != start) {
    const auto it = came_from.find(current);
    if (it == came_from.end()) {
      result.message = "surface A* parent chain is incomplete";
      result.metrics.failure_reason = result.message;
      return result;
    }
    current = it->second;
    result.cells.push_back(current);
  }
  std::reverse(result.cells.begin(), result.cells.end());
  result.path.reserve(result.cells.size());
  for (const GridIndex & cell : result.cells) {
    result.path.push_back({
      (static_cast<double>(cell.x) + 0.5) * snapshot.resolution_m,
      (static_cast<double>(cell.y) + 0.5) * snapshot.resolution_m,
      (static_cast<double>(cell.z) + 0.5) * snapshot.resolution_m});
  }
  result.success = true;
  result.message = "path found";
  result.metrics.success = true;
  fillMetrics(snapshot, result);
  return result;
}

double SurfaceAstarPlanner::transitionCost(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to,
  const GridIndex * previous) const
{
  const double step = gridDistance(from, to) * snapshot.resolution_m;
  const double clearance_penalty = options_.w_clearance * snapshot.clearance.clearancePenalty(to);
  double slope_penalty = 0.0;
  const auto from_surface = snapshot.surface.surface_cells.find(from);
  const auto to_surface = snapshot.surface.surface_cells.find(to);
  if (from_surface != snapshot.surface.surface_cells.end() &&
    to_surface != snapshot.surface.surface_cells.end())
  {
    slope_penalty =
      options_.w_slope * std::abs(to_surface->second.height_m - from_surface->second.height_m);
  }

  double turn_penalty = 0.0;
  if (previous != nullptr) {
    const int ax = from.x - previous->x;
    const int ay = from.y - previous->y;
    const int bx = to.x - from.x;
    const int by = to.y - from.y;
    if (ax * bx + ay * by <= 0) {
      turn_penalty = options_.w_turn;
    }
  }

  return step + clearance_penalty + slope_penalty + turn_penalty;
}

void SurfaceAstarPlanner::fillMetrics(
  const NavigationSnapshot & snapshot, SurfacePlanResult & result) const
{
  if (result.cells.empty()) {
    return;
  }
  double clearance_sum = 0.0;
  result.metrics.min_path_clearance_m = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    const double clearance = snapshot.clearance.clearanceDistance(result.cells[i]);
    result.metrics.min_path_clearance_m =
      std::min(result.metrics.min_path_clearance_m, clearance);
    clearance_sum += clearance;
    result.metrics.clearance_cost_sum += snapshot.clearance.clearancePenalty(result.cells[i]);
    if (clearance < 0.30) {
      ++result.metrics.low_clearance_samples;
    }
    if (i > 0U) {
      result.metrics.path_length_m += gridDistance(result.cells[i - 1U], result.cells[i]) *
        snapshot.resolution_m;
    }
  }
  result.metrics.mean_path_clearance_m = clearance_sum / static_cast<double>(result.cells.size());
  if (!std::isfinite(result.metrics.min_path_clearance_m)) {
    result.metrics.min_path_clearance_m = 0.0;
  }
}

}  // namespace tgw_planner::core
