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
: options_(options), footprint_(options.footprint)
{
  options_.max_iterations = std::max<std::uint32_t>(1U, options_.max_iterations);
  options_.swept_sample_step_m = std::max(0.01, options_.swept_sample_step_m);
  options_.shortcut_sample_step_m = std::max(0.01, options_.shortcut_sample_step_m);
  options_.shortcut_clearance_ratio = std::clamp(options_.shortcut_clearance_ratio, 0.0, 1.0);
}

SurfacePlanResult SurfaceAstarPlanner::plan(
  const NavigationSnapshot & snapshot, const GridIndex & start, const GridIndex & goal) const
{
  SurfacePlanResult result;
  if (!isCellTraversable(snapshot, start)) {
    result.message = "start is not traversable";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (!isCellTraversable(snapshot, goal)) {
    result.message = "goal is not traversable";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (options_.require_footprint_support) {
    const Point3 start_point = cellCenter(start, snapshot.resolution_m);
    const Point3 goal_point = cellCenter(goal, snapshot.resolution_m);
    const double yaw = std::atan2(goal_point.y - start_point.y, goal_point.x - start_point.x);
    if (!isFootprintSupported(snapshot, start_point, yaw)) {
      result.message = "start footprint is not fully supported";
      result.metrics.failure_reason = result.message;
      return result;
    }
    if (!isFootprintSupported(snapshot, goal_point, yaw)) {
      result.message = "goal footprint is not fully supported";
      result.metrics.failure_reason = result.message;
      return result;
    }
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
          if (dx == 0 && dy == 0) {
            continue;
          }
          const GridIndex neighbor{current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          if (!isTransitionAllowed(snapshot, current.cell, neighbor)) {
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
  result.raw_cells = result.cells;
  result.raw_path = cellsToPath(result.raw_cells, snapshot.resolution_m);
  result.metrics.raw_path_waypoints = static_cast<std::uint32_t>(result.cells.size());
  for (std::size_t i = 1; i < result.cells.size(); ++i) {
    result.metrics.raw_path_length_m += gridDistance(result.cells[i - 1U], result.cells[i]) *
      snapshot.resolution_m;
  }
  if (options_.enable_shortcut) {
    const std::vector<GridIndex> shortcut_cells = shortcutPath(snapshot, result.cells);
    if (shortcut_cells.size() >= 2U && shortcut_cells.size() < result.cells.size()) {
      result.metrics.shortcut_count =
        static_cast<std::uint32_t>(result.cells.size() - shortcut_cells.size());
      result.cells = shortcut_cells;
    }
  }
  result.path = cellsToPath(result.cells, snapshot.resolution_m);
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
  const double risk_penalty = options_.w_risk * snapshot.risk.riskCost(to);
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

  return step + clearance_penalty + risk_penalty + slope_penalty + turn_penalty;
}

bool SurfaceAstarPlanner::isCellTraversable(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  return snapshot.surface.traversable_cells.find(cell) != snapshot.surface.traversable_cells.end() &&
         snapshot.surface.blocked_cells.find(cell) == snapshot.surface.blocked_cells.end() &&
         snapshot.surface.forbidden_cells.find(cell) == snapshot.surface.forbidden_cells.end();
}

bool SurfaceAstarPlanner::isFootprintSupported(
  const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const
{
  if (!options_.require_footprint_support) {
    return true;
  }
  for (const Point3 & sample :
    footprint_.sampleFootprint(point, yaw_rad, snapshot.resolution_m))
  {
    if (!isCellTraversable(snapshot, worldToGrid(sample, snapshot.resolution_m))) {
      return false;
    }
  }
  return true;
}

bool SurfaceAstarPlanner::isTransitionAllowed(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const
{
  if (!isCellTraversable(snapshot, to)) {
    return false;
  }
  if (!options_.require_footprint_support) {
    return true;
  }

  const Point3 from_point = cellCenter(from, snapshot.resolution_m);
  const Point3 to_point = cellCenter(to, snapshot.resolution_m);
  const double segment_length = distance3d(from_point, to_point);
  const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / options_.swept_sample_step_m)));
  const double yaw = std::atan2(to_point.y - from_point.y, to_point.x - from_point.x);
  for (int step = 0; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Point3 sample{
      from_point.x + (to_point.x - from_point.x) * t,
      from_point.y + (to_point.y - from_point.y) * t,
      from_point.z + (to_point.z - from_point.z) * t};
    if (!isFootprintSupported(snapshot, sample, yaw)) {
      return false;
    }
  }
  return true;
}

Point3 SurfaceAstarPlanner::cellCenter(const GridIndex & cell, double resolution_m) const
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

GridIndex SurfaceAstarPlanner::worldToGrid(const Point3 & point, double resolution_m) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

std::vector<Point3> SurfaceAstarPlanner::cellsToPath(
  const std::vector<GridIndex> & cells, double resolution_m) const
{
  std::vector<Point3> path;
  path.reserve(cells.size());
  for (const GridIndex & cell : cells) {
    path.push_back(cellCenter(cell, resolution_m));
  }
  return path;
}

std::vector<GridIndex> SurfaceAstarPlanner::shortcutPath(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells) const
{
  if (raw_cells.size() < 3U) {
    return raw_cells;
  }

  std::vector<GridIndex> simplified;
  simplified.reserve(raw_cells.size());
  std::size_t i = 0U;
  simplified.push_back(raw_cells.front());
  while (i + 1U < raw_cells.size()) {
    std::size_t best = i + 1U;
    for (std::size_t candidate = raw_cells.size() - 1U; candidate > i + 1U; --candidate) {
      if (isShortcutAllowed(snapshot, raw_cells, i, candidate)) {
        best = candidate;
        break;
      }
    }
    simplified.push_back(raw_cells[best]);
    i = best;
  }
  return simplified;
}

bool SurfaceAstarPlanner::isShortcutAllowed(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
  std::size_t from_index, std::size_t to_index) const
{
  const Point3 from = cellCenter(raw_cells[from_index], snapshot.resolution_m);
  const Point3 to = cellCenter(raw_cells[to_index], snapshot.resolution_m);
  const double segment_length = distance3d(from, to);
  if (segment_length <= snapshot.resolution_m) {
    return true;
  }

  const double raw_min_clearance = minRawClearance(snapshot, raw_cells, from_index, to_index);
  const double required_clearance = std::max(
    raw_min_clearance * options_.shortcut_clearance_ratio,
    options_.footprint.width_m * 0.5 + options_.shortcut_safety_margin_m);
  const double yaw = std::atan2(to.y - from.y, to.x - from.x);
  const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / options_.shortcut_sample_step_m)));
  double shortcut_min_clearance = std::numeric_limits<double>::infinity();
  GridIndex previous_cell = raw_cells[from_index];
  for (int step = 0; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Point3 sample{
      from.x + (to.x - from.x) * t,
      from.y + (to.y - from.y) * t,
      from.z + (to.z - from.z) * t};
    const GridIndex sample_cell = worldToGrid(sample, snapshot.resolution_m);
    if (!isCellTraversable(snapshot, sample_cell)) {
      return false;
    }
    if (!isFootprintSupported(snapshot, sample, yaw)) {
      return false;
    }
    if (step > 0 && !isTransitionAllowed(snapshot, previous_cell, sample_cell)) {
      return false;
    }
    shortcut_min_clearance =
      std::min(shortcut_min_clearance, snapshot.clearance.clearanceDistance(sample_cell));
    previous_cell = sample_cell;
  }
  return shortcut_min_clearance >= required_clearance;
}

double SurfaceAstarPlanner::minRawClearance(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
  std::size_t from_index, std::size_t to_index) const
{
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = from_index; i <= to_index; ++i) {
    min_clearance = std::min(min_clearance, snapshot.clearance.clearanceDistance(raw_cells[i]));
  }
  return std::isfinite(min_clearance) ? min_clearance : 0.0;
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
    const double risk = snapshot.risk.riskCost(result.cells[i]);
    result.metrics.risk_cost_sum += risk;
    result.metrics.max_path_risk = std::max(result.metrics.max_path_risk, risk);
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
