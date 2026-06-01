#include "tgw_planner/core/voxel_astar_planner.hpp"

#include <algorithm>
#include <chrono>
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
  GridIndex idx;
  double f{0.0};
  double g{0.0};
};

struct QueueNodeCompare
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

double pointDistance(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

VoxelAstarPlanner::VoxelAstarPlanner(std::uint32_t max_iterations)
: max_iterations_(std::max<std::uint32_t>(1, max_iterations))
{
}

PlanResult VoxelAstarPlanner::plan(
  const NavigationMap & map, const Point3 & start, const Point3 & goal) const
{
  const auto total_t0 = std::chrono::steady_clock::now();
  PlanResult result;

  if (!map.ready()) {
    result.message = "navigation map is not ready";
    result.metrics.failure_reason = result.message;
    return result;
  }

  if (!snapToTraversable(map, start, result.start_cell, result.metrics.start_snap_distance_m)) {
    result.message = "start cannot be snapped to a traversable cell";
    result.metrics.failure_reason = result.message;
    return result;
  }
  result.start_snap_success = true;
  if (!snapToTraversable(map, goal, result.goal_cell, result.metrics.goal_snap_distance_m)) {
    result.message = "goal cannot be snapped to a traversable cell";
    result.metrics.failure_reason = result.message;
    return result;
  }
  result.goal_snap_success = true;

  const auto search_t0 = std::chrono::steady_clock::now();
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed;

  g_score[result.start_cell] = 0.0;
  open_set.push({result.start_cell, gridDistance(result.start_cell, result.goal_cell) * map.resolution(), 0.0});

  bool found = false;
  while (!open_set.empty() && result.metrics.expanded_nodes < max_iterations_) {
    result.metrics.max_open_set_size =
      std::max(result.metrics.max_open_set_size, static_cast<std::uint32_t>(open_set.size()));
    const QueueNode current = open_set.top();
    open_set.pop();

    const auto best_g = g_score.find(current.idx);
    if (best_g == g_score.end() || current.g > best_g->second + 1.0e-9) {
      continue;
    }

    if (current.idx == result.goal_cell) {
      found = true;
      break;
    }

    if (!closed.insert(current.idx).second) {
      continue;
    }
    ++result.metrics.expanded_nodes;

    const int max_step_cells = map.maxStepCells();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const GridIndex neighbor{current.idx.x + dx, current.idx.y + dy, current.idx.z + dz};
          if (!map.isTraversable(neighbor)) {
            continue;
          }
          if (std::abs(dz) > 1 && !map.isStairTraversable(neighbor) &&
            !map.hasContinuousSupport(neighbor))
          {
            continue;
          }
          const double vertical_penalty =
            std::abs(dz) > 1 ? 0.05 * static_cast<double>(std::abs(dz) - 1) : 0.0;
          const double step_cost = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) *
            map.resolution() + vertical_penalty;
          const double tentative_g = current.g + step_cost + map.getRiskCost(neighbor);
          const auto old_g = g_score.find(neighbor);
          if (old_g != g_score.end() && tentative_g >= old_g->second) {
            continue;
          }
          if (closed.find(neighbor) != closed.end()) {
            ++result.metrics.reopened_nodes;
          }
          came_from[neighbor] = current.idx;
          g_score[neighbor] = tentative_g;
          const double h = gridDistance(neighbor, result.goal_cell) * map.resolution();
          open_set.push({neighbor, tentative_g + h, tentative_g});
          ++result.metrics.generated_nodes;
        }
      }
    }
  }

  const auto search_t1 = std::chrono::steady_clock::now();
  result.metrics.search_time_ms =
    std::chrono::duration<double, std::milli>(search_t1 - search_t0).count();

  if (!found) {
    result.message = result.metrics.expanded_nodes >= max_iterations_ ?
      "A* reached max_iterations before finding a path" : "A* failed to find a path";
    result.metrics.failure_reason = result.message;
    const auto total_t1 = std::chrono::steady_clock::now();
    result.metrics.total_plan_time_ms =
      std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
    return result;
  }

  result.path = reconstructPath(map, came_from, result.start_cell, result.goal_cell);
  fillPathMetrics(result.path, result.metrics);
  result.success = true;
  result.message = "ok";
  result.metrics.success = true;
  const auto total_t1 = std::chrono::steady_clock::now();
  result.metrics.total_plan_time_ms =
    std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
  return result;
}

bool VoxelAstarPlanner::snapToTraversable(
  const NavigationMap & map, const Point3 & seed, GridIndex & snapped,
  double & snap_distance_m) const
{
  const GridIndex center = map.worldToGrid(seed);
  const GridIndex min_idx = map.worldToGrid(map.boundsMin());
  const GridIndex max_idx = map.worldToGrid(map.boundsMax());
  const int min_z = min_idx.z - 1;
  const int max_z = max_idx.z + static_cast<int>(std::ceil(map.robotHeight() / map.resolution()));
  const int xy_radius_cells = std::max(
    1, static_cast<int>(std::ceil(std::max(0.8, 2.0 * map.robotRadius()) / map.resolution())));

  for (int r = 0; r <= xy_radius_cells; ++r) {
    bool found_in_ring = false;
    double best_z_distance = std::numeric_limits<double>::infinity();
    double best_xy_distance = std::numeric_limits<double>::infinity();
    double best_signed_z = std::numeric_limits<double>::infinity();
    GridIndex best;

    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const double xy_distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
          map.resolution();
        if (xy_distance > std::max(0.8, 2.0 * map.robotRadius()) + 1.0e-9) {
          continue;
        }

        for (int z = min_z; z <= max_z; ++z) {
          const GridIndex candidate{center.x + dx, center.y + dy, z};
          if (!map.isTraversable(candidate)) {
            continue;
          }
          const Point3 candidate_point = map.gridToWorld(candidate);
          const double signed_z = candidate_point.z - seed.z;
          const double z_distance = std::abs(signed_z);
          const bool better =
            !found_in_ring || z_distance < best_z_distance - 1.0e-9 ||
            (std::abs(z_distance - best_z_distance) <= 1.0e-9 &&
            xy_distance < best_xy_distance - 1.0e-9) ||
            (std::abs(z_distance - best_z_distance) <= 1.0e-9 &&
            std::abs(xy_distance - best_xy_distance) <= 1.0e-9 && signed_z < best_signed_z);
          if (better) {
            found_in_ring = true;
            best_z_distance = z_distance;
            best_xy_distance = xy_distance;
            best_signed_z = signed_z;
            best = candidate;
          }
        }
      }
    }

    if (found_in_ring) {
      snapped = best;
      snap_distance_m = pointDistance(seed, map.gridToWorld(best));
      return true;
    }
  }

  const double snap_radius_m = std::max(1.0, 3.0 * map.robotRadius());
  const int radius_cells = std::max(1, static_cast<int>(std::ceil(snap_radius_m / map.resolution())));

  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  GridIndex best;

  for (int r = 0; r <= radius_cells; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
            continue;
          }
          const GridIndex candidate{center.x + dx, center.y + dy, center.z + dz};
          if (!map.isTraversable(candidate)) {
            continue;
          }
          const double distance = pointDistance(seed, map.gridToWorld(candidate));
          if (distance < best_distance) {
            best_distance = distance;
            best = candidate;
            found = true;
          }
        }
      }
    }
    if (found) {
      snapped = best;
      snap_distance_m = best_distance;
      return true;
    }
  }

  return false;
}

std::vector<Point3> VoxelAstarPlanner::reconstructPath(
  const NavigationMap & map,
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
  const GridIndex & start, const GridIndex & goal) const
{
  std::vector<GridIndex> cells;
  GridIndex current = goal;
  cells.push_back(current);

  while (current != start) {
    const auto it = came_from.find(current);
    if (it == came_from.end()) {
      break;
    }
    current = it->second;
    cells.push_back(current);
  }

  std::reverse(cells.begin(), cells.end());
  std::vector<Point3> path;
  path.reserve(cells.size());
  for (const auto & cell : cells) {
    path.push_back(map.gridToWorld(cell));
  }
  return path;
}

void VoxelAstarPlanner::fillPathMetrics(
  const std::vector<Point3> & path, PlannerMetrics & metrics) const
{
  metrics.path_waypoints = static_cast<std::uint32_t>(path.size());
  metrics.path_length_m = 0.0;
  metrics.path_vertical_gain_m = 0.0;
  metrics.path_vertical_loss_m = 0.0;

  for (std::size_t i = 1; i < path.size(); ++i) {
    metrics.path_length_m += pointDistance(path[i - 1], path[i]);
    const double dz = path[i].z - path[i - 1].z;
    if (dz > 0.0) {
      metrics.path_vertical_gain_m += dz;
    } else {
      metrics.path_vertical_loss_m += -dz;
    }
  }
}

}  // namespace tgw_planner::core
