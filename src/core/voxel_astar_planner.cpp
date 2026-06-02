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
struct SearchState
{
  GridIndex cell;
  int stair_flight_id{-1};

  bool operator==(const SearchState & other) const
  {
    return cell == other.cell && stair_flight_id == other.stair_flight_id;
  }
};

struct SearchStateHash
{
  std::size_t operator()(const SearchState & state) const
  {
    std::size_t seed = GridIndexHash{}(state.cell);
    seed ^= std::hash<int>{}(state.stair_flight_id) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct QueueNode
{
  SearchState state;
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
  std::unordered_map<SearchState, double, SearchStateHash> g_score;
  std::unordered_map<SearchState, SearchState, SearchStateHash> came_from;
  std::unordered_set<SearchState, SearchStateHash> closed;

  const SearchState start_state{result.start_cell, map.stairFlightId(result.start_cell)};
  SearchState goal_state{result.goal_cell, map.stairFlightId(result.goal_cell)};
  g_score[start_state] = 0.0;
  open_set.push({start_state, gridDistance(result.start_cell, result.goal_cell) * map.resolution(), 0.0});

  bool found = false;
  while (!open_set.empty() && result.metrics.expanded_nodes < max_iterations_) {
    result.metrics.max_open_set_size =
      std::max(result.metrics.max_open_set_size, static_cast<std::uint32_t>(open_set.size()));
    const QueueNode current = open_set.top();
    open_set.pop();

    const auto best_g = g_score.find(current.state);
    if (best_g == g_score.end() || current.g > best_g->second + 1.0e-9) {
      continue;
    }

    if (current.state.cell == result.goal_cell) {
      goal_state = current.state;
      found = true;
      break;
    }

    if (!closed.insert(current.state).second) {
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
          const GridIndex neighbor{
            current.state.cell.x + dx, current.state.cell.y + dy, current.state.cell.z + dz};
          if (!map.isTraversable(neighbor)) {
            continue;
          }
          if (std::abs(dz) > 1 && !map.isStairTraversable(neighbor) &&
            !map.hasContinuousSupport(neighbor))
          {
            continue;
          }
          if (!map.isStairTransitionAllowed(current.state.cell, neighbor)) {
            continue;
          }
          if (!map.isFootprintTransitionSupported(current.state.cell, neighbor)) {
            continue;
          }
          const SearchState neighbor_state{neighbor, map.stairFlightId(neighbor)};
          const double vertical_penalty =
            std::abs(dz) > 1 ? 0.05 * static_cast<double>(std::abs(dz) - 1) : 0.0;
          const bool touches_stair =
            map.isStairTraversable(current.state.cell) || map.isStairTraversable(neighbor);
          const double stair_diagonal_penalty =
            touches_stair && dx != 0 && dy != 0 ? 0.4 : 0.0;
          const double stair_center_cost =
            0.5 * (map.getStairCenterCost(current.state.cell) + map.getStairCenterCost(neighbor));
          const double step_cost = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) *
            map.resolution() + vertical_penalty + stair_diagonal_penalty + stair_center_cost;
          const double tentative_g = current.g + step_cost + map.getRiskCost(neighbor);
          const auto old_g = g_score.find(neighbor_state);
          if (old_g != g_score.end() && tentative_g >= old_g->second) {
            continue;
          }
          if (closed.find(neighbor_state) != closed.end()) {
            ++result.metrics.reopened_nodes;
          }
          came_from[neighbor_state] = current.state;
          g_score[neighbor_state] = tentative_g;
          const double h = gridDistance(neighbor, result.goal_cell) * map.resolution();
          open_set.push({neighbor_state, tentative_g + h, tentative_g});
          ++result.metrics.generated_nodes;
        }
      }
    }
  }

  const auto search_t1 = std::chrono::steady_clock::now();
  result.metrics.search_time_ms =
    std::chrono::duration<double, std::milli>(search_t1 - search_t0).count();

  if (!found) {
    double closest_distance_cells = std::numeric_limits<double>::infinity();
    for (const auto & state : closed) {
      const double distance_cells = gridDistance(state.cell, result.goal_cell);
      if (distance_cells < closest_distance_cells) {
        closest_distance_cells = distance_cells;
        result.closest_closed_cell = state.cell;
        result.closest_closed_cell_valid = true;
      }
    }
    if (result.closest_closed_cell_valid) {
      result.closest_closed_distance_m = closest_distance_cells * map.resolution();
    }
    result.message = result.metrics.expanded_nodes >= max_iterations_ ?
      "A* reached max_iterations before finding a path" : "A* failed to find a path";
    result.metrics.failure_reason = result.message;
    const auto total_t1 = std::chrono::steady_clock::now();
    result.metrics.total_plan_time_ms =
      std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
    return result;
  }

  std::vector<GridIndex> cells;
  SearchState current_state = goal_state;
  cells.push_back(current_state.cell);
  while (!(current_state == start_state)) {
    const auto it = came_from.find(current_state);
    if (it == came_from.end()) {
      break;
    }
    current_state = it->second;
    cells.push_back(current_state.cell);
  }
  std::reverse(cells.begin(), cells.end());
  const std::vector<Point3> raw_path = rawPathFromCells(map, cells);
  PlannerMetrics raw_metrics;
  fillPathMetrics(raw_path, raw_metrics);
  result.metrics.raw_path_waypoints = raw_metrics.path_waypoints;
  result.metrics.raw_path_length_m = raw_metrics.path_length_m;
  result.metrics.raw_path_vertical_gain_m = raw_metrics.path_vertical_gain_m;
  result.metrics.raw_path_vertical_loss_m = raw_metrics.path_vertical_loss_m;

  PathPostprocessStats postprocess_stats;
  std::vector<Point3> processed_path = postProcessPath(map, cells, postprocess_stats);
  result.metrics.postprocess_floor_shortcuts = postprocess_stats.floor_shortcuts;
  result.metrics.postprocess_stair_centerline_replacements =
    postprocess_stats.stair_centerline_replacements;

  std::string validation_failure;
  if (validateFinalPath(map, processed_path, validation_failure)) {
    result.path = std::move(processed_path);
    result.metrics.final_path_validated = true;
  } else {
    const std::string postprocess_failure = validation_failure;
    std::string raw_validation_failure;
    if (validateFinalPath(map, raw_path, raw_validation_failure)) {
      result.path = raw_path;
      result.metrics.final_path_validated = true;
      result.metrics.final_path_fallback_to_raw = true;
      result.metrics.final_path_validation_failure = postprocess_failure;
    } else {
      result.path = raw_path;
      result.metrics.final_path_validated = false;
      result.metrics.final_path_fallback_to_raw = true;
      result.metrics.final_path_validation_failure = postprocess_failure;
      result.success = false;
      result.message = "final voxel path validation failed: " + postprocess_failure +
        "; raw fallback failed: " + raw_validation_failure;
      result.metrics.failure_reason = result.message;
      const auto total_t1 = std::chrono::steady_clock::now();
      result.metrics.total_plan_time_ms =
        std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
      return result;
    }
  }
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

std::vector<Point3> VoxelAstarPlanner::postProcessPath(
  const NavigationMap & map, const std::vector<GridIndex> & cells,
  PathPostprocessStats & stats) const
{
  if (cells.empty()) {
    return {};
  }

  std::vector<Point3> path;
  auto append_points = [&](const std::vector<Point3> & points) {
    for (const auto & point : points) {
      if (!path.empty() && map.worldToGrid(path.back()) == map.worldToGrid(point)) {
        continue;
      }
      path.push_back(point);
    }
  };

  std::size_t run_begin = 0U;
  while (run_begin < cells.size()) {
    const int flight_id = map.stairFlightId(cells[run_begin]);
    std::size_t run_end = run_begin + 1U;
    while (run_end < cells.size() && map.stairFlightId(cells[run_end]) == flight_id) {
      ++run_end;
    }

    std::vector<GridIndex> run_cells(cells.begin() + static_cast<std::ptrdiff_t>(run_begin),
      cells.begin() + static_cast<std::ptrdiff_t>(run_end));
    const auto raw_run_points = rawPathFromCells(map, run_cells);
    std::vector<Point3> run_points;
    if (flight_id >= 0) {
      run_points = centerlineStairRun(map, flight_id, run_cells);
      std::string validation_failure;
      if (!validateFinalPath(map, run_points, validation_failure)) {
        run_points = raw_run_points;
      }
      if (run_points.size() < run_cells.size()) {
        ++stats.stair_centerline_replacements;
      }
    } else {
      run_points = simplifyFloorRun(map, run_cells);
      std::string validation_failure;
      if (!validateFinalPath(map, run_points, validation_failure)) {
        run_points = raw_run_points;
      }
      if (run_points.size() < run_cells.size()) {
        ++stats.floor_shortcuts;
      }
    }
    if (!path.empty() && !run_points.empty()) {
      std::vector<Point3> connection_check;
      connection_check.reserve(run_points.size() + 1U);
      connection_check.push_back(path.back());
      connection_check.insert(connection_check.end(), run_points.begin(), run_points.end());
      std::string validation_failure;
      if (!validateFinalPath(map, connection_check, validation_failure)) {
        run_points = raw_run_points;
      }
    }
    append_points(run_points);
    run_begin = run_end;
  }

  if (path.empty()) {
    path.push_back(map.gridToWorld(cells.front()));
    if (cells.size() > 1U) {
      path.push_back(map.gridToWorld(cells.back()));
    }
  }
  return path;
}

std::vector<Point3> VoxelAstarPlanner::simplifyFloorRun(
  const NavigationMap & map, const std::vector<GridIndex> & cells) const
{
  if (cells.empty()) {
    return {};
  }
  if (cells.size() <= 2U) {
    std::vector<Point3> points;
    points.reserve(cells.size());
    for (const auto & cell : cells) {
      points.push_back(map.gridToWorld(cell));
    }
    return points;
  }

  std::vector<Point3> simplified;
  simplified.push_back(map.gridToWorld(cells.front()));

  std::size_t current = 0U;
  while (current + 1U < cells.size()) {
    std::size_t best = current + 1U;
    const std::size_t max_lookahead =
      std::min(cells.size() - 1U, current + static_cast<std::size_t>(
        std::max(4, static_cast<int>(std::ceil(8.0 / map.resolution())))));
    for (std::size_t candidate = max_lookahead; candidate > current + 1U; --candidate) {
      if (isLineTraversable(
          map, map.gridToWorld(cells[current]), map.gridToWorld(cells[candidate]), true))
      {
        best = candidate;
        break;
      }
    }
    simplified.push_back(map.gridToWorld(cells[best]));
    current = best;
  }
  return simplified;
}

std::vector<Point3> VoxelAstarPlanner::centerlineStairRun(
  const NavigationMap & map, int stair_flight_id, const std::vector<GridIndex> & cells) const
{
  if (cells.empty()) {
    return {};
  }
  const auto & flights = map.stairFlights();
  if (stair_flight_id < 0 || static_cast<std::size_t>(stair_flight_id) >= flights.size() ||
    flights[static_cast<std::size_t>(stair_flight_id)].centerline.size() < 2U)
  {
    std::vector<Point3> fallback;
    fallback.reserve(cells.size());
    for (const auto & cell : cells) {
      fallback.push_back(map.gridToWorld(cell));
    }
    return fallback;
  }

  const auto & centerline = flights[static_cast<std::size_t>(stair_flight_id)].centerline;
  auto snap_to_safe_stair_cell = [&](const Point3 & point) {
    const GridIndex center = map.worldToGrid(point);
    const int xy_radius =
      std::max(2, static_cast<int>(std::ceil(0.45 / std::max(map.resolution(), 1.0e-6))));
    const int z_radius = std::max(1, map.maxStepCells());
    bool found = false;
    double best_distance = std::numeric_limits<double>::infinity();
    Point3 best_point = point;

    for (int dx = -xy_radius; dx <= xy_radius; ++dx) {
      for (int dy = -xy_radius; dy <= xy_radius; ++dy) {
        for (int dz = -z_radius; dz <= z_radius; ++dz) {
          const GridIndex candidate{center.x + dx, center.y + dy, center.z + dz};
          if (map.stairFlightId(candidate) != stair_flight_id ||
            !map.isInsideStairSafeCorridor(candidate, stair_flight_id) ||
            !map.isTraversable(candidate))
          {
            continue;
          }
          const Point3 candidate_point = map.gridToWorld(candidate);
          const double distance = pointDistance(point, candidate_point);
          if (!found || distance < best_distance) {
            found = true;
            best_distance = distance;
            best_point = candidate_point;
          }
        }
      }
    }
    return best_point;
  };

  const Point3 run_start = map.gridToWorld(cells.front());
  const Point3 run_end = map.gridToWorld(cells.back());
  auto nearest_centerline_index = [&](const Point3 & point) {
    std::size_t best = 0U;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < centerline.size(); ++i) {
      const double distance = pointDistance(point, centerline[i]);
      if (distance < best_distance) {
        best_distance = distance;
        best = i;
      }
    }
    return best;
  };

  std::size_t start_index = nearest_centerline_index(run_start);
  std::size_t end_index = nearest_centerline_index(run_end);
  std::vector<Point3> points;
  points.push_back(run_start);
  if (start_index <= end_index) {
    for (std::size_t i = start_index; i <= end_index; ++i) {
      points.push_back(snap_to_safe_stair_cell(centerline[i]));
    }
  } else {
    for (std::size_t i = start_index + 1U; i > end_index; --i) {
      points.push_back(snap_to_safe_stair_cell(centerline[i - 1U]));
    }
  }
  points.push_back(run_end);

  std::vector<Point3> resampled;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!resampled.empty() && pointDistance(resampled.back(), points[i]) <= 0.5 * map.resolution()) {
      continue;
    }
    resampled.push_back(points[i]);
  }
  return resampled;
}

bool VoxelAstarPlanner::isLineTraversable(
  const NavigationMap & map, const Point3 & from, const Point3 & to, bool require_floor) const
{
  const double distance = pointDistance(from, to);
  const int samples =
    std::max(1, static_cast<int>(std::ceil(distance / std::max(0.5 * map.resolution(), 1.0e-6))));
  GridIndex previous = map.worldToGrid(from);
  if (!map.isTraversable(previous) || (require_floor && map.stairFlightId(previous) >= 0)) {
    return false;
  }
  for (int i = 1; i <= samples; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples);
    const Point3 point{
      from.x + (to.x - from.x) * ratio,
      from.y + (to.y - from.y) * ratio,
      from.z + (to.z - from.z) * ratio};
    const GridIndex current = map.worldToGrid(point);
    if (current == previous) {
      continue;
    }
    if (!map.isTraversable(current) || (require_floor && map.stairFlightId(current) >= 0)) {
      return false;
    }
    if (!map.isStairTransitionAllowed(previous, current)) {
      return false;
    }
    if (!map.isFootprintTransitionSupported(previous, current)) {
      return false;
    }
    previous = current;
  }
  return true;
}

std::vector<Point3> VoxelAstarPlanner::rawPathFromCells(
  const NavigationMap & map, const std::vector<GridIndex> & cells) const
{
  std::vector<Point3> path;
  path.reserve(cells.size());
  for (const auto & cell : cells) {
    path.push_back(map.gridToWorld(cell));
  }
  return path;
}

bool VoxelAstarPlanner::validateFinalPath(
  const NavigationMap & map, const std::vector<Point3> & path, std::string & failure) const
{
  failure.clear();
  if (path.empty()) {
    failure = "empty_path";
    return false;
  }

  GridIndex previous = map.worldToGrid(path.front());
  if (!map.isTraversable(previous)) {
    failure = "start_not_traversable";
    return false;
  }

  auto validate_edge = [&](const GridIndex & from, const GridIndex & to, const std::size_t segment)
    -> bool
    {
      if (!map.isTraversable(to)) {
        failure = "sample_not_traversable segment=" + std::to_string(segment) + " cell=[" +
          std::to_string(to.x) + "," + std::to_string(to.y) + "," + std::to_string(to.z) + "]";
        return false;
      }
      if (!map.isStairTransitionAllowed(from, to)) {
        failure = "illegal_stair_transition segment=" + std::to_string(segment) + " from=[" +
          std::to_string(from.x) + "," + std::to_string(from.y) + "," +
          std::to_string(from.z) + "] to=[" + std::to_string(to.x) + "," +
          std::to_string(to.y) + "," + std::to_string(to.z) + "]";
        return false;
      }
      if (!map.isFootprintTransitionSupported(from, to)) {
        failure = "unsupported_footprint_transition segment=" + std::to_string(segment) +
          " from=[" + std::to_string(from.x) + "," + std::to_string(from.y) + "," +
          std::to_string(from.z) + "] to=[" + std::to_string(to.x) + "," +
          std::to_string(to.y) + "," + std::to_string(to.z) + "]";
        return false;
      }
      return true;
    };

  const auto is_search_edge = [&](const GridIndex & from, const GridIndex & to) {
      const int dx = to.x - from.x;
      const int dy = to.y - from.y;
      const int dz = to.z - from.z;
      return std::abs(dx) <= 1 && std::abs(dy) <= 1 && (dx != 0 || dy != 0) &&
             std::abs(dz) <= map.maxStepCells();
    };

  for (std::size_t segment = 1U; segment < path.size(); ++segment) {
    const Point3 & from = path[segment - 1U];
    const Point3 & to = path[segment];
    const GridIndex endpoint = map.worldToGrid(to);
    if (endpoint == previous) {
      continue;
    }
    if (is_search_edge(previous, endpoint)) {
      if (!validate_edge(previous, endpoint, segment)) {
        return false;
      }
      previous = endpoint;
      continue;
    }

    const double distance = pointDistance(from, to);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(distance / std::max(0.5 * map.resolution(), 1.0e-6))));

    for (int i = 1; i <= samples; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(samples);
      const Point3 point{
        from.x + (to.x - from.x) * ratio,
        from.y + (to.y - from.y) * ratio,
        from.z + (to.z - from.z) * ratio};
      const GridIndex current = map.worldToGrid(point);
      if (current == previous) {
        continue;
      }
      if (!is_search_edge(previous, current)) {
        failure = "sample_skips_search_edge segment=" + std::to_string(segment) + " from=[" +
          std::to_string(previous.x) + "," + std::to_string(previous.y) + "," +
          std::to_string(previous.z) + "] to=[" + std::to_string(current.x) + "," +
          std::to_string(current.y) + "," + std::to_string(current.z) + "]";
        return false;
      }
      if (!validate_edge(previous, current, segment)) {
        return false;
      }
      previous = current;
    }
  }

  return true;
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
