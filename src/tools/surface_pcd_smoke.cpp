#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/probabilistic_voxel_map.hpp"
#include "tgw_planner/core/risk_field.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace
{
using tgw_planner::core::GridIndex;
using tgw_planner::core::GridIndexHash;
using tgw_planner::core::MappingOptions;
using tgw_planner::core::NavigationSnapshot;
using tgw_planner::core::Point3;
using tgw_planner::core::ProbabilisticVoxelMap;
using tgw_planner::core::RiskField;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfaceExtractionOptions;
using tgw_planner::core::SurfaceExtractor;
using tgw_planner::core::SurfacePlannerOptions;

double argDouble(char ** argv, int index)
{
  return std::strtod(argv[index], nullptr);
}

bool snapToTraversable(
  const NavigationSnapshot & snapshot, const Point3 & point, GridIndex & snapped,
  double & distance_m)
{
  const GridIndex seed{
    static_cast<int>(std::floor(point.x / snapshot.resolution_m)),
    static_cast<int>(std::floor(point.y / snapshot.resolution_m)),
    static_cast<int>(std::floor(point.z / snapshot.resolution_m))};
  const int max_radius_cells = std::max(3, static_cast<int>(std::ceil(2.0 / snapshot.resolution_m)));
  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();
  GridIndex best;
  for (int radius = 0; radius <= max_radius_cells; ++radius) {
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const GridIndex candidate{seed.x + dx, seed.y + dy, seed.z + dz};
          if (snapshot.surface.traversable_cells.find(candidate) ==
            snapshot.surface.traversable_cells.end())
          {
            continue;
          }
          const Point3 world{
            (static_cast<double>(candidate.x) + 0.5) * snapshot.resolution_m,
            (static_cast<double>(candidate.y) + 0.5) * snapshot.resolution_m,
            (static_cast<double>(candidate.z) + 0.5) * snapshot.resolution_m};
          const double d = tgw_planner::core::distance3d(point, world);
          if (d < best_distance) {
            best_distance = d;
            best = candidate;
            found = true;
          }
        }
      }
    }
    if (found) {
      snapped = best;
      distance_m = best_distance;
      return true;
    }
  }
  return false;
}

GridIndex worldToGrid(const Point3 & point, double resolution_m)
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

Point3 cellCenter(const GridIndex & cell, double resolution_m)
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

bool segmentTraversable(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to)
{
  const Point3 from_point = cellCenter(from, snapshot.resolution_m);
  const Point3 to_point = cellCenter(to, snapshot.resolution_m);
  const double segment_length = tgw_planner::core::distance3d(from_point, to_point);
  const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / 0.05)));
  for (int step = 0; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Point3 sample{
      from_point.x + (to_point.x - from_point.x) * t,
      from_point.y + (to_point.y - from_point.y) * t,
      from_point.z + (to_point.z - from_point.z) * t};
    if (snapshot.surface.traversable_cells.find(worldToGrid(sample, snapshot.resolution_m)) ==
      snapshot.surface.traversable_cells.end())
    {
      return false;
    }
  }
  return true;
}

struct ComponentSummary
{
  int start_component{-1};
  int goal_component{-1};
  std::size_t start_component_size{0U};
  std::size_t goal_component_size{0U};
  std::size_t component_count{0U};
  std::size_t largest_component_size{0U};
  double start_goal_component_gap_m{0.0};
  GridIndex gap_start_cell;
  GridIndex gap_goal_cell;
  std::size_t gap_line_cells{0U};
  std::size_t gap_line_occupied_cells{0U};
  std::size_t gap_line_surface_cells{0U};
  std::size_t gap_line_traversable_cells{0U};
  std::size_t gap_line_forbidden_cells{0U};
};

std::vector<GridIndex> rasterLine(const GridIndex & from, const GridIndex & to)
{
  const int steps = std::max({
    std::abs(to.x - from.x), std::abs(to.y - from.y), std::abs(to.z - from.z), 1});
  std::vector<GridIndex> cells;
  cells.reserve(static_cast<std::size_t>(steps + 1));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    cells.push_back({
      static_cast<int>(std::lround(static_cast<double>(from.x) + (to.x - from.x) * t)),
      static_cast<int>(std::lround(static_cast<double>(from.y) + (to.y - from.y) * t)),
      static_cast<int>(std::lround(static_cast<double>(from.z) + (to.z - from.z) * t))});
  }
  cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
  return cells;
}

ComponentSummary summarizeComponents(
  const NavigationSnapshot & snapshot, const ProbabilisticVoxelMap & map,
  const GridIndex & start, const GridIndex & goal)
{
  ComponentSummary summary;
  std::unordered_map<GridIndex, int, GridIndexHash> component_by_cell;
  std::vector<std::size_t> component_sizes;
  const int max_step_cells =
    std::max(1, static_cast<int>(std::ceil(0.30 / snapshot.resolution_m)));

  for (const GridIndex & seed : snapshot.surface.traversable_cells) {
    if (component_by_cell.find(seed) != component_by_cell.end()) {
      continue;
    }
    const int component_id = static_cast<int>(component_sizes.size());
    component_sizes.push_back(0U);
    std::queue<GridIndex> queue;
    queue.push(seed);
    component_by_cell[seed] = component_id;
    while (!queue.empty()) {
      const GridIndex current = queue.front();
      queue.pop();
      ++component_sizes[component_id];
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            if (dx == 0 && dy == 0) {
              continue;
            }
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (snapshot.surface.traversable_cells.find(neighbor) ==
              snapshot.surface.traversable_cells.end())
            {
              continue;
            }
            if (component_by_cell.find(neighbor) != component_by_cell.end()) {
              continue;
            }
            if (!segmentTraversable(snapshot, current, neighbor)) {
              continue;
            }
            component_by_cell[neighbor] = component_id;
            queue.push(neighbor);
          }
        }
      }
    }
  }

  summary.component_count = component_sizes.size();
  for (const std::size_t size : component_sizes) {
    summary.largest_component_size = std::max(summary.largest_component_size, size);
  }
  const auto start_it = component_by_cell.find(start);
  if (start_it != component_by_cell.end()) {
    summary.start_component = start_it->second;
    summary.start_component_size = component_sizes[static_cast<std::size_t>(start_it->second)];
  }
  const auto goal_it = component_by_cell.find(goal);
  if (goal_it != component_by_cell.end()) {
    summary.goal_component = goal_it->second;
    summary.goal_component_size = component_sizes[static_cast<std::size_t>(goal_it->second)];
  }
  if (summary.start_component >= 0 && summary.goal_component >= 0 &&
    summary.start_component != summary.goal_component)
  {
    std::vector<GridIndex> start_cells;
    std::vector<GridIndex> goal_cells;
    start_cells.reserve(summary.start_component_size);
    goal_cells.reserve(summary.goal_component_size);
    for (const auto & entry : component_by_cell) {
      if (entry.second == summary.start_component) {
        start_cells.push_back(entry.first);
      } else if (entry.second == summary.goal_component) {
        goal_cells.push_back(entry.first);
      }
    }

    double best_distance_cells = std::numeric_limits<double>::infinity();
    for (const GridIndex & a : start_cells) {
      for (const GridIndex & b : goal_cells) {
        const double dx = static_cast<double>(a.x - b.x);
        const double dy = static_cast<double>(a.y - b.y);
        const double dz = static_cast<double>(a.z - b.z);
        const double distance_cells = dx * dx + dy * dy + dz * dz;
        if (distance_cells < best_distance_cells) {
          best_distance_cells = distance_cells;
          summary.gap_start_cell = a;
          summary.gap_goal_cell = b;
        }
      }
    }
    if (std::isfinite(best_distance_cells)) {
      summary.start_goal_component_gap_m = std::sqrt(best_distance_cells) * snapshot.resolution_m;
      for (const GridIndex & cell : rasterLine(summary.gap_start_cell, summary.gap_goal_cell)) {
        ++summary.gap_line_cells;
        if (map.isOccupied(cell)) {
          ++summary.gap_line_occupied_cells;
        }
        if (snapshot.surface.surface_cells.find(cell) != snapshot.surface.surface_cells.end()) {
          ++summary.gap_line_surface_cells;
        }
        if (snapshot.surface.traversable_cells.find(cell) !=
          snapshot.surface.traversable_cells.end())
        {
          ++summary.gap_line_traversable_cells;
        }
        if (snapshot.surface.forbidden_cells.find(cell) != snapshot.surface.forbidden_cells.end()) {
          ++summary.gap_line_forbidden_cells;
        }
      }
    }
  }
  return summary;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 9) {
    std::cerr << "usage: tgw_surface_pcd_smoke <pcd> <resolution_m> "
      "<start_x> <start_y> <start_z> <goal_x> <goal_y> <goal_z> "
      "[require_footprint=0]\n";
    return 2;
  }

  const std::string pcd_path = argv[1];
  const double resolution_m = argDouble(argv, 2);
  const Point3 start{argDouble(argv, 3), argDouble(argv, 4), argDouble(argv, 5)};
  const Point3 goal{argDouble(argv, 6), argDouble(argv, 7), argDouble(argv, 8)};
  const bool require_footprint = argc >= 10 && std::string(argv[9]) == "1";

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_path, cloud) != 0) {
    std::cerr << "failed to load PCD: " << pcd_path << "\n";
    return 1;
  }

  MappingOptions map_options;
  map_options.resolution_m = resolution_m;
  map_options.min_static_hits = 1;
  map_options.min_distinct_views = 1;
  map_options.min_static_lifetime_sec = 0.0;
  map_options.enable_dynamic_filter = false;
  ProbabilisticVoxelMap map(map_options);
  for (const auto & point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    map.updateHit(map.worldToGrid({point.x, point.y, point.z}), 0.0, 1);
  }

  SurfaceExtractionOptions surface_options;
  surface_options.min_static_hits = 1;
  surface_options.require_static_support = false;
  surface_options.require_observed_free_space = false;
  SurfaceExtractor extractor(surface_options);

  NavigationSnapshot snapshot;
  snapshot.resolution_m = resolution_m;
  snapshot.surface = extractor.extract(map);
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  RiskField risk;
  risk.compute(snapshot.surface, snapshot.clearance);
  snapshot.risk = risk;

  GridIndex snapped_start;
  GridIndex snapped_goal;
  double start_snap = 0.0;
  double goal_snap = 0.0;
  const bool start_ok = snapToTraversable(snapshot, start, snapped_start, start_snap);
  const bool goal_ok = snapToTraversable(snapshot, goal, snapped_goal, goal_snap);
  if (!start_ok || !goal_ok) {
    std::cout << "success=false reason=snap_failed"
      << " start_ok=" << (start_ok ? "true" : "false")
      << " goal_ok=" << (goal_ok ? "true" : "false")
      << " occupied_voxels=" << map.occupiedVoxels().size()
      << " traversable_cells=" << snapshot.surface.traversable_cells.size()
      << "\n";
    return 1;
  }

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = require_footprint;
  planner_options.enable_shortcut = true;
  planner_options.w_clearance = 1.2;
  SurfaceAstarPlanner planner(planner_options);
  const ComponentSummary components = summarizeComponents(snapshot, map, snapped_start, snapped_goal);
  const auto result = planner.plan(snapshot, snapped_start, snapped_goal);

  std::cout << "success=" << (result.success ? "true" : "false")
    << " reason=\"" << result.metrics.failure_reason << "\""
    << " source_points=" << cloud.size()
    << " occupied_voxels=" << map.occupiedVoxels().size()
    << " surface_cells=" << snapshot.surface.surface_cells.size()
    << " traversable_cells=" << snapshot.surface.traversable_cells.size()
    << " boundary_cells=" << snapshot.surface.boundary_cells.size()
    << " risk_cells=" << snapshot.risk.risks().size()
    << " start_snap_distance_m=" << start_snap
    << " goal_snap_distance_m=" << goal_snap
    << " surface_component_count=" << components.component_count
    << " largest_surface_component_size=" << components.largest_component_size
    << " start_surface_component=" << components.start_component
    << " goal_surface_component=" << components.goal_component
    << " start_surface_component_size=" << components.start_component_size
    << " goal_surface_component_size=" << components.goal_component_size
    << " start_goal_component_gap_m=" << components.start_goal_component_gap_m
    << " gap_start_cell=[" << components.gap_start_cell.x << "," <<
      components.gap_start_cell.y << "," << components.gap_start_cell.z << "]"
    << " gap_goal_cell=[" << components.gap_goal_cell.x << "," <<
      components.gap_goal_cell.y << "," << components.gap_goal_cell.z << "]"
    << " gap_line_cells=" << components.gap_line_cells
    << " gap_line_occupied_cells=" << components.gap_line_occupied_cells
    << " gap_line_surface_cells=" << components.gap_line_surface_cells
    << " gap_line_traversable_cells=" << components.gap_line_traversable_cells
    << " gap_line_forbidden_cells=" << components.gap_line_forbidden_cells
    << " expanded_nodes=" << result.metrics.expanded_nodes
    << " raw_path_waypoints=" << result.metrics.raw_path_waypoints
    << " raw_path_length_m=" << result.metrics.raw_path_length_m
    << " shortcut_count=" << result.metrics.shortcut_count
    << " final_path_validated=" << (result.metrics.final_path_validated ? "true" : "false")
    << " final_path_fallback_to_raw=" <<
      (result.metrics.final_path_fallback_to_raw ? "true" : "false")
    << " path_waypoints=" << result.path.size()
    << " path_length_m=" << result.metrics.path_length_m
    << " min_path_clearance_m=" << result.metrics.min_path_clearance_m
    << " mean_path_clearance_m=" << result.metrics.mean_path_clearance_m
    << "\n";
  return result.success ? 0 : 1;
}
