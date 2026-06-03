#include "tgw_planner/core/pcd_artifact_detector.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>

#include "tgw_planner/core/navigation_map.hpp"

namespace tgw_planner::core
{
namespace
{
struct ColumnStats
{
  int z_min{std::numeric_limits<int>::max()};
  int z_max{std::numeric_limits<int>::min()};
  int count{0};
};

std::int64_t keyFor(int x, int y)
{
  return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y);
}
}  // namespace

bool possibleVerticalPcdArtifactsDetected(
  const std::unordered_set<GridIndex, GridIndexHash> & occupied_cells, double resolution_m)
{
  std::unordered_map<std::int64_t, ColumnStats> columns;
  columns.reserve(occupied_cells.size());
  for (const auto & cell : occupied_cells) {
    auto & stats = columns[keyFor(cell.x, cell.y)];
    stats.z_min = std::min(stats.z_min, cell.z);
    stats.z_max = std::max(stats.z_max, cell.z);
    ++stats.count;
  }

  std::unordered_set<std::int64_t> candidates;
  candidates.reserve(columns.size());
  for (const auto & [key, stats] : columns) {
    const double height_m = static_cast<double>(stats.z_max - stats.z_min + 1) * resolution_m;
    if (stats.count >= 4 && height_m >= 0.60 && height_m <= 2.40) {
      candidates.insert(key);
    }
  }

  std::unordered_set<std::int64_t> visited;
  visited.reserve(candidates.size());
  for (const auto start_key : candidates) {
    if (visited.find(start_key) != visited.end()) {
      continue;
    }

    std::queue<std::int64_t> queue;
    queue.push(start_key);
    visited.insert(start_key);
    int component_size = 0;
    int z_min = std::numeric_limits<int>::max();
    int z_max = std::numeric_limits<int>::min();
    while (!queue.empty()) {
      const std::int64_t key = queue.front();
      queue.pop();
      ++component_size;
      const auto stats_it = columns.find(key);
      if (stats_it != columns.end()) {
        z_min = std::min(z_min, stats_it->second.z_min);
        z_max = std::max(z_max, stats_it->second.z_max);
      }

      const int x = static_cast<int>(key >> 32);
      const int y = static_cast<int>(static_cast<std::int32_t>(key & 0xffffffff));
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const std::int64_t neighbor_key = keyFor(x + dx, y + dy);
          if (candidates.find(neighbor_key) == candidates.end() ||
            visited.find(neighbor_key) != visited.end())
          {
            continue;
          }
          visited.insert(neighbor_key);
          queue.push(neighbor_key);
        }
      }
    }

    const double height_m = static_cast<double>(z_max - z_min + 1) * resolution_m;
    const double area_m2 = static_cast<double>(component_size) * resolution_m * resolution_m;
    if (component_size >= 4 && component_size <= 80 && height_m >= 0.60 && height_m <= 2.40 &&
      area_m2 <= 3.20)
    {
      return true;
    }
  }

  return false;
}

bool possiblePcdArtifactsDetected(
  const std::unordered_set<GridIndex, GridIndexHash> & occupied_cells, double resolution_m,
  std::size_t rejected_collision_cells, std::size_t rejected_stair_noise_cells,
  std::size_t rejected_short_low_cells, std::size_t rejected_width_prefilter_cells)
{
  const std::size_t rejected_artifact_like =
    rejected_collision_cells + rejected_stair_noise_cells + rejected_short_low_cells +
    rejected_width_prefilter_cells;
  const std::size_t threshold = std::max<std::size_t>(200U, occupied_cells.size() / 100U);
  return rejected_artifact_like >= threshold ||
         possibleVerticalPcdArtifactsDetected(occupied_cells, resolution_m);
}

bool possiblePcdArtifactsDetected(const NavigationMap & map)
{
  return possiblePcdArtifactsDetected(
    map.occupiedCells(), map.resolution(), map.rejectedCollisionCells().size(),
    map.rejectedStairNoiseCells().size(), map.rejectedShortLowCells().size(),
    map.rejectedWidthPrefilterCells().size());
}

}  // namespace tgw_planner::core
