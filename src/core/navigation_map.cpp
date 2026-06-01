#include "tgw_planner/core/navigation_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

#include "pcl/filters/filter.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

namespace tgw_planner::core
{
namespace
{
template<typename T>
T clampValue(T value, T low, T high)
{
  return std::max(low, std::min(value, high));
}

Point3 orderedMin(const Point3 & a, const Point3 & b)
{
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

Point3 orderedMax(const Point3 & a, const Point3 & b)
{
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

}  // namespace

bool NavigationMap::loadFromPcd(
  const std::string & pcd_file, double requested_resolution_m, double robot_radius_m,
  double robot_height_m, const std::string & map_frame, const std::string & map_id,
  BuildStats & stats)
{
  const auto t0 = std::chrono::steady_clock::now();
  stats = BuildStats{};
  stats.source_pcd = pcd_file;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, cloud) != 0) {
    stats.message = "failed to load PCD file: " + pcd_file;
    return false;
  }

  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, finite_indices);
  stats.source_points = cloud.size();
  if (cloud.empty()) {
    stats.message = "PCD file has no finite points";
    return false;
  }

  resolution_m_ = requested_resolution_m > 0.0 ? requested_resolution_m : 0.20;
  resolution_m_ = clampValue(resolution_m_, 0.05, 1.0);
  robot_radius_m_ = std::max(0.01, robot_radius_m);
  robot_height_m_ = std::max(0.20, robot_height_m);
  risk_inflation_radius_m_ = std::max(robot_radius_m_, 2.0 * resolution_m_);
  map_frame_ = map_frame.empty() ? "map" : map_frame;
  map_id_ = map_id.empty() ? "tgw_nav_map" : map_id;

  occupied_cells_.clear();
  traversable_cells_.clear();
  forbidden_cells_.clear();
  surface_candidate_cells_.clear();
  accepted_floor_cells_.clear();
  accepted_stair_cells_.clear();
  rejected_ceiling_cells_.clear();
  rejected_clearance_cells_.clear();
  rejected_collision_cells_.clear();
  stair_slopes_.clear();
  stair_segment_by_cell_.clear();
  stair_segments_.clear();
  blocked_cells_.clear();
  risk_cost_.clear();
  columns_.clear();
  has_bounds_ = false;
  ready_ = false;
  octree_ = std::make_shared<octomap::OcTree>(resolution_m_);

  occupied_cells_.reserve(cloud.size() / 2U);
  for (const auto & point : cloud.points) {
    const GridIndex idx = worldToGrid({point.x, point.y, point.z});
    if (occupied_cells_.insert(idx).second) {
      updateGridBounds(idx);
    }
  }
  buildColumns();

  for (const auto & idx : occupied_cells_) {
    const Point3 center = gridToWorld(idx);
    octree_->updateNode(
      octomap::point3d(
        static_cast<float>(center.x), static_cast<float>(center.y), static_cast<float>(center.z)),
      true);
  }
  octree_->updateInnerOccupancy();

  refreshMetricBounds();
  rebuildTraversableLayer();
  rebuildRiskLayer();
  ready_ = true;

  const auto t1 = std::chrono::steady_clock::now();
  stats.success = true;
  stats.message = "ok";
  stats.build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  stats.bounds_min = bounds_min_;
  stats.bounds_max = bounds_max_;
  stats.counts = counts();
  return true;
}

bool NavigationMap::saveToMapPackage(
  const std::string & map_dir, const std::string & source_pcd, std::string & message) const
{
  if (!ready_ || !octree_) {
    message = "navigation map is not ready";
    return false;
  }
  if (map_dir.empty()) {
    message = "map_dir is empty";
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(map_dir, ec);
  if (ec) {
    message = "failed to create map package directory: " + ec.message();
    return false;
  }

  const auto map_path = std::filesystem::path(map_dir) / "map.bt";
  if (!octree_->writeBinary(map_path.string())) {
    message = "failed to write " + map_path.string();
    return false;
  }

  const std::time_t now = std::time(nullptr);
  std::tm utc_time{};
  gmtime_r(&now, &utc_time);
  const MapCounts map_counts = counts();

  const auto metadata_path = std::filesystem::path(map_dir) / "metadata.yaml";
  std::ofstream metadata(metadata_path);
  if (!metadata) {
    message = "failed to write " + metadata_path.string();
    return false;
  }
  metadata << "map_id: " << map_id_ << "\n";
  metadata << "frame_id: " << map_frame_ << "\n";
  metadata << "source_pcd: " << source_pcd << "\n";
  metadata << "resolution_m: " << resolution_m_ << "\n";
  metadata << "robot_radius_m: " << robot_radius_m_ << "\n";
  metadata << "created_at: \"" << std::put_time(&utc_time, "%Y-%m-%dT%H:%M:%SZ") << "\"\n";
  metadata << "bounds:\n";
  metadata << "  min: [" << bounds_min_.x << ", " << bounds_min_.y << ", " << bounds_min_.z << "]\n";
  metadata << "  max: [" << bounds_max_.x << ", " << bounds_max_.y << ", " << bounds_max_.z << "]\n";
  metadata << "counts:\n";
  metadata << "  occupied_cells: " << map_counts.occupied_cells << "\n";
  metadata << "  traversable_cells: " << map_counts.traversable_cells << "\n";
  metadata << "  blocked_cells: " << map_counts.blocked_cells << "\n";
  metadata << "  risk_cells: " << map_counts.risk_cells << "\n";

  const auto blocked_path = std::filesystem::path(map_dir) / "blocked_regions.yaml";
  std::ofstream blocked(blocked_path);
  if (!blocked) {
    message = "failed to write " + blocked_path.string();
    return false;
  }
  blocked << "blocked_regions: []\n";
  blocked << "blocked_cells:\n";
  for (const auto & cell : blocked_cells_) {
    const Point3 point = gridToWorld(cell);
    blocked << "  - [" << point.x << ", " << point.y << ", " << point.z << "]\n";
  }

  const auto readme_path = std::filesystem::path(map_dir) / "README.generated.txt";
  std::ofstream readme(readme_path);
  if (readme) {
    readme << "Generated by tgw_planner from " << source_pcd << "\n";
  }

  message = "saved map package to " + std::filesystem::absolute(map_dir).string();
  return true;
}

void NavigationMap::buildColumns()
{
  columns_.clear();
  columns_.reserve(occupied_cells_.size() / 4U);

  std::unordered_map<XYIndex, std::vector<int>, XYIndexHash> z_by_column;
  z_by_column.reserve(occupied_cells_.size() / 4U);
  for (const auto & cell : occupied_cells_) {
    z_by_column[{cell.x, cell.y}].push_back(cell.z);
  }

  for (auto & entry : z_by_column) {
    auto & z_values = entry.second;
    std::sort(z_values.begin(), z_values.end());
    z_values.erase(std::unique(z_values.begin(), z_values.end()), z_values.end());
    if (z_values.empty()) {
      continue;
    }

    ColumnInfo column;
    column.xy = entry.first;
    ZRun current{z_values.front(), z_values.front()};
    for (std::size_t i = 1; i < z_values.size(); ++i) {
      const int z = z_values[i];
      if (z == current.z_max + 1) {
        current.z_max = z;
        continue;
      }
      column.occupied_runs.push_back(current);
      current = {z, z};
    }
    column.occupied_runs.push_back(current);
    columns_[entry.first] = std::move(column);
  }
}

GridIndex NavigationMap::worldToGrid(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m_)),
    static_cast<int>(std::floor(point.y / resolution_m_)),
    static_cast<int>(std::floor(point.z / resolution_m_))};
}

Point3 NavigationMap::gridToWorld(const GridIndex & idx) const
{
  return {
    (static_cast<double>(idx.x) + 0.5) * resolution_m_,
    (static_cast<double>(idx.y) + 0.5) * resolution_m_,
    (static_cast<double>(idx.z) + 0.5) * resolution_m_};
}

bool NavigationMap::isInsideBounds(const GridIndex & idx) const
{
  if (!has_bounds_) {
    return false;
  }
  return idx.x >= min_idx_.x - 1 && idx.x <= max_idx_.x + 1 && idx.y >= min_idx_.y - 1 &&
         idx.y <= max_idx_.y + 1 && idx.z >= min_idx_.z - 1 && idx.z <= max_idx_.z + 4;
}

bool NavigationMap::isOccupied(const GridIndex & idx) const
{
  return occupied_cells_.find(idx) != occupied_cells_.end();
}

bool NavigationMap::isBlocked(const GridIndex & idx) const
{
  return blocked_cells_.find(idx) != blocked_cells_.end();
}

bool NavigationMap::hasGroundSupport(const GridIndex & idx) const
{
  return isOccupied({idx.x, idx.y, idx.z - 1});
}

bool NavigationMap::isInsideHorizontalRadius(int dx, int dy, int radius_cells) const
{
  if (std::abs(dx) > radius_cells || std::abs(dy) > radius_cells) {
    return false;
  }
  const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_m_;
  return distance <= robot_radius_m_ + 1.0e-9;
}

const ColumnInfo * NavigationMap::findColumn(int x, int y) const
{
  const auto it = columns_.find({x, y});
  return it == columns_.end() ? nullptr : &it->second;
}

bool NavigationMap::hasHeadClearanceInColumn(const GridIndex & idx, int height_cells) const
{
  const ColumnInfo * column = findColumn(idx.x, idx.y);
  if (!column) {
    return true;
  }

  const int z_min = idx.z;
  const int z_max = idx.z + height_cells;
  for (const auto & run : column->occupied_runs) {
    if (run.z_max < z_min) {
      continue;
    }
    if (run.z_min > z_max) {
      return true;
    }
    return false;
  }
  return true;
}

int NavigationMap::overheadDistanceCells(
  const GridIndex & idx, int height_cells, bool & overhead_known) const
{
  overhead_known = false;
  const ColumnInfo * column = findColumn(idx.x, idx.y);
  if (!column) {
    return 0;
  }

  const int head_top_z = idx.z + height_cells;
  for (const auto & run : column->occupied_runs) {
    if (run.z_max <= head_top_z) {
      continue;
    }
    overhead_known = true;
    return std::max(0, run.z_min - head_top_z - 1);
  }
  return 0;
}

bool NavigationMap::isCollisionFreeForRobot(const GridIndex & idx) const
{
  const int radius_cells = std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / resolution_m_)));
  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(robot_height_m_ / resolution_m_)));
  const int low_step_clearance_cells =
    std::max(0, static_cast<int>(std::floor(0.35 / resolution_m_)));
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (!isInsideHorizontalRadius(dx, dy, radius_cells)) {
        continue;
      }
      const GridIndex footprint{idx.x + dx, idx.y + dy, idx.z};
      if (isBlocked(footprint)) {
        return false;
      }
      for (int dz = 0; dz <= height_cells; ++dz) {
        const GridIndex check{idx.x + dx, idx.y + dy, idx.z + dz};
        if (isBlocked(check)) {
          return false;
        }
        if (dz <= low_step_clearance_cells) {
          continue;
        }
        if (isOccupied(check)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool NavigationMap::isTraversable(const GridIndex & idx) const
{
  return traversable_cells_.find(idx) != traversable_cells_.end() && !isBlocked(idx);
}

bool NavigationMap::isStairTraversable(const GridIndex & idx) const
{
  return accepted_stair_cells_.find(idx) != accepted_stair_cells_.end() && !isBlocked(idx);
}

bool NavigationMap::hasContinuousSupport(const GridIndex & idx) const
{
  int support_count = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (isOccupied({idx.x + dx, idx.y + dy, idx.z - 1})) {
        ++support_count;
      }
    }
  }
  return support_count >= 3;
}

bool NavigationMap::stairSlope(const GridIndex & idx, double & slope_x, double & slope_y) const
{
  slope_x = 0.0;
  slope_y = 0.0;
  const auto it = stair_slopes_.find(idx);
  if (it == stair_slopes_.end() || !it->second.valid) {
    return false;
  }
  slope_x = it->second.x;
  slope_y = it->second.y;
  return true;
}

int NavigationMap::stairSegmentId(const GridIndex & idx) const
{
  const auto it = stair_segment_by_cell_.find(idx);
  return it == stair_segment_by_cell_.end() ? -1 : it->second;
}

bool NavigationMap::stairCellsSlopeCompatible(const GridIndex & from, const GridIndex & to) const
{
  if (!isStairTraversable(from) || !isStairTraversable(to)) {
    return true;
  }
  if (from.x == to.x && from.y == to.y) {
    return false;
  }

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  const bool from_has_slope = stairSlope(from, from_x, from_y);
  const bool to_has_slope = stairSlope(to, to_x, to_y);

  if (from_has_slope && to_has_slope) {
    const double dot = from_x * to_x + from_y * to_y;
    return dot >= 0.20;
  }
  if (from.z != to.z) {
    return from_has_slope || to_has_slope;
  }
  return true;
}

bool NavigationMap::stairSideDirection(const GridIndex & idx, int & side_dx, int & side_dy) const
{
  side_dx = 0;
  side_dy = 0;

  double slope_x = 0.0;
  double slope_y = 0.0;
  if (stairSlope(idx, slope_x, slope_y)) {
    const double perp_x = -slope_y;
    const double perp_y = slope_x;
    constexpr double component_threshold = 0.35;
    if (std::abs(perp_x) >= component_threshold) {
      side_dx = perp_x > 0.0 ? 1 : -1;
    }
    if (std::abs(perp_y) >= component_threshold) {
      side_dy = perp_y > 0.0 ? 1 : -1;
    }
    if (side_dx == 0 && side_dy == 0) {
      if (std::abs(perp_x) > std::abs(perp_y)) {
        side_dx = perp_x > 0.0 ? 1 : -1;
      } else {
        side_dy = perp_y > 0.0 ? 1 : -1;
      }
    }
    return side_dx != 0 || side_dy != 0;
  }

  int axis_x = 0;
  int axis_y = 0;
  if (!stairAxis(idx, axis_x, axis_y)) {
    return false;
  }
  side_dx = axis_x != 0 && axis_y != 0 ? axis_y : (axis_x != 0 ? 0 : 1);
  side_dy = axis_x != 0 && axis_y != 0 ? -axis_x : (axis_x != 0 ? 1 : 0);
  return side_dx != 0 || side_dy != 0;
}

bool NavigationMap::hasNearbyAcceptedFloor(const GridIndex & idx) const
{
  const int max_step_cells = maxStepCells();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
        const GridIndex neighbor{idx.x + dx, idx.y + dy, idx.z + dz};
        if (accepted_floor_cells_.find(neighbor) != accepted_floor_cells_.end()) {
          return true;
        }
      }
    }
  }
  return false;
}

bool NavigationMap::isStairEndpointCell(const GridIndex & idx) const
{
  const int segment_id = stairSegmentId(idx);
  if (segment_id < 0 || static_cast<std::size_t>(segment_id) >= stair_segments_.size()) {
    return true;
  }
  const auto & segment = stair_segments_[segment_id];
  const int terminal_window = std::max(1, maxStepCells());
  const bool near_low_end = idx.z <= segment.z_min + terminal_window;
  const bool near_high_end = idx.z >= segment.z_max - terminal_window;
  if (!(near_low_end || near_high_end)) {
    return false;
  }
  return hasNearbyAcceptedFloor(idx);
}

void NavigationMap::rebuildStairSegments()
{
  stair_slopes_.clear();
  stair_segment_by_cell_.clear();
  stair_segments_.clear();
  if (accepted_stair_cells_.empty()) {
    return;
  }

  const int max_step_cells = maxStepCells();
  stair_slopes_.reserve(accepted_stair_cells_.size());
  stair_segment_by_cell_.reserve(accepted_stair_cells_.size());

  for (const auto & cell : accepted_stair_cells_) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          if (dz == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (accepted_stair_cells_.find(neighbor) == accepted_stair_cells_.end()) {
            continue;
          }
          const double weight = static_cast<double>(std::abs(dz));
          const double direction = dz > 0 ? 1.0 : -1.0;
          sum_x += direction * static_cast<double>(dx) * weight;
          sum_y += direction * static_cast<double>(dy) * weight;
        }
      }
    }

    StairSlope slope;
    const double norm = std::hypot(sum_x, sum_y);
    if (norm > 1.0e-9) {
      slope.x = sum_x / norm;
      slope.y = sum_y / norm;
      slope.valid = true;
    }
    stair_slopes_[cell] = slope;
  }

  for (const auto & seed : accepted_stair_cells_) {
    if (stair_segment_by_cell_.find(seed) != stair_segment_by_cell_.end()) {
      continue;
    }

    StairSegmentInfo segment;
    segment.id = static_cast<int>(stair_segments_.size());
    segment.z_min = seed.z;
    segment.z_max = seed.z;

    std::deque<GridIndex> queue;
    stair_segment_by_cell_[seed] = segment.id;
    queue.push_back(seed);
    while (!queue.empty()) {
      const GridIndex current = queue.front();
      queue.pop_front();
      segment.cells.push_back(current);
      segment.z_min = std::min(segment.z_min, current.z);
      segment.z_max = std::max(segment.z_max, current.z);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (accepted_stair_cells_.find(neighbor) == accepted_stair_cells_.end() ||
              stair_segment_by_cell_.find(neighbor) != stair_segment_by_cell_.end())
            {
              continue;
            }
            if (!stairCellsSlopeCompatible(current, neighbor)) {
              continue;
            }
            stair_segment_by_cell_[neighbor] = segment.id;
            queue.push_back(neighbor);
          }
        }
      }
    }

    int turning_edges = 0;
    int comparable_edges = 0;
    for (const auto & cell : segment.cells) {
      double cell_x = 0.0;
      double cell_y = 0.0;
      if (!stairSlope(cell, cell_x, cell_y)) {
        continue;
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z};
          if (stairSegmentId(neighbor) != segment.id) {
            continue;
          }
          double neighbor_x = 0.0;
          double neighbor_y = 0.0;
          if (!stairSlope(neighbor, neighbor_x, neighbor_y)) {
            continue;
          }
          ++comparable_edges;
          if (cell_x * neighbor_x + cell_y * neighbor_y < 0.85) {
            ++turning_edges;
          }
        }
      }
    }
    segment.spiral_like = comparable_edges > 0 && turning_edges * 4 > comparable_edges;
    stair_segments_.push_back(std::move(segment));
  }
}

bool NavigationMap::stairAxis(const GridIndex & idx, int & axis_x, int & axis_y) const
{
  axis_x = 0;
  axis_y = 0;
  if (!isStairTraversable(idx)) {
    return false;
  }

  double slope_x = 0.0;
  double slope_y = 0.0;
  if (stairSlope(idx, slope_x, slope_y)) {
    constexpr double component_threshold = 0.35;
    if (std::abs(slope_x) >= component_threshold) {
      axis_x = slope_x > 0.0 ? 1 : -1;
    }
    if (std::abs(slope_y) >= component_threshold) {
      axis_y = slope_y > 0.0 ? 1 : -1;
    }
    if (axis_x == 0 && axis_y == 0) {
      if (std::abs(slope_x) > std::abs(slope_y)) {
        axis_x = slope_x > 0.0 ? 1 : -1;
      } else {
        axis_y = slope_y > 0.0 ? 1 : -1;
      }
    }
    return true;
  }

  int x_score = 0;
  int y_score = 0;
  const int max_step_cells = maxStepCells();
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
        if (dz == 0) {
          continue;
        }
        const GridIndex neighbor{idx.x + dx, idx.y + dy, idx.z + dz};
        if (!isStairTraversable(neighbor)) {
          continue;
        }
        const int weight = std::abs(dz);
        x_score += std::abs(dx) * weight;
        y_score += std::abs(dy) * weight;
      }
    }
  }

  if (x_score == 0 && y_score == 0) {
    return false;
  }
  const int max_score = std::max(x_score, y_score);
  const int min_score = std::min(x_score, y_score);
  if (min_score > 0 && min_score * 2 >= max_score) {
    axis_x = 1;
    axis_y = 1;
  } else if (x_score > y_score) {
    axis_x = 1;
  } else {
    axis_y = 1;
  }
  return true;
}

bool NavigationMap::isStairTransitionAllowed(const GridIndex & from, const GridIndex & to) const
{
  const bool from_stair = isStairTraversable(from);
  const bool to_stair = isStairTraversable(to);
  if (!from_stair && !to_stair) {
    return true;
  }

  const int center_side_cells =
    std::max(1, static_cast<int>(std::ceil(0.5 * robot_radius_m_ / resolution_m_)));
  if (from_stair != to_stair) {
    const GridIndex stair_cell = from_stair ? from : to;
    return isStairEndpointCell(stair_cell) && isStairCenterCell(stair_cell, center_side_cells);
  }

  const int from_segment = stairSegmentId(from);
  const int to_segment = stairSegmentId(to);
  if (from_segment >= 0 && to_segment >= 0) {
    return from_segment == to_segment;
  }

  int from_axis_x = 0;
  int from_axis_y = 0;
  int to_axis_x = 0;
  int to_axis_y = 0;
  if (!stairAxis(from, from_axis_x, from_axis_y) || !stairAxis(to, to_axis_x, to_axis_y)) {
    return true;
  }
  const bool compatible_axis = from_axis_x == to_axis_x && from_axis_y == to_axis_y;
  if (!compatible_axis && from.z != to.z && !isStairCenterCell(from, 1) && !isStairCenterCell(to, 1)) {
    return false;
  }

  return stairCellsSlopeCompatible(from, to);
}

bool NavigationMap::isStairCenterCell(const GridIndex & idx, int min_side_cells) const
{
  int side_dx = 0;
  int side_dy = 0;
  if (!stairSideDirection(idx, side_dx, side_dy)) {
    return true;
  }

  const int left = stairSideRunLength(idx, side_dx, side_dy);
  const int right = stairSideRunLength(idx, -side_dx, -side_dy);
  if (std::max(left, right) < min_side_cells) {
    return true;
  }
  return std::min(left, right) >= min_side_cells;
}

int NavigationMap::stairSideRunLength(const GridIndex & idx, int side_dx, int side_dy) const
{
  int length = 0;
  const int max_width_cells = std::max(1, static_cast<int>(std::ceil(2.0 / resolution_m_)));
  for (int step = 1; step <= max_width_cells; ++step) {
    const GridIndex probe{idx.x + side_dx * step, idx.y + side_dy * step, idx.z};
    if (!isStairTraversable(probe)) {
      break;
    }
    ++length;
  }
  return length;
}

double NavigationMap::getStairCenterCost(const GridIndex & idx) const
{
  int side_dx = 0;
  int side_dy = 0;
  if (!stairSideDirection(idx, side_dx, side_dy)) {
    return 0.0;
  }

  const int left = stairSideRunLength(idx, side_dx, side_dy);
  const int right = stairSideRunLength(idx, -side_dx, -side_dy);
  const int side_sum = left + right;
  if (side_sum == 0) {
    return 2.0;
  }

  const int min_side = std::min(left, right);
  const int desired_margin_cells =
    std::max(1, static_cast<int>(std::ceil(robot_radius_m_ / resolution_m_)));
  const int edge_shortfall = std::max(0, desired_margin_cells - min_side);
  const double imbalance =
    static_cast<double>(std::abs(left - right)) / static_cast<double>(side_sum + 1);
  return 0.35 * static_cast<double>(edge_shortfall) + 0.45 * imbalance;
}

int NavigationMap::maxStepCells() const
{
  const double max_step_height_m = std::max(0.25, 0.5 * robot_radius_m_);
  return std::max(1, static_cast<int>(std::ceil(max_step_height_m / resolution_m_)));
}

void NavigationMap::rebuildTraversableLayer()
{
  traversable_cells_.clear();
  forbidden_cells_.clear();
  surface_candidate_cells_.clear();
  accepted_floor_cells_.clear();
  accepted_stair_cells_.clear();
  rejected_ceiling_cells_.clear();
  rejected_clearance_cells_.clear();
  rejected_collision_cells_.clear();

  std::vector<SurfaceCandidate> candidates;
  candidates.reserve(columns_.size() * 2U);
  std::unordered_map<GridIndex, std::size_t, GridIndexHash> candidate_index;
  candidate_index.reserve(columns_.size() * 2U);

  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(robot_height_m_ / resolution_m_)));
  const int min_overhead_cells =
    std::max(1, static_cast<int>(std::ceil(0.30 / resolution_m_)));

  auto reject_candidate = [&](const GridIndex & idx, SurfaceRejectReason reason) {
    forbidden_cells_.insert(idx);
    if (reason == SurfaceRejectReason::Clearance) {
      rejected_clearance_cells_.insert(idx);
    } else if (reason == SurfaceRejectReason::Collision) {
      rejected_collision_cells_.insert(idx);
    } else if (reason == SurfaceRejectReason::CeilingLike) {
      rejected_ceiling_cells_.insert(idx);
    }
  };

  for (const auto & entry : columns_) {
    const ColumnInfo & column = entry.second;
    for (const auto & run : column.occupied_runs) {
      SurfaceCandidate candidate;
      candidate.stand = {column.xy.x, column.xy.y, run.z_max + 1};
      candidate.support_z_min = run.z_min;
      candidate.support_z_max = run.z_max;
      bool overhead_known = false;
      candidate.overhead_distance_cells =
        overheadDistanceCells(candidate.stand, height_cells, overhead_known);
      candidate.overhead_known = overhead_known;
      candidate.overhead_clear =
        !overhead_known || candidate.overhead_distance_cells >= min_overhead_cells;
      surface_candidate_cells_.insert(candidate.stand);

      if (!isInsideBounds(candidate.stand) || isOccupied(candidate.stand) ||
        isBlocked(candidate.stand))
      {
        candidate.reject_reason = SurfaceRejectReason::Collision;
        reject_candidate(candidate.stand, candidate.reject_reason);
        continue;
      }
      if (!hasHeadClearanceInColumn(candidate.stand, height_cells)) {
        candidate.reject_reason = SurfaceRejectReason::Clearance;
        reject_candidate(candidate.stand, candidate.reject_reason);
        continue;
      }
      if (!isCollisionFreeForRobot(candidate.stand)) {
        candidate.reject_reason = SurfaceRejectReason::Collision;
        reject_candidate(candidate.stand, candidate.reject_reason);
        continue;
      }

      const std::size_t index = candidates.size();
      candidate_index[candidate.stand] = index;
      candidates.push_back(candidate);
    }
  }

  if (candidates.empty()) {
    return;
  }

  const int max_step_cells = maxStepCells();
  const int flat_z_range_cells =
    std::max(1, static_cast<int>(std::ceil(0.25 / resolution_m_)));
  const std::size_t min_flat_component_size = 25U;
  const std::size_t min_stair_component_size = 12U;
  const double min_floor_overhead_ratio = 0.55;
  const double min_stair_overhead_ratio = 0.35;

  std::vector<std::vector<std::size_t>> components;
  std::vector<bool> visited(candidates.size(), false);
  for (std::size_t seed = 0; seed < candidates.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    std::vector<std::size_t> component;
    std::deque<std::size_t> queue;
    visited[seed] = true;
    queue.push_back(seed);
    while (!queue.empty()) {
      const std::size_t current_index = queue.front();
      queue.pop_front();
      component.push_back(current_index);
      const GridIndex current = candidates[current_index].stand;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            const auto neighbor_it = candidate_index.find(neighbor);
            if (neighbor_it == candidate_index.end() || visited[neighbor_it->second]) {
              continue;
            }
            visited[neighbor_it->second] = true;
            queue.push_back(neighbor_it->second);
          }
        }
      }
    }
    components.push_back(std::move(component));
  }

  std::vector<SurfaceKind> component_kind(components.size(), SurfaceKind::Unknown);
  for (std::size_t component_id = 0; component_id < components.size(); ++component_id) {
    const auto & component = components[component_id];
    int z_min = std::numeric_limits<int>::max();
    int z_max = std::numeric_limits<int>::min();
    std::size_t overhead_clear_count = 0U;
    std::unordered_map<int, std::size_t> z_histogram;
    z_histogram.reserve(component.size());

    for (const std::size_t index : component) {
      const auto & candidate = candidates[index];
      z_min = std::min(z_min, candidate.stand.z);
      z_max = std::max(z_max, candidate.stand.z);
      if (candidate.overhead_clear) {
        ++overhead_clear_count;
      }
      ++z_histogram[candidate.stand.z];
    }

    const int z_range = z_max - z_min;
    const double overhead_ratio =
      static_cast<double>(overhead_clear_count) / static_cast<double>(component.size());
    std::size_t largest_z_slice = 0U;
    for (const auto & z_entry : z_histogram) {
      largest_z_slice = std::max(largest_z_slice, z_entry.second);
    }
    const bool has_landing =
      largest_z_slice >= std::max<std::size_t>(8U, component.size() / 5U);

    SurfaceKind kind = SurfaceKind::Noise;
    if (component.size() >= min_flat_component_size && z_range <= flat_z_range_cells) {
      kind = overhead_ratio >= min_floor_overhead_ratio ?
        SurfaceKind::Floor : SurfaceKind::CeilingLike;
    } else if (
      component.size() >= min_stair_component_size && z_range > flat_z_range_cells &&
      overhead_ratio >= min_stair_overhead_ratio && has_landing)
    {
      kind = SurfaceKind::Stair;
    }
    component_kind[component_id] = kind;
  }

  for (std::size_t component_id = 0; component_id < components.size(); ++component_id) {
    if (component_kind[component_id] != SurfaceKind::Floor) {
      continue;
    }
    for (const std::size_t index : components[component_id]) {
      accepted_floor_cells_.insert(candidates[index].stand);
    }
  }

  for (std::size_t component_id = 0; component_id < components.size(); ++component_id) {
    if (component_kind[component_id] != SurfaceKind::Stair) {
      continue;
    }

    bool touches_floor = false;
    for (const std::size_t index : components[component_id]) {
      const GridIndex current = candidates[index].stand;
      for (int dx = -1; dx <= 1 && !touches_floor; ++dx) {
        for (int dy = -1; dy <= 1 && !touches_floor; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (accepted_floor_cells_.find(neighbor) != accepted_floor_cells_.end()) {
              touches_floor = true;
              break;
            }
          }
        }
      }
    }

    if (!touches_floor) {
      bool has_internal_landing = false;
      std::unordered_map<int, std::size_t> z_histogram;
      for (const std::size_t index : components[component_id]) {
        ++z_histogram[candidates[index].stand.z];
      }
      for (const auto & z_entry : z_histogram) {
        if (z_entry.second >= 8U) {
          has_internal_landing = true;
          break;
        }
      }
      if (!has_internal_landing) {
        component_kind[component_id] = SurfaceKind::Noise;
      }
    }
  }

  for (std::size_t component_id = 0; component_id < components.size(); ++component_id) {
    const SurfaceKind kind = component_kind[component_id];
    auto has_vertical_neighbor = [&](const GridIndex & current) {
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            if (dz == 0) {
              continue;
            }
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (candidate_index.find(neighbor) != candidate_index.end()) {
              return true;
            }
          }
        }
      }
      return false;
    };

    for (const std::size_t index : components[component_id]) {
      SurfaceCandidate & candidate = candidates[index];
      candidate.kind = kind;
      if (kind == SurfaceKind::Floor) {
        accepted_floor_cells_.insert(candidate.stand);
        traversable_cells_.insert(candidate.stand);
      } else if (kind == SurfaceKind::Stair) {
        if (has_vertical_neighbor(candidate.stand)) {
          candidate.kind = SurfaceKind::Stair;
          accepted_stair_cells_.insert(candidate.stand);
        } else {
          candidate.kind = SurfaceKind::Floor;
          accepted_floor_cells_.insert(candidate.stand);
        }
        traversable_cells_.insert(candidate.stand);
      } else if (kind == SurfaceKind::CeilingLike) {
        candidate.reject_reason = SurfaceRejectReason::CeilingLike;
        rejected_ceiling_cells_.insert(candidate.stand);
        forbidden_cells_.insert(candidate.stand);
      } else {
        candidate.reject_reason = SurfaceRejectReason::Noise;
        forbidden_cells_.insert(candidate.stand);
      }
    }
  }

  rebuildStairSegments();

  for (const auto & blocked : blocked_cells_) {
    forbidden_cells_.insert(blocked);
  }
}

void NavigationMap::rebuildRiskLayer()
{
  risk_cost_.clear();
  if (blocked_cells_.empty() || traversable_cells_.empty()) {
    return;
  }

  const int radius_cells =
    std::max(1, static_cast<int>(std::ceil(risk_inflation_radius_m_ / resolution_m_)));
  for (const auto & blocked : blocked_cells_) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
          const GridIndex idx{blocked.x + dx, blocked.y + dy, blocked.z + dz};
          if (!isTraversable(idx)) {
            continue;
          }
          const double distance =
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_m_;
          if (distance > risk_inflation_radius_m_) {
            continue;
          }
          const double cost = 1.0 - distance / risk_inflation_radius_m_;
          auto it = risk_cost_.find(idx);
          if (it == risk_cost_.end() || cost > it->second) {
            risk_cost_[idx] = cost;
          }
        }
      }
    }
  }
}

std::uint32_t NavigationMap::addBlockedRegion(const Point3 & min, const Point3 & max)
{
  const Point3 lo = orderedMin(min, max);
  const Point3 hi = orderedMax(min, max);
  const GridIndex min_idx = worldToGrid(lo);
  const GridIndex max_idx = worldToGrid(hi);

  std::uint32_t affected = 0;
  for (int x = min_idx.x; x <= max_idx.x; ++x) {
    for (int y = min_idx.y; y <= max_idx.y; ++y) {
      for (int z = min_idx.z; z <= max_idx.z; ++z) {
        const GridIndex idx{x, y, z};
        if (traversable_cells_.find(idx) == traversable_cells_.end()) {
          continue;
        }
        if (blocked_cells_.insert(idx).second) {
          ++affected;
        }
      }
    }
  }

  if (affected > 0) {
    rebuildTraversableLayer();
    rebuildRiskLayer();
  }
  return affected;
}

std::uint32_t NavigationMap::removeBlockedRegion(const Point3 & min, const Point3 & max)
{
  const Point3 lo = orderedMin(min, max);
  const Point3 hi = orderedMax(min, max);
  const GridIndex min_idx = worldToGrid(lo);
  const GridIndex max_idx = worldToGrid(hi);

  std::vector<GridIndex> to_remove;
  for (const auto & idx : blocked_cells_) {
    if (idx.x >= min_idx.x && idx.x <= max_idx.x && idx.y >= min_idx.y && idx.y <= max_idx.y &&
      idx.z >= min_idx.z && idx.z <= max_idx.z)
    {
      to_remove.push_back(idx);
    }
  }

  for (const auto & idx : to_remove) {
    blocked_cells_.erase(idx);
  }
  if (!to_remove.empty()) {
    rebuildTraversableLayer();
    rebuildRiskLayer();
  }
  return static_cast<std::uint32_t>(to_remove.size());
}

std::uint32_t NavigationMap::clearBlockedRegions()
{
  const auto affected = static_cast<std::uint32_t>(blocked_cells_.size());
  blocked_cells_.clear();
  if (affected > 0) {
    rebuildTraversableLayer();
    rebuildRiskLayer();
  }
  return affected;
}

double NavigationMap::getRiskCost(const GridIndex & idx) const
{
  const auto it = risk_cost_.find(idx);
  return it == risk_cost_.end() ? 0.0 : it->second;
}

MapCounts NavigationMap::counts() const
{
  return {
    static_cast<std::uint32_t>(occupied_cells_.size()),
    static_cast<std::uint32_t>(traversable_cells_.size()),
    static_cast<std::uint32_t>(blocked_cells_.size()),
    static_cast<std::uint32_t>(risk_cost_.size())};
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::occupiedCells() const
{
  return occupied_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::traversableCells() const
{
  return traversable_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::forbiddenCells() const
{
  return forbidden_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::surfaceCandidateCells() const
{
  return surface_candidate_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::acceptedFloorCells() const
{
  return accepted_floor_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::acceptedStairCells() const
{
  return accepted_stair_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedCeilingCells() const
{
  return rejected_ceiling_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedClearanceCells() const
{
  return rejected_clearance_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedCollisionCells() const
{
  return rejected_collision_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::blockedCells() const
{
  return blocked_cells_;
}

const std::unordered_map<GridIndex, double, GridIndexHash> & NavigationMap::riskCosts() const
{
  return risk_cost_;
}

double NavigationMap::resolution() const
{
  return resolution_m_;
}

double NavigationMap::robotRadius() const
{
  return robot_radius_m_;
}

double NavigationMap::robotHeight() const
{
  return robot_height_m_;
}

double NavigationMap::riskInflationRadius() const
{
  return risk_inflation_radius_m_;
}

const std::string & NavigationMap::mapFrame() const
{
  return map_frame_;
}

const std::string & NavigationMap::mapId() const
{
  return map_id_;
}

const Point3 & NavigationMap::boundsMin() const
{
  return bounds_min_;
}

const Point3 & NavigationMap::boundsMax() const
{
  return bounds_max_;
}

bool NavigationMap::ready() const
{
  return ready_;
}

void NavigationMap::updateGridBounds(const GridIndex & idx)
{
  if (!has_bounds_) {
    min_idx_ = idx;
    max_idx_ = idx;
    has_bounds_ = true;
    return;
  }
  min_idx_.x = std::min(min_idx_.x, idx.x);
  min_idx_.y = std::min(min_idx_.y, idx.y);
  min_idx_.z = std::min(min_idx_.z, idx.z);
  max_idx_.x = std::max(max_idx_.x, idx.x);
  max_idx_.y = std::max(max_idx_.y, idx.y);
  max_idx_.z = std::max(max_idx_.z, idx.z);
}

void NavigationMap::refreshMetricBounds()
{
  if (!has_bounds_) {
    bounds_min_ = {};
    bounds_max_ = {};
    return;
  }
  bounds_min_ = gridToWorld(min_idx_);
  bounds_max_ = gridToWorld(max_idx_);
}

}  // namespace tgw_planner::core
