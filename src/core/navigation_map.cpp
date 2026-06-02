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
#include <queue>

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

double pointDistance3d(const Point3 & a, const Point3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

double Vec2::norm() const
{
  return std::hypot(x, y);
}

Vec2 Vec2::normalized() const
{
  const double n = norm();
  if (n <= 1.0e-9) {
    return {};
  }
  return {x / n, y / n};
}

double Vec2::dot(const Vec2 & other) const
{
  return x * other.x + y * other.y;
}

Vec2 Vec2::perpendicular() const
{
  return {-y, x};
}

bool NavigationMap::loadFromPcd(
  const std::string & pcd_file, double requested_resolution_m, double robot_radius_m,
  double robot_height_m, const std::string & map_frame, const std::string & map_id,
  BuildStats & stats, double robot_length_m, double robot_width_m, double base_to_front_m)
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
  robot_length_m_ = std::max(0.10, robot_length_m);
  robot_width_m_ = std::max(0.10, robot_width_m);
  robot_height_m_ = std::max(0.20, robot_height_m);
  base_to_front_m_ = clampValue(base_to_front_m, 0.01, robot_length_m_ - 0.01);
  robot_radius_m_ = std::max(0.01, robot_radius_m);
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
  rejected_stair_noise_cells_.clear();
  loose_stair_fragment_cells_.clear();
  rejected_short_low_cells_.clear();
  rejected_width_prefilter_cells_.clear();
  rescued_stair_fragment_cells_.clear();
  missing_stair_recovery_cells_.clear();
  loose_stair_fragments_.clear();
  fragment_bridges_.clear();
  stair_flight_diagnostics_ = StairFlightDiagnostics{};
  rejected_stair_noise_cells_.clear();
  loose_stair_fragment_cells_.clear();
  rejected_short_low_cells_.clear();
  rejected_width_prefilter_cells_.clear();
  rescued_stair_fragment_cells_.clear();
  missing_stair_recovery_cells_.clear();
  floor_components_.clear();
  landing_components_.clear();
  stair_flights_.clear();
  loose_stair_fragments_.clear();
  fragment_bridges_.clear();
  stair_cell_info_.clear();
  stair_slopes_.clear();
  stair_segment_by_cell_.clear();
  stair_segments_.clear();
  stair_flight_diagnostics_ = StairFlightDiagnostics{};
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
  metadata << "robot_length_m: " << robot_length_m_ << "\n";
  metadata << "robot_width_m: " << robot_width_m_ << "\n";
  metadata << "robot_height_m: " << robot_height_m_ << "\n";
  metadata << "base_to_front_m: " << base_to_front_m_ << "\n";
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

bool NavigationMap::isFootprintCollisionFreeAt(
  const GridIndex & idx, double heading_x, double heading_y) const
{
  const double heading_norm = std::hypot(heading_x, heading_y);
  if (heading_norm <= 1.0e-9) {
    return isCollisionFreeForRobot(idx);
  }

  const double forward_x = heading_x / heading_norm;
  const double forward_y = heading_y / heading_norm;
  const double side_x = -forward_y;
  const double side_y = forward_x;
  const double front_m = base_to_front_m_;
  const double rear_m = std::max(0.01, robot_length_m_ - base_to_front_m_);
  const double half_width_m = 0.5 * robot_width_m_;
  const double sample_step_m = std::max(0.05, 0.5 * resolution_m_);
  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(robot_height_m_ / resolution_m_)));
  const int low_step_clearance_cells =
    std::max(0, maxStepCells());
  const Point3 origin = gridToWorld(idx);

  std::unordered_set<GridIndex, GridIndexHash> footprint_columns;
  footprint_columns.reserve(64U);
  for (double local_x = -rear_m; local_x <= front_m + 1.0e-9; local_x += sample_step_m) {
    for (double local_y = -half_width_m; local_y <= half_width_m + 1.0e-9; local_y += sample_step_m) {
      const Point3 sample{
        origin.x + local_x * forward_x + local_y * side_x,
        origin.y + local_x * forward_y + local_y * side_y,
        origin.z};
      footprint_columns.insert(worldToGrid(sample));
    }
  }

  for (const auto & column : footprint_columns) {
    const GridIndex footprint{column.x, column.y, idx.z};
    if (isBlocked(footprint)) {
      return false;
    }
    for (int dz = 0; dz <= height_cells; ++dz) {
      const GridIndex check{column.x, column.y, idx.z + dz};
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
  return true;
}

bool NavigationMap::isFootprintTransitionSafe(const GridIndex & from, const GridIndex & to) const
{
  const double heading_x = static_cast<double>(to.x - from.x);
  const double heading_y = static_cast<double>(to.y - from.y);
  if (std::hypot(heading_x, heading_y) <= 1.0e-9) {
    return false;
  }
  return isFootprintCollisionFreeAt(from, heading_x, heading_y) &&
         isFootprintCollisionFreeAt(to, heading_x, heading_y);
}

bool NavigationMap::hasTraversableSupportNearColumn(int x, int y, int z, int max_dz) const
{
  for (int dz = 0; dz <= max_dz; ++dz) {
    if (isTraversable({x, y, z + dz}) || isTraversable({x, y, z - dz})) {
      return true;
    }
    if (isOccupied({x, y, z + dz - 1}) || isOccupied({x, y, z - dz - 1})) {
      return true;
    }
  }
  return false;
}

bool NavigationMap::isFootprintSupportedAt(
  const GridIndex & idx, double heading_x, double heading_y) const
{
  return isFootprintSupportedAtPoint(gridToWorld(idx), idx.z, heading_x, heading_y);
}

bool NavigationMap::isFootprintSupportedAtPoint(
  const Point3 & origin, int stand_z, double heading_x, double heading_y) const
{
  const double heading_norm = std::hypot(heading_x, heading_y);
  if (heading_norm <= 1.0e-9) {
    return hasTraversableSupportNearColumn(
      worldToGrid(origin).x, worldToGrid(origin).y, stand_z, maxStepCells());
  }

  const double forward_x = heading_x / heading_norm;
  const double forward_y = heading_y / heading_norm;
  const double side_x = -forward_y;
  const double side_y = forward_x;
  const double front_m = base_to_front_m_;
  const double rear_m = std::max(0.01, robot_length_m_ - base_to_front_m_);
  const double half_width_m = 0.5 * robot_width_m_;
  const double length_m = front_m + rear_m;
  const double sample_step_m = std::max(0.05, 0.5 * resolution_m_);

  std::unordered_map<GridIndex, int, GridIndexHash> column_band_mask;
  column_band_mask.reserve(64U);
  for (double local_x = -rear_m; local_x <= front_m + 1.0e-9; local_x += sample_step_m) {
    const double along_ratio = clampValue((local_x + rear_m) / length_m, 0.0, 1.0);
    const int band = along_ratio < (1.0 / 3.0) ? 0 : (along_ratio < (2.0 / 3.0) ? 1 : 2);
    for (double local_y = -half_width_m; local_y <= half_width_m + 1.0e-9; local_y += sample_step_m) {
      const Point3 sample{
        origin.x + local_x * forward_x + local_y * side_x,
        origin.y + local_x * forward_y + local_y * side_y,
        origin.z};
      column_band_mask[worldToGrid(sample)] |= (1 << band);
    }
  }

  if (column_band_mask.empty()) {
    return false;
  }

  const int support_z_tolerance = maxStepCells();
  int supported_columns = 0;
  int band_total[3] = {0, 0, 0};
  int band_supported[3] = {0, 0, 0};
  for (const auto & entry : column_band_mask) {
    const GridIndex & column = entry.first;
    const bool supported =
      hasTraversableSupportNearColumn(column.x, column.y, stand_z, support_z_tolerance);
    if (supported) {
      ++supported_columns;
    }
    for (int band = 0; band < 3; ++band) {
      if ((entry.second & (1 << band)) == 0) {
        continue;
      }
      ++band_total[band];
      if (supported) {
        ++band_supported[band];
      }
    }
  }

  const double support_ratio =
    static_cast<double>(supported_columns) / static_cast<double>(column_band_mask.size());
  if (support_ratio < 0.35) {
    return false;
  }

  for (int band = 0; band < 3; ++band) {
    if (band_total[band] == 0) {
      return false;
    }
    const double band_ratio =
      static_cast<double>(band_supported[band]) / static_cast<double>(band_total[band]);
    if (band_ratio < 0.10) {
      return false;
    }
  }
  return true;
}

bool NavigationMap::isFootprintTransitionSupported(
  const GridIndex & from, const GridIndex & to) const
{
  const double heading_x = static_cast<double>(to.x - from.x);
  const double heading_y = static_cast<double>(to.y - from.y);
  if (std::hypot(heading_x, heading_y) <= 1.0e-9) {
    return false;
  }
  const Point3 from_point = gridToWorld(from);
  const Point3 to_point = gridToWorld(to);
  const Point3 midpoint{
    0.5 * (from_point.x + to_point.x),
    0.5 * (from_point.y + to_point.y),
    0.5 * (from_point.z + to_point.z)};
  const int midpoint_z =
    static_cast<int>(std::floor(midpoint.z / resolution_m_));
  return isFootprintSupportedAtPoint(from_point, from.z, heading_x, heading_y) &&
         isFootprintSupportedAtPoint(midpoint, midpoint_z, heading_x, heading_y) &&
         isFootprintSupportedAtPoint(to_point, to.z, heading_x, heading_y);
}

bool NavigationMap::isTraversable(const GridIndex & idx) const
{
  return traversable_cells_.find(idx) != traversable_cells_.end() && !isBlocked(idx);
}

bool NavigationMap::isStairTraversable(const GridIndex & idx) const
{
  return accepted_stair_cells_.find(idx) != accepted_stair_cells_.end() && !isBlocked(idx);
}

int NavigationMap::stairFlightId(const GridIndex & cell) const
{
  const auto it = stair_cell_info_.find(cell);
  return it == stair_cell_info_.end() ? -1 : it->second.stair_flight_id;
}

bool NavigationMap::isStairCell(const GridIndex & cell) const
{
  return isStairTraversable(cell);
}

bool NavigationMap::isFloorOrLandingCell(const GridIndex & cell) const
{
  return accepted_floor_cells_.find(cell) != accepted_floor_cells_.end() && !isBlocked(cell);
}

bool NavigationMap::isInsideStairSafeCorridor(
  const GridIndex & cell, int stair_flight_id) const
{
  if (stair_flight_id < 0 ||
    static_cast<std::size_t>(stair_flight_id) >= stair_flights_.size())
  {
    return false;
  }
  const auto info_it = stair_cell_info_.find(cell);
  if (info_it == stair_cell_info_.end() || info_it->second.stair_flight_id != stair_flight_id) {
    return false;
  }
  const auto & flight = stair_flights_[stair_flight_id];
  return info_it->second.lateral_error_m <= flight.safe_half_width_m + 1.0e-9;
}

double NavigationMap::lateralDistanceToStairCenterline(
  const GridIndex & cell, int stair_flight_id) const
{
  if (stair_flight_id < 0 ||
    static_cast<std::size_t>(stair_flight_id) >= stair_flights_.size())
  {
    return std::numeric_limits<double>::infinity();
  }
  const auto info_it = stair_cell_info_.find(cell);
  if (info_it == stair_cell_info_.end() || info_it->second.stair_flight_id != stair_flight_id) {
    return std::numeric_limits<double>::infinity();
  }
  return info_it->second.lateral_error_m;
}

bool NavigationMap::isNearFlightEndpoint(
  const GridIndex & cell, const StairFlight & flight, const Point3 & endpoint) const
{
  (void)flight;
  const Point3 point = gridToWorld(cell);
  const double portal_radius_m =
    std::max({0.45, 2.0 * robot_width_m_, 3.0 * resolution_m_});
  return pointDistance3d(point, endpoint) <= portal_radius_m;
}

bool NavigationMap::isNearLowPortal(const GridIndex & cell, int stair_flight_id) const
{
  if (stair_flight_id < 0 ||
    static_cast<std::size_t>(stair_flight_id) >= stair_flights_.size())
  {
    return false;
  }
  return isNearFlightEndpoint(cell, stair_flights_[stair_flight_id],
    stair_flights_[stair_flight_id].low_endpoint);
}

bool NavigationMap::isNearHighPortal(const GridIndex & cell, int stair_flight_id) const
{
  if (stair_flight_id < 0 ||
    static_cast<std::size_t>(stair_flight_id) >= stair_flights_.size())
  {
    return false;
  }
  return isNearFlightEndpoint(cell, stair_flights_[stair_flight_id],
    stair_flights_[stair_flight_id].high_endpoint);
}

bool NavigationMap::isNearStairPortal(const GridIndex & cell, int stair_flight_id) const
{
  return isNearLowPortal(cell, stair_flight_id) || isNearHighPortal(cell, stair_flight_id);
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

bool NavigationMap::isStairFlightEdgeAllowed(const GridIndex & from, const GridIndex & to) const
{
  if (!isStairTraversable(from) || !isStairTraversable(to)) {
    return false;
  }
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  const int dz = to.z - from.z;
  if (dx == 0 && dy == 0) {
    return false;
  }

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  if (!stairSlope(from, from_x, from_y) || !stairSlope(to, to_x, to_y)) {
    return false;
  }

  const double slope_dot = from_x * to_x + from_y * to_y;
  if (slope_dot < 0.55) {
    return false;
  }

  const double move_x = static_cast<double>(dx);
  const double move_y = static_cast<double>(dy);
  const double move_norm = std::hypot(move_x, move_y);
  if (move_norm <= 1.0e-9) {
    return false;
  }

  const double along_from = (move_x / move_norm) * from_x + (move_y / move_norm) * from_y;
  const double along_to = (move_x / move_norm) * to_x + (move_y / move_norm) * to_y;
  constexpr double along_threshold = 0.35;
  if (dz == 0) {
    if (isTreadBridgeAllowed(from, to)) {
      return true;
    }
    return std::abs(along_from) <= along_threshold && std::abs(along_to) <= along_threshold;
  }

  const double vertical_direction = dz > 0 ? 1.0 : -1.0;
  return vertical_direction * along_from >= along_threshold &&
         vertical_direction * along_to >= along_threshold;
}

bool NavigationMap::isTreadBridgeAllowed(const GridIndex & from, const GridIndex & to) const
{
  if (!isStairTraversable(from) || !isStairTraversable(to) || from.z != to.z) {
    return false;
  }

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  if (!stairSlope(from, from_x, from_y) || !stairSlope(to, to_x, to_y)) {
    return false;
  }
  if (from_x * to_x + from_y * to_y < 0.55) {
    return false;
  }

  const double move_x = static_cast<double>(to.x - from.x);
  const double move_y = static_cast<double>(to.y - from.y);
  const double move_norm = std::hypot(move_x, move_y);
  if (move_norm <= 1.0e-9) {
    return false;
  }

  const double along_from = (move_x / move_norm) * from_x + (move_y / move_norm) * from_y;
  const double along_to = (move_x / move_norm) * to_x + (move_y / move_norm) * to_y;
  constexpr double along_threshold = 0.35;
  if (std::abs(along_from) <= along_threshold && std::abs(along_to) <= along_threshold) {
    return false;
  }

  if (hasLargeLandingPlateauAround(from) || hasLargeLandingPlateauAround(to)) {
    return false;
  }
  if (!hasUpDownStairEvidenceAround(from) || !hasUpDownStairEvidenceAround(to)) {
    return false;
  }
  return isFootprintTransitionSafe(from, to) && isFootprintTransitionSupported(from, to);
}

bool NavigationMap::hasUpDownStairEvidenceAround(const GridIndex & cell) const
{
  double slope_x = 0.0;
  double slope_y = 0.0;
  if (!stairSlope(cell, slope_x, slope_y)) {
    return false;
  }

  const int search_radius = std::max(2, maxStepCells());
  bool has_lower_behind = false;
  bool has_higher_ahead = false;
  for (int dx = -search_radius; dx <= search_radius; ++dx) {
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const double along =
        (static_cast<double>(dx) * slope_x + static_cast<double>(dy) * slope_y) * resolution_m_;
      if (std::abs(along) < 0.5 * resolution_m_) {
        continue;
      }
      for (int dz = -maxStepCells(); dz <= maxStepCells(); ++dz) {
        if (dz == 0) {
          continue;
        }
        const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
        if (!isStairTraversable(neighbor)) {
          continue;
        }
        if (dz < 0 && along < 0.0) {
          has_lower_behind = true;
        } else if (dz > 0 && along > 0.0) {
          has_higher_ahead = true;
        }
        if (has_lower_behind && has_higher_ahead) {
          return true;
        }
      }
    }
  }
  return false;
}

bool NavigationMap::hasLargeLandingPlateauAround(const GridIndex & cell) const
{
  double slope_x = 0.0;
  double slope_y = 0.0;
  if (!stairSlope(cell, slope_x, slope_y)) {
    return false;
  }

  auto same_height_stair_near = [&](int x, int y) {
    for (int dz = -1; dz <= 1; ++dz) {
      if (isStairTraversable({x, y, cell.z + dz})) {
        return true;
      }
    }
    return false;
  };

  int axis_x = 0;
  int axis_y = 0;
  if (std::abs(slope_x) >= 0.35) {
    axis_x = slope_x > 0.0 ? 1 : -1;
  }
  if (std::abs(slope_y) >= 0.35) {
    axis_y = slope_y > 0.0 ? 1 : -1;
  }
  if (axis_x == 0 && axis_y == 0) {
    if (std::abs(slope_x) > std::abs(slope_y)) {
      axis_x = slope_x > 0.0 ? 1 : -1;
    } else {
      axis_y = slope_y > 0.0 ? 1 : -1;
    }
  }

  const int max_run_cells =
    std::max(3, static_cast<int>(std::ceil(std::max(0.45, 4.0 * resolution_m_) / resolution_m_)));
  int run_cells = 1;
  for (int direction : {-1, 1}) {
    for (int step = 1; step <= max_run_cells; ++step) {
      const int x = cell.x + direction * axis_x * step;
      const int y = cell.y + direction * axis_y * step;
      if (!same_height_stair_near(x, y)) {
        break;
      }
      ++run_cells;
    }
  }
  return run_cells >= max_run_cells;
}

bool NavigationMap::isStairSegmentBridgeAllowed(const GridIndex & from, const GridIndex & to) const
{
  if (!isStairTraversable(from) || !isStairTraversable(to) || from.z == to.z) {
    return false;
  }

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  if (!stairSlope(from, from_x, from_y) || !stairSlope(to, to_x, to_y)) {
    return false;
  }
  if (from_x * to_x + from_y * to_y < 0.20) {
    return false;
  }

  const double move_x = static_cast<double>(to.x - from.x);
  const double move_y = static_cast<double>(to.y - from.y);
  const double move_norm = std::hypot(move_x, move_y);
  if (move_norm <= 1.0e-9) {
    return false;
  }

  const double direction = to.z > from.z ? 1.0 : -1.0;
  const double align =
    direction * (move_x / move_norm * from_x + move_y / move_norm * from_y);
  return align >= 0.45;
}

bool NavigationMap::isStairSameHeightTransferAllowed(const GridIndex & from, const GridIndex & to) const
{
  if (!isStairTraversable(from) || !isStairTraversable(to) || from.z != to.z) {
    return true;
  }

  const double move_x = static_cast<double>(to.x - from.x);
  const double move_y = static_cast<double>(to.y - from.y);
  const double move_norm = std::hypot(move_x, move_y);
  if (move_norm <= 1.0e-9) {
    return false;
  }

  double from_x = 0.0;
  double from_y = 0.0;
  double to_x = 0.0;
  double to_y = 0.0;
  const bool from_has_slope = stairSlope(from, from_x, from_y);
  const bool to_has_slope = stairSlope(to, to_x, to_y);
  if (!from_has_slope && !to_has_slope) {
    return true;
  }

  const double unit_x = move_x / move_norm;
  const double unit_y = move_y / move_norm;
  const double from_along =
    from_has_slope ? std::abs(unit_x * from_x + unit_y * from_y) : 1.0;
  const double to_along = to_has_slope ? std::abs(unit_x * to_x + unit_y * to_y) : 1.0;
  const bool moves_along_stair = std::max(from_along, to_along) >= 0.45;
  if (moves_along_stair) {
    return true;
  }

  if (from_has_slope && to_has_slope) {
    return false;
  }

  const GridIndex sloped_cell = from_has_slope ? from : to;
  const int center_side_cells =
    std::max(1, static_cast<int>(std::ceil(0.5 * robot_width_m_ / resolution_m_)));
  return isStairEndpointCell(sloped_cell) && isStairCenterCell(sloped_cell, center_side_cells) &&
         isFootprintTransitionSafe(from, to);
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

  std::unordered_set<GridIndex, GridIndexHash> visited_stair_cells;
  visited_stair_cells.reserve(accepted_stair_cells_.size());

  for (const auto & seed : accepted_stair_cells_) {
    if (visited_stair_cells.find(seed) != visited_stair_cells.end()) {
      continue;
    }

    StairSegmentInfo segment;
    segment.z_min = seed.z;
    segment.z_max = seed.z;

    std::deque<GridIndex> queue;
    visited_stair_cells.insert(seed);
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
              visited_stair_cells.find(neighbor) != visited_stair_cells.end())
            {
              continue;
            }
            if (!isStairFlightEdgeAllowed(current, neighbor)) {
              continue;
            }
            visited_stair_cells.insert(neighbor);
            queue.push_back(neighbor);
          }
        }
      }
    }

    std::unordered_set<GridIndex, GridIndexHash> segment_cell_set;
    segment_cell_set.reserve(segment.cells.size());
    for (const auto & cell : segment.cells) {
      segment_cell_set.insert(cell);
    }

    int turning_edges = 0;
    int comparable_edges = 0;
    double slope_sum_x = 0.0;
    double slope_sum_y = 0.0;
    std::size_t slope_count = 0U;
    for (const auto & cell : segment.cells) {
      double cell_x = 0.0;
      double cell_y = 0.0;
      if (!stairSlope(cell, cell_x, cell_y)) {
        continue;
      }
      slope_sum_x += cell_x;
      slope_sum_y += cell_y;
      ++slope_count;
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z};
          if (segment_cell_set.find(neighbor) == segment_cell_set.end()) {
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
    const double average_slope_norm = slope_count == 0U ?
      0.0 : std::hypot(slope_sum_x, slope_sum_y) / static_cast<double>(slope_count);
    segment.spiral_like =
      average_slope_norm < 0.65 && comparable_edges > 0 && turning_edges * 4 > comparable_edges;

    ++stair_flight_diagnostics_.raw_segments;
    if (!isStairFlightWideEnough(segment)) {
      ++stair_flight_diagnostics_.segment_width_rejected;
      for (const auto & cell : segment.cells) {
        rejected_width_prefilter_cells_.insert(cell);
      }
    }

    segment.id = static_cast<int>(stair_segments_.size());
    for (const auto & cell : segment.cells) {
      stair_segment_by_cell_[cell] = segment.id;
    }
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

  if (from_stair != to_stair) {
    const GridIndex stair_cell = from_stair ? from : to;
    const GridIndex floor_cell = from_stair ? to : from;
    const int flight_id = stairFlightId(stair_cell);
    if (flight_id < 0) {
      return false;
    }
    if (!isInsideStairSafeCorridor(stair_cell, flight_id)) {
      return false;
    }
    if (isNearStairPortal(stair_cell, flight_id) || isNearStairPortal(floor_cell, flight_id)) {
      return true;
    }
    const int center_side_cells =
      std::max(1, static_cast<int>(std::ceil(0.5 * robot_width_m_ / resolution_m_)));
    return isStairEndpointCell(stair_cell) && isStairCenterCell(stair_cell, center_side_cells);
  }

  const int from_flight = stairFlightId(from);
  const int to_flight = stairFlightId(to);
  if (from_flight < 0 || to_flight < 0) {
    return false;
  }
  if (from_flight != to_flight) {
    const auto & from_info = stair_flights_[from_flight];
    const auto & to_info = stair_flights_[to_flight];
    const bool shared_component =
      (from_info.low_component_id >= 0 &&
      (from_info.low_component_id == to_info.low_component_id ||
      from_info.low_component_id == to_info.high_component_id)) ||
      (from_info.high_component_id >= 0 &&
      (from_info.high_component_id == to_info.low_component_id ||
      from_info.high_component_id == to_info.high_component_id));
    return shared_component && isNearStairPortal(from, from_flight) &&
           isNearStairPortal(to, to_flight);
  }
  if (!isInsideStairSafeCorridor(from, from_flight) ||
    !isInsideStairSafeCorridor(to, to_flight))
  {
    return false;
  }

  const auto & flight = stair_flights_[from_flight];
  const double move_x = static_cast<double>(to.x - from.x) * resolution_m_;
  const double move_y = static_cast<double>(to.y - from.y) * resolution_m_;
  const double move_norm = std::hypot(move_x, move_y);
  if (move_norm <= 1.0e-9) {
    return false;
  }

  const auto from_info_it = stair_cell_info_.find(from);
  const auto to_info_it = stair_cell_info_.find(to);
  if (from_info_it == stair_cell_info_.end() || to_info_it == stair_cell_info_.end()) {
    return false;
  }

  const double along = (move_x / move_norm) * flight.uphill_axis.x +
    (move_y / move_norm) * flight.uphill_axis.y;
  const double lateral_delta =
    std::abs(to_info_it->second.lateral_t - from_info_it->second.lateral_t);
  const double lateral_limit = std::max(0.25, 0.65 * robot_width_m_);
  if (lateral_delta > lateral_limit) {
    return false;
  }

  const int dz = to.z - from.z;
  if (dz != 0) {
    const double vertical_direction = dz > 0 ? 1.0 : -1.0;
    return vertical_direction * along >= 0.25;
  }

  const double from_error = from_info_it->second.lateral_error_m;
  const double to_error = to_info_it->second.lateral_error_m;
  const bool centering_move = to_error <= from_error + 0.05 && std::abs(along) < 0.55;
  return std::abs(along) >= 0.25 || centering_move;
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

bool NavigationMap::isStairFlightWideEnough(const StairSegmentInfo & segment) const
{
  if (segment.cells.empty()) {
    return false;
  }

  const int required_width_cells = std::max(
    2, static_cast<int>(std::ceil((robot_width_m_ + std::max(0.10, resolution_m_)) /
      resolution_m_)));
  const int required_center_side_cells = 1;
  const int max_width_cells = std::max(1, static_cast<int>(std::ceil(2.0 / resolution_m_)));
  const int z_tolerance = std::max(1, maxStepCells() / 2);

  auto has_stair_near = [&](int x, int y, int z) {
    for (int dz = -z_tolerance; dz <= z_tolerance; ++dz) {
      if (accepted_stair_cells_.find({x, y, z + dz}) != accepted_stair_cells_.end()) {
        return true;
      }
    }
    return false;
  };

  std::size_t wide_cells = 0U;
  std::size_t center_cells = 0U;
  for (const auto & cell : segment.cells) {
    int side_dx = 0;
    int side_dy = 0;
    if (!stairSideDirection(cell, side_dx, side_dy)) {
      continue;
    }

    int left = 0;
    for (int step = 1; step <= max_width_cells; ++step) {
      if (!has_stair_near(cell.x + side_dx * step, cell.y + side_dy * step, cell.z)) {
        break;
      }
      ++left;
    }

    int right = 0;
    for (int step = 1; step <= max_width_cells; ++step) {
      if (!has_stair_near(cell.x - side_dx * step, cell.y - side_dy * step, cell.z)) {
        break;
      }
      ++right;
    }

    const int width_cells = left + right + 1;
    if (width_cells >= required_width_cells) {
      ++wide_cells;
    }
    if (std::min(left, right) >= required_center_side_cells) {
      ++center_cells;
    }
  }

  const std::size_t min_supported_cells =
    std::max<std::size_t>(3U, segment.cells.size() / 20U);
  return wide_cells >= min_supported_cells && center_cells >= min_supported_cells;
}

void NavigationMap::recoverMissingStairCells()
{
  missing_stair_recovery_cells_.clear();

  std::unordered_set<GridIndex, GridIndexHash> candidate_pool = rejected_stair_noise_cells_;
  std::vector<GridIndex> recovered;
  const int max_step_cells = maxStepCells();
  const int search_radius = std::max(2, max_step_cells);
  const int height_cells =
    std::max(1, static_cast<int>(std::ceil(robot_height_m_ / resolution_m_)));

  for (const auto & cell : candidate_pool) {
    if (accepted_stair_cells_.find(cell) != accepted_stair_cells_.end()) {
      continue;
    }
    if (accepted_floor_cells_.find(cell) != accepted_floor_cells_.end()) {
      continue;
    }
    if (!isInsideBounds(cell) || isOccupied(cell) || isBlocked(cell)) {
      continue;
    }
    if (!hasHeadClearanceInColumn(cell, height_cells)) {
      continue;
    }

    bool near_floor = false;
    bool near_stair = false;
    int z_min = cell.z;
    int z_max = cell.z;
    int transition_neighbors = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (accepted_floor_cells_.find(neighbor) != accepted_floor_cells_.end()) {
            near_floor = true;
          }
          const bool neighbor_is_stair_like =
            accepted_stair_cells_.find(neighbor) != accepted_stair_cells_.end() ||
            rejected_stair_noise_cells_.find(neighbor) != rejected_stair_noise_cells_.end();
          if (!neighbor_is_stair_like || dz == 0) {
            continue;
          }
          near_stair = true;
          ++transition_neighbors;
          z_min = std::min(z_min, neighbor.z);
          z_max = std::max(z_max, neighbor.z);
          const double direction = dz > 0 ? 1.0 : -1.0;
          const double weight = static_cast<double>(std::abs(dz));
          sum_x += direction * static_cast<double>(dx) * weight;
          sum_y += direction * static_cast<double>(dy) * weight;
        }
      }
    }

    if (!near_floor || !near_stair || transition_neighbors < 4) {
      continue;
    }
    const double norm = std::hypot(sum_x, sum_y);
    if (norm <= 1.0e-9 || z_max - z_min < 2) {
      continue;
    }
    recovered.push_back(cell);
  }

  for (const auto & cell : accepted_floor_cells_) {
    if (!isInsideBounds(cell) || isOccupied(cell) || isBlocked(cell)) {
      continue;
    }
    if (!hasHeadClearanceInColumn(cell, height_cells)) {
      continue;
    }

    int transition_neighbors = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          if (dz == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (accepted_stair_cells_.find(neighbor) == accepted_stair_cells_.end() &&
            rejected_stair_noise_cells_.find(neighbor) == rejected_stair_noise_cells_.end())
          {
            continue;
          }
          ++transition_neighbors;
          const double direction = dz > 0 ? 1.0 : -1.0;
          const double weight = static_cast<double>(std::abs(dz));
          sum_x += direction * static_cast<double>(dx) * weight;
          sum_y += direction * static_cast<double>(dy) * weight;
        }
      }
    }
    const double norm = std::hypot(sum_x, sum_y);
    if (transition_neighbors < 4 || norm <= 1.0e-9) {
      continue;
    }

    const double axis_x = sum_x / norm;
    const double axis_y = sum_y / norm;
    bool has_lower_behind = false;
    bool has_higher_ahead = false;
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const double along =
          (static_cast<double>(dx) * axis_x + static_cast<double>(dy) * axis_y) * resolution_m_;
        if (std::abs(along) < 0.5 * resolution_m_) {
          continue;
        }
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          if (dz == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (accepted_stair_cells_.find(neighbor) == accepted_stair_cells_.end() &&
            rejected_stair_noise_cells_.find(neighbor) == rejected_stair_noise_cells_.end())
          {
            continue;
          }
          if (dz < 0 && along < 0.0) {
            has_lower_behind = true;
          } else if (dz > 0 && along > 0.0) {
            has_higher_ahead = true;
          }
        }
      }
    }
    if (!has_lower_behind || !has_higher_ahead) {
      continue;
    }

    int axis_dx = 0;
    int axis_dy = 0;
    if (std::abs(axis_x) >= 0.35) {
      axis_dx = axis_x > 0.0 ? 1 : -1;
    }
    if (std::abs(axis_y) >= 0.35) {
      axis_dy = axis_y > 0.0 ? 1 : -1;
    }
    if (axis_dx == 0 && axis_dy == 0) {
      if (std::abs(axis_x) > std::abs(axis_y)) {
        axis_dx = axis_x > 0.0 ? 1 : -1;
      } else {
        axis_dy = axis_y > 0.0 ? 1 : -1;
      }
    }

    const int max_tread_run_cells =
      std::max(3, static_cast<int>(std::ceil(std::max(0.55, robot_length_m_) / resolution_m_)));
    int same_height_run = 1;
    for (int direction : {-1, 1}) {
      for (int step = 1; step <= max_tread_run_cells; ++step) {
        const GridIndex probe{
          cell.x + direction * axis_dx * step, cell.y + direction * axis_dy * step, cell.z};
        if (accepted_floor_cells_.find(probe) == accepted_floor_cells_.end()) {
          break;
        }
        ++same_height_run;
      }
    }
    if (same_height_run >= max_tread_run_cells) {
      continue;
    }

    const int side_dx = -axis_dy;
    const int side_dy = axis_dx;
    const int required_side_cells =
      std::max(1, static_cast<int>(std::ceil(0.5 * robot_width_m_ / resolution_m_)));
    auto has_surface_at = [&](int x, int y) {
      for (int dz = -1; dz <= 1; ++dz) {
        const GridIndex probe{x, y, cell.z + dz};
        if (accepted_floor_cells_.find(probe) != accepted_floor_cells_.end() ||
          accepted_stair_cells_.find(probe) != accepted_stair_cells_.end() ||
          rejected_stair_noise_cells_.find(probe) != rejected_stair_noise_cells_.end())
        {
          return true;
        }
      }
      return false;
    };
    int left = 0;
    int right = 0;
    for (int step = 1; step <= required_side_cells; ++step) {
      if (has_surface_at(cell.x + side_dx * step, cell.y + side_dy * step)) {
        ++left;
      }
      if (has_surface_at(cell.x - side_dx * step, cell.y - side_dy * step)) {
        ++right;
      }
    }
    if (std::min(left, right) < required_side_cells) {
      continue;
    }

    recovered.push_back(cell);
  }

  for (const auto & cell : recovered) {
    accepted_stair_cells_.insert(cell);
    traversable_cells_.insert(cell);
    forbidden_cells_.erase(cell);
    rejected_stair_noise_cells_.erase(cell);
    accepted_floor_cells_.erase(cell);
    missing_stair_recovery_cells_.insert(cell);
  }
  stair_flight_diagnostics_.recovered_stair_cell_count = missing_stair_recovery_cells_.size();
}

void NavigationMap::rebuildFloorComponents()
{
  floor_components_.clear();
  landing_components_.clear();
  if (accepted_floor_cells_.empty()) {
    return;
  }

  const int flat_z_tolerance =
    std::max(1, static_cast<int>(std::ceil(0.25 / resolution_m_)));
  std::unordered_set<GridIndex, GridIndexHash> visited;
  visited.reserve(accepted_floor_cells_.size());

  for (const auto & seed : accepted_floor_cells_) {
    if (visited.find(seed) != visited.end()) {
      continue;
    }

    FloorComponent component;
    component.id = static_cast<int>(floor_components_.size());
    component.min_idx = seed;
    component.max_idx = seed;

    std::deque<GridIndex> queue;
    visited.insert(seed);
    queue.push_back(seed);
    Point3 sum;
    double z_sum = 0.0;

    while (!queue.empty()) {
      const GridIndex current = queue.front();
      queue.pop_front();
      component.cells.push_back(current);
      component.min_idx.x = std::min(component.min_idx.x, current.x);
      component.min_idx.y = std::min(component.min_idx.y, current.y);
      component.min_idx.z = std::min(component.min_idx.z, current.z);
      component.max_idx.x = std::max(component.max_idx.x, current.x);
      component.max_idx.y = std::max(component.max_idx.y, current.y);
      component.max_idx.z = std::max(component.max_idx.z, current.z);

      const Point3 point = gridToWorld(current);
      sum.x += point.x;
      sum.y += point.y;
      sum.z += point.z;
      z_sum += point.z;

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          for (int dz = -flat_z_tolerance; dz <= flat_z_tolerance; ++dz) {
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (accepted_floor_cells_.find(neighbor) == accepted_floor_cells_.end() ||
              visited.find(neighbor) != visited.end())
            {
              continue;
            }
            visited.insert(neighbor);
            queue.push_back(neighbor);
          }
        }
      }
    }

    if (component.cells.empty()) {
      continue;
    }
    const double scale = 1.0 / static_cast<double>(component.cells.size());
    component.centroid = {sum.x * scale, sum.y * scale, sum.z * scale};
    component.mean_z = z_sum * scale;
    component.min_z = gridToWorld(component.min_idx).z;
    component.max_z = gridToWorld(component.max_idx).z;

    bool near_stair = false;
    const int stair_radius_cells = std::max(2, maxStepCells());
    for (const auto & cell : component.cells) {
      for (int dx = -stair_radius_cells; dx <= stair_radius_cells && !near_stair; ++dx) {
        for (int dy = -stair_radius_cells; dy <= stair_radius_cells && !near_stair; ++dy) {
          for (int dz = -maxStepCells(); dz <= maxStepCells(); ++dz) {
            if (accepted_stair_cells_.find({cell.x + dx, cell.y + dy, cell.z + dz}) !=
              accepted_stair_cells_.end())
            {
              near_stair = true;
              break;
            }
          }
        }
      }
      if (near_stair) {
        break;
      }
    }

    component.is_landing =
      near_stair && component.cells.size() < std::max<std::size_t>(
        80U, static_cast<std::size_t>(std::ceil(80.0 / (resolution_m_ * resolution_m_))));
    floor_components_.push_back(component);
    if (component.is_landing) {
      landing_components_.push_back(component);
    }
  }
}

int NavigationMap::nearestFloorComponent(const Point3 & point, double max_distance_m) const
{
  int best_id = -1;
  double best_distance = max_distance_m;
  for (const auto & component : floor_components_) {
    for (const auto & cell : component.cells) {
      const Point3 cell_point = gridToWorld(cell);
      const double dz = std::abs(cell_point.z - point.z);
      if (dz > std::max(0.45, 3.0 * resolution_m_)) {
        continue;
      }
      const double distance = pointDistance3d(point, cell_point);
      if (distance < best_distance) {
        best_distance = distance;
        best_id = component.id;
      }
    }
  }
  return best_id;
}

bool NavigationMap::hasFloorBetween(const Point3 & a, const Point3 & b) const
{
  constexpr int samples = 12;
  for (int i = 2; i < samples - 2; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(samples - 1);
    const Point3 point{
      a.x + (b.x - a.x) * ratio,
      a.y + (b.y - a.y) * ratio,
      a.z + (b.z - a.z) * ratio};
    const GridIndex center = worldToGrid(point);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (accepted_floor_cells_.find({center.x + dx, center.y + dy, center.z + dz}) !=
            accepted_floor_cells_.end())
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool NavigationMap::fitStairFragmentLoose(
  const StairSegmentInfo & segment, LooseStairFragment & fragment,
  StairFlightRejectReason * reject_reason) const
{
  auto set_reason = [&](StairFlightRejectReason reason) {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    fragment.reject_reason = reason;
  };
  set_reason(StairFlightRejectReason::None);

  if (segment.cells.size() < 5U) {
    set_reason(StairFlightRejectReason::TooFewCells);
    return false;
  }

  Vec2 uphill;
  for (const auto & cell : segment.cells) {
    double slope_x = 0.0;
    double slope_y = 0.0;
    if (stairSlope(cell, slope_x, slope_y)) {
      uphill.x += slope_x;
      uphill.y += slope_y;
    }
  }
  uphill = uphill.normalized();

  Point3 centroid;
  for (const auto & cell : segment.cells) {
    const Point3 point = gridToWorld(cell);
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }
  const double inv_count = 1.0 / static_cast<double>(segment.cells.size());
  centroid.x *= inv_count;
  centroid.y *= inv_count;
  centroid.z *= inv_count;

  if (uphill.norm() <= 1.0e-9) {
    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    double sxz = 0.0;
    double syz = 0.0;
    for (const auto & cell : segment.cells) {
      const Point3 point = gridToWorld(cell);
      const double x = point.x - centroid.x;
      const double y = point.y - centroid.y;
      const double z = point.z - centroid.z;
      cxx += x * x;
      cxy += x * y;
      cyy += y * y;
      sxz += x * z;
      syz += y * z;
    }
    const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    uphill = Vec2{std::cos(angle), std::sin(angle)}.normalized();
    if (uphill.x * sxz + uphill.y * syz < 0.0) {
      uphill.x = -uphill.x;
      uphill.y = -uphill.y;
    }
  }
  if (uphill.norm() <= 1.0e-9) {
    set_reason(StairFlightRejectReason::NoAxis);
    return false;
  }

  Vec2 side = uphill.perpendicular();
  struct Projection
  {
    GridIndex cell;
    Point3 point;
    double s{0.0};
    double t{0.0};
  };

  std::vector<Projection> projections;
  projections.reserve(segment.cells.size());
  auto rebuild_projections = [&]() {
    projections.clear();
    fragment.s_min = std::numeric_limits<double>::infinity();
    fragment.s_max = -std::numeric_limits<double>::infinity();
    fragment.t_min = std::numeric_limits<double>::infinity();
    fragment.t_max = -std::numeric_limits<double>::infinity();
    fragment.z_min = std::numeric_limits<double>::infinity();
    fragment.z_max = -std::numeric_limits<double>::infinity();
    for (const auto & cell : segment.cells) {
      const Point3 point = gridToWorld(cell);
      const double s = point.x * uphill.x + point.y * uphill.y;
      const double t = point.x * side.x + point.y * side.y;
      projections.push_back({cell, point, s, t});
      fragment.s_min = std::min(fragment.s_min, s);
      fragment.s_max = std::max(fragment.s_max, s);
      fragment.t_min = std::min(fragment.t_min, t);
      fragment.t_max = std::max(fragment.t_max, t);
      fragment.z_min = std::min(fragment.z_min, point.z);
      fragment.z_max = std::max(fragment.z_max, point.z);
    }
  };
  rebuild_projections();

  fragment.length_m = fragment.s_max - fragment.s_min;
  fragment.height_m = fragment.z_max - fragment.z_min;
  if (fragment.length_m < std::max(0.20, 2.0 * resolution_m_) &&
    fragment.height_m < std::max(0.15, 1.5 * resolution_m_))
  {
    set_reason(StairFlightRejectReason::TooFewCells);
    return false;
  }

  double low_z_sum = 0.0;
  double high_z_sum = 0.0;
  std::size_t low_z_count = 0U;
  std::size_t high_z_count = 0U;
  const double terminal_s_window =
    std::max(2.0 * resolution_m_, 0.20 * std::max(fragment.length_m, resolution_m_));
  for (const auto & p : projections) {
    if (p.s <= fragment.s_min + terminal_s_window) {
      low_z_sum += p.point.z;
      ++low_z_count;
    }
    if (p.s >= fragment.s_max - terminal_s_window) {
      high_z_sum += p.point.z;
      ++high_z_count;
    }
  }
  if (low_z_count > 0U && high_z_count > 0U &&
    high_z_sum / static_cast<double>(high_z_count) <
    low_z_sum / static_cast<double>(low_z_count))
  {
    uphill.x = -uphill.x;
    uphill.y = -uphill.y;
    side = uphill.perpendicular();
    rebuild_projections();
    fragment.length_m = fragment.s_max - fragment.s_min;
    fragment.height_m = fragment.z_max - fragment.z_min;
  }

  double s_mean = 0.0;
  double z_mean = 0.0;
  for (const auto & p : projections) {
    s_mean += p.s;
    z_mean += p.point.z;
  }
  s_mean /= static_cast<double>(projections.size());
  z_mean /= static_cast<double>(projections.size());

  double s_var = 0.0;
  double sz_cov = 0.0;
  for (const auto & p : projections) {
    const double ds = p.s - s_mean;
    s_var += ds * ds;
    sz_cov += ds * (p.point.z - z_mean);
  }
  if (s_var <= 1.0e-9) {
    set_reason(StairFlightRejectReason::NoAxis);
    return false;
  }

  const double signed_slope = sz_cov / s_var;
  if (signed_slope < -0.05) {
    set_reason(StairFlightRejectReason::NegativeSlope);
    return false;
  }
  fragment.slope = std::abs(signed_slope);
  if (fragment.slope < 0.12 || fragment.slope > 2.20) {
    set_reason(StairFlightRejectReason::SlopeOutOfRange);
    return false;
  }

  const double intercept = z_mean - signed_slope * s_mean;
  double residual_sum = 0.0;
  int monotonic_edges = 0;
  int nonmonotonic_edges = 0;
  for (const auto & p : projections) {
    const double dz = p.point.z - (signed_slope * p.s + intercept);
    residual_sum += dz * dz;
  }
  fragment.residual = std::sqrt(residual_sum / static_cast<double>(projections.size()));
  if (fragment.residual > std::max(0.45, 5.0 * resolution_m_)) {
    set_reason(StairFlightRejectReason::ResidualTooHigh);
    return false;
  }

  for (const auto & a : projections) {
    for (const auto & b : projections) {
      const double ds = b.s - a.s;
      if (std::abs(ds) < resolution_m_ || std::abs(ds) > 2.5 * resolution_m_) {
        continue;
      }
      const double dz = b.point.z - a.point.z;
      if (ds * dz >= -resolution_m_) {
        ++monotonic_edges;
      } else {
        ++nonmonotonic_edges;
      }
    }
  }
  if (monotonic_edges > 0 && nonmonotonic_edges * 3 > monotonic_edges) {
    set_reason(StairFlightRejectReason::NonMonotonic);
    return false;
  }

  std::vector<double> t_values;
  t_values.reserve(projections.size());
  for (const auto & p : projections) {
    t_values.push_back(p.t);
  }
  std::sort(t_values.begin(), t_values.end());
  auto percentile = [&](double q) {
    const std::size_t index = static_cast<std::size_t>(
      clampValue(q, 0.0, 1.0) * static_cast<double>(t_values.size() - 1U));
    return t_values[index];
  };
  const double t_low = percentile(0.05);
  const double t_high = percentile(0.95);
  fragment.width_m = std::max(0.0, t_high - t_low + resolution_m_);
  const double center_t = 0.5 * (t_low + t_high);
  const double portal_search_m = std::max(1.25, 3.0 * robot_length_m_);

  fragment.id = segment.id;
  fragment.cells = segment.cells;
  fragment.uphill_axis = uphill;
  fragment.side_axis = side;
  fragment.t_min = t_low;
  fragment.t_max = t_high;
  fragment.low_endpoint = {
    uphill.x * fragment.s_min + side.x * center_t,
    uphill.y * fragment.s_min + side.y * center_t,
    signed_slope * fragment.s_min + intercept};
  fragment.high_endpoint = {
    uphill.x * fragment.s_max + side.x * center_t,
    uphill.y * fragment.s_max + side.y * center_t,
    signed_slope * fragment.s_max + intercept};
  fragment.low_component_id = nearestFloorComponent(fragment.low_endpoint, portal_search_m);
  fragment.high_component_id = nearestFloorComponent(fragment.high_endpoint, portal_search_m);
  fragment.prefilter_width_ok = isStairFlightWideEnough(segment);

  const double min_flight_height_m = std::max(0.45, 4.0 * resolution_m_);
  const double min_flight_length_m = std::max(0.60, robot_length_m_);
  const double safety_margin_m = std::max(0.25, 0.5 * resolution_m_);
  fragment.strict_fit_ok =
    fragment.length_m >= min_flight_length_m &&
    fragment.height_m >= min_flight_height_m &&
    fragment.slope >= 0.30 && fragment.slope <= 1.65 &&
    fragment.residual <= std::max(0.35, 3.5 * resolution_m_) &&
    fragment.width_m >= robot_width_m_ + safety_margin_m &&
    (fragment.low_component_id >= 0 || fragment.high_component_id >= 0) &&
    !(fragment.low_component_id >= 0 && fragment.high_component_id >= 0 &&
    fragment.low_component_id == fragment.high_component_id &&
    fragment.height_m > std::max(0.5, 4.0 * resolution_m_));

  if (!fragment.strict_fit_ok) {
    if (fragment.length_m < min_flight_length_m || fragment.height_m < min_flight_height_m) {
      set_reason(StairFlightRejectReason::TooShortOrLow);
    } else if (fragment.slope < 0.30 || fragment.slope > 1.65) {
      set_reason(StairFlightRejectReason::SlopeOutOfRange);
    } else if (fragment.residual > std::max(0.35, 3.5 * resolution_m_)) {
      set_reason(StairFlightRejectReason::ResidualTooHigh);
    } else if (fragment.width_m < robot_width_m_ + safety_margin_m) {
      set_reason(StairFlightRejectReason::TooNarrow);
    } else if (fragment.low_component_id < 0 && fragment.high_component_id < 0) {
      set_reason(StairFlightRejectReason::MissingPortals);
    } else {
      set_reason(StairFlightRejectReason::SameFloorBothEnds);
    }
  }

  return true;
}

bool NavigationMap::fitStairFlightStrict(
  const LooseStairFragment & fragment, StairFlight & flight,
  StairFlightRejectReason * reject_reason) const
{
  StairSegmentInfo segment;
  segment.id = fragment.id;
  segment.cells = fragment.cells;
  segment.z_min = fragment.cells.empty() ? 0 : fragment.cells.front().z;
  segment.z_max = segment.z_min;
  for (const auto & cell : segment.cells) {
    segment.z_min = std::min(segment.z_min, cell.z);
    segment.z_max = std::max(segment.z_max, cell.z);
  }
  return fitStairFlightFromSegment(segment, flight, reject_reason);
}

bool NavigationMap::areLooseFragmentsCompatible(
  const LooseStairFragment & a, const LooseStairFragment & b, FragmentBridge * bridge) const
{
  const double axis_dot = a.uphill_axis.dot(b.uphill_axis);
  if (axis_dot < 0.82) {
    return false;
  }
  if (std::abs(a.side_axis.dot(b.side_axis)) < 0.75) {
    return false;
  }

  const double side_gap =
    std::max({a.t_min - b.t_max, b.t_min - a.t_max, 0.0});
  if (side_gap > std::max(0.75, 1.5 * robot_width_m_ + 2.0 * resolution_m_)) {
    return false;
  }

  const double a_to_b = pointDistance3d(a.high_endpoint, b.low_endpoint);
  const double b_to_a = pointDistance3d(b.high_endpoint, a.low_endpoint);
  const bool forward_ab = a.high_endpoint.z <= b.low_endpoint.z + std::max(0.30, 3.0 * resolution_m_);
  const bool forward_ba = b.high_endpoint.z <= a.low_endpoint.z + std::max(0.30, 3.0 * resolution_m_);
  const bool use_ab = a_to_b <= b_to_a;
  const Point3 from = use_ab ? a.high_endpoint : b.high_endpoint;
  const Point3 to = use_ab ? b.low_endpoint : a.low_endpoint;
  const double endpoint_distance = use_ab ? a_to_b : b_to_a;
  if ((use_ab && !forward_ab) || (!use_ab && !forward_ba)) {
    return false;
  }
  if (endpoint_distance > std::max(1.60, 10.0 * resolution_m_)) {
    return false;
  }

  const double dz = to.z - from.z;
  const double xy = std::hypot(to.x - from.x, to.y - from.y);
  if (xy > 1.0e-6) {
    const double bridge_slope = std::abs(dz) / xy;
    if (bridge_slope > 2.20) {
      return false;
    }
  }
  if (std::abs(dz) > std::max(0.80, 8.0 * resolution_m_)) {
    return false;
  }
  if (hasFloorBetween(from, to)) {
    return false;
  }

  const double merged_height =
    std::max(a.z_max, b.z_max) - std::min(a.z_min, b.z_min);
  const double merged_length =
    std::max(a.s_max, b.s_max) - std::min(a.s_min, b.s_min);
  if (merged_length > 1.0e-6) {
    const double merged_slope = merged_height / merged_length;
    if (merged_slope < 0.12 || merged_slope > 2.20) {
      return false;
    }
  }

  if (bridge != nullptr) {
    bridge->from_fragment_id = use_ab ? a.id : b.id;
    bridge->to_fragment_id = use_ab ? b.id : a.id;
    bridge->from = from;
    bridge->to = to;
  }
  return true;
}

bool NavigationMap::rescueLooseFragment(
  const LooseStairFragment & fragment, const std::vector<std::vector<int>> & graph,
  const std::vector<bool> & strict_ok, StairFragmentRescueReason & reason) const
{
  reason = StairFragmentRescueReason::None;
  if (fragment.id < 0 || static_cast<std::size_t>(fragment.id) >= graph.size()) {
    return false;
  }

  int strict_neighbors = 0;
  int loose_neighbors = 0;
  for (const int neighbor_id : graph[static_cast<std::size_t>(fragment.id)]) {
    if (neighbor_id < 0 || static_cast<std::size_t>(neighbor_id) >= strict_ok.size()) {
      continue;
    }
    ++loose_neighbors;
    if (strict_ok[static_cast<std::size_t>(neighbor_id)]) {
      ++strict_neighbors;
    }
  }

  const bool has_portal = fragment.low_component_id >= 0 || fragment.high_component_id >= 0;
  const bool has_two_portals =
    fragment.low_component_id >= 0 && fragment.high_component_id >= 0 &&
    fragment.low_component_id != fragment.high_component_id;
  const bool monotonic = fragment.slope >= 0.12 && fragment.residual <= std::max(0.45, 5.0 * resolution_m_);

  if (fragment.strict_fit_ok) {
    reason = StairFragmentRescueReason::Strict;
    return true;
  }
  if (has_portal && strict_neighbors > 0) {
    reason = StairFragmentRescueReason::ConnectsFloor;
    return true;
  }
  if (strict_neighbors >= 2 || loose_neighbors >= 2) {
    reason = StairFragmentRescueReason::BridgesFragments;
    return true;
  }
  if (strict_neighbors == 1 && fragment.reject_reason == StairFlightRejectReason::TooShortOrLow) {
    reason = StairFragmentRescueReason::ExtendsFlight;
    return true;
  }
  if (has_two_portals && monotonic && fragment.height_m >= std::max(0.25, 2.0 * resolution_m_)) {
    reason = StairFragmentRescueReason::BetweenFloors;
    return true;
  }
  return false;
}

StairSegmentInfo NavigationMap::makeSegmentFromFragmentIds(const std::vector<int> & fragment_ids) const
{
  StairSegmentInfo segment;
  for (const int fragment_id : fragment_ids) {
    if (fragment_id < 0 ||
      static_cast<std::size_t>(fragment_id) >= loose_stair_fragments_.size())
    {
      continue;
    }
    const auto & fragment = loose_stair_fragments_[static_cast<std::size_t>(fragment_id)];
    segment.cells.insert(segment.cells.end(), fragment.cells.begin(), fragment.cells.end());
  }
  std::sort(
    segment.cells.begin(), segment.cells.end(),
    [](const GridIndex & lhs, const GridIndex & rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      if (lhs.y != rhs.y) {
        return lhs.y < rhs.y;
      }
      return lhs.z < rhs.z;
    });
  segment.cells.erase(std::unique(segment.cells.begin(), segment.cells.end()), segment.cells.end());
  if (!segment.cells.empty()) {
    segment.z_min = segment.cells.front().z;
    segment.z_max = segment.cells.front().z;
    for (const auto & cell : segment.cells) {
      segment.z_min = std::min(segment.z_min, cell.z);
      segment.z_max = std::max(segment.z_max, cell.z);
    }
  }
  return segment;
}

bool NavigationMap::fitStairFlightFromSegment(
  const StairSegmentInfo & segment, StairFlight & flight,
  StairFlightRejectReason * reject_reason) const
{
  auto reject = [&](StairFlightRejectReason reason) {
    if (reject_reason != nullptr) {
      *reject_reason = reason;
    }
    return false;
  };
  if (reject_reason != nullptr) {
    *reject_reason = StairFlightRejectReason::None;
  }

  if (segment.cells.size() < 8U) {
    return reject(StairFlightRejectReason::TooFewCells);
  }

  Vec2 axis_sum;
  for (const auto & cell : segment.cells) {
    double slope_x = 0.0;
    double slope_y = 0.0;
    if (stairSlope(cell, slope_x, slope_y)) {
      axis_sum.x += slope_x;
      axis_sum.y += slope_y;
    }
  }

  Vec2 uphill = axis_sum.normalized();
  if (uphill.norm() <= 1.0e-9) {
    Point3 centroid;
    for (const auto & cell : segment.cells) {
      const Point3 point = gridToWorld(cell);
      centroid.x += point.x;
      centroid.y += point.y;
      centroid.z += point.z;
    }
    const double inv_count = 1.0 / static_cast<double>(segment.cells.size());
    centroid.x *= inv_count;
    centroid.y *= inv_count;
    centroid.z *= inv_count;

    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;
    double sxz = 0.0;
    double syz = 0.0;
    for (const auto & cell : segment.cells) {
      const Point3 point = gridToWorld(cell);
      const double x = point.x - centroid.x;
      const double y = point.y - centroid.y;
      const double z = point.z - centroid.z;
      cxx += x * x;
      cxy += x * y;
      cyy += y * y;
      sxz += x * z;
      syz += y * z;
    }
    const double angle = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
    uphill = Vec2{std::cos(angle), std::sin(angle)}.normalized();
    if (uphill.x * sxz + uphill.y * syz < 0.0) {
      uphill.x = -uphill.x;
      uphill.y = -uphill.y;
    }
  }
  if (uphill.norm() <= 1.0e-9) {
    return reject(StairFlightRejectReason::NoAxis);
  }

  Vec2 side = uphill.perpendicular();
  struct Projection
  {
    GridIndex cell;
    Point3 point;
    double s{0.0};
    double t{0.0};
  };

  std::vector<Projection> projections;
  projections.reserve(segment.cells.size());
  double s_min = 0.0;
  double s_max = 0.0;
  double t_min = 0.0;
  double t_max = 0.0;
  double z_min = 0.0;
  double z_max = 0.0;

  auto rebuild_projections = [&]() {
    projections.clear();
    s_min = std::numeric_limits<double>::infinity();
    s_max = -std::numeric_limits<double>::infinity();
    t_min = std::numeric_limits<double>::infinity();
    t_max = -std::numeric_limits<double>::infinity();
    z_min = std::numeric_limits<double>::infinity();
    z_max = -std::numeric_limits<double>::infinity();
    for (const auto & cell : segment.cells) {
      const Point3 point = gridToWorld(cell);
      const double s = point.x * uphill.x + point.y * uphill.y;
      const double t = point.x * side.x + point.y * side.y;
      projections.push_back({cell, point, s, t});
      s_min = std::min(s_min, s);
      s_max = std::max(s_max, s);
      t_min = std::min(t_min, t);
      t_max = std::max(t_max, t);
      z_min = std::min(z_min, point.z);
      z_max = std::max(z_max, point.z);
    }
  };
  rebuild_projections();

  const double length_m = s_max - s_min;
  const double height_m = z_max - z_min;
  const double min_flight_height_m = std::max(0.45, 4.0 * resolution_m_);
  const double min_flight_length_m = std::max(0.60, robot_length_m_);
  if (length_m < min_flight_length_m || height_m < min_flight_height_m) {
    return reject(StairFlightRejectReason::TooShortOrLow);
  }

  double low_z_sum = 0.0;
  double high_z_sum = 0.0;
  std::size_t low_z_count = 0U;
  std::size_t high_z_count = 0U;
  const double terminal_s_window = std::max(2.0 * resolution_m_, 0.20 * length_m);
  for (const auto & p : projections) {
    if (p.s <= s_min + terminal_s_window) {
      low_z_sum += p.point.z;
      ++low_z_count;
    }
    if (p.s >= s_max - terminal_s_window) {
      high_z_sum += p.point.z;
      ++high_z_count;
    }
  }
  if (low_z_count > 0U && high_z_count > 0U &&
    high_z_sum / static_cast<double>(high_z_count) <
    low_z_sum / static_cast<double>(low_z_count))
  {
    uphill.x = -uphill.x;
    uphill.y = -uphill.y;
    side = uphill.perpendicular();
    rebuild_projections();
  }

  double s_mean = 0.0;
  double z_mean = 0.0;
  for (const auto & p : projections) {
    s_mean += p.s;
    z_mean += p.point.z;
  }
  s_mean /= static_cast<double>(projections.size());
  z_mean /= static_cast<double>(projections.size());

  double s_var = 0.0;
  double sz_cov = 0.0;
  for (const auto & p : projections) {
    const double ds = p.s - s_mean;
    s_var += ds * ds;
    sz_cov += ds * (p.point.z - z_mean);
  }
  if (s_var <= 1.0e-9) {
    return reject(StairFlightRejectReason::NoAxis);
  }
  const double slope = sz_cov / s_var;
  if (slope < 0.0) {
    return reject(StairFlightRejectReason::NegativeSlope);
  }
  const double abs_slope = std::abs(slope);
  if (abs_slope < 0.30 || abs_slope > 1.65) {
    return reject(StairFlightRejectReason::SlopeOutOfRange);
  }

  double residual_sum = 0.0;
  int monotonic_edges = 0;
  int nonmonotonic_edges = 0;
  const double intercept = z_mean - slope * s_mean;
  for (const auto & p : projections) {
    const double dz = p.point.z - (slope * p.s + intercept);
    residual_sum += dz * dz;
  }
  const double rms_residual =
    std::sqrt(residual_sum / static_cast<double>(projections.size()));
  if (rms_residual > std::max(0.35, 3.5 * resolution_m_)) {
    return reject(StairFlightRejectReason::ResidualTooHigh);
  }

  for (const auto & a : projections) {
    for (const auto & b : projections) {
      const double ds = b.s - a.s;
      if (std::abs(ds) < resolution_m_ || std::abs(ds) > 2.5 * resolution_m_) {
        continue;
      }
      const double dz = b.point.z - a.point.z;
      if (ds * dz >= -resolution_m_) {
        ++monotonic_edges;
      } else {
        ++nonmonotonic_edges;
      }
    }
  }
  if (monotonic_edges > 0 && nonmonotonic_edges * 4 > monotonic_edges) {
    return reject(StairFlightRejectReason::NonMonotonic);
  }

  std::vector<double> t_values;
  t_values.reserve(projections.size());
  for (const auto & p : projections) {
    t_values.push_back(p.t);
  }
  std::sort(t_values.begin(), t_values.end());
  const auto percentile = [&](double q) {
    const std::size_t index = static_cast<std::size_t>(
      clampValue(q, 0.0, 1.0) * static_cast<double>(t_values.size() - 1U));
    return t_values[index];
  };
  const double t_low = percentile(0.05);
  const double t_high = percentile(0.95);
  const double width_m = std::max(0.0, t_high - t_low + resolution_m_);
  const double safety_margin_m = std::max(0.25, 0.5 * resolution_m_);
  if (width_m < robot_width_m_ + safety_margin_m) {
    return reject(StairFlightRejectReason::TooNarrow);
  }

  struct TreadSliceEvidence
  {
    std::vector<double> s_values;
    std::vector<double> t_values;
  };
  std::unordered_map<int, TreadSliceEvidence> tread_slices;
  tread_slices.reserve(projections.size());
  for (const auto & p : projections) {
    auto & slice = tread_slices[p.cell.z];
    slice.s_values.push_back(p.s);
    slice.t_values.push_back(p.t);
  }

  int supported_tread_levels = 0;
  int min_supported_z = std::numeric_limits<int>::max();
  int max_supported_z = std::numeric_limits<int>::min();
  const double min_tread_lateral_width_m = robot_width_m_ + std::max(0.10, resolution_m_);
  const double min_tread_along_span_m = std::max(0.15, 1.5 * resolution_m_);
  for (auto & entry : tread_slices) {
    auto & slice = entry.second;
    if (slice.s_values.size() < 4U || slice.t_values.size() < 4U) {
      continue;
    }
    std::sort(slice.s_values.begin(), slice.s_values.end());
    std::sort(slice.t_values.begin(), slice.t_values.end());
    const double s_span = slice.s_values.back() - slice.s_values.front() + resolution_m_;
    const double t_span = slice.t_values.back() - slice.t_values.front() + resolution_m_;
    if (s_span < min_tread_along_span_m || t_span < min_tread_lateral_width_m) {
      continue;
    }
    ++supported_tread_levels;
    min_supported_z = std::min(min_supported_z, entry.first);
    max_supported_z = std::max(max_supported_z, entry.first);
  }
  const int min_supported_tread_levels =
    std::max(3, static_cast<int>(std::ceil(min_flight_height_m / resolution_m_)));
  if (supported_tread_levels < min_supported_tread_levels ||
    (max_supported_z - min_supported_z) * resolution_m_ < min_flight_height_m)
  {
    return reject(StairFlightRejectReason::TooShortOrLow);
  }

  const double safe_half_width_m =
    std::max(0.5 * resolution_m_, 0.5 * width_m - 0.5 * robot_width_m_ - safety_margin_m);
  const double center_t = 0.5 * (t_low + t_high);

  const double portal_search_m = std::max(1.25, 3.0 * robot_length_m_);
  Point3 low_endpoint{
    uphill.x * s_min + side.x * center_t,
    uphill.y * s_min + side.y * center_t,
    slope * s_min + intercept};
  Point3 high_endpoint{
    uphill.x * s_max + side.x * center_t,
    uphill.y * s_max + side.y * center_t,
    slope * s_max + intercept};
  const int low_component = nearestFloorComponent(low_endpoint, portal_search_m);
  const int high_component = nearestFloorComponent(high_endpoint, portal_search_m);
  if (low_component < 0 && high_component < 0) {
    return reject(StairFlightRejectReason::MissingPortals);
  }
  if (low_component >= 0 && high_component >= 0 && low_component == high_component &&
    height_m > std::max(0.5, 4.0 * resolution_m_))
  {
    return reject(StairFlightRejectReason::SameFloorBothEnds);
  }
  const double slice_m = std::max(2.0 * resolution_m_, 0.20);
  const int bin_count = std::max(2, static_cast<int>(std::ceil(length_m / slice_m)));
  std::vector<std::vector<const Projection *>> bins(static_cast<std::size_t>(bin_count));
  for (const auto & p : projections) {
    const int bin = clampValue(
      static_cast<int>(std::floor((p.s - s_min) / slice_m)), 0, bin_count - 1);
    bins[static_cast<std::size_t>(bin)].push_back(&p);
  }

  std::vector<Point3> centerline;
  centerline.reserve(static_cast<std::size_t>(bin_count));
  for (int bin = 0; bin < bin_count; ++bin) {
    auto & slice = bins[static_cast<std::size_t>(bin)];
    if (slice.size() < 2U) {
      continue;
    }
    std::vector<double> slice_t;
    slice_t.reserve(slice.size());
    double s_sum = 0.0;
    double z_sum = 0.0;
    for (const auto * p : slice) {
      slice_t.push_back(p->t);
      s_sum += p->s;
      z_sum += p->point.z;
    }
    std::sort(slice_t.begin(), slice_t.end());
    const double left_t = slice_t.front();
    const double right_t = slice_t.back();
    const double s = s_sum / static_cast<double>(slice.size());
    const double t = 0.5 * (left_t + right_t);
    centerline.push_back({
      uphill.x * s + side.x * t,
      uphill.y * s + side.y * t,
      z_sum / static_cast<double>(slice.size())});
  }

  if (centerline.size() < 2U) {
    centerline = {low_endpoint, high_endpoint};
  } else {
    centerline.front() = low_endpoint;
    centerline.back() = high_endpoint;
  }

  flight.cells = segment.cells;
  flight.uphill_axis = uphill;
  flight.side_axis = side;
  flight.z_min = z_min;
  flight.z_max = z_max;
  flight.length_m = length_m;
  flight.width_m = width_m;
  flight.slope = abs_slope;
  flight.low_component_id = low_component;
  flight.high_component_id = high_component;
  flight.low_endpoint = low_endpoint;
  flight.high_endpoint = high_endpoint;
  flight.centerline = std::move(centerline);
  flight.safe_half_width_m = safe_half_width_m;
  flight.score = static_cast<double>(segment.cells.size()) /
    (1.0 + rms_residual + std::max(0.0, robot_width_m_ + safety_margin_m - width_m));
  return true;
}

void NavigationMap::rebuildStairFlights()
{
  stair_flights_.clear();
  stair_cell_info_.clear();
  rejected_stair_noise_cells_.clear();
  loose_stair_fragments_.clear();
  fragment_bridges_.clear();
  loose_stair_fragment_cells_.clear();
  rejected_short_low_cells_.clear();
  rescued_stair_fragment_cells_.clear();

  for (const auto & segment : stair_segments_) {
    LooseStairFragment fragment;
    StairFlightRejectReason reject_reason = StairFlightRejectReason::None;
    if (!fitStairFragmentLoose(segment, fragment, &reject_reason)) {
      ++stair_flight_diagnostics_.fit_rejected;
      const auto reject_index = static_cast<std::size_t>(reject_reason);
      if (reject_index < stair_flight_diagnostics_.fit_reject_counts.size()) {
        ++stair_flight_diagnostics_.fit_reject_counts[reject_index];
      }
      for (const auto & cell : segment.cells) {
        rejected_stair_noise_cells_.insert(cell);
      }
      continue;
    }
    fragment.id = static_cast<int>(loose_stair_fragments_.size());
    if (fragment.reject_reason == StairFlightRejectReason::None && !fragment.strict_fit_ok) {
      fragment.reject_reason = reject_reason;
    }
    for (const auto & cell : fragment.cells) {
      loose_stair_fragment_cells_.insert(cell);
    }
    loose_stair_fragments_.push_back(std::move(fragment));
  }

  stair_flight_diagnostics_.loose_fragment_count = loose_stair_fragments_.size();

  std::vector<std::vector<int>> fragment_graph(loose_stair_fragments_.size());
  for (std::size_t i = 0; i < loose_stair_fragments_.size(); ++i) {
    for (std::size_t j = i + 1; j < loose_stair_fragments_.size(); ++j) {
      FragmentBridge bridge;
      if (!areLooseFragmentsCompatible(loose_stair_fragments_[i], loose_stair_fragments_[j], &bridge)) {
        continue;
      }
      fragment_graph[i].push_back(static_cast<int>(j));
      fragment_graph[j].push_back(static_cast<int>(i));
      fragment_bridges_.push_back(bridge);
    }
  }

  std::vector<bool> strict_ok(loose_stair_fragments_.size(), false);
  for (std::size_t i = 0; i < loose_stair_fragments_.size(); ++i) {
    StairFlight strict_flight;
    StairFlightRejectReason strict_reason = StairFlightRejectReason::None;
    strict_ok[i] = fitStairFlightStrict(loose_stair_fragments_[i], strict_flight, &strict_reason);
    loose_stair_fragments_[i].strict_fit_ok = strict_ok[i];
    if (strict_ok[i]) {
      ++stair_flight_diagnostics_.strict_fragment_count;
      loose_stair_fragments_[i].rescue_reason = StairFragmentRescueReason::Strict;
    } else {
      loose_stair_fragments_[i].reject_reason = strict_reason;
      const auto reject_index = static_cast<std::size_t>(strict_reason);
      if (reject_index < stair_flight_diagnostics_.fit_reject_counts.size()) {
        ++stair_flight_diagnostics_.fit_reject_counts[reject_index];
      }
    }
  }

  std::vector<bool> accepted_fragment(loose_stair_fragments_.size(), false);
  for (std::size_t i = 0; i < loose_stair_fragments_.size(); ++i) {
    StairFragmentRescueReason rescue_reason = StairFragmentRescueReason::None;
    if (rescueLooseFragment(
        loose_stair_fragments_[i], fragment_graph, strict_ok, rescue_reason))
    {
      accepted_fragment[i] = true;
      loose_stair_fragments_[i].rescued = !strict_ok[i];
      loose_stair_fragments_[i].rescue_reason = rescue_reason;
      if (!strict_ok[i]) {
        ++stair_flight_diagnostics_.rescued_fragment_count;
        for (const auto & cell : loose_stair_fragments_[i].cells) {
          rescued_stair_fragment_cells_.insert(cell);
        }
      }
    } else if (loose_stair_fragments_[i].reject_reason == StairFlightRejectReason::TooShortOrLow) {
      for (const auto & cell : loose_stair_fragments_[i].cells) {
        rejected_short_low_cells_.insert(cell);
      }
    }
  }

  std::vector<StairFlight> candidate_flights;
  std::vector<bool> visited(loose_stair_fragments_.size(), false);
  for (std::size_t seed = 0; seed < loose_stair_fragments_.size(); ++seed) {
    if (!accepted_fragment[seed] || visited[seed]) {
      continue;
    }
    std::vector<int> chain;
    std::deque<int> queue;
    visited[seed] = true;
    queue.push_back(static_cast<int>(seed));
    bool chain_has_strict = false;
    while (!queue.empty()) {
      const int current = queue.front();
      queue.pop_front();
      chain.push_back(current);
      chain_has_strict = chain_has_strict || strict_ok[static_cast<std::size_t>(current)];
      for (const int neighbor : fragment_graph[static_cast<std::size_t>(current)]) {
        if (neighbor < 0 || static_cast<std::size_t>(neighbor) >= accepted_fragment.size()) {
          continue;
        }
        if (!accepted_fragment[static_cast<std::size_t>(neighbor)] ||
          visited[static_cast<std::size_t>(neighbor)])
        {
          continue;
        }
        visited[static_cast<std::size_t>(neighbor)] = true;
        queue.push_back(neighbor);
      }
    }

    StairSegmentInfo merged_segment = makeSegmentFromFragmentIds(chain);
    StairFlight merged_flight;
    StairFlightRejectReason chain_reject = StairFlightRejectReason::None;
    if (chain_has_strict && fitStairFlightFromSegment(merged_segment, merged_flight, &chain_reject)) {
      candidate_flights.push_back(std::move(merged_flight));
      continue;
    }

    for (const int fragment_id : chain) {
      if (!strict_ok[static_cast<std::size_t>(fragment_id)]) {
        continue;
      }
      StairFlight flight;
      if (fitStairFlightStrict(loose_stair_fragments_[static_cast<std::size_t>(fragment_id)], flight)) {
        candidate_flights.push_back(std::move(flight));
      }
    }
  }

  stair_flight_diagnostics_.accepted_candidates = candidate_flights.size();
  stair_flight_diagnostics_.merged_candidates = candidate_flights.size();
  stair_flight_diagnostics_.final_stair_flight_count = candidate_flights.size();
  std::unordered_set<GridIndex, GridIndexHash> accepted_flight_cells;
  accepted_flight_cells.reserve(accepted_stair_cells_.size());

  for (auto & flight : candidate_flights) {
    flight.id = static_cast<int>(stair_flights_.size());
    const double center_t = 0.5 *
      (flight.low_endpoint.x * flight.side_axis.x + flight.low_endpoint.y * flight.side_axis.y +
      flight.high_endpoint.x * flight.side_axis.x + flight.high_endpoint.y * flight.side_axis.y);
    for (const auto & cell : flight.cells) {
      const Point3 point = gridToWorld(cell);
      StairCellInfo info;
      info.stair_flight_id = flight.id;
      info.along_s = point.x * flight.uphill_axis.x + point.y * flight.uphill_axis.y;
      info.lateral_t = point.x * flight.side_axis.x + point.y * flight.side_axis.y;
      info.lateral_error_m = std::abs(info.lateral_t - center_t);
      if (info.lateral_error_m <= flight.safe_half_width_m + 1.0e-9) {
        stair_cell_info_[cell] = info;
        accepted_flight_cells.insert(cell);
      } else {
        rejected_stair_noise_cells_.insert(cell);
      }
    }
    stair_flights_.push_back(std::move(flight));
  }

  for (const auto & cell : accepted_stair_cells_) {
    if (accepted_flight_cells.find(cell) == accepted_flight_cells.end()) {
      if (hasNearbyAcceptedFloor(cell)) {
        rejected_stair_noise_cells_.erase(cell);
        forbidden_cells_.erase(cell);
        accepted_floor_cells_.insert(cell);
        traversable_cells_.insert(cell);
      } else {
        rejected_stair_noise_cells_.insert(cell);
        traversable_cells_.erase(cell);
        forbidden_cells_.insert(cell);
      }
    }
  }
  accepted_stair_cells_ = std::move(accepted_flight_cells);
  rebuildFloorComponents();
}

double NavigationMap::getStairCenterCost(const GridIndex & idx) const
{
  const int flight_id = stairFlightId(idx);
  if (flight_id >= 0 && static_cast<std::size_t>(flight_id) < stair_flights_.size()) {
    const auto info_it = stair_cell_info_.find(idx);
    if (info_it == stair_cell_info_.end()) {
      return 4.0;
    }
    const auto & flight = stair_flights_[flight_id];
    const double normalized_error =
      flight.safe_half_width_m > 1.0e-9 ?
      info_it->second.lateral_error_m / flight.safe_half_width_m : 1.0;
    return 1.2 * normalized_error * normalized_error;
  }

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
    std::max(1, static_cast<int>(std::ceil(0.5 * robot_width_m_ / resolution_m_)));
  const int edge_shortfall = std::max(0, desired_margin_cells - min_side);
  const double imbalance =
    static_cast<double>(std::abs(left - right)) / static_cast<double>(side_sum + 1);
  return 0.35 * static_cast<double>(edge_shortfall) + 0.45 * imbalance;
}

std::vector<std::vector<Point3>> NavigationMap::stairCenterlines() const
{
  std::vector<std::vector<Point3>> centerlines;
  centerlines.reserve(stair_flights_.size());
  for (const auto & flight : stair_flights_) {
    if (flight.centerline.size() >= 2U) {
      centerlines.push_back(flight.centerline);
    }
  }
  return centerlines;
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

    SurfaceKind kind = SurfaceKind::NoiseRejected;
    if (component.size() >= min_flat_component_size && z_range <= flat_z_range_cells) {
      kind = overhead_ratio >= min_floor_overhead_ratio ?
        SurfaceKind::Floor : SurfaceKind::CeilingRejected;
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
        component_kind[component_id] = SurfaceKind::NoiseRejected;
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
      } else if (kind == SurfaceKind::CeilingRejected) {
        candidate.reject_reason = SurfaceRejectReason::CeilingLike;
        rejected_ceiling_cells_.insert(candidate.stand);
        forbidden_cells_.insert(candidate.stand);
      } else {
        candidate.reject_reason = SurfaceRejectReason::Noise;
        rejected_stair_noise_cells_.insert(candidate.stand);
        forbidden_cells_.insert(candidate.stand);
      }
    }
  }

  rebuildFloorComponents();
  recoverMissingStairCells();
  rebuildFloorComponents();
  rebuildStairSegments();
  rebuildStairFlights();

  for (const auto & blocked : blocked_cells_) {
    forbidden_cells_.insert(blocked);
  }
}

void NavigationMap::rebuildRiskLayer()
{
  risk_cost_.clear();
  if (traversable_cells_.empty()) {
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

  struct ClearanceNode
  {
    GridIndex cell;
    double distance_m{0.0};
  };

  struct ClearanceNodeCompare
  {
    bool operator()(const ClearanceNode & lhs, const ClearanceNode & rhs) const
    {
      return lhs.distance_m > rhs.distance_m;
    }
  };

  const double boundary_cost_weight = 0.35;
  const int z_tolerance = 1;
  auto is_floor_surface = [&](const GridIndex & idx) {
    return isTraversable(idx) && !isStairTraversable(idx);
  };
  auto has_nearby_floor_surface = [&](const GridIndex & idx) {
    for (int dz = -z_tolerance; dz <= z_tolerance; ++dz) {
      if (is_floor_surface({idx.x, idx.y, idx.z + dz})) {
        return true;
      }
    }
    return false;
  };

  std::priority_queue<
    ClearanceNode, std::vector<ClearanceNode>, ClearanceNodeCompare> clearance_queue;
  std::unordered_map<GridIndex, double, GridIndexHash> clearance_m;
  clearance_m.reserve(accepted_floor_cells_.size());

  for (const auto & cell : traversable_cells_) {
    if (!is_floor_surface(cell)) {
      continue;
    }
    bool boundary = false;
    for (int dx = -1; dx <= 1 && !boundary; ++dx) {
      for (int dy = -1; dy <= 1 && !boundary; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (!has_nearby_floor_surface({cell.x + dx, cell.y + dy, cell.z})) {
          boundary = true;
        }
      }
    }
    if (!boundary) {
      continue;
    }
    clearance_m[cell] = 0.0;
    clearance_queue.push({cell, 0.0});
  }

  while (!clearance_queue.empty()) {
    const ClearanceNode current = clearance_queue.top();
    clearance_queue.pop();
    const auto best_it = clearance_m.find(current.cell);
    if (best_it == clearance_m.end() || current.distance_m > best_it->second + 1.0e-9) {
      continue;
    }
    if (current.distance_m > risk_inflation_radius_m_) {
      continue;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        for (int dz = -z_tolerance; dz <= z_tolerance; ++dz) {
          const GridIndex neighbor{current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          if (!is_floor_surface(neighbor)) {
            continue;
          }
          const double step_distance =
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_m_;
          const double candidate_distance = current.distance_m + step_distance;
          if (candidate_distance > risk_inflation_radius_m_) {
            continue;
          }
          const auto old_it = clearance_m.find(neighbor);
          if (old_it != clearance_m.end() && candidate_distance >= old_it->second - 1.0e-9) {
            continue;
          }
          clearance_m[neighbor] = candidate_distance;
          clearance_queue.push({neighbor, candidate_distance});
        }
      }
    }
  }

  for (const auto & entry : clearance_m) {
    const double normalized = std::max(0.0, 1.0 - entry.second / risk_inflation_radius_m_);
    const double cost = boundary_cost_weight * normalized * normalized;
    auto it = risk_cost_.find(entry.first);
    if (it == risk_cost_.end() || cost > it->second) {
      risk_cost_[entry.first] = cost;
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

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedStairNoiseCells() const
{
  return rejected_stair_noise_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::looseStairFragmentCells() const
{
  return loose_stair_fragment_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedShortLowCells() const
{
  return rejected_short_low_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rejectedWidthPrefilterCells() const
{
  return rejected_width_prefilter_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::rescuedStairFragmentCells() const
{
  return rescued_stair_fragment_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::missingStairRecoveryCells() const
{
  return missing_stair_recovery_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & NavigationMap::blockedCells() const
{
  return blocked_cells_;
}

const std::unordered_map<GridIndex, double, GridIndexHash> & NavigationMap::riskCosts() const
{
  return risk_cost_;
}

const std::vector<FloorComponent> & NavigationMap::floorComponents() const
{
  return floor_components_;
}

const std::vector<FloorComponent> & NavigationMap::landingComponents() const
{
  return landing_components_;
}

const std::vector<StairFlight> & NavigationMap::stairFlights() const
{
  return stair_flights_;
}

const std::vector<LooseStairFragment> & NavigationMap::looseStairFragments() const
{
  return loose_stair_fragments_;
}

const std::vector<FragmentBridge> & NavigationMap::fragmentBridges() const
{
  return fragment_bridges_;
}

const StairFlightDiagnostics & NavigationMap::stairFlightDiagnostics() const
{
  return stair_flight_diagnostics_;
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

double NavigationMap::robotLength() const
{
  return robot_length_m_;
}

double NavigationMap::robotWidth() const
{
  return robot_width_m_;
}

double NavigationMap::baseToFront() const
{
  return base_to_front_m_;
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
