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
  blocked_cells_.clear();
  risk_cost_.clear();
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

void NavigationMap::rebuildTraversableLayer()
{
  traversable_cells_.clear();
  traversable_cells_.reserve(occupied_cells_.size() / 4U);
  std::unordered_set<GridIndex, GridIndexHash> candidates;
  candidates.reserve(occupied_cells_.size() / 4U);
  int min_candidate_z = std::numeric_limits<int>::max();

  for (const auto & support : occupied_cells_) {
    const GridIndex candidate{support.x, support.y, support.z + 1};
    if (!isInsideBounds(candidate) || isOccupied(candidate) || isBlocked(candidate)) {
      continue;
    }
    if (!hasGroundSupport(candidate) || !isCollisionFreeForRobot(candidate)) {
      continue;
    }
    candidates.insert(candidate);
    min_candidate_z = std::min(min_candidate_z, candidate.z);
  }

  if (candidates.empty()) {
    return;
  }

  const int seed_band_cells = std::max(1, static_cast<int>(std::ceil(0.6 / resolution_m_)));
  const int max_step_cells = std::max(1, static_cast<int>(std::ceil(0.45 / resolution_m_)));
  std::deque<GridIndex> queue;
  for (const auto & candidate : candidates) {
    if (candidate.z <= min_candidate_z + seed_band_cells) {
      traversable_cells_.insert(candidate);
      queue.push_back(candidate);
    }
  }

  while (!queue.empty()) {
    const GridIndex current = queue.front();
    queue.pop_front();

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
          if (candidates.find(neighbor) == candidates.end()) {
            continue;
          }
          if (traversable_cells_.find(neighbor) != traversable_cells_.end()) {
            continue;
          }
          traversable_cells_.insert(neighbor);
          queue.push_back(neighbor);
        }
      }
    }
  }

  if (traversable_cells_.empty()) {
    for (const auto & candidate : candidates) {
      traversable_cells_.insert(candidate);
    }
    return;
  }

  const std::size_t min_component_size = 20U;
  std::unordered_set<GridIndex, GridIndexHash> visited = traversable_cells_;
  for (const auto & seed : candidates) {
    if (visited.find(seed) != visited.end()) {
      continue;
    }

    std::vector<GridIndex> component;
    std::deque<GridIndex> component_queue;
    visited.insert(seed);
    component_queue.push_back(seed);
    while (!component_queue.empty()) {
      const GridIndex current = component_queue.front();
      component_queue.pop_front();
      component.push_back(current);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -max_step_cells; dz <= max_step_cells; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const GridIndex neighbor{current.x + dx, current.y + dy, current.z + dz};
            if (candidates.find(neighbor) == candidates.end() ||
              visited.find(neighbor) != visited.end())
            {
              continue;
            }
            visited.insert(neighbor);
            component_queue.push_back(neighbor);
          }
        }
      }
    }

    if (component.size() >= min_component_size) {
      const auto minmax_z = std::minmax_element(
        component.begin(), component.end(),
        [](const GridIndex & lhs, const GridIndex & rhs) {return lhs.z < rhs.z;});
      const int component_height_cells = minmax_z.second->z - minmax_z.first->z;
      if (component_height_cells >= max_step_cells * 2) {
        for (const auto & idx : component) {
          traversable_cells_.insert(idx);
        }
        continue;
      }
    }
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
