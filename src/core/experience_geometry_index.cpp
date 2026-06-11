#include "tgw_planner/core/experience_geometry_index.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace tgw_planner::core
{

namespace
{
constexpr int kNeighborRadius = 1;
constexpr int kMinLocalSupportCells = 3;
}

ExperienceGeometryIndexBuildResult ExperienceGeometryIndex::build(
  const N3NavResource & resource,
  ExperienceGeometryIndexOptions options)
{
  const auto t0 = std::chrono::steady_clock::now();
  options_ = options;
  if (options_.raw_resolution_m <= 0.0) {
    options_.raw_resolution_m = 0.05;
  }
  if (options_.nav_resolution_m <= 0.0) {
    options_.nav_resolution_m = 0.10;
  }
  if (options_.body_clearance_height_m < 0.0) {
    options_.body_clearance_height_m = 0.0;
  }

  support_columns_.clear();
  raw_geometry_.clear();
  support_candidates_.clear();
  debug_world_points_.clear();
  metrics_ = {};

  std::size_t total_points = 0U;
  for (const N3KeyframeLite & keyframe : resource.keyframes) {
    total_points += keyframe.cloud_body.size();
  }
  if (total_points == 0U) {
    metrics_.error_code = "pbstream_no_keyframe_points";
    metrics_.message = "keyframes contain no point cloud geometry";
    return metrics_;
  }

  const std::size_t max_debug_points = options_.max_debug_world_points;
  const std::size_t debug_stride =
    max_debug_points > 0U && total_points > max_debug_points ?
    ((total_points + max_debug_points - 1U) / max_debug_points) : 1U;
  if (max_debug_points > 0U) {
    debug_world_points_.reserve((total_points + debug_stride - 1U) / debug_stride);
  }

  std::size_t point_index = 0U;
  for (const N3KeyframeLite & keyframe : resource.keyframes) {
    for (const PointXYZI & point_body : keyframe.cloud_body) {
      const Point3 world = transformPoint(
        keyframe.pose_optimized, {point_body.x, point_body.y, point_body.z});
      ++metrics_.transformed_points;
      support_columns_[rawColumnKey(world)].push_back(world.z);

      const GridIndex cell = navCellKey(world);
      SurfaceCell & surface_cell = raw_geometry_[cell];
      surface_cell.cell = cell;
      surface_cell.support = {cell.x, cell.y, cell.z - 1};
      surface_cell.label = SurfaceLabel::GeometrySupport;
      surface_cell.reachability = ReachabilityLabel::Unknown;
      surface_cell.height_m = world.z;
      surface_cell.confidence = std::max(surface_cell.confidence, 0.25);

      if (max_debug_points > 0U && (point_index % debug_stride) == 0U) {
        debug_world_points_.push_back(world);
      }
      ++point_index;
    }
  }

  for (auto & entry : support_columns_) {
    auto & heights = entry.second;
    std::sort(heights.begin(), heights.end());
    heights.erase(std::unique(heights.begin(), heights.end()), heights.end());
  }

  markBodyObstructions(raw_geometry_);
  support_candidates_ = buildSupportCandidates();
  markBodyObstructions(support_candidates_);

  metrics_.raw_geometry_cell_count = raw_geometry_.size();
  metrics_.support_candidate_count = support_candidates_.size();
  metrics_.support_column_count = support_columns_.size();
  metrics_.debug_world_point_count = debug_world_points_.size();
  if (support_candidates_.empty()) {
    metrics_.error_code = "pbstream_no_support_candidates";
    metrics_.message = "keyframes are present but contain no usable support candidates";
    return metrics_;
  }
  metrics_.success = true;
  const auto t1 = std::chrono::steady_clock::now();
  metrics_.build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return metrics_;
}

bool ExperienceGeometryIndex::empty() const
{
  return support_candidates_.empty();
}

double ExperienceGeometryIndex::rawResolution() const
{
  return options_.raw_resolution_m;
}

double ExperienceGeometryIndex::navResolution() const
{
  return options_.nav_resolution_m;
}

const SupportColumns & ExperienceGeometryIndex::supportColumns() const
{
  return support_columns_;
}

const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> &
ExperienceGeometryIndex::rawGeometry() const
{
  return raw_geometry_;
}

const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> &
ExperienceGeometryIndex::supportCandidates() const
{
  return support_candidates_;
}

const std::vector<Point3> & ExperienceGeometryIndex::debugWorldPoints() const
{
  return debug_world_points_;
}

const ExperienceGeometryIndexBuildResult & ExperienceGeometryIndex::metrics() const
{
  return metrics_;
}

GridIndex ExperienceGeometryIndex::rawColumnKey(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.raw_resolution_m)),
    static_cast<int>(std::floor(point.y / options_.raw_resolution_m)),
    0};
}

GridIndex ExperienceGeometryIndex::navCellKey(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.nav_resolution_m)),
    static_cast<int>(std::floor(point.y / options_.nav_resolution_m)),
    static_cast<int>(std::floor(point.z / options_.nav_resolution_m))};
}

void ExperienceGeometryIndex::markBodyObstructions(
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry) const
{
  const int body_clearance_cells = std::max(
    0, static_cast<int>(std::ceil(
      options_.body_clearance_height_m / options_.nav_resolution_m)));
  if (body_clearance_cells == 0) {
    return;
  }

  for (auto & entry : geometry) {
    const GridIndex cell = entry.first;
    for (int dz = 1; dz <= body_clearance_cells; ++dz) {
      if (geometry.find({cell.x, cell.y, cell.z + dz}) != geometry.end()) {
        entry.second.body_obstructed = true;
        break;
      }
    }
  }
}

std::unordered_map<GridIndex, SurfaceCell, GridIndexHash>
ExperienceGeometryIndex::buildSupportCandidates() const
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> candidates;
  const double max_local_height_spread = std::max(0.12, options_.nav_resolution_m * 1.5);

  for (const auto & entry : raw_geometry_) {
    const GridIndex & cell = entry.first;
    const SurfaceCell & raw_cell = entry.second;
    if (raw_cell.body_obstructed) {
      continue;
    }

    int support_neighbors = 0;
    double min_height = raw_cell.height_m;
    double max_height = raw_cell.height_m;
    for (int dx = -kNeighborRadius; dx <= kNeighborRadius; ++dx) {
      for (int dy = -kNeighborRadius; dy <= kNeighborRadius; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          const auto neighbor_it = raw_geometry_.find({cell.x + dx, cell.y + dy, cell.z + dz});
          if (neighbor_it == raw_geometry_.end()) {
            continue;
          }
          if (std::abs(neighbor_it->second.height_m - raw_cell.height_m) >
            max_local_height_spread)
          {
            continue;
          }
          ++support_neighbors;
          min_height = std::min(min_height, neighbor_it->second.height_m);
          max_height = std::max(max_height, neighbor_it->second.height_m);
        }
      }
    }

    if (support_neighbors < kMinLocalSupportCells) {
      continue;
    }
    if ((max_height - min_height) > max_local_height_spread) {
      continue;
    }

    SurfaceCell candidate = raw_cell;
    candidate.label = SurfaceLabel::GeometrySupport;
    candidate.reachability = ReachabilityLabel::Unknown;
    candidate.confidence = std::max(candidate.confidence, 0.35);
    candidates[cell] = candidate;
  }

  return candidates;
}

}  // namespace tgw_planner::core
