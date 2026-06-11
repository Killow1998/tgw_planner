#include "tgw_planner/core/trajectory_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tgw_planner::core
{
namespace
{

GridIndex supportColumnKey(const Point3 & point, double raw_resolution_m)
{
  return {
    static_cast<int>(std::floor(point.x / raw_resolution_m)),
    static_cast<int>(std::floor(point.y / raw_resolution_m)),
    0};
}

std::vector<double> collectCandidateHeights(
  const SupportColumns & columns,
  const Point3 & trajectory_position,
  const TrajectoryProjectorOptions & options,
  int radius_cells)
{
  std::vector<double> candidates;
  const GridIndex center = supportColumnKey(trajectory_position, options.raw_resolution_m);
  for (int dx = -radius_cells; dx <= radius_cells; ++dx)
  {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      const GridIndex key{center.x + dx, center.y + dy, 0};
      const auto it = columns.find(key);
      if (it == columns.end()) {
        continue;
      }
      for (const double support_z : it->second) {
        const double below = trajectory_position.z - support_z;
        if (below >= options.search_below_min_m && below <= options.search_below_max_m) {
          candidates.push_back(support_z);
        }
      }
    }
  }
  return candidates;
}

double chooseSupportHeight(
  const std::vector<double> & candidates,
  const bool has_previous,
  const double previous_support_z)
{
  if (candidates.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return *std::min_element(
    candidates.begin(), candidates.end(),
    [&](double lhs, double rhs) {
      if (has_previous) {
        return std::abs(lhs - previous_support_z) < std::abs(rhs - previous_support_z);
      }
      return lhs > rhs;
    });
}

bool hasCandidateNearPreviousSupport(
  const std::vector<double> & candidates,
  double previous_support_z,
  double max_support_jump_m)
{
  return std::any_of(
    candidates.begin(), candidates.end(),
    [&](double support_z) {
      return std::abs(support_z - previous_support_z) <= max_support_jump_m;
    });
}

double yawFromPose(const Pose3 & pose)
{
  return std::atan2(pose.rotation[3], pose.rotation[0]);
}

double yawBetween(const Point3 & from, const Point3 & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

bool hasSupportNear(
  const SupportColumns & columns,
  const Point3 & point,
  double target_z,
  const TrajectoryProjectorOptions & options)
{
  const GridIndex center = supportColumnKey(point, options.raw_resolution_m);
  const int radius_cells = std::max(
    options.support_xy_search_radius_cells, options.support_xy_retry_radius_cells);
  for (int dx = -radius_cells; dx <= radius_cells; ++dx)
  {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      const auto it = columns.find({center.x + dx, center.y + dy, 0});
      if (it == columns.end()) {
        continue;
      }
      for (const double support_z : it->second) {
        if (std::abs(support_z - target_z) <= options.footprint_support_height_tolerance_m) {
          return true;
        }
      }
    }
  }
  return false;
}

std::unordered_set<GridIndex, GridIndexHash> sweepFootprintCells(
  const SupportColumns & columns,
  const ProjectedSupportSample & sample,
  double yaw_rad,
  const TrajectoryProjectorOptions & options)
{
  const double length = std::max(0.10, options.footprint_length_m);
  const double width = std::max(0.10, options.footprint_width_m);
  const double front = std::clamp(options.footprint_base_to_front_m, 0.01, length - 0.01);
  const double rear = length - front;
  const double half_width = 0.5 * width;
  const double step = std::max(0.05, 0.5 * options.resolution_m);
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);

  std::size_t total_samples = 0U;
  std::size_t supported_samples = 0U;
  std::unordered_set<GridIndex, GridIndexHash> cells;
  for (double x = -rear; x <= front + 1.0e-9; x += step) {
    for (double y = -half_width; y <= half_width + 1.0e-9; y += step) {
      ++total_samples;
      Point3 world{
        sample.support_position.x + x * cos_yaw - y * sin_yaw,
        sample.support_position.y + x * sin_yaw + y * cos_yaw,
        sample.support_position.z};
      if (!hasSupportNear(columns, world, sample.support_position.z, options)) {
        continue;
      }
      ++supported_samples;
      cells.insert({
        static_cast<int>(std::floor(world.x / options.resolution_m)),
        static_cast<int>(std::floor(world.y / options.resolution_m)),
        static_cast<int>(std::floor(world.z / options.resolution_m))});
    }
  }

  const double support_ratio = total_samples == 0U ? 0.0 :
    static_cast<double>(supported_samples) / static_cast<double>(total_samples);
  if (support_ratio < options.min_footprint_support_ratio) {
    cells.clear();
  }
  return cells;
}

std::unordered_set<GridIndex, GridIndexHash> sweepBridgeFootprintCells(
  const ProjectedSupportSample & sample,
  double yaw_rad,
  const TrajectoryProjectorOptions & options)
{
  const double length = std::max(0.10, options.footprint_length_m);
  const double width = std::max(0.10, options.footprint_width_m);
  const double front = std::clamp(options.footprint_base_to_front_m, 0.01, length - 0.01);
  const double rear = length - front;
  const double half_width = 0.5 * width;
  const double step = std::max(0.05, 0.5 * options.resolution_m);
  const double cos_yaw = std::cos(yaw_rad);
  const double sin_yaw = std::sin(yaw_rad);

  std::unordered_set<GridIndex, GridIndexHash> cells;
  for (double x = -rear; x <= front + 1.0e-9; x += step) {
    for (double y = -half_width; y <= half_width + 1.0e-9; y += step) {
      const Point3 world{
        sample.support_position.x + x * cos_yaw - y * sin_yaw,
        sample.support_position.y + x * sin_yaw + y * cos_yaw,
        sample.support_position.z};
      cells.insert({
        static_cast<int>(std::floor(world.x / options.resolution_m)),
        static_cast<int>(std::floor(world.y / options.resolution_m)),
        static_cast<int>(std::floor(world.z / options.resolution_m))});
    }
  }
  return cells;
}

void addTrajectoryBridgeSeeds(
  TrajectoryProjectionResult & result,
  const TrajectoryProjectorOptions & options)
{
  if (result.accepted_projected_support_samples.size() < 2U) {
    return;
  }
  const double step_m = std::max(0.05, options.trajectory_bridge_sample_step_m);
  for (std::size_t i = 1U; i < result.accepted_projected_support_samples.size(); ++i) {
    const ProjectedSupportSample & previous = result.accepted_projected_support_samples[i - 1U];
    const ProjectedSupportSample & current = result.accepted_projected_support_samples[i];
    const int bridge_id = static_cast<int>(i - 1U);
    const double gap_m = distance3d(previous.support_position, current.support_position);
    const double height_delta_m = std::abs(
      current.support_position.z - previous.support_position.z);
    if (gap_m <= options.resolution_m * 1.5 ||
      gap_m > options.max_trajectory_bridge_gap_m ||
      height_delta_m > options.max_trajectory_bridge_height_delta_m)
    {
      continue;
    }

    const int samples = std::max(1, static_cast<int>(std::ceil(gap_m / step_m)));
    const double yaw = yawBetween(previous.support_position, current.support_position);
    TrajectoryBridgeSegment segment;
    segment.bridge_id = bridge_id;
    segment.entry_support_cell = previous.support_cell;
    segment.exit_support_cell = current.support_cell;
    segment.gap_length_m = gap_m;
    segment.height_delta_m = height_delta_m;
    for (int sample_index = 1; sample_index < samples; ++sample_index) {
      const double t = static_cast<double>(sample_index) / static_cast<double>(samples);
      ProjectedSupportSample bridge;
      bridge.seq = current.seq;
      bridge.timestamp = previous.timestamp + t * (current.timestamp - previous.timestamp);
      bridge.trajectory_position = {
        previous.trajectory_position.x + t *
        (current.trajectory_position.x - previous.trajectory_position.x),
        previous.trajectory_position.y + t *
        (current.trajectory_position.y - previous.trajectory_position.y),
        previous.trajectory_position.z + t *
        (current.trajectory_position.z - previous.trajectory_position.z)};
      bridge.support_position = {
        previous.support_position.x + t *
        (current.support_position.x - previous.support_position.x),
        previous.support_position.y + t *
        (current.support_position.y - previous.support_position.y),
        previous.support_position.z + t *
        (current.support_position.z - previous.support_position.z)};
      const auto footprint_cells = sweepBridgeFootprintCells(bridge, yaw, options);
      const bool bridge_endpoint = sample_index == 1 || sample_index == samples - 1;
      for (const GridIndex & cell : footprint_cells) {
        if (result.observed_seed_cells.find(cell) != result.observed_seed_cells.end()) {
          continue;
        }
        if (result.bridge_seed_cells.insert(cell).second) {
          ++result.trajectory_bridge_seed_count;
        }
        result.bridged_seed_cells.insert(cell);
        segment.footprint_cells_ordered.push_back(cell);
        const auto order_it = segment.cell_order.find(cell);
        if (order_it == segment.cell_order.end() || sample_index < order_it->second) {
          segment.cell_order[cell] = sample_index;
        }
        BridgeCellMetadata & metadata = result.bridge_cell_metadata[cell];
        if (metadata.bridge_id < 0 || bridge_endpoint) {
          metadata.bridge_id = bridge_id;
          metadata.bridge_order = sample_index;
          metadata.bridge_endpoint = bridge_endpoint;
          metadata.height_m = bridge.support_position.z;
          metadata.confidence = 0.30;
        }
      }
    }
    if (!segment.footprint_cells_ordered.empty()) {
      result.bridge_segments.push_back(std::move(segment));
    }
  }
}

}  // namespace

TrajectoryProjector::TrajectoryProjector(TrajectoryProjectorOptions options)
: options_(options)
{
  if (options_.resolution_m <= 0.0) {
    options_.resolution_m = 0.10;
  }
  if (options_.raw_resolution_m <= 0.0) {
    options_.raw_resolution_m = 0.05;
  }
  if (options_.search_below_min_m < 0.0) {
    options_.search_below_min_m = 0.0;
  }
  if (options_.search_below_max_m < options_.search_below_min_m) {
    options_.search_below_max_m = options_.search_below_min_m;
  }
  if (options_.max_support_jump_m < 0.0) {
    options_.max_support_jump_m = 0.0;
  }
  if (options_.support_xy_search_radius_cells < 0) {
    options_.support_xy_search_radius_cells = 0;
  }
  if (options_.support_xy_retry_radius_cells < options_.support_xy_search_radius_cells) {
    options_.support_xy_retry_radius_cells = options_.support_xy_search_radius_cells;
  }
  options_.footprint_length_m = std::max(0.10, options_.footprint_length_m);
  options_.footprint_width_m = std::max(0.10, options_.footprint_width_m);
  options_.footprint_base_to_front_m =
    std::clamp(options_.footprint_base_to_front_m, 0.01, options_.footprint_length_m - 0.01);
  options_.min_footprint_support_ratio =
    std::clamp(options_.min_footprint_support_ratio, 0.0, 1.0);
  if (options_.footprint_support_height_tolerance_m < 0.0) {
    options_.footprint_support_height_tolerance_m = 0.0;
  }
  options_.max_trajectory_bridge_gap_m = std::max(0.0, options_.max_trajectory_bridge_gap_m);
  options_.max_trajectory_bridge_height_delta_m =
    std::max(0.0, options_.max_trajectory_bridge_height_delta_m);
  options_.trajectory_bridge_sample_step_m =
    std::max(0.05, options_.trajectory_bridge_sample_step_m);
}

TrajectoryProjectionResult TrajectoryProjector::project(const N3NavResource & resource) const
{
  ExperienceGeometryIndex geometry;
  ExperienceGeometryIndexOptions geometry_options;
  geometry_options.raw_resolution_m = options_.raw_resolution_m;
  geometry_options.nav_resolution_m = options_.resolution_m;
  geometry_options.body_clearance_height_m = 0.0;
  geometry_options.max_debug_world_points = 0U;
  geometry.build(resource, geometry_options);
  return project(resource, geometry);
}

TrajectoryProjectionResult TrajectoryProjector::project(
  const N3NavResource & resource,
  const ExperienceGeometryIndex & geometry) const
{
  TrajectoryProjectionResult result;
  const SupportColumns & columns = geometry.supportColumns();
  bool has_previous_support = false;
  double previous_support_z = 0.0;

  for (const auto & pose : resource.dense_trajectory) {
    Point3 trajectory_position = pose.pose_world_lidar.translation;
    trajectory_position.x += options_.lidar_to_footprint_x_m;
    trajectory_position.y += options_.lidar_to_footprint_y_m;

    std::vector<double> candidates = collectCandidateHeights(
      columns, trajectory_position, options_, options_.support_xy_search_radius_cells);
    if (candidates.empty() &&
      options_.support_xy_retry_radius_cells > options_.support_xy_search_radius_cells)
    {
      candidates = collectCandidateHeights(
        columns, trajectory_position, options_, options_.support_xy_retry_radius_cells);
      if (!candidates.empty()) {
        ++result.retry_support_samples;
      }
    }
    if (candidates.empty()) {
      result.rejected_samples.push_back(
        {pose.seq, pose.timestamp, trajectory_position, "support_projection_failed"});
      continue;
    }

    double support_z = chooseSupportHeight(
      candidates, has_previous_support, previous_support_z);
    if (!std::isfinite(support_z)) {
      result.rejected_samples.push_back(
        {pose.seq, pose.timestamp, trajectory_position, "support_projection_failed"});
      continue;
    }
    if (has_previous_support &&
      std::abs(support_z - previous_support_z) > options_.max_support_jump_m)
    {
      if (options_.allow_support_reanchor_on_jump &&
        !hasCandidateNearPreviousSupport(
          candidates, previous_support_z, options_.max_support_jump_m))
      {
        support_z = chooseSupportHeight(candidates, false, previous_support_z);
        ++result.reanchored_support_samples;
      } else {
        result.rejected_samples.push_back(
          {pose.seq, pose.timestamp, trajectory_position,
            "support_projection_ambiguous_multifloor"});
        continue;
      }
    }

    const Point3 support_position{trajectory_position.x, trajectory_position.y, support_z};
    const GridIndex support_cell = worldToGrid(support_position);
    ProjectedSupportSample sample{
      pose.seq, pose.timestamp, trajectory_position, support_position, support_cell};
    const auto footprint_cells = sweepFootprintCells(
      columns, sample, yawFromPose(pose.pose_world_lidar), options_);
    if (footprint_cells.empty()) {
      ++result.footprint_rejected_samples;
    } else {
      result.observed_seed_cells.insert(footprint_cells.begin(), footprint_cells.end());
      result.proven_seed_cells.insert(footprint_cells.begin(), footprint_cells.end());
      result.accepted_projected_support_samples.push_back(sample);
    }
    result.projected_support_samples.push_back(sample);
    previous_support_z = support_z;
    has_previous_support = true;
  }
  addTrajectoryBridgeSeeds(result, options_);
  return result;
}

GridIndex TrajectoryProjector::worldToGrid(const Point3 & point) const
{
  return {
    static_cast<int>(std::floor(point.x / options_.resolution_m)),
    static_cast<int>(std::floor(point.y / options_.resolution_m)),
    static_cast<int>(std::floor(point.z / options_.resolution_m))};
}

}  // namespace tgw_planner::core
