#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/experience_geometry_index.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

struct TrajectoryProjectorOptions
{
  double resolution_m{0.10};
  double raw_resolution_m{0.05};
  double lidar_to_footprint_x_m{0.0};
  double lidar_to_footprint_y_m{0.0};
  double search_below_min_m{0.10};
  double search_below_max_m{1.00};
  double max_support_jump_m{0.30};
  bool allow_support_reanchor_on_jump{true};
  int support_xy_search_radius_cells{2};
  int support_xy_retry_radius_cells{8};
  double max_trajectory_bridge_gap_m{2.00};
  double max_trajectory_bridge_height_delta_m{0.80};
  double trajectory_bridge_sample_step_m{0.10};
  double footprint_length_m{0.70};
  double footprint_width_m{0.43};
  double footprint_base_to_front_m{0.20};
  double min_footprint_support_ratio{0.50};
  double footprint_support_height_tolerance_m{0.15};
};

struct ProjectedSupportSample
{
  std::uint64_t seq{0};
  double timestamp{0.0};
  Point3 trajectory_position;
  Point3 support_position;
  GridIndex support_cell;
};

struct RejectedProjectionSample
{
  std::uint64_t seq{0};
  double timestamp{0.0};
  Point3 trajectory_position;
  std::string reason;
};

struct TrajectoryProjectionResult
{
  std::unordered_set<GridIndex, GridIndexHash> observed_seed_cells;
  std::unordered_set<GridIndex, GridIndexHash> bridge_seed_cells;
  std::unordered_set<GridIndex, GridIndexHash> proven_seed_cells;
  std::unordered_set<GridIndex, GridIndexHash> bridged_seed_cells;
  std::unordered_map<GridIndex, BridgeCellMetadata, GridIndexHash> bridge_cell_metadata;
  std::vector<TrajectoryBridgeSegment> bridge_segments;
  std::vector<ProjectedSupportSample> accepted_projected_support_samples;
  std::vector<ProjectedSupportSample> projected_support_samples;
  std::vector<RejectedProjectionSample> rejected_samples;
  std::size_t footprint_rejected_samples{0};
  std::size_t reanchored_support_samples{0};
  std::size_t retry_support_samples{0};
  std::size_t trajectory_bridge_seed_count{0};
};

class TrajectoryProjector
{
public:
  explicit TrajectoryProjector(TrajectoryProjectorOptions options = {});

  TrajectoryProjectionResult project(const N3NavResource & resource) const;
  TrajectoryProjectionResult project(
    const N3NavResource & resource,
    const ExperienceGeometryIndex & geometry) const;

private:
  GridIndex worldToGrid(const Point3 & point) const;

  TrajectoryProjectorOptions options_;
};

}  // namespace tgw_planner::core
