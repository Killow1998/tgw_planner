#pragma once

#include <cstdint>
#include <vector>

#include "tgw_planner/core/mapping_options.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/probabilistic_voxel_map.hpp"

namespace tgw_planner::core
{

struct SelfFilterBox
{
  double min_x{-0.60};
  double max_x{0.60};
  double min_y{-0.40};
  double max_y{0.40};
  double min_z{-0.35};
  double max_z{0.80};
};

struct ScanInput
{
  std::vector<Point3> points_sensor_frame;
  Pose3 sensor_pose_map;
  double stamp_sec{0.0};
  int view_id{0};
};

struct RaycastStats
{
  std::uint64_t input_points{0};
  std::uint64_t inserted_points{0};
  std::uint64_t filtered_invalid{0};
  std::uint64_t filtered_range{0};
  std::uint64_t filtered_self{0};
  std::uint64_t miss_updates{0};
  std::uint64_t hit_updates{0};
  std::uint64_t dynamic_suspect_voxels_after_decay{0};
  std::uint64_t static_candidate_voxels_after_decay{0};
};

class RaycastIntegrator
{
public:
  RaycastIntegrator(MappingOptions options = {}, SelfFilterBox self_filter_box = {});

  RaycastStats insertScan(const ScanInput & scan, ProbabilisticVoxelMap & map) const;

private:
  bool isInvalid(const Point3 & point) const;
  bool isSelfPoint(const Point3 & point_sensor_frame) const;
  std::vector<GridIndex> rayVoxels(
    const GridIndex & origin, const GridIndex & endpoint) const;

  MappingOptions options_;
  SelfFilterBox self_filter_box_;
};

}  // namespace tgw_planner::core
