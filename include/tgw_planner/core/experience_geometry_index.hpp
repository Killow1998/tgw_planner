#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

using SupportColumns = std::unordered_map<GridIndex, std::vector<double>, GridIndexHash>;

struct ExperienceGeometryIndexOptions
{
  double raw_resolution_m{0.05};
  double nav_resolution_m{0.10};
  double body_clearance_height_m{0.65};
  double trajectory_roi_distance_m{0.0};
  std::size_t max_debug_world_points{400000U};
};

struct ExperienceGeometryIndexBuildResult
{
  bool success{false};
  std::string error_code;
  std::string message;
  std::size_t transformed_points{0};
  std::size_t raw_geometry_cell_count{0};
  std::size_t support_candidate_count{0};
  std::size_t support_column_count{0};
  std::size_t debug_world_point_count{0};
  std::size_t roi_skipped_points{0};
  double build_time_ms{0.0};
  double transform_insert_time_ms{0.0};
  double support_column_sort_time_ms{0.0};
  double raw_body_obstruction_time_ms{0.0};
  double support_candidate_time_ms{0.0};
  double support_body_obstruction_time_ms{0.0};
};

class ExperienceGeometryIndex
{
public:
  ExperienceGeometryIndex() = default;

  ExperienceGeometryIndexBuildResult build(
    const N3NavResource & resource,
    ExperienceGeometryIndexOptions options = {});

  bool empty() const;
  double rawResolution() const;
  double navResolution() const;
  const SupportColumns & supportColumns() const;
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & rawGeometry() const;
  const std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & supportCandidates() const;
  const std::vector<Point3> & debugWorldPoints() const;
  const ExperienceGeometryIndexBuildResult & metrics() const;

private:
  GridIndex rawColumnKey(const Point3 & point) const;
  GridIndex navCellKey(const Point3 & point) const;
  void markBodyObstructions(
    std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> & geometry) const;
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> buildSupportCandidates() const;

  ExperienceGeometryIndexOptions options_;
  SupportColumns support_columns_;
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> raw_geometry_;
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> support_candidates_;
  std::vector<Point3> debug_world_points_;
  ExperienceGeometryIndexBuildResult metrics_;
};

}  // namespace tgw_planner::core
