#pragma once

#include <unordered_map>
#include <unordered_set>

#include "tgw_planner/core/grid_index.hpp"

namespace tgw_planner::core
{

enum class SurfaceLabel
{
  Unknown,
  ReachableSeed,
  GeometrySupport,
  Expanded
};

enum class ReachabilityLabel
{
  Unknown,
  ProvenReachable,
  InferredReachable,
  LowConfidenceReachable,
  Forbidden
};

struct SurfaceCell
{
  GridIndex cell;
  GridIndex support;
  SurfaceLabel label{SurfaceLabel::Unknown};
  ReachabilityLabel reachability{ReachabilityLabel::Unknown};
  double height_m{0.0};
  double slope_m{0.0};
  double confidence{0.0};
  bool body_obstructed{false};
  bool hole_filled{false};
};

struct SurfaceMap
{
  std::unordered_map<GridIndex, SurfaceCell, GridIndexHash> surface_cells;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells;
  std::unordered_set<GridIndex, GridIndexHash> blocked_cells;
  std::unordered_set<GridIndex, GridIndexHash> forbidden_cells;
  std::unordered_set<GridIndex, GridIndexHash> boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> dropoff_boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> wall_boundary_cells;
  std::unordered_set<GridIndex, GridIndexHash> forbidden_boundary_cells;
};

}  // namespace tgw_planner::core
