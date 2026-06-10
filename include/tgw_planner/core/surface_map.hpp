#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"

namespace tgw_planner::core
{

enum class SurfaceLabel
{
  Unknown,
  ReachableSeed,
  TrajectoryBridge,
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
  int support_component_id{-1};
  int surface_layer_id{-1};
  int bridge_id{-1};
  int bridge_order{-1};
  double height_m{0.0};
  double slope_m{0.0};
  double confidence{0.0};
  bool body_obstructed{false};
  bool hole_filled{false};
  bool bridge_endpoint{false};
};

struct BridgeCellMetadata
{
  int bridge_id{-1};
  int bridge_order{-1};
  bool bridge_endpoint{false};
  double height_m{0.0};
  double confidence{0.30};
};

struct TrajectoryBridgeSegment
{
  int bridge_id{-1};
  GridIndex entry_support_cell;
  GridIndex exit_support_cell;
  std::vector<GridIndex> footprint_cells_ordered;
  std::unordered_map<GridIndex, int, GridIndexHash> cell_order;
  double gap_length_m{0.0};
  double height_delta_m{0.0};
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
