#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "octomap/OcTree.h"

#include "tgw_planner/core/grid_index.hpp"

namespace tgw_planner::core
{

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct MapCounts
{
  std::uint32_t occupied_cells{0};
  std::uint32_t traversable_cells{0};
  std::uint32_t blocked_cells{0};
  std::uint32_t risk_cells{0};
};

struct BuildStats
{
  bool success{false};
  std::string message;
  std::string source_pcd;
  std::uint64_t source_points{0};
  double build_time_ms{0.0};
  Point3 bounds_min;
  Point3 bounds_max;
  MapCounts counts;
};

struct XYIndex
{
  int x{0};
  int y{0};

  bool operator==(const XYIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct XYIndexHash
{
  std::size_t operator()(const XYIndex & idx) const
  {
    std::size_t seed = std::hash<int>{}(idx.x);
    seed ^= std::hash<int>{}(idx.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct ZRun
{
  int z_min{0};
  int z_max{0};
};

struct ColumnInfo
{
  XYIndex xy;
  std::vector<ZRun> occupied_runs;
};

enum class SurfaceKind
{
  Unknown,
  Floor,
  Stair,
  CeilingLike,
  Noise
};

enum class SurfaceRejectReason
{
  None,
  Clearance,
  Collision,
  CeilingLike,
  Noise
};

struct SurfaceCandidate
{
  GridIndex stand;
  int support_z_min{0};
  int support_z_max{0};
  int overhead_distance_cells{0};
  bool overhead_known{false};
  bool overhead_clear{false};
  SurfaceKind kind{SurfaceKind::Unknown};
  SurfaceRejectReason reject_reason{SurfaceRejectReason::None};
};

struct StairSlope
{
  double x{0.0};
  double y{0.0};
  bool valid{false};
};

struct StairSegmentInfo
{
  int id{-1};
  std::vector<GridIndex> cells;
  int z_min{0};
  int z_max{0};
  bool spiral_like{false};
};

class NavigationMap
{
public:
  NavigationMap() = default;

  bool loadFromPcd(
    const std::string & pcd_file, double requested_resolution_m, double robot_radius_m,
    double robot_height_m, const std::string & map_frame, const std::string & map_id,
    BuildStats & stats);
  bool saveToMapPackage(
    const std::string & map_dir, const std::string & source_pcd, std::string & message) const;

  GridIndex worldToGrid(const Point3 & point) const;
  Point3 gridToWorld(const GridIndex & idx) const;

  bool isInsideBounds(const GridIndex & idx) const;
  bool isOccupied(const GridIndex & idx) const;
  bool isBlocked(const GridIndex & idx) const;
  bool hasGroundSupport(const GridIndex & idx) const;
  bool isCollisionFreeForRobot(const GridIndex & idx) const;
  bool isTraversable(const GridIndex & idx) const;
  bool isStairTraversable(const GridIndex & idx) const;
  bool hasContinuousSupport(const GridIndex & idx) const;
  bool isStairTransitionAllowed(const GridIndex & from, const GridIndex & to) const;
  double getStairCenterCost(const GridIndex & idx) const;
  int maxStepCells() const;

  void rebuildTraversableLayer();
  void rebuildRiskLayer();

  std::uint32_t addBlockedRegion(const Point3 & min, const Point3 & max);
  std::uint32_t removeBlockedRegion(const Point3 & min, const Point3 & max);
  std::uint32_t clearBlockedRegions();

  double getRiskCost(const GridIndex & idx) const;
  MapCounts counts() const;

  const std::unordered_set<GridIndex, GridIndexHash> & occupiedCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & traversableCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & forbiddenCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & surfaceCandidateCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & acceptedFloorCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & acceptedStairCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedCeilingCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedClearanceCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedCollisionCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & blockedCells() const;
  const std::unordered_map<GridIndex, double, GridIndexHash> & riskCosts() const;

  double resolution() const;
  double robotRadius() const;
  double robotHeight() const;
  double riskInflationRadius() const;
  const std::string & mapFrame() const;
  const std::string & mapId() const;
  const Point3 & boundsMin() const;
  const Point3 & boundsMax() const;
  bool ready() const;

private:
  void buildColumns();
  void updateGridBounds(const GridIndex & idx);
  void refreshMetricBounds();
  bool isInsideHorizontalRadius(int dx, int dy, int radius_cells) const;
  const ColumnInfo * findColumn(int x, int y) const;
  bool hasHeadClearanceInColumn(const GridIndex & idx, int height_cells) const;
  int overheadDistanceCells(const GridIndex & idx, int height_cells, bool & overhead_known) const;
  void rebuildStairSegments();
  bool stairSlope(const GridIndex & idx, double & slope_x, double & slope_y) const;
  bool stairAxis(const GridIndex & idx, int & axis_x, int & axis_y) const;
  int stairSegmentId(const GridIndex & idx) const;
  bool stairCellsSlopeCompatible(const GridIndex & from, const GridIndex & to) const;
  bool isStairSegmentBridgeAllowed(const GridIndex & from, const GridIndex & to) const;
  bool isStairSameHeightTransferAllowed(const GridIndex & from, const GridIndex & to) const;
  bool stairSideDirection(const GridIndex & idx, int & side_dx, int & side_dy) const;
  bool isStairCenterCell(const GridIndex & idx, int min_side_cells) const;
  bool isStairEndpointCell(const GridIndex & idx) const;
  bool hasNearbyAcceptedFloor(const GridIndex & idx) const;
  int stairSideRunLength(const GridIndex & idx, int side_dx, int side_dy) const;

  std::shared_ptr<octomap::OcTree> octree_;
  std::unordered_set<GridIndex, GridIndexHash> occupied_cells_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> forbidden_cells_;
  std::unordered_set<GridIndex, GridIndexHash> surface_candidate_cells_;
  std::unordered_set<GridIndex, GridIndexHash> accepted_floor_cells_;
  std::unordered_set<GridIndex, GridIndexHash> accepted_stair_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rejected_ceiling_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rejected_clearance_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rejected_collision_cells_;
  std::unordered_set<GridIndex, GridIndexHash> blocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> risk_cost_;
  std::unordered_map<XYIndex, ColumnInfo, XYIndexHash> columns_;
  std::unordered_map<GridIndex, StairSlope, GridIndexHash> stair_slopes_;
  std::unordered_map<GridIndex, int, GridIndexHash> stair_segment_by_cell_;
  std::vector<StairSegmentInfo> stair_segments_;

  double resolution_m_{0.20};
  double robot_radius_m_{0.35};
  double robot_height_m_{0.80};
  double risk_inflation_radius_m_{0.40};
  std::string map_frame_{"map"};
  std::string map_id_{"tgw_nav_map"};
  Point3 bounds_min_;
  Point3 bounds_max_;
  GridIndex min_idx_;
  GridIndex max_idx_;
  bool has_bounds_{false};
  bool ready_{false};
};

}  // namespace tgw_planner::core
