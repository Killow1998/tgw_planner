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

  void rebuildTraversableLayer();
  void rebuildRiskLayer();

  std::uint32_t addBlockedRegion(const Point3 & min, const Point3 & max);
  std::uint32_t removeBlockedRegion(const Point3 & min, const Point3 & max);
  std::uint32_t clearBlockedRegions();

  double getRiskCost(const GridIndex & idx) const;
  MapCounts counts() const;

  const std::unordered_set<GridIndex, GridIndexHash> & occupiedCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & traversableCells() const;
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
  void updateGridBounds(const GridIndex & idx);
  void refreshMetricBounds();
  bool isInsideHorizontalRadius(int dx, int dy, int radius_cells) const;

  std::shared_ptr<octomap::OcTree> octree_;
  std::unordered_set<GridIndex, GridIndexHash> occupied_cells_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> blocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> risk_cost_;

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
