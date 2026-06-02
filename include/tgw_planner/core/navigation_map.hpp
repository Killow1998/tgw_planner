#pragma once

#include <cstdint>
#include <array>
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

struct Vec2
{
  double x{0.0};
  double y{0.0};

  double norm() const;
  Vec2 normalized() const;
  double dot(const Vec2 & other) const;
  Vec2 perpendicular() const;
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
  Landing,
  Stair,
  CeilingRejected,
  NoiseRejected
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

enum class StairFlightRejectReason
{
  None = 0,
  TooFewCells,
  NoAxis,
  TooShortOrLow,
  NegativeSlope,
  SlopeOutOfRange,
  ResidualTooHigh,
  NonMonotonic,
  TooNarrow,
  MissingPortals,
  SameFloorBothEnds,
  Count
};

enum class StairFragmentRescueReason
{
  None = 0,
  Strict,
  ConnectsFloor,
  BridgesFragments,
  ExtendsFlight,
  BetweenFloors,
  FillsGap
};

struct LooseStairFragment
{
  int id{-1};
  std::vector<GridIndex> cells;
  Vec2 uphill_axis;
  Vec2 side_axis;
  double s_min{0.0};
  double s_max{0.0};
  double t_min{0.0};
  double t_max{0.0};
  double z_min{0.0};
  double z_max{0.0};
  double length_m{0.0};
  double height_m{0.0};
  double width_m{0.0};
  double slope{0.0};
  double residual{0.0};
  int low_component_id{-1};
  int high_component_id{-1};
  Point3 low_endpoint;
  Point3 high_endpoint;
  bool prefilter_width_ok{false};
  bool strict_fit_ok{false};
  bool rescued{false};
  bool curved_like{false};
  StairFlightRejectReason reject_reason{StairFlightRejectReason::None};
  StairFragmentRescueReason rescue_reason{StairFragmentRescueReason::None};
};

struct FragmentBridge
{
  int from_fragment_id{-1};
  int to_fragment_id{-1};
  Point3 from;
  Point3 to;
};

struct StairFlightDiagnostics
{
  std::size_t raw_segments{0U};
  std::size_t segment_width_rejected{0U};
  std::size_t fit_rejected{0U};
  std::size_t accepted_candidates{0U};
  std::size_t merged_candidates{0U};
  std::size_t loose_fragment_count{0U};
  std::size_t strict_fragment_count{0U};
  std::size_t rescued_fragment_count{0U};
  std::size_t recovered_stair_cell_count{0U};
  std::size_t final_stair_flight_count{0U};
  std::array<std::size_t, static_cast<std::size_t>(StairFlightRejectReason::Count)> fit_reject_counts{};
};

struct FloorComponent
{
  int id{-1};
  std::vector<GridIndex> cells;
  double mean_z{0.0};
  double min_z{0.0};
  double max_z{0.0};
  Point3 centroid;
  GridIndex min_idx;
  GridIndex max_idx;
  bool is_landing{false};
};

enum class StairFlightType
{
  Straight,
  Curved,
  Spiral
};

struct StairFlight
{
  int id{-1};
  StairFlightType type{StairFlightType::Straight};
  std::vector<GridIndex> cells;
  Vec2 uphill_axis;
  Vec2 side_axis;
  std::vector<Vec2> local_tangents;
  std::vector<Vec2> local_normals;
  double z_min{0.0};
  double z_max{0.0};
  double length_m{0.0};
  double width_m{0.0};
  double slope{0.0};
  double score{0.0};
  int low_component_id{-1};
  int high_component_id{-1};
  Point3 low_endpoint;
  Point3 high_endpoint;
  std::vector<Point3> centerline;
  double safe_half_width_m{0.0};
};

struct StairCellInfo
{
  int stair_flight_id{-1};
  double along_s{0.0};
  double lateral_t{0.0};
  double lateral_error_m{0.0};
};

class NavigationMap
{
public:
  NavigationMap() = default;

  bool loadFromPcd(
    const std::string & pcd_file, double requested_resolution_m, double robot_radius_m,
    double robot_height_m, const std::string & map_frame, const std::string & map_id,
    BuildStats & stats, double robot_length_m = 0.70, double robot_width_m = 0.43,
    double base_to_front_m = 0.20);
  bool saveToMapPackage(
    const std::string & map_dir, const std::string & source_pcd, std::string & message) const;

  GridIndex worldToGrid(const Point3 & point) const;
  Point3 gridToWorld(const GridIndex & idx) const;

  bool isInsideBounds(const GridIndex & idx) const;
  bool isOccupied(const GridIndex & idx) const;
  bool isBlocked(const GridIndex & idx) const;
  bool hasGroundSupport(const GridIndex & idx) const;
  bool isCollisionFreeForRobot(const GridIndex & idx) const;
  bool isFootprintCollisionFreeAt(
    const GridIndex & idx, double heading_x, double heading_y) const;
  bool isFootprintTransitionSafe(const GridIndex & from, const GridIndex & to) const;
  bool isFootprintSupportedAt(
    const GridIndex & idx, double heading_x, double heading_y) const;
  bool isFootprintTransitionSupported(const GridIndex & from, const GridIndex & to) const;
  bool isTraversable(const GridIndex & idx) const;
  bool isStairTraversable(const GridIndex & idx) const;
  bool hasContinuousSupport(const GridIndex & idx) const;
  bool isStairTransitionAllowed(const GridIndex & from, const GridIndex & to) const;
  double getStairCenterCost(const GridIndex & idx) const;
  std::vector<std::vector<Point3>> stairCenterlines() const;
  int stairFlightId(const GridIndex & cell) const;
  int floorComponentId(const GridIndex & cell) const;
  bool isStairCell(const GridIndex & cell) const;
  bool isFloorOrLandingCell(const GridIndex & cell) const;
  bool isInsideStairSafeCorridor(const GridIndex & cell, int stair_flight_id) const;
  double lateralDistanceToStairCenterline(const GridIndex & cell, int stair_flight_id) const;
  double distanceToFlightCenterline(const GridIndex & cell, int stair_flight_id) const;
  bool isNearStairPortal(const GridIndex & cell, int stair_flight_id) const;
  bool isNearLowPortal(const GridIndex & cell, int stair_flight_id) const;
  bool isNearHighPortal(const GridIndex & cell, int stair_flight_id) const;
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
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedStairNoiseCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & looseStairFragmentCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedShortLowCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rejectedWidthPrefilterCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & rescuedStairFragmentCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & missingStairRecoveryCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & blockedCells() const;
  const std::unordered_map<GridIndex, double, GridIndexHash> & riskCosts() const;
  const std::vector<FloorComponent> & floorComponents() const;
  const std::vector<FloorComponent> & landingComponents() const;
  const std::vector<StairFlight> & stairFlights() const;
  const std::vector<LooseStairFragment> & looseStairFragments() const;
  const std::vector<FragmentBridge> & fragmentBridges() const;
  const StairFlightDiagnostics & stairFlightDiagnostics() const;

  double resolution() const;
  double robotRadius() const;
  double robotHeight() const;
  double robotLength() const;
  double robotWidth() const;
  double baseToFront() const;
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
  int supportTopZNearColumn(int x, int y, int stand_z, int max_dz) const;
  void rebuildStairSegments();
  bool stairSlope(const GridIndex & idx, double & slope_x, double & slope_y) const;
  bool stairAxis(const GridIndex & idx, int & axis_x, int & axis_y) const;
  int stairSegmentId(const GridIndex & idx) const;
  bool stairCellsSlopeCompatible(const GridIndex & from, const GridIndex & to) const;
  bool isStairFlightEdgeAllowed(const GridIndex & from, const GridIndex & to) const;
  bool isTreadBridgeAllowed(const GridIndex & from, const GridIndex & to) const;
  bool hasUpDownStairEvidenceAround(const GridIndex & cell) const;
  bool hasLargeLandingPlateauAround(const GridIndex & cell) const;
  bool isStairSegmentBridgeAllowed(const GridIndex & from, const GridIndex & to) const;
  bool isStairSameHeightTransferAllowed(const GridIndex & from, const GridIndex & to) const;
  bool stairSideDirection(const GridIndex & idx, int & side_dx, int & side_dy) const;
  bool isStairCenterCell(const GridIndex & idx, int min_side_cells) const;
  bool isStairEndpointCell(const GridIndex & idx) const;
  bool hasNearbyAcceptedFloor(const GridIndex & idx) const;
  int stairSideRunLength(const GridIndex & idx, int side_dx, int side_dy) const;
  bool isStairFlightWideEnough(const StairSegmentInfo & segment) const;
  bool hasTraversableSupportNearColumn(int x, int y, int z, int max_dz) const;
  bool isFootprintSupportedAtPoint(
    const Point3 & origin, int stand_z, double heading_x, double heading_y) const;
  void recoverMissingStairCells();
  void rebuildFloorComponents();
  void rebuildStairFlights();
  bool fitStairFragmentLoose(
    const StairSegmentInfo & segment, LooseStairFragment & fragment,
    StairFlightRejectReason * reject_reason = nullptr) const;
  bool fitStairFlightStrict(
    const LooseStairFragment & fragment, StairFlight & flight,
    StairFlightRejectReason * reject_reason = nullptr) const;
  bool areLooseFragmentsCompatible(
    const LooseStairFragment & a, const LooseStairFragment & b,
    FragmentBridge * bridge = nullptr) const;
  bool rescueLooseFragment(
    const LooseStairFragment & fragment, const std::vector<std::vector<int>> & graph,
    const std::vector<bool> & strict_ok, StairFragmentRescueReason & reason) const;
  StairSegmentInfo makeSegmentFromFragmentIds(const std::vector<int> & fragment_ids) const;
  bool fitStairFlightFromSegment(
    const StairSegmentInfo & segment, StairFlight & flight,
    StairFlightRejectReason * reject_reason = nullptr) const;
  bool isCurvedStairCandidate(const StairSegmentInfo & segment) const;
  bool fitCurvedStairFlightFromSegment(
    const StairSegmentInfo & segment, StairFlight & flight,
    StairFlightRejectReason * reject_reason = nullptr) const;
  int nearestFloorComponent(const Point3 & point, double max_distance_m) const;
  bool hasFloorBetween(const Point3 & a, const Point3 & b) const;
  bool isNearFlightEndpoint(
    const GridIndex & cell, const StairFlight & flight, const Point3 & endpoint) const;

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
  std::unordered_set<GridIndex, GridIndexHash> rejected_stair_noise_cells_;
  std::unordered_set<GridIndex, GridIndexHash> loose_stair_fragment_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rejected_short_low_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rejected_width_prefilter_cells_;
  std::unordered_set<GridIndex, GridIndexHash> rescued_stair_fragment_cells_;
  std::unordered_set<GridIndex, GridIndexHash> missing_stair_recovery_cells_;
  std::unordered_set<GridIndex, GridIndexHash> blocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> risk_cost_;
  std::unordered_map<XYIndex, ColumnInfo, XYIndexHash> columns_;
  std::unordered_map<GridIndex, StairSlope, GridIndexHash> stair_slopes_;
  std::unordered_map<GridIndex, int, GridIndexHash> stair_segment_by_cell_;
  std::vector<StairSegmentInfo> stair_segments_;
  std::vector<FloorComponent> floor_components_;
  std::vector<FloorComponent> landing_components_;
  std::vector<StairFlight> stair_flights_;
  std::vector<LooseStairFragment> loose_stair_fragments_;
  std::vector<FragmentBridge> fragment_bridges_;
  std::unordered_map<GridIndex, StairCellInfo, GridIndexHash> stair_cell_info_;
  StairFlightDiagnostics stair_flight_diagnostics_;

  double resolution_m_{0.20};
  double robot_radius_m_{0.35};
  double robot_height_m_{0.50};
  double robot_length_m_{0.70};
  double robot_width_m_{0.43};
  double base_to_front_m_{0.20};
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
