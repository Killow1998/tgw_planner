#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

class SurfaceTransitionValidator;
struct SurfacePlannerOptions;

struct SurfaceNodeId
{
  std::uint32_t id{0};

  bool operator==(const SurfaceNodeId & other) const
  {
    return id == other.id;
  }

  bool operator!=(const SurfaceNodeId & other) const
  {
    return !(*this == other);
  }
};

enum class SurfaceEdgeKind
{
  NormalSurface,
  TrajectoryBridge
};

struct SurfaceNode
{
  SurfaceNodeId id;
  GridIndex cell;
  int surface_id{-1};
  int support_component_id{-1};
  int surface_layer_id{-1};
  int bridge_id{-1};
  int bridge_order{-1};
  int x{0};
  int y{0};
  double z{0.0};
  ReachabilityLabel reachability{ReachabilityLabel::Unknown};
  double clearance_m{0.0};
  double risk{0.0};
  double confidence{0.0};
  bool bridge{false};
  bool bridge_endpoint{false};
  std::uint8_t directional_footprint_mask{0U};
};

struct SurfaceEdge
{
  SurfaceNodeId from;
  SurfaceNodeId to;
  double length_xy_m{0.0};
  double dz_m{0.0};
  double slope{0.0};
  double cost{0.0};
  SurfaceEdgeKind kind{SurfaceEdgeKind::NormalSurface};
};

struct SurfaceGraphComponentInfo
{
  int id{-1};
  std::size_t size{0};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
};

struct SurfaceGraphBuildOptions
{
  double max_edge_height_delta_m{0.30};
  double max_bridge_edge_height_delta_m{0.80};
  double max_bridge_attach_height_delta_m{0.35};
  double max_edge_slope{3.0};
  double max_bridge_edge_slope{8.0};
  double w_clearance{0.8};
  double w_risk{1.5};
  double w_slope{0.3};
  double w_unknown{2.0};
  double w_bridge{2.5};
  double max_surface_safety_multiplier{5.0};
};

struct SurfaceGraphBuildMetrics
{
  std::size_t graph_edges{0};
  std::size_t graph_normal_edges{0};
  std::size_t graph_bridge_edges{0};
  std::size_t graph_rejected_cross_component_edges{0};
  std::size_t graph_rejected_large_dz_edges{0};
  std::size_t graph_rejected_large_slope_edges{0};
  std::size_t graph_rejected_invalid_bridge_edges{0};
  double max_graph_edge_dz_m{0.0};
  double max_graph_edge_slope{0.0};
};

class ExperienceSurfaceGraph
{
public:
  void build(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator,
    SurfaceGraphBuildOptions options = {});
  void build(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator,
    const SurfacePlannerOptions & planner_options,
    SurfaceGraphBuildOptions options = {});

  bool empty() const;
  double resolution() const;
  const std::vector<SurfaceNode> & nodes() const;
  const std::vector<std::vector<SurfaceEdge>> & adjacency() const;
  const std::unordered_map<GridIndex, std::vector<SurfaceNodeId>, GridIndexHash> &
  xyToNodes() const;

  const SurfaceNode * node(SurfaceNodeId id) const;
  SurfaceNodeId nodeIdForCell(const GridIndex & cell) const;
  int componentId(SurfaceNodeId id) const;
  bool sameComponent(SurfaceNodeId a, SurfaceNodeId b) const;
  bool isValid(SurfaceNodeId id) const;
  std::size_t componentCount() const;
  std::size_t largestComponentSize() const;
  std::size_t multifloorComponentCount(double min_z_range_m) const;
  double componentMinZ(int id) const;
  double componentMaxZ(int id) const;
  const std::vector<SurfaceGraphComponentInfo> & components() const;
  const SurfaceGraphBuildMetrics & metrics() const;

private:
  struct BridgeAttachment
  {
    int entry_surface_layer_id{-1};
    int exit_surface_layer_id{-1};
    int entry_order{-1};
    int exit_order{-1};
  };

  Point3 cellCenter(const GridIndex & cell, double resolution_m) const;
  void buildBridgeAttachments(const NavigationSnapshot & snapshot);
  int surfaceLayerForCell(const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  bool isBridgeCell(const NavigationSnapshot & snapshot, const GridIndex & cell) const;
  bool isSurfaceGraphNode(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator,
    const GridIndex & cell) const;
  std::optional<SurfaceEdge> makeContinuousSurfaceEdge(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator,
    const SurfaceGraphBuildOptions & options,
    const SurfaceNode & from,
    const SurfaceNode & to,
    int dx,
    int dy,
    SurfaceGraphBuildMetrics & metrics) const;
  bool isValidBridgeTransition(const SurfaceNode & from, const SurfaceNode & to) const;
  bool isValidNormalTransition(const SurfaceNode & from, const SurfaceNode & to) const;
  void computeComponents();

  std::vector<SurfaceNode> nodes_;
  std::vector<std::vector<SurfaceEdge>> adjacency_;
  std::unordered_map<GridIndex, std::vector<SurfaceNodeId>, GridIndexHash> xy_to_nodes_;
  std::unordered_map<GridIndex, SurfaceNodeId, GridIndexHash> cell_to_node_;
  std::unordered_map<int, BridgeAttachment> bridge_attachments_;
  std::vector<int> component_id_;
  std::vector<SurfaceGraphComponentInfo> components_;
  SurfaceGraphBuildMetrics metrics_;
  SurfaceGraphBuildOptions options_;
  double resolution_m_{0.10};
};

}  // namespace tgw_planner::core
