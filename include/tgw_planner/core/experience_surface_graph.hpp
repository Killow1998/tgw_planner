#pragma once

#include <cstdint>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/surface_map.hpp"

namespace tgw_planner::core
{

class SurfaceTransitionValidator;

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
  int x{0};
  int y{0};
  double z{0.0};
  ReachabilityLabel reachability{ReachabilityLabel::Unknown};
  double clearance_m{0.0};
  double risk{0.0};
  double confidence{0.0};
  bool bridge{false};
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

class ExperienceSurfaceGraph
{
public:
  void build(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator);

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

private:
  Point3 cellCenter(const GridIndex & cell, double resolution_m) const;
  void computeComponents();

  std::vector<SurfaceNode> nodes_;
  std::vector<std::vector<SurfaceEdge>> adjacency_;
  std::unordered_map<GridIndex, std::vector<SurfaceNodeId>, GridIndexHash> xy_to_nodes_;
  std::unordered_map<GridIndex, SurfaceNodeId, GridIndexHash> cell_to_node_;
  std::vector<int> component_id_;
  std::vector<SurfaceGraphComponentInfo> components_;
  double resolution_m_{0.10};
};

}  // namespace tgw_planner::core
