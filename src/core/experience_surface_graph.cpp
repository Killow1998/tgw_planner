#include "tgw_planner/core/experience_surface_graph.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{
namespace
{
constexpr std::uint32_t kInvalidNodeId = std::numeric_limits<std::uint32_t>::max();
}

void ExperienceSurfaceGraph::build(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator)
{
  nodes_.clear();
  adjacency_.clear();
  xy_to_nodes_.clear();
  cell_to_node_.clear();
  component_id_.clear();
  components_.clear();
  resolution_m_ = snapshot.resolution_m;

  nodes_.reserve(snapshot.surface.traversable_cells.size());
  for (const GridIndex & cell : snapshot.surface.traversable_cells) {
    if (!validator.isEndpointCell(snapshot, cell)) {
      continue;
    }

    SurfaceNode node;
    node.id = {static_cast<std::uint32_t>(nodes_.size())};
    node.cell = cell;
    node.x = cell.x;
    node.y = cell.y;
    const auto surface_it = snapshot.surface.surface_cells.find(cell);
    if (surface_it != snapshot.surface.surface_cells.end()) {
      const SurfaceCell & surface_cell = surface_it->second;
      node.z = surface_cell.height_m;
      node.reachability = surface_cell.reachability;
      node.confidence = surface_cell.confidence;
      node.bridge = surface_cell.label == SurfaceLabel::TrajectoryBridge ||
        surface_cell.reachability == ReachabilityLabel::LowConfidenceReachable;
    } else {
      node.z = cellCenter(cell, snapshot.resolution_m).z;
    }
    const auto reachability_it = snapshot.reachability.find(cell);
    if (reachability_it != snapshot.reachability.end()) {
      node.reachability = reachability_it->second;
      node.bridge = node.bridge ||
        reachability_it->second == ReachabilityLabel::LowConfidenceReachable;
    }
    node.clearance_m = snapshot.clearance.clearanceDistance(cell);
    node.risk = snapshot.risk.riskCost(cell);

    nodes_.push_back(node);
    cell_to_node_[cell] = node.id;
    xy_to_nodes_[{cell.x, cell.y, 0}].push_back(node.id);
  }

  adjacency_.resize(nodes_.size());
  for (const SurfaceNode & node : nodes_) {
    std::vector<SurfaceEdge> & edges = adjacency_[node.id.id];
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex xy_key{node.x + dx, node.y + dy, 0};
        const auto candidates_it = xy_to_nodes_.find(xy_key);
        if (candidates_it == xy_to_nodes_.end()) {
          continue;
        }
        for (const SurfaceNodeId candidate_id : candidates_it->second) {
          if (candidate_id == node.id) {
            continue;
          }
          const SurfaceNode & candidate = nodes_[candidate_id.id];
          const TransitionReport report = validator.validate(
            snapshot, node.cell, candidate.cell);
          if (!report.allowed) {
            continue;
          }
          const double length_xy =
            std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
            snapshot.resolution_m;
          const double dz = candidate.z - node.z;
          SurfaceEdge edge;
          edge.from = node.id;
          edge.to = candidate_id;
          edge.length_xy_m = length_xy;
          edge.dz_m = dz;
          edge.slope = length_xy > 1.0e-9 ? std::abs(dz) / length_xy : 0.0;
          edge.kind = (node.bridge || candidate.bridge) ?
            SurfaceEdgeKind::TrajectoryBridge : SurfaceEdgeKind::NormalSurface;
          edge.cost = length_xy + 0.1 * std::abs(dz) +
            (edge.kind == SurfaceEdgeKind::TrajectoryBridge ? 1.0 : 0.0);
          edges.push_back(edge);
        }
      }
    }
  }

  computeComponents();
}

bool ExperienceSurfaceGraph::empty() const
{
  return nodes_.empty();
}

double ExperienceSurfaceGraph::resolution() const
{
  return resolution_m_;
}

const std::vector<SurfaceNode> & ExperienceSurfaceGraph::nodes() const
{
  return nodes_;
}

const std::vector<std::vector<SurfaceEdge>> & ExperienceSurfaceGraph::adjacency() const
{
  return adjacency_;
}

const std::unordered_map<GridIndex, std::vector<SurfaceNodeId>, GridIndexHash> &
ExperienceSurfaceGraph::xyToNodes() const
{
  return xy_to_nodes_;
}

const SurfaceNode * ExperienceSurfaceGraph::node(SurfaceNodeId id) const
{
  if (!isValid(id)) {
    return nullptr;
  }
  return &nodes_[id.id];
}

SurfaceNodeId ExperienceSurfaceGraph::nodeIdForCell(const GridIndex & cell) const
{
  const auto it = cell_to_node_.find(cell);
  if (it == cell_to_node_.end()) {
    return {kInvalidNodeId};
  }
  return it->second;
}

int ExperienceSurfaceGraph::componentId(SurfaceNodeId id) const
{
  if (!isValid(id) || id.id >= component_id_.size()) {
    return -1;
  }
  return component_id_[id.id];
}

bool ExperienceSurfaceGraph::sameComponent(SurfaceNodeId a, SurfaceNodeId b) const
{
  const int a_id = componentId(a);
  return a_id >= 0 && a_id == componentId(b);
}

bool ExperienceSurfaceGraph::isValid(SurfaceNodeId id) const
{
  return id.id < nodes_.size();
}

std::size_t ExperienceSurfaceGraph::componentCount() const
{
  return components_.size();
}

std::size_t ExperienceSurfaceGraph::largestComponentSize() const
{
  std::size_t out = 0U;
  for (const SurfaceGraphComponentInfo & component : components_) {
    out = std::max(out, component.size);
  }
  return out;
}

std::size_t ExperienceSurfaceGraph::multifloorComponentCount(double min_z_range_m) const
{
  std::size_t out = 0U;
  for (const SurfaceGraphComponentInfo & component : components_) {
    if (component.max_z - component.min_z >= min_z_range_m) {
      ++out;
    }
  }
  return out;
}

double ExperienceSurfaceGraph::componentMinZ(int id) const
{
  if (id < 0 || static_cast<std::size_t>(id) >= components_.size()) {
    return 0.0;
  }
  return components_[static_cast<std::size_t>(id)].min_z;
}

double ExperienceSurfaceGraph::componentMaxZ(int id) const
{
  if (id < 0 || static_cast<std::size_t>(id) >= components_.size()) {
    return 0.0;
  }
  return components_[static_cast<std::size_t>(id)].max_z;
}

const std::vector<SurfaceGraphComponentInfo> & ExperienceSurfaceGraph::components() const
{
  return components_;
}

Point3 ExperienceSurfaceGraph::cellCenter(const GridIndex & cell, double resolution_m) const
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

void ExperienceSurfaceGraph::computeComponents()
{
  component_id_.assign(nodes_.size(), -1);
  components_.clear();

  for (const SurfaceNode & seed : nodes_) {
    if (component_id_[seed.id.id] >= 0) {
      continue;
    }
    const int component_id = static_cast<int>(components_.size());
    SurfaceGraphComponentInfo info;
    info.id = component_id;

    std::queue<SurfaceNodeId> queue;
    queue.push(seed.id);
    component_id_[seed.id.id] = component_id;
    while (!queue.empty()) {
      const SurfaceNodeId current = queue.front();
      queue.pop();
      const SurfaceNode & node = nodes_[current.id];
      ++info.size;
      info.min_z = std::min(info.min_z, node.z);
      info.max_z = std::max(info.max_z, node.z);

      for (const SurfaceEdge & edge : adjacency_[current.id]) {
        if (!isValid(edge.to) || component_id_[edge.to.id] >= 0) {
          continue;
        }
        component_id_[edge.to.id] = component_id;
        queue.push(edge.to);
      }
    }
    components_.push_back(info);
  }

  for (SurfaceNode & node : nodes_) {
    node.surface_id = component_id_[node.id.id];
  }
}

}  // namespace tgw_planner::core
