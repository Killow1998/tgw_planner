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

bool isBridgeLabel(const SurfaceCell & cell)
{
  return cell.label == SurfaceLabel::TrajectoryBridge;
}

int directionIndex(int dx, int dy)
{
  if (dx == 1 && dy == 0) {
    return 0;
  }
  if (dx == 1 && dy == 1) {
    return 1;
  }
  if (dx == 0 && dy == 1) {
    return 2;
  }
  if (dx == -1 && dy == 1) {
    return 3;
  }
  if (dx == -1 && dy == 0) {
    return 4;
  }
  if (dx == -1 && dy == -1) {
    return 5;
  }
  if (dx == 0 && dy == -1) {
    return 6;
  }
  if (dx == 1 && dy == -1) {
    return 7;
  }
  return -1;
}

bool hasDirectionalFootprintSupport(const SurfaceNode & node, int dx, int dy)
{
  const int index = directionIndex(dx, dy);
  return index >= 0 && (node.directional_footprint_mask & (1U << index)) != 0U;
}

bool hasTraversableCellAtXY(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  int x,
  int y,
  int min_z,
  int max_z)
{
  for (int z = min_z; z <= max_z; ++z) {
    if (validator.isCellTraversable(snapshot, {x, y, z})) {
      return true;
    }
  }
  return false;
}

bool hasDiagonalCornerSupport(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  const GridIndex & from,
  const GridIndex & to)
{
  if (std::abs(to.x - from.x) != 1 || std::abs(to.y - from.y) != 1) {
    return true;
  }
  const int min_z = std::min(from.z, to.z);
  const int max_z = std::max(from.z, to.z);
  return hasTraversableCellAtXY(snapshot, validator, to.x, from.y, min_z, max_z) &&
         hasTraversableCellAtXY(snapshot, validator, from.x, to.y, min_z, max_z);
}
}

void ExperienceSurfaceGraph::build(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  SurfaceGraphBuildOptions options)
{
  nodes_.clear();
  adjacency_.clear();
  xy_to_nodes_.clear();
  cell_to_node_.clear();
  bridge_attachments_.clear();
  component_id_.clear();
  components_.clear();
  metrics_ = {};
  options_ = options;
  resolution_m_ = snapshot.resolution_m;
  buildBridgeAttachments(snapshot);

  nodes_.reserve(snapshot.surface.traversable_cells.size());
  for (const GridIndex & cell : snapshot.surface.traversable_cells) {
    if (!isSurfaceGraphNode(snapshot, validator, cell)) {
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
      node.support_component_id = surface_cell.support_component_id;
      node.surface_layer_id = surface_cell.surface_layer_id;
      node.bridge_id = surface_cell.bridge_id;
      node.bridge_order = surface_cell.bridge_order;
      node.bridge_endpoint = surface_cell.bridge_endpoint;
      node.confidence = surface_cell.confidence;
      node.bridge = isBridgeLabel(surface_cell);
    } else {
      node.z = cellCenter(cell, snapshot.resolution_m).z;
    }
    const auto reachability_it = snapshot.reachability.find(cell);
    if (reachability_it != snapshot.reachability.end()) {
      node.reachability = reachability_it->second;
    }
    node.clearance_m = snapshot.clearance.clearanceDistance(cell);
    node.risk = snapshot.risk.riskCost(cell);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int index = directionIndex(dx, dy);
        if (index < 0) {
          continue;
        }
        const double yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
        if (validator.isCellCenterFootprintSupported(snapshot, cell, yaw)) {
          node.directional_footprint_mask |= static_cast<std::uint8_t>(1U << index);
        }
      }
    }

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
          const std::optional<SurfaceEdge> edge = makeContinuousSurfaceEdge(
            snapshot, validator, options, node, candidate, dx, dy);
          if (edge.has_value()) {
            edges.push_back(*edge);
          }
        }
      }
    }
  }

  computeComponents();
}

void ExperienceSurfaceGraph::build(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  const SurfacePlannerOptions & planner_options,
  SurfaceGraphBuildOptions options)
{
  options.w_clearance = planner_options.w_clearance;
  options.w_risk = planner_options.w_risk;
  options.w_slope = planner_options.w_slope;
  options.w_unknown = planner_options.w_unknown;
  options.w_bridge = planner_options.w_bridge;
  build(snapshot, validator, options);
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

const SurfaceGraphBuildMetrics & ExperienceSurfaceGraph::metrics() const
{
  return metrics_;
}

Point3 ExperienceSurfaceGraph::cellCenter(const GridIndex & cell, double resolution_m) const
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

void ExperienceSurfaceGraph::buildBridgeAttachments(const NavigationSnapshot & snapshot)
{
  bridge_attachments_.clear();
  for (const TrajectoryBridgeSegment & segment : snapshot.bridge_segments) {
    if (segment.bridge_id < 0) {
      continue;
    }
    BridgeAttachment attachment;
    attachment.entry_surface_layer_id =
      surfaceLayerForCell(snapshot, segment.entry_support_cell);
    attachment.exit_surface_layer_id =
      surfaceLayerForCell(snapshot, segment.exit_support_cell);
    for (const auto & entry : segment.cell_order) {
      if (attachment.entry_order < 0 || entry.second < attachment.entry_order) {
        attachment.entry_order = entry.second;
      }
      if (attachment.exit_order < 0 || entry.second > attachment.exit_order) {
        attachment.exit_order = entry.second;
      }
    }
    bridge_attachments_[segment.bridge_id] = attachment;
  }
}

int ExperienceSurfaceGraph::surfaceLayerForCell(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  const auto exact_it = snapshot.surface.surface_cells.find(cell);
  if (exact_it == snapshot.surface.surface_cells.end()) {
    return -1;
  }
  return exact_it->second.surface_layer_id;
}

bool ExperienceSurfaceGraph::isBridgeCell(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  const auto surface_it = snapshot.surface.surface_cells.find(cell);
  if (surface_it != snapshot.surface.surface_cells.end() && isBridgeLabel(surface_it->second)) {
    return true;
  }
  const auto reachability_it = snapshot.reachability.find(cell);
  (void)reachability_it;
  return false;
}

bool ExperienceSurfaceGraph::isSurfaceGraphNode(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  const GridIndex & cell) const
{
  if (!validator.isCellTraversable(snapshot, cell)) {
    return false;
  }
  if (isBridgeCell(snapshot, cell)) {
    return true;
  }
  return validator.isEndpointCell(snapshot, cell);
}

std::optional<SurfaceEdge> ExperienceSurfaceGraph::makeContinuousSurfaceEdge(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator,
  const SurfaceGraphBuildOptions & options,
  const SurfaceNode & from,
  const SurfaceNode & to,
  int dx,
  int dy)
{
  const double length_xy =
    std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * snapshot.resolution_m;
  if (length_xy <= 1.0e-9) {
    return std::nullopt;
  }

  const double dz = to.z - from.z;
  const double abs_dz = std::abs(dz);
  const bool bridge_edge = from.bridge || to.bridge;
  const bool bridge_attach_edge = bridge_edge && !(from.bridge && to.bridge);
  const double max_height_delta = bridge_edge ?
    (bridge_attach_edge ?
      options.max_bridge_attach_height_delta_m : options.max_bridge_edge_height_delta_m) :
    options.max_edge_height_delta_m;
  if (abs_dz > max_height_delta) {
    ++metrics_.graph_rejected_large_dz_edges;
    return std::nullopt;
  }

  const double slope = abs_dz / length_xy;
  const double max_slope = bridge_edge ? options.max_bridge_edge_slope : options.max_edge_slope;
  if (slope > max_slope) {
    ++metrics_.graph_rejected_large_slope_edges;
    return std::nullopt;
  }

  if (bridge_edge) {
    if (!isValidBridgeTransition(from, to)) {
      ++metrics_.graph_rejected_invalid_bridge_edges;
      return std::nullopt;
    }
  } else {
    if (!isValidNormalTransition(from, to)) {
      ++metrics_.graph_rejected_cross_component_edges;
      return std::nullopt;
    }
    const int max_step_cells = std::max(
      1,
      static_cast<int>(std::ceil(
        options.max_edge_height_delta_m / snapshot.resolution_m)));
    const bool direct_surface_neighbor =
      std::abs(to.cell.x - from.cell.x) <= 1 &&
      std::abs(to.cell.y - from.cell.y) <= 1 &&
      std::abs(to.cell.z - from.cell.z) <= max_step_cells;
    if (direct_surface_neighbor) {
      if (!hasDiagonalCornerSupport(snapshot, validator, from.cell, to.cell)) {
        return std::nullopt;
      }
      if (!hasDirectionalFootprintSupport(from, dx, dy) ||
        !hasDirectionalFootprintSupport(to, dx, dy))
      {
        return std::nullopt;
      }
    } else {
      const TransitionReport report = validator.validate(snapshot, from.cell, to.cell);
      if (!report.allowed) {
        return std::nullopt;
      }
    }
  }

  SurfaceEdge edge;
  edge.from = from.id;
  edge.to = to.id;
  edge.length_xy_m = length_xy;
  edge.dz_m = dz;
  edge.slope = slope;
  edge.kind = bridge_edge ? SurfaceEdgeKind::TrajectoryBridge : SurfaceEdgeKind::NormalSurface;
  const double clearance_penalty =
    options.w_clearance * snapshot.clearance.clearancePenalty(to.cell);
  const double risk_penalty = options.w_risk * to.risk;
  const double unknown_penalty =
    options.w_unknown *
    (!snapshot.observed_free_cells.empty() &&
    snapshot.observed_free_cells.find(to.cell) == snapshot.observed_free_cells.end() ? 1.0 : 0.0);
  const double slope_penalty = options.w_slope * abs_dz;
  const double bridge_penalty =
    options.w_bridge *
    (edge.kind == SurfaceEdgeKind::TrajectoryBridge ||
    to.reachability == ReachabilityLabel::LowConfidenceReachable ? 1.0 : 0.0);
  const double safety_multiplier = std::clamp(
    clearance_penalty + risk_penalty + unknown_penalty + slope_penalty,
    0.0,
    std::max(0.0, options.max_surface_safety_multiplier));
  edge.cost = length_xy * (1.0 + safety_multiplier) + bridge_penalty;
  ++metrics_.graph_edges;
  metrics_.max_graph_edge_dz_m = std::max(metrics_.max_graph_edge_dz_m, abs_dz);
  metrics_.max_graph_edge_slope = std::max(metrics_.max_graph_edge_slope, slope);
  if (edge.kind == SurfaceEdgeKind::TrajectoryBridge) {
    ++metrics_.graph_bridge_edges;
  } else {
    ++metrics_.graph_normal_edges;
  }
  return edge;
}

bool ExperienceSurfaceGraph::isValidBridgeTransition(
  const SurfaceNode & from, const SurfaceNode & to) const
{
  if (from.bridge && to.bridge) {
    return from.bridge_id >= 0 && from.bridge_id == to.bridge_id &&
           from.bridge_order >= 0 && to.bridge_order >= 0 &&
           std::abs(from.bridge_order - to.bridge_order) <= 1;
  }
  const SurfaceNode & bridge_node = from.bridge ? from : to;
  const SurfaceNode & normal_node = from.bridge ? to : from;
  if (!bridge_node.bridge_endpoint || normal_node.surface_layer_id < 0) {
    return false;
  }
  const auto attachment_it = bridge_attachments_.find(bridge_node.bridge_id);
  if (attachment_it == bridge_attachments_.end()) {
    return false;
  }
  const BridgeAttachment & attachment = attachment_it->second;
  const bool entry_match =
    attachment.entry_surface_layer_id >= 0 &&
    normal_node.surface_layer_id == attachment.entry_surface_layer_id &&
    bridge_node.bridge_order == attachment.entry_order;
  const bool exit_match =
    attachment.exit_surface_layer_id >= 0 &&
    normal_node.surface_layer_id == attachment.exit_surface_layer_id &&
    bridge_node.bridge_order == attachment.exit_order;
  return entry_match || exit_match;
}

bool ExperienceSurfaceGraph::isValidNormalTransition(
  const SurfaceNode & from, const SurfaceNode & to) const
{
  return from.surface_layer_id >= 0 &&
         from.surface_layer_id == to.surface_layer_id;
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
