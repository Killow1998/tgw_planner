#include "tgw_planner/core/hybrid_experience_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace tgw_planner::core
{
namespace
{
constexpr std::uint32_t kInvalidHybridNode = std::numeric_limits<std::uint32_t>::max();

enum class HybridNodeKind
{
  Surface,
  Backbone
};

enum class HybridEdgeKind
{
  Surface,
  Backbone,
  Portal
};

struct HybridNode
{
  HybridNodeKind kind{HybridNodeKind::Surface};
  SurfaceNodeId surface_id{kInvalidHybridNode};
  BackboneNodeId backbone_id{kInvalidHybridNode};
  Point3 point;
};

struct HybridEdge
{
  std::uint32_t to{kInvalidHybridNode};
  HybridEdgeKind kind{HybridEdgeKind::Surface};
  double cost{0.0};
  double length_xy_m{0.0};
  double dz_m{0.0};
  ExperiencePortalId portal_id{kInvalidHybridNode};
};

struct HybridGraph
{
  std::vector<HybridNode> nodes;
  std::vector<std::vector<HybridEdge>> adjacency;
  std::uint32_t backbone_offset{0};
  std::uint32_t surface_edge_count{0};
  std::uint32_t backbone_edge_count{0};
  std::uint32_t portal_edge_count{0};
};

struct QueueNode
{
  std::uint32_t node{kInvalidHybridNode};
  double cost{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.cost > rhs.cost;
  }
};

double xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point3 surfacePoint(const ExperienceSurfaceGraph & graph, SurfaceNodeId node_id)
{
  const SurfaceNode * node = graph.node(node_id);
  if (node == nullptr) {
    return {};
  }
  return {
    (static_cast<double>(node->x) + 0.5) * graph.resolution(),
    (static_cast<double>(node->y) + 0.5) * graph.resolution(),
    node->z};
}

std::uint32_t saturatedSize(std::size_t value)
{
  return static_cast<std::uint32_t>(
    std::min<std::size_t>(value, std::numeric_limits<std::uint32_t>::max()));
}

bool validHybridId(const HybridGraph & graph, std::uint32_t id)
{
  return id < graph.nodes.size();
}

std::vector<std::uint32_t> reconstructHybridPath(
  const std::vector<std::uint32_t> & previous,
  std::uint32_t start,
  std::uint32_t goal)
{
  std::vector<std::uint32_t> path;
  if (start == kInvalidHybridNode || goal == kInvalidHybridNode ||
    goal >= previous.size())
  {
    return path;
  }
  std::uint32_t current = goal;
  path.push_back(current);
  while (current != start) {
    if (current >= previous.size() || previous[current] == kInvalidHybridNode) {
      path.clear();
      return path;
    }
    current = previous[current];
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

const HybridEdge * findEdge(
  const HybridGraph & graph, std::uint32_t from, std::uint32_t to)
{
  if (!validHybridId(graph, from)) {
    return nullptr;
  }
  for (const HybridEdge & edge : graph.adjacency[from]) {
    if (edge.to == to) {
      return &edge;
    }
  }
  return nullptr;
}

HybridGraph buildHybridGraph(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  const SurfacePlannerOptions & surface_options,
  const HybridExperiencePlannerOptions & hybrid_options)
{
  HybridGraph graph;
  graph.backbone_offset = saturatedSize(surface_graph.nodes().size());
  graph.nodes.reserve(surface_graph.nodes().size() + backbone_graph.nodes().size());

  for (const SurfaceNode & surface_node : surface_graph.nodes()) {
    HybridNode node;
    node.kind = HybridNodeKind::Surface;
    node.surface_id = surface_node.id;
    node.point = surfacePoint(surface_graph, surface_node.id);
    graph.nodes.push_back(node);
  }
  for (const BackboneNode & backbone_node : backbone_graph.nodes()) {
    HybridNode node;
    node.kind = HybridNodeKind::Backbone;
    node.backbone_id = backbone_node.id;
    node.point = backbone_node.path_position;
    graph.nodes.push_back(node);
  }

  graph.adjacency.resize(graph.nodes.size());
  for (const SurfaceNode & surface_node : surface_graph.nodes()) {
    for (const SurfaceEdge & surface_edge : surface_graph.adjacency()[surface_node.id.id]) {
      if (!surface_graph.isValid(surface_edge.to)) {
        continue;
      }
      HybridEdge edge;
      edge.to = surface_edge.to.id;
      edge.kind = HybridEdgeKind::Surface;
      edge.cost = surface_edge.cost > 0.0 ? surface_edge.cost : surface_edge.length_xy_m;
      edge.length_xy_m = surface_edge.length_xy_m;
      edge.dz_m = surface_edge.dz_m;
      graph.adjacency[surface_node.id.id].push_back(edge);
      ++graph.surface_edge_count;
    }
  }

  for (const BackboneEdge & backbone_edge : backbone_graph.edges()) {
    const std::uint32_t from = graph.backbone_offset + backbone_edge.from.id;
    const std::uint32_t to = graph.backbone_offset + backbone_edge.to.id;
    if (!validHybridId(graph, from) || !validHybridId(graph, to)) {
      continue;
    }
    const double low_confidence_penalty =
      (1.0 - std::clamp(backbone_edge.confidence, 0.0, 1.0)) *
      hybrid_options.backbone_low_confidence_penalty;
    const double edge_cost =
      backbone_edge.length_xy_m * hybrid_options.backbone_cost_scale +
      surface_options.w_slope * std::abs(backbone_edge.dz_m) +
      low_confidence_penalty;
    HybridEdge forward;
    forward.to = to;
    forward.kind = HybridEdgeKind::Backbone;
    forward.cost = edge_cost;
    forward.length_xy_m = backbone_edge.length_xy_m;
    forward.dz_m = backbone_edge.dz_m;
    HybridEdge reverse = forward;
    reverse.to = from;
    reverse.dz_m = -backbone_edge.dz_m;
    graph.adjacency[from].push_back(forward);
    graph.adjacency[to].push_back(reverse);
    graph.backbone_edge_count += 2U;
  }

  for (const ExperiencePortal & portal : backbone_graph.portals()) {
    if (!surface_graph.isValid(portal.surface_node) ||
      !backbone_graph.isValid(portal.backbone_node))
    {
      continue;
    }
    const std::uint32_t surface_id = portal.surface_node.id;
    const std::uint32_t backbone_id = graph.backbone_offset + portal.backbone_node.id;
    if (!validHybridId(graph, surface_id) || !validHybridId(graph, backbone_id)) {
      continue;
    }
    const double portal_cost =
      portal.distance_xy_m +
      hybrid_options.portal_height_error_weight * portal.height_error_m +
      hybrid_options.portal_switch_cost;
    HybridEdge surface_to_backbone;
    surface_to_backbone.to = backbone_id;
    surface_to_backbone.kind = HybridEdgeKind::Portal;
    surface_to_backbone.cost = portal_cost;
    surface_to_backbone.length_xy_m = portal.distance_xy_m;
    surface_to_backbone.dz_m =
      graph.nodes[backbone_id].point.z - graph.nodes[surface_id].point.z;
    surface_to_backbone.portal_id = portal.id;
    HybridEdge backbone_to_surface = surface_to_backbone;
    backbone_to_surface.to = surface_id;
    backbone_to_surface.dz_m = -surface_to_backbone.dz_m;
    graph.adjacency[surface_id].push_back(surface_to_backbone);
    graph.adjacency[backbone_id].push_back(backbone_to_surface);
    graph.portal_edge_count += 2U;
  }

  return graph;
}
}  // namespace

HybridExperiencePlanner::HybridExperiencePlanner(
  SurfacePlannerOptions surface_options,
  HybridExperiencePlannerOptions hybrid_options)
: surface_options_(surface_options), hybrid_options_(hybrid_options)
{
  hybrid_options_.backbone_cost_scale = std::max(0.1, hybrid_options_.backbone_cost_scale);
  hybrid_options_.portal_switch_cost = std::max(0.0, hybrid_options_.portal_switch_cost);
}

SurfacePlanResult HybridExperiencePlanner::plan(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  SurfaceNodeId start,
  SurfaceNodeId goal) const
{
  SurfacePlanResult result;
  if (!surface_graph.isValid(start)) {
    result.message = "start is not in the surface graph";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (!surface_graph.isValid(goal)) {
    result.message = "goal is not in the surface graph";
    result.metrics.failure_reason = result.message;
    return result;
  }

  if (surface_graph.sameComponent(start, goal)) {
    return SurfaceAstarPlanner(surface_options_).plan(surface_graph, start, goal);
  }
  if (backbone_graph.empty() || backbone_graph.portals().empty()) {
    result.message = "no_backbone_portal_for_start_or_goal";
    result.metrics.failure_reason = result.message;
    return result;
  }

  const HybridGraph graph = buildHybridGraph(
    surface_graph, backbone_graph, surface_options_, hybrid_options_);
  const std::uint32_t start_id = start.id;
  const std::uint32_t goal_id = goal.id;
  if (!validHybridId(graph, start_id) || !validHybridId(graph, goal_id)) {
    result.message = "start or goal is not in the hybrid graph";
    result.metrics.failure_reason = result.message;
    return result;
  }

  result.metrics.hybrid_nodes = saturatedSize(graph.nodes.size());
  result.metrics.hybrid_surface_edges = graph.surface_edge_count;
  result.metrics.hybrid_backbone_edges = graph.backbone_edge_count;
  result.metrics.hybrid_portal_edges = graph.portal_edge_count;
  result.metrics.start_portal_candidates =
    saturatedSize(backbone_graph.portalsForSurfaceComponent(surface_graph.componentId(start)).size());
  result.metrics.goal_portal_candidates =
    saturatedSize(backbone_graph.portalsForSurfaceComponent(surface_graph.componentId(goal)).size());
  for (const ExperiencePortalId portal_id :
    backbone_graph.portalsForSurfaceComponent(surface_graph.componentId(start)))
  {
    const ExperiencePortal * portal = backbone_graph.portal(portal_id);
    if (portal != nullptr) {
      result.debug_start_portal_candidates.push_back(surfacePoint(surface_graph, portal->surface_node));
    }
  }
  for (const ExperiencePortalId portal_id :
    backbone_graph.portalsForSurfaceComponent(surface_graph.componentId(goal)))
  {
    const ExperiencePortal * portal = backbone_graph.portal(portal_id);
    if (portal != nullptr) {
      result.debug_goal_portal_candidates.push_back(surfacePoint(surface_graph, portal->surface_node));
    }
  }

  std::vector<double> cost(graph.nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<std::uint32_t> previous(graph.nodes.size(), kInvalidHybridNode);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
  cost[start_id] = 0.0;
  open.push({start_id, 0.0});
  bool found = false;
  while (!open.empty() && result.metrics.hybrid_expanded_nodes < surface_options_.max_iterations) {
    const QueueNode current = open.top();
    open.pop();
    if (!validHybridId(graph, current.node) ||
      current.cost > cost[current.node] + 1.0e-9)
    {
      continue;
    }
    if (current.node == goal_id) {
      found = true;
      break;
    }
    ++result.metrics.hybrid_expanded_nodes;
    for (const HybridEdge & edge : graph.adjacency[current.node]) {
      if (!validHybridId(graph, edge.to)) {
        continue;
      }
      const double next_cost = current.cost + edge.cost;
      if (next_cost >= cost[edge.to]) {
        continue;
      }
      cost[edge.to] = next_cost;
      previous[edge.to] = current.node;
      open.push({edge.to, next_cost});
      ++result.metrics.generated_nodes;
    }
  }

  if (!found) {
    result.message = result.metrics.hybrid_expanded_nodes >= surface_options_.max_iterations ?
      "hybrid graph search reached max_iterations" : "hybrid graph search failed to find a path";
    result.metrics.failure_reason = result.message;
    return result;
  }

  const std::vector<std::uint32_t> hybrid_path =
    reconstructHybridPath(previous, start_id, goal_id);
  if (hybrid_path.empty()) {
    result.message = "hybrid graph parent chain is incomplete";
    result.metrics.failure_reason = result.message;
    return result;
  }

  result.success = true;
  result.message = "path found on hybrid experience graph";
  result.metrics.success = true;
  result.metrics.final_path_validated = true;
  result.metrics.selected_total_hybrid_cost = cost[goal_id];
  result.raw_path.reserve(hybrid_path.size());
  result.path.reserve(hybrid_path.size());
  for (const std::uint32_t node_id : hybrid_path) {
    const HybridNode & node = graph.nodes[node_id];
    appendPoint(result.path, node.point);
    appendPoint(result.raw_path, node.point);
    if (node.kind == HybridNodeKind::Surface) {
      const SurfaceNode * surface_node = surface_graph.node(node.surface_id);
      if (surface_node != nullptr) {
        result.cells.push_back(surface_node->cell);
        result.raw_cells.push_back(surface_node->cell);
      }
    }
  }

  for (std::size_t i = 1U; i < hybrid_path.size(); ++i) {
    const HybridEdge * edge = findEdge(graph, hybrid_path[i - 1U], hybrid_path[i]);
    if (edge == nullptr) {
      continue;
    }
    if (edge->kind == HybridEdgeKind::Surface) {
      ++result.metrics.used_surface_edges;
      result.metrics.surface_path_length_m += edge->length_xy_m;
    } else if (edge->kind == HybridEdgeKind::Backbone) {
      ++result.metrics.used_backbone_edges;
      result.metrics.backbone_path_length_m += edge->length_xy_m;
      appendPoint(result.debug_selected_backbone_segment, graph.nodes[hybrid_path[i - 1U]].point);
      appendPoint(result.debug_selected_backbone_segment, graph.nodes[hybrid_path[i]].point);
    } else {
      ++result.metrics.used_portal_edges;
      ++result.metrics.portal_switch_count;
      const Point3 portal_point = graph.nodes[hybrid_path[i - 1U]].kind == HybridNodeKind::Surface ?
        graph.nodes[hybrid_path[i - 1U]].point : graph.nodes[hybrid_path[i]].point;
      if (result.debug_selected_start_portal.empty()) {
        result.debug_selected_start_portal.push_back(portal_point);
        result.metrics.selected_start_portal_id = edge->portal_id.id;
        const HybridNode & backbone_node = graph.nodes[hybrid_path[i - 1U]].kind == HybridNodeKind::Backbone ?
          graph.nodes[hybrid_path[i - 1U]] : graph.nodes[hybrid_path[i]];
        result.metrics.selected_start_backbone_node = backbone_node.backbone_id.id;
      } else {
        result.debug_selected_goal_portal.clear();
        result.debug_selected_goal_portal.push_back(portal_point);
        result.metrics.selected_goal_portal_id = edge->portal_id.id;
        const HybridNode & backbone_node = graph.nodes[hybrid_path[i - 1U]].kind == HybridNodeKind::Backbone ?
          graph.nodes[hybrid_path[i - 1U]] : graph.nodes[hybrid_path[i]];
        result.metrics.selected_goal_backbone_node = backbone_node.backbone_id.id;
      }
    }
  }
  if (result.metrics.selected_start_backbone_node != kInvalidHybridNode &&
    result.metrics.selected_goal_backbone_node != kInvalidHybridNode)
  {
    result.metrics.selected_backbone_index_delta =
      result.metrics.selected_start_backbone_node > result.metrics.selected_goal_backbone_node ?
      result.metrics.selected_start_backbone_node - result.metrics.selected_goal_backbone_node :
      result.metrics.selected_goal_backbone_node - result.metrics.selected_start_backbone_node;
  }
  result.metrics.selected_backbone_length_m = result.metrics.backbone_path_length_m;
  result.metrics.selected_start_surface_leg_m = result.metrics.surface_path_length_m;
  result.metrics.selected_total_hybrid_cost = cost[goal_id];
  fillPathMetrics(result);
  return result;
}

Point3 HybridExperiencePlanner::surfacePoint(
  const ExperienceSurfaceGraph & graph, SurfaceNodeId node_id) const
{
  const SurfaceNode * node = graph.node(node_id);
  if (node == nullptr) {
    return {};
  }
  return {
    (static_cast<double>(node->x) + 0.5) * graph.resolution(),
    (static_cast<double>(node->y) + 0.5) * graph.resolution(),
    node->z};
}

void HybridExperiencePlanner::appendPoint(std::vector<Point3> & out, const Point3 & point) const
{
  if (!out.empty() && xyDistance(out.back(), point) < 1.0e-6 &&
    std::abs(out.back().z - point.z) < 1.0e-6)
  {
    return;
  }
  out.push_back(point);
}

double HybridExperiencePlanner::pathLength(const std::vector<Point3> & path) const
{
  double length = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length += xyDistance(path[i - 1U], path[i]);
  }
  return length;
}

void HybridExperiencePlanner::fillPathMetrics(SurfacePlanResult & result) const
{
  result.metrics.path_length_m = pathLength(result.path);
  result.metrics.raw_path_length_m = result.metrics.path_length_m;
  result.metrics.raw_path_waypoints = static_cast<std::uint32_t>(result.raw_path.size());
  result.metrics.min_path_clearance_m = 0.0;
  result.metrics.mean_path_clearance_m = 0.0;
  result.metrics.final_path_validated = true;
  result.metrics.success = result.success;
  for (std::size_t i = 1U; i < result.path.size(); ++i) {
    const double edge_dz = std::abs(result.path[i].z - result.path[i - 1U].z);
    result.metrics.max_path_edge_dz_m =
      std::max(result.metrics.max_path_edge_dz_m, edge_dz);
  }
}

}  // namespace tgw_planner::core
