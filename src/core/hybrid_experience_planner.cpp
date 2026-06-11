#include "tgw_planner/core/hybrid_experience_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>

namespace tgw_planner::core
{
namespace
{
constexpr std::uint32_t kInvalidHybridNode = std::numeric_limits<std::uint32_t>::max();

struct QueueNode
{
  std::uint32_t node{kInvalidHybridNode};
  double cost{0.0};
  double priority{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.priority > rhs.priority;
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

bool validHybridId(const HybridExperienceGraph & graph, std::uint32_t id)
{
  return graph.isValid(id);
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
  const HybridExperienceGraph & graph, std::uint32_t from, std::uint32_t to)
{
  if (!validHybridId(graph, from)) {
    return nullptr;
  }
  for (const HybridEdge & edge : graph.adjacency()[from]) {
    if (edge.to == to) {
      return &edge;
    }
  }
  return nullptr;
}

PathPointKind nodePathKind(
  const HybridExperienceGraph & graph, const std::vector<std::uint32_t> & path, std::size_t index)
{
  if (index > 0U) {
    const HybridEdge * edge = findEdge(graph, path[index - 1U], path[index]);
    if (edge != nullptr && edge->kind == HybridEdgeKind::Portal) {
      return PathPointKind::Portal;
    }
  }
  if (index + 1U < path.size()) {
    const HybridEdge * edge = findEdge(graph, path[index], path[index + 1U]);
    if (edge != nullptr && edge->kind == HybridEdgeKind::Portal) {
      return PathPointKind::Portal;
    }
  }
  const HybridNode & node = graph.nodes()[path[index]];
  return node.kind == HybridNodeKind::Backbone ? PathPointKind::Backbone : PathPointKind::Surface;
}

double edgeSlope(double length_xy_m, double dz_m)
{
  return length_xy_m > 1.0e-9 ? std::abs(dz_m) / length_xy_m :
         std::numeric_limits<double>::infinity();
}

}  // namespace

HybridBackboneEdgeValidation validateBackboneEdgeForHybrid(
  double length_xy_m,
  double dz_m,
  const HybridExperiencePlannerOptions & options)
{
  HybridBackboneEdgeValidation result;
  result.slope = edgeSlope(length_xy_m, dz_m);
  if (!std::isfinite(length_xy_m) || !std::isfinite(dz_m)) {
    result.reason = "non_finite";
    return result;
  }
  if (length_xy_m > options.max_backbone_edge_xy_gap_m) {
    result.reason = "xy_gap";
    return result;
  }
  if (std::abs(dz_m) > options.max_backbone_edge_dz_m) {
    result.reason = "height_delta";
    return result;
  }
  if (result.slope > options.max_backbone_edge_slope) {
    result.reason = "slope";
    return result;
  }
  result.allowed = true;
  result.reason = "";
  return result;
}

HybridBackboneEdgeValidation validateBackboneEdgeForHybrid(
  const BackboneEdge & edge,
  const HybridExperiencePlannerOptions & options)
{
  return validateBackboneEdgeForHybrid(edge.length_xy_m, edge.dz_m, options);
}

bool isBackboneEdgeAllowedForHybrid(
  const BackboneEdge & edge,
  const HybridExperiencePlannerOptions & options)
{
  return validateBackboneEdgeForHybrid(edge, options).allowed;
}

namespace
{

bool validateHybridPath(
  const HybridExperienceGraph & graph,
  const ExperienceBackboneGraph & backbone_graph,
  const std::vector<std::uint32_t> & path,
  const HybridExperiencePlannerOptions & options,
  std::string & failure_reason,
  SurfacePlanMetrics & metrics)
{
  if (path.empty()) {
    failure_reason = "path_validation_failed_empty_hybrid_path";
    return false;
  }
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const std::uint32_t from = path[i - 1U];
    const std::uint32_t to = path[i];
    if (!validHybridId(graph, from) || !validHybridId(graph, to)) {
      failure_reason = "path_validation_failed_invalid_hybrid_node";
      return false;
    }
    const HybridEdge * edge = findEdge(graph, from, to);
    if (edge == nullptr) {
      failure_reason = "path_validation_failed_missing_hybrid_edge";
      return false;
    }
    const double abs_dz = std::abs(edge->dz_m);
    metrics.max_path_edge_dz_m = std::max(metrics.max_path_edge_dz_m, abs_dz);
    if (!std::isfinite(edge->cost) || edge->cost < 0.0) {
      failure_reason = "path_validation_failed_invalid_hybrid_edge_cost";
      return false;
    }
    if (edge->kind == HybridEdgeKind::Surface) {
      if (graph.nodes()[from].kind != HybridNodeKind::Surface ||
        graph.nodes()[to].kind != HybridNodeKind::Surface)
      {
        failure_reason = "path_validation_failed_invalid_surface_edge";
        return false;
      }
      continue;
    }
    if (edge->kind == HybridEdgeKind::Backbone) {
      const HybridBackboneEdgeValidation edge_validation =
        validateBackboneEdgeForHybrid(edge->length_xy_m, edge->dz_m, options);
      if (graph.nodes()[from].kind != HybridNodeKind::Backbone ||
        graph.nodes()[to].kind != HybridNodeKind::Backbone ||
        !edge_validation.allowed)
      {
        std::ostringstream message;
        message << "path_validation_failed_invalid_backbone_edge"
          << " reason=" << edge_validation.reason
          << " from=" << graph.nodes()[from].backbone_id.id
          << " to=" << graph.nodes()[to].backbone_id.id
          << " xy=" << edge->length_xy_m
          << " dz=" << abs_dz
          << " slope=" << edge_validation.slope
          << " max_xy=" << options.max_backbone_edge_xy_gap_m
          << " max_dz=" << options.max_backbone_edge_dz_m
          << " max_slope=" << options.max_backbone_edge_slope;
        failure_reason = message.str();
        return false;
      }
      continue;
    }
    if (edge->kind == HybridEdgeKind::Portal) {
      const ExperiencePortal * portal = backbone_graph.portal(edge->portal_id);
      if (portal == nullptr ||
        portal->distance_xy_m > options.max_portal_xy_distance_m ||
        portal->height_error_m > options.max_portal_height_error_m)
      {
        failure_reason = "path_validation_failed_invalid_portal_edge";
        return false;
      }
      continue;
    }
  }
  return true;
}

}  // namespace

void HybridExperienceGraph::build(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  const SurfacePlannerOptions & surface_options,
  const HybridExperiencePlannerOptions & hybrid_options)
{
  nodes_.clear();
  adjacency_.clear();
  surface_edge_count_ = 0U;
  backbone_edge_count_ = 0U;
  portal_edge_count_ = 0U;
  backbone_offset_ = saturatedSize(surface_graph.nodes().size());
  nodes_.reserve(surface_graph.nodes().size() + backbone_graph.nodes().size());

  for (const SurfaceNode & surface_node : surface_graph.nodes()) {
    HybridNode node;
    node.kind = HybridNodeKind::Surface;
    node.surface_id = surface_node.id;
    node.point = surfacePoint(surface_graph, surface_node.id);
    nodes_.push_back(node);
  }
  for (const BackboneNode & backbone_node : backbone_graph.nodes()) {
    HybridNode node;
    node.kind = HybridNodeKind::Backbone;
    node.backbone_id = backbone_node.id;
    node.point = backbone_node.path_position;
    nodes_.push_back(node);
  }

  adjacency_.resize(nodes_.size());
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
      adjacency_[surface_node.id.id].push_back(edge);
      ++surface_edge_count_;
    }
  }

  for (const BackboneEdge & backbone_edge : backbone_graph.edges()) {
    if (!isBackboneEdgeAllowedForHybrid(backbone_edge, hybrid_options)) {
      continue;
    }
    const std::uint32_t from = backbone_offset_ + backbone_edge.from.id;
    const std::uint32_t to = backbone_offset_ + backbone_edge.to.id;
    if (!isValid(from) || !isValid(to)) {
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
    adjacency_[from].push_back(forward);
    adjacency_[to].push_back(reverse);
    backbone_edge_count_ += 2U;
  }

  for (const ExperiencePortal & portal : backbone_graph.portals()) {
    if (!surface_graph.isValid(portal.surface_node) ||
      !backbone_graph.isValid(portal.backbone_node))
    {
      continue;
    }
    const std::uint32_t surface_id = portal.surface_node.id;
    const std::uint32_t backbone_id = backbone_offset_ + portal.backbone_node.id;
    if (!isValid(surface_id) || !isValid(backbone_id)) {
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
      nodes_[backbone_id].point.z - nodes_[surface_id].point.z;
    surface_to_backbone.portal_id = portal.id;
    HybridEdge backbone_to_surface = surface_to_backbone;
    backbone_to_surface.to = surface_id;
    backbone_to_surface.dz_m = -surface_to_backbone.dz_m;
    adjacency_[surface_id].push_back(surface_to_backbone);
    adjacency_[backbone_id].push_back(backbone_to_surface);
    portal_edge_count_ += 2U;
  }
}

bool HybridExperienceGraph::empty() const
{
  return nodes_.empty();
}

bool HybridExperienceGraph::isValid(std::uint32_t id) const
{
  return id < nodes_.size();
}

const std::vector<HybridNode> & HybridExperienceGraph::nodes() const
{
  return nodes_;
}

const std::vector<std::vector<HybridEdge>> & HybridExperienceGraph::adjacency() const
{
  return adjacency_;
}

std::uint32_t HybridExperienceGraph::backboneOffset() const
{
  return backbone_offset_;
}

std::uint32_t HybridExperienceGraph::surfaceEdgeCount() const
{
  return surface_edge_count_;
}

std::uint32_t HybridExperienceGraph::backboneEdgeCount() const
{
  return backbone_edge_count_;
}

std::uint32_t HybridExperienceGraph::portalEdgeCount() const
{
  return portal_edge_count_;
}

HybridExperiencePlanner::HybridExperiencePlanner(
  SurfacePlannerOptions surface_options,
  HybridExperiencePlannerOptions hybrid_options)
: surface_options_(surface_options), hybrid_options_(hybrid_options)
{
  hybrid_options_.backbone_cost_scale = std::max(0.1, hybrid_options_.backbone_cost_scale);
  hybrid_options_.portal_switch_cost = std::max(0.0, hybrid_options_.portal_switch_cost);
  hybrid_options_.max_backbone_edge_xy_gap_m =
    std::max(0.05, hybrid_options_.max_backbone_edge_xy_gap_m);
  hybrid_options_.max_backbone_edge_dz_m =
    std::max(0.05, hybrid_options_.max_backbone_edge_dz_m);
  hybrid_options_.max_backbone_edge_slope =
    std::max(0.1, hybrid_options_.max_backbone_edge_slope);
  hybrid_options_.max_portal_xy_distance_m =
    std::max(0.0, hybrid_options_.max_portal_xy_distance_m);
  hybrid_options_.max_portal_height_error_m =
    std::max(0.0, hybrid_options_.max_portal_height_error_m);
  hybrid_options_.surface_target_speed_mps =
    std::max(0.0, hybrid_options_.surface_target_speed_mps);
  hybrid_options_.backbone_target_speed_mps =
    std::max(0.0, hybrid_options_.backbone_target_speed_mps);
  hybrid_options_.portal_target_speed_mps =
    std::max(0.0, hybrid_options_.portal_target_speed_mps);
}

SurfacePlanResult HybridExperiencePlanner::plan(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  SurfaceNodeId start,
  SurfaceNodeId goal) const
{
  HybridExperienceGraph graph;
  graph.build(surface_graph, backbone_graph, surface_options_, hybrid_options_);
  return plan(surface_graph, backbone_graph, graph, start, goal);
}

SurfacePlanResult HybridExperiencePlanner::plan(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  const HybridExperienceGraph & graph,
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

  const bool same_surface_component = surface_graph.sameComponent(start, goal);
  if (backbone_graph.empty() || backbone_graph.portals().empty()) {
    if (same_surface_component) {
      return SurfaceAstarPlanner(surface_options_).plan(surface_graph, start, goal);
    }
    result.message = "no_backbone_portal_for_start_or_goal";
    result.metrics.failure_reason = result.message;
    return result;
  }

  if (graph.empty()) {
    result.message = "hybrid graph is empty";
    result.metrics.failure_reason = result.message;
    return result;
  }
  const std::uint32_t start_id = start.id;
  const std::uint32_t goal_id = goal.id;
  if (!validHybridId(graph, start_id) || !validHybridId(graph, goal_id)) {
    result.message = "start or goal is not in the hybrid graph";
    result.metrics.failure_reason = result.message;
    return result;
  }

  result.metrics.hybrid_nodes = saturatedSize(graph.nodes().size());
  result.metrics.hybrid_surface_edges = graph.surfaceEdgeCount();
  result.metrics.hybrid_backbone_edges = graph.backboneEdgeCount();
  result.metrics.hybrid_portal_edges = graph.portalEdgeCount();
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

  std::vector<double> cost(graph.nodes().size(), std::numeric_limits<double>::infinity());
  std::vector<std::uint32_t> previous(graph.nodes().size(), kInvalidHybridNode);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
  const auto heuristic = [&](std::uint32_t node_id) {
      return validHybridId(graph, node_id) ?
             xyDistance(graph.nodes()[node_id].point, graph.nodes()[goal_id].point) : 0.0;
    };
  cost[start_id] = 0.0;
  open.push({start_id, 0.0, heuristic(start_id)});
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
    for (const HybridEdge & edge : graph.adjacency()[current.node]) {
      if (!validHybridId(graph, edge.to)) {
        continue;
      }
      const double next_cost = current.cost + edge.cost;
      if (next_cost >= cost[edge.to]) {
        continue;
      }
      cost[edge.to] = next_cost;
      previous[edge.to] = current.node;
      open.push({edge.to, next_cost, next_cost + heuristic(edge.to)});
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

  std::string validation_failure;
  if (!validateHybridPath(
      graph, backbone_graph, hybrid_path, hybrid_options_, validation_failure, result.metrics))
  {
    result.message = validation_failure;
    result.metrics.failure_reason = result.message;
    result.metrics.final_path_validation_failure = validation_failure;
    return result;
  }

  result.success = true;
  result.message = "path found on hybrid experience graph";
  result.metrics.success = true;
  result.metrics.final_path_validated = true;
  result.metrics.selected_total_hybrid_cost = cost[goal_id];
  result.raw_path.reserve(hybrid_path.size());
  result.path.reserve(hybrid_path.size());
  result.path_kinds.reserve(hybrid_path.size());
  double clearance_sum = 0.0;
  std::uint32_t clearance_samples = 0U;
  for (std::size_t i = 0U; i < hybrid_path.size(); ++i) {
    const std::uint32_t node_id = hybrid_path[i];
    const HybridNode & node = graph.nodes()[node_id];
    const PathPointKind kind = nodePathKind(graph, hybrid_path, i);
    int component_id = -1;
    double confidence = 1.0;
    if (node.kind == HybridNodeKind::Surface) {
      component_id = surface_graph.componentId(node.surface_id);
      const SurfaceNode * surface_node = surface_graph.node(node.surface_id);
      if (surface_node != nullptr) {
        confidence = surface_node->confidence;
        const double clearance = surface_node->clearance_m;
        if (!result.metrics.has_min_path_clearance_cell ||
          clearance < result.metrics.min_path_clearance_m)
        {
          result.metrics.min_path_clearance_m = clearance;
          result.metrics.min_path_clearance_cell = surface_node->cell;
          result.metrics.has_min_path_clearance_cell = true;
        }
        clearance_sum += clearance;
        ++clearance_samples;
        result.metrics.clearance_cost_sum +=
          std::isfinite(clearance) ? 1.0 / (clearance + 0.05) : 0.0;
        result.metrics.risk_cost_sum += surface_node->risk;
        result.metrics.max_path_risk =
          std::max(result.metrics.max_path_risk, surface_node->risk);
        if (clearance < 0.30) {
          ++result.metrics.low_clearance_samples;
        }
      }
    } else if (node.kind == HybridNodeKind::Backbone) {
      const BackboneNode * backbone_node = backbone_graph.node(node.backbone_id);
      if (backbone_node != nullptr) {
        confidence = backbone_node->low_confidence ? 0.5 : 1.0;
        component_id = backbone_node->nearest_surface_component_id;
      }
    }
    appendPathPoint(result, node.point, kind);
    appendGlobalPathPoint(result, node.point, kind, component_id, confidence);
    appendPoint(result.raw_path, node.point);
    if (node.kind == HybridNodeKind::Surface) {
      const SurfaceNode * surface_node = surface_graph.node(node.surface_id);
      if (surface_node != nullptr) {
        result.cells.push_back(surface_node->cell);
        result.raw_cells.push_back(surface_node->cell);
      }
    }
  }
  if (clearance_samples > 0U) {
    result.metrics.mean_path_clearance_m =
      clearance_sum / static_cast<double>(clearance_samples);
  }

  std::size_t first_portal_edge_index = std::numeric_limits<std::size_t>::max();
  std::size_t last_portal_edge_index = std::numeric_limits<std::size_t>::max();
  for (std::size_t i = 1U; i < hybrid_path.size(); ++i) {
    const HybridEdge * edge = findEdge(graph, hybrid_path[i - 1U], hybrid_path[i]);
    if (edge != nullptr && edge->kind == HybridEdgeKind::Portal) {
      if (first_portal_edge_index == std::numeric_limits<std::size_t>::max()) {
        first_portal_edge_index = i;
      }
      last_portal_edge_index = i;
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
      if (first_portal_edge_index == std::numeric_limits<std::size_t>::max() ||
        i < first_portal_edge_index)
      {
        result.metrics.selected_start_surface_leg_m += edge->length_xy_m;
      } else if (i > last_portal_edge_index) {
        result.metrics.selected_goal_surface_leg_m += edge->length_xy_m;
      }
    } else if (edge->kind == HybridEdgeKind::Backbone) {
      ++result.metrics.used_backbone_edges;
      result.metrics.backbone_path_length_m += edge->length_xy_m;
      appendPoint(
        result.debug_selected_backbone_segment, graph.nodes()[hybrid_path[i - 1U]].point);
      appendPoint(result.debug_selected_backbone_segment, graph.nodes()[hybrid_path[i]].point);
    } else {
      ++result.metrics.used_portal_edges;
      ++result.metrics.portal_switch_count;
      const Point3 portal_point =
        graph.nodes()[hybrid_path[i - 1U]].kind == HybridNodeKind::Surface ?
        graph.nodes()[hybrid_path[i - 1U]].point : graph.nodes()[hybrid_path[i]].point;
      if (result.debug_selected_start_portal.empty()) {
        result.debug_selected_start_portal.push_back(portal_point);
        result.metrics.selected_start_portal_id = edge->portal_id.id;
        const HybridNode & backbone_node =
          graph.nodes()[hybrid_path[i - 1U]].kind == HybridNodeKind::Backbone ?
          graph.nodes()[hybrid_path[i - 1U]] : graph.nodes()[hybrid_path[i]];
        result.metrics.selected_start_backbone_node = backbone_node.backbone_id.id;
      } else {
        result.debug_selected_goal_portal.clear();
        result.debug_selected_goal_portal.push_back(portal_point);
        result.metrics.selected_goal_portal_id = edge->portal_id.id;
        const HybridNode & backbone_node =
          graph.nodes()[hybrid_path[i - 1U]].kind == HybridNodeKind::Backbone ?
          graph.nodes()[hybrid_path[i - 1U]] : graph.nodes()[hybrid_path[i]];
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
  result.metrics.selected_total_hybrid_cost = cost[goal_id];
  fillGlobalPathYaws(result);
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

void HybridExperiencePlanner::appendPathPoint(
  SurfacePlanResult & result, const Point3 & point, PathPointKind kind) const
{
  if (!result.path.empty() && xyDistance(result.path.back(), point) < 1.0e-6 &&
    std::abs(result.path.back().z - point.z) < 1.0e-6)
  {
    if (!result.path_kinds.empty() &&
      result.path_kinds.back() != PathPointKind::Portal &&
      kind == PathPointKind::Portal)
    {
      result.path_kinds.back() = kind;
    }
    return;
  }
  result.path.push_back(point);
  result.path_kinds.push_back(kind);
}

void HybridExperiencePlanner::appendGlobalPathPoint(
  SurfacePlanResult & result,
  const Point3 & point,
  PathPointKind kind,
  int surface_component_id,
  double confidence) const
{
  if (!result.global_path.empty() && xyDistance(result.global_path.back().position, point) < 1.0e-6 &&
    std::abs(result.global_path.back().position.z - point.z) < 1.0e-6)
  {
    if (result.global_path.back().kind != PathPointKind::Portal &&
      kind == PathPointKind::Portal)
    {
      result.global_path.back().kind = kind;
      result.global_path.back().target_speed_mps = hybrid_options_.portal_target_speed_mps;
    }
    return;
  }
  double target_speed = hybrid_options_.surface_target_speed_mps;
  if (kind == PathPointKind::Backbone) {
    target_speed = hybrid_options_.backbone_target_speed_mps;
  } else if (kind == PathPointKind::Portal) {
    target_speed = hybrid_options_.portal_target_speed_mps;
  }
  result.global_path.push_back({
      point,
      0.0,
      kind,
      target_speed,
      confidence,
      surface_component_id});
}

void HybridExperiencePlanner::fillGlobalPathYaws(SurfacePlanResult & result) const
{
  for (std::size_t i = 0U; i < result.global_path.size(); ++i) {
    if (result.global_path.size() == 1U) {
      result.global_path[i].yaw_hint_rad = 0.0;
      continue;
    }
    const Point3 & from =
      i + 1U < result.global_path.size() ? result.global_path[i].position :
      result.global_path[i - 1U].position;
    const Point3 & to =
      i + 1U < result.global_path.size() ? result.global_path[i + 1U].position :
      result.global_path[i].position;
    result.global_path[i].yaw_hint_rad = std::atan2(to.y - from.y, to.x - from.x);
  }
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
  result.metrics.final_path_validated = true;
  result.metrics.success = result.success;
  for (std::size_t i = 1U; i < result.path.size(); ++i) {
    const double edge_dz = std::abs(result.path[i].z - result.path[i - 1U].z);
    result.metrics.max_path_edge_dz_m =
      std::max(result.metrics.max_path_edge_dz_m, edge_dz);
  }
  if (!result.metrics.has_min_path_clearance_cell) {
    result.metrics.min_path_clearance_m = 0.0;
    result.metrics.mean_path_clearance_m = 0.0;
  }
}

}  // namespace tgw_planner::core
