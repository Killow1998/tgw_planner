#include "tgw_planner/core/hybrid_experience_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace tgw_planner::core
{
namespace
{
constexpr std::uint32_t kInvalidNodeId = std::numeric_limits<std::uint32_t>::max();

struct QueueNode
{
  SurfaceNodeId node;
  double cost{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.cost > rhs.cost;
  }
};

struct SurfaceDistanceTree
{
  SurfaceNodeId source{kInvalidNodeId};
  std::vector<double> cost;
  std::vector<SurfaceNodeId> previous;
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

SurfaceDistanceTree buildSurfaceDistanceTree(
  const ExperienceSurfaceGraph & graph, SurfaceNodeId source)
{
  SurfaceDistanceTree tree;
  tree.source = source;
  tree.cost.assign(graph.nodes().size(), std::numeric_limits<double>::infinity());
  tree.previous.assign(graph.nodes().size(), {kInvalidNodeId});
  if (!graph.isValid(source)) {
    return tree;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
  tree.cost[source.id] = 0.0;
  open.push({source, 0.0});
  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();
    if (!graph.isValid(current.node) ||
      current.cost > tree.cost[current.node.id] + 1.0e-9)
    {
      continue;
    }
    for (const SurfaceEdge & edge : graph.adjacency()[current.node.id]) {
      if (!graph.isValid(edge.to)) {
        continue;
      }
      const double edge_cost = edge.cost > 0.0 ? edge.cost : edge.length_xy_m;
      const double next_cost = current.cost + edge_cost;
      if (next_cost >= tree.cost[edge.to.id]) {
        continue;
      }
      tree.cost[edge.to.id] = next_cost;
      tree.previous[edge.to.id] = current.node;
      open.push({edge.to, next_cost});
    }
  }
  return tree;
}

std::vector<SurfaceNodeId> reconstructSurfacePath(
  const ExperienceSurfaceGraph & graph,
  const SurfaceDistanceTree & tree,
  SurfaceNodeId target)
{
  std::vector<SurfaceNodeId> out;
  if (!graph.isValid(tree.source) || !graph.isValid(target) ||
    target.id >= tree.cost.size() || !std::isfinite(tree.cost[target.id]))
  {
    return out;
  }
  SurfaceNodeId current = target;
  out.push_back(current);
  while (current != tree.source) {
    if (current.id >= tree.previous.size() || !graph.isValid(tree.previous[current.id])) {
      out.clear();
      return out;
    }
    current = tree.previous[current.id];
    out.push_back(current);
  }
  std::reverse(out.begin(), out.end());
  return out;
}

std::vector<Point3> nodesToPoints(
  const ExperienceSurfaceGraph & graph, const std::vector<SurfaceNodeId> & nodes)
{
  std::vector<Point3> out;
  out.reserve(nodes.size());
  for (const SurfaceNodeId node_id : nodes) {
    out.push_back(surfacePoint(graph, node_id));
  }
  return out;
}

std::vector<GridIndex> nodesToCells(
  const ExperienceSurfaceGraph & graph, const std::vector<SurfaceNodeId> & nodes)
{
  std::vector<GridIndex> out;
  out.reserve(nodes.size());
  for (const SurfaceNodeId node_id : nodes) {
    const SurfaceNode * node = graph.node(node_id);
    if (node != nullptr) {
      out.push_back(node->cell);
    }
  }
  return out;
}

std::uint32_t saturatedSize(std::size_t value)
{
  return static_cast<std::uint32_t>(
    std::min<std::size_t>(value, std::numeric_limits<std::uint32_t>::max()));
}
}  // namespace

HybridExperiencePlanner::HybridExperiencePlanner(
  SurfacePlannerOptions surface_options,
  HybridExperiencePlannerOptions hybrid_options)
: surface_options_(surface_options), hybrid_options_(hybrid_options)
{
  hybrid_options_.max_portal_candidates_per_side =
    std::max<std::size_t>(1U, hybrid_options_.max_portal_candidates_per_side);
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

  SurfaceAstarPlanner surface_planner(surface_options_);
  if (surface_graph.sameComponent(start, goal)) {
    return surface_planner.plan(surface_graph, start, goal);
  }
  if (backbone_graph.empty() || backbone_graph.portals().empty()) {
    result.message = "no_backbone_portal_for_start_or_goal";
    result.metrics.failure_reason = result.message;
    return result;
  }

  const int start_component = surface_graph.componentId(start);
  const int goal_component = surface_graph.componentId(goal);
  const std::vector<ExperiencePortalId> start_portals =
    sortedPortalCandidates(surface_graph, backbone_graph, start_component, start);
  const std::vector<ExperiencePortalId> goal_portals =
    sortedPortalCandidates(surface_graph, backbone_graph, goal_component, goal);
  if (start_portals.empty() || goal_portals.empty()) {
    result.message = "no_backbone_portal_for_start_or_goal";
    result.metrics.failure_reason = result.message;
    return result;
  }
  result.metrics.start_portal_candidates = saturatedSize(start_portals.size());
  result.metrics.goal_portal_candidates = saturatedSize(goal_portals.size());
  for (const ExperiencePortalId portal_id : start_portals) {
    const ExperiencePortal * portal = backbone_graph.portal(portal_id);
    if (portal != nullptr) {
      result.debug_start_portal_candidates.push_back(surfacePoint(surface_graph, portal->surface_node));
    }
  }
  for (const ExperiencePortalId portal_id : goal_portals) {
    const ExperiencePortal * portal = backbone_graph.portal(portal_id);
    if (portal != nullptr) {
      result.debug_goal_portal_candidates.push_back(surfacePoint(surface_graph, portal->surface_node));
    }
  }

  const SurfaceDistanceTree start_tree = buildSurfaceDistanceTree(surface_graph, start);
  const SurfaceDistanceTree goal_tree = buildSurfaceDistanceTree(surface_graph, goal);

  bool found = false;
  double best_cost = std::numeric_limits<double>::infinity();
  ExperiencePortalId best_start_portal_id{kInvalidNodeId};
  ExperiencePortalId best_goal_portal_id{kInvalidNodeId};
  double best_start_surface_cost = 0.0;
  double best_goal_surface_cost = 0.0;
  double best_backbone_cost = 0.0;
  for (const ExperiencePortalId start_portal_id : start_portals) {
    const ExperiencePortal * start_portal = backbone_graph.portal(start_portal_id);
    if (start_portal == nullptr) {
      continue;
    }
    if (start_portal->surface_node.id >= start_tree.cost.size() ||
      !std::isfinite(start_tree.cost[start_portal->surface_node.id]))
    {
      continue;
    }
    for (const ExperiencePortalId goal_portal_id : goal_portals) {
      ++result.metrics.evaluated_portal_pairs;
      const ExperiencePortal * goal_portal = backbone_graph.portal(goal_portal_id);
      if (goal_portal == nullptr) {
        continue;
      }
      if (goal_portal->surface_node.id >= goal_tree.cost.size() ||
        !std::isfinite(goal_tree.cost[goal_portal->surface_node.id]))
      {
        continue;
      }
      const double start_surface_cost = start_tree.cost[start_portal->surface_node.id];
      const double goal_surface_cost = goal_tree.cost[goal_portal->surface_node.id];
      const double backbone_cost = backbone_graph.pathLengthBetween(
        start_portal->backbone_node, goal_portal->backbone_node);
      const double cost = start_surface_cost + backbone_cost + goal_surface_cost +
        start_portal->distance_xy_m + goal_portal->distance_xy_m +
        0.25 * (start_portal->height_error_m + goal_portal->height_error_m);
      if (cost >= best_cost) {
        continue;
      }
      best_cost = cost;
      best_start_portal_id = start_portal_id;
      best_goal_portal_id = goal_portal_id;
      best_start_surface_cost = start_surface_cost;
      best_goal_surface_cost = goal_surface_cost;
      best_backbone_cost = backbone_cost;
      found = true;
    }
  }

  if (!found) {
    result.message = "no_surface_path_to_backbone_portal";
    result.metrics.failure_reason = result.message;
    return result;
  }

  const ExperiencePortal * best_start_portal = backbone_graph.portal(best_start_portal_id);
  const ExperiencePortal * best_goal_portal = backbone_graph.portal(best_goal_portal_id);
  if (best_start_portal == nullptr || best_goal_portal == nullptr) {
    result.message = "no_surface_path_to_backbone_portal";
    result.metrics.failure_reason = result.message;
    return result;
  }
  std::vector<SurfaceNodeId> start_leg_nodes = reconstructSurfacePath(
    surface_graph, start_tree, best_start_portal->surface_node);
  std::vector<SurfaceNodeId> goal_to_portal_nodes = reconstructSurfacePath(
    surface_graph, goal_tree, best_goal_portal->surface_node);
  if (start_leg_nodes.empty() || goal_to_portal_nodes.empty()) {
    result.message = "no_surface_path_to_backbone_portal";
    result.metrics.failure_reason = result.message;
    return result;
  }
  std::reverse(goal_to_portal_nodes.begin(), goal_to_portal_nodes.end());

  const std::vector<Point3> start_leg_path = nodesToPoints(surface_graph, start_leg_nodes);
  const std::vector<Point3> backbone_path = backbone_graph.pathPositionsBetween(
    best_start_portal->backbone_node, best_goal_portal->backbone_node);
  const std::vector<Point3> goal_leg_path = nodesToPoints(surface_graph, goal_to_portal_nodes);
  if (backbone_path.empty()) {
    result.message = "no_surface_path_to_backbone_portal";
    result.metrics.failure_reason = result.message;
    return result;
  }

  result.success = true;
  result.message = "path found via dense trajectory backbone";
  result.metrics.success = true;
  result.metrics.final_path_validated = true;
  result.cells = nodesToCells(surface_graph, start_leg_nodes);
  const std::vector<GridIndex> goal_cells = nodesToCells(surface_graph, goal_to_portal_nodes);
  result.cells.insert(result.cells.end(), goal_cells.begin(), goal_cells.end());
  appendPath(result.path, start_leg_path);
  appendPath(result.path, backbone_path);
  appendPath(result.path, goal_leg_path);
  result.raw_cells = result.cells;
  result.raw_path = result.path;
  result.debug_selected_start_portal.push_back(surfacePoint(surface_graph, best_start_portal->surface_node));
  result.debug_selected_goal_portal.push_back(surfacePoint(surface_graph, best_goal_portal->surface_node));
  result.debug_selected_backbone_segment = backbone_path;
  result.metrics.selected_start_portal_id = best_start_portal_id.id;
  result.metrics.selected_goal_portal_id = best_goal_portal_id.id;
  result.metrics.selected_start_backbone_node = best_start_portal->backbone_node.id;
  result.metrics.selected_goal_backbone_node = best_goal_portal->backbone_node.id;
  result.metrics.selected_backbone_index_delta =
    best_start_portal->backbone_node.id > best_goal_portal->backbone_node.id ?
    best_start_portal->backbone_node.id - best_goal_portal->backbone_node.id :
    best_goal_portal->backbone_node.id - best_start_portal->backbone_node.id;
  result.metrics.selected_backbone_length_m = best_backbone_cost;
  result.metrics.selected_start_surface_leg_m = best_start_surface_cost;
  result.metrics.selected_goal_surface_leg_m = best_goal_surface_cost;
  result.metrics.selected_total_hybrid_cost = best_cost;
  fillPathMetrics(result);
  return result;
}

std::vector<ExperiencePortalId> HybridExperiencePlanner::sortedPortalCandidates(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph,
  int surface_component_id,
  SurfaceNodeId query_node) const
{
  std::vector<ExperiencePortalId> candidates =
    backbone_graph.portalsForSurfaceComponent(surface_component_id);
  if (candidates.empty() || !surface_graph.isValid(query_node)) {
    return {};
  }

  const Point3 query = surfacePoint(surface_graph, query_node);
  std::sort(
    candidates.begin(), candidates.end(),
    [&](ExperiencePortalId lhs, ExperiencePortalId rhs) {
      const ExperiencePortal * left = backbone_graph.portal(lhs);
      const ExperiencePortal * right = backbone_graph.portal(rhs);
      if (left == nullptr || right == nullptr) {
        return right != nullptr;
      }
      const Point3 left_point = surfacePoint(surface_graph, left->surface_node);
      const Point3 right_point = surfacePoint(surface_graph, right->surface_node);
      const double left_score = xyDistance(query, left_point) + 0.25 * left->distance_xy_m;
      const double right_score = xyDistance(query, right_point) + 0.25 * right->distance_xy_m;
      return left_score < right_score;
    });
  if (candidates.size() > hybrid_options_.max_portal_candidates_per_side) {
    candidates.resize(hybrid_options_.max_portal_candidates_per_side);
  }
  return candidates;
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

void HybridExperiencePlanner::appendPath(
  std::vector<Point3> & out, const std::vector<Point3> & in) const
{
  for (const Point3 & point : in) {
    if (!out.empty() && xyDistance(out.back(), point) < 1.0e-6 &&
      std::abs(out.back().z - point.z) < 1.0e-6)
    {
      continue;
    }
    out.push_back(point);
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
