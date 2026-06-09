#include "tgw_planner/core/hybrid_experience_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tgw_planner::core
{
namespace
{
double xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
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

  bool found = false;
  double best_cost = std::numeric_limits<double>::infinity();
  SurfacePlanResult best_result;
  for (const ExperiencePortalId start_portal_id : start_portals) {
    const ExperiencePortal * start_portal = backbone_graph.portal(start_portal_id);
    if (start_portal == nullptr) {
      continue;
    }
    const SurfacePlanResult start_leg =
      surface_planner.plan(surface_graph, start, start_portal->surface_node);
    if (!start_leg.success) {
      continue;
    }
    for (const ExperiencePortalId goal_portal_id : goal_portals) {
      const ExperiencePortal * goal_portal = backbone_graph.portal(goal_portal_id);
      if (goal_portal == nullptr) {
        continue;
      }
      const SurfacePlanResult goal_leg =
        surface_planner.plan(surface_graph, goal_portal->surface_node, goal);
      if (!goal_leg.success) {
        continue;
      }
      const std::vector<Point3> backbone_path = backbone_graph.pathPositionsBetween(
        start_portal->backbone_node, goal_portal->backbone_node);
      if (backbone_path.empty()) {
        continue;
      }
      const double cost = start_leg.metrics.path_length_m +
        backbone_graph.pathLengthBetween(start_portal->backbone_node, goal_portal->backbone_node) +
        goal_leg.metrics.path_length_m + start_portal->distance_xy_m + goal_portal->distance_xy_m;
      if (cost >= best_cost) {
        continue;
      }

      SurfacePlanResult candidate;
      candidate.success = true;
      candidate.message = "path found via dense trajectory backbone";
      candidate.metrics.success = true;
      candidate.metrics.final_path_validated = true;
      candidate.metrics.expanded_nodes =
        start_leg.metrics.expanded_nodes + goal_leg.metrics.expanded_nodes;
      candidate.metrics.generated_nodes =
        start_leg.metrics.generated_nodes + goal_leg.metrics.generated_nodes;
      candidate.cells = start_leg.cells;
      candidate.cells.insert(candidate.cells.end(), goal_leg.cells.begin(), goal_leg.cells.end());
      appendPath(candidate.path, start_leg.path);
      appendPath(candidate.path, backbone_path);
      appendPath(candidate.path, goal_leg.path);
      candidate.raw_cells = candidate.cells;
      candidate.raw_path = candidate.path;
      fillPathMetrics(candidate);
      best_cost = cost;
      best_result = candidate;
      found = true;
    }
  }

  if (!found) {
    result.message = "no_surface_path_to_backbone_portal";
    result.metrics.failure_reason = result.message;
    return result;
  }
  return best_result;
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
