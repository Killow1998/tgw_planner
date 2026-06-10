#include "tgw_planner/core/surface_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace tgw_planner::core
{
namespace
{
struct QueueNode
{
  GridIndex cell;
  double f{0.0};
  double g{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

struct GraphQueueNode
{
  SurfaceNodeId node;
  double f{0.0};
  double g{0.0};
};

struct GraphQueueCompare
{
  bool operator()(const GraphQueueNode & lhs, const GraphQueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

double gridDistance(const GridIndex & a, const GridIndex & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string cellString(const GridIndex & cell)
{
  return "[" + std::to_string(cell.x) + "," + std::to_string(cell.y) + "," +
         std::to_string(cell.z) + "]";
}
}  // namespace

SurfaceTransitionValidator::SurfaceTransitionValidator(SurfacePlannerOptions options)
: options_(options), footprint_(options.footprint)
{
  options_.max_iterations = std::max<std::uint32_t>(1U, options_.max_iterations);
  options_.max_step_height_m = std::max(0.05, options_.max_step_height_m);
  options_.swept_sample_step_m = std::max(0.01, options_.swept_sample_step_m);
  options_.shortcut_sample_step_m = std::max(0.01, options_.shortcut_sample_step_m);
  options_.shortcut_clearance_ratio = std::clamp(options_.shortcut_clearance_ratio, 0.0, 1.0);
  options_.final_validation_min_clearance_m =
    std::max(0.0, options_.final_validation_min_clearance_m);
}

TransitionReport SurfaceTransitionValidator::validate(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const
{
  if (from == to) {
    return {isCellTraversable(snapshot, from), isCellTraversable(snapshot, from) ? "" : "from_not_traversable"};
  }
  if (!isCellTraversable(snapshot, from)) {
    return {false, "from_not_traversable"};
  }
  if (!isCellTraversable(snapshot, to)) {
    return {false, "to_not_traversable"};
  }
  if (!isDiagonalCornerSupported(snapshot, from, to)) {
    return {false, "diagonal_corner_unsupported"};
  }

  const Point3 from_point = cellCenter(from, snapshot.resolution_m);
  const Point3 to_point = cellCenter(to, snapshot.resolution_m);
  const double yaw = std::atan2(to_point.y - from_point.y, to_point.x - from_point.x);
  if (isDirectSurfaceNeighbor(snapshot, from, to)) {
    if (!isFootprintSupported(snapshot, from_point, yaw)) {
      return {false, "from_footprint_not_supported"};
    }
    if (!isFootprintSupported(snapshot, to_point, yaw)) {
      return {false, "to_footprint_not_supported"};
    }
    return {true, ""};
  }

  const double segment_length = distance3d(from_point, to_point);
  const int steps = std::max(
    1, static_cast<int>(std::ceil(segment_length / options_.swept_sample_step_m)));
  for (int step = 0; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Point3 sample{
      from_point.x + (to_point.x - from_point.x) * t,
      from_point.y + (to_point.y - from_point.y) * t,
      from_point.z + (to_point.z - from_point.z) * t};
    if (!isCellTraversable(snapshot, worldToGrid(sample, snapshot.resolution_m))) {
      return {false, "swept_sample_not_traversable"};
    }
    if (!isFootprintSupported(snapshot, sample, yaw)) {
      return {false, "swept_footprint_not_supported"};
    }
  }
  return {true, ""};
}

std::vector<GridIndex> SurfaceTransitionValidator::validNeighbors(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  std::vector<GridIndex> neighbors;
  for (const GridIndex & candidate : snapshot.surface.traversable_cells) {
    const int dx = candidate.x - cell.x;
    const int dy = candidate.y - cell.y;
    if (dx == 0 && dy == 0) {
      continue;
    }
    if (std::abs(dx) > 1 || std::abs(dy) > 1) {
      continue;
    }
    if (validate(snapshot, cell, candidate).allowed) {
      neighbors.push_back(candidate);
    }
  }
  return neighbors;
}

bool SurfaceTransitionValidator::isCellTraversable(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  return snapshot.surface.traversable_cells.find(cell) != snapshot.surface.traversable_cells.end() &&
         snapshot.surface.blocked_cells.find(cell) == snapshot.surface.blocked_cells.end() &&
         snapshot.surface.forbidden_cells.find(cell) == snapshot.surface.forbidden_cells.end();
}

bool SurfaceTransitionValidator::isEndpointCell(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  if (!isCellTraversable(snapshot, cell)) {
    return false;
  }
  if (!options_.require_footprint_support) {
    return true;
  }
  const Point3 point = cellCenter(cell, snapshot.resolution_m);
  constexpr double headings[] = {0.0, 1.5707963267948966, 3.141592653589793, -1.5707963267948966};
  for (const double yaw : headings) {
    if (isFootprintSupported(snapshot, point, yaw)) {
      return true;
    }
  }
  return false;
}

bool SurfaceTransitionValidator::isFootprintSupported(
  const NavigationSnapshot & snapshot, const Point3 & point, double yaw_rad) const
{
  if (!options_.require_footprint_support) {
    return true;
  }
  return footprint_.isSupported(snapshot.surface, point, yaw_rad, snapshot.resolution_m);
}

bool SurfaceTransitionValidator::isDirectSurfaceNeighbor(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const
{
  const int dx = std::abs(to.x - from.x);
  const int dy = std::abs(to.y - from.y);
  const int dz = std::abs(to.z - from.z);
  if (dx == 0 && dy == 0) {
    return false;
  }
  const int max_step_cells =
    std::max(1, static_cast<int>(std::ceil(options_.max_step_height_m / snapshot.resolution_m)));
  return dx <= 1 && dy <= 1 && dz <= max_step_cells;
}

bool SurfaceTransitionValidator::isDiagonalCornerSupported(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to) const
{
  if (std::abs(to.x - from.x) != 1 || std::abs(to.y - from.y) != 1) {
    return true;
  }
  const int min_z = std::min(from.z, to.z);
  const int max_z = std::max(from.z, to.z);
  return hasTraversableCellAtXY(snapshot, to.x, from.y, min_z, max_z) &&
         hasTraversableCellAtXY(snapshot, from.x, to.y, min_z, max_z);
}

bool SurfaceTransitionValidator::hasTraversableCellAtXY(
  const NavigationSnapshot & snapshot, int x, int y, int min_z, int max_z) const
{
  for (int z = min_z; z <= max_z; ++z) {
    if (isCellTraversable(snapshot, {x, y, z})) {
      return true;
    }
  }
  return false;
}

Point3 SurfaceTransitionValidator::cellCenter(const GridIndex & cell, double resolution_m) const
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

GridIndex SurfaceTransitionValidator::worldToGrid(const Point3 & point, double resolution_m) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

SurfaceAstarPlanner::SurfaceAstarPlanner(SurfacePlannerOptions options)
: options_(options), transition_validator_(options)
{
  options_.max_iterations = std::max<std::uint32_t>(1U, options_.max_iterations);
  options_.max_step_height_m = std::max(0.05, options_.max_step_height_m);
  options_.swept_sample_step_m = std::max(0.01, options_.swept_sample_step_m);
  options_.shortcut_sample_step_m = std::max(0.01, options_.shortcut_sample_step_m);
  options_.shortcut_clearance_ratio = std::clamp(options_.shortcut_clearance_ratio, 0.0, 1.0);
  options_.final_validation_min_clearance_m =
    std::max(0.0, options_.final_validation_min_clearance_m);
}

SurfacePlanResult SurfaceAstarPlanner::plan(
  const NavigationSnapshot & snapshot, const GridIndex & start, const GridIndex & goal) const
{
  ExperienceSurfaceGraph graph;
  SurfaceGraphBuildOptions graph_options;
  graph_options.max_edge_height_delta_m = options_.max_step_height_m;
  graph.build(snapshot, transition_validator_, graph_options);
  const SurfaceNodeId start_node = graph.nodeIdForCell(start);
  const SurfaceNodeId goal_node = graph.nodeIdForCell(goal);
  return plan(graph, start_node, goal_node);
}

SurfacePlanResult SurfaceAstarPlanner::plan(
  const ExperienceSurfaceGraph & graph, SurfaceNodeId start, SurfaceNodeId goal) const
{
  SurfacePlanResult result;
  const SurfaceNode * start_node = graph.node(start);
  const SurfaceNode * goal_node = graph.node(goal);
  if (start_node == nullptr) {
    result.message = "start is not in the surface graph";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (goal_node == nullptr) {
    result.message = "goal is not in the surface graph";
    result.metrics.failure_reason = result.message;
    return result;
  }
  if (!graph.sameComponent(start, goal)) {
    result.message = "no_path_on_experience_surface_different_components";
    result.metrics.failure_reason = result.message;
    return result;
  }

  std::priority_queue<GraphQueueNode, std::vector<GraphQueueNode>, GraphQueueCompare> open;
  std::vector<double> g_score(graph.nodes().size(), std::numeric_limits<double>::infinity());
  std::vector<SurfaceNodeId> came_from(graph.nodes().size(), {std::numeric_limits<std::uint32_t>::max()});
  std::vector<bool> closed(graph.nodes().size(), false);

  auto heuristic = [&](SurfaceNodeId id) {
    const SurfaceNode & node = graph.nodes()[id.id];
    const double dx = static_cast<double>(node.x - goal_node->x) * graph.resolution();
    const double dy = static_cast<double>(node.y - goal_node->y) * graph.resolution();
    const double dz = node.z - goal_node->z;
    return std::sqrt(dx * dx + dy * dy) + 0.1 * std::abs(dz);
  };

  g_score[start.id] = 0.0;
  open.push({start, heuristic(start), 0.0});

  bool found = false;
  while (!open.empty() && result.metrics.expanded_nodes < options_.max_iterations) {
    const GraphQueueNode current = open.top();
    open.pop();
    if (!graph.isValid(current.node) || current.g > g_score[current.node.id] + 1.0e-9) {
      continue;
    }
    if (current.node == goal) {
      found = true;
      break;
    }
    if (closed[current.node.id]) {
      continue;
    }
    closed[current.node.id] = true;
    ++result.metrics.expanded_nodes;

    const SurfaceNodeId previous = came_from[current.node.id];
    for (const SurfaceEdge & edge : graph.adjacency()[current.node.id]) {
      if (!graph.isValid(edge.to)) {
        continue;
      }
      const SurfaceNode & neighbor = graph.nodes()[edge.to.id];
      const double clearance_penalty = options_.w_clearance *
        (std::isfinite(neighbor.clearance_m) ? 1.0 / (neighbor.clearance_m + 0.05) : 0.0);
      const double risk_penalty = options_.w_risk * neighbor.risk;
      const double slope_penalty = options_.w_slope * std::abs(edge.dz_m);
      const double bridge_penalty =
        edge.kind == SurfaceEdgeKind::TrajectoryBridge ? options_.w_bridge : 0.0;
      double turn_penalty = 0.0;
      if (graph.isValid(previous)) {
        const SurfaceNode & prev = graph.nodes()[previous.id];
        const SurfaceNode & from = graph.nodes()[current.node.id];
        const double ax = static_cast<double>(from.x - prev.x);
        const double ay = static_cast<double>(from.y - prev.y);
        const double bx = static_cast<double>(neighbor.x - from.x);
        const double by = static_cast<double>(neighbor.y - from.y);
        const double a_norm = std::hypot(ax, ay);
        const double b_norm = std::hypot(bx, by);
        if (a_norm > 1.0e-9 && b_norm > 1.0e-9) {
          const double heading_dot = std::clamp((ax * bx + ay * by) / (a_norm * b_norm), -1.0, 1.0);
          turn_penalty = options_.w_turn * (1.0 - heading_dot);
        }
      }
      const double tentative_g = current.g + edge.length_xy_m + clearance_penalty +
        risk_penalty + slope_penalty + bridge_penalty + turn_penalty;
      if (tentative_g >= g_score[edge.to.id]) {
        continue;
      }
      came_from[edge.to.id] = current.node;
      g_score[edge.to.id] = tentative_g;
      open.push({edge.to, tentative_g + heuristic(edge.to), tentative_g});
      ++result.metrics.generated_nodes;
    }
  }

  if (!found) {
    result.message = result.metrics.expanded_nodes >= options_.max_iterations ?
      "surface A* reached max_iterations" : "surface A* failed to find a path";
    result.metrics.failure_reason = result.message;
    return result;
  }

  SurfaceNodeId current = goal;
  std::vector<SurfaceNodeId> node_path;
  node_path.push_back(current);
  result.cells.push_back(graph.nodes()[current.id].cell);
  while (current != start) {
    const SurfaceNodeId parent = came_from[current.id];
    if (!graph.isValid(parent)) {
      result.message = "surface A* parent chain is incomplete";
      result.metrics.failure_reason = result.message;
      return result;
    }
    current = parent;
    node_path.push_back(current);
    result.cells.push_back(graph.nodes()[current.id].cell);
  }
  std::reverse(node_path.begin(), node_path.end());
  std::reverse(result.cells.begin(), result.cells.end());

  std::string graph_validation_failure;
  if (!validateGraphPath(graph, node_path, graph_validation_failure, result.metrics)) {
    result.message = graph_validation_failure;
    result.metrics.failure_reason = result.message;
    result.metrics.final_path_validation_failure = graph_validation_failure;
    return result;
  }

  result.raw_cells = result.cells;
  result.raw_path.reserve(result.raw_cells.size());
  for (const GridIndex & cell : result.raw_cells) {
    const SurfaceNodeId id = graph.nodeIdForCell(cell);
    const SurfaceNode * node = graph.node(id);
    if (node != nullptr) {
      result.raw_path.push_back({
        (static_cast<double>(node->x) + 0.5) * graph.resolution(),
        (static_cast<double>(node->y) + 0.5) * graph.resolution(),
        node->z});
    }
  }
  result.metrics.raw_path_waypoints = static_cast<std::uint32_t>(result.cells.size());
  for (std::size_t i = 1; i < result.cells.size(); ++i) {
    const SurfaceNode * a = graph.node(graph.nodeIdForCell(result.cells[i - 1U]));
    const SurfaceNode * b = graph.node(graph.nodeIdForCell(result.cells[i]));
    if (a != nullptr && b != nullptr) {
      const double dx = static_cast<double>(a->x - b->x) * graph.resolution();
      const double dy = static_cast<double>(a->y - b->y) * graph.resolution();
      result.metrics.raw_path_length_m += std::hypot(dx, dy);
    }
  }
  result.path = result.raw_path;
  result.path_kinds.assign(result.path.size(), PathPointKind::Surface);
  result.global_path.clear();
  result.global_path.reserve(result.path.size());
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    double yaw = 0.0;
    if (result.path.size() > 1U) {
      const Point3 & from = i + 1U < result.path.size() ? result.path[i] : result.path[i - 1U];
      const Point3 & to = i + 1U < result.path.size() ? result.path[i + 1U] : result.path[i];
      yaw = std::atan2(to.y - from.y, to.x - from.x);
    }
    int component_id = -1;
    double confidence = 1.0;
    if (i < result.cells.size()) {
      const SurfaceNodeId node_id = graph.nodeIdForCell(result.cells[i]);
      const SurfaceNode * node = graph.node(node_id);
      if (node != nullptr) {
        component_id = graph.componentId(node_id);
        confidence = node->confidence;
      }
    }
    result.global_path.push_back({
        result.path[i],
        yaw,
        PathPointKind::Surface,
        0.6,
        confidence,
        component_id});
  }
  result.metrics.final_path_validated = true;
  result.success = true;
  result.message = "path found";
  result.metrics.success = true;

  double clearance_sum = 0.0;
  result.metrics.min_path_clearance_m = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    const SurfaceNode * node = graph.node(graph.nodeIdForCell(result.cells[i]));
    if (node == nullptr) {
      continue;
    }
    const double clearance = node->clearance_m;
    if (clearance < result.metrics.min_path_clearance_m) {
      result.metrics.min_path_clearance_m = clearance;
      result.metrics.min_path_clearance_cell = result.cells[i];
      result.metrics.has_min_path_clearance_cell = true;
    }
    clearance_sum += clearance;
    result.metrics.clearance_cost_sum +=
      std::isfinite(clearance) ? 1.0 / (clearance + 0.05) : 0.0;
    result.metrics.risk_cost_sum += node->risk;
    result.metrics.max_path_risk = std::max(result.metrics.max_path_risk, node->risk);
    if (clearance < 0.30) {
      ++result.metrics.low_clearance_samples;
    }
    if (i > 0U) {
      const SurfaceNode * prev = graph.node(graph.nodeIdForCell(result.cells[i - 1U]));
      if (prev != nullptr) {
        result.metrics.path_length_m += std::hypot(
          static_cast<double>(node->x - prev->x) * graph.resolution(),
          static_cast<double>(node->y - prev->y) * graph.resolution());
      }
    }
  }
  if (!result.cells.empty()) {
    result.metrics.mean_path_clearance_m = clearance_sum / static_cast<double>(result.cells.size());
  }
  if (!std::isfinite(result.metrics.min_path_clearance_m)) {
    result.metrics.min_path_clearance_m = 0.0;
  }
  return result;
}

bool SurfaceAstarPlanner::validateGraphPath(
  const ExperienceSurfaceGraph & graph,
  const std::vector<SurfaceNodeId> & node_path,
  std::string & failure_reason,
  SurfacePlanMetrics & metrics) const
{
  for (std::size_t i = 1U; i < node_path.size(); ++i) {
    const SurfaceNodeId from = node_path[i - 1U];
    const SurfaceNodeId to = node_path[i];
    if (!graph.isValid(from) || !graph.isValid(to)) {
      failure_reason = "path_validation_failed_invalid_graph_node";
      return false;
    }
    const SurfaceEdge * path_edge = nullptr;
    for (const SurfaceEdge & edge : graph.adjacency()[from.id]) {
      if (edge.to == to) {
        path_edge = &edge;
        break;
      }
    }
    if (path_edge == nullptr) {
      failure_reason = "path_validation_failed_missing_graph_edge";
      return false;
    }
    const double abs_dz = std::abs(path_edge->dz_m);
    metrics.max_path_edge_dz_m = std::max(metrics.max_path_edge_dz_m, abs_dz);
    if (path_edge->kind == SurfaceEdgeKind::NormalSurface &&
      abs_dz > options_.max_step_height_m)
    {
      ++metrics.path_layer_jump_edges;
      failure_reason = "path_validation_failed_layer_jump_edge";
      return false;
    }
    const SurfaceNode * from_node = graph.node(from);
    const SurfaceNode * to_node = graph.node(to);
    if (from_node == nullptr || to_node == nullptr) {
      failure_reason = "path_validation_failed_invalid_graph_node";
      return false;
    }
    if (path_edge->kind == SurfaceEdgeKind::NormalSurface) {
      if (from_node->surface_layer_id < 0 || to_node->surface_layer_id < 0)
      {
        failure_reason = "path_validation_failed_missing_surface_layer";
        return false;
      }
      if (from_node->surface_layer_id != to_node->surface_layer_id) {
        failure_reason = "path_validation_failed_cross_surface_layer_edge";
        return false;
      }
    }
    if (path_edge->kind == SurfaceEdgeKind::TrajectoryBridge) {
      if (!from_node->bridge && !to_node->bridge) {
        failure_reason = "path_validation_failed_invalid_bridge_edge";
        return false;
      }
      if (from_node->bridge && to_node->bridge) {
        if (from_node->bridge_id < 0 || from_node->bridge_id != to_node->bridge_id ||
          std::abs(from_node->bridge_order - to_node->bridge_order) > 1)
        {
          failure_reason = "path_validation_failed_invalid_bridge_edge";
          return false;
        }
      } else {
        const SurfaceNode * bridge_node = from_node->bridge ? from_node : to_node;
        const SurfaceNode * normal_node = from_node->bridge ? to_node : from_node;
        if (!bridge_node->bridge_endpoint || normal_node->surface_layer_id < 0) {
          failure_reason = "path_validation_failed_invalid_bridge_edge";
          return false;
        }
      }
      if (abs_dz > 0.0 && !std::isfinite(abs_dz)) {
        failure_reason = "path_validation_failed_invalid_bridge_edge";
        return false;
      }
    }
  }
  return true;
}

double SurfaceAstarPlanner::transitionCost(
  const NavigationSnapshot & snapshot, const GridIndex & from, const GridIndex & to,
  const GridIndex * previous) const
{
  const double step = gridDistance(from, to) * snapshot.resolution_m;
  const double clearance_penalty = options_.w_clearance * snapshot.clearance.clearancePenalty(to);
  const double risk_penalty = options_.w_risk * snapshot.risk.riskCost(to);
  const double unknown_penalty = options_.w_unknown * unknownPenalty(snapshot, to);
  double bridge_penalty = 0.0;
  double slope_penalty = 0.0;
  const auto from_surface = snapshot.surface.surface_cells.find(from);
  const auto to_surface = snapshot.surface.surface_cells.find(to);
  if (from_surface != snapshot.surface.surface_cells.end() &&
    to_surface != snapshot.surface.surface_cells.end())
  {
    slope_penalty =
      options_.w_slope * std::abs(to_surface->second.height_m - from_surface->second.height_m);
    if (to_surface->second.label == SurfaceLabel::TrajectoryBridge ||
      to_surface->second.reachability == ReachabilityLabel::LowConfidenceReachable)
    {
      bridge_penalty = options_.w_bridge;
    }
  }

  double turn_penalty = 0.0;
  if (previous != nullptr) {
    const double ax = static_cast<double>(from.x - previous->x);
    const double ay = static_cast<double>(from.y - previous->y);
    const double bx = static_cast<double>(to.x - from.x);
    const double by = static_cast<double>(to.y - from.y);
    const double a_norm = std::hypot(ax, ay);
    const double b_norm = std::hypot(bx, by);
    if (a_norm > 1.0e-9 && b_norm > 1.0e-9) {
      const double heading_dot = std::clamp((ax * bx + ay * by) / (a_norm * b_norm), -1.0, 1.0);
      turn_penalty = options_.w_turn * (1.0 - heading_dot);
    }
  }

  return step + clearance_penalty + risk_penalty + unknown_penalty + bridge_penalty +
         slope_penalty + turn_penalty;
}

double SurfaceAstarPlanner::unknownPenalty(
  const NavigationSnapshot & snapshot, const GridIndex & cell) const
{
  if (snapshot.observed_free_cells.empty()) {
    return 0.0;
  }
  return snapshot.observed_free_cells.find(cell) == snapshot.observed_free_cells.end() ? 1.0 : 0.0;
}

Point3 SurfaceAstarPlanner::cellCenter(const GridIndex & cell, double resolution_m) const
{
  return {
    (static_cast<double>(cell.x) + 0.5) * resolution_m,
    (static_cast<double>(cell.y) + 0.5) * resolution_m,
    (static_cast<double>(cell.z) + 0.5) * resolution_m};
}

GridIndex SurfaceAstarPlanner::worldToGrid(const Point3 & point, double resolution_m) const
{
  return {
    static_cast<int>(std::floor(point.x / resolution_m)),
    static_cast<int>(std::floor(point.y / resolution_m)),
    static_cast<int>(std::floor(point.z / resolution_m))};
}

std::vector<Point3> SurfaceAstarPlanner::cellsToPath(
  const std::vector<GridIndex> & cells, double resolution_m) const
{
  std::vector<Point3> path;
  path.reserve(cells.size());
  for (const GridIndex & cell : cells) {
    path.push_back(cellCenter(cell, resolution_m));
  }
  return path;
}

std::vector<GridIndex> SurfaceAstarPlanner::shortcutPath(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells) const
{
  if (raw_cells.size() < 3U) {
    return raw_cells;
  }

  std::vector<GridIndex> simplified;
  simplified.reserve(raw_cells.size());
  std::size_t i = 0U;
  simplified.push_back(raw_cells.front());
  while (i + 1U < raw_cells.size()) {
    std::size_t best = i + 1U;
    for (std::size_t candidate = raw_cells.size() - 1U; candidate > i + 1U; --candidate) {
      if (isShortcutAllowed(snapshot, raw_cells, i, candidate)) {
        best = candidate;
        break;
      }
    }
    simplified.push_back(raw_cells[best]);
    i = best;
  }
  return simplified;
}

bool SurfaceAstarPlanner::isShortcutAllowed(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
  std::size_t from_index, std::size_t to_index) const
{
  const Point3 from = cellCenter(raw_cells[from_index], snapshot.resolution_m);
  const Point3 to = cellCenter(raw_cells[to_index], snapshot.resolution_m);
  const double segment_length = distance3d(from, to);
  if (segment_length <= snapshot.resolution_m) {
    return true;
  }

  const double raw_min_clearance = minRawClearance(snapshot, raw_cells, from_index, to_index);
  const double required_clearance = std::max(
    raw_min_clearance * options_.shortcut_clearance_ratio,
    options_.footprint.width_m * 0.5 + options_.shortcut_safety_margin_m);
  const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / options_.shortcut_sample_step_m)));
  double shortcut_min_clearance = std::numeric_limits<double>::infinity();
  GridIndex previous_cell = raw_cells[from_index];
  for (int step = 0; step <= steps; ++step) {
    const double t = static_cast<double>(step) / static_cast<double>(steps);
    const Point3 sample{
      from.x + (to.x - from.x) * t,
      from.y + (to.y - from.y) * t,
      from.z + (to.z - from.z) * t};
    const GridIndex sample_cell = worldToGrid(sample, snapshot.resolution_m);
    if (!transition_validator_.isCellTraversable(snapshot, sample_cell)) {
      return false;
    }
    if (step > 0 && !transition_validator_.validate(snapshot, previous_cell, sample_cell).allowed) {
      return false;
    }
    shortcut_min_clearance =
      std::min(shortcut_min_clearance, snapshot.clearance.clearanceDistance(sample_cell));
    previous_cell = sample_cell;
  }
  return shortcut_min_clearance >= required_clearance;
}

double SurfaceAstarPlanner::minRawClearance(
  const NavigationSnapshot & snapshot, const std::vector<GridIndex> & raw_cells,
  std::size_t from_index, std::size_t to_index) const
{
  double min_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = from_index; i <= to_index; ++i) {
    min_clearance = std::min(min_clearance, snapshot.clearance.clearanceDistance(raw_cells[i]));
  }
  return std::isfinite(min_clearance) ? min_clearance : 0.0;
}

bool SurfaceAstarPlanner::validatePath(
  const NavigationSnapshot & snapshot, const std::vector<Point3> & path,
  std::string & failure_reason) const
{
  if (path.empty()) {
    failure_reason = "final path is empty";
    return false;
  }

  if (path.size() == 1U) {
    const GridIndex cell = worldToGrid(path.front(), snapshot.resolution_m);
    if (!transition_validator_.isCellTraversable(snapshot, cell)) {
      failure_reason = "final path sample is not traversable";
      return false;
    }
    if (!hasRequiredFinalClearance(snapshot, cell, failure_reason)) {
      return false;
    }
    if (!transition_validator_.isEndpointCell(snapshot, cell)) {
      failure_reason = "final path footprint is not fully supported";
      return false;
    }
    failure_reason.clear();
    return true;
  }

  for (std::size_t i = 1; i < path.size(); ++i) {
    const Point3 & from = path[i - 1U];
    const Point3 & to = path[i];
    const GridIndex from_cell = worldToGrid(from, snapshot.resolution_m);
    const GridIndex to_cell = worldToGrid(to, snapshot.resolution_m);
    const auto direct_report = transition_validator_.validate(snapshot, from_cell, to_cell);
    if (direct_report.allowed) {
      if (i == 1U && !hasRequiredFinalClearance(snapshot, from_cell, failure_reason)) {
        return false;
      }
      if (!hasRequiredFinalClearance(snapshot, to_cell, failure_reason)) {
        return false;
      }
      continue;
    }

    const double segment_length = distance3d(from, to);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(segment_length / options_.shortcut_sample_step_m)));
    GridIndex previous_cell = worldToGrid(from, snapshot.resolution_m);
    for (int step = 0; step <= steps; ++step) {
      if (i > 1U && step == 0) {
        continue;
      }
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const Point3 sample{
        from.x + (to.x - from.x) * t,
        from.y + (to.y - from.y) * t,
        from.z + (to.z - from.z) * t};
      const GridIndex sample_cell = worldToGrid(sample, snapshot.resolution_m);
      if (!transition_validator_.isCellTraversable(snapshot, sample_cell)) {
        failure_reason = "final path sample is not traversable segment=" +
          std::to_string(i) + " step=" + std::to_string(step) +
          " from=" + cellString(from_cell) + " to=" + cellString(to_cell) +
          " sample=" + cellString(sample_cell) +
          " delta=[" + std::to_string(to_cell.x - from_cell.x) + "," +
          std::to_string(to_cell.y - from_cell.y) + "," +
          std::to_string(to_cell.z - from_cell.z) + "]";
        return false;
      }
      if (!hasRequiredFinalClearance(snapshot, sample_cell, failure_reason)) {
        return false;
      }
      if (step > 0 && !transition_validator_.validate(snapshot, previous_cell, sample_cell).allowed) {
        failure_reason = "final path transition is not allowed segment=" +
          std::to_string(i) + " step=" + std::to_string(step) +
          " previous=" + cellString(previous_cell) + " sample=" + cellString(sample_cell);
        return false;
      }
      previous_cell = sample_cell;
    }
  }

  failure_reason.clear();
  return true;
}

bool SurfaceAstarPlanner::hasRequiredFinalClearance(
  const NavigationSnapshot & snapshot, const GridIndex & cell, std::string & failure_reason) const
{
  if (options_.final_validation_min_clearance_m <= 0.0) {
    return true;
  }
  const double clearance = snapshot.clearance.clearanceDistance(cell);
  if (clearance >= options_.final_validation_min_clearance_m) {
    return true;
  }
  failure_reason = "final path clearance below minimum cell=" + cellString(cell);
  return false;
}

void SurfaceAstarPlanner::fillMetrics(
  const NavigationSnapshot & snapshot, SurfacePlanResult & result) const
{
  if (result.cells.empty()) {
    return;
  }
  double clearance_sum = 0.0;
  result.metrics.min_path_clearance_m = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    const double clearance = snapshot.clearance.clearanceDistance(result.cells[i]);
    if (clearance < result.metrics.min_path_clearance_m) {
      result.metrics.min_path_clearance_m = clearance;
      result.metrics.min_path_clearance_cell = result.cells[i];
      result.metrics.has_min_path_clearance_cell = true;
    }
    clearance_sum += clearance;
    result.metrics.clearance_cost_sum += snapshot.clearance.clearancePenalty(result.cells[i]);
    result.metrics.unknown_cost_sum += unknownPenalty(snapshot, result.cells[i]);
    const double risk = snapshot.risk.riskCost(result.cells[i]);
    result.metrics.risk_cost_sum += risk;
    result.metrics.max_path_risk = std::max(result.metrics.max_path_risk, risk);
    if (clearance < 0.30) {
      ++result.metrics.low_clearance_samples;
    }
    if (i > 0U) {
      result.metrics.path_length_m += gridDistance(result.cells[i - 1U], result.cells[i]) *
        snapshot.resolution_m;
    }
  }
  result.metrics.mean_path_clearance_m = clearance_sum / static_cast<double>(result.cells.size());
  if (!std::isfinite(result.metrics.min_path_clearance_m)) {
    result.metrics.min_path_clearance_m = 0.0;
  }
}

}  // namespace tgw_planner::core
