#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace
{
using tgw_planner::core::BackboneEdge;
using tgw_planner::core::ExperienceBackboneGraph;
using tgw_planner::core::ExperienceBackboneOptions;
using tgw_planner::core::ExperienceBuildResult;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::ExperienceSurfaceGraph;
using tgw_planner::core::GridIndex;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::HybridExperiencePlannerOptions;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::Point3;
using tgw_planner::core::SurfaceEdge;
using tgw_planner::core::SurfaceGraphBuildOptions;
using tgw_planner::core::SurfaceNode;
using tgw_planner::core::SurfaceNodeId;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfacePlanResult;
using tgw_planner::core::SurfaceTransitionValidator;
using tgw_planner::core::TrajectoryProjectionResult;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::TrajectoryProjectorOptions;

struct SweepOptions
{
  std::string pbstream_path;
  bool auto_bands{true};
  double low_z_max{0.0};
  double high_z_min{0.0};
  std::size_t sample_pairs{50};
};

struct HybridConnectivity
{
  std::vector<int> component;
  std::size_t component_count{0};
  std::size_t surface_edges{0};
  std::size_t backbone_edges{0};
  std::size_t portal_edges{0};
};

double parseDouble(const char * text)
{
  char * end = nullptr;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    throw std::runtime_error(std::string("invalid number: ") + text);
  }
  return value;
}

std::size_t parseSize(const char * text)
{
  char * end = nullptr;
  const unsigned long value = std::strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    throw std::runtime_error(std::string("invalid integer: ") + text);
  }
  return static_cast<std::size_t>(value);
}

SweepOptions parseArgs(int argc, char ** argv)
{
  if (argc < 2 || argc > 5) {
    throw std::runtime_error(
      "usage: tgw_experience_global_sweep <n3map.pbstream> [low_z_max high_z_min [sample_pairs]]");
  }
  SweepOptions options;
  options.pbstream_path = argv[1];
  if (argc >= 4) {
    options.auto_bands = false;
    options.low_z_max = parseDouble(argv[2]);
    options.high_z_min = parseDouble(argv[3]);
  }
  if (argc == 5) {
    options.sample_pairs = parseSize(argv[4]);
  }
  return options;
}

TrajectoryProjectorOptions projectorOptions()
{
  TrajectoryProjectorOptions options;
  options.resolution_m = 0.10;
  options.raw_resolution_m = 0.05;
  options.search_below_min_m = 0.10;
  options.search_below_max_m = 1.00;
  options.max_support_jump_m = 0.35;
  options.allow_support_reanchor_on_jump = true;
  options.support_xy_search_radius_cells = 2;
  options.support_xy_retry_radius_cells = 8;
  options.max_trajectory_bridge_gap_m = 2.00;
  options.max_trajectory_bridge_height_delta_m = 0.80;
  options.trajectory_bridge_sample_step_m = 0.10;
  options.footprint_length_m = 0.70;
  options.footprint_width_m = 0.43;
  options.footprint_base_to_front_m = 0.20;
  options.min_footprint_support_ratio = 0.50;
  options.footprint_support_height_tolerance_m = 0.20;
  return options;
}

ExperienceSurfaceBuilderOptions builderOptions()
{
  ExperienceSurfaceBuilderOptions options;
  options.resolution_m = 0.10;
  options.body_clearance_height_m = 0.65;
  options.projector = projectorOptions();
  options.expander.expansion_radius_cells = 2;
  options.expander.max_expansion_steps = 12;
  options.expander.vertical_tolerance_cells = 3;
  options.expander.max_expansion_step_height_m = 0.28;
  options.expander.experience_anchor_radius_cells = 24;
  options.expander.experience_anchor_height_tolerance_m = 0.35;
  options.expander.experience_anchor_vertical_tolerance_cells = 3;
  options.expander.enable_hole_filling = true;
  options.expander.hole_fill_iterations = 2;
  options.expander.min_hole_fill_neighbors = 5;
  options.expander.max_hole_fill_height_spread_m = 0.12;
  return options;
}

SurfacePlannerOptions plannerOptions()
{
  SurfacePlannerOptions options;
  options.max_step_height_m = 0.35;
  options.max_iterations = 250000;
  options.w_bridge = 2.5;
  options.footprint.length_m = 0.70;
  options.footprint.width_m = 0.43;
  options.footprint.base_to_front_m = 0.20;
  options.footprint.height_m = 0.65;
  options.footprint.support_height_tolerance_m = 0.20;
  options.footprint.min_support_ratio = 0.80;
  options.require_footprint_support = true;
  options.enable_shortcut = true;
  return options;
}

SurfaceGraphBuildOptions graphOptions()
{
  SurfaceGraphBuildOptions options;
  options.max_edge_height_delta_m = 0.35;
  options.max_bridge_edge_height_delta_m = 0.80;
  options.max_bridge_attach_height_delta_m = 0.35;
  options.max_edge_slope = 3.0;
  options.max_bridge_edge_slope = 8.0;
  return options;
}

ExperienceBackboneOptions backboneOptions()
{
  ExperienceBackboneOptions options;
  options.min_node_spacing_m = 0.20;
  options.max_portal_xy_distance_m = 1.20;
  options.max_portal_height_error_m = 0.45;
  options.min_portal_clearance_m = 0.0;
  return options;
}

HybridExperiencePlannerOptions hybridOptions()
{
  HybridExperiencePlannerOptions options;
  options.backbone_cost_scale = 1.2;
  options.portal_switch_cost = 0.5;
  options.portal_height_error_weight = 0.25;
  options.backbone_low_confidence_penalty = 0.5;
  return options;
}

void chooseAutoBands(
  const ExperienceSurfaceGraph & graph,
  double * low_z_max,
  double * high_z_min)
{
  std::vector<double> heights;
  heights.reserve(graph.nodes().size());
  for (const SurfaceNode & node : graph.nodes()) {
    heights.push_back(node.z);
  }
  if (heights.empty()) {
    *low_z_max = 0.0;
    *high_z_min = 0.0;
    return;
  }
  std::sort(heights.begin(), heights.end());
  const std::size_t low_index = heights.size() / 5U;
  const std::size_t high_index = (heights.size() * 4U) / 5U;
  *low_z_max = heights[low_index];
  *high_z_min = heights[std::min(high_index, heights.size() - 1U)];
}

HybridConnectivity buildHybridConnectivity(
  const ExperienceSurfaceGraph & surface_graph,
  const ExperienceBackboneGraph & backbone_graph)
{
  HybridConnectivity out;
  const std::size_t surface_count = surface_graph.nodes().size();
  const std::size_t backbone_count = backbone_graph.nodes().size();
  const std::size_t total = surface_count + backbone_count;
  std::vector<std::vector<std::uint32_t>> adjacency(total);
  std::vector<std::vector<std::uint32_t>> reverse_adjacency(total);

  const auto add_edge = [&](std::uint32_t from, std::uint32_t to) {
      if (from >= total || to >= total) {
        return;
      }
      adjacency[from].push_back(to);
      reverse_adjacency[to].push_back(from);
    };

  for (const SurfaceNode & node : surface_graph.nodes()) {
    for (const SurfaceEdge & edge : surface_graph.adjacency()[node.id.id]) {
      if (!surface_graph.isValid(edge.to)) {
        continue;
      }
      add_edge(node.id.id, edge.to.id);
      ++out.surface_edges;
    }
  }
  for (const BackboneEdge & edge : backbone_graph.edges()) {
    const std::uint32_t from = static_cast<std::uint32_t>(surface_count + edge.from.id);
    const std::uint32_t to = static_cast<std::uint32_t>(surface_count + edge.to.id);
    if (from >= total || to >= total) {
      continue;
    }
    add_edge(from, to);
    add_edge(to, from);
    out.backbone_edges += 2U;
  }
  for (const auto & portal : backbone_graph.portals()) {
    if (!surface_graph.isValid(portal.surface_node) ||
      !backbone_graph.isValid(portal.backbone_node))
    {
      continue;
    }
    const std::uint32_t surface = portal.surface_node.id;
    const std::uint32_t backbone = static_cast<std::uint32_t>(surface_count + portal.backbone_node.id);
    if (surface >= total || backbone >= total) {
      continue;
    }
    add_edge(surface, backbone);
    add_edge(backbone, surface);
    out.portal_edges += 2U;
  }

  out.component.assign(total, -1);
  std::vector<char> visited(total, 0);
  std::vector<std::uint32_t> order;
  order.reserve(total);
  for (std::uint32_t seed = 0; seed < total; ++seed) {
    if (visited[seed]) {
      continue;
    }
    visited[seed] = 1;
    std::vector<std::pair<std::uint32_t, std::size_t>> stack;
    stack.push_back({seed, 0U});
    while (!stack.empty()) {
      auto & frame = stack.back();
      const std::uint32_t current = frame.first;
      if (frame.second < adjacency[current].size()) {
        const std::uint32_t next = adjacency[current][frame.second++];
        if (next >= total || visited[next]) {
          continue;
        }
        visited[next] = 1;
        stack.push_back({next, 0U});
      } else {
        order.push_back(current);
        stack.pop_back();
      }
    }
  }

  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    const std::uint32_t seed = *it;
    if (out.component[seed] >= 0) {
      continue;
    }
    const int component_id = static_cast<int>(out.component_count++);
    out.component[seed] = component_id;
    std::vector<std::uint32_t> stack;
    stack.push_back(seed);
    while (!stack.empty()) {
      const std::uint32_t current = stack.back();
      stack.pop_back();
      for (const std::uint32_t next : reverse_adjacency[current]) {
        if (next >= total || out.component[next] >= 0) {
          continue;
        }
        out.component[next] = component_id;
        stack.push_back(next);
      }
    }
  }
  return out;
}

std::unordered_map<int, std::size_t> countComponents(
  const std::vector<SurfaceNodeId> & nodes,
  const HybridConnectivity & connectivity)
{
  std::unordered_map<int, std::size_t> counts;
  for (const SurfaceNodeId node : nodes) {
    if (node.id < connectivity.component.size()) {
      ++counts[connectivity.component[node.id]];
    }
  }
  return counts;
}

int largestSharedComponent(
  const std::unordered_map<int, std::size_t> & low_components,
  const std::unordered_map<int, std::size_t> & high_components)
{
  int best_component = -1;
  std::size_t best_score = 0U;
  for (const auto & low_entry : low_components) {
    const auto high_it = high_components.find(low_entry.first);
    if (high_it == high_components.end()) {
      continue;
    }
    const std::size_t score = std::min(low_entry.second, high_it->second);
    if (score > best_score) {
      best_score = score;
      best_component = low_entry.first;
    }
  }
  return best_component;
}

std::vector<SurfaceNodeId> filterNodesByComponent(
  const std::vector<SurfaceNodeId> & nodes,
  const HybridConnectivity & connectivity,
  int component_id)
{
  std::vector<SurfaceNodeId> out;
  out.reserve(nodes.size());
  for (const SurfaceNodeId node : nodes) {
    if (node.id < connectivity.component.size() &&
      connectivity.component[node.id] == component_id)
    {
      out.push_back(node);
    }
  }
  return out;
}

std::vector<SurfaceNodeId> sampleNodes(const std::vector<SurfaceNodeId> & nodes, std::size_t max_count)
{
  if (nodes.size() <= max_count || max_count == 0U) {
    return nodes;
  }
  std::vector<SurfaceNodeId> out;
  out.reserve(max_count);
  for (std::size_t i = 0; i < max_count; ++i) {
    const std::size_t index = (i * nodes.size()) / max_count;
    out.push_back(nodes[std::min(index, nodes.size() - 1U)]);
  }
  return out;
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
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const SweepOptions sweep_options = parseArgs(argc, argv);
    const auto start_time = std::chrono::steady_clock::now();

    N3MapReader reader;
    const auto read_result = reader.readPbstream(sweep_options.pbstream_path);
    if (!read_result.success) {
      std::cerr << "read_failed error_code=" << read_result.error_code <<
        " message=" << read_result.message << std::endl;
      return 2;
    }

    const TrajectoryProjectionResult projection =
      TrajectoryProjector(projectorOptions()).project(read_result.resource);
    const ExperienceBuildResult build =
      ExperienceSurfaceBuilder(builderOptions()).build(read_result.resource);
    if (!build.success) {
      std::cerr << "build_failed error_code=" << build.error_code <<
        " message=" << build.message << std::endl;
      return 3;
    }

    const SurfacePlannerOptions surface_options = plannerOptions();
    ExperienceSurfaceGraph surface_graph;
    surface_graph.build(
      build.snapshot, SurfaceTransitionValidator(surface_options), graphOptions());

    ExperienceBackboneGraph backbone_graph;
    backbone_graph.build(read_result.resource, projection, surface_graph, backboneOptions());

    double low_z_max = sweep_options.low_z_max;
    double high_z_min = sweep_options.high_z_min;
    if (sweep_options.auto_bands) {
      chooseAutoBands(surface_graph, &low_z_max, &high_z_min);
    }

    std::vector<SurfaceNodeId> low_nodes;
    std::vector<SurfaceNodeId> high_nodes;
    low_nodes.reserve(surface_graph.nodes().size());
    high_nodes.reserve(surface_graph.nodes().size());
    for (const SurfaceNode & node : surface_graph.nodes()) {
      if (node.z <= low_z_max) {
        low_nodes.push_back(node.id);
      }
      if (node.z >= high_z_min) {
        high_nodes.push_back(node.id);
      }
    }

    const HybridConnectivity connectivity = buildHybridConnectivity(surface_graph, backbone_graph);
    const auto low_components = countComponents(low_nodes, connectivity);
    const auto high_components = countComponents(high_nodes, connectivity);
    std::size_t cross_connected_low = 0U;
    std::size_t cross_connected_high = 0U;
    for (const auto & entry : low_components) {
      if (high_components.find(entry.first) != high_components.end()) {
        cross_connected_low += entry.second;
      }
    }
    for (const auto & entry : high_components) {
      if (low_components.find(entry.first) != low_components.end()) {
        cross_connected_high += entry.second;
      }
    }
    const bool all_cross_connected =
      !low_nodes.empty() && !high_nodes.empty() &&
      cross_connected_low == low_nodes.size() &&
      cross_connected_high == high_nodes.size() &&
      low_components.size() == 1U &&
      high_components.size() == 1U &&
      low_components.begin()->first == high_components.begin()->first;

    const int dominant_shared_component =
      largestSharedComponent(low_components, high_components);
    const std::vector<SurfaceNodeId> dominant_low_nodes =
      filterNodesByComponent(low_nodes, connectivity, dominant_shared_component);
    const std::vector<SurfaceNodeId> dominant_high_nodes =
      filterNodesByComponent(high_nodes, connectivity, dominant_shared_component);
    const double dominant_low_coverage =
      low_nodes.empty() ? 0.0 :
      static_cast<double>(dominant_low_nodes.size()) / static_cast<double>(low_nodes.size());
    const double dominant_high_coverage =
      high_nodes.empty() ? 0.0 :
      static_cast<double>(dominant_high_nodes.size()) / static_cast<double>(high_nodes.size());

    HybridExperiencePlanner planner(surface_options, hybridOptions());
    const std::vector<SurfaceNodeId> sampled_low =
      sampleNodes(dominant_low_nodes, sweep_options.sample_pairs);
    const std::vector<SurfaceNodeId> sampled_high =
      sampleNodes(dominant_high_nodes, sweep_options.sample_pairs);
    std::size_t sampled_queries = 0U;
    std::size_t sampled_success = 0U;
    std::string first_failure;
    std::size_t first_failure_index = 0U;
    for (std::size_t i = 0; i < sampled_low.size() && i < sampled_high.size(); ++i) {
      const SurfacePlanResult plan = planner.plan(surface_graph, backbone_graph, sampled_low[i], sampled_high[i]);
      ++sampled_queries;
      if (plan.success) {
        ++sampled_success;
      } else if (first_failure.empty()) {
        first_failure_index = i;
        const Point3 low = surfacePoint(surface_graph, sampled_low[i]);
        const Point3 high = surfacePoint(surface_graph, sampled_high[i]);
        first_failure = plan.message + " low=(" + std::to_string(low.x) + "," +
          std::to_string(low.y) + "," + std::to_string(low.z) + ") high=(" +
          std::to_string(high.x) + "," + std::to_string(high.y) + "," +
          std::to_string(high.z) + ") low_node=" + std::to_string(sampled_low[i].id) +
          " high_node=" + std::to_string(sampled_high[i].id) +
          " low_hybrid_component=" +
          std::to_string(connectivity.component[sampled_low[i].id]) +
          " high_hybrid_component=" +
          std::to_string(connectivity.component[sampled_high[i].id]) +
          " low_surface_component=" +
          std::to_string(surface_graph.componentId(sampled_low[i])) +
          " high_surface_component=" +
          std::to_string(surface_graph.componentId(sampled_high[i])) +
          " hybrid_expanded=" +
          std::to_string(plan.metrics.hybrid_expanded_nodes) +
          " generated=" + std::to_string(plan.metrics.generated_nodes) +
          " start_portal_candidates=" +
          std::to_string(plan.metrics.start_portal_candidates) +
          " goal_portal_candidates=" +
          std::to_string(plan.metrics.goal_portal_candidates);
      }
    }
    const bool main_component_sweep_success =
      dominant_shared_component >= 0 &&
      !dominant_low_nodes.empty() &&
      !dominant_high_nodes.empty() &&
      sampled_queries > 0U &&
      sampled_success == sampled_queries;

    const auto end_time = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::cout << "global_sweep_success=" <<
      (all_cross_connected && main_component_sweep_success ? "true" : "false") << "\n";
    std::cout << "pbstream=" << sweep_options.pbstream_path << "\n";
    std::cout << "low_z_max=" << low_z_max << " high_z_min=" << high_z_min <<
      " auto_bands=" << (sweep_options.auto_bands ? "true" : "false") << "\n";
    std::cout << "surface_nodes=" << surface_graph.nodes().size() <<
      " surface_components=" << surface_graph.componentCount() <<
      " low_nodes=" << low_nodes.size() <<
      " high_nodes=" << high_nodes.size() << "\n";
    std::cout << "backbone_nodes=" << backbone_graph.nodes().size() <<
      " backbone_edges=" << backbone_graph.edges().size() <<
      " portals=" << backbone_graph.portals().size() << "\n";
    std::cout << "hybrid_components=" << connectivity.component_count <<
      " hybrid_surface_edges=" << connectivity.surface_edges <<
      " hybrid_backbone_edges=" << connectivity.backbone_edges <<
      " hybrid_portal_edges=" << connectivity.portal_edges << "\n";
    std::cout << "low_components=" << low_components.size() <<
      " high_components=" << high_components.size() <<
      " cross_connected_low=" << cross_connected_low <<
      " cross_connected_high=" << cross_connected_high <<
      " all_low_high_connected=" << (all_cross_connected ? "true" : "false") << "\n";
    std::cout << "dominant_shared_component=" << dominant_shared_component <<
      " dominant_low_nodes=" << dominant_low_nodes.size() <<
      " dominant_high_nodes=" << dominant_high_nodes.size() <<
      " dominant_low_coverage=" << dominant_low_coverage <<
      " dominant_high_coverage=" << dominant_high_coverage <<
      " main_component_sweep_success=" << (main_component_sweep_success ? "true" : "false") << "\n";
    std::cout << "sampled_queries=" << sampled_queries <<
      " sampled_success=" << sampled_success << "\n";
    if (!first_failure.empty()) {
      std::cout << "first_sample_failure_index=" << first_failure_index << "\n";
      std::cout << "first_sample_failure=" << first_failure << "\n";
    }
    std::cout << "elapsed_ms=" << elapsed_ms << "\n";

    return all_cross_connected && main_component_sweep_success ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << std::endl;
    return 2;
  }
}
