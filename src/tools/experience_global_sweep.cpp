#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/experience_planner_defaults.hpp"
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
using tgw_planner::core::ExperienceGeometryIndex;
using tgw_planner::core::ExperienceGeometryIndexBuildResult;
using tgw_planner::core::ExperienceGeometryIndexOptions;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::ExperienceSurfaceGraph;
using tgw_planner::core::GridIndex;
using tgw_planner::core::GlobalPathPoint;
using tgw_planner::core::HybridExperienceGraph;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::HybridExperiencePlannerOptions;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::PathPointKind;
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
using tgw_planner::core::defaultExperienceBackboneOptions;
using tgw_planner::core::defaultExperienceSurfaceBuilderOptions;
using tgw_planner::core::defaultHybridExperiencePlannerOptions;
using tgw_planner::core::defaultSurfaceGraphBuildOptions;
using tgw_planner::core::defaultSurfacePlannerOptions;
using tgw_planner::core::defaultTrajectoryProjectorOptions;
using tgw_planner::core::isBackboneEdgeAllowedForHybrid;

enum class QueryMode
{
  LowHigh,
  SameBand
};

struct SweepOptions
{
  std::string pbstream_path;
  bool auto_bands{true};
  double low_z_max{0.0};
  double high_z_min{0.0};
  bool has_same_band_bounds{false};
  bool has_same_z_min{false};
  bool has_same_z_max{false};
  double same_z_min{0.0};
  double same_z_max{0.0};
  std::size_t sample_pairs{50};
  bool strict_all{false};
  QueryMode query_mode{QueryMode::LowHigh};
  std::string export_jsonl_path;
};

struct HybridConnectivity
{
  std::vector<int> component;
  std::size_t component_count{0};
  std::size_t surface_edges{0};
  std::size_t backbone_edges{0};
  std::size_t portal_edges{0};
};

struct QualityStats
{
  std::size_t successful_queries{0};
  std::size_t suspicious_detour_count{0};
  double detour_sum{0.0};
  double max_detour_ratio{0.0};
  double backbone_ratio_sum{0.0};
  double max_backbone_ratio{0.0};
  std::uint32_t max_portal_switch_count{0};
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

std::string jsonEscape(const std::string & text)
{
  std::string out;
  out.reserve(text.size());
  for (const char ch : text) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += ch;
        break;
    }
  }
  return out;
}

const char * pathKindName(PathPointKind kind)
{
  switch (kind) {
    case PathPointKind::Surface:
      return "surface";
    case PathPointKind::Backbone:
      return "backbone";
    case PathPointKind::Portal:
      return "portal";
    case PathPointKind::Unknown:
    default:
      return "unknown";
  }
}

const char * queryModeName(QueryMode mode)
{
  switch (mode) {
    case QueryMode::LowHigh:
      return "low-high";
    case QueryMode::SameBand:
      return "same-band";
    default:
      return "unknown";
  }
}

double xyDistance(const Point3 & a, const Point3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pathLength(const std::vector<Point3> & path)
{
  double length = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    length += xyDistance(path[i - 1U], path[i]);
  }
  return length;
}

void writePointJson(std::ostream & out, const Point3 & point)
{
  out << "{\"x\":" << point.x << ",\"y\":" << point.y << ",\"z\":" << point.z << "}";
}

void writePointArrayJson(std::ostream & out, const std::vector<Point3> & points)
{
  out << "[";
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    writePointJson(out, points[i]);
  }
  out << "]";
}

void writePathJson(
  std::ostream & out,
  const std::vector<Point3> & points,
  const std::vector<PathPointKind> & kinds)
{
  out << "[";
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    const PathPointKind kind = i < kinds.size() ? kinds[i] : PathPointKind::Surface;
    out << "{\"x\":" << points[i].x <<
      ",\"y\":" << points[i].y <<
      ",\"z\":" << points[i].z <<
      ",\"kind\":\"" << pathKindName(kind) << "\"}";
  }
  out << "]";
}

void writeGlobalPathJson(std::ostream & out, const SurfacePlanResult & plan)
{
  if (plan.global_path.empty()) {
    writePathJson(out, plan.path, plan.path_kinds);
    return;
  }
  out << "[";
  for (std::size_t i = 0U; i < plan.global_path.size(); ++i) {
    if (i > 0U) {
      out << ",";
    }
    const GlobalPathPoint & point = plan.global_path[i];
    out << "{\"x\":" << point.position.x <<
      ",\"y\":" << point.position.y <<
      ",\"z\":" << point.position.z <<
      ",\"kind\":\"" << pathKindName(point.kind) <<
      "\",\"yaw_hint_rad\":" << point.yaw_hint_rad <<
      ",\"target_speed_mps\":" << point.target_speed_mps <<
      ",\"confidence\":" << point.confidence <<
      ",\"surface_component_id\":" << point.surface_component_id << "}";
  }
  out << "]";
}

void writeQueryJsonl(
  std::ostream & out,
  std::size_t query_id,
  const Point3 & start,
  const Point3 & goal,
  const SurfacePlanResult & plan,
  const ExperienceBackboneGraph & backbone_graph,
  double xy_distance,
  double detour_ratio,
  double backbone_ratio)
{
  out << std::setprecision(10);
  out << "{\"query_id\":" << query_id <<
    ",\"success\":" << (plan.success ? "true" : "false") <<
    ",\"failure_reason\":\"" << jsonEscape(plan.success ? "" : plan.message) << "\"";
  out << ",\"start\":";
  writePointJson(out, start);
  out << ",\"goal\":";
  writePointJson(out, goal);
  out << ",\"snapped_start\":";
  writePointJson(out, start);
  out << ",\"snapped_goal\":";
  writePointJson(out, goal);
  out << ",\"path\":";
  writeGlobalPathJson(out, plan);
  out << ",\"used_backbone\":";
  writePointArrayJson(out, plan.debug_selected_backbone_segment);
  std::vector<Point3> backbone_points;
  backbone_points.reserve(backbone_graph.nodes().size());
  for (const auto & node : backbone_graph.nodes()) {
    backbone_points.push_back(node.path_position);
  }
  out << ",\"global_backbone\":";
  writePointArrayJson(out, backbone_points);
  out << ",\"selected_start_portal\":";
  writePointArrayJson(out, plan.debug_selected_start_portal);
  out << ",\"selected_goal_portal\":";
  writePointArrayJson(out, plan.debug_selected_goal_portal);
  out << ",\"metrics\":{";
  out << "\"path_length_m\":" << plan.metrics.path_length_m <<
    ",\"xy_start_goal_distance_m\":" << xy_distance <<
    ",\"detour_xy\":" << detour_ratio <<
    ",\"detour_ratio\":" << detour_ratio <<
    ",\"backbone_ratio\":" << backbone_ratio <<
    ",\"surface_ratio\":" <<
    (plan.metrics.path_length_m > 1.0e-6 ?
    plan.metrics.surface_path_length_m / plan.metrics.path_length_m : 0.0) <<
    ",\"backbone_path_length_m\":" << plan.metrics.backbone_path_length_m <<
    ",\"surface_path_length_m\":" << plan.metrics.surface_path_length_m <<
    ",\"portal_switch_count\":" << plan.metrics.portal_switch_count <<
    ",\"used_backbone_edges\":" << plan.metrics.used_backbone_edges <<
    ",\"used_surface_edges\":" << plan.metrics.used_surface_edges <<
    ",\"used_portal_edges\":" << plan.metrics.used_portal_edges <<
    ",\"max_path_edge_dz_m\":" << plan.metrics.max_path_edge_dz_m;
  out << "}}\n";
}

SweepOptions parseArgs(int argc, char ** argv)
{
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--export-jsonl" || arg == "--mode" ||
      arg == "--same-z-min" || arg == "--same-z-max")
    {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " requires a value");
      }
      positional.push_back(arg);
      positional.push_back(argv[++i]);
      continue;
    }
    positional.push_back(arg);
  }

  SweepOptions options;
  std::vector<std::string> values;
  for (std::size_t i = 0U; i < positional.size(); ++i) {
    const std::string & arg = positional[i];
    if (arg == "--export-jsonl") {
      options.export_jsonl_path = positional[++i];
    } else if (arg == "--mode") {
      const std::string mode = positional[++i];
      if (mode == "low-high") {
        options.query_mode = QueryMode::LowHigh;
      } else if (mode == "same-band") {
        options.query_mode = QueryMode::SameBand;
      } else {
        throw std::runtime_error("unknown --mode: " + mode);
      }
    } else if (arg == "--same-z-min") {
      options.same_z_min = parseDouble(positional[++i].c_str());
      options.has_same_band_bounds = true;
      options.has_same_z_min = true;
    } else if (arg == "--same-z-max") {
      options.same_z_max = parseDouble(positional[++i].c_str());
      options.has_same_band_bounds = true;
      options.has_same_z_max = true;
    } else if (arg == "--strict-all") {
      options.strict_all = true;
    } else if (arg == "--dominant-only") {
      options.strict_all = false;
    } else {
      values.push_back(arg);
    }
  }

  if (values.empty() || values.size() > 4U || values.size() == 2U) {
    throw std::runtime_error(
      "usage: tgw_experience_global_sweep <n3map.pbstream> [low_z_max high_z_min [sample_pairs]] "
      "[--mode low-high|same-band] [--same-z-min M --same-z-max M] "
      "[--export-jsonl /tmp/tgw_sweep_paths.jsonl] [--strict-all|--dominant-only]");
  }
  options.pbstream_path = values[0];
  if (values.size() >= 3U) {
    options.auto_bands = false;
    options.low_z_max = parseDouble(values[1].c_str());
    options.high_z_min = parseDouble(values[2].c_str());
  }
  if (values.size() == 4U) {
    options.sample_pairs = parseSize(values[3].c_str());
  }
  if (options.has_same_band_bounds && (!options.has_same_z_min || !options.has_same_z_max)) {
    throw std::runtime_error("--same-z-min and --same-z-max must be provided together");
  }
  if (options.has_same_band_bounds && options.same_z_min > options.same_z_max) {
    throw std::runtime_error("--same-z-min must be <= --same-z-max");
  }
  return options;
}

TrajectoryProjectorOptions projectorOptions()
{
  return defaultTrajectoryProjectorOptions();
}

ExperienceSurfaceBuilderOptions builderOptions()
{
  return defaultExperienceSurfaceBuilderOptions();
}

ExperienceGeometryIndexOptions geometryIndexOptions()
{
  ExperienceGeometryIndexOptions options;
  const ExperienceSurfaceBuilderOptions builder = builderOptions();
  options.raw_resolution_m = builder.projector.raw_resolution_m;
  options.nav_resolution_m = builder.resolution_m;
  options.body_clearance_height_m = builder.body_clearance_height_m;
  options.max_debug_world_points = 0U;
  return options;
}

SurfacePlannerOptions plannerOptions()
{
  return defaultSurfacePlannerOptions();
}

SurfaceGraphBuildOptions graphOptions()
{
  return defaultSurfaceGraphBuildOptions();
}

ExperienceBackboneOptions backboneOptions()
{
  return defaultExperienceBackboneOptions();
}

HybridExperiencePlannerOptions hybridOptions()
{
  return defaultHybridExperiencePlannerOptions();
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
  const ExperienceBackboneGraph & backbone_graph,
  const HybridExperiencePlannerOptions & hybrid_options)
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
    if (!isBackboneEdgeAllowedForHybrid(edge, hybrid_options)) {
      continue;
    }
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

int largestComponent(const std::unordered_map<int, std::size_t> & components)
{
  int best_component = -1;
  std::size_t best_count = 0U;
  for (const auto & entry : components) {
    if (entry.second > best_count) {
      best_count = entry.second;
      best_component = entry.first;
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

std::vector<SurfaceNodeId> reversedGoalSamples(std::vector<SurfaceNodeId> samples)
{
  std::reverse(samples.begin(), samples.end());
  if (samples.size() > 1U) {
    bool all_same_pair = true;
    for (std::size_t i = 0U; i < samples.size(); ++i) {
      if (samples[i].id != samples[samples.size() - 1U - i].id) {
        all_same_pair = false;
        break;
      }
    }
    if (all_same_pair) {
      std::rotate(samples.begin(), samples.begin() + 1, samples.end());
    }
  }
  return samples;
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

    ExperienceGeometryIndex geometry;
    const ExperienceGeometryIndexBuildResult geometry_result =
      geometry.build(read_result.resource, geometryIndexOptions());
    if (!geometry_result.success) {
      std::cerr << "geometry_failed error_code=" << geometry_result.error_code <<
        " message=" << geometry_result.message << std::endl;
      return 3;
    }
    const TrajectoryProjectionResult projection =
      TrajectoryProjector(projectorOptions()).project(read_result.resource, geometry);
    const ExperienceBuildResult build =
      ExperienceSurfaceBuilder(builderOptions()).build(read_result.resource, geometry, projection);
    if (!build.success) {
      std::cerr << "build_failed error_code=" << build.error_code <<
        " message=" << build.message << std::endl;
      return 3;
    }

    const SurfacePlannerOptions surface_options = plannerOptions();
    ExperienceSurfaceGraph surface_graph;
    surface_graph.build(
      build.snapshot, SurfaceTransitionValidator(surface_options), surface_options, graphOptions());

    ExperienceBackboneGraph backbone_graph;
    backbone_graph.build(read_result.resource, projection, surface_graph, backboneOptions());

    double low_z_max = sweep_options.low_z_max;
    double high_z_min = sweep_options.high_z_min;
    if (sweep_options.auto_bands) {
      chooseAutoBands(surface_graph, &low_z_max, &high_z_min);
    }

    double same_z_min = sweep_options.same_z_min;
    double same_z_max = sweep_options.same_z_max;
    if (sweep_options.query_mode == QueryMode::SameBand && !sweep_options.has_same_band_bounds) {
      same_z_min = low_z_max;
      same_z_max = high_z_min;
    }

    std::vector<SurfaceNodeId> low_nodes;
    std::vector<SurfaceNodeId> high_nodes;
    std::vector<SurfaceNodeId> same_band_nodes;
    low_nodes.reserve(surface_graph.nodes().size());
    high_nodes.reserve(surface_graph.nodes().size());
    same_band_nodes.reserve(surface_graph.nodes().size());
    for (const SurfaceNode & node : surface_graph.nodes()) {
      if (node.z <= low_z_max) {
        low_nodes.push_back(node.id);
      }
      if (node.z >= high_z_min) {
        high_nodes.push_back(node.id);
      }
      if (node.z >= same_z_min && node.z <= same_z_max) {
        same_band_nodes.push_back(node.id);
      }
    }

    const HybridExperiencePlannerOptions hybrid_options = hybridOptions();
    HybridExperienceGraph hybrid_graph;
    hybrid_graph.build(surface_graph, backbone_graph, surface_options, hybrid_options);
    const HybridConnectivity connectivity =
      buildHybridConnectivity(surface_graph, backbone_graph, hybrid_options);
    const auto low_components = countComponents(low_nodes, connectivity);
    const auto high_components = countComponents(high_nodes, connectivity);
    const auto same_band_components = countComponents(same_band_nodes, connectivity);
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
    const bool all_same_band_connected =
      !same_band_nodes.empty() &&
      same_band_components.size() == 1U;

    const int dominant_shared_component =
      sweep_options.query_mode == QueryMode::SameBand ?
      largestComponent(same_band_components) :
      largestSharedComponent(low_components, high_components);
    const std::vector<SurfaceNodeId> dominant_low_nodes =
      filterNodesByComponent(low_nodes, connectivity, dominant_shared_component);
    const std::vector<SurfaceNodeId> dominant_high_nodes =
      filterNodesByComponent(high_nodes, connectivity, dominant_shared_component);
    const std::vector<SurfaceNodeId> dominant_same_band_nodes =
      filterNodesByComponent(same_band_nodes, connectivity, dominant_shared_component);
    const double dominant_low_coverage =
      low_nodes.empty() ? 0.0 :
      static_cast<double>(dominant_low_nodes.size()) / static_cast<double>(low_nodes.size());
    const double dominant_high_coverage =
      high_nodes.empty() ? 0.0 :
      static_cast<double>(dominant_high_nodes.size()) / static_cast<double>(high_nodes.size());
    const double dominant_same_band_coverage =
      same_band_nodes.empty() ? 0.0 :
      static_cast<double>(dominant_same_band_nodes.size()) /
      static_cast<double>(same_band_nodes.size());

    HybridExperiencePlanner planner(surface_options, hybrid_options);
    std::vector<SurfaceNodeId> sampled_start;
    std::vector<SurfaceNodeId> sampled_goal;
    if (sweep_options.query_mode == QueryMode::SameBand) {
      sampled_start = sampleNodes(dominant_same_band_nodes, sweep_options.sample_pairs);
      sampled_goal = reversedGoalSamples(sampled_start);
      if (sampled_goal.size() > 1U) {
        for (std::size_t i = 0U; i < sampled_start.size() && i < sampled_goal.size(); ++i) {
          if (sampled_start[i].id == sampled_goal[i].id) {
            std::rotate(sampled_goal.begin(), sampled_goal.begin() + 1, sampled_goal.end());
            break;
          }
        }
      }
    } else {
      sampled_start = sampleNodes(dominant_low_nodes, sweep_options.sample_pairs);
      sampled_goal = sampleNodes(dominant_high_nodes, sweep_options.sample_pairs);
    }
    std::ofstream jsonl_out;
    if (!sweep_options.export_jsonl_path.empty()) {
      jsonl_out.open(sweep_options.export_jsonl_path);
      if (!jsonl_out.is_open()) {
        throw std::runtime_error("failed to open export JSONL: " + sweep_options.export_jsonl_path);
      }
      jsonl_out << std::setprecision(10);
    }
    std::size_t sampled_queries = 0U;
    std::size_t sampled_success = 0U;
    std::string first_failure;
    std::size_t first_failure_index = 0U;
    QualityStats quality;
    for (std::size_t i = 0; i < sampled_start.size() && i < sampled_goal.size(); ++i) {
      const SurfacePlanResult plan =
        planner.plan(surface_graph, backbone_graph, hybrid_graph, sampled_start[i], sampled_goal[i]);
      ++sampled_queries;
      const Point3 start = surfacePoint(surface_graph, sampled_start[i]);
      const Point3 goal = surfacePoint(surface_graph, sampled_goal[i]);
      const double xy_distance = xyDistance(start, goal);
      const double plan_length = plan.metrics.path_length_m > 0.0 ?
        plan.metrics.path_length_m : pathLength(plan.path);
      const double detour_ratio = plan.success ?
        plan_length / std::max(xy_distance, 1.0e-6) : 0.0;
      const double backbone_ratio = plan.success && plan_length > 1.0e-6 ?
        plan.metrics.backbone_path_length_m / plan_length : 0.0;
      if (jsonl_out.is_open()) {
        writeQueryJsonl(
          jsonl_out, i, start, goal, plan, backbone_graph, xy_distance, detour_ratio, backbone_ratio);
      }
      if (plan.success) {
        ++sampled_success;
        ++quality.successful_queries;
        quality.detour_sum += detour_ratio;
        quality.max_detour_ratio = std::max(quality.max_detour_ratio, detour_ratio);
        quality.backbone_ratio_sum += backbone_ratio;
        quality.max_backbone_ratio = std::max(quality.max_backbone_ratio, backbone_ratio);
        quality.max_portal_switch_count =
          std::max(quality.max_portal_switch_count, plan.metrics.portal_switch_count);
        if (detour_ratio > 3.0 || (xy_distance < 8.0 && backbone_ratio > 0.8)) {
          ++quality.suspicious_detour_count;
        }
      } else if (first_failure.empty()) {
        first_failure_index = i;
        first_failure = plan.message + " start=(" + std::to_string(start.x) + "," +
          std::to_string(start.y) + "," + std::to_string(start.z) + ") goal=(" +
          std::to_string(goal.x) + "," + std::to_string(goal.y) + "," +
          std::to_string(goal.z) + ") start_node=" + std::to_string(sampled_start[i].id) +
          " goal_node=" + std::to_string(sampled_goal[i].id) +
          " start_hybrid_component=" +
          std::to_string(connectivity.component[sampled_start[i].id]) +
          " goal_hybrid_component=" +
          std::to_string(connectivity.component[sampled_goal[i].id]) +
          " start_surface_component=" +
          std::to_string(surface_graph.componentId(sampled_start[i])) +
          " goal_surface_component=" +
          std::to_string(surface_graph.componentId(sampled_goal[i])) +
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
      (sweep_options.query_mode == QueryMode::SameBand ?
      dominant_same_band_nodes.size() >= 2U :
      (!dominant_low_nodes.empty() && !dominant_high_nodes.empty())) &&
      sampled_queries > 0U &&
      sampled_success == sampled_queries;
    const bool sweep_success = sweep_options.strict_all ?
      ((sweep_options.query_mode == QueryMode::SameBand ?
      all_same_band_connected : all_cross_connected) && main_component_sweep_success) :
      main_component_sweep_success;
    const double mean_detour_ratio = quality.successful_queries > 0U ?
      quality.detour_sum / static_cast<double>(quality.successful_queries) : 0.0;
    const double mean_backbone_ratio = quality.successful_queries > 0U ?
      quality.backbone_ratio_sum / static_cast<double>(quality.successful_queries) : 0.0;

    const auto end_time = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

    std::cout << "global_sweep_success=" <<
      (sweep_success ? "true" : "false") << "\n";
    std::cout << "pbstream=" << sweep_options.pbstream_path << "\n";
    std::cout << "query_mode=" << queryModeName(sweep_options.query_mode) << "\n";
    std::cout << "low_z_max=" << low_z_max << " high_z_min=" << high_z_min <<
      " auto_bands=" << (sweep_options.auto_bands ? "true" : "false") << "\n";
    if (sweep_options.query_mode == QueryMode::SameBand) {
      std::cout << "same_z_min=" << same_z_min << " same_z_max=" << same_z_max <<
        " same_band_auto=" << (!sweep_options.has_same_band_bounds ? "true" : "false") << "\n";
    }
    std::cout << "surface_nodes=" << surface_graph.nodes().size() <<
      " surface_components=" << surface_graph.componentCount() <<
      " low_nodes=" << low_nodes.size() <<
      " high_nodes=" << high_nodes.size() <<
      " same_band_nodes=" << same_band_nodes.size() << "\n";
    std::cout << "backbone_nodes=" << backbone_graph.nodes().size() <<
      " backbone_edges=" << backbone_graph.edges().size() <<
      " portals=" << backbone_graph.portals().size() << "\n";
    std::cout << "hybrid_components=" << connectivity.component_count <<
      " hybrid_surface_edges=" << connectivity.surface_edges <<
      " hybrid_backbone_edges=" << connectivity.backbone_edges <<
      " hybrid_portal_edges=" << connectivity.portal_edges << "\n";
    std::cout << "low_components=" << low_components.size() <<
      " high_components=" << high_components.size() <<
      " same_band_components=" << same_band_components.size() <<
      " cross_connected_low=" << cross_connected_low <<
      " cross_connected_high=" << cross_connected_high <<
      " all_low_high_connected=" << (all_cross_connected ? "true" : "false") <<
      " all_same_band_connected=" << (all_same_band_connected ? "true" : "false") << "\n";
    std::cout << "dominant_shared_component=" << dominant_shared_component <<
      " dominant_low_nodes=" << dominant_low_nodes.size() <<
      " dominant_high_nodes=" << dominant_high_nodes.size() <<
      " dominant_same_band_nodes=" << dominant_same_band_nodes.size() <<
      " dominant_low_coverage=" << dominant_low_coverage <<
      " dominant_high_coverage=" << dominant_high_coverage <<
      " dominant_same_band_coverage=" << dominant_same_band_coverage <<
      " main_component_sweep_success=" << (main_component_sweep_success ? "true" : "false") << "\n";
    std::cout << "sampled_queries=" << sampled_queries <<
      " sampled_success=" << sampled_success << "\n";
    std::cout << "max_detour_ratio=" << quality.max_detour_ratio <<
      " mean_detour_ratio=" << mean_detour_ratio <<
      " suspicious_detour_count=" << quality.suspicious_detour_count <<
      " max_backbone_ratio=" << quality.max_backbone_ratio <<
      " mean_backbone_ratio=" << mean_backbone_ratio <<
      " max_portal_switch_count=" << quality.max_portal_switch_count << "\n";
    if (!sweep_options.export_jsonl_path.empty()) {
      std::cout << "export_jsonl=" << sweep_options.export_jsonl_path << "\n";
    }
    if (!first_failure.empty()) {
      std::cout << "first_sample_failure_index=" << first_failure_index << "\n";
      std::cout << "first_sample_failure=" << first_failure << "\n";
    }
    std::cout << "elapsed_ms=" << elapsed_ms << "\n";

    return sweep_success ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << std::endl;
    return 2;
  }
}
