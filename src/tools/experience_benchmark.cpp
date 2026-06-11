#include <chrono>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <sys/resource.h>
#include <unordered_map>

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/experience_geometry_index.hpp"
#include "tgw_planner/core/experience_planner_defaults.hpp"
#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace
{
using tgw_planner::core::ExperienceBackboneGraph;
using tgw_planner::core::ExperienceBuildResult;
using tgw_planner::core::ExperienceGeometryIndex;
using tgw_planner::core::ExperienceGeometryIndexBuildResult;
using tgw_planner::core::ExperienceGeometryIndexOptions;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceGraph;
using tgw_planner::core::HybridExperienceGraph;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3MapReadResult;
using tgw_planner::core::SurfaceGraphBuildOptions;
using tgw_planner::core::SurfaceNode;
using tgw_planner::core::SurfaceNodeId;
using tgw_planner::core::SurfacePlanResult;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfaceTransitionValidator;
using tgw_planner::core::TrajectoryProjectionResult;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::defaultExperienceBackboneOptions;
using tgw_planner::core::defaultExperienceSurfaceBuilderOptions;
using tgw_planner::core::defaultHybridExperiencePlannerOptions;
using tgw_planner::core::defaultSurfaceGraphBuildOptions;
using tgw_planner::core::defaultSurfacePlannerOptions;
using tgw_planner::core::defaultTrajectoryProjectorOptions;

double elapsedMs(const std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

double peakRssMb()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0.0;
  }
  return static_cast<double>(usage.ru_maxrss) / 1024.0;
}

ExperienceGeometryIndexOptions geometryOptions()
{
  const auto builder = defaultExperienceSurfaceBuilderOptions();
  ExperienceGeometryIndexOptions options;
  options.raw_resolution_m = builder.projector.raw_resolution_m;
  options.nav_resolution_m = builder.resolution_m;
  options.body_clearance_height_m = builder.body_clearance_height_m;
  options.max_debug_world_points = 0U;
  return options;
}

std::pair<SurfaceNodeId, SurfaceNodeId> chooseFirstQuery(const ExperienceSurfaceGraph & graph)
{
  std::unordered_map<int, SurfaceNodeId> first_by_component;
  for (const SurfaceNode & node : graph.nodes()) {
    const int component = graph.componentId(node.id);
    const auto inserted = first_by_component.emplace(component, node.id);
    if (!inserted.second) {
      return {inserted.first->second, node.id};
    }
  }
  return {{std::numeric_limits<std::uint32_t>::max()}, {std::numeric_limits<std::uint32_t>::max()}};
}

void writeJsonField(const char * name, double value, bool comma = true)
{
  std::cout << "\"" << name << "\":" << value;
  if (comma) {
    std::cout << ",";
  }
}

void writeJsonField(const char * name, std::size_t value, bool comma = true)
{
  std::cout << "\"" << name << "\":" << value;
  if (comma) {
    std::cout << ",";
  }
}

void writeJsonString(const char * name, const std::string & value, bool comma = true)
{
  std::cout << "\"" << name << "\":\"";
  for (const char ch : value) {
    if (ch == '"' || ch == '\\') {
      std::cout << '\\';
    }
    std::cout << ch;
  }
  std::cout << "\"";
  if (comma) {
    std::cout << ",";
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: tgw_experience_benchmark <n3map.pbstream>" << std::endl;
    return 2;
  }

  const std::string pbstream_path = argv[1];
  const auto t_read = std::chrono::steady_clock::now();
  N3MapReader reader;
  const N3MapReadResult read = reader.readPbstream(pbstream_path);
  const double read_ms = elapsedMs(t_read);
  if (!read.success) {
    std::cout << "{";
    writeJsonString("status", "read_failed");
    writeJsonString("error_code", read.error_code);
    writeJsonString("message", read.message, false);
    std::cout << "}" << std::endl;
    return 3;
  }

  ExperienceGeometryIndex geometry;
  const auto t_geometry = std::chrono::steady_clock::now();
  const ExperienceGeometryIndexBuildResult geometry_result =
    geometry.build(read.resource, geometryOptions());
  const double geometry_ms = elapsedMs(t_geometry);
  if (!geometry_result.success) {
    std::cout << "{";
    writeJsonString("status", "geometry_failed");
    writeJsonString("error_code", geometry_result.error_code);
    writeJsonString("message", geometry_result.message, false);
    std::cout << "}" << std::endl;
    return 4;
  }

  const auto t_projection = std::chrono::steady_clock::now();
  const TrajectoryProjectionResult projection =
    TrajectoryProjector(defaultTrajectoryProjectorOptions()).project(read.resource, geometry);
  const double projection_ms = elapsedMs(t_projection);

  const auto t_surface = std::chrono::steady_clock::now();
  const ExperienceBuildResult surface =
    ExperienceSurfaceBuilder(defaultExperienceSurfaceBuilderOptions()).build(
    read.resource, geometry, projection);
  const double surface_ms = elapsedMs(t_surface);
  if (!surface.success) {
    std::cout << "{";
    writeJsonString("status", "surface_failed");
    writeJsonString("error_code", surface.error_code);
    writeJsonString("message", surface.message, false);
    std::cout << "}" << std::endl;
    return 5;
  }

  const SurfacePlannerOptions planner_options = defaultSurfacePlannerOptions();
  SurfaceGraphBuildOptions graph_options = defaultSurfaceGraphBuildOptions();
  graph_options.max_edge_height_delta_m = planner_options.max_step_height_m;
  const auto t_surface_graph = std::chrono::steady_clock::now();
  ExperienceSurfaceGraph surface_graph;
  surface_graph.build(
    surface.snapshot, SurfaceTransitionValidator(planner_options), planner_options, graph_options);
  const double surface_graph_ms = elapsedMs(t_surface_graph);

  const auto t_backbone = std::chrono::steady_clock::now();
  ExperienceBackboneGraph backbone_graph;
  backbone_graph.build(
    read.resource, projection, surface_graph, defaultExperienceBackboneOptions());
  const double backbone_ms = elapsedMs(t_backbone);

  const auto hybrid_options = defaultHybridExperiencePlannerOptions();
  const auto t_hybrid = std::chrono::steady_clock::now();
  HybridExperienceGraph hybrid_graph;
  hybrid_graph.build(surface_graph, backbone_graph, planner_options, hybrid_options);
  const double hybrid_ms = elapsedMs(t_hybrid);

  double first_query_ms = 0.0;
  bool first_query_success = false;
  const auto query = chooseFirstQuery(surface_graph);
  if (surface_graph.isValid(query.first) && surface_graph.isValid(query.second)) {
    const auto t_query = std::chrono::steady_clock::now();
    const SurfacePlanResult plan =
      HybridExperiencePlanner(planner_options, hybrid_options).plan(
      surface_graph, backbone_graph, hybrid_graph, query.first, query.second);
    first_query_ms = elapsedMs(t_query);
    first_query_success = plan.success;
  }

  std::cout << "{";
  writeJsonString("status", "ok");
  writeJsonField("read_pbstream_ms", read_ms);
  writeJsonField("geometry_index_build_ms", geometry_ms);
  writeJsonField("trajectory_projection_ms", projection_ms);
  writeJsonField("surface_build_ms", surface_ms);
  writeJsonField("surface_graph_build_ms", surface_graph_ms);
  writeJsonField("backbone_build_ms", backbone_ms);
  writeJsonField("hybrid_graph_build_ms", hybrid_ms);
  writeJsonField("first_query_ms", first_query_ms);
  writeJsonField("first_query_success", static_cast<std::size_t>(first_query_success ? 1U : 0U));
  writeJsonField("peak_rss_mb", peakRssMb());
  writeJsonField("keyframes", read.resource.keyframes.size());
  writeJsonField("dense_trajectory", read.resource.dense_trajectory.size());
  writeJsonField("transformed_points", geometry_result.transformed_points);
  writeJsonField("raw_geometry_cells", geometry_result.raw_geometry_cell_count);
  writeJsonField("support_candidates", geometry_result.support_candidate_count);
  writeJsonField("support_columns", geometry_result.support_column_count);
  writeJsonField("surface_nodes", surface_graph.nodes().size());
  writeJsonField("surface_edges", surface_graph.metrics().graph_edges);
  writeJsonField("backbone_nodes", backbone_graph.nodes().size());
  writeJsonField("backbone_edges", backbone_graph.edges().size());
  writeJsonField("portals", backbone_graph.portals().size());
  writeJsonField("hybrid_nodes", hybrid_graph.nodes().size());
  writeJsonField(
    "hybrid_edges",
    static_cast<std::size_t>(
      hybrid_graph.surfaceEdgeCount() + hybrid_graph.backboneEdgeCount() +
      hybrid_graph.portalEdgeCount()),
    false);
  std::cout << "}" << std::endl;
  return 0;
}
