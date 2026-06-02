#include <iostream>

#include "tgw_planner/core/probabilistic_voxel_map.hpp"
#include "tgw_planner/core/raycast_integrator.hpp"
#include "tgw_planner/core/risk_field.hpp"
#include "tgw_planner/core/robot_footprint.hpp"
#include "tgw_planner/core/clearance_field.hpp"
#include "tgw_planner/core/map_snapshot.hpp"
#include "tgw_planner/core/path_validator.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

using tgw_planner::core::GridIndex;
using tgw_planner::core::MappingOptions;
using tgw_planner::core::Point3;
using tgw_planner::core::Pose3;
using tgw_planner::core::ProbabilisticVoxelMap;
using tgw_planner::core::RaycastIntegrator;
using tgw_planner::core::RiskField;
using tgw_planner::core::RobotFootprint;
using tgw_planner::core::ScanInput;
using tgw_planner::core::ClearanceField;
using tgw_planner::core::NavigationSnapshot;
using tgw_planner::core::PathValidator;
using tgw_planner::core::SurfaceExtractionOptions;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfaceExtractor;

#define CHECK(condition) \
  do { \
    if (!(condition)) { \
      std::cerr << "CHECK failed: " #condition "\n"; \
      return 1; \
    } \
  } while (false)

int main()
{
  MappingOptions options;
  options.resolution_m = 0.10;
  options.min_range_m = 0.05;
  options.max_range_m = 5.0;
  options.min_static_hits = 2;
  options.min_distinct_views = 2;
  options.min_static_lifetime_sec = 0.5;

  ProbabilisticVoxelMap map(options);
  RaycastIntegrator integrator(options);

  Pose3 pose;
  pose.translation = {0.0, 0.0, 0.0};
  ScanInput scan;
  scan.sensor_pose_map = pose;
  scan.stamp_sec = 0.0;
  scan.view_id = 1;
  scan.points_sensor_frame.push_back({1.0, 0.0, 0.0});
  const auto stats = integrator.insertScan(scan, map);
  CHECK(stats.inserted_points == 1U);
  CHECK(stats.hit_updates == 1U);
  CHECK(stats.miss_updates > 0U);

  const GridIndex endpoint = map.worldToGrid({1.0, 0.0, 0.0});
  const GridIndex free_cell = map.worldToGrid({0.5, 0.0, 0.0});
  CHECK(map.isOccupied(endpoint));
  CHECK(map.lookup(free_cell) != nullptr);
  CHECK(map.probability(free_cell) < 0.5F);

  map.updateHit(endpoint, 1.0, 2);
  CHECK(map.lookup(endpoint)->static_candidate);
  CHECK(!map.lookup(endpoint)->dynamic_suspect);

  GridIndex transient{20, 0, 0};
  map.updateHit(transient, 0.0, 1);
  map.updateMiss(transient, 0.2, 2);
  map.updateMiss(transient, 0.3, 3);
  map.updateMiss(transient, 0.4, 4);
  map.updateMiss(transient, 0.5, 5);
  CHECK(map.lookup(transient)->dynamic_suspect);
  CHECK(map.isFree(transient));

  RobotFootprint footprint;
  const std::vector<Point3> samples = footprint.sampleFootprint({0.0, 0.0, 0.0}, 0.0, 0.10);
  CHECK(!samples.empty());
  CHECK(footprint.containsBodyPoint({0.0, 0.0, 0.0}));
  CHECK(!footprint.containsBodyPoint({2.0, 0.0, 0.0}));

  ProbabilisticVoxelMap corridor_map(options);
  for (int x = 0; x <= 6; ++x) {
    for (int y = 0; y <= 4; ++y) {
      corridor_map.updateHit({x, y, 0}, 0.0, 1);
    }
  }
  SurfaceExtractionOptions surface_options;
  surface_options.min_static_hits = 1;
  surface_options.require_static_support = false;
  SurfaceExtractor surface_extractor(surface_options);
  const auto surface = surface_extractor.extract(corridor_map);
  CHECK(surface.traversable_cells.size() == 35U);
  const GridIndex edge{0, 2, 1};
  const GridIndex center{3, 2, 1};
  CHECK(surface.boundary_cells.find(edge) != surface.boundary_cells.end());
  CHECK(surface.boundary_cells.find(center) == surface.boundary_cells.end());

  ClearanceField clearance;
  clearance.compute(surface.traversable_cells, surface.boundary_cells, options.resolution_m);
  CHECK(clearance.clearanceDistance(center) > clearance.clearanceDistance(edge));
  CHECK(clearance.clearancePenalty(center) < clearance.clearancePenalty(edge));
  RiskField risk;
  risk.compute(surface, clearance);
  CHECK(risk.riskCost(edge) > risk.riskCost(center));
  CHECK(!risk.risks().empty());
  const auto medial_axis = clearance.medialAxisCells(0.10);
  bool medial_has_center = false;
  bool medial_has_edge = false;
  for (const GridIndex & cell : medial_axis) {
    medial_has_center = medial_has_center || cell == center;
    medial_has_edge = medial_has_edge || cell == edge;
  }
  CHECK(medial_has_center);
  CHECK(!medial_has_edge);

  ProbabilisticVoxelMap planner_map(options);
  for (int x = -6; x <= 18; ++x) {
    for (int y = 0; y <= 8; ++y) {
      planner_map.updateHit({x, y, 0}, 0.0, 1);
    }
  }
  NavigationSnapshot snapshot;
  snapshot.resolution_m = options.resolution_m;
  snapshot.surface = surface_extractor.extract(planner_map);
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);
  SurfacePlannerOptions planner_options;
  planner_options.w_clearance = 2.0;
  SurfaceAstarPlanner planner(planner_options);
  const auto plan = planner.plan(snapshot, {2, 4, 1}, {10, 4, 1});
  CHECK(plan.success);
  bool used_center_lane = false;
  for (const auto & cell : plan.cells) {
    used_center_lane = used_center_lane || cell.y == 4;
  }
  CHECK(used_center_lane);
  CHECK(plan.metrics.min_path_clearance_m >= 0.0);
  CHECK(plan.metrics.mean_path_clearance_m > 0.0);
  PathValidator validator(footprint);
  const auto validation = validator.validate(snapshot, plan.path);
  if (!validation.valid) {
    std::cerr << "validation failed: " << validation.failure_reason << "\n";
  }
  CHECK(validation.valid);
  CHECK(validation.checked_samples > plan.path.size());
  CHECK(validation.mean_clearance_m > 0.0);

  std::cout << "phase1_phase2_phase3_core_smoke passed\n";
  return 0;
}

#undef CHECK
