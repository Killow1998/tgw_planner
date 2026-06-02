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
using tgw_planner::core::SelfFilterBox;
using tgw_planner::core::ClearanceField;
using tgw_planner::core::NavigationSnapshot;
using tgw_planner::core::PathValidationOptions;
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
  CHECK(stats.dynamic_suspect_voxels_after_decay == map.dynamicSuspectVoxels().size());
  CHECK(stats.static_candidate_voxels_after_decay == map.staticCandidateVoxels().size());

  const GridIndex endpoint = map.worldToGrid({1.0, 0.0, 0.0});
  const GridIndex free_cell = map.worldToGrid({0.5, 0.0, 0.0});
  CHECK(map.isOccupied(endpoint));
  CHECK(map.lookup(free_cell) != nullptr);
  CHECK(map.probability(free_cell) < 0.5F);

  ProbabilisticVoxelMap mount_filter_map(options);
  SelfFilterBox disabled_body_box;
  disabled_body_box.enabled = false;
  SelfFilterBox mount_box;
  mount_box.enabled = true;
  mount_box.min_x = 0.90;
  mount_box.max_x = 1.10;
  mount_box.min_y = -0.10;
  mount_box.max_y = 0.10;
  mount_box.min_z = -0.10;
  mount_box.max_z = 0.10;
  RaycastIntegrator mount_filter_integrator(options, disabled_body_box, mount_box);
  ScanInput mount_filter_scan;
  mount_filter_scan.sensor_pose_map = pose;
  mount_filter_scan.stamp_sec = 0.0;
  mount_filter_scan.view_id = 1;
  mount_filter_scan.points_sensor_frame.push_back({1.0, 0.0, 0.0});
  mount_filter_scan.points_sensor_frame.push_back({1.5, 0.0, 0.0});
  const auto mount_filter_stats =
    mount_filter_integrator.insertScan(mount_filter_scan, mount_filter_map);
  CHECK(mount_filter_stats.filtered_self == 1U);
  CHECK(mount_filter_stats.inserted_points == 1U);
  CHECK(!mount_filter_map.isOccupied(mount_filter_map.worldToGrid({1.0, 0.0, 0.0})));
  CHECK(mount_filter_map.isOccupied(mount_filter_map.worldToGrid({1.5, 0.0, 0.0})));

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
  surface_options.require_observed_free_space = false;
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

  NavigationSnapshot clearance_snapshot;
  clearance_snapshot.resolution_m = options.resolution_m;
  clearance_snapshot.surface = surface;
  clearance_snapshot.clearance = clearance;
  clearance_snapshot.risk = risk;
  PathValidationOptions clearance_validation_options;
  clearance_validation_options.require_footprint_support = false;
  clearance_validation_options.min_clearance_m = clearance.clearanceDistance(center) + 0.01;
  PathValidator clearance_validator(footprint, clearance_validation_options);
  const auto low_clearance_validation =
    clearance_validator.validate(clearance_snapshot, {corridor_map.gridToWorld(center)});
  CHECK(!low_clearance_validation.valid);
  CHECK(low_clearance_validation.low_clearance_samples == 1U);
  CHECK(low_clearance_validation.failure_reason == "final path clearance below minimum");
  PathValidationOptions clearance_report_options;
  clearance_report_options.require_footprint_support = false;
  clearance_report_options.low_clearance_report_threshold_m =
    clearance.clearanceDistance(center) + 0.01;
  PathValidator clearance_report_validator(footprint, clearance_report_options);
  const auto low_clearance_report =
    clearance_report_validator.validate(clearance_snapshot, {corridor_map.gridToWorld(center)});
  CHECK(low_clearance_report.valid);
  CHECK(low_clearance_report.low_clearance_samples == 1U);

  ProbabilisticVoxelMap floor_ceiling_map(options);
  for (int x = 0; x <= 2; ++x) {
    for (int y = 0; y <= 2; ++y) {
      floor_ceiling_map.updateHit({x, y, 0}, 0.0, 1);
      floor_ceiling_map.updateHit({x, y, 20}, 0.0, 1);
      floor_ceiling_map.updateMiss({x, y, 1}, 0.1, 1);
      floor_ceiling_map.updateMiss({x, y, 1}, 0.2, 1);
    }
  }
  SurfaceExtractionOptions observed_surface_options;
  observed_surface_options.min_static_hits = 1;
  observed_surface_options.require_static_support = false;
  observed_surface_options.require_observed_free_space = true;
  SurfaceExtractor observed_surface_extractor(observed_surface_options);
  const auto observed_surface = observed_surface_extractor.extract(floor_ceiling_map);
  CHECK(observed_surface.traversable_cells.find({1, 1, 1}) !=
    observed_surface.traversable_cells.end());
  CHECK(observed_surface.traversable_cells.find({1, 1, 21}) ==
    observed_surface.traversable_cells.end());
  CHECK(observed_surface.forbidden_cells.find({1, 1, 21}) !=
    observed_surface.forbidden_cells.end());

  ProbabilisticVoxelMap sampled_pcd_map(options);
  sampled_pcd_map.updateHit({0, 0, 0}, 0.0, 1);
  sampled_pcd_map.updateHit({0, 0, 1}, 0.0, 1);
  SurfaceExtractionOptions solid_hit_options;
  solid_hit_options.min_static_hits = 1;
  solid_hit_options.require_static_support = false;
  solid_hit_options.require_observed_free_space = false;
  const auto solid_hit_surface = SurfaceExtractor(solid_hit_options).extract(sampled_pcd_map);
  CHECK(solid_hit_surface.traversable_cells.find({0, 0, 1}) ==
    solid_hit_surface.traversable_cells.end());
  SurfaceExtractionOptions sampled_hit_options = solid_hit_options;
  sampled_hit_options.treat_hits_as_surface_samples = true;
  const auto sampled_hit_surface = SurfaceExtractor(sampled_hit_options).extract(sampled_pcd_map);
  CHECK(sampled_hit_surface.traversable_cells.find({0, 0, 1}) !=
    sampled_hit_surface.traversable_cells.end());

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
  CHECK(!plan.raw_cells.empty());
  CHECK(!plan.raw_path.empty());
  CHECK(plan.raw_cells.size() == plan.raw_path.size());
  CHECK(plan.metrics.raw_path_waypoints >= plan.cells.size());
  CHECK(plan.metrics.raw_path_length_m >= plan.metrics.path_length_m * 0.99);
  CHECK(plan.metrics.final_path_validated);
  CHECK(!plan.metrics.final_path_fallback_to_raw);
  bool used_center_lane = false;
  for (const auto & cell : plan.cells) {
    used_center_lane = used_center_lane || cell.y == 4;
  }
  CHECK(used_center_lane);
  CHECK(plan.metrics.min_path_clearance_m >= 0.0);
  CHECK(plan.metrics.mean_path_clearance_m > 0.0);

  SurfacePlannerOptions center_bias_options;
  center_bias_options.w_clearance = 5.0;
  center_bias_options.w_risk = 0.0;
  center_bias_options.w_slope = 0.0;
  center_bias_options.w_turn = 0.2;
  center_bias_options.require_footprint_support = false;
  center_bias_options.enable_shortcut = false;
  SurfaceAstarPlanner center_bias_planner(center_bias_options);
  const auto center_bias_plan = center_bias_planner.plan(snapshot, {2, 1, 1}, {16, 1, 1});
  CHECK(center_bias_plan.success);
  bool moved_to_center_lane = false;
  bool stayed_wall_hugging = true;
  for (const auto & cell : center_bias_plan.cells) {
    if (cell.x >= 5 && cell.x <= 13 && cell.y == 4) {
      moved_to_center_lane = true;
    }
    if (cell.x >= 5 && cell.x <= 13 && cell.y > 1) {
      stayed_wall_hugging = false;
    }
  }
  CHECK(moved_to_center_lane);
  CHECK(!stayed_wall_hugging);

  SurfacePlannerOptions clearance_shortcut_options = center_bias_options;
  clearance_shortcut_options.enable_shortcut = true;
  clearance_shortcut_options.shortcut_safety_margin_m = 0.15;
  SurfaceAstarPlanner clearance_shortcut_planner(clearance_shortcut_options);
  const auto clearance_shortcut_plan =
    clearance_shortcut_planner.plan(snapshot, {2, 1, 1}, {16, 1, 1});
  CHECK(clearance_shortcut_plan.success);
  CHECK(clearance_shortcut_plan.metrics.final_path_validated);
  CHECK(clearance_shortcut_plan.cells.size() > 2U);
  bool clearance_shortcut_kept_center_lane = false;
  for (const auto & cell : clearance_shortcut_plan.cells) {
    if (cell.x >= 5 && cell.x <= 13 && cell.y == 4) {
      clearance_shortcut_kept_center_lane = true;
    }
  }
  CHECK(clearance_shortcut_kept_center_lane);

  PathValidator validator(footprint);
  const auto validation = validator.validate(snapshot, plan.path);
  if (!validation.valid) {
    std::cerr << "validation failed: " << validation.failure_reason << "\n";
  }
  CHECK(validation.valid);
  CHECK(validation.checked_samples > plan.path.size());
  CHECK(validation.mean_clearance_m > 0.0);

  NavigationSnapshot fallback_snapshot;
  fallback_snapshot.resolution_m = options.resolution_m;
  for (int x = 0; x <= 6; ++x) {
    for (int y = 0; y <= 6; ++y) {
      const GridIndex cell{x, y, 0};
      fallback_snapshot.surface.traversable_cells.insert(cell);
      fallback_snapshot.surface.surface_cells[cell].cell = cell;
      fallback_snapshot.surface.surface_cells[cell].support = {x, y, -1};
      fallback_snapshot.surface.surface_cells[cell].height_m = fallback_snapshot.resolution_m * 0.5;
    }
  }
  fallback_snapshot.surface.boundary_cells.insert({3, 3, 0});
  fallback_snapshot.clearance.compute(
    fallback_snapshot.surface.traversable_cells, fallback_snapshot.surface.boundary_cells,
    fallback_snapshot.resolution_m);
  fallback_snapshot.risk.compute(fallback_snapshot.surface, fallback_snapshot.clearance);
  SurfacePlannerOptions fallback_options;
  fallback_options.require_footprint_support = false;
  fallback_options.w_clearance = 12.0;
  fallback_options.w_risk = 0.0;
  fallback_options.w_slope = 0.0;
  fallback_options.w_turn = 0.0;
  fallback_options.enable_shortcut = true;
  fallback_options.shortcut_clearance_ratio = 0.0;
  fallback_options.shortcut_safety_margin_m = 0.0;
  fallback_options.final_validation_min_clearance_m = 0.15;
  fallback_options.footprint.width_m = 0.0;
  SurfaceAstarPlanner fallback_planner(fallback_options);
  const auto fallback_plan =
    fallback_planner.plan(fallback_snapshot, {0, 3, 0}, {6, 3, 0});
  CHECK(fallback_plan.success);
  CHECK(fallback_plan.metrics.final_path_validated);
  CHECK(fallback_plan.metrics.final_path_fallback_to_raw);
  CHECK(
    fallback_plan.metrics.final_path_validation_failure.find("clearance below minimum") !=
    std::string::npos);
  bool fallback_uses_low_clearance_center = false;
  for (const auto & cell : fallback_plan.cells) {
    fallback_uses_low_clearance_center =
      fallback_uses_low_clearance_center || cell == GridIndex{3, 3, 0};
  }
  CHECK(!fallback_uses_low_clearance_center);

  NavigationSnapshot blocked_snapshot;
  blocked_snapshot.resolution_m = options.resolution_m;
  blocked_snapshot.surface.traversable_cells.insert({0, 0, 0});
  blocked_snapshot.surface.traversable_cells.insert({1, 0, 0});
  blocked_snapshot.surface.traversable_cells.insert({2, 0, 0});
  blocked_snapshot.surface.blocked_cells.insert({1, 0, 0});
  SurfacePlannerOptions blocked_planner_options;
  blocked_planner_options.require_footprint_support = false;
  SurfaceAstarPlanner blocked_planner(blocked_planner_options);
  const auto blocked_plan = blocked_planner.plan(blocked_snapshot, {0, 0, 0}, {2, 0, 0});
  CHECK(!blocked_plan.success);

  PathValidationOptions blocked_validation_options;
  blocked_validation_options.require_footprint_support = false;
  PathValidator blocked_validator(footprint, blocked_validation_options);
  const auto blocked_validation = blocked_validator.validate(
    blocked_snapshot, {{0.05, 0.05, 0.05}, {0.15, 0.05, 0.05}, {0.25, 0.05, 0.05}});
  CHECK(!blocked_validation.valid);
  CHECK(blocked_validation.failure_reason == "final path sample is blocked");

  NavigationSnapshot detour_snapshot;
  detour_snapshot.resolution_m = options.resolution_m;
  for (int x = 0; x <= 4; ++x) {
    for (int y = 0; y <= 2; ++y) {
      detour_snapshot.surface.traversable_cells.insert({x, y, 0});
    }
  }
  detour_snapshot.surface.blocked_cells.insert({2, 1, 0});
  SurfacePlannerOptions detour_options;
  detour_options.require_footprint_support = false;
  detour_options.enable_shortcut = true;
  SurfaceAstarPlanner detour_planner(detour_options);
  const auto detour_plan = detour_planner.plan(detour_snapshot, {0, 1, 0}, {4, 1, 0});
  CHECK(detour_plan.success);
  CHECK(detour_plan.metrics.final_path_validated);
  bool detour_used_blocked = false;
  bool detour_left_center_line = false;
  for (const auto & cell : detour_plan.cells) {
    detour_used_blocked = detour_used_blocked || cell == GridIndex{2, 1, 0};
    detour_left_center_line = detour_left_center_line || cell.y != 1;
  }
  CHECK(!detour_used_blocked);
  CHECK(detour_left_center_line);

  NavigationSnapshot unreachable_snapshot;
  unreachable_snapshot.resolution_m = options.resolution_m;
  for (int x = 0; x <= 4; ++x) {
    for (int y = 0; y <= 2; ++y) {
      unreachable_snapshot.surface.traversable_cells.insert({x, y, 0});
    }
  }
  for (int y = 0; y <= 2; ++y) {
    unreachable_snapshot.surface.blocked_cells.insert({2, y, 0});
  }
  SurfaceAstarPlanner unreachable_planner(detour_options);
  const auto unreachable_plan = unreachable_planner.plan(unreachable_snapshot, {0, 1, 0}, {4, 1, 0});
  CHECK(!unreachable_plan.success);
  CHECK(unreachable_plan.metrics.failure_reason == "surface A* failed to find a path");

  std::cout << "phase1_phase2_phase3_core_smoke passed\n";
  return 0;
}

#undef CHECK
