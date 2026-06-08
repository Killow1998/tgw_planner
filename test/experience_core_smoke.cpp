#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/path_validator.hpp"
#include "tgw_planner/core/reachable_expander.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace
{
using tgw_planner::core::ClearanceField;
using tgw_planner::core::ExperienceSnapshot;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::GridIndex;
using tgw_planner::core::N3KeyframeLite;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3NavResource;
using tgw_planner::core::N3TrajectoryPose;
using tgw_planner::core::PathValidationOptions;
using tgw_planner::core::PathValidator;
using tgw_planner::core::Point3;
using tgw_planner::core::PointXYZI;
using tgw_planner::core::Pose3;
using tgw_planner::core::ReachabilityLabel;
using tgw_planner::core::ReachableExpander;
using tgw_planner::core::ReachableExpanderOptions;
using tgw_planner::core::RiskField;
using tgw_planner::core::RobotFootprint;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfaceCell;
using tgw_planner::core::SurfaceLabel;
using tgw_planner::core::SurfaceMap;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::TrajectoryProjector;
using tgw_planner::core::TrajectoryProjectorOptions;

void require(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}

ExperienceSnapshot makeFlatSnapshot()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  for (int x = 0; x <= 6; ++x) {
    for (int y = -1; y <= 1; ++y) {
      const GridIndex cell{x, y, 0};
      SurfaceCell surface_cell;
      surface_cell.cell = cell;
      surface_cell.support = {x, y, -1};
      surface_cell.label = SurfaceLabel::Expanded;
      surface_cell.reachability = ReachabilityLabel::InferredReachable;
      surface_cell.height_m = 0.0;
      surface_cell.confidence = 1.0;
      snapshot.surface.surface_cells[cell] = surface_cell;
      snapshot.surface.traversable_cells.insert(cell);
      snapshot.reachability[cell] = ReachabilityLabel::InferredReachable;
      if (x == 0 || x == 6 || y == -1 || y == 1) {
        snapshot.surface.boundary_cells.insert(cell);
        snapshot.surface.wall_boundary_cells.insert(cell);
      }
    }
  }

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);
  return snapshot;
}

void testClearanceAndRisk()
{
  const ExperienceSnapshot snapshot = makeFlatSnapshot();
  require(
    snapshot.clearance.clearanceDistance({2, 0, 0}) >
    snapshot.clearance.clearanceDistance({0, 0, 0}),
    "center corridor cell should have more clearance than the boundary");
  require(
    snapshot.risk.riskCost({0, 0, 0}) > snapshot.risk.riskCost({2, 0, 0}),
    "boundary cell should have higher risk than center");
}

void testFootprintPlannerAndValidator()
{
  ExperienceSnapshot snapshot = makeFlatSnapshot();

  RobotFootprint footprint;
  require(
    footprint.isSupported(snapshot.surface, {1.25, 0.25, 0.25}, 0.0, snapshot.resolution_m),
    "default footprint should be supported on the 3-cell-wide corridor");

  SurfacePlannerOptions planner_options;
  planner_options.max_iterations = 50000;
  planner_options.require_footprint_support = true;
  planner_options.enable_shortcut = true;
  SurfaceAstarPlanner planner(planner_options);
  const auto plan = planner.plan(snapshot, {1, 0, 0}, {5, 0, 0});
  require(plan.success, "surface A* should succeed on the flat corridor");
  require(plan.metrics.final_path_validated, "final path should validate");
  require(!plan.metrics.final_path_fallback_to_raw, "flat corridor should not need raw fallback");
  require(plan.metrics.path_length_m > 0.0, "planner should report positive path length");

  PathValidationOptions validation_options;
  validation_options.require_footprint_support = true;
  PathValidator validator(footprint, validation_options);
  const auto report = validator.validate(snapshot, plan.path);
  require(report.valid, "planner path should pass explicit path validator");
  require(report.min_clearance_m > 0.0, "validated path should have positive clearance");
}

void testExperienceBuilderSkeleton()
{
  N3MapReader reader;
  const auto read_result = reader.readPbstream("/tmp/example.pbstream");
  require(!read_result.success, "missing pbstream should fail explicitly");
  require(
    read_result.error_code == "pbstream_open_failed",
    "missing pbstream should report open failure");

  N3NavResource resource;
  resource.map_frame = "map";
  resource.body_frame = "base";
  resource.version = "2.3.0";
  resource.dense_trajectory_source = "native";
  resource.has_native_dense_trajectory = true;

  for (int i = 0; i < 3; ++i) {
    N3TrajectoryPose pose;
    pose.seq = static_cast<std::uint64_t>(i);
    pose.timestamp = static_cast<double>(i);
    pose.pose_world_lidar.translation = {0.5 * static_cast<double>(i), 0.0, 0.5};
    resource.dense_trajectory.push_back(pose);
  }

  N3KeyframeLite keyframe;
  keyframe.id = 1;
  keyframe.timestamp = 0.0;
  keyframe.pose_optimized.translation = {0.0, 0.0, 0.0};
  keyframe.cloud_body = {
    PointXYZI{0.0, 0.0, 0.0, 1.0},
    PointXYZI{0.5, 0.0, 0.0, 1.0},
    PointXYZI{1.0, 0.0, 0.0, 1.0},
    PointXYZI{0.5, 0.5, 0.0, 1.0}};
  resource.keyframes.push_back(keyframe);

  N3NavResource missing_dense = resource;
  missing_dense.dense_trajectory.clear();
  const auto missing_dense_result = reader.validate(missing_dense);
  require(!missing_dense_result.success, "missing dense trajectory should fail validation");
  require(
    missing_dense_result.error_code == "pbstream_missing_dense_trajectory",
    "missing dense trajectory should use stable error code");

  N3NavResource fallback_dense = resource;
  fallback_dense.dense_trajectory_from_keyframe_fallback = true;
  const auto fallback_dense_result = reader.validate(fallback_dense);
  require(!fallback_dense_result.success, "keyframe fallback dense trajectory should fail");
  require(
    fallback_dense_result.error_code ==
    "pbstream_dense_trajectory_from_fallback_not_allowed",
    "fallback dense trajectory should use stable error code");

  N3NavResource degraded_dense = resource;
  degraded_dense.dense_trajectory_degraded = true;
  const auto degraded_dense_result = reader.validate(degraded_dense);
  require(!degraded_dense_result.success, "degraded dense trajectory should fail");
  require(
    degraded_dense_result.error_code == "pbstream_dense_trajectory_degraded_not_allowed",
    "degraded dense trajectory should use stable error code");

  N3NavResource empty_cloud = resource;
  empty_cloud.keyframes.front().cloud_body.clear();
  const auto empty_cloud_result = reader.validate(empty_cloud);
  require(!empty_cloud_result.success, "empty keyframe cloud should fail validation");
  require(
    empty_cloud_result.error_code == "pbstream_missing_keyframe_clouds",
    "empty keyframe cloud should use stable error code");

  TrajectoryProjectorOptions projector_options;
  projector_options.resolution_m = 0.50;
  projector_options.raw_resolution_m = 0.50;
  projector_options.search_below_min_m = 0.10;
  projector_options.search_below_max_m = 1.00;
  projector_options.max_support_jump_m = 0.30;
  projector_options.footprint_length_m = 0.10;
  projector_options.footprint_width_m = 0.10;
  projector_options.footprint_base_to_front_m = 0.05;
  projector_options.min_footprint_support_ratio = 0.50;
  projector_options.max_trajectory_bridge_gap_m = 1.00;
  projector_options.max_trajectory_bridge_height_delta_m = 0.50;
  TrajectoryProjector projector(projector_options);
  const auto projection = projector.project(resource);
  require(
    projection.projected_support_samples.size() == resource.dense_trajectory.size(),
    "trajectory samples should project onto support in the toy scene");
  require(projection.rejected_samples.empty(), "toy support projection should not reject samples");
  require(!projection.proven_seed_cells.empty(), "support projection should emit proven cells");

  N3NavResource too_high = resource;
  too_high.dense_trajectory.front().pose_world_lidar.translation.z = 2.0;
  const auto rejected_projection = projector.project(too_high);
  require(
    !rejected_projection.rejected_samples.empty(),
    "support outside the vertical band should be rejected");

  N3NavResource step_resource = resource;
  step_resource.dense_trajectory.clear();
  N3TrajectoryPose lower_pose;
  lower_pose.seq = 1;
  lower_pose.timestamp = 1.0;
  lower_pose.pose_world_lidar.translation = {0.0, 0.0, 0.5};
  step_resource.dense_trajectory.push_back(lower_pose);
  N3TrajectoryPose upper_pose;
  upper_pose.seq = 2;
  upper_pose.timestamp = 2.0;
  upper_pose.pose_world_lidar.translation = {4.0, 0.0, 1.0};
  step_resource.dense_trajectory.push_back(upper_pose);
  N3KeyframeLite upper_keyframe;
  upper_keyframe.id = 2;
  upper_keyframe.timestamp = 1.0;
  upper_keyframe.pose_optimized.translation = {0.0, 0.0, 0.0};
  upper_keyframe.cloud_body = {PointXYZI{4.0, 0.0, 0.5, 1.0}};
  step_resource.keyframes.push_back(upper_keyframe);
  const auto reanchored_projection = projector.project(step_resource);
  require(
    reanchored_projection.reanchored_support_samples == 1U,
    "support projection should reanchor when the previous support layer disappears");
  require(
    reanchored_projection.rejected_samples.empty(),
    "support reanchor should avoid a false multifloor rejection");

  N3NavResource bridge_resource = resource;
  bridge_resource.dense_trajectory.clear();
  N3TrajectoryPose bridge_start;
  bridge_start.seq = 1;
  bridge_start.timestamp = 1.0;
  bridge_start.pose_world_lidar.translation = {0.0, 0.0, 0.5};
  bridge_resource.dense_trajectory.push_back(bridge_start);
  N3TrajectoryPose bridge_end;
  bridge_end.seq = 2;
  bridge_end.timestamp = 2.0;
  bridge_end.pose_world_lidar.translation = {0.8, 0.0, 0.5};
  bridge_resource.dense_trajectory.push_back(bridge_end);
  const auto bridged_projection = projector.project(bridge_resource);
  require(
    bridged_projection.trajectory_bridge_seed_count > 0U,
    "short support gaps between walked trajectory samples should emit bridge seeds");

  ExperienceSurfaceBuilderOptions options;
  options.resolution_m = 0.50;
  options.projector.raw_resolution_m = 0.50;
  options.projector.footprint_length_m = 0.10;
  options.projector.footprint_width_m = 0.10;
  options.projector.footprint_base_to_front_m = 0.05;
  options.expander.expansion_radius_cells = 1;
  options.expander.max_expansion_steps = 1;
  options.expander.vertical_tolerance_cells = 0;

  ExperienceSurfaceBuilder builder(options);
  const auto build_result = builder.build(resource);
  require(build_result.success, "experience builder should accept a minimal valid resource");
  require(build_result.build_time_ms > 0.0, "experience builder should report build time");
  require(build_result.geometry_cell_count > 0U, "experience builder should keep geometry cells");
  require(build_result.proven_seed_count > 0U, "experience builder should project trajectory seeds");
  require(
    !build_result.snapshot.surface.traversable_cells.empty(),
    "experience builder should emit traversable cells");
}

void testReachableExpansionHeightGate()
{
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> seeds;
  seeds.insert({0, 0, 0});

  std::unordered_map<GridIndex, SurfaceCell, tgw_planner::core::GridIndexHash> geometry;
  SurfaceCell seed;
  seed.cell = {0, 0, 0};
  seed.height_m = 0.0;
  geometry[seed.cell] = seed;

  SurfaceCell low_neighbor;
  low_neighbor.cell = {1, 0, 0};
  low_neighbor.height_m = 0.10;
  geometry[low_neighbor.cell] = low_neighbor;

  SurfaceCell high_neighbor;
  high_neighbor.cell = {0, 1, 0};
  high_neighbor.height_m = 1.00;
  geometry[high_neighbor.cell] = high_neighbor;

  ReachableExpanderOptions options;
  options.expansion_radius_cells = 1;
  options.max_expansion_steps = 1;
  options.vertical_tolerance_cells = 0;
  options.max_expansion_step_height_m = 0.20;
  options.experience_anchor_height_tolerance_m = 2.0;
  ReachableExpander expander(options);
  const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> no_bridge_seeds;
  const auto expanded = expander.expand(seeds, no_bridge_seeds, geometry);

  require(
    expanded.traversable_cells.find(low_neighbor.cell) != expanded.traversable_cells.end(),
    "low height neighbor should pass conservative expansion");
  require(
    expanded.traversable_cells.find(high_neighbor.cell) == expanded.traversable_cells.end(),
    "high step neighbor should be rejected by expansion height gate");
  require(
    expanded.rejected_expansion_count > 0U,
    "height-gated expansion should report rejected candidates");
}

void testReachableExpansionRejectsBodyObstruction()
{
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> seeds;
  seeds.insert({0, 0, 0});

  std::unordered_map<GridIndex, SurfaceCell, tgw_planner::core::GridIndexHash> geometry;
  SurfaceCell seed;
  seed.cell = {0, 0, 0};
  seed.height_m = 0.0;
  geometry[seed.cell] = seed;

  SurfaceCell floor_at_wall;
  floor_at_wall.cell = {1, 0, 0};
  floor_at_wall.height_m = 0.0;
  geometry[floor_at_wall.cell] = floor_at_wall;
  for (int z = 1; z <= 5; ++z) {
    SurfaceCell wall;
    wall.cell = {1, 0, z};
    wall.height_m = 0.1 * static_cast<double>(z);
    geometry[wall.cell] = wall;
  }

  ReachableExpanderOptions options;
  options.resolution_m = 0.10;
  options.expansion_radius_cells = 1;
  options.max_expansion_steps = 1;
  options.vertical_tolerance_cells = 0;
  options.max_expansion_step_height_m = 0.20;
  options.body_clearance_cells = 5;
  options.enable_hole_filling = false;
  ReachableExpander expander(options);
  const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> no_bridge_seeds;
  const auto expanded = expander.expand(seeds, no_bridge_seeds, geometry);

  require(
    expanded.traversable_cells.find(floor_at_wall.cell) == expanded.traversable_cells.end(),
    "floor cell with body-height wall occupancy above should not be expanded");
  require(
    expanded.body_obstructed_rejected_count > 0U,
    "body obstruction rejection should be counted");
}

void testReachableExpansionFillsSmallHole()
{
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> seeds;
  std::unordered_map<GridIndex, SurfaceCell, tgw_planner::core::GridIndexHash> geometry;
  for (int x = 0; x <= 2; ++x) {
    for (int y = 0; y <= 2; ++y) {
      if (x == 1 && y == 1) {
        continue;
      }
      const GridIndex cell{x, y, 0};
      seeds.insert(cell);
      SurfaceCell surface_cell;
      surface_cell.cell = cell;
      surface_cell.height_m = 0.0;
      geometry[cell] = surface_cell;
    }
  }

  ReachableExpanderOptions options;
  options.resolution_m = 0.10;
  options.expansion_radius_cells = 0;
  options.max_expansion_steps = 0;
  options.vertical_tolerance_cells = 0;
  options.enable_hole_filling = true;
  options.hole_fill_iterations = 1;
  options.min_hole_fill_neighbors = 5;
  options.max_hole_fill_height_spread_m = 0.05;
  ReachableExpander expander(options);
  const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> no_bridge_seeds;
  const auto expanded = expander.expand(seeds, no_bridge_seeds, geometry);

  const GridIndex hole{1, 1, 0};
  require(
    expanded.traversable_cells.find(hole) != expanded.traversable_cells.end(),
    "small surrounded floor hole should be filled");
  require(expanded.hole_filled_count == 1U, "hole fill should report one added cell");
}

void testReachableExpansionRejectsAnchorEnvelopeEscape()
{
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> seeds;
  seeds.insert({0, 0, 0});

  std::unordered_map<GridIndex, SurfaceCell, tgw_planner::core::GridIndexHash> geometry;
  SurfaceCell seed;
  seed.cell = {0, 0, 0};
  seed.height_m = 0.0;
  geometry[seed.cell] = seed;

  SurfaceCell ceiling;
  ceiling.cell = {1, 0, 10};
  ceiling.height_m = 1.0;
  geometry[ceiling.cell] = ceiling;

  ReachableExpanderOptions options;
  options.resolution_m = 0.10;
  options.expansion_radius_cells = 1;
  options.max_expansion_steps = 1;
  options.vertical_tolerance_cells = 10;
  options.max_expansion_step_height_m = 2.0;
  options.experience_anchor_radius_cells = 2;
  options.experience_anchor_height_tolerance_m = 0.20;
  options.enable_hole_filling = false;
  ReachableExpander expander(options);
  const std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> no_bridge_seeds;
  const auto expanded = expander.expand(seeds, no_bridge_seeds, geometry);

  require(
    expanded.traversable_cells.find(ceiling.cell) == expanded.traversable_cells.end(),
    "expansion should not escape the trajectory height envelope onto a ceiling");
  require(
    expanded.anchor_envelope_rejected_count > 0U,
    "anchor envelope rejection should be counted");
}

void testBridgeSeedsDoNotAnchorExpansion()
{
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> observed;
  observed.insert({0, 0, 0});
  std::unordered_set<GridIndex, tgw_planner::core::GridIndexHash> bridge;
  bridge.insert({3, 0, 0});

  std::unordered_map<GridIndex, SurfaceCell, tgw_planner::core::GridIndexHash> geometry;
  SurfaceCell observed_cell;
  observed_cell.cell = {0, 0, 0};
  observed_cell.height_m = 0.0;
  geometry[observed_cell.cell] = observed_cell;

  SurfaceCell bridge_neighbor;
  bridge_neighbor.cell = {4, 0, 0};
  bridge_neighbor.height_m = 0.0;
  geometry[bridge_neighbor.cell] = bridge_neighbor;

  ReachableExpanderOptions options;
  options.resolution_m = 0.10;
  options.expansion_radius_cells = 1;
  options.max_expansion_steps = 10;
  options.vertical_tolerance_cells = 0;
  options.experience_anchor_radius_cells = 20;
  options.experience_anchor_vertical_tolerance_cells = 0;
  options.enable_hole_filling = false;
  ReachableExpander expander(options);
  const auto expanded = expander.expand(observed, bridge, geometry);

  require(
    expanded.traversable_cells.find({3, 0, 0}) != expanded.traversable_cells.end(),
    "bridge corridor cell should be added to traversable output");
  require(
    expanded.traversable_cells.find(bridge_neighbor.cell) == expanded.traversable_cells.end(),
    "bridge seed should not expand into neighboring geometry");
  require(
    expanded.bridge_used_as_expansion_anchor == 0U,
    "bridge cells must never be counted as expansion anchors");
}
}  // namespace

int main()
{
  testClearanceAndRisk();
  testFootprintPlannerAndValidator();
  testExperienceBuilderSkeleton();
  testReachableExpansionHeightGate();
  testReachableExpansionRejectsBodyObstruction();
  testReachableExpansionFillsSmallHole();
  testReachableExpansionRejectsAnchorEnvelopeEscape();
  testBridgeSeedsDoNotAnchorExpansion();
  std::cout << "PASS experience_core_smoke" << std::endl;
  return 0;
}
