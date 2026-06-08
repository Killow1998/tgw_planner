#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/path_validator.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

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
using tgw_planner::core::RiskField;
using tgw_planner::core::RobotFootprint;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfaceCell;
using tgw_planner::core::SurfaceLabel;
using tgw_planner::core::SurfaceMap;
using tgw_planner::core::SurfacePlannerOptions;

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
    pose.pose_world_lidar.translation = {0.5 * static_cast<double>(i), 0.0, 0.0};
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

  ExperienceSurfaceBuilderOptions options;
  options.resolution_m = 0.50;
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
}  // namespace

int main()
{
  testClearanceAndRisk();
  testFootprintPlannerAndValidator();
  testExperienceBuilderSkeleton();
  std::cout << "PASS experience_core_smoke" << std::endl;
  return 0;
}
