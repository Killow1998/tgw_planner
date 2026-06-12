#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/local_path_smoother.hpp"
#include "tgw_planner/core/n3map_reader.hpp"
#include "tgw_planner/core/path_validator.hpp"
#include "tgw_planner/core/regulated_pure_pursuit.hpp"
#include "tgw_planner/core/reachable_expander.hpp"
#include "tgw_planner/core/rolling_local_map.hpp"
#include "tgw_planner/core/route_progress_tracker.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace
{
using tgw_planner::core::ClearanceField;
using tgw_planner::core::ExperienceSnapshot;
using tgw_planner::core::ExperienceBackboneGraph;
using tgw_planner::core::ExperienceBackboneOptions;
using tgw_planner::core::ExperienceSurfaceGraph;
using tgw_planner::core::ExperienceSurfaceBuilder;
using tgw_planner::core::ExperienceSurfaceBuilderOptions;
using tgw_planner::core::GridIndex;
using tgw_planner::core::GlobalPathPoint;
using tgw_planner::core::HybridExperiencePlanner;
using tgw_planner::core::HybridExperiencePlannerOptions;
using tgw_planner::core::LocalPathResult;
using tgw_planner::core::LocalPathSmoother;
using tgw_planner::core::LocalPathSmootherOptions;
using tgw_planner::core::N3KeyframeLite;
using tgw_planner::core::N3MapReader;
using tgw_planner::core::N3NavResource;
using tgw_planner::core::N3TrajectoryPose;
using tgw_planner::core::PathValidationOptions;
using tgw_planner::core::PathValidator;
using tgw_planner::core::PathPointKind;
using tgw_planner::core::Point3;
using tgw_planner::core::PointXYZI;
using tgw_planner::core::Pose3;
using tgw_planner::core::ProjectedSupportSample;
using tgw_planner::core::ReachabilityLabel;
using tgw_planner::core::ReachableExpander;
using tgw_planner::core::ReachableExpanderOptions;
using tgw_planner::core::RiskField;
using tgw_planner::core::RobotFootprint;
using tgw_planner::core::RobotFootprintOptions;
using tgw_planner::core::RegulatedPurePursuitController;
using tgw_planner::core::RegulatedPurePursuitOptions;
using tgw_planner::core::RollingLocalMap;
using tgw_planner::core::RoutePose2D;
using tgw_planner::core::RouteProgressState;
using tgw_planner::core::RouteProgressTracker;
using tgw_planner::core::RouteProgressTrackerOptions;
using tgw_planner::core::SurfaceAstarPlanner;
using tgw_planner::core::SurfaceCell;
using tgw_planner::core::SurfaceGraphBuildOptions;
using tgw_planner::core::SurfaceLabel;
using tgw_planner::core::SurfaceMap;
using tgw_planner::core::SurfaceNodeId;
using tgw_planner::core::SurfacePlannerOptions;
using tgw_planner::core::SurfaceTransitionValidator;
using tgw_planner::core::TrajectoryBridgeSegment;
using tgw_planner::core::TrajectoryProjectionResult;
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
      surface_cell.support_component_id = 1;
      surface_cell.surface_layer_id = 1;
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

void testFootprintSupportRatio()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0) {
        continue;
      }
      const GridIndex cell{x, y, 0};
      SurfaceCell surface_cell;
      surface_cell.cell = cell;
      surface_cell.label = SurfaceLabel::Expanded;
      surface_cell.reachability = ReachabilityLabel::InferredReachable;
      snapshot.surface.surface_cells[cell] = surface_cell;
      snapshot.surface.traversable_cells.insert(cell);
    }
  }

  RobotFootprintOptions options;
  options.length_m = 0.60;
  options.width_m = 0.40;
  options.min_support_ratio = 0.75;
  RobotFootprint footprint(options);
  const auto report = footprint.supportReport(snapshot.surface, {0.0, 0.0, 0.25}, 0.0, 0.50);
  require(report.total_samples > report.supported_samples, "one missing footprint sample should be counted");
  require(report.support_ratio >= 0.75, "support ratio should tolerate a small sparse hole");
  require(
    footprint.isSupported(snapshot.surface, {0.0, 0.0, 0.25}, 0.0, 0.50),
    "footprint ratio check should accept sparse but mostly supported footprint");
}

GlobalPathPoint makeGlobalPathPoint(
  double x, double y, double z, PathPointKind kind, double target_speed_mps)
{
  GlobalPathPoint point;
  point.position = {x, y, z};
  point.kind = kind;
  point.target_speed_mps = target_speed_mps;
  point.confidence = 1.0;
  return point;
}

void testRouteProgressTrackerBuildsLocalWindowWithoutLayerJump()
{
  RouteProgressTrackerOptions options;
  options.projection_window_m = 2.0;
  options.local_route_length_m = 3.0;
  RouteProgressTracker tracker(options);
  std::vector<GlobalPathPoint> path{
    makeGlobalPathPoint(0.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(10.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(10.0, 0.0, 8.0, PathPointKind::Portal, 0.0),
    makeGlobalPathPoint(0.0, 0.0, 8.0, PathPointKind::Surface, 0.0)};
  require(tracker.setPath(path), "route progress tracker should accept overlapping XY path");

  const RouteProgressState state = tracker.update({0.10, 0.0, 0.0});
  require(state.valid, "route progress tracker state should be valid");
  require(state.progress_m < 1.0, "route progress should stay near the start");
  require(
    state.projected_point.position.z < 1.0,
    "route progress should not jump to a later overlapping height layer");
  require(state.local_route.size() >= 2U, "route progress should expose a local route window");
}

void testRouteProgressTrackerRejectsFarProjection()
{
  RouteProgressTrackerOptions options;
  options.projection_window_m = 3.0;
  options.max_projection_lateral_error_m = 0.75;
  RouteProgressTracker tracker(options);
  std::vector<GlobalPathPoint> path{
    makeGlobalPathPoint(0.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(5.0, 0.0, 0.0, PathPointKind::Surface, 0.0)};
  require(tracker.setPath(path), "route tracker should accept straight path");

  const RouteProgressState state = tracker.update({1.0, 2.0, 0.0});
  require(!state.valid, "route tracker should reject poses far from the route");
  require(
    state.status == "route_projection_failed",
    "route tracker should expose projection failure as the stop reason");
}

void testLocalPathSmootherBuildsShortSmoothPath()
{
  RouteProgressTracker tracker;
  std::vector<GlobalPathPoint> path{
    makeGlobalPathPoint(0.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(1.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(1.0, 1.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(2.0, 1.0, 0.0, PathPointKind::Surface, 0.0)};
  require(tracker.setPath(path), "route tracker should accept corner path");
  const RouteProgressState route = tracker.update({0.0, 0.0, 0.0});

  LocalPathSmootherOptions options;
  options.enable_collision_check = true;
  LocalPathSmoother smoother(options);
  RollingLocalMap local_map;
  const LocalPathResult local_path = smoother.build({0.0, 0.0, 0.0}, route, local_map);
  require(local_path.success, "local path smoother should build a no-obstacle local path");
  require(local_path.path.size() > route.local_route.size(), "local path should be smoothed/resampled");
  require(
    local_path.smoothness_rad_per_m <= options.max_smoothness_rad_per_m,
    "local path smoothness should be bounded");
  require(
    local_path.max_turn_angle_rad <= options.max_turn_angle_rad,
    "local path max turn should be bounded");
}

void testLocalPathSmootherDoesNotCutAcrossCorner()
{
  RouteProgressTrackerOptions tracker_options;
  tracker_options.local_route_length_m = 5.0;
  RouteProgressTracker tracker(tracker_options);
  std::vector<GlobalPathPoint> path{
    makeGlobalPathPoint(0.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(1.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(1.0, 2.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(3.0, 2.0, 0.0, PathPointKind::Surface, 0.0)};
  require(tracker.setPath(path), "route tracker should accept L-shaped path");
  const RouteProgressState route = tracker.update({0.0, 0.0, 0.0});

  LocalPathSmootherOptions options;
  options.corner_cut_turn_angle_rad = 0.50;
  options.max_route_deviation_m = 0.30;
  LocalPathSmoother smoother(options);
  const LocalPathResult local_path = smoother.build({0.0, 0.0, 0.0}, route, RollingLocalMap());

  require(local_path.success, "corner-constrained local path should be valid");
  require(!local_path.path.empty(), "corner-constrained local path should contain points");
  const Point3 & end = local_path.path.back();
  require(end.x > 0.80 && end.x < 1.20, "local target should stop at the first corner x");
  require(std::abs(end.y) < 0.20, "local target should not cut past the first corner y");
  for (const Point3 & point : local_path.path) {
    require(
      point.y < 0.35,
      "local path should stay inside the validated corridor before the first corner");
  }
}

void testLocalPathSmootherFallsBackToRouteWhenBezierLeavesCorridor()
{
  RouteProgressTrackerOptions tracker_options;
  tracker_options.local_route_length_m = 3.0;
  RouteProgressTracker tracker(tracker_options);
  std::vector<GlobalPathPoint> path{
    makeGlobalPathPoint(0.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(1.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(2.0, 0.0, 0.0, PathPointKind::Surface, 0.0),
    makeGlobalPathPoint(3.0, 0.0, 0.0, PathPointKind::Surface, 0.0)};
  require(tracker.setPath(path), "route tracker should accept straight fallback path");
  const RouteProgressState route = tracker.update({0.0, 0.0, 1.57079632679});

  LocalPathSmootherOptions options;
  options.max_route_deviation_m = 0.10;
  options.bezier_handle_ratio = 0.80;
  LocalPathSmoother smoother(options);
  const LocalPathResult local_path = smoother.build({0.0, 0.0, 1.57079632679}, route, RollingLocalMap());

  require(local_path.success, "local smoother should fall back instead of stopping tracking");
  require(
    local_path.message == "ok_route_following_fallback_after_local_path_outside_global_corridor",
    "local smoother should expose the route-following fallback reason");
  require(!local_path.path.empty(), "fallback local path should contain route points");
  for (const Point3 & point : local_path.path) {
    require(std::abs(point.y) < 1.0e-6, "fallback should stay on the original route corridor");
  }
}

void testRegulatedPurePursuitCruisesOnStraightPath()
{
  RegulatedPurePursuitOptions options;
  options.desired_linear_speed_mps = 0.60;
  options.max_linear_speed_mps = 0.80;
  options.max_linear_accel_mps2 = 10.0;
  RegulatedPurePursuitController controller(options);
  const std::vector<Point3> local_path{{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
  const auto command = controller.computeCommand(
    {0.0, 0.0, 0.0}, local_path, 5.0, 0.1, RollingLocalMap());
  require(command.valid, "RPP command should be valid on a straight path");
  require(std::abs(command.linear_speed_mps - 0.60) < 1.0e-6, "RPP should cruise at desired speed");
  require(std::abs(command.yaw_rate_radps) < 1.0e-6, "RPP should not turn on a straight path");
}

void testRegulatedPurePursuitSlowsForCurvature()
{
  RegulatedPurePursuitOptions options;
  options.desired_linear_speed_mps = 1.00;
  options.max_linear_speed_mps = 1.00;
  options.max_lateral_accel_mps2 = 0.10;
  options.max_linear_accel_mps2 = 10.0;
  RegulatedPurePursuitController controller(options);
  const std::vector<Point3> local_path{{0.0, 0.0, 0.0}, {0.6, 0.6, 0.0}, {1.0, 1.0, 0.0}};
  const auto command = controller.computeCommand(
    {0.0, 0.0, 0.0}, local_path, 5.0, 0.1, RollingLocalMap());
  require(command.valid, "RPP command should be valid on a curved path");
  require(command.linear_speed_mps < 1.0, "RPP should regulate speed on high curvature");
  require(std::abs(command.yaw_rate_radps) > 1.0e-3, "RPP should turn on a curved path");
}

void testRegulatedPurePursuitSlowsNearGoal()
{
  RegulatedPurePursuitOptions options;
  options.desired_linear_speed_mps = 0.80;
  options.max_linear_speed_mps = 0.80;
  options.goal_slowdown_distance_m = 1.0;
  options.max_linear_accel_mps2 = 10.0;
  RegulatedPurePursuitController controller(options);
  const std::vector<Point3> local_path{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}};
  const auto command = controller.computeCommand(
    {0.0, 0.0, 0.0}, local_path, 0.25, 0.1, RollingLocalMap());
  require(command.valid, "RPP command should be valid near the goal");
  require(command.linear_speed_mps < 0.30, "RPP should slow down near the goal");
}

void testRegulatedPurePursuitGoalToleranceIsDistance()
{
  RegulatedPurePursuitOptions options;
  options.desired_linear_speed_mps = 0.60;
  options.max_linear_speed_mps = 0.80;
  options.min_linear_speed_mps = 0.50;
  options.goal_tolerance_m = 0.05;
  options.goal_slowdown_distance_m = 1.0;
  options.max_linear_accel_mps2 = 10.0;
  RegulatedPurePursuitController controller(options);
  const std::vector<Point3> local_path{{0.0, 0.0, 0.0}, {0.8, 0.0, 0.0}};
  const auto command = controller.computeCommand(
    {0.0, 0.0, 0.0}, local_path, 0.20, 0.1, RollingLocalMap());
  require(command.valid, "RPP command should remain valid above goal tolerance");
  require(!command.goal_reached, "RPP should not use min speed as a goal distance threshold");
}

void testRegulatedPurePursuitStopsForLocalObstacle()
{
  RegulatedPurePursuitOptions options;
  options.desired_linear_speed_mps = 0.50;
  options.max_linear_accel_mps2 = 10.0;
  RegulatedPurePursuitController controller(options);
  RollingLocalMap local_map;
  local_map.setObstaclePoints({{0.4, 0.0, 0.0}});
  const std::vector<Point3> local_path{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
  const auto command = controller.computeCommand(
    {0.0, 0.0, 0.0}, local_path, 5.0, 0.1, local_map);
  require(command.valid, "RPP command should remain valid when blocked");
  require(command.linear_speed_mps == 0.0, "RPP should stop when the local collision arc is blocked");
  require(command.status == "collision_arc_blocked", "RPP should expose the collision stop reason");
}

void testRollingLocalMapExpiresOldObstacles()
{
  tgw_planner::core::RollingLocalMapOptions options;
  options.time_window_s = 1.0;
  options.max_obstacle_points = 10;
  RollingLocalMap local_map(options);
  local_map.addObstaclePoints({{0.4, 0.0, 0.0}}, 0.0);
  require(local_map.obstacleCount() == 1U, "rolling local map should store obstacle points");
  require(
    !local_map.checkPoint({0.4, 0.0, 0.0}).collision_free,
    "rolling local map should report a current obstacle collision");
  local_map.prune(2.0);
  require(local_map.obstacleCount() == 0U, "rolling local map should expire old obstacle points");
  require(
    local_map.checkPoint({0.4, 0.0, 0.0}).collision_free,
    "expired obstacle should not block collision checks");
}

void testLayeredSurfaceGraph()
{
  ExperienceSnapshot snapshot = makeFlatSnapshot();

  for (int x = 0; x <= 6; ++x) {
    for (int y = -1; y <= 1; ++y) {
      const GridIndex cell{x, y, 4};
      SurfaceCell surface_cell;
      surface_cell.cell = cell;
      surface_cell.label = SurfaceLabel::Expanded;
      surface_cell.reachability = ReachabilityLabel::InferredReachable;
      surface_cell.support_component_id = 2;
      surface_cell.surface_layer_id = 2;
      surface_cell.height_m = 2.0;
      surface_cell.confidence = 1.0;
      snapshot.surface.surface_cells[cell] = surface_cell;
      snapshot.surface.traversable_cells.insert(cell);
    }
  }
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = true;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator);

  const auto xy_it = graph.xyToNodes().find({2, 0, 0});
  require(xy_it != graph.xyToNodes().end(), "surface graph should index nodes by XY");
  require(
    xy_it->second.size() >= 2U,
    "same XY with two valid surface heights should keep separate graph nodes");

  const auto plan = SurfaceAstarPlanner(planner_options).plan(snapshot, {1, 0, 0}, {5, 0, 0});
  require(plan.success, "graph-backed planner should succeed on a flat corridor");
  require(plan.metrics.final_path_validated, "graph-backed planner should validate graph edges");

  for (int y = -1; y <= 1; ++y) {
    snapshot.surface.traversable_cells.erase({3, y, 0});
    snapshot.surface.surface_cells.erase({3, y, 0});
  }
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);
  const auto disconnected_plan =
    SurfaceAstarPlanner(planner_options).plan(snapshot, {1, 0, 0}, {5, 0, 0});
  require(!disconnected_plan.success, "graph-backed planner should fail across graph components");
  require(
    disconnected_plan.message == "no_path_on_experience_surface_different_components",
    "graph-backed planner should fail before A* when components differ");
}

void testSurfaceGraphRejectsLayerJump()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  SurfaceCell lower;
  lower.cell = {0, 0, 0};
  lower.label = SurfaceLabel::Expanded;
  lower.reachability = ReachabilityLabel::InferredReachable;
  lower.support_component_id = 1;
  lower.surface_layer_id = 1;
  lower.height_m = 0.0;
  lower.confidence = 1.0;
  snapshot.surface.surface_cells[lower.cell] = lower;
  snapshot.surface.traversable_cells.insert(lower.cell);

  SurfaceCell upper = lower;
  upper.cell = {1, 0, 0};
  upper.height_m = 2.0;
  snapshot.surface.surface_cells[upper.cell] = upper;
  snapshot.surface.traversable_cells.insert(upper.cell);

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  planner_options.max_step_height_m = 0.35;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator);

  const SurfaceNodeId lower_node = graph.nodeIdForCell(lower.cell);
  const SurfaceNodeId upper_node = graph.nodeIdForCell(upper.cell);
  require(graph.isValid(lower_node), "lower layer jump test node should exist");
  require(graph.isValid(upper_node), "upper layer jump test node should exist");
  require(
    !graph.sameComponent(lower_node, upper_node),
    "surface graph must reject adjacent XY edges with large true surface-height jumps");
}

void testSurfaceGraphBridgeHeightPolicy()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  auto add_bridge_cell = [&](const GridIndex & cell, double height_m) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::TrajectoryBridge;
    surface_cell.reachability = ReachabilityLabel::LowConfidenceReachable;
    surface_cell.bridge_id = 7;
    surface_cell.bridge_order = cell.x;
    surface_cell.bridge_endpoint = cell.x == 0 || cell.x == 2;
    surface_cell.height_m = height_m;
    surface_cell.confidence = 0.30;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  add_bridge_cell({0, 0, 0}, 0.0);
  add_bridge_cell({1, 0, 0}, 0.60);
  add_bridge_cell({2, 0, 0}, 1.80);

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  planner_options.max_step_height_m = 0.35;
  SurfaceTransitionValidator validator(planner_options);
  SurfaceGraphBuildOptions graph_options;
  graph_options.max_edge_height_delta_m = planner_options.max_step_height_m;
  graph_options.max_bridge_edge_height_delta_m = 0.80;

  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator, graph_options);

  const SurfaceNodeId bridge_a = graph.nodeIdForCell({0, 0, 0});
  const SurfaceNodeId bridge_b = graph.nodeIdForCell({1, 0, 0});
  const SurfaceNodeId bridge_too_high = graph.nodeIdForCell({2, 0, 0});
  require(graph.isValid(bridge_a), "first bridge policy test node should exist");
  require(graph.isValid(bridge_b), "second bridge policy test node should exist");
  require(graph.isValid(bridge_too_high), "third bridge policy test node should exist");
  require(
    graph.sameComponent(bridge_a, bridge_b),
    "trajectory bridge edge should allow a bounded bridge-height change");
  require(
    !graph.sameComponent(bridge_b, bridge_too_high),
    "trajectory bridge edge should reject jumps beyond the bridge-height policy");
}

void testLowConfidenceIsNotBridge()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  SurfaceCell a;
  a.cell = {0, 0, 0};
  a.label = SurfaceLabel::Expanded;
  a.reachability = ReachabilityLabel::LowConfidenceReachable;
  a.support_component_id = 1;
  a.surface_layer_id = 9;
  a.height_m = 0.0;
  a.confidence = 0.35;
  snapshot.surface.surface_cells[a.cell] = a;
  snapshot.surface.traversable_cells.insert(a.cell);
  snapshot.reachability[a.cell] = a.reachability;

  SurfaceCell b = a;
  b.cell = {1, 0, 0};
  b.support_component_id = 2;
  b.surface_layer_id = 9;
  snapshot.surface.surface_cells[b.cell] = b;
  snapshot.surface.traversable_cells.insert(b.cell);
  snapshot.reachability[b.cell] = b.reachability;

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator);

  const SurfaceNodeId a_node = graph.nodeIdForCell(a.cell);
  const SurfaceNodeId b_node = graph.nodeIdForCell(b.cell);
  require(graph.isValid(a_node), "first low confidence node should exist");
  require(graph.isValid(b_node), "second low confidence node should exist");
  require(!graph.node(a_node)->bridge, "low confidence node should not become a bridge");
  require(!graph.node(b_node)->bridge, "low confidence node should not become a bridge");
  require(
    graph.sameComponent(a_node, b_node),
    "normal surface edges should use surface layer ids, not support lineage ids");
  bool found_penalized_edge = false;
  for (const auto & edge : graph.adjacency()[a_node.id]) {
    if (edge.to == b_node && edge.cost > edge.length_xy_m + 2.0) {
      found_penalized_edge = true;
      break;
    }
  }
  require(
    found_penalized_edge,
    "surface graph edge cost should include low-confidence safety penalties");
}

void testSurfaceGraphRejectsMissingSupportLineage()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  SurfaceCell anchored;
  anchored.cell = {0, 0, 0};
  anchored.label = SurfaceLabel::Expanded;
  anchored.reachability = ReachabilityLabel::InferredReachable;
  anchored.support_component_id = 1;
  anchored.surface_layer_id = 1;
  anchored.height_m = 0.0;
  anchored.confidence = 1.0;
  snapshot.surface.surface_cells[anchored.cell] = anchored;
  snapshot.surface.traversable_cells.insert(anchored.cell);
  snapshot.reachability[anchored.cell] = anchored.reachability;

  SurfaceCell unanchored = anchored;
  unanchored.cell = {1, 0, 0};
  unanchored.support_component_id = -1;
  unanchored.surface_layer_id = -1;
  snapshot.surface.surface_cells[unanchored.cell] = unanchored;
  snapshot.surface.traversable_cells.insert(unanchored.cell);
  snapshot.reachability[unanchored.cell] = unanchored.reachability;

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator);

  const SurfaceNodeId anchored_node = graph.nodeIdForCell(anchored.cell);
  const SurfaceNodeId unanchored_node = graph.nodeIdForCell(unanchored.cell);
  require(graph.isValid(anchored_node), "anchored support lineage node should exist");
  require(graph.isValid(unanchored_node), "unanchored support lineage node should exist");
  require(
    !graph.sameComponent(anchored_node, unanchored_node),
    "normal graph edges should reject cells without anchored support lineage");
}

void testBridgeEndpointAttachesOnlyToIntendedComponent()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  auto add_normal_cell = [&](const GridIndex & cell, int component_id) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::Expanded;
    surface_cell.reachability = ReachabilityLabel::InferredReachable;
    surface_cell.support_component_id = component_id;
    surface_cell.surface_layer_id = component_id;
    surface_cell.height_m = 0.0;
    surface_cell.confidence = 1.0;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };
  auto add_bridge_cell = [&](const GridIndex & cell, int order, bool endpoint) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::TrajectoryBridge;
    surface_cell.reachability = ReachabilityLabel::LowConfidenceReachable;
    surface_cell.bridge_id = 11;
    surface_cell.bridge_order = order;
    surface_cell.bridge_endpoint = endpoint;
    surface_cell.height_m = 0.0;
    surface_cell.confidence = 0.30;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  add_normal_cell({-1, 0, 0}, 101);
  add_normal_cell({2, 0, 0}, 202);
  add_normal_cell({0, 1, 0}, 303);
  add_bridge_cell({0, 0, 0}, 1, true);
  add_bridge_cell({1, 0, 0}, 2, true);

  TrajectoryBridgeSegment segment;
  segment.bridge_id = 11;
  segment.entry_support_cell = {-1, 0, 0};
  segment.exit_support_cell = {2, 0, 0};
  segment.footprint_cells_ordered = {{0, 0, 0}, {1, 0, 0}};
  segment.cell_order[{0, 0, 0}] = 1;
  segment.cell_order[{1, 0, 0}] = 2;
  segment.gap_length_m = 1.0;
  segment.height_delta_m = 0.0;
  snapshot.bridge_segments.push_back(segment);

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph graph;
  graph.build(snapshot, validator);

  const SurfaceNodeId entry = graph.nodeIdForCell({-1, 0, 0});
  const SurfaceNodeId bridge = graph.nodeIdForCell({0, 0, 0});
  const SurfaceNodeId wrong = graph.nodeIdForCell({0, 1, 0});
  require(graph.isValid(entry), "bridge attach test entry node should exist");
  require(graph.isValid(bridge), "bridge attach test bridge node should exist");
  require(graph.isValid(wrong), "bridge attach test wrong normal node should exist");
  require(
    graph.sameComponent(entry, bridge),
    "bridge endpoint should attach to its intended entry support component");
  require(
    !graph.sameComponent(wrong, bridge),
    "bridge endpoint should not attach to an unrelated adjacent support component");
}

void testHybridPlannerUsesDenseTrajectoryBackboneAcrossIslands()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 0.50;

  auto add_surface_cell = [&](const GridIndex & cell, double height_m, int support_component_id) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::Expanded;
    surface_cell.reachability = ReachabilityLabel::InferredReachable;
    surface_cell.support_component_id = support_component_id;
    surface_cell.surface_layer_id = support_component_id;
    surface_cell.height_m = height_m;
    surface_cell.confidence = 1.0;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  for (int x = 0; x <= 2; ++x) {
    add_surface_cell({x, 0, 0}, 0.0, 1);
  }
  for (int x = 8; x <= 10; ++x) {
    add_surface_cell({x, 0, 4}, 2.0, 2);
  }

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph surface_graph;
  surface_graph.build(snapshot, validator);

  const SurfaceNodeId start = surface_graph.nodeIdForCell({0, 0, 0});
  const SurfaceNodeId goal = surface_graph.nodeIdForCell({10, 0, 4});
  require(surface_graph.isValid(start), "hybrid test start surface node should exist");
  require(surface_graph.isValid(goal), "hybrid test goal surface node should exist");
  require(
    !surface_graph.sameComponent(start, goal),
    "surface islands should remain disconnected without dense trajectory topology");

  N3NavResource resource;
  resource.map_frame = "map";
  resource.body_frame = "base";
  resource.has_native_dense_trajectory = true;
  resource.dense_trajectory_source = "native";

  TrajectoryProjectionResult projection;
  for (int i = 0; i <= 10; ++i) {
    const double t = static_cast<double>(i) / 10.0;
    const Point3 support{
      0.25 + 5.0 * t,
      0.25,
      2.0 * t};
    N3TrajectoryPose pose;
    pose.seq = static_cast<std::uint64_t>(i);
    pose.timestamp = static_cast<double>(i);
    pose.pose_world_lidar.translation = {support.x, support.y, support.z + 0.50};
    resource.dense_trajectory.push_back(pose);

    ProjectedSupportSample sample;
    sample.seq = pose.seq;
    sample.timestamp = pose.timestamp;
    sample.trajectory_position = pose.pose_world_lidar.translation;
    sample.support_position = support;
    projection.accepted_projected_support_samples.push_back(sample);
  }

  ExperienceBackboneOptions backbone_options;
  backbone_options.min_node_spacing_m = 0.10;
  backbone_options.max_portal_xy_distance_m = 0.60;
  backbone_options.max_portal_height_error_m = 0.30;
  ExperienceBackboneGraph backbone;
  backbone.build(resource, projection, surface_graph, backbone_options);
  require(backbone.nodes().size() >= 2U, "dense trajectory should create backbone nodes");
  require(backbone.portals().size() >= 2U, "surface islands should attach to backbone portals");

  const auto plan = HybridExperiencePlanner(planner_options).plan(
    surface_graph, backbone, start, goal);
  require(plan.success, "hybrid planner should cross disconnected surface islands through backbone");
  require(
    plan.message == "path found on hybrid experience graph",
    "hybrid planner should report unified hybrid graph usage");
  require(plan.metrics.used_backbone_edges > 0U, "hybrid planner should use backbone edges");
  require(plan.metrics.used_portal_edges >= 2U, "hybrid planner should enter and leave portals");
  require(plan.path.size() > 4U, "hybrid path should include surface and backbone waypoints");
}

void testHybridPlannerOptimizesPortalPairCost()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 1.0;

  auto add_surface_cell = [&](const GridIndex & cell, double height_m, int support_component_id) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::Expanded;
    surface_cell.reachability = ReachabilityLabel::InferredReachable;
    surface_cell.support_component_id = support_component_id;
    surface_cell.surface_layer_id = support_component_id;
    surface_cell.height_m = height_m;
    surface_cell.confidence = 1.0;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  for (int x = 0; x <= 2; ++x) {
    add_surface_cell({x, 0, 0}, 0.0, 1);
  }
  for (int x = 4; x <= 6; ++x) {
    add_surface_cell({x, 0, 0}, 0.0, 2);
  }

  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  SurfaceTransitionValidator validator(planner_options);
  ExperienceSurfaceGraph surface_graph;
  surface_graph.build(snapshot, validator);

  const SurfaceNodeId start = surface_graph.nodeIdForCell({0, 0, 0});
  const SurfaceNodeId goal = surface_graph.nodeIdForCell({6, 0, 0});
  require(surface_graph.isValid(start), "portal pair test start node should exist");
  require(surface_graph.isValid(goal), "portal pair test goal node should exist");
  require(
    !surface_graph.sameComponent(start, goal),
    "portal pair test islands should be disconnected by surface graph");

  N3NavResource resource;
  resource.map_frame = "map";
  resource.body_frame = "base";
  resource.has_native_dense_trajectory = true;
  resource.dense_trajectory_source = "native";
  TrajectoryProjectionResult projection;

  auto add_backbone_sample = [&](int seq, const Point3 & support) {
    N3TrajectoryPose pose;
    pose.seq = static_cast<std::uint64_t>(seq);
    pose.timestamp = static_cast<double>(seq);
    pose.pose_world_lidar.translation = {support.x, support.y, support.z + 0.50};
    resource.dense_trajectory.push_back(pose);

    ProjectedSupportSample sample;
    sample.seq = pose.seq;
    sample.timestamp = pose.timestamp;
    sample.trajectory_position = pose.pose_world_lidar.translation;
    sample.support_position = support;
    projection.accepted_projected_support_samples.push_back(sample);
  };

  add_backbone_sample(0, {0.5, 0.5, 0.0});
  for (int i = 1; i < 49; ++i) {
    add_backbone_sample(i, {100.0 + static_cast<double>(i), 20.0, 0.0});
  }
  add_backbone_sample(49, {4.5, 0.5, 0.0});
  add_backbone_sample(50, {4.0, 0.5, 0.0});
  add_backbone_sample(51, {3.5, 0.5, 0.0});
  add_backbone_sample(52, {3.0, 0.5, 0.0});
  add_backbone_sample(53, {2.5, 0.5, 0.0});
  for (int i = 54; i < 99; ++i) {
    add_backbone_sample(i, {200.0 + static_cast<double>(i), -20.0, 0.0});
  }
  add_backbone_sample(99, {6.5, 0.5, 0.0});

  ExperienceBackboneOptions backbone_options;
  backbone_options.min_node_spacing_m = 0.0;
  backbone_options.max_portal_xy_distance_m = 0.75;
  backbone_options.max_portal_height_error_m = 0.20;
  ExperienceBackboneGraph backbone;
  backbone.build(resource, projection, surface_graph, backbone_options);
  require(backbone.portals().size() >= 4U, "portal pair test should expose multiple portals");

  HybridExperiencePlannerOptions hybrid_options;
  const auto plan = HybridExperiencePlanner(planner_options, hybrid_options).plan(
    surface_graph, backbone, start, goal);
  require(plan.success, "hybrid planner should find a route through candidate portal pairs");
  require(
    plan.metrics.used_portal_edges >= 2U,
    "hybrid graph search should use portal edges to change graph layers");
  require(
    plan.metrics.selected_backbone_index_delta <= 5U,
    "hybrid graph search should choose the short backbone interval, not the locally nearest endpoints");
  require(
    plan.metrics.selected_backbone_length_m < 10.0,
    "selected backbone leg should be the short unified-graph route");
}

void testHybridPlannerRejectsInvalidBackboneEdge()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 1.0;

  auto add_surface_cell = [&](const GridIndex & cell, int support_component_id) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::Expanded;
    surface_cell.reachability = ReachabilityLabel::InferredReachable;
    surface_cell.support_component_id = support_component_id;
    surface_cell.surface_layer_id = support_component_id;
    surface_cell.height_m = 0.0;
    surface_cell.confidence = 1.0;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  add_surface_cell({0, 0, 0}, 1);
  add_surface_cell({10, 0, 0}, 2);
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  ExperienceSurfaceGraph surface_graph;
  surface_graph.build(snapshot, SurfaceTransitionValidator(planner_options));

  const SurfaceNodeId start = surface_graph.nodeIdForCell({0, 0, 0});
  const SurfaceNodeId goal = surface_graph.nodeIdForCell({10, 0, 0});
  require(surface_graph.isValid(start), "invalid backbone test start node should exist");
  require(surface_graph.isValid(goal), "invalid backbone test goal node should exist");
  require(
    !surface_graph.sameComponent(start, goal),
    "invalid backbone test surface nodes should remain disconnected");

  N3NavResource resource;
  resource.map_frame = "map";
  resource.body_frame = "base";
  resource.has_native_dense_trajectory = true;
  resource.dense_trajectory_source = "native";
  TrajectoryProjectionResult projection;

  auto add_backbone_sample = [&](int seq, const Point3 & support) {
    N3TrajectoryPose pose;
    pose.seq = static_cast<std::uint64_t>(seq);
    pose.timestamp = static_cast<double>(seq);
    pose.pose_world_lidar.translation = {support.x, support.y, support.z + 0.50};
    resource.dense_trajectory.push_back(pose);

    ProjectedSupportSample sample;
    sample.seq = pose.seq;
    sample.timestamp = pose.timestamp;
    sample.trajectory_position = pose.pose_world_lidar.translation;
    sample.support_position = support;
    projection.accepted_projected_support_samples.push_back(sample);
  };

  add_backbone_sample(0, {0.5, 0.5, 0.0});
  add_backbone_sample(1, {10.5, 0.5, 0.0});

  ExperienceBackboneOptions backbone_options;
  backbone_options.min_node_spacing_m = 0.0;
  backbone_options.max_portal_xy_distance_m = 0.75;
  backbone_options.max_portal_height_error_m = 0.20;
  ExperienceBackboneGraph backbone;
  backbone.build(resource, projection, surface_graph, backbone_options);
  require(backbone.portals().size() == 2U, "invalid backbone test should expose two portals");

  HybridExperiencePlannerOptions hybrid_options;
  hybrid_options.max_backbone_edge_xy_gap_m = 1.0;
  const auto plan = HybridExperiencePlanner(planner_options, hybrid_options).plan(
    surface_graph, backbone, start, goal);
  require(
    !plan.success,
    "hybrid planner should not use an over-limit backbone edge as a valid connector");
}

void testBackboneCreatesComponentDiversePortals()
{
  ExperienceSnapshot snapshot;
  snapshot.map_frame = "map";
  snapshot.resolution_m = 1.0;

  auto add_surface_cell = [&](const GridIndex & cell, double height_m, int layer_id) {
    SurfaceCell surface_cell;
    surface_cell.cell = cell;
    surface_cell.label = SurfaceLabel::Expanded;
    surface_cell.reachability = ReachabilityLabel::InferredReachable;
    surface_cell.support_component_id = layer_id;
    surface_cell.surface_layer_id = layer_id;
    surface_cell.height_m = height_m;
    surface_cell.confidence = 1.0;
    snapshot.surface.surface_cells[cell] = surface_cell;
    snapshot.surface.traversable_cells.insert(cell);
    snapshot.reachability[cell] = surface_cell.reachability;
  };

  add_surface_cell({0, 0, 0}, 0.0, 1);
  add_surface_cell({0, 0, 2}, 1.0, 2);
  snapshot.clearance.compute(
    snapshot.surface.traversable_cells, snapshot.surface.boundary_cells, snapshot.resolution_m);
  snapshot.risk.compute(snapshot.surface, snapshot.clearance);

  SurfacePlannerOptions planner_options;
  planner_options.require_footprint_support = false;
  ExperienceSurfaceGraph surface_graph;
  surface_graph.build(snapshot, SurfaceTransitionValidator(planner_options), planner_options);

  N3NavResource resource;
  resource.map_frame = "map";
  resource.body_frame = "base";
  resource.has_native_dense_trajectory = true;
  resource.dense_trajectory_source = "native";
  N3TrajectoryPose pose;
  pose.seq = 1;
  pose.timestamp = 1.0;
  pose.pose_world_lidar.translation = {0.5, 0.5, 1.0};
  resource.dense_trajectory.push_back(pose);

  TrajectoryProjectionResult projection;
  ProjectedSupportSample sample;
  sample.seq = pose.seq;
  sample.timestamp = pose.timestamp;
  sample.trajectory_position = pose.pose_world_lidar.translation;
  sample.support_position = {0.5, 0.5, 0.5};
  projection.accepted_projected_support_samples.push_back(sample);

  ExperienceBackboneOptions options;
  options.min_node_spacing_m = 0.0;
  options.max_portal_xy_distance_m = 0.25;
  options.max_portal_height_error_m = 0.60;
  options.max_portals_per_node = 2;
  ExperienceBackboneGraph backbone;
  backbone.build(resource, projection, surface_graph, options);

  require(backbone.nodes().size() == 1U, "single trajectory sample should create one backbone node");
  require(
    backbone.portals().size() == 2U,
    "one backbone node should retain component-diverse portals");
  require(
    backbone.portals()[0].surface_component_id != backbone.portals()[1].surface_component_id,
    "component-diverse portals should attach to different surface components");
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
  testFootprintSupportRatio();
  testRouteProgressTrackerBuildsLocalWindowWithoutLayerJump();
  testRouteProgressTrackerRejectsFarProjection();
  testLocalPathSmootherBuildsShortSmoothPath();
  testLocalPathSmootherDoesNotCutAcrossCorner();
  testLocalPathSmootherFallsBackToRouteWhenBezierLeavesCorridor();
  testRegulatedPurePursuitCruisesOnStraightPath();
  testRegulatedPurePursuitSlowsForCurvature();
  testRegulatedPurePursuitSlowsNearGoal();
  testRegulatedPurePursuitGoalToleranceIsDistance();
  testRegulatedPurePursuitStopsForLocalObstacle();
  testRollingLocalMapExpiresOldObstacles();
  testLayeredSurfaceGraph();
  testSurfaceGraphRejectsLayerJump();
  testSurfaceGraphBridgeHeightPolicy();
  testLowConfidenceIsNotBridge();
  testSurfaceGraphRejectsMissingSupportLineage();
  testBridgeEndpointAttachesOnlyToIntendedComponent();
  testHybridPlannerUsesDenseTrajectoryBackboneAcrossIslands();
  testHybridPlannerOptimizesPortalPairCost();
  testHybridPlannerRejectsInvalidBackboneEdge();
  testBackboneCreatesComponentDiversePortals();
  testExperienceBuilderSkeleton();
  testReachableExpansionHeightGate();
  testReachableExpansionRejectsBodyObstruction();
  testReachableExpansionFillsSmallHole();
  testReachableExpansionRejectsAnchorEnvelopeEscape();
  testBridgeSeedsDoNotAnchorExpansion();
  std::cout << "PASS experience_core_smoke" << std::endl;
  return 0;
}
