#include "tgw_planner/core/experience_planner_defaults.hpp"

namespace tgw_planner::core
{

TrajectoryProjectorOptions defaultTrajectoryProjectorOptions()
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

ExperienceSurfaceBuilderOptions defaultExperienceSurfaceBuilderOptions()
{
  ExperienceSurfaceBuilderOptions options;
  options.resolution_m = 0.10;
  options.body_clearance_height_m = 0.65;
  options.geometry_roi_distance_to_trajectory_m = 1.2;
  options.projector = defaultTrajectoryProjectorOptions();
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

SurfacePlannerOptions defaultSurfacePlannerOptions()
{
  SurfacePlannerOptions options;
  options.max_step_height_m = 0.35;
  options.max_iterations = 500000;
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

SurfaceGraphBuildOptions defaultSurfaceGraphBuildOptions()
{
  SurfaceGraphBuildOptions options;
  options.max_edge_height_delta_m = 0.35;
  options.max_bridge_edge_height_delta_m = 0.80;
  options.max_bridge_attach_height_delta_m = 0.35;
  options.max_edge_slope = 3.0;
  options.max_bridge_edge_slope = 8.0;
  options.max_surface_safety_multiplier = 5.0;
  return options;
}

ExperienceBackboneOptions defaultExperienceBackboneOptions()
{
  ExperienceBackboneOptions options;
  options.min_node_spacing_m = 0.20;
  options.max_portal_xy_distance_m = 1.20;
  options.max_portal_height_error_m = 0.45;
  options.min_portal_clearance_m = 0.0;
  options.max_portals_per_node = 3;
  return options;
}

HybridExperiencePlannerOptions defaultHybridExperiencePlannerOptions()
{
  HybridExperiencePlannerOptions options;
  options.backbone_cost_scale = 5.0;
  options.portal_switch_cost = 10.0;
  options.portal_height_error_weight = 0.25;
  options.backbone_low_confidence_penalty = 0.5;
  options.max_backbone_edge_xy_gap_m = 1.00;
  options.max_backbone_edge_dz_m = 0.85;
  options.max_backbone_edge_slope = 4.0;
  options.max_portal_xy_distance_m = 1.20;
  options.max_portal_height_error_m = 0.45;
  options.surface_target_speed_mps = 0.6;
  options.backbone_target_speed_mps = 0.3;
  options.portal_target_speed_mps = 0.15;
  return options;
}

}  // namespace tgw_planner::core
