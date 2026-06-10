#pragma once

#include "tgw_planner/core/experience_backbone_graph.hpp"
#include "tgw_planner/core/experience_surface_builder.hpp"
#include "tgw_planner/core/experience_surface_graph.hpp"
#include "tgw_planner/core/hybrid_experience_planner.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"
#include "tgw_planner/core/trajectory_projector.hpp"

namespace tgw_planner::core
{

TrajectoryProjectorOptions defaultTrajectoryProjectorOptions();
ExperienceSurfaceBuilderOptions defaultExperienceSurfaceBuilderOptions();
SurfacePlannerOptions defaultSurfacePlannerOptions();
SurfaceGraphBuildOptions defaultSurfaceGraphBuildOptions();
ExperienceBackboneOptions defaultExperienceBackboneOptions();
HybridExperiencePlannerOptions defaultHybridExperiencePlannerOptions();

}  // namespace tgw_planner::core
