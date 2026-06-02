#pragma once

#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/planning_types.hpp"
#include "tgw_planner/core/surface_extractor.hpp"

namespace tgw_planner::core
{

struct RobotFootprintOptions
{
  double length_m{0.70};
  double width_m{0.43};
  double height_m{0.50};
  double base_to_front_m{0.20};
};

class RobotFootprint
{
public:
  explicit RobotFootprint(RobotFootprintOptions options = {});

  std::vector<Point3> sampleFootprint(
    const Point3 & center, double yaw_rad, double resolution_m) const;

  bool containsBodyPoint(const Point3 & point_base) const;

  bool isSupported(
    const SurfaceMap & surface, const Point3 & center, double yaw_rad,
    double resolution_m) const;

  const RobotFootprintOptions & options() const;

private:
  GridIndex worldToGrid(const Point3 & point, double resolution_m) const;

  RobotFootprintOptions options_;
};

}  // namespace tgw_planner::core
