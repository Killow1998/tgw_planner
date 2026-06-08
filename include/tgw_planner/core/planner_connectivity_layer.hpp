#pragma once

#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

#include "tgw_planner/core/grid_index.hpp"
#include "tgw_planner/core/surface_astar_planner.hpp"

namespace tgw_planner::core
{

struct PlannerComponentInfo
{
  int id{-1};
  std::size_t size{0};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
};

class PlannerConnectivityLayer
{
public:
  void build(
    const NavigationSnapshot & snapshot,
    const SurfaceTransitionValidator & validator);

  int componentId(const GridIndex & cell) const;
  bool sameComponent(const GridIndex & a, const GridIndex & b) const;
  bool isPlannerReachableCell(const GridIndex & cell) const;
  std::size_t componentSize(int id) const;
  double componentMinZ(int id) const;
  double componentMaxZ(int id) const;
  std::size_t componentCount() const;
  std::size_t largestComponentSize() const;
  std::size_t multifloorComponentCount(double min_z_range_m) const;

  const std::unordered_map<GridIndex, int, GridIndexHash> & componentIds() const;
  const std::vector<PlannerComponentInfo> & components() const;

private:
  std::unordered_map<GridIndex, int, GridIndexHash> component_id_;
  std::vector<PlannerComponentInfo> components_;
};

}  // namespace tgw_planner::core
