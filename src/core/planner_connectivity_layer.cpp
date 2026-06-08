#include "tgw_planner/core/planner_connectivity_layer.hpp"

#include <algorithm>
#include <queue>
#include <unordered_set>

namespace tgw_planner::core
{

void PlannerConnectivityLayer::build(
  const NavigationSnapshot & snapshot,
  const SurfaceTransitionValidator & validator)
{
  component_id_.clear();
  components_.clear();

  std::unordered_set<GridIndex, GridIndexHash> endpoint_cells;
  endpoint_cells.reserve(snapshot.surface.traversable_cells.size());
  for (const GridIndex & cell : snapshot.surface.traversable_cells) {
    if (validator.isEndpointCell(snapshot, cell)) {
      endpoint_cells.insert(cell);
    }
  }

  for (const GridIndex & seed : endpoint_cells) {
    if (component_id_.find(seed) != component_id_.end()) {
      continue;
    }

    const int component_id = static_cast<int>(components_.size());
    PlannerComponentInfo info;
    info.id = component_id;
    std::queue<GridIndex> queue;
    queue.push(seed);
    component_id_[seed] = component_id;

    while (!queue.empty()) {
      const GridIndex current = queue.front();
      queue.pop();
      ++info.size;
      const double z = (static_cast<double>(current.z) + 0.5) * snapshot.resolution_m;
      info.min_z = std::min(info.min_z, z);
      info.max_z = std::max(info.max_z, z);

      for (const GridIndex & neighbor : validator.validNeighbors(snapshot, current)) {
        if (component_id_.find(neighbor) != component_id_.end() ||
          endpoint_cells.find(neighbor) == endpoint_cells.end())
        {
          continue;
        }
        component_id_[neighbor] = component_id;
        queue.push(neighbor);
      }
    }

    components_.push_back(info);
  }
}

int PlannerConnectivityLayer::componentId(const GridIndex & cell) const
{
  const auto it = component_id_.find(cell);
  return it == component_id_.end() ? -1 : it->second;
}

bool PlannerConnectivityLayer::sameComponent(const GridIndex & a, const GridIndex & b) const
{
  const int a_id = componentId(a);
  return a_id >= 0 && a_id == componentId(b);
}

bool PlannerConnectivityLayer::isPlannerReachableCell(const GridIndex & cell) const
{
  return component_id_.find(cell) != component_id_.end();
}

std::size_t PlannerConnectivityLayer::componentSize(int id) const
{
  if (id < 0 || static_cast<std::size_t>(id) >= components_.size()) {
    return 0U;
  }
  return components_[static_cast<std::size_t>(id)].size;
}

double PlannerConnectivityLayer::componentMinZ(int id) const
{
  if (id < 0 || static_cast<std::size_t>(id) >= components_.size()) {
    return 0.0;
  }
  return components_[static_cast<std::size_t>(id)].min_z;
}

double PlannerConnectivityLayer::componentMaxZ(int id) const
{
  if (id < 0 || static_cast<std::size_t>(id) >= components_.size()) {
    return 0.0;
  }
  return components_[static_cast<std::size_t>(id)].max_z;
}

std::size_t PlannerConnectivityLayer::componentCount() const
{
  return components_.size();
}

std::size_t PlannerConnectivityLayer::largestComponentSize() const
{
  std::size_t out = 0U;
  for (const PlannerComponentInfo & component : components_) {
    out = std::max(out, component.size);
  }
  return out;
}

std::size_t PlannerConnectivityLayer::multifloorComponentCount(double min_z_range_m) const
{
  std::size_t out = 0U;
  for (const PlannerComponentInfo & component : components_) {
    if (component.max_z - component.min_z >= min_z_range_m) {
      ++out;
    }
  }
  return out;
}

const std::unordered_map<GridIndex, int, GridIndexHash> &
PlannerConnectivityLayer::componentIds() const
{
  return component_id_;
}

const std::vector<PlannerComponentInfo> & PlannerConnectivityLayer::components() const
{
  return components_;
}

}  // namespace tgw_planner::core
