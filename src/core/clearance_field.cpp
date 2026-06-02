#include "tgw_planner/core/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace tgw_planner::core
{
namespace
{
struct QueueItem
{
  GridIndex cell;
  double distance{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueItem & lhs, const QueueItem & rhs) const
  {
    return lhs.distance > rhs.distance;
  }
};
}  // namespace

void ClearanceField::compute(
  const std::unordered_set<GridIndex, GridIndexHash> & traversable,
  const std::unordered_set<GridIndex, GridIndexHash> & boundary,
  double resolution_m)
{
  distance_m_.clear();
  if (traversable.empty()) {
    return;
  }

  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> queue;
  for (const GridIndex & cell : boundary) {
    if (traversable.find(cell) == traversable.end()) {
      continue;
    }
    distance_m_[cell] = 0.0;
    queue.push({cell, 0.0});
  }

  if (queue.empty()) {
    for (const GridIndex & cell : traversable) {
      distance_m_[cell] = std::numeric_limits<double>::infinity();
    }
    return;
  }

  while (!queue.empty()) {
    const QueueItem current = queue.top();
    queue.pop();
    const auto best = distance_m_.find(current.cell);
    if (best == distance_m_.end() || current.distance > best->second + 1.0e-9) {
      continue;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{
            current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          if (traversable.find(neighbor) == traversable.end()) {
            continue;
          }
          const double step =
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_m;
          const double candidate = current.distance + step;
          const auto old = distance_m_.find(neighbor);
          if (old != distance_m_.end() && candidate >= old->second) {
            continue;
          }
          distance_m_[neighbor] = candidate;
          queue.push({neighbor, candidate});
        }
      }
    }
  }
}

double ClearanceField::clearanceDistance(const GridIndex & cell) const
{
  const auto it = distance_m_.find(cell);
  return it == distance_m_.end() ? 0.0 : it->second;
}

double ClearanceField::clearancePenalty(const GridIndex & cell) const
{
  const double distance = clearanceDistance(cell);
  if (!std::isfinite(distance)) {
    return 0.0;
  }
  return 1.0 / (distance + 0.05);
}

std::vector<GridIndex> ClearanceField::medialAxisCells(
  double min_clearance_m, double ridge_tolerance_m) const
{
  std::vector<GridIndex> out;
  const double min_clearance = std::max(0.0, min_clearance_m);
  const double tolerance = std::max(0.0, ridge_tolerance_m);
  for (const auto & entry : distance_m_) {
    const GridIndex & cell = entry.first;
    const double distance = entry.second;
    if (!std::isfinite(distance) || distance < min_clearance) {
      continue;
    }

    bool local_maximum = true;
    for (int dx = -1; dx <= 1 && local_maximum; ++dx) {
      for (int dy = -1; dy <= 1 && local_maximum; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          const auto neighbor_it = distance_m_.find(neighbor);
          if (neighbor_it == distance_m_.end() || !std::isfinite(neighbor_it->second)) {
            continue;
          }
          if (neighbor_it->second > distance + tolerance) {
            local_maximum = false;
            break;
          }
        }
      }
    }
    if (local_maximum) {
      out.push_back(cell);
    }
  }
  return out;
}

const std::unordered_map<GridIndex, double, GridIndexHash> & ClearanceField::distances() const
{
  return distance_m_;
}

}  // namespace tgw_planner::core
