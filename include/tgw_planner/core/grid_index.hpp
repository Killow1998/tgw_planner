#pragma once

#include <cstddef>
#include <functional>

namespace tgw_planner::core
{

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const GridIndex & other) const
  {
    return !(*this == other);
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & idx) const
  {
    std::size_t seed = std::hash<int>{}(idx.x);
    seed ^= std::hash<int>{}(idx.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(idx.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

}  // namespace tgw_planner::core
