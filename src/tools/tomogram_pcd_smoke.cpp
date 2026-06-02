#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace
{
constexpr double kUnsetGround = -1.0e9;
constexpr double kUnsetCeiling = 1.0e9;
constexpr double kBarrierCost = 50.0;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Node
{
  int layer{0};
  int x{0};
  int y{0};

  bool operator==(const Node & other) const
  {
    return layer == other.layer && x == other.x && y == other.y;
  }
};

struct NodeHash
{
  std::size_t operator()(const Node & node) const
  {
    std::size_t seed = std::hash<int>{}(node.layer);
    seed ^= std::hash<int>{}(node.x) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(node.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct QueueNode
{
  Node node;
  double f{0.0};
  double g{0.0};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

struct ComponentDiagnostics
{
  std::size_t component_count{0U};
  std::size_t largest_component_size{0U};
  int start_component{-1};
  int goal_component{-1};
  std::size_t start_component_size{0U};
  std::size_t goal_component_size{0U};
  double start_goal_component_gap_m{0.0};
  Node gap_start_node;
  Node gap_goal_node;
  double gap_start_ground_m{0.0};
  double gap_goal_ground_m{0.0};
  double gap_start_cost{0.0};
  double gap_goal_cost{0.0};
  int gap_start_gateway{0};
  int gap_goal_gateway{0};
};

double argDouble(char ** argv, int index)
{
  return std::strtod(argv[index], nullptr);
}

class Tomogram
{
public:
  Tomogram(double resolution_m, double slice_dh, double slice_h0)
  : resolution_m_(resolution_m), slice_dh_(slice_dh), slice_h0_(slice_h0)
  {
  }

  bool build(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
    if (cloud.empty()) {
      return false;
    }
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      min_x = std::min<double>(min_x, point.x);
      min_y = std::min<double>(min_y, point.y);
      min_z = std::min<double>(min_z, point.z);
      max_x = std::max<double>(max_x, point.x);
      max_y = std::max<double>(max_y, point.y);
      max_z = std::max<double>(max_z, point.z);
    }
    if (!std::isfinite(min_x)) {
      return false;
    }

    constexpr double margin_m = 1.0;
    origin_x_ = std::floor((min_x - margin_m) / resolution_m_) * resolution_m_;
    origin_y_ = std::floor((min_y - margin_m) / resolution_m_) * resolution_m_;
    nx_ = std::max(1, static_cast<int>(std::ceil((max_x + margin_m - origin_x_) / resolution_m_)));
    ny_ = std::max(1, static_cast<int>(std::ceil((max_y + margin_m - origin_y_) / resolution_m_)));
    if (!std::isfinite(slice_h0_)) {
      slice_h0_ = std::floor(min_z / slice_dh_) * slice_dh_;
    }
    nlayers_ = std::max(1, static_cast<int>(std::ceil((max_z - slice_h0_) / slice_dh_)) + 2);
    ground_.assign(size(), kUnsetGround);
    ceiling_.assign(size(), kUnsetCeiling);
    cost_.assign(size(), kBarrierCost);
    gateway_.assign(size(), 0);

    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const int x = worldToX(point.x);
      const int y = worldToY(point.y);
      if (!inside(x, y)) {
        continue;
      }
      for (int layer = 0; layer < nlayers_; ++layer) {
        const double slice = sliceHeight(layer);
        const std::size_t idx = index(layer, x, y);
        if (point.z <= slice) {
          ground_[idx] = std::max<double>(ground_[idx], point.z);
        } else {
          ceiling_[idx] = std::min<double>(ceiling_[idx], point.z);
        }
      }
    }

    computeTraversability();
    simplifyLayers();
    computeGateways();
    return true;
  }

  bool snap(const Point3 & point, int forced_layer, Node & node, double & snap_distance_m) const
  {
    const int sx = worldToX(point.x);
    const int sy = worldToY(point.y);
    const int max_radius = std::max(3, static_cast<int>(std::ceil(2.0 / resolution_m_)));
    bool found = false;
    double best = std::numeric_limits<double>::infinity();
    Node best_node;
    for (int radius = 0; radius <= max_radius; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          const int x = sx + dx;
          const int y = sy + dy;
          if (!inside(x, y)) {
            continue;
          }
          const int layer_begin = forced_layer >= 0 ? forced_layer : 0;
          const int layer_end = forced_layer >= 0 ? forced_layer + 1 : nlayers_;
          for (int layer = layer_begin; layer < layer_end; ++layer) {
            const Node candidate{layer, x, y};
            if (!traversable(candidate)) {
              continue;
            }
            const Point3 world = nodeWorld(candidate);
            const double distance = std::sqrt(
              (world.x - point.x) * (world.x - point.x) +
              (world.y - point.y) * (world.y - point.y) +
              (world.z - point.z) * (world.z - point.z));
            if (distance < best) {
              best = distance;
              best_node = candidate;
              found = true;
            }
          }
        }
      }
      if (found) {
        node = best_node;
        snap_distance_m = best;
        return true;
      }
    }
    return false;
  }

  bool plan(const Node & start, const Node & goal, std::vector<Node> & path, std::uint32_t & expanded) const
  {
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;
    std::unordered_map<Node, double, NodeHash> g_score;
    std::unordered_map<Node, Node, NodeHash> parent;
    std::unordered_set<Node, NodeHash> closed;
    g_score[start] = 0.0;
    open.push({start, heuristic(start, goal), 0.0});

    while (!open.empty() && expanded < 2000000U) {
      const QueueNode current = open.top();
      open.pop();
      const auto best_it = g_score.find(current.node);
      if (best_it == g_score.end() || current.g > best_it->second + 1.0e-9) {
        continue;
      }
      if (!closed.insert(current.node).second) {
        continue;
      }
      if (current.node == goal) {
        Node trace = goal;
        path.push_back(trace);
        while (!(trace == start)) {
          trace = parent.at(trace);
          path.push_back(trace);
        }
        std::reverse(path.begin(), path.end());
        return true;
      }
      ++expanded;

      for (const Node & neighbor : neighbors(current.node)) {
        const double tentative = current.g + transitionCost(current.node, neighbor);
        const auto old_it = g_score.find(neighbor);
        if (old_it != g_score.end() && tentative >= old_it->second) {
          continue;
        }
        parent[neighbor] = current.node;
        g_score[neighbor] = tentative;
        open.push({neighbor, tentative + heuristic(neighbor, goal), tentative});
      }
    }
    return false;
  }

  double pathLength(const std::vector<Node> & path) const
  {
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Point3 a = nodeWorld(path[i - 1U]);
      const Point3 b = nodeWorld(path[i]);
      length += std::sqrt(
        (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
        (a.z - b.z) * (a.z - b.z));
    }
    return length;
  }

  std::size_t traversableCount() const
  {
    return static_cast<std::size_t>(
      std::count_if(cost_.begin(), cost_.end(), [](double cost) {return cost < kBarrierCost;}));
  }

  int nx() const {return nx_;}
  int ny() const {return ny_;}
  int nlayers() const {return nlayers_;}
  std::size_t gatewayCount() const
  {
    return static_cast<std::size_t>(
      std::count_if(gateway_.begin(), gateway_.end(), [](int gateway) {return gateway != 0;}));
  }

  ComponentDiagnostics componentDiagnostics(const Node & start, const Node & goal) const
  {
    ComponentDiagnostics diagnostics;
    std::unordered_set<Node, NodeHash> unvisited;
    unvisited.reserve(traversableCount());
    for (int layer = 0; layer < nlayers_; ++layer) {
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          const Node node{layer, x, y};
          if (traversable(node)) {
            unvisited.insert(node);
          }
        }
      }
    }

    std::vector<std::vector<Node>> components;
    while (!unvisited.empty()) {
      const Node seed = *unvisited.begin();
      unvisited.erase(seed);
      std::vector<Node> component;
      std::queue<Node> queue;
      queue.push(seed);
      component.push_back(seed);
      while (!queue.empty()) {
        const Node current = queue.front();
        queue.pop();
        for (const Node & neighbor : neighbors(current)) {
          const auto it = unvisited.find(neighbor);
          if (it == unvisited.end()) {
            continue;
          }
          queue.push(neighbor);
          component.push_back(neighbor);
          unvisited.erase(it);
        }
      }
      diagnostics.largest_component_size =
        std::max(diagnostics.largest_component_size, component.size());
      components.push_back(std::move(component));
    }
    diagnostics.component_count = components.size();

    auto component_index = [&](const Node & node) {
      for (std::size_t i = 0; i < components.size(); ++i) {
        if (std::find(components[i].begin(), components[i].end(), node) != components[i].end()) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };
    diagnostics.start_component = component_index(start);
    diagnostics.goal_component = component_index(goal);
    if (diagnostics.start_component >= 0) {
      diagnostics.start_component_size =
        components[static_cast<std::size_t>(diagnostics.start_component)].size();
    }
    if (diagnostics.goal_component >= 0) {
      diagnostics.goal_component_size =
        components[static_cast<std::size_t>(diagnostics.goal_component)].size();
    }
    if (diagnostics.start_component < 0 || diagnostics.goal_component < 0) {
      return diagnostics;
    }
    if (diagnostics.start_component == diagnostics.goal_component) {
      diagnostics.start_goal_component_gap_m = 0.0;
      diagnostics.gap_start_node = start;
      diagnostics.gap_goal_node = goal;
    } else {
      const auto & start_component =
        components[static_cast<std::size_t>(diagnostics.start_component)];
      const auto & goal_component =
        components[static_cast<std::size_t>(diagnostics.goal_component)];
      double best_distance = std::numeric_limits<double>::infinity();
      for (const Node & a : start_component) {
        const Point3 aw = nodeWorld(a);
        for (const Node & b : goal_component) {
          const Point3 bw = nodeWorld(b);
          const double distance = std::sqrt(
            (aw.x - bw.x) * (aw.x - bw.x) + (aw.y - bw.y) * (aw.y - bw.y) +
            (aw.z - bw.z) * (aw.z - bw.z));
          if (distance < best_distance) {
            best_distance = distance;
            diagnostics.gap_start_node = a;
            diagnostics.gap_goal_node = b;
          }
        }
      }
      diagnostics.start_goal_component_gap_m =
        std::isfinite(best_distance) ? best_distance : 0.0;
    }

    diagnostics.gap_start_ground_m = groundAt(diagnostics.gap_start_node);
    diagnostics.gap_goal_ground_m = groundAt(diagnostics.gap_goal_node);
    diagnostics.gap_start_cost = costAt(diagnostics.gap_start_node);
    diagnostics.gap_goal_cost = costAt(diagnostics.gap_goal_node);
    diagnostics.gap_start_gateway = gatewayAt(diagnostics.gap_start_node);
    diagnostics.gap_goal_gateway = gatewayAt(diagnostics.gap_goal_node);
    return diagnostics;
  }

private:
  std::size_t size() const
  {
    return static_cast<std::size_t>(std::max(0, nlayers_) * std::max(0, nx_) * std::max(0, ny_));
  }

  std::size_t index(int layer, int x, int y) const
  {
    return static_cast<std::size_t>((layer * ny_ + y) * nx_ + x);
  }

  bool inside(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < nx_ && y < ny_;
  }

  int worldToX(double x) const
  {
    return static_cast<int>(std::floor((x - origin_x_) / resolution_m_));
  }

  int worldToY(double y) const
  {
    return static_cast<int>(std::floor((y - origin_y_) / resolution_m_));
  }

  double sliceHeight(int layer) const
  {
    return slice_h0_ + static_cast<double>(layer) * slice_dh_;
  }

  bool hasGround(int layer, int x, int y) const
  {
    return ground_[index(layer, x, y)] > kUnsetGround * 0.5;
  }

  bool traversable(const Node & node) const
  {
    return inside(node.x, node.y) && node.layer >= 0 && node.layer < nlayers_ &&
           cost_[index(node.layer, node.x, node.y)] < kBarrierCost;
  }

  double groundAt(const Node & node) const
  {
    return ground_[index(node.layer, node.x, node.y)];
  }

  double costAt(const Node & node) const
  {
    return cost_[index(node.layer, node.x, node.y)];
  }

  int gatewayAt(const Node & node) const
  {
    return gateway_[index(node.layer, node.x, node.y)];
  }

  Point3 nodeWorld(const Node & node) const
  {
    return {
      origin_x_ + (static_cast<double>(node.x) + 0.5) * resolution_m_,
      origin_y_ + (static_cast<double>(node.y) + 0.5) * resolution_m_,
      ground_[index(node.layer, node.x, node.y)] + 0.15};
  }

  void computeTraversability()
  {
    const double interval_min = 0.50;
    const double interval_free = 0.65;
    const double slope_max = 0.40;
    const double step_stand = 1.2 * resolution_m_ * std::tan(slope_max);
    const double step_cross = 0.30;
    constexpr int kernel_radius = 3;
    const int standable_threshold =
      static_cast<int>(0.40 * std::pow(2 * kernel_radius + 1, 2)) - 1;

    std::vector<double> grad_sq(size(), 0.0);
    std::vector<double> grad_max(size(), 0.0);
    for (int layer = 0; layer < nlayers_; ++layer) {
      for (int y = 1; y + 1 < ny_; ++y) {
        for (int x = 1; x + 1 < nx_; ++x) {
          if (!hasGround(layer, x, y)) {
            continue;
          }
          const std::size_t idx = index(layer, x, y);
          const double gx = std::max(
            hasGround(layer, x - 1, y) ? std::pow(ground_[idx] - ground_[index(layer, x - 1, y)], 2.0) : 0.0,
            hasGround(layer, x + 1, y) ? std::pow(ground_[idx] - ground_[index(layer, x + 1, y)], 2.0) : 0.0);
          const double gy = std::max(
            hasGround(layer, x, y - 1) ? std::pow(ground_[idx] - ground_[index(layer, x, y - 1)], 2.0) : 0.0,
            hasGround(layer, x, y + 1) ? std::pow(ground_[idx] - ground_[index(layer, x, y + 1)], 2.0) : 0.0);
          grad_sq[idx] = gx + gy;
          grad_max[idx] = std::max(gx, gy);
        }
      }
    }

    for (int layer = 0; layer < nlayers_; ++layer) {
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          const std::size_t idx = index(layer, x, y);
          if (!hasGround(layer, x, y)) {
            continue;
          }
          const double ceiling = ceiling_[idx] >= kUnsetCeiling * 0.5 ? 10.0 : ceiling_[idx];
          const double interval = ceiling - ground_[idx];
          if (interval < interval_min) {
            cost_[idx] = kBarrierCost;
            continue;
          }
          double cost = std::max(0.0, 20.0 * (interval_free - interval));
          if (grad_sq[idx] <= step_stand * step_stand) {
            cost += 15.0 * grad_sq[idx] / std::max(1.0e-9, step_stand * step_stand);
            cost_[idx] = cost;
            continue;
          }
          if (grad_max[idx] > step_cross * step_cross) {
            cost_[idx] = kBarrierCost;
            continue;
          }
          int standable = 0;
          for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
            for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
              const int nx = x + dx;
              const int ny = y + dy;
              if (!inside(nx, ny) || !hasGround(layer, nx, ny)) {
                continue;
              }
              if (grad_sq[index(layer, nx, ny)] < step_stand * step_stand) {
                ++standable;
              }
            }
          }
          if (standable < standable_threshold) {
            cost_[idx] = kBarrierCost;
          } else {
            cost_[idx] = cost + 20.0 * grad_max[idx] / std::max(1.0e-9, step_cross * step_cross);
          }
        }
      }
    }
  }

  void computeGateways()
  {
    for (int layer = 0; layer + 1 < nlayers_; ++layer) {
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          if (!hasGround(layer, x, y) || !hasGround(layer + 1, x, y)) {
            continue;
          }
          const std::size_t lower = index(layer, x, y);
          const std::size_t upper = index(layer + 1, x, y);
          if (std::abs(ground_[upper] - ground_[lower]) >= 0.10) {
            continue;
          }
          const double diff_cost = cost_[upper] - cost_[lower];
          if (diff_cost < -8.0) {
            gateway_[lower] = 2;
          } else if (diff_cost > 8.0) {
            gateway_[upper] = -2;
          }
        }
      }
    }
  }

  void simplifyLayers()
  {
    if (nlayers_ <= 2) {
      return;
    }
    std::vector<int> keep_layers;
    keep_layers.push_back(0);
    int lower_layer = 0;
    int middle_layer = 1;
    while (middle_layer < nlayers_ - 2) {
      bool unique = false;
      for (int y = 0; y < ny_ && !unique; ++y) {
        for (int x = 0; x < nx_; ++x) {
          if (!hasGround(middle_layer, x, y) || !hasGround(lower_layer, x, y) ||
            !hasGround(middle_layer + 1, x, y))
          {
            continue;
          }
          const std::size_t middle = index(middle_layer, x, y);
          const std::size_t lower = index(lower_layer, x, y);
          const std::size_t upper = index(middle_layer + 1, x, y);
          const bool lower_ground_or_better_cost =
            ground_[middle] - ground_[lower] > 0.0 || cost_[lower] > cost_[middle];
          const bool upper_ground_increases = ground_[upper] - ground_[middle] > 0.0;
          const bool middle_traversable = cost_[middle] < kBarrierCost;
          if (lower_ground_or_better_cost && upper_ground_increases && middle_traversable) {
            unique = true;
            break;
          }
        }
      }
      if (unique) {
        keep_layers.push_back(middle_layer);
        lower_layer = middle_layer;
      }
      ++middle_layer;
    }
    keep_layers.push_back(middle_layer);

    const int old_nlayers = nlayers_;
    const std::vector<double> old_ground = ground_;
    const std::vector<double> old_ceiling = ceiling_;
    const std::vector<double> old_cost = cost_;
    nlayers_ = static_cast<int>(keep_layers.size());
    ground_.assign(size(), kUnsetGround);
    ceiling_.assign(size(), kUnsetCeiling);
    cost_.assign(size(), kBarrierCost);
    gateway_.assign(size(), 0);
    for (int new_layer = 0; new_layer < nlayers_; ++new_layer) {
      const int old_layer = std::clamp(keep_layers[static_cast<std::size_t>(new_layer)], 0, old_nlayers - 1);
      for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
          const std::size_t old_idx =
            static_cast<std::size_t>((old_layer * ny_ + y) * nx_ + x);
          const std::size_t new_idx = index(new_layer, x, y);
          ground_[new_idx] = old_ground[old_idx];
          ceiling_[new_idx] = old_ceiling[old_idx];
          cost_[new_idx] = old_cost[old_idx];
        }
      }
    }
  }

  std::vector<Node> neighbors(const Node & node) const
  {
    std::vector<Node> out;
    out.reserve(18U);
    const int search_layer = decideSearchLayer(node);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Node neighbor{search_layer, node.x + dx, node.y + dy};
        if (!inside(neighbor.x, neighbor.y) || neighbor.layer < 0 || neighbor.layer >= nlayers_) {
          continue;
        }
        const double dz = std::abs(
          ground_[index(node.layer, node.x, node.y)] -
          ground_[index(neighbor.layer, neighbor.x, neighbor.y)]);
        const bool gateway_neighbor =
          std::abs(gateway_[index(neighbor.layer, neighbor.x, neighbor.y)]) > 0;
        if (traversable(neighbor) || (gateway_neighbor && dz <= 0.30)) {
          out.push_back(neighbor);
        }
      }
    }
    return out;
  }

  int decideSearchLayer(const Node & node) const
  {
    const double current_height = ground_[index(node.layer, node.x, node.y)];
    for (const int offset : {0, -1, 1}) {
      const int layer = node.layer + offset;
      if (layer < 0 || layer >= nlayers_) {
        continue;
      }
      if (!hasGround(layer, node.x, node.y)) {
        continue;
      }
      if (std::abs(ground_[index(layer, node.x, node.y)] - current_height) > 0.20) {
        continue;
      }
      const int gateway = gateway_[index(layer, node.x, node.y)];
      if (gateway > 0) {
        return std::min(layer + 1, nlayers_ - 1);
      }
      if (gateway < 0) {
        return std::max(layer - 1, 0);
      }
    }
    return node.layer;
  }

  double transitionCost(const Node & from, const Node & to) const
  {
    const Point3 a = nodeWorld(from);
    const Point3 b = nodeWorld(to);
    const double distance = std::sqrt(
      (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
      (a.z - b.z) * (a.z - b.z));
    const double to_cost = cost_[index(to.layer, to.x, to.y)];
    const double step_cost = to_cost < 5.0 ? 0.0 : 0.02 * to_cost;
    return distance + step_cost;
  }

  double heuristic(const Node & from, const Node & to) const
  {
    const Point3 a = nodeWorld(from);
    const Point3 b = nodeWorld(to);
    return std::sqrt(
      (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
      (a.z - b.z) * (a.z - b.z));
  }

  double resolution_m_{0.20};
  double slice_dh_{0.50};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double slice_h0_{0.0};
  int nx_{0};
  int ny_{0};
  int nlayers_{0};
  std::vector<double> ground_;
  std::vector<double> ceiling_;
  std::vector<double> cost_;
  std::vector<int> gateway_;
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 9) {
    std::cerr << "usage: tgw_tomogram_pcd_smoke <pcd> <resolution_m> "
      "<start_x> <start_y> <start_z> <goal_x> <goal_y> <goal_z> "
      "[slice_dh=0.5] [slice_h0=nan] [forced_layer=-1]\n";
    return 2;
  }

  const std::string pcd_path = argv[1];
  const double resolution_m = argDouble(argv, 2);
  const Point3 start{argDouble(argv, 3), argDouble(argv, 4), argDouble(argv, 5)};
  const Point3 goal{argDouble(argv, 6), argDouble(argv, 7), argDouble(argv, 8)};
  const double slice_dh = argc >= 10 ? argDouble(argv, 9) : 0.50;
  const double slice_h0 = argc >= 11 ? argDouble(argv, 10) :
    std::numeric_limits<double>::quiet_NaN();
  const int forced_layer = argc >= 12 ? std::atoi(argv[11]) : -1;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(pcd_path, cloud) != 0) {
    std::cerr << "failed to load PCD: " << pcd_path << "\n";
    return 1;
  }

  Tomogram tomogram(resolution_m, slice_dh, slice_h0);
  if (!tomogram.build(cloud)) {
    std::cerr << "failed to build tomogram\n";
    return 1;
  }

  Node snapped_start;
  Node snapped_goal;
  double start_snap = 0.0;
  double goal_snap = 0.0;
  const bool start_ok = tomogram.snap(start, forced_layer, snapped_start, start_snap);
  const bool goal_ok = tomogram.snap(goal, forced_layer, snapped_goal, goal_snap);
  if (!start_ok || !goal_ok) {
    std::cout << "success=false reason=snap_failed"
      << " start_ok=" << (start_ok ? "true" : "false")
      << " goal_ok=" << (goal_ok ? "true" : "false")
      << " source_points=" << cloud.size()
      << " tomogram_layers=" << tomogram.nlayers()
      << " traversable_cells=" << tomogram.traversableCount()
      << " gateway_cells=" << tomogram.gatewayCount()
      << "\n";
    return 1;
  }

  std::vector<Node> path;
  std::uint32_t expanded = 0U;
  const bool success = tomogram.plan(snapped_start, snapped_goal, path, expanded);
  const ComponentDiagnostics components =
    tomogram.componentDiagnostics(snapped_start, snapped_goal);
  std::cout << "success=" << (success ? "true" : "false")
    << " source_points=" << cloud.size()
    << " tomogram_grid=[" << tomogram.nx() << "," << tomogram.ny() << "," <<
      tomogram.nlayers() << "]"
    << " traversable_cells=" << tomogram.traversableCount()
    << " gateway_cells=" << tomogram.gatewayCount()
    << " component_count=" << components.component_count
    << " largest_component_size=" << components.largest_component_size
    << " start_component=" << components.start_component
    << " goal_component=" << components.goal_component
    << " start_component_size=" << components.start_component_size
    << " goal_component_size=" << components.goal_component_size
    << " start_goal_component_gap_m=" << components.start_goal_component_gap_m
    << " gap_start_node=[" << components.gap_start_node.layer << "," <<
      components.gap_start_node.x << "," << components.gap_start_node.y << "]"
    << " gap_goal_node=[" << components.gap_goal_node.layer << "," <<
      components.gap_goal_node.x << "," << components.gap_goal_node.y << "]"
    << " gap_start_ground_m=" << components.gap_start_ground_m
    << " gap_goal_ground_m=" << components.gap_goal_ground_m
    << " gap_start_cost=" << components.gap_start_cost
    << " gap_goal_cost=" << components.gap_goal_cost
    << " gap_start_gateway=" << components.gap_start_gateway
    << " gap_goal_gateway=" << components.gap_goal_gateway
    << " start_snap_distance_m=" << start_snap
    << " goal_snap_distance_m=" << goal_snap
    << " start_node=[" << snapped_start.layer << "," << snapped_start.x << "," <<
      snapped_start.y << "]"
    << " goal_node=[" << snapped_goal.layer << "," << snapped_goal.x << "," <<
      snapped_goal.y << "]"
    << " expanded_nodes=" << expanded
    << " path_waypoints=" << path.size()
    << " path_length_m=" << tomogram.pathLength(path)
    << "\n";
  return success ? 0 : 1;
}
