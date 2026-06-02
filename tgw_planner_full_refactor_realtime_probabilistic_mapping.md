# tgw_planner 全量重构指令：实时概率地图 + Raycasting + Clearance-Aware Surface Planning

## 0. 任务定位

这是一次**架构级重构**，不是在现有 `StairFlight` / 直线楼梯 / 后处理路径平滑逻辑上继续打补丁。

当前方向已经暴露出问题：

- 为了“楼梯走中线”，系统不断引入 `StairFlight`、fragment rescue、直楼梯拟合、旋转楼梯拟合、中心线替换等特殊逻辑。
- 旋转楼梯、脏点云、人类操作员伪影、稀疏点云都会让“先识别完整楼梯，再沿楼梯中心线走”的方案越来越复杂。
- 真正的目标不是完美建模每一段楼梯，而是：  
  **在可通行 surface 上规划一条远离墙、远离边缘、远离 drop-off、尽量位于狭窄通道中线的安全路径。**

因此，本次重构目标是：

```text
实时点云 + 位姿
    ↓
raycast 概率占据地图
    ↓
稳定静态结构估计
    ↓
可通行 surface 提取
    ↓
boundary / clearance field / risk field
    ↓
clearance-aware A*
    ↓
最终路径验证 + 可执行 path
```

楼梯语义只作为**薄语义层**，用于速度、运动模式和额外安全策略，不再作为路径居中的唯一来源。

---

## 1. 明确要求

### 1.1 不要继续做兼容性补丁

不要再做以下方向：

```text
继续调直楼梯阈值
继续为每种楼梯写特殊 fitter
继续让 StairFlight centerline 成为唯一中线来源
继续为了 PCT 单地图的 24 条 centerline 过拟合
继续只在静态 PCD 上做 occupied voxel heuristic
```

### 1.2 允许内部完全重写

可以重写以下内部模块：

```text
NavigationMap
VoxelAstarPlanner
traversability extraction
stair segmentation
path postprocessing
map building
```

ROS 话题和服务名称可以尽量保留，方便 RViz 调试和现有脚本继续使用；但如果旧接口阻碍新设计，可以新增新接口并逐步弃用旧接口。

### 1.3 PCD 模式降级为离线/调试模式

PCD 模式必须保留，但要明确声明：

> PCD 是没有时间、没有 scan origin、没有 ray clearing 信息的静态结果。如果 PCD 中存在人类操作员、移动物体、SLAM ghost、悬浮点、动态物体残影，这些伪影会被当成真实静态结构，严重影响地面、楼梯、墙体、边缘和可通行区域判断。

因此：

```text
PCD mode = offline debug / clean static map import
Realtime raycast mode = 实机推荐建图模式
```

---

## 2. 新系统架构

重构后建议包内核心结构如下：

```text
tgw_planner/
├── include/tgw_planner/core/
│   ├── grid_index.hpp
│   ├── probabilistic_voxel_map.hpp
│   ├── raycast_integrator.hpp
│   ├── surface_extractor.hpp
│   ├── clearance_field.hpp
│   ├── surface_astar_planner.hpp
│   ├── robot_footprint.hpp
│   ├── map_snapshot.hpp
│   └── planning_types.hpp
│
├── src/core/
│   ├── probabilistic_voxel_map.cpp
│   ├── raycast_integrator.cpp
│   ├── surface_extractor.cpp
│   ├── clearance_field.cpp
│   ├── surface_astar_planner.cpp
│   └── robot_footprint.cpp
│
├── src/humble/
│   ├── realtime_mapping_node.cpp
│   ├── pcd_import_node.cpp
│   ├── planner_node.cpp
│   └── rviz_debug_node.cpp  # optional; can be folded into planner_node initially
│
├── srv/
│   ├── StartMapping.srv
│   ├── StopMapping.srv
│   ├── SaveMap.srv
│   ├── LoadMap.srv
│   ├── ExportStaticCloud.srv
│   ├── PlanPath.srv
│   └── SetBlockedRegion.srv
│
└── msg/
    ├── PlannerStats.msg
    ├── MappingStats.msg
    └── MapBuildStats.msg
```

如果为了 MVP 简化，`realtime_mapping_node` 和 `planner_node` 可以先合并成一个 `tgw_planner_node`，但核心 C++ 类必须分离，避免继续形成巨型 `NavigationMap`。

---

## 3. 两种地图输入模式

### 3.1 Mode A: PCD 离线导入模式

输入：

```text
pcd_file:=/path/to/clean_static_map.pcd
```

用途：

```text
仿真验证
离线测试
干净静态地图导入
算法回归测试
```

强制要求：

1. 启动时打印明显 WARN：

```text
[PCD MODE WARNING]
Loading a final PCD without scan origins or timestamps.
Dynamic objects, human operators, SLAM ghost artifacts, and temporary obstacles
cannot be ray-cleared in PCD mode and may be treated as static structure.
This can severely corrupt floor/stair/traversability extraction.
Use realtime raycast mapping mode or a cleaned static PCD for deployment.
```

2. `/planner_stats_json` 或 `/map_build_stats_json` 中加入：

```json
{
  "map_input_mode": "pcd",
  "pcd_artifact_warning": true
}
```

3. 如果 PCD 模式下检测到大量悬浮/小簇/人形竖直结构，打印：

```text
[PCD MODE WARNING] possible artifacts detected; traversability may be unreliable
```

### 3.2 Mode B: 实时 Raycast 概率建图模式

输入：

```text
/tgw_mapping/points      sensor_msgs/PointCloud2
/tf 或 /tgw_mapping/pose  sensor pose in map frame
```

输出：

```text
概率占据地图
稳定静态结构层
free/occupied/unknown voxel layer
surface layer
clearance layer
navigation map snapshot
```

这是实机推荐模式。

---

## 4. 概率地图核心设计

### 4.1 VoxelState

新增：

```cpp
struct VoxelState
{
  float log_odds{0.0f};

  uint16_t hit_count{0};
  uint16_t miss_count{0};
  uint16_t ray_pass_count{0};

  double first_seen_time{0.0};
  double last_seen_time{0.0};
  double last_hit_time{0.0};
  double last_miss_time{0.0};

  uint16_t distinct_view_count{0};

  bool occupied{false};
  bool free{false};
  bool dynamic_suspect{false};
  bool static_candidate{false};
};
```

### 4.2 ProbabilisticVoxelMap

新增：

```cpp
class ProbabilisticVoxelMap
{
public:
  explicit ProbabilisticVoxelMap(double resolution_m);

  GridIndex worldToGrid(const Point3& p) const;
  Point3 gridToWorld(const GridIndex& idx) const;

  void updateHit(const GridIndex& idx, double stamp_sec, int view_id);
  void updateMiss(const GridIndex& idx, double stamp_sec, int view_id);

  bool isOccupied(const GridIndex& idx) const;
  bool isFree(const GridIndex& idx) const;
  bool isUnknown(const GridIndex& idx) const;

  float probability(const GridIndex& idx) const;
  const VoxelState* lookup(const GridIndex& idx) const;

  std::vector<GridIndex> occupiedVoxels() const;
  std::vector<GridIndex> freeVoxels() const;
  std::vector<GridIndex> staticCandidateVoxels() const;

  void decayDynamic(double now_sec);
  void clear();
};
```

### 4.3 Log-odds 参数

默认参数尽量少，从 occupancy mapping 常见设定开始：

```text
p_hit: 0.70
p_miss: 0.40
p_occupied_threshold: 0.65
p_free_threshold: 0.35
log_odds_min: -4.0
log_odds_max: 4.0
```

不要把这些散落在代码里。统一放入：

```cpp
struct MappingOptions
{
  double resolution_m{0.10};
  double max_range_m{30.0};
  double min_range_m{0.30};

  double p_hit{0.70};
  double p_miss{0.40};
  double p_occupied_threshold{0.65};
  double p_free_threshold{0.35};

  int min_static_hits{3};
  int min_distinct_views{2};
  double min_static_lifetime_sec{1.0};
  double dynamic_clear_ratio_threshold{0.65};

  bool enable_self_filter{true};
  bool enable_dynamic_filter{true};
};
```

---

## 5. Raycast Integrator

### 5.1 输入

```cpp
struct ScanInput
{
  std::vector<Point3> points_sensor_frame;
  Pose3 sensor_pose_map;
  double stamp_sec;
  int view_id;
};
```

### 5.2 自身点云过滤

Raycast 前过滤：

```text
min_range
max_range
NaN/Inf
机器人自身 bbox
传感器支架 bbox
过近点
```

默认自身过滤区域：

```text
robot_body_box in base frame:
  x: [-0.60, 0.60]
  y: [-0.40, 0.40]
  z: [-0.35, 0.80]
```

该参数必须可配置。

### 5.3 Raycast 更新

对每个点：

```text
sensor_origin -> endpoint
沿 ray 的中间 voxel：updateMiss()
endpoint voxel：updateHit()
```

使用 3D DDA / Bresenham。

伪代码：

```cpp
void RaycastIntegrator::insertScan(
    const sensor_msgs::msg::PointCloud2& cloud,
    const Transform& sensor_to_map,
    double stamp_sec)
{
  for point in cloud:
    if invalid / self / too near / too far:
      continue

    endpoint_map = sensor_to_map * point
    for voxel along ray excluding endpoint:
      map.updateMiss(voxel, stamp_sec, view_id)

    map.updateHit(endpoint_voxel, stamp_sec, view_id)
}
```

### 5.4 动态伪影抑制

一个 voxel 如果满足：

```text
hit_count 少
miss_count / ray_pass_count 多
被后续 ray 多次穿透
存在时间短
distinct_view_count 少
```

则标记：

```text
dynamic_suspect = true
static_candidate = false
```

这能抑制：

```text
人类操作员
移动人
临时物体
SLAM ghost
短时悬浮噪声
```

注意：如果一个人长时间静止并被多视角反复观察，概率地图仍可能把他当静态结构。因此必须在文档里说明：

> Raycast 和 temporal filtering 可以显著减少移动人和短时伪影，但不能保证删除长时间静止的人或长期存在的临时物体。建图时仍应尽量保持场景干净，必要时通过人工 blocked/erase API 修正。

---

## 6. Surface Extractor

目标不是直接识别楼梯中心线，而是提取**可站立 surface**。

输入：

```text
ProbabilisticVoxelMap
```

输出：

```text
surface_cells
traversable_cells
forbidden_cells
boundary_cells
surface_class labels: floor_like / slope_like / stair_like / unknown
```

### 6.1 Surface candidate

从概率 occupied voxels 的 vertical column 中提取 support surface：

```text
occupied z-run 顶部 + 上方 free/head clearance
```

不能再使用 raw PCD 的 “occupied 上方一格就是可走”。

### 6.2 Head clearance

候选站立位置必须满足：

```text
机器人高度范围内没有 occupied
或 occupied probability 低于阈值
```

### 6.3 Support check

候选必须有 support：

```text
下方存在稳定 occupied support
support voxel static_candidate == true
support 不是 dynamic_suspect
```

### 6.4 Footprint support

对矩形 footprint 采样，而不是点机器人：

```cpp
bool RobotFootprint::isSupported(
    const SurfaceMap& surface,
    const Point3& center,
    double yaw) const;
```

默认机器人尺寸：

```text
length: 0.70 m
width: 0.43 m
height: 0.50 m
base_to_front: 0.20 m
```

### 6.5 Thin semantic labels

只输出薄语义：

```text
floor_like
slope_like
stair_like
narrow
unknown
```

不要要求完整 StairFlight 才允许通行。

---

## 7. Boundary Layer

新增：

```cpp
class BoundaryExtractor
{
public:
  void rebuildBoundaryLayer(
      const SurfaceMap& surface,
      const ProbabilisticVoxelMap& occupancy);
};
```

一个 traversable cell 如果满足任一条件，则为 boundary：

```text
邻域中有非 traversable
邻域中有 forbidden / blocked
邻域中有 occupied wall / obstacle
邻域中 support 缺失形成 drop-off
邻域中高度突变超过 max_step_height
footprint 在邻域方向不安全
```

输出：

```text
boundary_cells_
dropoff_boundary_cells_
wall_boundary_cells_
forbidden_boundary_cells_
```

---

## 8. Clearance Field

新增：

```cpp
class ClearanceField
{
public:
  void compute(
      const std::unordered_set<GridIndex>& traversable,
      const std::unordered_set<GridIndex>& boundary,
      double resolution_m);

  double clearanceDistance(const GridIndex& cell) const;
  double clearancePenalty(const GridIndex& cell) const;
};
```

### 8.1 计算方法

使用 multi-source Dijkstra / brushfire：

```text
所有 boundary cell 初始 distance=0
在 traversable surface graph 上传播
edge cost = 3D Euclidean distance
```

### 8.2 Clearance penalty

建议：

```cpp
double clearancePenalty(GridIndex c)
{
  const double d = clearanceDistance(c);
  return 1.0 / (d + 0.05);
}
```

或者：

```cpp
exp(-d / sigma)
```

### 8.3 路径统计

规划后输出：

```text
min_path_clearance_m
mean_path_clearance_m
clearance_cost_sum
low_clearance_samples
```

---

## 9. Planner 重构

### 9.1 SurfaceAstarPlanner

替代或重写 `VoxelAstarPlanner`。

```cpp
class SurfaceAstarPlanner
{
public:
  PlanResult plan(
      const NavigationSnapshot& map,
      const Point3& start,
      const Point3& goal);
};
```

搜索空间：

```text
traversable surface cells
```

硬约束：

```text
cell traversable
not blocked
not forbidden
footprint supported
swept footprint transition supported
no illegal drop-off
```

软代价：

```cpp
cost =
  step_distance
  + w_clearance * clearancePenalty(neighbor)
  + w_risk * riskCost(neighbor)
  + w_slope * slopePenalty(neighbor)
  + w_turn * turnPenalty(current, neighbor)
  + w_unknown * unknownPenalty(neighbor);
```

默认权重：

```text
w_clearance: 0.8
w_risk: 1.5
w_slope: 0.3
w_turn: 0.1
w_unknown: 2.0
```

这些权重可以作为参数，但不要把系统调参变成主要开发方向。

### 9.2 旋转楼梯处理

不再要求旋转楼梯必须被建模成 `CurvedStairFlight`。

只要：

```text
旋转楼梯 surface traversable
内外边界正确
clearance field 正确
```

A* 就应该自然沿旋转楼梯中间绕行。

---

## 10. Path Post-processing

后处理不能只检查可通行，还要检查 clearance。

### 10.1 Shortcut 规则

`line-of-sight shortcut` 允许条件：

```text
所有采样点 traversable
footprint transition valid
不穿越 forbidden / blocked
不穿越非法高度突变
shortcut 的 min clearance >= 原路径该段 min clearance * 0.8
shortcut 的 min clearance >= robot_width / 2 + safety_margin
```

否则拒绝 shortcut。

### 10.2 最终路径验证

任何后处理后的路径必须验证：

```cpp
bool validateFinalPath(
  const NavigationSnapshot& map,
  const std::vector<Point3>& path,
  ValidationReport& report);
```

验证失败：

```text
fallback raw A* path
如果 raw A* 也失败，返回 failure
```

必须输出：

```text
final_path_validated
final_path_fallback_to_raw
final_path_validation_failure
```

---

## 11. ROS 接口

### 11.1 Realtime Mapping Node

节点：

```text
/tgw_realtime_mapping_node
```

订阅：

```text
/tgw_mapping/points sensor_msgs/PointCloud2
/tf
```

可选订阅：

```text
/tgw_mapping/pose geometry_msgs/PoseStamped
```

发布：

```text
/tgw_map/occupied_cloud
/tgw_map/free_cloud
/tgw_map/dynamic_suspect_cloud
/tgw_map/static_candidate_cloud
/tgw_map/surface_cloud
/tgw_map/traversable_cloud
/tgw_map/boundary_cloud
/tgw_map/clearance_cloud
/tgw_map/medial_axis_cloud
/tgw_map/stats_json
```

服务：

```text
/tgw_mapping/start
/tgw_mapping/stop
/tgw_mapping/pause
/tgw_mapping/clear
/tgw_mapping/save_map
/tgw_mapping/export_static_pcd
/tgw_mapping/get_snapshot
```

### 11.2 Planner Node

节点：

```text
/tgw_planner_node
```

输入：

```text
/map snapshot from realtime mapping
或 clean PCD imported map
/start_pose
/goal_pose
```

输出：

```text
/planned_path
/planned_path_marker
/planner_stats
/planner_stats_json
```

服务：

```text
/plan_path
/nav_map/set_blocked_region
```

---

## 12. RViz Debug Topics

必须提供：

```text
/tgw_map/occupied_cloud
/tgw_map/free_cloud
/tgw_map/dynamic_suspect_cloud
/tgw_map/static_candidate_cloud
/tgw_map/surface_cloud
/tgw_map/traversable_cloud
/tgw_map/boundary_cloud
/tgw_map/dropoff_boundary_cloud
/tgw_map/wall_boundary_cloud
/tgw_map/clearance_cloud
/tgw_map/medial_axis_cloud
/tgw_map/forbidden_cloud
/tgw_map/blocked_cloud
/planned_path
/planned_path_marker
/start_marker
/goal_marker
```

旧的 stair-specific debug 可以暂时保留，但不能作为主路径逻辑的必需项。

---

## 13. PCD 模式警告和文档

新增文档：

```text
docs/map_input_modes.md
```

必须明确写：

```text
PCD 模式无法利用 ray clearing 和 temporal consistency。
如果 PCD 里包含人类操作员、移动物体残影、SLAM ghost、临时障碍、机器人自身点云，
系统会把它们视为静态 occupied structure。
这会严重影响：
- floor extraction
- stair/slope detection
- boundary detection
- clearance field
- traversability
- path planning
```

建议：

```text
实机部署必须使用 realtime raycast mapping mode。
PCD 模式只用于 clean static PCD、仿真、离线测试。
```

---

## 14. 测试要求

### 14.1 单元测试

新增 C++ 或 Python 测试：

```text
test_log_odds_update
test_raycast_free_clearing
test_dynamic_object_cleared_after_misses
test_static_wall_kept_after_multiple_views
test_surface_extraction_floor_ceiling
test_clearance_centerline_corridor
test_shortcut_rejected_when_clearance_drops
test_final_path_fallback_to_raw
```

### 14.2 场景测试

必须至少覆盖：

```text
1. PCT building
2. PCT spiral
3. straight stair synthetic
4. switchback stair synthetic
5. spiral stair synthetic
6. flat corridor narrow passage
7. negative sample: wall/ceiling/railings but no stairs
8. dirty PCD: human-like vertical artifact
9. dynamic scan sequence: human appears then disappears
10. blocked region detour / unreachable
```

### 14.3 成功标准

每个场景输出：

```json
{
  "success": true,
  "final_path_validated": true,
  "fallback_to_raw": false,
  "min_path_clearance_m": 0.32,
  "mean_path_clearance_m": 0.55,
  "expanded_nodes": 12345,
  "path_length_m": 42.0,
  "build_time_ms": 1000.0
}
```

不要只检查 `success=True`。

---

## 15. Codex 执行阶段

### Phase 1: Core Map Rewrite

实现：

```text
ProbabilisticVoxelMap
RaycastIntegrator
RobotFootprint
MappingOptions
```

验收：

```text
能插入简单 scan
raycast 能标记 free 和 occupied
后续 ray 能清除动态 voxel
```

### Phase 2: Surface + Clearance

实现：

```text
SurfaceExtractor
BoundaryExtractor
ClearanceField
```

验收：

```text
平面走廊中 clearance 最大区域在中间
墙边 clearance 小
drop-off 边缘 clearance 小
```

### Phase 3: Planner Rewrite

实现：

```text
SurfaceAstarPlanner
clearance-aware cost
final path validation
clearance-aware shortcut
```

验收：

```text
窄通道走中线
旋转楼梯沿中间绕行
平面 shortcut 不贴墙
```

### Phase 4: Realtime ROS Integration

实现：

```text
/tgw_realtime_mapping_node
/tgw_planner_node
snapshot service
debug topics
```

验收：

```text
实时点云 + 位姿能建图
RViz 能看到 occupancy/free/surface/boundary/clearance
可以从 start/goal 规划路径
```

### Phase 5: PCD Import Mode

实现：

```text
pcd_import_node
PCD mode warning
artifact diagnostics
clean PCD input
```

验收：

```text
加载 PCD 时有明确 warning
clean PCD 可以规划
dirty PCD 会提示 artifact risk
```

### Phase 6: Regression Tests

实现：

```text
scripts/run_reference_pcd_smoke_tests.sh
scripts/run_realtime_mapping_sim_tests.sh
scripts/run_dirty_map_tests.sh
```

验收：

```text
PCT building pass
PCT spiral pass
dirty dynamic artifact test pass
blocked region test pass
```

---

## 16. 明确禁止事项

Codex 不要做：

```text
不要继续围绕 StairFlight 打补丁
不要为了一个地图调常数
不要用中心线数量作为主验收
不要只验证一张 PCD
不要输出未经 final validation 的后处理路径
不要把 PCD 模式当作实机推荐建图模式
不要把动态伪影当静态结构
```

---

## 17. 最终目标

最终系统应该满足：

```text
实时输入点云和位姿
    ↓
概率 raycast 地图
    ↓
动态伪影通过 miss/ray clearing/temporal consistency 被削弱
    ↓
静态 surface 被提取
    ↓
clearance field 引导路径走中线
    ↓
楼梯/旋转楼梯/窄走廊不需要单独硬编码中线
    ↓
机器人得到一条经过 footprint 验证、远离边界的全局路径
```

一句话：

**从“识别楼梯中心线”重构为“在概率静态 surface 上做 clearance-aware 安全路径规划”。**
