cmake_minimum_required(VERSION 3.10)
project(tgw_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(octomap REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PlannerStats.msg
  srv/PlanPath.srv
  srv/SetBlockedRegion.srv
  srv/SetClickMode.srv
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
)

add_library(tgw_planner_core SHARED
  src/core/clearance_field.cpp
  src/core/navigation_map.cpp
  src/core/probabilistic_voxel_map.cpp
  src/core/raycast_integrator.cpp
  src/core/robot_footprint.cpp
  src/core/surface_extractor.cpp
  src/core/voxel_astar_planner.cpp
)
target_include_directories(tgw_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(tgw_planner_core
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
target_compile_definitions(tgw_planner_core PUBLIC ${PCL_DEFINITIONS})

add_executable(tgw_planner_node
  src/humble/tgw_planner_node.cpp
)
target_link_libraries(tgw_planner_node
  tgw_planner_core
)
ament_target_dependencies(tgw_planner_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_executable(tgw_clicked_point_router_node
  src/humble/clicked_point_router_node.cpp
)
ament_target_dependencies(tgw_clicked_point_router_node
  geometry_msgs
  rclcpp
)

add_library(tgw_planner_rviz_plugins SHARED
  src/rviz_plugins/tgw_pose_tool.cpp
)
ament_target_dependencies(tgw_planner_rviz_plugins
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  visualization_msgs
)
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tgw_planner_node "${cpp_typesupport_target}")
target_link_libraries(tgw_clicked_point_router_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  add_executable(tgw_phase1_core_smoke
    test/phase1_core_smoke.cpp
  )
  target_link_libraries(tgw_phase1_core_smoke
    tgw_planner_core
  )
  add_test(NAME tgw_phase1_core_smoke COMMAND tgw_phase1_core_smoke)
endif()

install(TARGETS
  tgw_planner_core
  tgw_planner_node
  tgw_clicked_point_router_node
  tgw_planner_rviz_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config docs launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
