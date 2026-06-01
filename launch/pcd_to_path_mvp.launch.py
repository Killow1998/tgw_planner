from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pcd_file = LaunchConfiguration("pcd_file")
    robot_radius_m = LaunchConfiguration("robot_radius_m")
    robot_height_m = LaunchConfiguration("robot_height_m")
    robot_length_m = LaunchConfiguration("robot_length_m")
    robot_width_m = LaunchConfiguration("robot_width_m")
    base_to_front_m = LaunchConfiguration("base_to_front_m")
    map_resolution_m = LaunchConfiguration("map_resolution_m")
    map_frame = LaunchConfiguration("map_frame")
    map_id = LaunchConfiguration("map_id")
    save_map_dir = LaunchConfiguration("save_map_dir")
    max_iterations = LaunchConfiguration("max_iterations")
    max_marker_cells = LaunchConfiguration("max_marker_cells")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("tgw_planner"), "rviz", "tgw_planner.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcd_file",
                default_value="",
            ),
            DeclareLaunchArgument("robot_radius_m", default_value="0.35"),
            DeclareLaunchArgument("robot_height_m", default_value="0.50"),
            DeclareLaunchArgument("robot_length_m", default_value="0.70"),
            DeclareLaunchArgument("robot_width_m", default_value="0.43"),
            DeclareLaunchArgument("base_to_front_m", default_value="0.20"),
            DeclareLaunchArgument("map_resolution_m", default_value="0.20"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("map_id", default_value="tgw_nav_map"),
            DeclareLaunchArgument("save_map_dir", default_value=""),
            DeclareLaunchArgument("max_iterations", default_value="250000"),
            DeclareLaunchArgument("max_marker_cells", default_value="120000"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="tgw_planner",
                executable="tgw_planner_node",
                name="tgw_planner_node",
                output="screen",
                parameters=[
                    {
                        "pcd_file": pcd_file,
                        "robot_radius_m": robot_radius_m,
                        "robot_height_m": robot_height_m,
                        "robot_length_m": robot_length_m,
                        "robot_width_m": robot_width_m,
                        "base_to_front_m": base_to_front_m,
                        "map_resolution_m": map_resolution_m,
                        "map_frame": map_frame,
                        "map_id": map_id,
                        "save_map_dir": save_map_dir,
                        "max_iterations": max_iterations,
                        "max_marker_cells": max_marker_cells,
                    }
                ],
            ),
            Node(
                package="tgw_planner",
                executable="tgw_clicked_point_router_node",
                name="tgw_clicked_point_router_node",
                output="screen",
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
