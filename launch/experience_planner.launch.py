from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("tgw_planner")
    params_file = LaunchConfiguration("params_file")
    pbstream_path = LaunchConfiguration("pbstream_path")
    use_clicked_point_router = LaunchConfiguration("use_clicked_point_router")

    return LaunchDescription(
        [
            DeclareLaunchArgument("pbstream_path", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "experience_planner_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_clicked_point_router", default_value="true"),
            Node(
                package="tgw_planner",
                executable="tgw_experience_planner_node",
                name="tgw_experience_planner_node",
                output="screen",
                parameters=[params_file, {"pbstream_path": pbstream_path}],
            ),
            Node(
                package="tgw_planner",
                executable="tgw_clicked_point_router_node",
                name="tgw_clicked_point_router_node",
                output="screen",
                condition=IfCondition(use_clicked_point_router),
            ),
        ]
    )
