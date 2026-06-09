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
    rviz_config = LaunchConfiguration("rviz_config")
    use_clicked_point_router = LaunchConfiguration("use_clicked_point_router")
    use_rviz = LaunchConfiguration("use_rviz")

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
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "experience_debug.rviz"]
                ),
            ),
            Node(
                package="tgw_planner",
                executable="tgw_experience_planner_node",
                name="tgw_experience_planner_node",
                output="log",
                arguments=[
                    "--ros-args",
                    "--disable-stdout-logs",
                    "--disable-rosout-logs",
                    "--disable-external-lib-logs",
                ],
                parameters=[params_file, {"pbstream_path": pbstream_path}],
            ),
            Node(
                package="tgw_planner",
                executable="tgw_clicked_point_router_node",
                name="tgw_clicked_point_router_node",
                output="log",
                arguments=[
                    "--ros-args",
                    "--disable-stdout-logs",
                    "--disable-rosout-logs",
                    "--disable-external-lib-logs",
                ],
                condition=IfCondition(use_clicked_point_router),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
