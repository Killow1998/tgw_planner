from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_pbstream(context):
    pbstream_path = LaunchConfiguration("pbstream_path").perform(context)
    if not pbstream_path:
        raise RuntimeError("pbstream_path is required")
    import os
    if not os.path.exists(pbstream_path):
        raise RuntimeError(f"pbstream_path does not exist: {pbstream_path}")
    return []


def generate_launch_description():
    package_share = FindPackageShare("tgw_planner")
    common_params_file = LaunchConfiguration("common_params_file")
    sim_params_file = LaunchConfiguration("sim_params_file")
    pbstream_path = LaunchConfiguration("pbstream_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_clicked_point_router = LaunchConfiguration("use_clicked_point_router")
    use_rviz = LaunchConfiguration("use_rviz")
    kinematic_replay_odom_topic = LaunchConfiguration("kinematic_replay_odom_topic")
    tracking_cmd_vel_topic = LaunchConfiguration("tracking_cmd_vel_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pbstream_path",
                default_value=PathJoinSubstitution(
                    [
                        package_share,
                        "docs",
                        "data",
                        "tgw_n3map_nav_filtered_20260610.pbstream",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "common_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "experience_planner_common.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "sim_params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "experience_planner_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_clicked_point_router", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "kinematic_replay_odom_topic", default_value="/tgw_experience/fake_odom"
            ),
            DeclareLaunchArgument("tracking_cmd_vel_topic", default_value="/tgw_experience/cmd_vel"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "experience_sim.rviz"]
                ),
            ),
            OpaqueFunction(function=_validate_pbstream),
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
                parameters=[
                    common_params_file,
                    sim_params_file,
                    {
                        "pbstream_path": pbstream_path,
                        "tracking_odom_topic": kinematic_replay_odom_topic,
                        "tracking_cmd_vel_topic": tracking_cmd_vel_topic,
                        "kinematic_replay_odom_topic": kinematic_replay_odom_topic,
                    },
                ],
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
