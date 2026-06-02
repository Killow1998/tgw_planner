from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    points_topic = LaunchConfiguration("points_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    map_frame = LaunchConfiguration("map_frame")
    sensor_frame = LaunchConfiguration("sensor_frame")
    use_tf = LaunchConfiguration("use_tf")
    assume_cloud_in_map_frame = LaunchConfiguration("assume_cloud_in_map_frame")
    resolution_m = LaunchConfiguration("resolution_m")
    max_range_m = LaunchConfiguration("max_range_m")
    min_range_m = LaunchConfiguration("min_range_m")
    max_points_per_scan = LaunchConfiguration("max_points_per_scan")
    publish_period_ms = LaunchConfiguration("publish_period_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument("points_topic", default_value="/tgw_mapping/points"),
            DeclareLaunchArgument("pose_topic", default_value="/tgw_mapping/pose"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("sensor_frame", default_value=""),
            DeclareLaunchArgument("use_tf", default_value="true"),
            DeclareLaunchArgument("assume_cloud_in_map_frame", default_value="false"),
            DeclareLaunchArgument("resolution_m", default_value="0.10"),
            DeclareLaunchArgument("max_range_m", default_value="30.0"),
            DeclareLaunchArgument("min_range_m", default_value="0.30"),
            DeclareLaunchArgument("max_points_per_scan", default_value="120000"),
            DeclareLaunchArgument("publish_period_ms", default_value="1000"),
            Node(
                package="tgw_planner",
                executable="tgw_realtime_mapping_node",
                name="tgw_realtime_mapping_node",
                output="screen",
                parameters=[
                    {
                        "points_topic": points_topic,
                        "pose_topic": pose_topic,
                        "map_frame": map_frame,
                        "sensor_frame": sensor_frame,
                        "use_tf": use_tf,
                        "assume_cloud_in_map_frame": assume_cloud_in_map_frame,
                        "resolution_m": resolution_m,
                        "max_range_m": max_range_m,
                        "min_range_m": min_range_m,
                        "max_points_per_scan": max_points_per_scan,
                        "publish_period_ms": publish_period_ms,
                    }
                ],
            ),
        ]
    )
