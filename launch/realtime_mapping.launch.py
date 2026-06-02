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
    p_hit = LaunchConfiguration("p_hit")
    p_miss = LaunchConfiguration("p_miss")
    p_occupied_threshold = LaunchConfiguration("p_occupied_threshold")
    p_free_threshold = LaunchConfiguration("p_free_threshold")
    min_static_hits = LaunchConfiguration("min_static_hits")
    min_distinct_views = LaunchConfiguration("min_distinct_views")
    min_static_lifetime_sec = LaunchConfiguration("min_static_lifetime_sec")
    enable_self_filter = LaunchConfiguration("enable_self_filter")
    enable_dynamic_filter = LaunchConfiguration("enable_dynamic_filter")
    max_points_per_scan = LaunchConfiguration("max_points_per_scan")
    publish_period_ms = LaunchConfiguration("publish_period_ms")
    medial_axis_min_clearance_m = LaunchConfiguration("medial_axis_min_clearance_m")
    surface_min_static_hits = LaunchConfiguration("surface_min_static_hits")
    surface_require_static_support = LaunchConfiguration("surface_require_static_support")
    robot_height_m = LaunchConfiguration("robot_height_m")
    robot_length_m = LaunchConfiguration("robot_length_m")
    robot_width_m = LaunchConfiguration("robot_width_m")
    base_to_front_m = LaunchConfiguration("base_to_front_m")
    max_step_height_m = LaunchConfiguration("max_step_height_m")
    planner_w_clearance = LaunchConfiguration("planner_w_clearance")
    planner_w_risk = LaunchConfiguration("planner_w_risk")
    planner_w_slope = LaunchConfiguration("planner_w_slope")
    planner_w_turn = LaunchConfiguration("planner_w_turn")
    planner_w_unknown = LaunchConfiguration("planner_w_unknown")
    planner_max_iterations = LaunchConfiguration("planner_max_iterations")
    planner_require_footprint = LaunchConfiguration("planner_require_footprint")
    planner_swept_sample_step_m = LaunchConfiguration("planner_swept_sample_step_m")
    planner_enable_shortcut = LaunchConfiguration("planner_enable_shortcut")
    planner_shortcut_sample_step_m = LaunchConfiguration("planner_shortcut_sample_step_m")
    planner_shortcut_clearance_ratio = LaunchConfiguration("planner_shortcut_clearance_ratio")
    planner_shortcut_safety_margin_m = LaunchConfiguration("planner_shortcut_safety_margin_m")
    validation_sample_step_m = LaunchConfiguration("validation_sample_step_m")
    validation_min_clearance_m = LaunchConfiguration("validation_min_clearance_m")
    validation_require_footprint = LaunchConfiguration("validation_require_footprint")
    risk_boundary = LaunchConfiguration("risk_boundary")
    risk_dropoff = LaunchConfiguration("risk_dropoff")
    risk_wall = LaunchConfiguration("risk_wall")
    risk_forbidden = LaunchConfiguration("risk_forbidden")
    risk_low_clearance = LaunchConfiguration("risk_low_clearance")
    risk_low_clearance_threshold_m = LaunchConfiguration("risk_low_clearance_threshold_m")

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
            DeclareLaunchArgument("p_hit", default_value="0.70"),
            DeclareLaunchArgument("p_miss", default_value="0.40"),
            DeclareLaunchArgument("p_occupied_threshold", default_value="0.65"),
            DeclareLaunchArgument("p_free_threshold", default_value="0.35"),
            DeclareLaunchArgument("min_static_hits", default_value="3"),
            DeclareLaunchArgument("min_distinct_views", default_value="2"),
            DeclareLaunchArgument("min_static_lifetime_sec", default_value="1.0"),
            DeclareLaunchArgument("enable_self_filter", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_filter", default_value="true"),
            DeclareLaunchArgument("max_points_per_scan", default_value="120000"),
            DeclareLaunchArgument("publish_period_ms", default_value="1000"),
            DeclareLaunchArgument("medial_axis_min_clearance_m", default_value="0.20"),
            DeclareLaunchArgument("surface_min_static_hits", default_value="1"),
            DeclareLaunchArgument("surface_require_static_support", default_value="false"),
            DeclareLaunchArgument("robot_height_m", default_value="0.50"),
            DeclareLaunchArgument("robot_length_m", default_value="0.70"),
            DeclareLaunchArgument("robot_width_m", default_value="0.43"),
            DeclareLaunchArgument("base_to_front_m", default_value="0.20"),
            DeclareLaunchArgument("max_step_height_m", default_value="0.30"),
            DeclareLaunchArgument("planner_w_clearance", default_value="0.80"),
            DeclareLaunchArgument("planner_w_risk", default_value="1.50"),
            DeclareLaunchArgument("planner_w_slope", default_value="0.30"),
            DeclareLaunchArgument("planner_w_turn", default_value="0.10"),
            DeclareLaunchArgument("planner_w_unknown", default_value="2.0"),
            DeclareLaunchArgument("planner_max_iterations", default_value="250000"),
            DeclareLaunchArgument("planner_require_footprint", default_value="true"),
            DeclareLaunchArgument("planner_swept_sample_step_m", default_value="0.05"),
            DeclareLaunchArgument("planner_enable_shortcut", default_value="true"),
            DeclareLaunchArgument("planner_shortcut_sample_step_m", default_value="0.05"),
            DeclareLaunchArgument("planner_shortcut_clearance_ratio", default_value="0.80"),
            DeclareLaunchArgument("planner_shortcut_safety_margin_m", default_value="0.02"),
            DeclareLaunchArgument("validation_sample_step_m", default_value="0.05"),
            DeclareLaunchArgument("validation_min_clearance_m", default_value="0.0"),
            DeclareLaunchArgument("validation_require_footprint", default_value="true"),
            DeclareLaunchArgument("risk_boundary", default_value="0.50"),
            DeclareLaunchArgument("risk_dropoff", default_value="1.50"),
            DeclareLaunchArgument("risk_wall", default_value="1.00"),
            DeclareLaunchArgument("risk_forbidden", default_value="2.00"),
            DeclareLaunchArgument("risk_low_clearance", default_value="1.00"),
            DeclareLaunchArgument("risk_low_clearance_threshold_m", default_value="0.35"),
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
                        "p_hit": p_hit,
                        "p_miss": p_miss,
                        "p_occupied_threshold": p_occupied_threshold,
                        "p_free_threshold": p_free_threshold,
                        "min_static_hits": min_static_hits,
                        "min_distinct_views": min_distinct_views,
                        "min_static_lifetime_sec": min_static_lifetime_sec,
                        "enable_self_filter": enable_self_filter,
                        "enable_dynamic_filter": enable_dynamic_filter,
                        "max_points_per_scan": max_points_per_scan,
                        "publish_period_ms": publish_period_ms,
                        "medial_axis_min_clearance_m": medial_axis_min_clearance_m,
                        "surface_min_static_hits": surface_min_static_hits,
                        "surface_require_static_support": surface_require_static_support,
                        "robot_height_m": robot_height_m,
                        "robot_length_m": robot_length_m,
                        "robot_width_m": robot_width_m,
                        "base_to_front_m": base_to_front_m,
                        "max_step_height_m": max_step_height_m,
                        "planner_w_clearance": planner_w_clearance,
                        "planner_w_risk": planner_w_risk,
                        "planner_w_slope": planner_w_slope,
                        "planner_w_turn": planner_w_turn,
                        "planner_w_unknown": planner_w_unknown,
                        "planner_max_iterations": planner_max_iterations,
                        "planner_require_footprint": planner_require_footprint,
                        "planner_swept_sample_step_m": planner_swept_sample_step_m,
                        "planner_enable_shortcut": planner_enable_shortcut,
                        "planner_shortcut_sample_step_m": planner_shortcut_sample_step_m,
                        "planner_shortcut_clearance_ratio": planner_shortcut_clearance_ratio,
                        "planner_shortcut_safety_margin_m": planner_shortcut_safety_margin_m,
                        "validation_sample_step_m": validation_sample_step_m,
                        "validation_min_clearance_m": validation_min_clearance_m,
                        "validation_require_footprint": validation_require_footprint,
                        "risk_boundary": risk_boundary,
                        "risk_dropoff": risk_dropoff,
                        "risk_wall": risk_wall,
                        "risk_forbidden": risk_forbidden,
                        "risk_low_clearance": risk_low_clearance,
                        "risk_low_clearance_threshold_m": risk_low_clearance_threshold_m,
                    }
                ],
            ),
        ]
    )
