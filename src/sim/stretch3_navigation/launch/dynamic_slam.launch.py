"""Dynamic RTAB-Map SLAM pipeline for Stretch3 in Isaac Sim.

Pipeline:
  RTAB-Map raw occupancy grid     -> /rtabmap/map
  semantic perception/projection  -> /semantic_regions
  dynamic_map_updater_node        -> /map
  Nav2 global StaticLayer         <- /map

The updater only clears stale occupied cells in /map after repeated LiDAR
free-space evidence through configured dynamic-object semantic regions. The
local costmap still uses live /laser_scan obstacle marking for collision safety.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


PERCEPTION_BACKENDS = {
    "locate_anything": {
        "package": "semantic_perception",
        "executable": "locate_anything_node",
        "name": "locate_anything_node",
        "default_params": "locate_anything_params.yaml",
    },
    "dino": {
        "package": "semantic_perception",
        "executable": "grounding_dino_node",
        "name": "grounding_dino_node",
        "default_params": "grounding_dino_params.yaml",
    },
}


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Enable use_sim_time for Isaac Sim.",
    ),
    DeclareLaunchArgument(
        "perception",
        default_value="locate_anything",
        choices=["locate_anything", "dino", "static", "none"],
        description="Semantic region source.",
    ),
    DeclareLaunchArgument(
        "target_frame",
        default_value="map",
        description="Frame for projected semantic ground polygons.",
    ),
    DeclareLaunchArgument(
        "region_hold_sec",
        default_value="-1.0",
        description="How long to keep confirmed regions alive; <0 holds forever.",
    ),
    DeclareLaunchArgument(
        "region_confirmation_hits",
        default_value="2",
        description="Number of spatially consistent detections required before "
                    "a semantic region is held in memory.",
    ),
    DeclareLaunchArgument(
        "region_match_distance_m",
        default_value="0.60",
        description="Maximum centroid distance for detections to count as the "
                    "same semantic region.",
    ),
    DeclareLaunchArgument(
        "pending_region_ttl_sec",
        default_value="5.0",
        description="How long an unconfirmed semantic region candidate may wait "
                    "for another matching detection.",
    ),
    DeclareLaunchArgument(
        "depth_topic",
        default_value="/depth",
        description="Aligned depth image topic.",
    ),
    DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera_info",
        description="Camera intrinsics topic.",
    ),
    DeclareLaunchArgument(
        "rgb_topic",
        default_value="/rgb",
        description="RGB image topic for perception.",
    ),
    DeclareLaunchArgument(
        "scan_topic",
        default_value="/laser_scan",
        description="LaserScan topic for RTAB-Map, Nav2, and the updater.",
    ),
    DeclareLaunchArgument(
        "detection_viz",
        default_value="true",
        description="Publish /semantic_detection_viz.",
    ),
    DeclareLaunchArgument(
        "rgb_rotation",
        default_value="clockwise_90",
        choices=["none", "clockwise_90", "counterclockwise_90", "180"],
        description="Right-angle correction applied to /rgb before the VLM "
                    "sees it (the Stretch head camera publishes sideways). "
                    "Detections are mapped back to raw-camera pixels, so "
                    "projection stays aligned with /depth. The same rotation "
                    "is applied to /semantic_detection_viz.",
    ),
    DeclareLaunchArgument(
        "perception_params_file",
        default_value="",
        description="Optional YAML file for the selected perception backend.",
    ),
]


def _default_perception_params_file(backend: str) -> str:
    return os.path.join(
        get_package_share_directory("semantic_perception"),
        "config",
        PERCEPTION_BACKENDS[backend]["default_params"],
    )


def _make_perception_node(context, *args, **kwargs):
    backend_name = LaunchConfiguration("perception").perform(context)
    backend = PERCEPTION_BACKENDS.get(backend_name)
    if backend is None:
        return []

    params_file = LaunchConfiguration("perception_params_file").perform(
        context
    )
    if not params_file:
        params_file = _default_perception_params_file(backend_name)

    targets_file = os.path.join(
        get_package_share_directory("semantic_perception"),
        "config",
        "semantic_targets.yaml",
    )

    return [
        Node(
            package=backend["package"],
            executable=backend["executable"],
            name=backend["name"],
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "detection_topic": "/semantic_detection",
                    "targets_file": targets_file,
                    "rgb_rotation": LaunchConfiguration("rgb_rotation"),
                },
            ],
            remappings=[("~/instruction", "/semantic_instruction")],
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")

    rtabmap_settings = PathJoinSubstitution(
        [FindPackageShare("stretch3_navigation"), "config", "rtabmap.yaml"]
    )
    nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("stretch3_navigation"),
            "config",
            "nav2_dynamic_slam_params.yaml",
        ]
    )
    dynamic_map_params_file = PathJoinSubstitution(
        [
            FindPackageShare("stretch3_navigation"),
            "config",
            "dynamic_map_updater.yaml",
        ]
    )

    remappings = [
        ("rgb/image", "/rgb"),
        ("rgb/camera_info", "/camera_info"),
        ("depth/image", "/depth"),
        ("scan", scan_topic),
        ("map", "/rtabmap/map"),
    ]

    rtabmap_slam = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_settings, {"use_sim_time": use_sim_time}],
        remappings=remappings,
        arguments=["-d"],
    )

    static_tf_world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    dynamic_map_updater = Node(
        package="dynamic_mapping",
        executable="dynamic_map_updater_node",
        name="dynamic_map_updater_node",
        output="screen",
        parameters=[
            dynamic_map_params_file,
            {
                "use_sim_time": use_sim_time,
                "scan_topic": scan_topic,
            },
        ],
    )

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    projection = Node(
        package="semantic_traversability",
        executable="projection_node",
        name="semantic_projection_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "regions_topic": "/semantic_regions",
            "target_frame": LaunchConfiguration("target_frame"),
            "region_hold_sec": ParameterValue(
                LaunchConfiguration("region_hold_sec"), value_type=float),
            "region_confirmation_hits": ParameterValue(
                LaunchConfiguration("region_confirmation_hits"), value_type=int),
            "region_match_distance_m": ParameterValue(
                LaunchConfiguration("region_match_distance_m"), value_type=float),
            "pending_region_ttl_sec": ParameterValue(
                LaunchConfiguration("pending_region_ttl_sec"), value_type=float),
        }],
        remappings=[("~/detection", "/semantic_detection")],
    )

    perception_node = OpaqueFunction(function=_make_perception_node)

    static_pub = Node(
        package="semantic_perception",
        executable="static_region_publisher",
        name="static_region_publisher",
        output="screen",
        condition=LaunchConfigurationEquals("perception", "static"),
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "map"}],
    )

    detection_viz = Node(
        package="semantic_perception",
        executable="detection_viz_node",
        name="detection_viz_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("detection_viz")),
        parameters=[{
            "use_sim_time": use_sim_time,
            "rgb_rotation": LaunchConfiguration("rgb_rotation"),
        }],
        remappings=[("/rgb", LaunchConfiguration("rgb_topic"))],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rtabmap_slam)
    ld.add_action(static_tf_world_to_map)
    ld.add_action(projection)
    ld.add_action(perception_node)
    ld.add_action(static_pub)
    ld.add_action(detection_viz)
    ld.add_action(dynamic_map_updater)
    ld.add_action(nav2)
    return ld
