"""Action-aware semantic navigation for Stretch3 in Isaac Sim.

Implements the approach from arXiv:2310.08873.

Brings up the Nav2 stack with the SemanticTraversabilityLayer plus the
perception/projection pipeline that feeds it:

  Nav2 (nav2_params.yaml, semantic layer enabled)
  + projection_node  (detection + /depth + /camera_info -> /semantic_regions)
  + perception       (one of):
      perception:=locate_anything -> LocateAnything node (default)
      perception:=dino            -> Grounding DINO node
      perception:=static          -> static_region_publisher
      perception:=none            -> publish /semantic_regions yourself

Prerequisites (provided by Isaac Sim): world->odom->base_link TF, /laser_scan,
/odom, /rgb, /depth, /camera_info.
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
        "use_sim_time", default_value="True",
        description="Enable use_sim_time for Isaac Sim",
    ),
    DeclareLaunchArgument(
        "perception", default_value="locate_anything",
        choices=["locate_anything", "dino", "static", "none"],
        description="Semantic region source: LocateAnything, Grounding DINO, "
                    "static test publisher, or none.",
    ),
    DeclareLaunchArgument(
        "target_frame", default_value="odom",
        description="Frame the projected ground polygons are published in.",
    ),
    DeclareLaunchArgument(
        "region_hold_sec", default_value="-1.0",
        description="How long to keep a detected region alive after the last "
                    "confirmed detection (rides out close-range detector "
                    "drop-outs). <0 = hold forever; 0 = no hold; >0 = seconds "
                    "(sim time).",
    ),
    DeclareLaunchArgument(
        "region_confirmation_hits", default_value="2",
        description="Number of spatially consistent detections required before "
                    "a semantic region is held in memory.",
    ),
    DeclareLaunchArgument(
        "region_match_distance_m", default_value="0.60",
        description="Maximum centroid distance for detections to count as the "
                    "same semantic region.",
    ),
    DeclareLaunchArgument(
        "pending_region_ttl_sec", default_value="5.0",
        description="How long an unconfirmed semantic region candidate may wait "
                    "for another matching detection.",
    ),
    DeclareLaunchArgument(
        "depth_topic", default_value="/depth",
        description="Aligned depth image topic.",
    ),
    DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera_info",
        description="Camera intrinsics topic.",
    ),
    DeclareLaunchArgument(
        "detection_viz", default_value="true",
        description="Draw VLM boxes onto the RGB image and publish "
                    "/semantic_detection_viz for RViz.",
    ),
    DeclareLaunchArgument(
        "rgb_topic", default_value="/rgb",
        description="RGB image topic for the VLM.",
    ),
    DeclareLaunchArgument(
        "perception_params_file",
        default_value="",
        description="Optional YAML file for the selected perception backend. "
                    "When empty, the backend default in semantic_perception/"
                    "config is used.",
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
                },
            ],
            remappings=[("~/instruction", "/semantic_instruction")],
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("stretch3_navigation"), "config", "nav2_params.yaml"]
    )

    nav2 = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ]
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
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "world"}],
    )

    detection_viz = Node(
        package="semantic_perception",
        executable="detection_viz_node",
        name="detection_viz_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("detection_viz")),
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/rgb", LaunchConfiguration("rgb_topic"))],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    ld.add_action(projection)
    ld.add_action(perception_node)
    ld.add_action(static_pub)
    ld.add_action(detection_viz)
    return ld
