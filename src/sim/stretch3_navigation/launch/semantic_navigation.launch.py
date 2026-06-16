"""Action-aware semantic navigation for Stretch3 in Isaac Sim (arXiv:2310.08873).

Brings up the Nav2 stack with the SemanticTraversabilityLayer plus the
perception/projection pipeline that feeds it:

  Nav2 (nav2_params.yaml, semantic layer enabled)
  + projection_node  (detection + /depth + /camera_info -> /semantic_regions)
  + perception       (one of):
      perception:=dino    -> Grounding DINO node  (needs model installed)
      perception:=static  -> static_region_publisher (milestone-1 testing)
      perception:=none    -> nothing (publish /semantic_regions yourself)

Prerequisites (provided by Isaac Sim): world->odom->base_link TF, /laser_scan,
/odom, /rgb, /depth, /camera_info.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Enable use_sim_time for Isaac Sim",
    ),
    DeclareLaunchArgument(
        "perception", default_value="dino",
        choices=["dino", "static", "none"],
        description="Semantic region source: Grounding DINO, static test "
                    "publisher, or none.",
    ),
    DeclareLaunchArgument(
        "target_frame", default_value="odom",
        description="Frame the projected ground polygons are published in.",
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
        "rgb_topic", default_value="/rgb",
        description="RGB image topic for the VLM.",
    ),
    DeclareLaunchArgument(
        "model_config", default_value="",
        description="Grounding DINO config .py path (perception:=dino).",
    ),
    DeclareLaunchArgument(
        "model_weights", default_value="",
        description="Grounding DINO weights .pth path (perception:=dino).",
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("stretch3_navigation"), "config", "nav2_params.yaml"]
    )
    targets_file = PathJoinSubstitution(
        [FindPackageShare("semantic_perception"), "config", "semantic_targets.yaml"]
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
        }],
        remappings=[("~/detection", "/semantic_detection")],
    )

    dino = Node(
        package="semantic_perception",
        executable="grounding_dino_node",
        name="grounding_dino_node",
        output="screen",
        condition=LaunchConfigurationEquals("perception", "dino"),
        parameters=[{
            "use_sim_time": use_sim_time,
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "detection_topic": "/semantic_detection",
            "targets_file": targets_file,
            "model_config": LaunchConfiguration("model_config"),
            "model_weights": LaunchConfiguration("model_weights"),
        }],
    )

    static_pub = Node(
        package="semantic_perception",
        executable="static_region_publisher",
        name="static_region_publisher",
        output="screen",
        condition=LaunchConfigurationEquals("perception", "static"),
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "world"}],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2)
    ld.add_action(projection)
    ld.add_action(dino)
    ld.add_action(static_pub)
    return ld
