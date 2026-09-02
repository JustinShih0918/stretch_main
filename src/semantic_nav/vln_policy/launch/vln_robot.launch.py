"""VLN pipeline for the *real* Stretch robot.

  ros2 launch vln_policy vln_robot.launch.py \
      backend:=streamvln|dummy|navila execution_mode:=cmd_vel|nav2

Difference from vln_demo.launch.py (Isaac Sim): this launch brings up
**nothing but the VLN nodes**. On the robot the camera driver and — for
execution_mode:=nav2 — nav2 itself are already running (stretch_core /
realsense2_camera / stretch_nav2), so this only attaches to their topics.
use_sim_time is False.

Wiring: the robot's topic, action and frame names are the ROBOT_* constants
right below. Edit them once here; they become the defaults of the matching
launch arguments and are handed to both vln_agent_node and vln_viz_node.
Everything else (speeds, timeouts, episode cap) lives in
config/vln_robot_params.yaml.

Check the names against the running robot before the first run:

  ros2 topic list | grep -Ei 'image|odom|cmd_vel'
  ros2 topic info -v <topic>        # type + QoS
  ros2 action list | grep navigate
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ─────────────────────────────────────────────────────────────────────────
#  EDIT ME — the names the robot actually publishes/serves.
#  (Defaults below are the stock hello-robot ones; replace with yours.)
# ─────────────────────────────────────────────────────────────────────────
ROBOT_RGB_TOPIC = "/camera/color/image_raw"    # sensor_msgs/Image, BEST_EFFORT
# "compressed" subscribes to <ROBOT_RGB_TOPIC>/compressed instead. Measured on
# this robot at 1280x720/15 Hz: 2700 KiB per raw frame (333 Mbit/s) vs 261 KiB
# (32 Mbit/s) for the JPEG the same driver already publishes. Use "raw" only
# for a camera that publishes no compressed topic.
ROBOT_RGB_TRANSPORT = "compressed"
ROBOT_ODOM_TOPIC = "/odom"                     # nav_msgs/Odometry
ROBOT_CMD_VEL_TOPIC = "/stretch/cmd_vel"       # geometry_msgs/Twist
ROBOT_NAV2_ACTION = "navigate_to_pose"         # NavigateToPose action server
# How a nav2-mode waypoint is handed to nav2:
#   "topic"  — publish PoseStamped on ROBOT_NAV2_GOAL_TOPIC, exactly what
#              RViz's goal tool does. Works even when the robot's RMW differs
#              from ours (topics interoperate across DDS vendors, services and
#              therefore actions do not). No result comes back, so a batch is
#              considered finished on odometry arrival and a nav2 abort shows
#              up as the goal timeout.
#   "action" — send a real navigate_to_pose goal: exact success/abort/cancel.
#              Prefer this once both ends run the same RMW_IMPLEMENTATION.
ROBOT_NAV2_GOAL_INTERFACE = "topic"
ROBOT_NAV2_GOAL_TOPIC = "/goal_pose"
# odom keeps the waypoints purely relative and immune to AMCL jumps; use
# "map" to get localization-corrected goals instead (what docker/vlm's
# projection node does with its polygons).
ROBOT_GOAL_FRAME = "odom"                      # frame of nav2-mode waypoints
ROBOT_MARKER_FRAME = "odom"                    # RViz fixed frame for /vln/*
ROBOT_RGB_ROTATION = "clockwise_90"            # upright camera -> no rotation
# ─────────────────────────────────────────────────────────────────────────

ARGUMENTS = [
    DeclareLaunchArgument(
        "backend", default_value="streamvln",
        choices=["streamvln", "dummy", "navila"],
        description="VLN policy backend.",
    ),
    DeclareLaunchArgument(
        "execution_mode", default_value="cmd_vel",
        choices=["cmd_vel", "nav2"],
        description="How discrete actions are executed: direct velocity "
                    "bursts, or one relative waypoint per batch through the "
                    "nav2 action server already running on the robot.",
    ),
    DeclareLaunchArgument(
        "server_url", default_value="http://localhost:18080",
        description="VLN inference server URL (streamvln/navila backends).",
    ),
    DeclareLaunchArgument(
        "rgb_topic", default_value=ROBOT_RGB_TOPIC,
        description="RGB image topic streamed to the VLN model.",
    ),
    DeclareLaunchArgument(
        "rgb_transport", default_value=ROBOT_RGB_TRANSPORT,
        choices=["compressed", "raw"],
        description="Subscribe to <rgb_topic>/compressed (10x less wire) or "
                    "the raw stream.",
    ),
    DeclareLaunchArgument(
        "rgb_rotation", default_value=ROBOT_RGB_ROTATION,
        choices=["none", "clockwise_90", "counterclockwise_90", "180"],
        description="Right-angle correction applied identically to model "
                    "input and the RViz camera HUD.",
    ),
    DeclareLaunchArgument(
        "odom_topic", default_value=ROBOT_ODOM_TOPIC,
        description="Odometry topic for the executors.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_topic", default_value=ROBOT_CMD_VEL_TOPIC,
        description="Velocity command topic (cmd_vel mode).",
    ),
    DeclareLaunchArgument(
        "nav2_action_name", default_value=ROBOT_NAV2_ACTION,
        description="navigate_to_pose action server on the robot "
                    "(nav2 mode).",
    ),
    DeclareLaunchArgument(
        "nav2_goal_interface", default_value=ROBOT_NAV2_GOAL_INTERFACE,
        choices=["topic", "action"],
        description="Hand nav2 waypoints over as /goal_pose messages "
                    "(cross-RMW safe, no result) or as action goals.",
    ),
    DeclareLaunchArgument(
        "nav2_goal_topic", default_value=ROBOT_NAV2_GOAL_TOPIC,
        description="Goal topic for nav2_goal_interface:=topic.",
    ),
    DeclareLaunchArgument(
        "goal_frame", default_value=ROBOT_GOAL_FRAME,
        description="Frame the nav2-mode waypoints are expressed in.",
    ),
    DeclareLaunchArgument(
        "marker_frame", default_value=ROBOT_MARKER_FRAME,
        description="Frame of /vln/viz_markers and /vln/path.",
    ),
    DeclareLaunchArgument(
        "dummy_actions", default_value="",
        description="CSV action script for backend:=dummy "
                    "(e.g. 'FORWARD,TURN_LEFT,FORWARD,STOP').",
    ),
    DeclareLaunchArgument(
        "params_file", default_value="",
        description="Optional YAML overriding "
                    "vln_policy/config/vln_robot_params.yaml.",
    ),
    DeclareLaunchArgument(
        "viz", default_value="true",
        description="Run vln_viz_node (/vln/viz_image HUD, /vln/viz_markers, "
                    "/vln/path for RViz).",
    ),
    DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Also open RViz with vln_policy/config/vln_demo.rviz.",
    ),
    DeclareLaunchArgument(
        "use_sim_time", default_value="False",
        description="Real hardware: keep False (wall clock).",
    ),
]


def _setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory("vln_policy"),
            "config",
            "vln_robot_params.yaml",
        )

    agent = Node(
        package="vln_policy",
        executable="vln_agent_node",
        name="vln_agent_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "backend": LaunchConfiguration("backend"),
                "execution_mode": LaunchConfiguration("execution_mode"),
                "server_url": LaunchConfiguration("server_url"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "rgb_transport": LaunchConfiguration("rgb_transport"),
                "rgb_rotation": LaunchConfiguration("rgb_rotation"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "nav2_action_name": LaunchConfiguration("nav2_action_name"),
                "nav2_goal_interface": LaunchConfiguration(
                    "nav2_goal_interface"
                ),
                "nav2_goal_topic": LaunchConfiguration("nav2_goal_topic"),
                "goal_frame": LaunchConfiguration("goal_frame"),
                "dummy_actions": LaunchConfiguration("dummy_actions"),
            },
        ],
        remappings=[("~/instruction", "/vln_instruction")],
    )
    actions = [agent]

    if LaunchConfiguration("viz").perform(context).lower() == "true":
        actions.append(
            Node(
                package="vln_policy",
                executable="vln_viz_node",
                name="vln_viz_node",
                output="screen",
                # The params file is passed here too, for its `vln_viz_node`
                # section: the step geometry (forward_step_m/turn_step_deg)
                # must match the agent's or the previewed ribbon lands
                # somewhere the robot never goes.
                parameters=[
                    params_file,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "rgb_transport": LaunchConfiguration("rgb_transport"),
                        "rgb_rotation": LaunchConfiguration("rgb_rotation"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "marker_frame": LaunchConfiguration("marker_frame"),
                    },
                ],
            )
        )

    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        rviz_config = os.path.join(
            get_package_share_directory("vln_policy"),
            "config",
            "vln_demo.rviz",
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
            )
        )

    # Deliberately no nav2 / camera bringup here: on the robot both are
    # already running (stretch_nav2, stretch_core + realsense2_camera), and
    # starting a second nav2 would fight the first one over /cmd_vel.
    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
