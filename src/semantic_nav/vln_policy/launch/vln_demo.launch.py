"""Standalone VLN demo for the Stretch robot in Isaac Sim.

  ros2 launch vln_policy vln_demo.launch.py \
      backend:=streamvln|dummy|navila execution_mode:=cmd_vel|nav2

Always launches vln_agent_node. When execution_mode:=nav2 it additionally
brings up nav2 with stretch3_navigation's params (goals are sent in the odom
frame, so no map server / AMCL is needed). No BT engine involved.

Prerequisites (provided by Isaac Sim playing a scene from isaacsim/assets/):
/rgb, /odom, /tf (world->odom->base_link), cmd_vel subscriber; plus
/laser_scan for the nav2 mode costmaps. For backend:=streamvln the inference
server from docker/vln must be up (see vln_policy/README.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
                    "bursts, or one relative waypoint per batch through "
                    "nav2 navigate_to_pose.",
    ),
    DeclareLaunchArgument(
        "server_url", default_value="http://localhost:18080",
        description="VLN inference server URL (streamvln/navila backends).",
    ),
    DeclareLaunchArgument(
        "rgb_topic", default_value="/rgb",
        description="RGB image topic streamed to the VLN model.",
    ),
    DeclareLaunchArgument(
        "odom_topic", default_value="/odom",
        description="Odometry topic for the executors.",
    ),
    DeclareLaunchArgument(
        "cmd_vel_topic", default_value="/cmd_vel",
        description="Velocity command topic (cmd_vel mode).",
    ),
    DeclareLaunchArgument(
        "dummy_actions", default_value="",
        description="CSV action script for backend:=dummy "
                    "(e.g. 'FORWARD,TURN_LEFT,FORWARD,STOP').",
    ),
    DeclareLaunchArgument(
        "params_file", default_value="",
        description="Optional YAML overriding "
                    "vln_policy/config/vln_agent_params.yaml.",
    ),
    DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Enable use_sim_time for Isaac Sim.",
    ),
]


def _setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(
            get_package_share_directory("vln_policy"),
            "config",
            "vln_agent_params.yaml",
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
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "dummy_actions": LaunchConfiguration("dummy_actions"),
            },
        ],
        remappings=[("~/instruction", "/vln_instruction")],
    )
    actions = [agent]

    if LaunchConfiguration("execution_mode").perform(context) == "nav2":
        # resolved lazily so cmd_vel mode works without the sim nav stack
        from launch.actions import IncludeLaunchDescription

        nav2_params = os.path.join(
            get_package_share_directory("stretch3_navigation"),
            "config",
            "nav2_params.yaml",
        )
        actions.append(
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_setup))
    return ld
