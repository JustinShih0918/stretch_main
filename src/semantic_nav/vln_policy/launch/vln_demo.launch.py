"""Standalone VLN demo for the Stretch robot in Isaac Sim.

  ros2 launch vln_policy vln_demo.launch.py \
      backend:=streamvln|dualvln|dummy|navila \
      execution_mode:=trajectory|cmd_vel|nav2

Always launches vln_agent_node. When execution_mode:=nav2 it additionally
brings up nav2 with stretch3_navigation's params (goals are sent in the odom
frame, so no map server / AMCL is needed). No BT engine involved.

Prerequisites (provided by Isaac Sim playing a scene from isaacsim/assets/):
/rgb, /odom, /tf (world->odom->base_link), cmd_vel subscriber; plus
/laser_scan for nav2 mode costmaps. Remote StreamVLN or DualVLN servers must
be up for their corresponding backend (see vln_policy/README.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from vln_policy.server_url import resolve_server_url

ARGUMENTS = [
    DeclareLaunchArgument(
        "backend", default_value="streamvln",
        choices=["streamvln", "dualvln", "dummy", "navila"],
        description="VLN policy backend.",
    ),
    DeclareLaunchArgument(
        "execution_mode", default_value="trajectory",
        choices=["trajectory", "cmd_vel", "nav2"],
        description="Shared trajectory follower (benchmark), direct velocity "
                    "bursts, or nav2 relative waypoints.",
    ),
    DeclareLaunchArgument(
        "server_url", default_value="",
        description="Explicit inference URL override. Empty selects the "
                    "backend-specific URL.",
    ),
    DeclareLaunchArgument(
        "streamvln_server_url",
        default_value=EnvironmentVariable(
            "STREAMVLN_SERVER_URL",
            default_value="http://140.114.89.63:18080",
        ),
        description="StreamVLN URL when server_url is empty.",
    ),
    DeclareLaunchArgument(
        "dualvln_server_url",
        default_value=EnvironmentVariable(
            "DUALVLN_SERVER_URL", default_value="http://localhost:18082"
        ),
        description="DualVLN URL when server_url is empty.",
    ),
    DeclareLaunchArgument(
        "rgb_topic", default_value="/rgb",
        description="RGB image topic streamed to the VLN model.",
    ),
    DeclareLaunchArgument(
        "depth_topic", default_value="/depth",
        description="Depth image paired with RGB for DualVLN.",
    ),
    DeclareLaunchArgument(
        "camera_info_topic", default_value="/camera_info",
        description="Calibration for the paired RGB/depth stream.",
    ),
    DeclareLaunchArgument(
        "sync_slop_s", default_value="0.08",
        description="Maximum RGB/depth synchronization offset.",
    ),
    DeclareLaunchArgument(
        "rgb_rotation", default_value="clockwise_90",
        choices=["none", "clockwise_90", "counterclockwise_90", "180"],
        description="Right-angle correction applied identically to model "
                    "input and the RViz camera HUD.",
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
        "v_lin", default_value="0.25",
        description="Maximum shared linear velocity in m/s.",
    ),
    DeclareLaunchArgument(
        "v_ang", default_value="0.5",
        description="Maximum shared angular velocity in rad/s.",
    ),
    DeclareLaunchArgument(
        "trajectory_lookahead_m", default_value="0.35",
        description="Trajectory follower lookahead in metres.",
    ),
    DeclareLaunchArgument(
        "trajectory_final_tolerance_m", default_value="0.12",
        description="Final path-position tolerance in metres.",
    ),
    DeclareLaunchArgument(
        "trajectory_turn_tolerance_deg", default_value="5.0",
        description="Tolerance for action-derived explicit turns.",
    ),
    DeclareLaunchArgument(
        "trajectory_watchdog_s", default_value="6.0",
        description="No-progress watchdog timeout.",
    ),
    DeclareLaunchArgument(
        "trajectory_linear_accel_mps2", default_value="0.5",
        description="Linear acceleration limit.",
    ),
    DeclareLaunchArgument(
        "trajectory_angular_accel_rps2", default_value="1.0",
        description="Angular acceleration limit.",
    ),
    DeclareLaunchArgument(
        "trajectory_odom_timeout_s", default_value="1.0",
        description="Maximum trajectory-mode odometry silence.",
    ),
    DeclareLaunchArgument(
        "tick_rate_hz", default_value="20.0",
        description="Agent and trajectory-controller tick rate.",
    ),
    DeclareLaunchArgument(
        "dualvln_replan_period_s", default_value="0.3",
        description="Minimum interval between DualVLN requests.",
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
        "viz", default_value="true",
        description="Run vln_viz_node (/vln/viz_image HUD, /vln/viz_markers, "
                    "/vln/path for RViz).",
    ),
    DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Also open RViz with vln_policy/config/vln_demo.rviz.",
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

    backend = LaunchConfiguration("backend").perform(context)
    server_url = resolve_server_url(
        backend,
        LaunchConfiguration("server_url").perform(context),
        LaunchConfiguration("streamvln_server_url").perform(context),
        LaunchConfiguration("dualvln_server_url").perform(context),
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
                "server_url": server_url,
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "sync_slop_s": LaunchConfiguration("sync_slop_s"),
                "rgb_rotation": LaunchConfiguration("rgb_rotation"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "v_lin": LaunchConfiguration("v_lin"),
                "v_ang": LaunchConfiguration("v_ang"),
                "trajectory_lookahead_m": LaunchConfiguration(
                    "trajectory_lookahead_m"
                ),
                "trajectory_final_tolerance_m": LaunchConfiguration(
                    "trajectory_final_tolerance_m"
                ),
                "trajectory_turn_tolerance_deg": LaunchConfiguration(
                    "trajectory_turn_tolerance_deg"
                ),
                "trajectory_watchdog_s": LaunchConfiguration(
                    "trajectory_watchdog_s"
                ),
                "trajectory_linear_accel_mps2": LaunchConfiguration(
                    "trajectory_linear_accel_mps2"
                ),
                "trajectory_angular_accel_rps2": LaunchConfiguration(
                    "trajectory_angular_accel_rps2"
                ),
                "trajectory_odom_timeout_s": LaunchConfiguration(
                    "trajectory_odom_timeout_s"
                ),
                "tick_rate_hz": LaunchConfiguration("tick_rate_hz"),
                "dualvln_replan_period_s": LaunchConfiguration(
                    "dualvln_replan_period_s"
                ),
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
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "rgb_rotation": LaunchConfiguration("rgb_rotation"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                }],
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
