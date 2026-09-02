"""Pure geometry helpers for VLN visualization (no ROS imports)."""

import math

from .backends.base import (
    BACKWARD,
    FORWARD,
    FORWARD_M,
    TURN_LEFT,
    TURN_RAD,
    TURN_RIGHT,
    OdomPose,
)


def action_trajectory(
    pose: OdomPose,
    actions,
    forward_m: float = FORWARD_M,
    turn_rad: float = TURN_RAD,
):
    """Points (x, y) visited while executing `actions` from `pose`, plus the
    final heading. Turns rotate in place; FORWARD adds a point.

    Pass the same forward_m/turn_rad the executor runs with, or the previewed
    path will not be where the robot actually goes.
    """
    x, y, yaw = pose.x, pose.y, pose.yaw
    points = [(x, y)]
    for action in actions:
        if action == FORWARD:
            x += forward_m * math.cos(yaw)
            y += forward_m * math.sin(yaw)
            points.append((x, y))
        elif action == BACKWARD:
            x -= forward_m * math.cos(yaw)
            y -= forward_m * math.sin(yaw)
            points.append((x, y))
        elif action == TURN_LEFT:
            yaw += turn_rad
        elif action == TURN_RIGHT:
            yaw -= turn_rad
    return points, yaw


def split_actions(status):
    """Current + pending motion actions of a VlnStatus-like object as a flat
    list (nav2 mode joins the batch with '+' in current_action)."""
    actions = []
    if status.current_action:
        actions.extend(a for a in status.current_action.split("+") if a)
    actions.extend(status.pending_actions)
    return [
        a for a in actions
        if a in (FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT)
    ]


def records_episode_path(state):
    """Return whether odometry belongs to an active policy episode."""
    return state in ("RESETTING", "THINKING", "EXECUTING")
