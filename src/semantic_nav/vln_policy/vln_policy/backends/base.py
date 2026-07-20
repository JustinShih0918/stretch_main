"""Backend abstraction for VLN policies.

A backend turns (instruction, latest RGB frame) into short batches of
discrete VLN-CE actions. The HTTP contract every self-hosted backend server
must implement is normative in vln_policy/DESIGN.md.
"""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

# Discrete VLN-CE action vocabulary and its geometry. Executors translate
# these into motion, so the constants live here, shared by both sides.
STOP = "STOP"
FORWARD = "FORWARD"
BACKWARD = "BACKWARD"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"
# StreamVLN's model/wire vocabulary.  BACKWARD is deliberately local-only:
# the robot-relative command interpreter can request it without pretending
# that the pretrained policy knows how to emit a fifth action token.
ACTIONS = (STOP, FORWARD, TURN_LEFT, TURN_RIGHT)
MOTION_ACTIONS = (FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT)

FORWARD_M = 0.25
TURN_RAD = math.radians(15.0)


class BackendError(RuntimeError):
    """Raised when a backend cannot reset or step (server down, bad reply)."""


@dataclass
class OdomPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class StepResult:
    actions: list = field(default_factory=list)
    done: bool = False
    detail: str = ""


def validate_actions(actions) -> list:
    """Return actions as a list of canonical tokens, or raise BackendError."""
    tokens = [str(a).strip().upper() for a in actions]
    for token in tokens:
        if token not in ACTIONS:
            raise BackendError(
                f"backend returned unknown action '{token}' "
                f"(expected one of {ACTIONS})"
            )
    return tokens


class VLNBackend(ABC):
    """One VLN policy episode source.

    Lifecycle: reset(instruction) starts an episode; step(...) is called with
    the latest RGB frame until it returns done=True (or the agent hits its
    step cap). Both may raise BackendError; the agent node degrades to an
    ERROR state and recovers on the next instruction.
    """

    name = "base"
    #: False for backends that ignore camera input (dummy) so the agent can
    #: run without a frame in hand.
    requires_rgb = True

    @abstractmethod
    def reset(self, instruction: str) -> None:
        ...

    @abstractmethod
    def step(self, rgb, odom: Optional[OdomPose]) -> StepResult:
        """rgb: HxWx3 uint8 RGB numpy array (None only if requires_rgb is
        False). odom: latest planar pose, may be None."""
        ...

    def close(self) -> None:
        pass
