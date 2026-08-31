"""Backend abstraction and protocol-v2 command types for VLN policies."""

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Union

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
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def as_dict(self) -> dict:
        return {
            "fx": float(self.fx), "fy": float(self.fy),
            "cx": float(self.cx), "cy": float(self.cy),
            "width": int(self.width), "height": int(self.height),
        }


@dataclass(frozen=True)
class TrajectoryPoint:
    """A robot-relative point: x forward, y left, in metres."""

    x: float
    y: float


@dataclass(frozen=True)
class DiscreteCommand:
    actions: tuple


@dataclass(frozen=True)
class TrajectoryCommand:
    points: tuple


@dataclass(frozen=True)
class StopCommand:
    pass


Command = Union[DiscreteCommand, TrajectoryCommand, StopCommand]


@dataclass
class StepTimings:
    """All fields are milliseconds; missing server fields remain ``None``."""

    client_ms: Optional[float] = None
    total_ms: Optional[float] = None
    preprocessing_ms: Optional[float] = None
    system1_ms: Optional[float] = None
    system2_ms: Optional[float] = None

    @property
    def server_compute_ms(self) -> Optional[float]:
        if self.total_ms is not None:
            return self.total_ms
        parts = [self.preprocessing_ms, self.system1_ms, self.system2_ms]
        present = [value for value in parts if value is not None]
        return sum(present) if present else None


@dataclass
class StepResult:
    actions: list = field(default_factory=list)
    trajectory: list = field(default_factory=list)
    done: bool = False
    detail: str = ""
    timings: StepTimings = field(default_factory=StepTimings)
    image_timestamp_s: Optional[float] = None

    @property
    def command(self) -> Command:
        if self.done and not self.actions and not self.trajectory:
            return StopCommand()
        if self.trajectory:
            return TrajectoryCommand(tuple(self.trajectory))
        if STOP in self.actions:
            return StopCommand()
        return DiscreteCommand(tuple(self.actions))

    @property
    def output_type(self) -> str:
        command = self.command
        if isinstance(command, TrajectoryCommand):
            return "trajectory"
        if isinstance(command, StopCommand):
            return "stop"
        return "actions"


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
    requires_depth = False

    @abstractmethod
    def reset(self, instruction: str) -> None:
        ...

    @abstractmethod
    def step(
        self,
        rgb,
        odom: Optional[OdomPose],
        *,
        depth=None,
        depth_scale_m: Optional[float] = None,
        intrinsics: Optional[CameraIntrinsics] = None,
        image_timestamp_s: Optional[float] = None,
    ) -> StepResult:
        """rgb: HxWx3 uint8 RGB numpy array (None only if requires_rgb is
        False). Depth is an optional synchronized uint16 image whose units are
        described by ``depth_scale_m``. odom is the latest planar pose."""
        ...

    def close(self) -> None:
        pass
