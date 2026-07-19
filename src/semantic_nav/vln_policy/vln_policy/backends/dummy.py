"""Scripted backend: replays a fixed action sequence.

Used to exercise the executors and the demo plumbing with no GPU, no server
and no camera (e.g. `backend:=dummy dummy_actions:=FORWARD,TURN_LEFT,STOP`).
"""

from typing import Optional

from .base import (
    STOP,
    OdomPose,
    StepResult,
    VLNBackend,
    validate_actions,
)

DEFAULT_SCRIPT = (
    "FORWARD,FORWARD,TURN_LEFT,FORWARD,FORWARD,TURN_RIGHT,FORWARD,STOP"
)


class DummyBackend(VLNBackend):
    name = "dummy"
    requires_rgb = False

    def __init__(self, actions_csv: str = DEFAULT_SCRIPT, chunk_size: int = 4):
        script = [t for t in actions_csv.split(",") if t.strip()]
        self._script = validate_actions(script)
        if STOP not in self._script:
            self._script.append(STOP)
        self._chunk_size = max(1, int(chunk_size))
        self._cursor = 0
        self.instruction = ""

    def reset(self, instruction: str) -> None:
        self.instruction = instruction
        self._cursor = 0

    def step(self, rgb, odom: Optional[OdomPose]) -> StepResult:
        if self._cursor >= len(self._script):
            return StepResult(actions=[STOP], done=True, detail="script done")
        chunk = self._script[self._cursor:self._cursor + self._chunk_size]
        # never split past a STOP: it terminates the episode
        if STOP in chunk:
            chunk = chunk[:chunk.index(STOP) + 1]
        self._cursor += len(chunk)
        done = chunk[-1] == STOP
        return StepResult(
            actions=chunk,
            done=done,
            detail=f"scripted {self._cursor}/{len(self._script)}",
        )
