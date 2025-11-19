from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence


@dataclass(frozen=True)
class Frame:
    """A single robot directive sampled at an absolute timestamp."""

    time: float
    x: float
    y: float
    rotation: float
    use_position: bool
    use_rotation: bool


@dataclass(frozen=True)
class RobotTimeline:
    """Chronologically ordered set of frames for a robot."""

    robot_id: str
    frames: Sequence[Frame]

    def __post_init__(self):
        if not self.frames:
            raise ValueError(f"Robot '{self.robot_id}' has no frames")

        times = [frame.time for frame in self.frames]
        if times != sorted(times):
            raise ValueError(f"Robot '{self.robot_id}' frames are not sorted by time")

    @property
    def start_time(self) -> float:
        return self.frames[0].time

    @property
    def end_time(self) -> float:
        return self.frames[-1].time

    def __len__(self) -> int:
        return len(self.frames)


def validate_timelines(timelines: Sequence[RobotTimeline]) -> List[RobotTimeline]:
    """Return timelines sorted by robot_id for deterministic processing."""

    if not timelines:
        raise ValueError("No timelines were supplied")

    return sorted(timelines, key=lambda tl: tl.robot_id)

