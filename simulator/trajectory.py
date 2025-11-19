from __future__ import annotations

from dataclasses import dataclass
from bisect import bisect_right
from typing import Dict, Iterable, List, Sequence

from .data_types import Frame, RobotTimeline, validate_timelines


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    rotation: float


def _lerp(a: float, b: float, ratio: float) -> float:
    return a + (b - a) * ratio


def _lerp_angle(a: float, b: float, ratio: float) -> float:
    # shortest angular distance interpolation
    diff = (b - a + 180.0) % 360.0 - 180.0
    return (a + diff * ratio + 360.0) % 360.0


class TrajectorySimulator:
    """Provides pose sampling for each robot timeline."""

    def __init__(self, timelines: Sequence[RobotTimeline]):
        self.timelines = {tl.robot_id: tl for tl in validate_timelines(timelines)}
        self._time_arrays: Dict[str, List[float]] = {
            tl.robot_id: [frame.time for frame in tl.frames] for tl in timelines
        }

        self.start_time = min(tl.start_time for tl in timelines)
        self.end_time = max(tl.end_time for tl in timelines)

    def robot_ids(self) -> List[str]:
        return list(self.timelines.keys())

    def sample(self, robot_id: str, time_point: float) -> Pose:
        timeline = self.timelines[robot_id]
        times = self._time_arrays[robot_id]
        frames = timeline.frames

        if time_point <= times[0]:
            anchor = frames[0]
            return Pose(anchor.x, anchor.y, anchor.rotation)

        if time_point >= times[-1]:
            anchor = frames[-1]
            return Pose(anchor.x, anchor.y, anchor.rotation)

        idx = bisect_right(times, time_point)
        prev_frame = frames[idx - 1]
        next_frame = frames[idx]

        span = max(next_frame.time - prev_frame.time, 1e-6)
        ratio = (time_point - prev_frame.time) / span

        x = _lerp(prev_frame.x, next_frame.x, ratio)
        y = _lerp(prev_frame.y, next_frame.y, ratio)
        rotation = _lerp_angle(prev_frame.rotation, next_frame.rotation, ratio)

        return Pose(x=x, y=y, rotation=rotation)

    def sample_robot_times(
        self, robot_id: str, time_points: Sequence[float]
    ) -> List[Pose]:
        """Sample a single robot over multiple time points."""

        return [self.sample(robot_id, t) for t in time_points]

    def sample_poses(self, time_points: Sequence[float]) -> Dict[str, List[Pose]]:
        """Sample all robots for each requested timestamp."""

        return {
            robot_id: self.sample_robot_times(robot_id, time_points)
            for robot_id in self.robot_ids()
        }

