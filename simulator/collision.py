from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from math import cos, radians, sin
from typing import Dict, Iterable, List, Sequence, Tuple

from .trajectory import Pose, TrajectorySimulator


@dataclass(frozen=True)
class CollisionEvent:
    time: float
    robot_a: str
    robot_b: str


@dataclass
class CollisionReport:
    sample_interval: float
    total_samples: int
    robot_length_mm: float
    robot_width_mm: float
    safety_margin_mm: float
    collisions: List[CollisionEvent]

    def has_collision(self) -> bool:
        return bool(self.collisions)

    def format_summary(self) -> str:
        header = (
            f"[Collision Report] samples={self.total_samples} "
            f"dt={self.sample_interval:.3f}s length={self.robot_length_mm}mm "
            f"width={self.robot_width_mm}mm margin={self.safety_margin_mm}mm"
        )
        if not self.collisions:
            return header + "\nNo collisions detected."

        lines = [header, f"{len(self.collisions)} collision(s) detected:"]
        for event in self.collisions:
            lines.append(
                f"  - t={event.time:.3f}s : {event.robot_a} vs {event.robot_b}"
            )
        return "\n".join(lines)


@dataclass(frozen=True)
class RectShape:
    vertices: Tuple[Tuple[float, float], ...]
    axes: Tuple[Tuple[float, float], ...]


def _create_rect_shape(
    pose: Pose, length_mm: float, width_mm: float
) -> RectShape:
    half_l = length_mm / 2.0
    half_w = width_mm / 2.0
    angle = radians(pose.rotation)
    forward = (sin(angle), -cos(angle))
    right = (cos(angle), sin(angle))

    corners = [
        (-half_l, -half_w),
        (-half_l, half_w),
        (half_l, half_w),
        (half_l, -half_w),
    ]

    vertices = tuple(
        (
            pose.x + forward[0] * along + right[0] * lateral,
            pose.y + forward[1] * along + right[1] * lateral,
        )
        for along, lateral in corners
    )

    return RectShape(vertices=vertices, axes=(forward, right))


def _dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _project(axis: Tuple[float, float], vertices: Tuple[Tuple[float, float], ...]):
    projections = [_dot(axis, v) for v in vertices]
    return min(projections), max(projections)


def rectangles_intersect(rect_a: RectShape, rect_b: RectShape) -> bool:
    axes = rect_a.axes + rect_b.axes
    for axis in axes:
        min_a, max_a = _project(axis, rect_a.vertices)
        min_b, max_b = _project(axis, rect_b.vertices)
        if max_a < min_b or max_b < min_a:
            return False
    return True


class CollisionChecker:
    def __init__(
        self,
        simulator: TrajectorySimulator,
        *,
        robot_length_mm: float = 72.0,
        robot_width_mm: float = 32.0,
        safety_margin_mm: float = 20.0,
        sample_interval: float = 0.05,
    ):
        if sample_interval <= 0:
            raise ValueError("sample_interval must be > 0")

        self.simulator = simulator
        self.robot_length_mm = robot_length_mm + 2 * safety_margin_mm
        self.robot_width_mm = robot_width_mm + 2 * safety_margin_mm
        self.safety_margin_mm = safety_margin_mm
        self.sample_interval = sample_interval

    def _time_grid(self, duration: float | None) -> List[float]:
        start = self.simulator.start_time
        end = (
            start + duration
            if duration is not None
            else self.simulator.end_time
        )

        times: List[float] = []
        t = start
        while t <= end + 1e-6:
            times.append(round(t, 6))
            t += self.sample_interval
        if times[-1] < end:
            times.append(round(end, 6))

        return times

    def run(self, duration: float | None = None) -> CollisionReport:
        times = self._time_grid(duration)
        pose_cache = self.simulator.sample_poses(times)
        robot_ids = self.simulator.robot_ids()

        rects_per_time: List[Dict[str, RectShape]] = []
        for idx, time_point in enumerate(times):
            snapshot = {
                robot_id: _create_rect_shape(
                    pose_cache[robot_id][idx],
                    self.robot_length_mm,
                    self.robot_width_mm,
                )
                for robot_id in robot_ids
            }
            rects_per_time.append(snapshot)

        first_collisions: Dict[Tuple[str, str], CollisionEvent] = {}

        for time_point, snapshot in zip(times, rects_per_time):
            for robot_a, robot_b in combinations(robot_ids, 2):
                pair = tuple(sorted((robot_a, robot_b)))
                if pair in first_collisions:
                    continue

                rect_a = snapshot[robot_a]
                rect_b = snapshot[robot_b]

                if rectangles_intersect(rect_a, rect_b):
                    first_collisions[pair] = CollisionEvent(
                        time=time_point,
                        robot_a=robot_a,
                        robot_b=robot_b,
                    )

        collisions = sorted(first_collisions.values(), key=lambda ev: ev.time)
        return CollisionReport(
            sample_interval=self.sample_interval,
            total_samples=len(times),
            robot_length_mm=self.robot_length_mm,
            robot_width_mm=self.robot_width_mm,
            safety_margin_mm=self.safety_margin_mm,
            collisions=collisions,
        )

