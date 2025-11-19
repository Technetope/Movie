import unittest

from simulator.collision import (
    CollisionChecker,
    rectangles_intersect,
    _create_rect_shape,
)
from simulator.data_types import Frame, RobotTimeline
from simulator.trajectory import Pose, TrajectorySimulator


class CollisionGeometryTests(unittest.TestCase):
    def test_rectangles_overlap(self):
        pose_a = Pose(x=0.0, y=0.0, rotation=0.0)
        pose_b = Pose(x=0.0, y=20.0, rotation=0.0)
        rect_a = _create_rect_shape(pose_a, 72.0, 32.0)
        rect_b = _create_rect_shape(pose_b, 72.0, 32.0)
        self.assertTrue(rectangles_intersect(rect_a, rect_b))

    def test_rectangles_separated(self):
        pose_a = Pose(x=0.0, y=0.0, rotation=0.0)
        pose_b = Pose(x=200.0, y=0.0, rotation=0.0)
        rect_a = _create_rect_shape(pose_a, 72.0, 32.0)
        rect_b = _create_rect_shape(pose_b, 72.0, 32.0)
        self.assertFalse(rectangles_intersect(rect_a, rect_b))

    def test_rotated_overlap(self):
        pose_a = Pose(x=0.0, y=0.0, rotation=45.0)
        pose_b = Pose(x=20.0, y=0.0, rotation=120.0)
        rect_a = _create_rect_shape(pose_a, 72.0, 32.0)
        rect_b = _create_rect_shape(pose_b, 72.0, 32.0)
        self.assertTrue(rectangles_intersect(rect_a, rect_b))


class CollisionCheckerTests(unittest.TestCase):
    def _make_simulator(self, offset_x: float):
        frames_a = [
            Frame(0.0, 0.0, 0.0, 0.0, True, True),
            Frame(2.0, 0.0, 0.0, 0.0, True, True),
        ]
        frames_b = [
            Frame(0.0, offset_x, 0.0, 0.0, True, True),
            Frame(2.0, offset_x, 0.0, 0.0, True, True),
        ]
        timeline_a = RobotTimeline("robot_00", frames_a)
        timeline_b = RobotTimeline("robot_01", frames_b)
        return TrajectorySimulator([timeline_a, timeline_b])

    def test_collision_detected_when_rectangles_overlap(self):
        sim = self._make_simulator(offset_x=0.0)
        checker = CollisionChecker(
            sim,
            robot_length_mm=72.0,
            robot_width_mm=32.0,
            safety_margin_mm=0.0,
            sample_interval=0.5,
        )
        report = checker.run(duration=2.0)
        self.assertTrue(report.has_collision())
        self.assertGreaterEqual(len(report.collisions), 1)

    def test_no_collision_when_far_apart(self):
        sim = self._make_simulator(offset_x=200.0)
        checker = CollisionChecker(
            sim,
            robot_length_mm=10.0,
            robot_width_mm=10.0,
            safety_margin_mm=0.0,
            sample_interval=0.5,
        )
        report = checker.run(duration=2.0)
        self.assertFalse(report.has_collision())


if __name__ == "__main__":
    unittest.main()

