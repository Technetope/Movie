import unittest

from simulator.data_types import Frame, RobotTimeline
from simulator.trajectory import TrajectorySimulator


def make_timeline():
    frames = [
        Frame(time=0, x=0, y=0, rotation=0, use_position=True, use_rotation=True),
        Frame(time=5, x=10, y=0, rotation=90, use_position=True, use_rotation=True),
    ]
    return RobotTimeline(robot_id="robot_00", frames=frames)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.sim = TrajectorySimulator([make_timeline()])

    def test_linear_interpolation_midpoint(self):
        pose = self.sim.sample("robot_00", 2.5)
        self.assertEqual(pose.x, 5)
        self.assertEqual(pose.y, 0)
        self.assertEqual(pose.rotation, 45)

    def test_clamp_before_start(self):
        pose = self.sim.sample("robot_00", -1)
        self.assertEqual(pose.x, 0)
        self.assertEqual(pose.rotation, 0)

    def test_clamp_after_end(self):
        pose = self.sim.sample("robot_00", 10)
        self.assertEqual(pose.x, 10)
        self.assertEqual(pose.rotation, 90)


if __name__ == "__main__":
    unittest.main()

