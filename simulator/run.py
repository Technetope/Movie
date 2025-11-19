from __future__ import annotations

import argparse
from pathlib import Path

from .collision import CollisionChecker
from .load_paths import load_robot_timelines
from .render_animation import render_animation
from .trajectory import TrajectorySimulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline simulator for robot JSON trajectories."
    )
    parser.add_argument(
        "--mode",
        choices=["render", "collision", "both"],
        default="render",
        help="Choose whether to render, run collision scan, or both.",
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="Path to quadrant_output/all_robots_merged.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("simulator_output.mp4"),
        help="Destination video file (mp4/gif supported).",
    )
    parser.add_argument("--fps", type=int, default=30, help="Animation frames per second.")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional duration override in seconds.",
    )
    parser.add_argument(
        "--trail",
        type=float,
        default=5.0,
        help="How many seconds of trajectory trail to show.",
    )
    parser.add_argument(
        "--robot-length-mm",
        type=float,
        default=72.0,
        help="Physical robot length (front-back) in millimeters.",
    )
    parser.add_argument(
        "--robot-width-mm",
        type=float,
        default=32.0,
        help="Physical robot width in millimeters.",
    )
    parser.add_argument(
        "--safety-margin-mm",
        type=float,
        default=20.0,
        help="Additional safety margin added to length/width (each dimension).",
    )
    parser.add_argument(
        "--collision-step",
        type=float,
        default=0.05,
        help="Sampling interval (seconds) for collision detection.",
    )
    parser.add_argument(
        "--field-bounds",
        type=float,
        nargs=4,
        metavar=("MIN_X", "MAX_X", "MIN_Y", "MAX_Y"),
        default=(34.0, 949.0, 35.0, 898.0),
        help="Field rectangle used for rendering (mm).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    timelines = load_robot_timelines(args.input)
    simulator = TrajectorySimulator(timelines)

    field_bounds = tuple(args.field_bounds)

    if args.mode in {"render", "both"}:
        render_animation(
            simulator,
            args.output,
            fps=args.fps,
            duration=args.duration,
            trail_window=args.trail,
            field_bounds=field_bounds,
            robot_length_mm=args.robot_length_mm,
            robot_width_mm=args.robot_width_mm,
            safety_margin_mm=args.safety_margin_mm,
        )

    if args.mode in {"collision", "both"}:
        checker = CollisionChecker(
            simulator,
            robot_length_mm=args.robot_length_mm,
            robot_width_mm=args.robot_width_mm,
            safety_margin_mm=args.safety_margin_mm,
            sample_interval=args.collision_step,
        )
        report = checker.run(duration=args.duration)
        print(report.format_summary())


if __name__ == "__main__":
    main()

