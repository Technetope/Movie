from __future__ import annotations

from bisect import bisect_left
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib import animation, patches

from .trajectory import Pose, TrajectorySimulator


def _prepare_pose_cache(
    simulator: TrajectorySimulator, frame_times: Sequence[float]
) -> Dict[str, List[Pose]]:
    return simulator.sample_poses(frame_times)


def render_animation(
    simulator: TrajectorySimulator,
    output_path: Path | str,
    *,
    fps: int = 30,
    duration: float | None = None,
    field_bounds: Tuple[float, float, float, float] = (34.0, 949.0, 35.0, 898.0),
    trail_window: float = 5.0,
    robot_length_mm: float = 72.0,
    robot_width_mm: float = 32.0,
    safety_margin_mm: float = 20.0,
) -> Path:
    """Render an MP4 animation of all robot trajectories."""

    if duration is None:
        duration = simulator.end_time - simulator.start_time
    duration = max(duration, 0.1)

    num_frames = int(duration * fps) + 1
    frame_times = [simulator.start_time + (i / fps) for i in range(num_frames)]

    pose_cache = _prepare_pose_cache(simulator, frame_times)
    robot_ids = simulator.robot_ids()

    length_eff = robot_length_mm + 2 * safety_margin_mm
    width_eff = robot_width_mm + 2 * safety_margin_mm

    fig, ax = plt.subplots(figsize=(8, 7))
    xmin, xmax, ymin, ymax = field_bounds
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Robot Trajectory Simulation")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")

    trails = []
    rectangles = []
    headings = []
    labels = []

    colors = plt.cm.get_cmap("tab20", len(robot_ids))

    for idx, robot_id in enumerate(robot_ids):
        line, = ax.plot([], [], lw=1.2, color=colors(idx), alpha=0.7)
        trails.append(line)

        pose0 = pose_cache[robot_id][0]
        verts = _rectangle_vertices(pose0, length_eff, width_eff)
        poly = patches.Polygon(
            verts,
            closed=True,
            edgecolor=colors(idx),
            facecolor=colors(idx),
            alpha=0.4,
            linewidth=1.5,
        )
        ax.add_patch(poly)
        rectangles.append(poly)

        heading_line, = ax.plot([], [], color="black", lw=1.0)
        headings.append(heading_line)

        label = ax.text(
            pose0.x,
            pose0.y - (length_eff / 2 + 15),
            robot_id,
            fontsize=8,
            ha="center",
        )
        labels.append(label)

    def _update(frame_idx: int):
        time_point = frame_times[frame_idx]
        start_time = time_point - trail_window
        start_idx = max(bisect_left(frame_times, start_time), 0)

        artists = []

        for idx, robot_id in enumerate(robot_ids):
            pose = pose_cache[robot_id][frame_idx]

            verts = _rectangle_vertices(pose, length_eff, width_eff)
            rectangles[idx].set_xy(verts)
            artists.append(rectangles[idx])

            segment = pose_cache[robot_id][start_idx : frame_idx + 1]
            trails[idx].set_data([p.x for p in segment], [p.y for p in segment])
            artists.append(trails[idx])

            heading_len = length_eff * 0.75
            dx = heading_len * _sin_deg(pose.rotation)
            dy = -heading_len * _cos_deg(pose.rotation)
            headings[idx].set_data(
                [pose.x, pose.x + dx],
                [pose.y, pose.y + dy],
            )
            artists.append(headings[idx])

            labels[idx].set_position((pose.x, pose.y - (length_eff / 2 + 15)))
            artists.append(labels[idx])

        ax.set_title(f"Robot Trajectory Simulation  t={time_point:.2f}s")
        return artists

    anim = animation.FuncAnimation(
        fig, _update, frames=num_frames, interval=1000 / fps, blit=True
    )

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    anim.save(output, writer="ffmpeg", fps=fps)
    plt.close(fig)

    return output


def _rectangle_vertices(pose: Pose, length_mm: float, width_mm: float) -> List[Tuple[float, float]]:
    from math import radians, sin, cos

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

    vertices = []
    for along, lateral in corners:
        vx = pose.x + forward[0] * along + right[0] * lateral
        vy = pose.y + forward[1] * along + right[1] * lateral
        vertices.append((vx, vy))
    return vertices


def _sin_deg(angle: float) -> float:
    from math import sin, radians

    return sin(radians(angle))


def _cos_deg(angle: float) -> float:
    from math import cos, radians

    return cos(radians(angle))

