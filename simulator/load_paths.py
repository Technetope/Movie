from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable, List

from .data_types import Frame, RobotTimeline, validate_timelines


def _parse_frame(payload: dict) -> Frame:
    """Convert a raw JSON frame into a Frame dataclass."""

    required_keys = ("time", "x", "y", "rotation", "use_position", "use_rotation")
    for key in required_keys:
        if key not in payload:
            raise ValueError(f"Frame is missing required key '{key}'")

    return Frame(
        time=float(payload["time"]),
        x=float(payload["x"]),
        y=float(payload["y"]),
        rotation=float(payload["rotation"]),
        use_position=bool(payload["use_position"]),
        use_rotation=bool(payload["use_rotation"]),
    )


def _parse_set(entry: dict) -> RobotTimeline:
    robot_id = entry.get("id")
    if not robot_id:
        raise ValueError("Robot set is missing 'id'")

    frames_raw = entry.get("frames")
    if not isinstance(frames_raw, Iterable):
        raise ValueError(f"Robot '{robot_id}' frames are not iterable")

    frames = [_parse_frame(frame) for frame in frames_raw]
    frames.sort(key=lambda frame: frame.time)

    return RobotTimeline(robot_id=robot_id, frames=frames)


def load_robot_timelines(path: Path | str) -> List[RobotTimeline]:
    """Load all robot timelines from the merged JSON file."""

    source = Path(path)
    if not source.exists():
        raise FileNotFoundError(source)

    data = json.loads(source.read_text())
    sets = data.get("sets")
    if sets is None:
        raise ValueError("JSON root must contain 'sets'")

    timelines = [_parse_set(entry) for entry in sets]

    return validate_timelines(timelines)

