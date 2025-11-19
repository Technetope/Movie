#!/usr/bin/env python3
"""
Merge multiple phase logs into a single all_robots.json stream (0-150s).

Usage:
  python tools/merge_all_robots.py \
      --segment 0 80 quadrant_output/all_robots.json \
      --segment 80 120 logs/swarm_raw.json \
      --segment 120 150 logs/swarm_align.json \
      --output quadrant_output/all_robots_merged.json
"""

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple


def load_log(path: Path) -> Dict[int, List[Dict]]:
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    if "by_robot" in data:
        return {int(k): v for k, v in data["by_robot"].items()}
    if "sets" in data:
        logs: Dict[int, List[Dict]] = {}
        for entry in data["sets"]:
            rid = int(entry["id"].split("_")[-1])
            logs[rid] = entry["frames"]
        return logs
    raise ValueError(f"Unsupported format: {path}")


def get_time(frame: Dict) -> float:
    return round(frame.get("time", frame.get("t")), 3)


def slice_frames(frames: List[Dict], start: float, end: float) -> List[Dict]:
    sliced = []
    for frame in frames:
        t = get_time(frame)
        if start <= t < end:
            frame = dict(frame)
            frame["time"] = t
            sliced.append(frame)
    return sliced


def convert_frame(frame: Dict, last_position: Tuple[float, float]) -> Tuple[Dict, Tuple[float, float]]:
    time = frame["time"]
    use_position = frame.get("use_position", frame.get("up", True))
    use_rotation = frame.get("use_rotation", frame.get("use_heading", frame.get("ur", frame.get("uh", False))))

    x = frame.get("x")
    y = frame.get("y")
    if x is None or y is None:
        pos = frame.get("position")
        if pos:
            x = pos.get("x", x)
            y = pos.get("y", y)

    if x is None or y is None:
        x, y = last_position
    else:
        last_position = (x, y)

    rotation = frame.get("rotation", frame.get("angle", frame.get("ang", 0.0)))
    sound = frame.get("sound")
    stop_distance = frame.get("stop_distance", frame.get("sd", 20.0))
    angle_tol = frame.get("angle_tolerance", frame.get("at", 5.0))

    return (
        {
            "time": time,
            "use_position": bool(use_position),
            "use_rotation": bool(use_rotation),
            "x": round(x, 3),
            "y": round(y, 3),
            "rotation": round(rotation % 360.0, 3),
            "stop_distance": stop_distance,
            "angle_tolerance": angle_tol,
            "sound": sound,
        },
        last_position,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--segment", nargs=3, action="append", metavar=("START", "END", "PATH"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    merged: Dict[int, List[Dict]] = {}
    last_positions: Dict[int, Tuple[float, float]] = {}
    for start_str, end_str, path_str in args.segment:
        start = float(start_str)
        end = float(end_str)
        logs = load_log(Path(path_str))
        for rid, frames in logs.items():
            merged.setdefault(rid, [])
            last_pos = last_positions.get(rid, (0.0, 0.0))
            for frame in slice_frames(frames, start, end):
                converted, last_pos = convert_frame(frame, last_pos)
                merged[rid].append(converted)
            last_positions[rid] = last_pos

    for frames in merged.values():
        frames.sort(key=lambda f: f["time"])

    sets = []
    for rid in sorted(merged.keys()):
        sets.append({"id": f"robot_{rid:02d}", "frames": merged[rid]})

    output_payload = {"sets": sets}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as fh:
        json.dump(output_payload, fh, ensure_ascii=False, indent=2)

    print(f"wrote merged timeline to {args.output}")


if __name__ == "__main__":
    main()

