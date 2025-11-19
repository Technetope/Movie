#!/usr/bin/env python3
"""
JSONファイルの整合性検証スクリプト（32.5–50 s スパイラル対応）
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

FIELD_MIN_X = 34.0
FIELD_MAX_X = 949.0
FIELD_MIN_Y = 35.0
FIELD_MAX_Y = 898.0

NUM_ROBOTS = 20
FORMATION_ROWS = 5
FORMATION_COLS = 4
GAP_X = 10.7
GAP_Y = 18.0

QUADRANT_CENTERS: Dict[str, Tuple[float, float]] = {
    "Q1": (262.75, 250.75),
    "Q2": (262.75, 682.25),
    "Q3": (720.25, 682.25),
    "Q4": (720.25, 250.75),
    "CENTER": (491.5, 466.5),
}

SPIRAL_START_TIME = 32.5
SPIRAL_END_TIME = 50.0
SPIRAL_DT = 0.5
PAIRWISE_SAFE_DISTANCE = 20.0  # mm
PAIRWISE_CHECK_START = 34.0

POSITION_SPEED_LIMIT = 200.0  # mm/s
ROTATION_SPEED_LIMIT = 320.0  # deg/s
SPIRAL_MAX_RADIUS = 400.0  # mm
SPIRAL_MAX_TANGENTIAL_SPEED = 160.0  # mm/s


def build_expected_center_positions() -> Dict[int, Tuple[float, float]]:
    cell_x = 32.0 + GAP_X
    cell_y = 72.0 + GAP_Y
    col_center = (FORMATION_COLS - 1) / 2.0
    row_center = (FORMATION_ROWS - 1) / 2.0
    center = (491.5, 466.5)
    expected: Dict[int, Tuple[float, float]] = {}
    for idx in range(NUM_ROBOTS):
        row = idx // FORMATION_COLS
        col = idx % FORMATION_COLS
        dx = (col - col_center) * cell_x
        dy = (row - row_center) * cell_y
        expected[idx] = (center[0] + dx, center[1] + dy)
    return expected


def load_robot_frames(output_dir: Path, robot_id: int) -> List[dict]:
    json_file = output_dir / f"robot_{robot_id:02d}.json"
    with json_file.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    frames = data.get("frames", [])
    frames.sort(key=lambda f: f.get("time", 0.0))
    return frames


def ensure_position_fields(frame: dict, robot_id: int, index: int, errors: List[str]) -> None:
    if frame.get("use_position"):
        if "x" not in frame or "y" not in frame:
            errors.append(f"robot {robot_id:02d} frame {index}: missing x/y")
            return
        x, y = frame["x"], frame["y"]
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            errors.append(f"robot {robot_id:02d} frame {index}: x/y not numeric")
            return
        if not (FIELD_MIN_X <= x <= FIELD_MAX_X):
            errors.append(f"robot {robot_id:02d} frame {index}: x out of bounds ({x:.1f})")
        if not (FIELD_MIN_Y <= y <= FIELD_MAX_Y):
            errors.append(f"robot {robot_id:02d} frame {index}: y out of bounds ({y:.1f})")


def ensure_rotation_field(frame: dict, robot_id: int, index: int, errors: List[str]) -> None:
    if frame.get("use_rotation"):
        rotation = frame.get("rotation")
        if rotation is None or not isinstance(rotation, (int, float)):
            errors.append(f"robot {robot_id:02d} frame {index}: rotation missing/invalid")


def validate_structure(frames: List[dict], robot_id: int) -> List[str]:
    errors: List[str] = []
    for idx, frame in enumerate(frames):
        if "time" not in frame or not isinstance(frame["time"], (int, float)):
            errors.append(f"robot {robot_id:02d} frame {idx}: invalid time")
            continue
        if not isinstance(frame.get("use_position"), bool):
            errors.append(f"robot {robot_id:02d} frame {idx}: use_position not bool")
        if not isinstance(frame.get("use_rotation"), bool):
            errors.append(f"robot {robot_id:02d} frame {idx}: use_rotation not bool")
        ensure_position_fields(frame, robot_id, idx, errors)
        ensure_rotation_field(frame, robot_id, idx, errors)
    return errors


def validate_time_order(frames: List[dict], robot_id: int) -> List[str]:
    errors: List[str] = []
    prev_time = None
    for idx, frame in enumerate(frames):
        t = frame["time"]
        if prev_time is not None and t < prev_time - 1e-6:
            errors.append(f"robot {robot_id:02d} frame {idx}: time decreased ({prev_time}→{t})")
        prev_time = t
    return errors


def validate_speed(frames: List[dict], robot_id: int) -> List[str]:
    warnings: List[str] = []
    prev = None
    for frame in frames:
        if frame.get("use_position"):
            t = frame["time"]
            if t < SPIRAL_START_TIME - 1e-6 or t > SPIRAL_END_TIME + 1e-6:
                continue
            pos = (frame["x"], frame["y"])
            if prev is not None:
                dt = t - prev[1]
                if dt > 0:
                    dist = math.hypot(pos[0] - prev[0][0], pos[1] - prev[0][1])
                    speed = dist / dt
                    if speed > POSITION_SPEED_LIMIT + 1e-3:
                        warnings.append(
                            f"robot {robot_id:02d}: translation speed {speed:.1f}mm/s at {prev[1]:.2f}→{t:.2f}"
                        )
            prev = (pos, t)
    return warnings


def validate_rotation(frames: List[dict], robot_id: int) -> List[str]:
    warnings: List[str] = []
    prev = None
    for frame in frames:
        if frame.get("use_rotation"):
            t = frame["time"]
            if t < SPIRAL_START_TIME - 1e-6 or t > SPIRAL_END_TIME + 1e-6:
                continue
            rot = frame["rotation"]
            if prev is not None:
                dt = t - prev[1]
                if dt > 0:
                    diff = (rot - prev[0]) % 360.0
                    if diff > 180.0:
                        diff = 360.0 - diff
                    speed = abs(diff) / dt
                    if speed > ROTATION_SPEED_LIMIT + 1e-3:
                        warnings.append(
                            f"robot {robot_id:02d}: rotation speed {speed:.1f}deg/s at {prev[1]:.2f}→{t:.2f}"
                        )
            prev = (rot, t)
    return warnings


def validate_baseline(frames: List[dict], robot_id: int, expected: Tuple[float, float]) -> List[str]:
    errors: List[str] = []
    target = None
    for frame in frames:
        if frame.get("use_position") and abs(frame["time"] - SPIRAL_START_TIME) < 1e-3:
            target = (frame["x"], frame["y"])
            break
    if target is None:
        errors.append(f"robot {robot_id:02d}: missing 32.5s position frame")
        return errors
    dx = target[0] - expected[0]
    dy = target[1] - expected[1]
    if math.hypot(dx, dy) > 1.0:
        errors.append(
            f"robot {robot_id:02d}: 32.5s position deviates from expected center grid "
            f"(diff={math.hypot(dx, dy):.1f}mm)"
        )
    return errors


def build_position_lookup(frames: List[dict]) -> Dict[float, Tuple[float, float]]:
    lookup: Dict[float, Tuple[float, float]] = {}
    for frame in frames:
        if frame.get("use_position"):
            lookup[frame["time"]] = (frame["x"], frame["y"])
    return lookup


def validate_pairwise(positions: Dict[int, Dict[float, Tuple[float, float]]]) -> List[str]:
    errors: List[str] = []
    t = max(SPIRAL_START_TIME, PAIRWISE_CHECK_START)
    while t <= SPIRAL_END_TIME + 1e-6:
        missing = [rid for rid, lookup in positions.items() if t not in lookup]
        if missing:
            errors.append(f"time {t:.2f}: missing positions for robots {missing}")
            t = round(t + SPIRAL_DT, 3)
            continue
        robots = sorted(positions.keys())
        for i, rid_a in enumerate(robots):
            pos_a = positions[rid_a][t]
            for rid_b in robots[i + 1 :]:
                pos_b = positions[rid_b][t]
                dist = math.hypot(pos_a[0] - pos_b[0], pos_a[1] - pos_b[1])
                if dist < PAIRWISE_SAFE_DISTANCE:
                    errors.append(
                        f"time {t:.2f}: robots {rid_a:02d}/{rid_b:02d} too close ({dist:.1f}mm)"
                    )
        t = round(t + SPIRAL_DT, 3)
    return errors


def validate_spiral_metrics(frames: List[dict], robot_id: int) -> Tuple[List[str], float, float]:
    errors: List[str] = []
    center = QUADRANT_CENTERS["CENTER"]
    max_radius = 0.0
    max_tangential_speed = 0.0
    prev_point: Tuple[float, float] | None = None
    prev_time: float | None = None

    for frame in frames:
        if not frame.get("use_position"):
            continue
        t = frame["time"]
        if t < SPIRAL_START_TIME - 1e-6 or t > SPIRAL_END_TIME + 1e-6:
            continue
        x, y = frame["x"], frame["y"]
        dx = x - center[0]
        dy = y - center[1]
        radius = math.hypot(dx, dy)
        max_radius = max(max_radius, radius)
        if radius > SPIRAL_MAX_RADIUS + 1e-3:
            errors.append(
                f"robot {robot_id:02d}: radius {radius:.1f}mm exceeds {SPIRAL_MAX_RADIUS}mm at t={t:.2f}"
            )
        if prev_point is not None and prev_time is not None:
            dt = t - prev_time
            if dt > 0:
                dist = math.hypot(x - prev_point[0], y - prev_point[1])
                tangential = dist / dt
                max_tangential_speed = max(max_tangential_speed, tangential)
                if tangential > SPIRAL_MAX_TANGENTIAL_SPEED + 1e-3:
                    errors.append(
                        f"robot {robot_id:02d}: tangential speed {tangential:.1f}mm/s exceeds {SPIRAL_MAX_TANGENTIAL_SPEED}mm/s at t={prev_time:.2f}→{t:.2f}"
                    )
        prev_point = (x, y)
        prev_time = t

    return errors, max_radius, max_tangential_speed


def main() -> None:
    output_dir = Path("quadrant_output")
    if not output_dir.exists():
        raise SystemExit("quadrant_output ディレクトリが見つかりません")

    expected_positions = build_expected_center_positions()

    all_errors: List[str] = []
    all_warnings: List[str] = []
    position_lookup: Dict[int, Dict[float, Tuple[float, float]]] = {}
    max_radius_global = 0.0
    max_tangential_global = 0.0

    for robot_id in range(NUM_ROBOTS):
        try:
            frames = load_robot_frames(output_dir, robot_id)
        except FileNotFoundError:
            all_errors.append(f"robot {robot_id:02d}: JSON file not found")
            continue
        except json.JSONDecodeError as exc:
            all_errors.append(f"robot {robot_id:02d}: JSON decode error ({exc})")
            continue

        all_errors.extend(validate_structure(frames, robot_id))
        all_errors.extend(validate_time_order(frames, robot_id))
        all_errors.extend(validate_baseline(frames, robot_id, expected_positions.get(robot_id, (0.0, 0.0))))
        all_warnings.extend(validate_speed(frames, robot_id))
        all_warnings.extend(validate_rotation(frames, robot_id))
        spiral_errors, radius_max, tangential_max = validate_spiral_metrics(frames, robot_id)
        all_errors.extend(spiral_errors)
        max_radius_global = max(max_radius_global, radius_max)
        max_tangential_global = max(max_tangential_global, tangential_max)
        position_lookup[robot_id] = build_position_lookup(frames)

    all_errors.extend(validate_pairwise(position_lookup))

    print("=== Validation Summary ===")
    print(f"Errors   : {len(all_errors)}")
    print(f"Warnings : {len(all_warnings)}")
    print(f"Max spiral radius     : {max_radius_global:.1f} mm")
    print(f"Max tangential speed  : {max_tangential_global:.1f} mm/s\n")

    if all_errors:
        print("Errors:")
        for msg in all_errors[:20]:
            print(f"  - {msg}")
        if len(all_errors) > 20:
            print(f"  - ... and {len(all_errors) - 20} more")
    else:
        print("No errors detected.")

    if all_warnings:
        print("\nWarnings:")
        for msg in all_warnings[:20]:
            print(f"  - {msg}")
        if len(all_warnings) > 20:
            print(f"  - ... and {len(all_warnings) - 20} more")
    else:
        print("\nNo warnings detected.")


if __name__ == "__main__":
    main()

