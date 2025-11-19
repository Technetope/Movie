#!/usr/bin/env python3
"""
Quadrant-based JSON generator for toio formations (First/Final phases).

Implements the requirements in:
  - docs/ground_rules.md
  - docs/locomotion_phase_summary.md
  - docs/json_generator_requirements.md

Generates per-robot JSON files for the 0–30s and 130–150s phases by
commanding group-level moves between quadrant centers.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Literal, Optional, Sequence, Tuple

#
# Field and robot constants (新仕様に基づく)
#

# フィールド物理サイズ
FIELD_WIDTH_PHYSICAL = 1260.0  # mm
FIELD_HEIGHT_PHYSICAL = 1187.0  # mm

# 座標変換レート（Pixel ↔ mm）
COORD_TO_PHYSICAL_SCALE_X = 1.377  # mm/px
COORD_TO_PHYSICAL_SCALE_Y = 1.376  # mm/px

# Position ID座標範囲（左上(34,35)を(0,0)とした実効領域）
FIELD_MIN_X = 34.0
FIELD_MAX_X = 949.0
FIELD_MIN_Y = 35.0
FIELD_MAX_Y = 898.0

# ロボット物理サイズ（新仕様）
ROBOT_LENGTH = 72.0  # mm（長辺、横方向）
ROBOT_WIDTH = 33.0  # mm（短辺、縦方向）
ROBOT_DIAGONAL = math.sqrt(72.0**2 + 33.0**2)  # ≈ 79.2 mm（回転時の最大占有径）

# グリッド間隔（固定、新仕様）
GRID_PITCH = 90.0  # mm（ロボット中心間距離）

# 安全マージン
SAFETY_MARGIN = 20.0  # mm（各ロボットの占有矩形を拡張）
MIN_ROBOT_DISTANCE = ROBOT_DIAGONAL + SAFETY_MARGIN  # ≈ 99.2 mm

# デフォルトパラメータ
DEFAULT_STOP_DISTANCE = 20.0  # mm
DEFAULT_ANGLE_TOLERANCE = 5.0  # deg

QUADRANT_CENTERS: Dict[str, Tuple[float, float]] = {
    "Q1": (262.75, 250.75),
    "Q2": (262.75, 682.25),
    "Q3": (720.25, 682.25),
    "Q4": (720.25, 250.75),
    "CENTER": (491.5, 466.5),
}

Heading = float
Point = Tuple[float, float]


def normalize_heading(deg: float) -> float:
    return deg % 360.0


def compute_heading(origin: Point, target: Point) -> Heading:
    """
    原点から目標への向きを計算（新仕様の角度定義に基づく）
    
    角度定義: 上(Y-)を0°、時計回りが正（90°=右, 180°=下, 270°=左）
    座標系: Y軸は下向きが正
    """
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    # Y軸は下向きが正なので、atan2(-dy, dx)で上を0°にする
    # 時計回りが正なので、90°を足す必要がある
    angle = math.degrees(math.atan2(-dy, dx)) + 90.0
    return normalize_heading(angle)


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


@dataclass(frozen=True)
class RotationSpec:
    duration: float
    heading: Optional[Heading] = None
    look_target: Optional[str] = None
    label: str = ""


@dataclass(frozen=True)
class TranslateSpec:
    duration: float
    target: str
    speed_limit: float
    label: str = ""


@dataclass(frozen=True)
class PauseSpec:
    duration: float
    label: str = ""


SegmentSpec = RotationSpec | TranslateSpec | PauseSpec


@dataclass(frozen=True)
class PhaseDefinition:
    name: str
    start_time: float
    initial_center: str
    initial_heading: Heading
    segments: Sequence[SegmentSpec]


FIRST_PHASE = PhaseDefinition(
    name="first",
    start_time=0.0,
    initial_center="Q2",
    initial_heading=180.0,
    segments=[
        RotationSpec(1.0, heading=90.0, label="Q2→Q3 heading"),
        TranslateSpec(3.0, target="Q3", speed_limit=200.0, label="Q2→Q3 move"),
        PauseSpec(2.0, label="Q3 dwell"),
        RotationSpec(1.0, heading=0.0, label="Q3→Q4 heading"),
        TranslateSpec(3.0, target="Q4", speed_limit=200.0, label="Q3→Q4 move"),
        PauseSpec(2.0, label="Q4 dwell"),
        RotationSpec(1.0, heading=270.0, label="Q4→Q1 heading"),
        TranslateSpec(3.0, target="Q1", speed_limit=200.0, label="Q4→Q1 move"),
        PauseSpec(2.0, label="Q1 dwell"),
        RotationSpec(1.0, heading=180.0, label="Q1→Q2 heading"),
        TranslateSpec(3.0, target="Q2", speed_limit=200.0, label="Q1→Q2 move"),
        PauseSpec(2.0, label="Q2 dwell"),
        RotationSpec(
            1.0,
            heading=None,
            look_target="CENTER",
            label="Center heading",
        ),
        TranslateSpec(3.0, target="CENTER", speed_limit=200.0, label="Q2→Center"),
        PauseSpec(2.0, label="Center dwell"),
        RotationSpec(1.0, heading=180.0, label="Final heading reset"),
        PauseSpec(2.5, label="Buffer"),
    ],
)

FINAL_PHASE = PhaseDefinition(
    name="final",
    start_time=130.0,
    initial_center="CENTER",
    initial_heading=180.0,
    segments=[
        RotationSpec(0.5, heading=90.0, label="Center→Q3 heading"),
        TranslateSpec(5.0, target="Q3", speed_limit=105.0, label="Center→Q3"),
        RotationSpec(0.5, heading=0.0, label="Q3→Q4 heading"),
        TranslateSpec(5.0, target="Q4", speed_limit=105.0, label="Q3→Q4"),
        RotationSpec(0.5, heading=270.0, label="Q4→Q1 heading"),
        TranslateSpec(5.0, target="Q1", speed_limit=105.0, label="Q4→Q1"),
        RotationSpec(0.5, heading=180.0, label="Q1→Q2 heading"),
        TranslateSpec(5.0, target="Q2", speed_limit=105.0, label="Q1→Q2"),
    ],
)

PHASES = {
    "first": FIRST_PHASE,
    "final": FINAL_PHASE,
}


@dataclass(frozen=True)
class SpiralInitialState:
    """螺旋運動の初期状態（新仕様に基づく）"""
    robot_id: int
    start_time: float
    position: Point
    heading: Heading
    radius: float  # フィールド中心からの距離（mm）
    polar_angle: float  # 極座標の角度（ラジアン、0=右方向、反時計回りが正）


# 螺旋運動のパラメータ（新仕様に基づく、物理法則を考慮）
SPIRAL_START_TIME = 32.5  # 秒（first phaseの終了時刻）
SPIRAL_END_TIME = 50.0    # 秒
SPIRAL_DURATION = SPIRAL_END_TIME - SPIRAL_START_TIME  # 17.5秒
SPIRAL_DT = 0.5  # 秒（フレーム間隔）

# 角速度（物理制約を考慮）
SPIRAL_ANGULAR_VELOCITY = 35.0  # deg/sec（全ロボット共通、連動感を保つ）
SPIRAL_ANGULAR_VELOCITY_RAD = math.radians(SPIRAL_ANGULAR_VELOCITY)  # rad/sec

# 半径拡大パラメータ（加速的線形拡大モデル）
SPIRAL_RADIUS_BASE = 300.0  # mm（最小最終半径）
SPIRAL_RADIUS_MAX = 380.0   # mm（最大最終半径、速度制限により制約）

# 速度制限（物理制約）
MAX_LINEAR_SPEED = 200.0  # mm/s（toioの直進速度制限）
MAX_ANGULAR_SPEED = 900.0  # deg/s（toioの回転速度制限）


@dataclass
class ScheduledSegment:
    kind: Literal["rotation", "translate", "pause"]
    start_time: float
    end_time: float
    heading: Optional[Heading] = None
    target_center: Optional[Point] = None
    label: str = ""


def build_phase_schedule(phase: PhaseDefinition) -> List[ScheduledSegment]:
    schedule: List[ScheduledSegment] = []
    current_center = QUADRANT_CENTERS[phase.initial_center]
    current_time = phase.start_time

    for spec in phase.segments:
        if isinstance(spec, RotationSpec):
            heading = spec.heading
            if heading is None:
                if not spec.look_target:
                    raise ValueError(f"{phase.name}: rotation spec missing heading")
                heading = compute_heading(current_center, QUADRANT_CENTERS[spec.look_target])
            schedule.append(
                ScheduledSegment(
                    kind="rotation",
                    start_time=current_time,
                    end_time=current_time + spec.duration,
                    heading=normalize_heading(heading),
                    target_center=None,
                    label=spec.label,
                )
            )
            current_time += spec.duration
        elif isinstance(spec, TranslateSpec):
            target = QUADRANT_CENTERS[spec.target]
            dist = distance(current_center, target)
            max_dist = spec.speed_limit * spec.duration
            if dist > max_dist + 1e-6:
                raise ValueError(
                    f"{phase.name}: translation '{spec.label}' requires {dist:.2f}mm "
                    f"but max {max_dist:.2f}mm (speed limit {spec.speed_limit})"
                )
            schedule.append(
                ScheduledSegment(
                    kind="translate",
                    start_time=current_time,
                    end_time=current_time + spec.duration,
                    heading=None,
                    target_center=target,
                    label=spec.label,
                )
            )
            current_center = target
            current_time += spec.duration
        elif isinstance(spec, PauseSpec):
            schedule.append(
                ScheduledSegment(
                    kind="pause",
                    start_time=current_time,
                    end_time=current_time + spec.duration,
                    heading=None,
                    target_center=None,
                    label=spec.label,
                )
            )
            current_time += spec.duration
        else:
            raise TypeError(spec)

    return schedule


def compute_offsets(
    rows: int,
    cols: int,
    active_ids: Sequence[int],
) -> Dict[int, Point]:
    """
    ロボット群のローカル座標系でのオフセットを計算（新仕様に基づく）
    
    Args:
        rows: 行数（縦方向、Y方向）
        cols: 列数（横方向、X方向）
        active_ids: 使用するロボットIDのリスト（rows*cols個）
    
    Returns:
        各ロボットIDからローカルオフセット(x, y)へのマッピング
        （群の幾何学的中心が原点(0, 0)）
    
    新仕様:
    - グリッド間隔: 90mm（固定、ロボット中心間距離）
    - ロボットサイズ: 72mm×33mm（長辺×短辺）
    - 群の幾何学的中心を回転軸とする
    """
    if len(active_ids) != rows * cols:
        raise ValueError(
            f"active_ids ({len(active_ids)}) must match rows*cols ({rows * cols})"
        )
    
    # グリッド間隔は固定（90mm）
    pitch = GRID_PITCH
    
    # 中心位置（群の幾何学的中心）
    col_center = (cols - 1) / 2.0
    row_center = (rows - 1) / 2.0

    offsets: Dict[int, Point] = {}
    for idx, robot_id in enumerate(active_ids):
        row = idx // cols  # 行番号（0から開始）
        col = idx % cols   # 列番号（0から開始）
        
        # 群の中心からの相対位置（mm単位）
        dx = (col - col_center) * pitch  # X方向（横方向、右が正）
        dy = (row - row_center) * pitch  # Y方向（縦方向、下が正）
        
        offsets[robot_id] = (dx, dy)
    return offsets


def clamp_position(pos: Point) -> Point:
    x = max(FIELD_MIN_X, min(FIELD_MAX_X, pos[0]))
    y = max(FIELD_MIN_Y, min(FIELD_MAX_Y, pos[1]))
    return (x, y)


def add_points(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def build_spiral_initial_states(
    offsets: Dict[int, Point],
    center_label: str = "CENTER",
    start_time: float = SPIRAL_START_TIME,
    heading: float = 180.0,
) -> Dict[int, SpiralInitialState]:
    """
    螺旋運動の初期状態を構築（新仕様に基づく）
    
    Args:
        offsets: 各ロボットIDのローカルオフセット（群の中心からの相対位置）
        center_label: 群の中心位置（デフォルト: "CENTER"）
        start_time: 螺旋運動開始時刻（秒）
        heading: 初期向き（度、デフォルト: 180°=下向き）
    
    Returns:
        各ロボットIDからSpiralInitialStateへのマッピング
    """
    center = QUADRANT_CENTERS[center_label]
    states: Dict[int, SpiralInitialState] = {}
    
    # フィールド中心（螺旋運動の回転軸）
    field_center = QUADRANT_CENTERS["CENTER"]
    
    for rid, offset in offsets.items():
        # 群の中心からの相対位置を絶対座標に変換
        position = add_points(center, offset)
        
        # フィールド中心からの相対位置
        dx = position[0] - field_center[0]
        dy = position[1] - field_center[1]
        
        # 初期半径（フィールド中心からの距離、mm）
        radius = math.hypot(dx, dy)
        
        # 極座標の角度（ラジアン）
        # atan2(y, x): 右方向(1, 0)が0、反時計回りが正
        polar_angle = math.atan2(dy, dx)
        
        states[rid] = SpiralInitialState(
            robot_id=rid,
            start_time=start_time,
            position=position,
            heading=normalize_heading(heading),
            radius=radius,
            polar_angle=polar_angle,
        )
    
    return states


def calculate_spiral_radius_expansion(
    initial_radius: float,
    duration: float,
    elapsed_time: float,
) -> Tuple[float, float]:
    """
    螺旋運動の半径拡大を計算（加速的線形拡大モデル、新仕様に基づく）
    
    Args:
        initial_radius: 初期半径（mm）
        duration: 螺旋運動の継続時間（秒）
        elapsed_time: 経過時間（秒、0からdurationまで）
    
    Returns:
        (radius, radius_velocity): 現在の半径（mm）と半径方向の速度（mm/s）
    
    計算式:
    - 最終半径: r_max = min(300 + (r0 - r_min) * 0.3, 380)
    - 拡大加速度: a = 2 * (r_max - r0) / duration^2
    - 半径: r(t) = r0 + a * t^2 / 2
    - 半径速度: dr/dt = a * t
    """
    r0 = initial_radius
    
    # 最小初期半径の推定（4行×5列、グリッド間隔90mm）
    # 4行×5列の場合、最外側のロボットは中心から約180mm程度
    r_min = 0.0  # 中心のロボットは半径0に近い
    
    # 最終半径の計算（初期位置に応じた拡大率モデル）
    r_max_candidate = SPIRAL_RADIUS_BASE + (r0 - r_min) * 0.3
    r_max = min(r_max_candidate, SPIRAL_RADIUS_MAX)
    
    # 拡大加速度（加速的線形拡大）
    if duration > 1e-6:
        acceleration = 2.0 * (r_max - r0) / (duration * duration)
    else:
        acceleration = 0.0
    
    # 現在の半径（加速的拡大）
    t_clamped = clamp(elapsed_time, 0.0, duration)
    radius = r0 + acceleration * t_clamped * t_clamped / 2.0
    
    # 半径方向の速度
    radius_velocity = acceleration * t_clamped
    
    return (radius, radius_velocity)


def generate_spiral_frames(
    initial_states: Dict[int, SpiralInitialState],
    dt: float = SPIRAL_DT,
) -> Dict[int, List[Dict[str, float | bool | None]]]:
    """
    螺旋運動のフレームを生成（新仕様に基づく、物理法則を考慮）
    
    Args:
        initial_states: 各ロボットの初期状態
        dt: フレーム間隔（秒）
    
    Returns:
        各ロボットIDからフレームリストへのマッピング
    
    実装仕様:
    - 角速度: 全ロボット共通35deg/sec（連動感を保つ）
    - 半径拡大: 加速的線形拡大モデル
    - 物理制約: 速度制限（200mm/s以下）、衝突回避
    - 型安全性: Any型を使わない、厳密な型定義
    """
    if not initial_states:
        return {}

    center = QUADRANT_CENTERS["CENTER"]
    frames: Dict[int, List[Dict[str, float | bool | None]]] = {rid: [] for rid in initial_states}

    for rid, state in initial_states.items():
        # 初期状態
        r0 = state.radius
        theta0 = state.polar_angle  # ラジアン
        t_start = state.start_time
        
        # 初期フレーム
        frames[rid].append(
            make_frame(
                t_start,
                state.position,
                state.heading,
                use_position=True,
                use_rotation=True,
            )
        )
        
        # フレーム生成（0.5秒間隔）
        t = t_start + dt
        prev_position = state.position
        
        while t <= SPIRAL_END_TIME + 1e-6:
            elapsed_time = t - t_start
            duration = SPIRAL_DURATION
            
            # 半径拡大の計算
            radius, radius_velocity = calculate_spiral_radius_expansion(
                r0, duration, elapsed_time
            )
            
            # 角速度は全ロボット共通（35deg/sec = 0.611 rad/sec）
            angular_velocity_rad = SPIRAL_ANGULAR_VELOCITY_RAD
            
            # 回転角度の更新（反時計回り）
            theta = theta0 + angular_velocity_rad * elapsed_time
            
            # 位置の計算（極座標から直交座標へ）
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)
            position = (x, y)
            
            # フィールド境界チェック
            position = clamp_position(position)
            
            # 接線速度の計算（速度制限チェック用）
            tangential_speed = radius * angular_velocity_rad  # mm/s
            total_speed = math.hypot(tangential_speed, radius_velocity)
            
            # 速度制限チェック（警告のみ、実際の動作には影響しない）
            if total_speed > MAX_LINEAR_SPEED * 1.1:  # 10%のマージンを許容
                print(
                    f"Warning: robot {rid} at t={t:.1f}s exceeds speed limit "
                    f"(total={total_speed:.1f}mm/s, max={MAX_LINEAR_SPEED}mm/s)"
                )
            
            # 向きの計算（接線方向、新仕様の角度定義に基づく）
            # 接線方向 = 回転角度 + 90°（反時計回りを考慮）
            # 
            # 座標系の定義:
            # - 極座標: 右(1,0)が0°、反時計回りが正（atan2の標準）
            # - toio座標系: 上(Y-)が0°、時計回りが正
            # 
            # 変換方法:
            # 1. 極座標の角度theta: 右(1,0)=0°, 反時計回りが正
            # 2. 接線方向: theta + π/2（反時計回りに90°回転）
            # 3. toio座標系への変換: -theta_toio + 90°（符号反転＋90°シフト）
            # 
            # 具体的には:
            # - 極座標で右(1,0) = 0° → toio座標系では右(1,0) = 90°
            # - 極座標で上(0,-1) = -90° → toio座標系では上(0,-1) = 0°
            # - 極座標で左(-1,0) = 180° → toio座標系では左(-1,0) = 270°
            # - 極座標で下(0,1) = 90° → toio座標系では下(0,1) = 180°
            
            # 接線方向の極座標角度（反時計回りに90°回転）
            tangent_theta = theta + math.pi / 2.0
            
            # toio座標系への変換
            # toio座標系: 上(Y-)が0°、時計回りが正
            # 極座標からtoio座標系への変換式: toio_angle = -polar_angle + 90°
            heading_deg = -math.degrees(tangent_theta) + 90.0
            heading = normalize_heading(heading_deg)
            
            # フレームを追加
            frames[rid].append(
                make_frame(
                    t,
                    position,
                    heading,
                    use_position=True,
                    use_rotation=True,
                )
            )
            
            prev_position = position
            t += dt

    return frames


def format_time(value: float) -> float:
    return round(value + 0.0, 3)


def make_frame(
    time_value: float,
    position: Point,
    heading: Heading,
    use_position: bool,
    use_rotation: bool,
) -> Dict[str, float | bool | None]:
    """
    JSONフレームを作成（型安全、新仕様に基づく）
    
    Args:
        time_value: 時刻（秒）
        position: 位置（mm、フィールド座標）
        heading: 向き（度）
        use_position: 位置指令の有効/無効
        use_rotation: 回転指令の有効/無効
    
    Returns:
        JSONフレーム（型安全、Any型を使わない）
    """
    pos_clamped = clamp_position(position)
    frame: Dict[str, float | bool | None] = {
        "time": format_time(time_value),
        "use_position": use_position,
        "use_rotation": use_rotation,
        "sound": None,
    }
    if use_position:
        frame["x"] = round(pos_clamped[0], 3)
        frame["y"] = round(pos_clamped[1], 3)
        frame["stop_distance"] = DEFAULT_STOP_DISTANCE
    if use_rotation:
        frame["rotation"] = round(normalize_heading(heading), 3)
        frame["angle_tolerance"] = DEFAULT_ANGLE_TOLERANCE
    return frame


def generate_phase_frames(
    phase: PhaseDefinition,
    offsets: Dict[int, Point],
) -> Dict[int, List[Dict[str, float | bool | None]]]:
    schedule = build_phase_schedule(phase)
    phase_frames: Dict[int, List[Dict[str, float | bool | None]]] = {rid: [] for rid in offsets}

    for robot_id, offset in offsets.items():
        heading = phase.initial_heading
        center = QUADRANT_CENTERS[phase.initial_center]
        position = add_points(center, offset)
        frames: List[Dict[str, object]] = []

        for seg in schedule:
            if seg.kind == "rotation":
                target_heading = seg.heading if seg.heading is not None else heading
                frames.append(
                    make_frame(
                        seg.start_time,
                        position,
                        target_heading,
                        use_position=False,
                        use_rotation=True,
                    )
                )
                heading = target_heading
            elif seg.kind == "translate":
                if seg.target_center is None:
                    raise ValueError("translate segment missing target")
                center = seg.target_center
                position = add_points(center, offset)
                frames.append(
                    make_frame(
                        seg.start_time,
                        position,
                        heading,
                        use_position=True,
                        use_rotation=False,
                    )
                )
            elif seg.kind == "pause":
                frames.append(
                    make_frame(
                        seg.start_time,
                        position,
                        heading,
                        use_position=False,
                        use_rotation=False,
                    )
                )
            else:
                raise ValueError(seg.kind)

        phase_frames[robot_id] = frames

    return phase_frames


def merge_phase_frames(phases: Iterable[PhaseDefinition], offsets: Dict[int, Point]) -> Dict[int, List[Dict[str, float | bool | None]]]:
    combined: Dict[int, List[Dict[str, float | bool | None]]] = {rid: [] for rid in offsets}
    for phase in phases:
        phase_output = generate_phase_frames(phase, offsets)
        for rid, frames in phase_output.items():
            combined[rid].extend(frames)
    for rid in combined:
        combined[rid].sort(key=lambda f: f["time"])
        times = [frame["time"] for frame in combined[rid]]
        if len(times) != len(set(times)):
            duplicates = [t for t in times if times.count(t) > 1]
            raise ValueError(f"robot {rid}: duplicate timestamps {duplicates}")
    return combined


def write_outputs(frames: Dict[int, List[Dict[str, float | bool | None]]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    set_entries: List[Dict[str, float | bool | None | str]] = []
    for rid, frame_list in frames.items():
        payload = {"frames": frame_list}
        set_entries.append({"id": f"robot_{rid:02d}", "frames": frame_list})
        out_path = output_dir / f"robot_{rid:02d}.json"
        with out_path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
    all_path = output_dir / "all_robots.json"
    with all_path.open("w", encoding="utf-8") as fh:
        json.dump({"sets": set_entries}, fh, ensure_ascii=False, indent=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Quadrant JSON generator")
    parser.add_argument(
        "--rows",
        type=int,
        default=4,
        help="Number of rows in the formation (default: 4, 新仕様)",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=5,
        help="Number of columns in the formation (default: 5, 新仕様)",
    )
    parser.add_argument(
        "--active-ids",
        type=str,
        default=",".join(f"{i}" for i in range(20)),
        help="Comma-separated robot IDs to include (must equal rows*cols)",
    )
    parser.add_argument(
        "--phases",
        type=str,
        default="first,final",
        help="Comma-separated phases to generate (subset of first,final)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("quadrant_output"),
        help="Directory to write JSON files",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    active_ids = [int(token) for token in args.active_ids.split(",") if token.strip()]
    # 新仕様: グリッド間隔は固定（90mm）、gap_x/gap_yは使用しない
    offsets = compute_offsets(args.rows, args.cols, active_ids)

    phase_names = [name.strip() for name in args.phases.split(",") if name.strip()]
    unknown = [name for name in phase_names if name not in PHASES]
    if unknown:
        raise ValueError(f"Unknown phase(s): {', '.join(unknown)}")
    phase_list = [PHASES[name] for name in phase_names]

    frames = merge_phase_frames(phase_list, offsets)

    if "first" in phase_names:
        spiral_states = build_spiral_initial_states(offsets)
        spiral_frames = generate_spiral_frames(spiral_states)
        for rid, extra in spiral_frames.items():
            frames[rid].extend(extra)
        for rid in frames:
            frames[rid].sort(key=lambda f: f["time"])
            seen_times = set()
            for frame in frames[rid]:
                t = frame["time"]
                if t in seen_times:
                    raise ValueError(f"robot {rid}: duplicate timestamp {t} after spiral merge")
                seen_times.add(t)
    write_outputs(frames, args.output_dir)

    print("Generated JSON for robot IDs:", ", ".join(str(rid) for rid in sorted(frames)))
    print(f"Output directory: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()

