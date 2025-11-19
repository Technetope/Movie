#!/usr/bin/env python3
"""
中央への移動と螺旋運動のスクリプト

1. JSONファイルの最後のフレームから各ロボットの現在位置を読み取る
2. 各ロボットを中央（フィールド中心）に移動させる（4行×5列のフォーメーション）
3. その後、螺旋運動を開始する

入力: JSONファイル（各ロボットの最後の位置）
出力: 中央への移動 + 螺旋運動のJSONファイル
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

# ============================================================================
# Type Definitions
# ============================================================================

Point = Tuple[float, float]  # フィールド座標（mm）
Heading = float  # 角度（度）

# ============================================================================
# Field Constants (新仕様に基づく)
# ============================================================================

# フィールド物理サイズ
FIELD_WIDTH_PHYSICAL = 1260.0  # mm
FIELD_HEIGHT_PHYSICAL = 1187.0  # mm

# Position ID座標範囲
FIELD_MIN_X = 34.0
FIELD_MAX_X = 949.0
FIELD_MIN_Y = 35.0
FIELD_MAX_Y = 898.0

# フィールド中心（螺旋運動の回転軸）
FIELD_CENTER_X = (FIELD_MIN_X + FIELD_MAX_X) / 2.0  # 491.5
FIELD_CENTER_Y = (FIELD_MIN_Y + FIELD_MAX_Y) / 2.0  # 466.5

# フィールド境界からのマージン（200mm、縦も横も）
FIELD_MARGIN = 200.0  # mm
FIELD_SAFE_MIN_X = FIELD_MIN_X + FIELD_MARGIN
FIELD_SAFE_MAX_X = FIELD_MAX_X - FIELD_MARGIN
FIELD_SAFE_MIN_Y = FIELD_MIN_Y + FIELD_MARGIN
FIELD_SAFE_MAX_Y = FIELD_MAX_Y - FIELD_MARGIN

# ロボット物理サイズ（新仕様）
ROBOT_LENGTH = 72.0  # mm（長辺、横方向）
ROBOT_WIDTH = 33.0   # mm（短辺、縦方向）
ROBOT_DIAGONAL = math.sqrt(72.0**2 + 33.0**2)  # ≈ 79.2 mm

# グリッド間隔（rectangle_formation_generator.pyに合わせる）
GRID_PITCH_COL = 150.0  # mm（列方向、X方向、長辺）
GRID_PITCH_ROW = 100.0  # mm（行方向、Y方向、短辺）

# 螺旋運動用のグリッド間隔（新仕様、90mm固定）
SPIRAL_GRID_PITCH = 90.0  # mm（螺旋運動時のロボット中心間距離）

# 安全マージン
SAFETY_MARGIN = 20.0  # mm
MIN_ROBOT_DISTANCE = ROBOT_DIAGONAL + SAFETY_MARGIN  # ≈ 99.2 mm

# フォーメーション定数
FORMATION_ROWS = 4  # 行数（縦方向、Y方向）
FORMATION_COLS = 5  # 列数（横方向、X方向）
NUM_ROBOTS = FORMATION_ROWS * FORMATION_COLS  # 20台

# 螺旋運動のパラメータ
SPIRAL_START_TIME = 0.0  # 螺旋運動開始時刻（映像編集側で調整）
SPIRAL_END_TIME = 17.5   # 螺旋運動終了時刻（17.5秒後）
SPIRAL_DT = 0.5          # フレーム間隔（秒）

# 角速度（時間とともに減衰：最初20°/sec → 最後5°/sec、よりゆっくり）
SPIRAL_ANGULAR_VELOCITY_INITIAL = 20.0  # deg/sec（開始時）
SPIRAL_ANGULAR_VELOCITY_FINAL = 5.0      # deg/sec（終了時）

# 回転開始時刻の分散（各ロボットで開始時刻をずらして分散させる）
SPIRAL_START_TIME_SPREAD = 3.0  # 秒（最大3秒のずれ）

# 半径拡大パラメータ
# 中心に残るロボットと遠くに行くロボットで分かれる
SPIRAL_RADIUS_CENTER_MAX = 150.0  # mm（中心付近のロボットの最大半径）
SPIRAL_RADIUS_OUTER_MAX = 350.0   # mm（外側のロボットの最大半径、マージン200mmを考慮）

# 速度制限
MAX_LINEAR_SPEED = 200.0  # mm/s
MAX_ANGULAR_SPEED = 900.0  # deg/s

# デフォルトパラメータ
DEFAULT_STOP_DISTANCE = 20.0  # mm
DEFAULT_ANGLE_TOLERANCE = 5.0  # deg

# ============================================================================
# Helper Functions
# ============================================================================


def normalize_heading(deg: float) -> float:
    """角度を0°～360°に正規化"""
    return deg % 360.0


def distance(a: Point, b: Point) -> float:
    """2点間の距離を計算（mm）"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def clamp_position(pos: Point) -> Point:
    """位置をフィールド境界内に制限（マージン200mmを考慮）"""
    x = max(FIELD_SAFE_MIN_X, min(FIELD_SAFE_MAX_X, pos[0]))
    y = max(FIELD_SAFE_MIN_Y, min(FIELD_SAFE_MAX_Y, pos[1]))
    return (x, y)


def compute_heading(origin: Point, target: Point) -> Heading:
    """
    原点から目標への向きを計算（新仕様の角度定義に基づく）
    
    角度定義: 上(Y-)を0°、時計回りが正（90°=右, 180°=下, 270°=左）
    座標系: Y軸は下向きが正
    """
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    angle = math.degrees(math.atan2(-dy, dx)) + 90.0
    return normalize_heading(angle)


# ============================================================================
# Formation Functions
# ============================================================================


def compute_formation_offsets(
    use_spiral_pitch: bool = False,
    use_rectangle_spacing: bool = False,
) -> Dict[int, Point]:
    """
    4行×5列のフォーメーションのローカルオフセットを計算
    
    Args:
        use_spiral_pitch: Trueの場合、螺旋運動用の90mm固定間隔を使用
        use_rectangle_spacing: Trueの場合、rectangle_formation_generator.pyと同じ間隔を使用
    
    Returns:
        各ロボットIDからローカルオフセット(x, y)へのマッピング
        （群の幾何学的中心が原点(0, 0)）
    """
    offsets: Dict[int, Point] = {}
    
    # 中心位置（群の幾何学的中心）
    col_center = (FORMATION_COLS - 1) / 2.0
    row_center = (FORMATION_ROWS - 1) / 2.0
    
    # グリッド間隔の選択
    if use_rectangle_spacing:
        # rectangle_formation_generator.pyと同じ間隔（最初との連動）
        pitch_col = GRID_PITCH_COL  # 150.0mm
        pitch_row = GRID_PITCH_ROW  # 100.0mm
    elif use_spiral_pitch:
        pitch_col = SPIRAL_GRID_PITCH
        pitch_row = SPIRAL_GRID_PITCH
    else:
        pitch_col = GRID_PITCH_COL
        pitch_row = GRID_PITCH_ROW
    
    for robot_id in range(NUM_ROBOTS):
        row = robot_id // FORMATION_COLS  # 行番号（0から開始）
        col = robot_id % FORMATION_COLS    # 列番号（0から開始）
        
        # 群の中心からの相対位置（mm単位）
        dx = (col - col_center) * pitch_col  # X方向（横方向、右が正）
        dy = (row - row_center) * pitch_row  # Y方向（縦方向、下が正）
        
        offsets[robot_id] = (dx, dy)
    
    return offsets


def calculate_target_positions(
    center: Point,
    use_spiral_pitch: bool = False,
    use_rectangle_spacing: bool = False,
) -> Dict[int, Point]:
    """
    中央での目標位置を計算（4行×5列のフォーメーション）
    
    Args:
        center: 群の中心位置（フィールド中心）
        use_spiral_pitch: Trueの場合、螺旋運動用の90mm固定間隔を使用
        use_rectangle_spacing: Trueの場合、rectangle_formation_generator.pyと同じ間隔を使用
    
    Returns:
        各ロボットIDから目標位置へのマッピング
    """
    offsets = compute_formation_offsets(
        use_spiral_pitch=use_spiral_pitch,
        use_rectangle_spacing=use_rectangle_spacing,
    )
    positions: Dict[int, Point] = {}
    
    for robot_id, offset in offsets.items():
        position = (center[0] + offset[0], center[1] + offset[1])
        # マージン200mmを考慮してクリップ
        x = max(FIELD_SAFE_MIN_X, min(FIELD_SAFE_MAX_X, position[0]))
        y = max(FIELD_SAFE_MIN_Y, min(FIELD_SAFE_MAX_Y, position[1]))
        positions[robot_id] = (x, y)
    
    return positions


# ============================================================================
# JSON Loading Functions
# ============================================================================


def load_last_position(json_path: Path) -> Tuple[Point, Heading, float]:
    """
    JSONファイルの最後のフレームから位置、向き、時刻を読み取る
    （静止状態のフレームはスキップして、実際に動きがある最後のフレームを取得）
    
    Args:
        json_path: JSONファイルのパス
    
    Returns:
        (position, heading, time): 最後の位置（mm）、向き（度）、時刻（秒）
    """
    with json_path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    
    frames = data.get("frames", [])
    if not frames:
        raise ValueError(f"No frames found in {json_path}")
    
    # 最後のフレームから逆順に検索し、位置が変化している最後のフレームを取得
    # （静止状態のフレームをスキップ）
    last_pos = None
    last_frame = frames[-1]
    
    # 最後から2番目以降のフレームを確認
    for i in range(len(frames) - 1, -1, -1):
        frame = frames[i]
        x = frame.get("x", 0.0)
        y = frame.get("y", 0.0)
        current_pos = (x, y)
        
        if last_pos is None:
            last_pos = current_pos
            last_frame = frame
        elif distance(current_pos, last_pos) > 0.1:  # 0.1mm以上の移動がある場合
            # 位置が変化しているフレームを見つけた
            last_frame = frame
            break
    
    x = last_frame.get("x", 0.0)
    y = last_frame.get("y", 0.0)
    rotation = last_frame.get("rotation", 180.0)
    time = last_frame.get("time", 0.0)
    
    return ((x, y), normalize_heading(rotation), time)


def load_all_last_positions_from_all_robots_json(json_path: Path) -> Tuple[Dict[int, Tuple[Point, Heading]], float]:
    """
    all_robots.jsonから全ロボットの最後の位置を読み込む
    
    Args:
        json_path: all_robots.jsonファイルのパス
    
    Returns:
        (positions, max_time): 各ロボットIDから(位置, 向き)へのマッピングと、最大時刻（秒）
    """
    with json_path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    
    sets = data.get("sets", [])
    if not sets:
        raise ValueError(f"No sets found in {json_path}")
    
    positions: Dict[int, Tuple[Point, Heading]] = {}
    max_time = 0.0
    
    for robot_set in sets:
        robot_id_str = robot_set.get("id", "")
        if not robot_id_str.startswith("robot_"):
            continue
        
        # robot_00 -> 0, robot_01 -> 1, etc.
        try:
            robot_id = int(robot_id_str.split("_")[1])
        except (ValueError, IndexError):
            continue
        
        frames = robot_set.get("frames", [])
        if not frames:
            continue
        
        # 最後のフレームから逆順に検索し、位置が変化している最後のフレームを取得
        # （静止状態のフレームをスキップ）
        last_pos = None
        last_frame = frames[-1]
        
        # 最後から2番目以降のフレームを確認
        for i in range(len(frames) - 1, -1, -1):
            frame = frames[i]
            x = frame.get("x", 0.0)
            y = frame.get("y", 0.0)
            current_pos = (x, y)
            
            if last_pos is None:
                last_pos = current_pos
                last_frame = frame
            elif distance(current_pos, last_pos) > 0.1:  # 0.1mm以上の移動がある場合
                # 位置が変化しているフレームを見つけた
                last_frame = frame
                break
        
        x = last_frame.get("x", 0.0)
        y = last_frame.get("y", 0.0)
        rotation = last_frame.get("rotation", 180.0)
        time = last_frame.get("time", 0.0)
        
        positions[robot_id] = ((x, y), normalize_heading(rotation))
        max_time = max(max_time, time)
    
    return (positions, max_time)


def load_all_last_positions(input_dir: Path) -> Tuple[Dict[int, Tuple[Point, Heading]], float]:
    """
    全ロボットの最後の位置を読み込む
    
    Args:
        input_dir: JSONファイルが格納されているディレクトリ、またはall_robots.jsonファイルのパス
    
    Returns:
        (positions, max_time): 各ロボットIDから(位置, 向き)へのマッピングと、最大時刻（秒）
    """
    # all_robots.jsonファイルかどうかをチェック
    if input_dir.is_file() and input_dir.name == "all_robots.json":
        return load_all_last_positions_from_all_robots_json(input_dir)
    
    # ディレクトリの場合、all_robots.jsonを探す
    all_robots_json = input_dir / "all_robots.json"
    if all_robots_json.exists():
        return load_all_last_positions_from_all_robots_json(all_robots_json)
    
    # 個別のJSONファイルを読み込む（従来の方法）
    positions: Dict[int, Tuple[Point, Heading]] = {}
    max_time = 0.0
    
    for robot_id in range(NUM_ROBOTS):
        json_path = input_dir / f"robot_{robot_id:02d}.json"
        if not json_path.exists():
            raise FileNotFoundError(f"JSON file not found: {json_path}")
        
        pos, heading, time = load_last_position(json_path)
        positions[robot_id] = (pos, heading)
        max_time = max(max_time, time)
    
    return (positions, max_time)


# ============================================================================
# Movement to Center
# ============================================================================


def generate_move_to_center_frames(
    start_positions: Dict[int, Tuple[Point, Heading]],
    target_positions: Dict[int, Point],
    start_time: float,
    speed_mm_per_sec: float,
) -> Dict[int, List[Dict[str, float | bool | None]]]:
    """
    中央への移動フレームを生成（フォーメーション全体を同じベクトルで移動）
    
    Args:
        start_positions: 各ロボットの開始位置と向き
        target_positions: 各ロボットの目標位置（フォーメーション）
        start_time: 開始時刻（秒）
        speed_mm_per_sec: 移動速度（mm/s）
    
    Returns:
        各ロボットIDからフレームリストへのマッピング
    """
    frames: Dict[int, List[Dict[str, float | bool | None]]] = {
        rid: [] for rid in range(NUM_ROBOTS)
    }
    
    # フォーメーションの中心を計算（開始位置と目標位置）
    start_center_x = sum(pos[0] for pos, _ in start_positions.values()) / NUM_ROBOTS
    start_center_y = sum(pos[1] for pos, _ in start_positions.values()) / NUM_ROBOTS
    target_center_x = sum(pos[0] for pos in target_positions.values()) / NUM_ROBOTS
    target_center_y = sum(pos[1] for pos in target_positions.values()) / NUM_ROBOTS
    
    # 移動ベクトル（フォーメーション全体の移動）
    dx_formation = target_center_x - start_center_x
    dy_formation = target_center_y - start_center_y
    dist_formation = math.hypot(dx_formation, dy_formation)
    
    # 移動時間を計算（ゆっくり移動）
    duration = dist_formation / speed_mm_per_sec if speed_mm_per_sec > 0 else 1.0
    
    # 移動方向の向き
    movement_heading = compute_heading(
        (start_center_x, start_center_y),
        (target_center_x, target_center_y),
    )
    
    # 各ロボットの開始位置からの相対オフセットを計算
    start_offsets: Dict[int, Point] = {}
    for robot_id, (start_pos, _) in start_positions.items():
        start_offsets[robot_id] = (
            start_pos[0] - start_center_x,
            start_pos[1] - start_center_y,
        )
    
    # 全ロボットに同じベクトルを適用して移動
    for robot_id in range(NUM_ROBOTS):
        start_pos, start_heading = start_positions[robot_id]
        offset = start_offsets[robot_id]
        
        # 開始フレーム（現在位置）
        frames[robot_id].append({
            "time": round(start_time, 3),
            "use_position": True,
            "use_rotation": False,
            "x": round(start_pos[0], 3),
            "y": round(start_pos[1], 3),
            "rotation": round(normalize_heading(start_heading), 3),
            "stop_distance": DEFAULT_STOP_DISTANCE,
            "sound": None,
        })
        
        # 移動フレームを生成（0.5秒間隔）
        t = start_time + SPIRAL_DT
        end_time = start_time + duration
        
        while t <= end_time + 0.001:  # 小さな誤差を許容
            progress = min((t - start_time) / duration, 1.0) if duration > 0 else 1.0
            
            # フォーメーション中心の位置を補間
            center_x = start_center_x + dx_formation * progress
            center_y = start_center_y + dy_formation * progress
            
            # 各ロボットの位置（相対オフセットを保持）
            x = center_x + offset[0]
            y = center_y + offset[1]
            pos = clamp_position((x, y))
            
            frames[robot_id].append({
                "time": round(t, 3),
                "use_position": True,
                "use_rotation": False,
                "x": round(pos[0], 3),
                "y": round(pos[1], 3),
                "rotation": round(normalize_heading(movement_heading), 3),
                "stop_distance": DEFAULT_STOP_DISTANCE,
                "sound": None,
            })
            
            t += SPIRAL_DT
        
        # 最終フレーム（目標位置、向きを設定）
        final_time = round(end_time, 3)
        has_final_frame = any(
            abs(f["time"] - final_time) < 0.001 for f in frames[robot_id]
        )
        
        if not has_final_frame:
            target_pos = target_positions[robot_id]
            frames[robot_id].append({
                "time": final_time,
                "use_position": True,
                "use_rotation": True,
                "x": round(target_pos[0], 3),
                "y": round(target_pos[1], 3),
                "rotation": round(normalize_heading(movement_heading), 3),
                "stop_distance": DEFAULT_STOP_DISTANCE,
                "angle_tolerance": DEFAULT_ANGLE_TOLERANCE,
                "sound": None,
            })
    
    return frames


# ============================================================================
# Spiral Motion
# ============================================================================


@dataclass(frozen=True)
class SpiralInitialState:
    """螺旋運動の初期状態"""
    robot_id: int
    start_time: float
    position: Point
    heading: Heading
    radius: float  # フィールド中心からの距離（mm）
    polar_angle: float  # 極座標の角度（ラジアン）


def build_spiral_initial_states(
    target_positions: Dict[int, Point],
    start_time: float,
    heading: float = 180.0,
) -> Dict[int, SpiralInitialState]:
    """
    螺旋運動の初期状態を構築（各ロボットの開始時刻をずらして分散させる）
    
    Args:
        target_positions: 各ロボットの目標位置（中央フォーメーション）
        start_time: 螺旋運動開始時刻（秒、基準時刻）
        heading: 初期向き（度、デフォルト: 180°=下向き）
    
    Returns:
        各ロボットIDからSpiralInitialStateへのマッピング
    """
    center = (FIELD_CENTER_X, FIELD_CENTER_Y)
    states: Dict[int, SpiralInitialState] = {}
    
    # 各ロボットの初期半径を計算
    radii: List[float] = []
    for robot_id, position in target_positions.items():
        dx = position[0] - center[0]
        dy = position[1] - center[1]
        radius = math.hypot(dx, dy)
        radii.append(radius)
    
    # 半径の範囲を計算（分散の基準）
    if radii:
        min_radius = min(radii)
        max_radius = max(radii)
        radius_range = max_radius - min_radius if max_radius > min_radius else 1.0
    else:
        radius_range = 1.0
    
    for robot_id, position in target_positions.items():
        # フィールド中心からの相対位置
        dx = position[0] - center[0]
        dy = position[1] - center[1]
        
        # 初期半径（フィールド中心からの距離、mm）
        radius = math.hypot(dx, dy)
        
        # 極座標の角度（ラジアン）
        polar_angle = math.atan2(dy, dx)
        
        # 開始時刻をずらす（半径と角度に基づいて分散）
        # 外側のロボットほど早く開始、内側のロボットほど遅く開始
        # 角度も考慮してより分散させる
        if radius_range > 1.0:
            radius_progress = (radius - min(radii)) / radius_range
        else:
            radius_progress = 0.5
        
        # 角度に基づく分散（0～2πを0～1に正規化）
        angle_progress = (polar_angle + math.pi) / (2.0 * math.pi)
        
        # 開始時刻のオフセット（半径と角度の組み合わせで分散）
        # 外側×角度で最大3秒のずれ
        time_offset = (
            (1.0 - radius_progress) * SPIRAL_START_TIME_SPREAD * 0.6  # 内側ほど遅く
            + angle_progress * SPIRAL_START_TIME_SPREAD * 0.4  # 角度で分散
        )
        
        robot_start_time = start_time + time_offset
        
        states[robot_id] = SpiralInitialState(
            robot_id=robot_id,
            start_time=robot_start_time,
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
    螺旋運動の半径拡大を計算（中心に残るロボットと遠くに行くロボットで分かれる）
    
    Args:
        initial_radius: 初期半径（mm）
        duration: 螺旋運動の継続時間（秒）
        elapsed_time: 経過時間（秒、0からdurationまで）
    
    Returns:
        (radius, radius_velocity): 現在の半径（mm）と半径方向の速度（mm/s）
    """
    r0 = initial_radius
    
    # 中心に残るロボットと遠くに行くロボットで分ける
    # 初期半径が小さい（中心付近）→ 小さな半径のまま
    # 初期半径が大きい（外側）→ 大きく拡大
    threshold_radius = 100.0  # mm（この半径以下は中心に残る）
    
    if r0 <= threshold_radius:
        # 中心に残るロボット：小さな半径のまま（少し拡大）
        r_max = min(SPIRAL_RADIUS_CENTER_MAX, r0 * 1.5)  # 最大1.5倍まで
    else:
        # 遠くに行くロボット：大きく拡大
        # 初期半径に応じて拡大率を調整
        expansion_factor = 1.0 + (SPIRAL_RADIUS_OUTER_MAX - r0) / r0 * 0.8
        r_max = min(r0 * expansion_factor, SPIRAL_RADIUS_OUTER_MAX)
    
    # 拡大加速度（加速的線形拡大）
    if duration > 1e-6:
        acceleration = 2.0 * (r_max - r0) / (duration * duration)
    else:
        acceleration = 0.0
    
    # 現在の半径（加速的拡大）
    t_clamped = max(0.0, min(elapsed_time, duration))
    radius = r0 + acceleration * t_clamped * t_clamped / 2.0
    
    # 半径方向の速度
    radius_velocity = acceleration * t_clamped
    
    return (radius, radius_velocity)


def generate_spiral_frames(
    initial_states: Dict[int, SpiralInitialState],
) -> Dict[int, List[Dict[str, float | bool | None]]]:
    """
    螺旋運動のフレームを生成（中心から放射状に線分を広げながら回転）
    
    螺旋運動 = フォーメーション全体が中心を軸に回転しながら、
    中心から放射状に線分を広げる（各ロボットの中心からの距離が拡大）
    
    Args:
        initial_states: 各ロボットの初期状態
    
    Returns:
        各ロボットIDからフレームリストへのマッピング
    """
    if not initial_states:
        return {}
    
    center = (FIELD_CENTER_X, FIELD_CENTER_Y)
    frames: Dict[int, List[Dict[str, float | bool | None]]] = {
        rid: [] for rid in initial_states
    }
    
    # 全ロボットの基準時刻を計算（最も早い開始時刻）
    base_start_time = min(state.start_time for state in initial_states.values())
    
    # 各ロボットの初期状態を保存
    initial_radii: Dict[int, float] = {}
    initial_angles: Dict[int, float] = {}
    for rid, state in initial_states.items():
        initial_radii[rid] = state.radius
        initial_angles[rid] = state.polar_angle
    
    # 基準時刻からフレーム生成（全ロボット共通の時刻軸）
    t = base_start_time
    end_time = base_start_time + SPIRAL_END_TIME
    
    while t <= end_time + 0.001:
        # 基準時刻からの経過時間
        elapsed_time = t - base_start_time
        
        # 角速度は時間とともに減衰（最初20°/sec → 最後5°/sec）
        if SPIRAL_END_TIME > 1e-6:
            progress = min(elapsed_time / SPIRAL_END_TIME, 1.0)
            # 現在の角速度（deg/sec）
            angular_velocity_deg = SPIRAL_ANGULAR_VELOCITY_INITIAL + (
                SPIRAL_ANGULAR_VELOCITY_FINAL - SPIRAL_ANGULAR_VELOCITY_INITIAL
            ) * progress
            # 角度の積分（二次関数）
            omega0_rad = math.radians(SPIRAL_ANGULAR_VELOCITY_INITIAL)
            omega1_rad = math.radians(SPIRAL_ANGULAR_VELOCITY_FINAL)
            delta_theta_rad = (
                omega0_rad * elapsed_time
                + (omega1_rad - omega0_rad) * elapsed_time * elapsed_time / (2.0 * SPIRAL_END_TIME)
            )
        else:
            angular_velocity_deg = SPIRAL_ANGULAR_VELOCITY_INITIAL
            delta_theta_rad = math.radians(angular_velocity_deg) * elapsed_time
        
        angular_velocity_rad = math.radians(angular_velocity_deg)
        
        # 各ロボットのフレームを生成
        for rid, state in initial_states.items():
            # このロボットの開始時刻を考慮
            robot_elapsed_time = max(0.0, t - state.start_time)
            
            if robot_elapsed_time < 0.0:
                # まだ開始していない場合はスキップ
                continue
            
            r0 = initial_radii[rid]
            theta0 = initial_angles[rid]
            
            # 半径拡大の計算（このロボットの経過時間で計算）
            robot_duration = SPIRAL_END_TIME
            radius, radius_velocity = calculate_spiral_radius_expansion(
                r0, robot_duration, robot_elapsed_time
            )
            
            # 回転角度の更新（全ロボット共通の回転角を使用）
            # フォーメーション全体が回転する
            theta = theta0 + delta_theta_rad
            
            # 位置の計算（極座標から直交座標へ）
            x = center[0] + radius * math.cos(theta)
            y = center[1] + radius * math.sin(theta)
            position = clamp_position((x, y))
            
            # 接線速度の計算（速度制限チェック用）
            tangential_speed = radius * angular_velocity_rad
            total_speed = math.hypot(tangential_speed, radius_velocity)
            
            # 速度制限チェック（警告のみ）
            if total_speed > MAX_LINEAR_SPEED * 1.1:
                print(
                    f"Warning: robot {rid} at t={t:.1f}s exceeds speed limit "
                    f"(total={total_speed:.1f}mm/s, max={MAX_LINEAR_SPEED}mm/s)"
                )
            
            # 向きの計算（接線方向、フォーメーション全体の回転に合わせる）
            tangent_theta = theta + math.pi / 2.0
            heading_deg = -math.degrees(tangent_theta) + 90.0
            heading = normalize_heading(heading_deg)
            
            # 初期フレーム（開始時刻）
            if abs(t - state.start_time) < 0.001:
                frames[rid].append({
                    "time": round(t, 3),
                    "use_position": True,
                    "use_rotation": True,
                    "x": round(state.position[0], 3),
                    "y": round(state.position[1], 3),
                    "rotation": round(state.heading, 3),
                    "stop_distance": DEFAULT_STOP_DISTANCE,
                    "angle_tolerance": DEFAULT_ANGLE_TOLERANCE,
                    "sound": None,
                })
            else:
                # フレームを追加
                frames[rid].append({
                    "time": round(t, 3),
                    "use_position": True,
                    "use_rotation": True,
                    "x": round(position[0], 3),
                    "y": round(position[1], 3),
                    "rotation": round(heading, 3),
                    "stop_distance": DEFAULT_STOP_DISTANCE,
                    "angle_tolerance": DEFAULT_ANGLE_TOLERANCE,
                    "sound": None,
                })
        
        t += SPIRAL_DT
    
    return frames


# ============================================================================
# Output Functions
# ============================================================================


def write_outputs(
    frames: Dict[int, List[Dict[str, float | bool | None]]],
    output_dir: Path,
) -> None:
    """JSONファイルを出力"""
    output_dir.mkdir(parents=True, exist_ok=True)
    
    set_entries: List[Dict[str, float | bool | None | str]] = []
    
    for robot_id, frame_list in frames.items():
        payload = {"frames": frame_list}
        set_entries.append({"id": f"robot_{robot_id:02d}", "frames": frame_list})
        
        out_path = output_dir / f"robot_{robot_id:02d}.json"
        with out_path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
    
    # 統合ファイル
    all_path = output_dir / "all_robots.json"
    with all_path.open("w", encoding="utf-8") as fh:
        json.dump({"sets": set_entries}, fh, ensure_ascii=False, indent=2)


# ============================================================================
# Main
# ============================================================================


def parse_args() -> argparse.Namespace:
    """コマンドライン引数を解析"""
    parser = argparse.ArgumentParser(
        description="中央への移動と螺旋運動のJSON生成"
    )
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="入力: all_robots.jsonファイルのパス、またはJSONファイルが格納されているディレクトリ",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("spiral_output"),
        help="出力ディレクトリ（デフォルト: spiral_output）",
    )
    parser.add_argument(
        "--move-speed",
        type=float,
        default=50.0,
        help="中央への移動速度（mm/s、デフォルト: 50.0）",
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=None,
        help="開始時刻（秒、指定しない場合は入力JSONの最後の時刻+0.5秒）",
    )
    return parser.parse_args()


def main() -> None:
    """メイン処理"""
    args = parse_args()
    
    print("中央への移動と螺旋運動のJSON生成を開始...")
    print(f"  入力: {args.input}")
    print(f"  出力ディレクトリ: {args.output_dir}")
    print(f"  移動速度: {args.move_speed}mm/s")
    
    # 1. 最後の位置を読み込む
    print("\n1. 最後の位置を読み込み中...")
    start_positions, input_end_time = load_all_last_positions(args.input)
    print(f"   {len(start_positions)}台のロボットの位置を読み込みました")
    print(f"   入力JSONの最後の時刻: {input_end_time:.3f}s")
    
    # 開始時刻の決定
    if args.start_time is not None:
        start_time = args.start_time
    else:
        # 入力JSONの最後の時刻+0.5秒から開始（連続性を保つ）
        start_time = input_end_time + SPIRAL_DT
    
    print(f"   中央への移動開始時刻: {start_time:.3f}s")
    
    # 2. 中央での目標位置を計算（4行×5列）
    # 中央への移動時はrectangle_formation_generator.pyの間隔を使用（最初との連動）
    # 螺旋運動開始時は90mm固定間隔に再配置
    print("\n2. 中央での目標位置を計算中...")
    center = (FIELD_CENTER_X, FIELD_CENTER_Y)
    
    # 中央への移動時の目標位置（rectangle_formation_generator.pyの間隔、最初との連動）
    move_target_positions = calculate_target_positions(
        center, use_spiral_pitch=False, use_rectangle_spacing=True
    )
    
    # 螺旋運動開始時の目標位置（90mm固定間隔）
    spiral_target_positions = calculate_target_positions(center, use_spiral_pitch=True)
    
    print(f"   フィールド中心: ({center[0]:.1f}, {center[1]:.1f})")
    print(f"   フィールド安全範囲: X[{FIELD_SAFE_MIN_X:.1f}, {FIELD_SAFE_MAX_X:.1f}], Y[{FIELD_SAFE_MIN_Y:.1f}, {FIELD_SAFE_MAX_Y:.1f}]")
    print(f"   移動時グリッド間隔: 列{GRID_PITCH_COL}mm × 行{GRID_PITCH_ROW}mm（rectangle_formation_generator.pyと同じ）")
    print(f"   螺旋運動時グリッド間隔: {SPIRAL_GRID_PITCH}mm（固定）")
    
    # 3. 中央への移動フレームを生成（rectangle_formation_generator.pyの間隔で配置）
    print("\n3. 中央への移動フレームを生成中...")
    move_frames = generate_move_to_center_frames(
        start_positions,
        move_target_positions,
        start_time,
        args.move_speed,
    )
    
    # 移動の終了時刻を計算
    move_end_time = start_time
    for robot_id in range(NUM_ROBOTS):
        if move_frames[robot_id]:
            last_time = move_frames[robot_id][-1]["time"]
            move_end_time = max(move_end_time, last_time)
    
    print(f"   移動終了時刻: {move_end_time:.1f}s")
    
    # 4. 螺旋運動開始前に90mm固定間隔に再配置（必要に応じて）
    # 移動終了位置と螺旋運動開始位置が異なる場合、追加の移動フレームを生成
    print("\n4. 螺旋運動の初期状態を構築中...")
    
    # 移動終了位置から螺旋運動開始位置への追加移動が必要かチェック
    needs_repositioning = False
    for robot_id in range(NUM_ROBOTS):
        move_end_pos = move_target_positions[robot_id]
        spiral_start_pos = spiral_target_positions[robot_id]
        if distance(move_end_pos, spiral_start_pos) > 1.0:  # 1mm以上の差がある場合
            needs_repositioning = True
            break
    
    if needs_repositioning:
        print("   90mm固定間隔への再配置が必要です")
        # 追加の移動フレームを生成
        # 開始時刻を移動終了時刻より確実に後にする（0.5秒以上の間隔を確保）
        reposition_start_time = move_end_time + SPIRAL_DT
        # 移動フレームの最後の時刻と重複しないように調整
        for robot_id in range(NUM_ROBOTS):
            if move_frames[robot_id]:
                last_time = move_frames[robot_id][-1]["time"]
                if reposition_start_time <= last_time:
                    reposition_start_time = last_time + SPIRAL_DT
        
        reposition_frames = generate_move_to_center_frames(
            {rid: (move_target_positions[rid], 180.0) for rid in range(NUM_ROBOTS)},
            spiral_target_positions,
            reposition_start_time,
            args.move_speed,
        )
        # 移動フレームに追加
        for robot_id in range(NUM_ROBOTS):
            move_frames[robot_id].extend(reposition_frames[robot_id])
            move_frames[robot_id].sort(key=lambda f: f["time"])
        # 再配置の終了時刻を更新
        for robot_id in range(NUM_ROBOTS):
            if move_frames[robot_id]:
                move_end_time = max(move_end_time, move_frames[robot_id][-1]["time"])
    
    # 螺旋運動の初期状態を構築（90mm固定間隔の位置を使用）
    spiral_states = build_spiral_initial_states(
        spiral_target_positions,
        move_end_time,
        heading=180.0,
    )
    
    # 5. 螺旋運動のフレームを生成
    print("\n5. 螺旋運動のフレームを生成中...")
    spiral_frames = generate_spiral_frames(spiral_states)
    
    # 6. フレームを統合
    print("\n6. フレームを統合中...")
    all_frames: Dict[int, List[Dict[str, float | bool | None]]] = {
        rid: [] for rid in range(NUM_ROBOTS)
    }
    
    for robot_id in range(NUM_ROBOTS):
        # すべてのフレームを統合
        all_frames[robot_id].extend(move_frames[robot_id])
        all_frames[robot_id].extend(spiral_frames[robot_id])
        
        # 時刻でソート
        all_frames[robot_id].sort(key=lambda f: f["time"])
        
        # 重複タイムスタンプを処理（同じ時刻のフレームが複数ある場合、後から追加されたものを優先）
        seen_times: Dict[float, int] = {}
        unique_frames: List[Dict[str, float | bool | None]] = []
        
        for frame in all_frames[robot_id]:
            t = frame["time"]
            if t in seen_times:
                # 重複している場合、既存のフレームを削除して新しいフレームを使用
                # （後から追加されたフレームを優先）
                idx = seen_times[t]
                unique_frames[idx] = frame
            else:
                seen_times[t] = len(unique_frames)
                unique_frames.append(frame)
        
        # 再度ソート（念のため）
        unique_frames.sort(key=lambda f: f["time"])
        all_frames[robot_id] = unique_frames
        
        # 最終チェック（重複があればエラー）
        times = [f["time"] for f in all_frames[robot_id]]
        if len(times) != len(set(times)):
            duplicates = [t for t in times if times.count(t) > 1]
            raise ValueError(
                f"robot {robot_id}: duplicate timestamps {duplicates} after deduplication"
            )
    
    # 7. 出力
    print("\n7. JSONファイルを出力中...")
    write_outputs(all_frames, args.output_dir)
    
    # 統計情報
    total_duration = 0.0
    for robot_id in range(NUM_ROBOTS):
        if all_frames[robot_id]:
            total_duration = max(
                total_duration, all_frames[robot_id][-1]["time"]
            )
    
    print(f"\n生成完了!")
    print(f"  ロボット数: {NUM_ROBOTS}台")
    print(f"  総フレーム数: {sum(len(f) for f in all_frames.values())}")
    print(f"  総継続時間: {total_duration:.1f}s")
    print(f"  出力ディレクトリ: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()

