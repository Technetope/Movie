#!/usr/bin/env python3
"""
toioロボット30台のJSONファイル生成スクリプト（0-50秒）
- 0-30秒: 最初のロコモーション（集団行動、fast→stop→fastパターン）
- 30-50秒: 螺旋運動（連続的な回転と拡散）
"""

import math
import json
from typing import List, Tuple, Dict, Any
from pathlib import Path

# ===== 定数定義 =====

# フィールド定数（toioマット座標系 - 3列×4行のマット構成）
# 物理的な全体寸法: 幅1200mm × 高さ1200mm（1.2m × 1.2m、ユーザー指摘に基づく）
# Position ID座標範囲（34-949）と物理サイズ（1200mm）の関係
PHYSICAL_FIELD_WIDTH = 1200.0   # mm（実際の物理幅、1.2m）
PHYSICAL_FIELD_HEIGHT = 1200.0  # mm（実際の物理高さ、1.2m）

# Position ID技術仕様の座標範囲
COORD_MIN_X = 34.0
COORD_MAX_X = 949.0
COORD_MIN_Y = 35.0
COORD_MAX_Y = 898.0
COORD_WIDTH = COORD_MAX_X - COORD_MIN_X   # 915座標単位
COORD_HEIGHT = COORD_MAX_Y - COORD_MIN_Y  # 863座標単位

# 座標→物理のスケール変換
COORD_TO_PHYSICAL_SCALE_X = PHYSICAL_FIELD_WIDTH / COORD_WIDTH   # mm/座標単位 = 約1.3115
COORD_TO_PHYSICAL_SCALE_Y = PHYSICAL_FIELD_HEIGHT / COORD_HEIGHT  # mm/座標単位 = 約1.3903

# Toioの実際のサイズ（ユーザー指摘に基づく、先に定義が必要）
# 物理フィールド幅1.2mに12個のToioが並ぶ = 1個あたり100mmの占有幅
TOIO_COUNT_IN_WIDTH = 12
TOIO_OCCUPIED_WIDTH_PHYSICAL = PHYSICAL_FIELD_WIDTH / TOIO_COUNT_IN_WIDTH  # mm = 100mm（占有幅）
ROBOT_DEPTH_MM = 72.0   # mm（奥行き、物理サイズ）
ROBOT_WIDTH_MM = 32.0   # mm（幅、物理サイズ）
TOIO_SPACING_PHYSICAL = TOIO_OCCUPIED_WIDTH_PHYSICAL - ROBOT_DEPTH_MM  # mm = 28mm（間隔）

# 座標単位でのToioサイズ（座標→物理スケールを使用）
TOIO_OCCUPIED_WIDTH_COORD = TOIO_OCCUPIED_WIDTH_PHYSICAL / COORD_TO_PHYSICAL_SCALE_X  # 約76.25座標単位
ROBOT_DEPTH_COORD = ROBOT_DEPTH_MM / COORD_TO_PHYSICAL_SCALE_X  # 約54.90座標単位
ROBOT_SIZE_COORD = ROBOT_DEPTH_COORD  # 衝突判定用（最大サイズとして奥行きを使用）

# Position ID座標をそのまま使用（座標値はmm単位として扱う前提で、後で変換が必要な場合は調整）
# ただし、グリッド配置は物理サイズに基づいて計算
FIELD_MIN_X = COORD_MIN_X
FIELD_MAX_X = COORD_MAX_X
FIELD_MIN_Y = COORD_MIN_Y
FIELD_MAX_Y = COORD_MAX_Y
FIELD_WIDTH = COORD_WIDTH   # 座標単位
FIELD_HEIGHT = COORD_HEIGHT  # 座標単位
FIELD_CENTER_X = (FIELD_MIN_X + FIELD_MAX_X) / 2.0  # 491.5座標単位
FIELD_CENTER_Y = (FIELD_MIN_Y + FIELD_MAX_Y) / 2.0  # 466.5座標単位

# 各マットの座標範囲（参考）
# 列1（左）: X: 34-339mm
# 列2（中）: X: 340-644mm
# 列3（右）: X: 645-949mm
# 行1（上）: Y: 35-250mm
# 行2: Y: 251-466mm
# 行3: Y: 467-682mm
# 行4（下）: Y: 683-898mm

# グリッド定数（初期配置：6列×5行、30台）
# 要件: フィールドを4象限に分けたうちの1つの象限に集約
GRID_COLS = 6            # 列数
GRID_ROWS = 5            # 行数

# 1象限のサイズ（フィールドを4等分、座標単位）
QUADRANT_WIDTH = FIELD_WIDTH / 2.0   # 座標単位 = 457.5（1象限の幅）
QUADRANT_HEIGHT = FIELD_HEIGHT / 2.0  # 座標単位 = 431.5（1象限の高さ）

# 初期配置を左下象限に集約（左下象限の中心に配置）
# 左下象限の範囲: X < FIELD_CENTER_X, Y >= FIELD_CENTER_Y
# 左下象限の中心を計算（座標単位）
QUADRANT_LL_CENTER_X = FIELD_MIN_X + QUADRANT_WIDTH / 2.0   # 座標単位 = 262.75（左下象限の中心X）
QUADRANT_LL_CENTER_Y = FIELD_CENTER_Y + QUADRANT_HEIGHT / 2.0  # 座標単位 = 682.25（左下象限の中心Y）

# グリッド間隔の計算（Toioの実際の占有幅に基づく）
# 6列の場合: ロボット中心間の間隔 = Toio占有幅 = 100mm（物理） = 約76.25座標単位
# ただし、座標系では直接座標値を扱うため、物理サイズから座標単位への変換が必要
GRID_COL_SPACING = TOIO_OCCUPIED_WIDTH_COORD   # 座標単位 = 約76.25（列間隔、ロボット中心間）
GRID_ROW_SPACING = TOIO_OCCUPIED_WIDTH_COORD   # 座標単位 = 約76.25（行間隔、ロボット中心間、縦方向も同じ）

# マージンを考慮した使用可能領域（1象限内）
QUADRANT_MARGIN_COORD = 20.0 / COORD_TO_PHYSICAL_SCALE_X   # 座標単位（約15.25座標単位、物理20mmに相当）
AVAILABLE_WIDTH = QUADRANT_WIDTH - 2 * QUADRANT_MARGIN_COORD   # 座標単位（1象限内の使用可能幅）
AVAILABLE_HEIGHT = QUADRANT_HEIGHT - 2 * QUADRANT_MARGIN_COORD  # 座標単位（1象限内の使用可能高さ）

# 初期配置の長方形の幅と高さ（座標単位）
GRID_WIDTH = GRID_COL_SPACING * (GRID_COLS - 1)   # 座標単位 = 約381.25（5つの間隔 × 76.25）
GRID_HEIGHT = GRID_ROW_SPACING * (GRID_ROWS - 1)  # 座標単位 = 約305.0（4つの間隔 × 76.25）

# 基準点（左上のロボット位置）: グリッドの中心が左下象限の中心に来るように配置
# グリッドの中心が左下象限の中心になるように開始位置を計算（座標単位）
GRID_START_X = QUADRANT_LL_CENTER_X - GRID_WIDTH / 2.0   # 座標単位（左下象限の中心から左にオフセット）
GRID_START_Y = QUADRANT_LL_CENTER_Y - GRID_HEIGHT / 2.0  # 座標単位（左下象限の中心から上にオフセット）

# 長方形の中心（計算用 - 左下象限の中心と一致するはず）
GRID_CENTER_X = GRID_START_X + GRID_WIDTH / 2.0   # 座標単位 = 262.75（左下象限の中心X）
GRID_CENTER_Y = GRID_START_Y + GRID_HEIGHT / 2.0  # 座標単位 = 682.25（左下象限の中心Y）

# フィールド中心との差分（20.5秒から30秒で移動する距離、座標単位）
CENTER_OFFSET_X = FIELD_CENTER_X - GRID_CENTER_X   # 座標単位 = 228.75（右方向に移動）
CENTER_OFFSET_Y = FIELD_CENTER_Y - GRID_CENTER_Y   # 座標単位 = -215.75（上方向に移動）

# 実際の使用可能幅・高さ（調整後）
AVAILABLE_WIDTH = GRID_WIDTH
AVAILABLE_HEIGHT = GRID_HEIGHT

# 最初のロコモーション定数（0-30秒）
# 速度は物理単位（mm/sec）で定義し、座標単位への変換は使用時に実行
STRAIGHT_SPEED_PHYSICAL = 200.0   # mm/sec（直進速度、物理単位）
ROTATION_SPEED = 180.0   # deg/sec（回転速度）
PAUSE_TIME = 3.0         # 秒（停止時間）
FRAME_INTERVAL = 0.5     # 秒（フレーム間隔）

# 速度を座標単位に変換
STRAIGHT_SPEED = STRAIGHT_SPEED_PHYSICAL / COORD_TO_PHYSICAL_SCALE_X   # 座標単位/sec

# 移動距離（物理単位で定義し、座標単位に変換）
PARALLEL_MOVE_DISTANCE_PHYSICAL = 400.0  # mm（並行移動距離、物理単位）
RECTANGULAR_MOVE_DISTANCE_PHYSICAL = 500.0  # mm（長方形移動距離、物理単位）
POSITION_ADJUST_DISTANCE_PHYSICAL = 250.0  # mm（位置調整移動距離、物理単位）

# 座標単位に変換
PARALLEL_MOVE_DISTANCE = PARALLEL_MOVE_DISTANCE_PHYSICAL / COORD_TO_PHYSICAL_SCALE_X   # 座標単位
RECTANGULAR_MOVE_DISTANCE = RECTANGULAR_MOVE_DISTANCE_PHYSICAL / COORD_TO_PHYSICAL_SCALE_Y   # 座標単位
POSITION_ADJUST_DISTANCE = POSITION_ADJUST_DISTANCE_PHYSICAL / COORD_TO_PHYSICAL_SCALE_X   # 座標単位


# 螺旋運動定数（30-50秒）
SPIRAL_START_TIME = 30.0     # 秒
SPIRAL_END_TIME = 50.0       # 秒
OMEGA_DEG_PER_SEC = 35.0     # deg/sec（基準角速度、個別に変動する）
R_BASE = 300.0               # mm（最小最終半径）
R_MIN = 82.0                 # mm（最小初期半径、推定）
K = 0.3                      # 拡大係数
R_MAX_MAX = 380.0            # mm（最大最終半径、速度制限により制約）

# 衝突チェック定数（座標単位、ロボットサイズを考慮）
COLLISION_SAFE_DISTANCE = ROBOT_SIZE_COORD * 2.5   # 約137.25座標単位（最小安全距離、ロボットサイズの2.5倍）
COLLISION_WARNING_DISTANCE = ROBOT_SIZE_COORD * 3.0  # 約164.70座標単位（警告閾値、ロボットサイズの3倍）
COLLISION_URGENT_DISTANCE = ROBOT_SIZE_COORD * 1.5   # 約82.35座標単位（緊急閾値、ロボットサイズの1.5倍）

# 出力ディレクトリ
OUTPUT_DIR = Path("output")
NUM_ROBOTS = 30

# グローバル変数: 動作7の移動ベクトル（全ロボット共通）
# これは動作5終了後の長方形の中心からフィールド中心への移動ベクトル
MOVE_TO_CENTER_VECTOR = None  # (dx, dy) のタプル


# ===== ヘルパー関数 =====

def normalize_angle(angle_deg: float) -> float:
    """角度を0°～360°に正規化"""
    return angle_deg % 360.0


def get_initial_position(robot_id: int) -> Tuple[float, float, float]:
    """
    ロボットIDから初期位置を計算
    
    Args:
        robot_id: ロボットID (0-29)
    
    Returns:
        (x, y, rotation): 初期座標（mm）と向き（度）
    """
    col = robot_id % GRID_COLS
    row = robot_id // GRID_COLS
    # 新しいグリッド間隔と開始位置を使用
    x = GRID_START_X + col * GRID_COL_SPACING
    y = GRID_START_Y + row * GRID_ROW_SPACING
    rotation = 180.0  # 下向き
    return x, y, rotation


# ===== 最初のロコモーション（0-30秒）の計算 =====

def calculate_first_locomotion_frame(robot_id: int, t: float) -> Dict[str, Any]:
    """
    最初のロコモーション（0-30秒）のフレームを計算
    
    Args:
        robot_id: ロボットID (0-29)
        t: 時刻（秒）
    
    Returns:
        JSONフレーム辞書
    """
    x_init, y_init, rotation_init = get_initial_position(robot_id)
    
    # 現在の状態
    x, y = x_init, y_init
    rotation = rotation_init
    use_position = False
    use_rotation = False
    
    # 各動作の状態を累積的に計算
    # 動作1: 初期ポジション (0.0秒)
    if t == 0.0:
        x, y = x_init, y_init
        rotation = rotation_init
        use_position = True  # 初期位置を設定
        use_rotation = True  # 初期向きを設定
    
    # 動作2: 回転運動（90°、0.0-0.5秒、停止3.5秒まで）
    elif 0.0 < t <= 0.5:  # 0.5秒を含む（回転運動中）
        elapsed = t - 0.0
        rotation = normalize_angle(rotation_init + 90.0 * (elapsed / 0.5))
        x, y = x_init, y_init  # 位置は保持
        use_position = False  # 回転運動中は位置を更新しない
        use_rotation = True   # 回転のみ
    elif 0.5 < t < 3.5:  # 停止中（0.5秒と3.5秒は境界なので除外）
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        x, y = x_init, y_init  # 位置は保持
        use_position = False  # 停止中
        use_rotation = False  # 停止中
    elif t == 3.5:  # 並行移動開始時刻
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        x, y = x_init, y_init  # 位置は保持（移動開始前）
        use_position = True   # 並行移動開始
        use_rotation = False  # 回転はしない
    
    # 動作3: 並行移動（270°方向 = 左方向、400mm、3.5-5.5秒、停止9.25秒まで）
    # toio座標系: 270° = 左向き（X軸負方向）
    elif 3.5 < t <= 5.5:  # 5.5秒を含む（並行移動中、400mm ÷ 200mm/sec = 2.0秒）
        elapsed = t - 3.5
        distance = STRAIGHT_SPEED * elapsed
        # 270°方向の移動: X負方向
        x = x_init - distance  # 左方向に移動
        rotation = normalize_angle(rotation_init + 90.0)  # 270°を保持
        use_position = True   # 並行移動中は位置を更新
        use_rotation = False  # 回転はしない
    elif 5.5 < t < 9.25:  # 停止中（5.5秒と9.25秒は境界なので除外）
        x = x_init - PARALLEL_MOVE_DISTANCE  # 左方向に400mm移動
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = False  # 停止中
        use_rotation = False  # 停止中
    elif t == 9.25:  # 長方形移動開始時刻
        x = x_init - PARALLEL_MOVE_DISTANCE  # 前の並行移動後の位置
        y = y_init
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = True   # 長方形移動開始
        use_rotation = False  # 回転はしない
    
    # 動作4: 長方形移動（下方向、500mm、9.25-11.75秒、停止14.75秒まで）
    # toio座標系: 270°方向のまま移動、でも下方向に移動（180°方向）
    elif 9.25 < t <= 11.75:  # 11.75秒を含む（長方形移動中）
        elapsed = t - 9.25
        distance = STRAIGHT_SPEED * elapsed
        x = x_init - PARALLEL_MOVE_DISTANCE  # 前の並行移動後の位置（左方向）
        y = y_init + distance  # 下方向に移動
        rotation = normalize_angle(rotation_init + 90.0)  # 270°を保持
        use_position = True   # 並行移動中は位置を更新
        use_rotation = False  # 回転はしない
    elif 11.75 < t < 14.75:  # 停止中（11.75秒と14.75秒は境界なので除外）
        x = x_init - PARALLEL_MOVE_DISTANCE
        y = y_init + RECTANGULAR_MOVE_DISTANCE
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = False  # 停止中
        use_rotation = False  # 停止中
    elif t == 14.75:  # 初期ポジションに戻る開始時刻
        x = x_init - PARALLEL_MOVE_DISTANCE
        y = y_init + RECTANGULAR_MOVE_DISTANCE
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = True   # 並行移動開始
        use_rotation = False  # 回転はしない
    
    # 動作5: 初期ポジションに戻る（14.75-17.5秒、停止20.5秒まで）
    # X方向: 左方向550mmから右方向550mmで初期位置に戻る
    # Y方向: 下方向500mmから上方向500mmで初期位置に戻る
    elif 14.75 < t <= 17.5:  # 17.5秒を含む（並行移動中）
        elapsed = t - 14.75
        distance = STRAIGHT_SPEED * elapsed
        # X方向: 左方向に移動した位置から右方向に戻る
        x = x_init - PARALLEL_MOVE_DISTANCE + distance  # 右方向に移動して戻る
        # Y方向: 下方向に移動した位置から上方向に戻る
        y = y_init + RECTANGULAR_MOVE_DISTANCE - distance  # 上方向に移動して戻る
        rotation = normalize_angle(rotation_init + 90.0)  # 270°を保持
        use_position = True   # 並行移動中は位置を更新
        use_rotation = False  # 回転はしない
    elif 17.5 < t < 20.5:  # 停止中（17.5秒と20.5秒は境界なので除外）
        x = x_init  # 初期ポジションに戻る（X座標）
        y = y_init  # 初期ポジションに戻る（Y座標）
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = False  # 停止中
        use_rotation = False  # 停止中
    elif t == 20.5:  # 135°回転開始時刻
        x = x_init  # 初期位置に戻っている
        y = y_init  # 初期位置に戻っている
        rotation = normalize_angle(rotation_init + 90.0)  # 270°
        use_position = False  # 回転運動開始
        use_rotation = True   # 回転のみ
    
    # 動作6: 135°回転（20.5-21.25秒）
    elif 20.5 < t <= 21.25:  # 21.25秒を含む（回転運動中）
        elapsed = t - 20.5
        rotation_before = normalize_angle(rotation_init + 90.0)  # 270°
        rotation = normalize_angle(rotation_before + 135.0 * (elapsed / 0.75))
        x = x_init  # 初期位置に戻っている
        y = y_init  # 初期位置に戻っている
        use_position = False  # 回転運動中は位置を更新しない
        use_rotation = True   # 回転のみ
    
    # 動作7: 中心への移動（長方形の中心がフィールド中心に来るように、21.25-30.0秒）
    # 20.5秒から30秒で初期位置から中心に向かって移動（斜め45度方向を想定）
    # 動作6終了後（21.25秒）から中心への移動を開始
    elif 21.25 < t < 30.0:  # 30.0秒は境界なので除外
        elapsed = t - 21.25  # 動作6終了後からの経過時間
        # 動作5終了後の位置（20.5秒時点で初期位置と同じ位置に戻っている）
        # 動作5で初期位置に戻るため、x_init, y_initに戻る
        x_after_action5 = x_init
        y_after_action5 = y_init  # 初期位置に戻る
        
        # 初期位置から中心への移動ベクトルを計算
        # 各ロボットの初期位置から、フィールド中心方向への移動ベクトル
        # 長方形の中心がフィールド中心に来るように移動
        dx_to_center = CENTER_OFFSET_X  # 長方形の中心からフィールド中心へのX差分
        dy_to_center = CENTER_OFFSET_Y  # 長方形の中心からフィールド中心へのY差分
        
        # 各ロボットは初期位置から、長方形の中心の移動と同期して移動
        # 長方形の形状を維持しながら中心に移動
        total_move_time = 30.0 - 21.25  # 8.75秒
        move_distance = STRAIGHT_SPEED * elapsed
        total_move_distance = STRAIGHT_SPEED * total_move_time  # 1750mm（最大）
        
        # 中心への移動距離
        center_distance = math.sqrt(dx_to_center**2 + dy_to_center**2)  # 50mm（X方向のみ）
        move_ratio = min(1.0, move_distance / center_distance) if center_distance > 0.1 else 1.0
        
        # 各ロボットは初期位置から、長方形の中心の移動と同期して移動
        x = x_after_action5 + dx_to_center * move_ratio
        y = y_after_action5 + dy_to_center * move_ratio
        
        # フィールド制限を適用
        x = max(FIELD_MIN_X, min(FIELD_MAX_X, x))
        y = max(FIELD_MIN_Y, min(FIELD_MAX_Y, y))
        
        # 移動方向をtoio座標系の角度に変換（中心方向）
        if abs(dx_to_center) < 0.1:  # ほぼ垂直方向
            rotation = 0.0 if dy_to_center < 0 else 180.0
        else:
            angle_rad = math.atan2(-dy_to_center, dx_to_center)  # toio座標系ではY軸が下方向なので-dy
            rotation = normalize_angle(math.degrees(angle_rad) + 90.0)
        
        use_position = True   # 位置移動中
        use_rotation = False  # 回転はしない（位置のみ移動）
    
    # 30.0秒時点: 中心に到達
    elif t == 30.0:
        # 動作5終了後の位置から中心への移動を完了（初期位置から中心へ）
        x_after_action5 = x_init
        y_after_action5 = y_init  # 初期位置に戻っている
        
        dx_to_center = CENTER_OFFSET_X
        dy_to_center = CENTER_OFFSET_Y
        
        x = x_after_action5 + dx_to_center
        y = y_after_action5 + dy_to_center
        
        # フィールド制限を適用
        x = max(FIELD_MIN_X, min(FIELD_MAX_X, x))
        y = max(FIELD_MIN_Y, min(FIELD_MAX_Y, y))
        
        # 移動方向をtoio座標系の角度に変換
        if abs(dx_to_center) < 0.1:
            rotation = 0.0 if dy_to_center < 0 else 180.0
        else:
            angle_rad = math.atan2(-dy_to_center, dx_to_center)
            rotation = normalize_angle(math.degrees(angle_rad) + 90.0)
        
        # 動作8: 45°回転して180°に戻す（30秒時点で完了）
        # 30秒時点で中心に到達し、その後45°回転して180°に戻す
        rotation = normalize_angle(180.0)  # 180°に戻す
        
        use_position = True   # 位置移動完了
        use_rotation = True   # 回転も完了（180°）
    
    # 30秒以降は螺旋運動に移行（別の関数で処理）
    else:
        # 30秒以降は処理しない（この関数は0-30秒のみを処理）
        pass
    
    # フィールド境界チェック（座標を範囲内に制限）
    x = max(FIELD_MIN_X, min(FIELD_MAX_X, x))
    y = max(FIELD_MIN_Y, min(FIELD_MAX_Y, y))
    
    # 時間の丸め処理（重要な時刻を正確に保持）
    # 重要な時刻のリスト
    important_times = [0.0, 0.5, 3.5, 6.25, 9.25, 11.75, 14.75, 17.5, 20.5, 21.25, 24.25, 25.5, 28.5, 28.75, 30.0]
    
    time_rounded = t
    # 重要な時刻に近い場合は正確に保持
    for important_time in important_times:
        if abs(t - important_time) < 0.001:
            time_rounded = important_time
            break
    else:
        # それ以外は小数点以下1桁に丸める
        time_rounded = round(t, 1)
    
    return {
        "time": time_rounded,
        "use_position": use_position,
        "use_rotation": use_rotation,
        "position": {
            "x": round(x, 1),
            "y": round(y, 1)
        },
        "rotation": round(normalize_angle(rotation), 1),
        "sound": None
    }


# ===== 螺旋運動の衝突チェック =====

def check_collisions_at_time(t: float, robot_positions: Dict[int, Tuple[float, float]]) -> List[Tuple[int, int, float]]:
    """
    指定時刻での全ロボット間の衝突を検出
    
    Args:
        t: 時刻（秒）
        robot_positions: ロボットID → (x, y) の辞書
    
    Returns:
        [(robot_i, robot_j, distance), ...] のリスト（距離が閾値未満のペア）
    """
    collisions = []
    robot_ids = list(robot_positions.keys())
    
    for i in range(len(robot_ids)):
        for j in range(i + 1, len(robot_ids)):
            robot_i = robot_ids[i]
            robot_j = robot_ids[j]
            
            x_i, y_i = robot_positions[robot_i]
            x_j, y_j = robot_positions[robot_j]
            
            # 距離を計算
            dx = x_j - x_i
            dy = y_j - y_i
            distance = math.sqrt(dx**2 + dy**2)
            
            # 衝突チェック（閾値未満のペアを検出）
            if distance < COLLISION_SAFE_DISTANCE:
                collisions.append((robot_i, robot_j, distance))
    
    return collisions


def adjust_spiral_params_for_collision(
    robot_id: int, 
    collision_pairs: List[Tuple[int, int, float]], 
    all_params: Dict[int, Tuple[float, float, float, float, float]],
    iteration: int = 0
) -> Tuple[float, float, float, float, float]:
    """
    衝突を回避するために螺旋パラメータを調整（改善版）
    
    Args:
        robot_id: ロボットID
        collision_pairs: 衝突ペアリスト
        all_params: 全ロボットのパラメータ辞書
        iteration: 反復回数（調整の強度を変える）
    
    Returns:
        調整後のパラメータ (r0, theta0_deg, r_max, a, omega_deg_per_sec)
    """
    r0, theta0_deg, r_max, a, omega = all_params[robot_id]
    
    # このロボットが衝突しているペアを特定
    collision_partners = []
    min_collision_dist = float('inf')
    collision_angles = []  # 衝突方向の角度を記録
    
    for i, j, dist in collision_pairs:
        if i == robot_id:
            partner_id = j
            collision_partners.append(partner_id)
            min_collision_dist = min(min_collision_dist, dist)
            # 衝突方向を計算（回避方向の計算に使用）
            if partner_id in all_params:
                _, theta_partner, _, _, _ = all_params[partner_id]
                # 衝突相手との角度差を計算
                angle_diff = normalize_angle(theta0_deg - theta_partner)
                collision_angles.append(angle_diff)
        elif j == robot_id:
            partner_id = i
            collision_partners.append(partner_id)
            min_collision_dist = min(min_collision_dist, dist)
            if partner_id in all_params:
                _, theta_partner, _, _, _ = all_params[partner_id]
                angle_diff = normalize_angle(theta0_deg - theta_partner)
                collision_angles.append(angle_diff)
    
    if not collision_partners:
        # 衝突していない場合は元のパラメータを返す
        return (r0, theta0_deg, r_max, a, omega)
    
    # 反復回数に応じて調整の強度を大幅に強化
    angle_adjustment_factor = 1.0 + iteration * 1.5  # 反復ごとに調整を強化（1.5倍、2倍、2.5倍...）
    a_adjustment_factor = 1.0 + iteration * 0.5
    omega_adjustment_factor = 1.0 + iteration * 0.8
    
    # 衝突距離に応じて角度オフセットを決定（距離が近いほど大きく調整）
    if min_collision_dist < COLLISION_URGENT_DISTANCE:
        # 緊急衝突: 非常に大きな角度オフセット（±30-45度）
        base_angle = 30.0 + iteration * 5.0
        angle_adjustment = ((robot_id * 17.3 + iteration * 7.7) % (base_angle * 2) - base_angle) * angle_adjustment_factor
        # 衝突方向に基づいて回避方向を決定
        if collision_angles:
            avg_collision_angle = sum(collision_angles) / len(collision_angles)
            # 衝突方向から離れる方向に調整
            avoidance_angle = normalize_angle(avg_collision_angle + 180.0)
            angle_adjustment += (avoidance_angle - theta0_deg) * 0.3
    else:
        # 通常の衝突: 中程度の角度オフセット（±15-25度）
        base_angle = 15.0 + iteration * 3.0
        angle_adjustment = ((robot_id * 13.7 + iteration * 6.1) % (base_angle * 2) - base_angle) * angle_adjustment_factor
        if collision_angles:
            avg_collision_angle = sum(collision_angles) / len(collision_angles)
            avoidance_angle = normalize_angle(avg_collision_angle + 180.0)
            angle_adjustment += (avoidance_angle - theta0_deg) * 0.2
    
    theta0_deg_adjusted = normalize_angle(theta0_deg + angle_adjustment)
    
    # 半径拡大加速度を調整（衝突を避けるために拡散を促進）
    # 衝突が多い場合は加速度を増やす（±10-20%）
    a_adjustment = 1.0 + ((robot_id * 17.3 + iteration * 9.1) % 40.0 - 10.0) / 100.0 * a_adjustment_factor
    a_adjusted = max(0.1, a * a_adjustment)  # 最小値を確保
    
    # 角速度も調整（衝突を避けるために角速度を変える、±3-5deg/sec）
    omega_adjustment = ((robot_id * 5.7 + iteration * 3.1) % 10.0 - 5.0) * omega_adjustment_factor
    omega_adjusted = max(10.0, min(50.0, omega + omega_adjustment))  # 範囲制限
    
    # 最終半径も調整（衝突を避けるために拡散距離を増やす）
    r_max_adjustment = 1.0 + ((robot_id * 7.3 + iteration * 4.1) % 20.0 - 5.0) / 100.0
    r_max_adjusted = min(R_MAX_MAX, r_max * r_max_adjustment)
    
    return (r0, theta0_deg_adjusted, r_max_adjusted, a_adjusted, omega_adjusted)


def validate_spiral_collisions(all_params: Dict[int, Tuple[float, float, float, float, float]]) -> Dict[int, Tuple[float, float, float, float, float]]:
    """
    螺旋運動の全時刻で衝突をチェックし、必要に応じてパラメータを調整（改善版）
    
    Args:
        all_params: 全ロボットのパラメータ辞書
    
    Returns:
        調整後のパラメータ辞書
    """
    adjusted_params = all_params.copy()
    max_iterations = 5  # 反復回数を増やす
    
    total_collisions_by_iteration = []
    
    for iteration in range(max_iterations):
        collisions_found = False
        total_collisions = 0
        urgent_collisions = 0
        
        # 30.0秒から50.0秒まで0.2秒間隔でチェック（より細かくチェック）
        t = SPIRAL_START_TIME
        check_interval = 0.2  # 0.5秒間隔から0.2秒間隔に変更
        
        while t <= SPIRAL_END_TIME:
            # 全ロボットの位置を計算
            robot_positions = {}
            for robot_id in range(NUM_ROBOTS):
                params = adjusted_params[robot_id]
                frame = calculate_spiral_frame(robot_id, t, params)
                robot_positions[robot_id] = (frame["position"]["x"], frame["position"]["y"])
            
            # 衝突を検出
            collisions = check_collisions_at_time(t, robot_positions)
            
            if collisions:
                collisions_found = True
                total_collisions += len(collisions)
                
                # 緊急衝突と通常衝突を分類
                urgent_pairs = []
                normal_pairs = []
                
                for robot_i, robot_j, distance in collisions:
                    if distance < COLLISION_URGENT_DISTANCE:
                        urgent_pairs.append((robot_i, robot_j, distance))
                        urgent_collisions += 1
                        if iteration == 0:  # 最初の反復のみ詳細表示
                            print(f"⚠️  緊急衝突: 時刻{t:.1f}秒、ロボット{robot_i:2d}と{robot_j:2d}、距離{distance:.1f}mm")
                    elif distance < COLLISION_SAFE_DISTANCE:
                        normal_pairs.append((robot_i, robot_j, distance))
                        if iteration == 0:
                            print(f"警告: 時刻{t:.1f}秒、ロボット{robot_i:2d}と{robot_j:2d}、距離{distance:.1f}mm")
                
                # 緊急衝突を優先的に処理
                all_pairs = urgent_pairs + normal_pairs
                
                # 衝突しているロボットのパラメータを調整
                adjusted_robots = set()
                for robot_i, robot_j, distance in all_pairs:
                    if robot_i not in adjusted_robots:
                        adjusted_params[robot_i] = adjust_spiral_params_for_collision(
                            robot_i, all_pairs, adjusted_params, iteration
                        )
                        adjusted_robots.add(robot_i)
                    if robot_j not in adjusted_robots:
                        adjusted_params[robot_j] = adjust_spiral_params_for_collision(
                            robot_j, all_pairs, adjusted_params, iteration
                        )
                        adjusted_robots.add(robot_j)
            
            t += check_interval
        
        total_collisions_by_iteration.append(total_collisions)
        
        # 反復結果を表示
        if iteration == 0 or total_collisions == 0:
            print(f"反復{iteration + 1}: 総衝突数={total_collisions}, 緊急衝突数={urgent_collisions}")
        
        # 衝突がなくなったら終了
        if not collisions_found:
            print(f"✓ 衝突解消完了（反復{iteration + 1}回目）")
            break
    
    # 最終結果を表示
    if total_collisions_by_iteration[-1] > 0:
        print(f"⚠️  警告: {total_collisions_by_iteration[-1]}件の衝突が残存しています")
    
    return adjusted_params


# ===== 螺旋運動（30-50秒）の計算 =====

def calculate_spiral_params(robot_id: int) -> Tuple[float, float, float, float, float]:
    """
    螺旋運動の初期パラメータを計算（中心からの距離に基づくバリエーション付き）
    
    Args:
        robot_id: ロボットID (0-29)
    
    Returns:
        (r0, theta0_deg, r_max, a, omega_deg_per_sec): 初期半径、初期角度、最終半径、拡大加速度、角速度
    """
    # 30秒時点での位置を計算（最初のロコモーションの最終位置）
    frame_30 = calculate_first_locomotion_frame(robot_id, 30.0)
    x_i = frame_30["position"]["x"]
    y_i = frame_30["position"]["y"]
    
    # 初期半径と角度（フィールド中心からの距離と角度）
    dx_i = x_i - FIELD_CENTER_X
    dy_i = y_i - FIELD_CENTER_Y
    r0_i = math.sqrt(dx_i**2 + dy_i**2)
    theta0_i_rad = math.atan2(dy_i, dx_i)
    theta0_i_deg = math.degrees(theta0_i_rad)
    theta0_i_deg = normalize_angle(theta0_i_deg)
    
    # 中心からの距離に基づいて分類してバリエーションを追加
    if r0_i < 150.0:
        # 中心近く: 角速度を小さく、半径拡大を緩やかに
        omega_base = 31.0 + (robot_id % 3) * 0.5  # 31-32deg/sec
        a_multiplier = 0.85 + (robot_id % 5) * 0.02  # 0.85-0.95倍
        angle_offset = ((robot_id * 7.3) % 4.0 - 2.0)  # ±2度
    elif r0_i < 300.0:
        # 中間距離: 基準値
        omega_base = 35.0 + ((robot_id % 3) - 1) * 0.5  # 34-36deg/sec
        a_multiplier = 1.0 + ((robot_id % 5) - 2) * 0.02  # 0.96-1.04倍
        angle_offset = ((robot_id * 7.3) % 10.0 - 5.0)  # ±5度
    else:
        # 外側: 角速度を大きく、半径拡大を急激に
        omega_base = 38.0 + (robot_id % 3) * 0.5  # 38-40deg/sec
        a_multiplier = 1.1 + (robot_id % 5) * 0.02  # 1.1-1.2倍
        angle_offset = ((robot_id * 7.3) % 16.0 - 8.0)  # ±8度
    
    # 初期角度にオフセットを追加
    theta0_i_deg = normalize_angle(theta0_i_deg + angle_offset)
    
    # 最終半径と拡大加速度
    r_max_i = min(R_BASE + (r0_i - R_MIN) * K, R_MAX_MAX)
    a_i = 2.0 * (r_max_i - r0_i) / 400.0 * a_multiplier  # 20秒で到達、加速度倍率を適用
    
    return r0_i, theta0_i_deg, r_max_i, a_i, omega_base


def calculate_spiral_frame(robot_id: int, t: float, params: Tuple[float, float, float, float, float]) -> Dict[str, Any]:
    """
    螺旋運動（30-50秒）の時刻tでのフレームを計算
    
    Args:
        robot_id: ロボットID (0-29)
        t: 時刻（秒）
        params: (r0, theta0_deg, r_max, a, omega_deg_per_sec)
    
    Returns:
        JSONフレーム辞書
    """
    r0, theta0_deg, r_max, a, omega_deg_per_sec = params
    
    # 時間の経過
    dt = t - SPIRAL_START_TIME
    
    # 半径の計算（加速的な線形拡大）
    r_t = r0 + a * dt**2 / 2.0
    
    # 回転角度の計算（反時計回り、個別の角速度を使用）
    theta_t_deg = theta0_deg + omega_deg_per_sec * dt
    theta_t_deg = normalize_angle(theta_t_deg)
    theta_t_rad = math.radians(theta_t_deg)
    
    # 位置の計算（極座標から直交座標へ）
    # toio座標系: X右方向、Y下方向
    # 標準極座標: 0度=右方向、90度=上方向
    # toio角度と標準極座標の変換: theta_toio = theta_standard + 90
    # なので、標準極座標で計算してから変換
    x_t = FIELD_CENTER_X + r_t * math.cos(theta_t_rad)
    y_t = FIELD_CENTER_Y + r_t * math.sin(theta_t_rad)
    
    # フィールド境界チェック（座標を範囲内に制限）
    x_t = max(FIELD_MIN_X, min(FIELD_MAX_X, x_t))
    y_t = max(FIELD_MIN_Y, min(FIELD_MAX_Y, y_t))
    
    # 向きの計算（接線方向 = 回転角度 + 90°）
    rotation_t = normalize_angle(theta_t_deg + 90.0)
    
    # 螺旋運動は回転しながら移動するので、両方true
    # （ただし、回転しながら移動する場合は両方trueが適切）
    return {
        "time": round(t, 1),
        "use_position": True,   # 螺旋運動中は位置を更新
        "use_rotation": True,   # 螺旋運動中は向きも更新（回転しながら移動）
        "position": {
            "x": round(x_t, 1),
            "y": round(y_t, 1)
        },
        "rotation": round(rotation_t, 1),
        "sound": None
    }


# ===== メイン処理 =====

def generate_robot_json(robot_id: int) -> List[Dict[str, Any]]:
    """
    ロボットIDから0-50秒のJSONフレームを生成
    
    Args:
        robot_id: ロボットID (0-29)
    
    Returns:
        JSONフレームのリスト
    """
    frames = []
    
    # Part 1: 最初のロコモーション（0-30秒）
    # 動作の開始・終了時刻を正確に記録し、動作中も適切な間隔でフレームを生成
    
    # 動作シーケンスの定義（開始時刻、終了時刻、動作タイプ）
    action_sequence = [
        (0.0, 0.0, "initial"),           # 初期ポジション
        (0.0, 0.5, "rotation"),          # 回転運動（90°）
        (0.5, 3.5, "pause"),             # 停止
        (3.5, 6.25, "straight"),         # 並行移動
        (6.25, 9.25, "pause"),           # 停止
        (9.25, 11.75, "straight"),       # 長方形移動
        (11.75, 14.75, "pause"),         # 停止
        (14.75, 17.5, "straight"),       # 初期ポジションに戻る
        (17.5, 20.5, "pause"),           # 停止
        (20.5, 21.25, "rotation"),       # 135°回転
        (21.25, 24.25, "pause"),         # 停止
        (24.25, 25.5, "straight"),       # 位置調整移動
        (25.5, 28.5, "pause"),           # 停止
        (28.5, 28.75, "rotation"),       # 45°回転
        (28.75, 30.0, "pause"),          # 停止
    ]
    
    # 各動作のフレームを生成（重複を避けるため、既に追加された時刻を記録）
    added_times = set()
    
    for start_time, end_time, action_type in action_sequence:
        if action_type == "initial":
            # 初期ポジションのみ
            if start_time not in added_times:
                frame = calculate_first_locomotion_frame(robot_id, start_time)
                frames.append(frame)
                added_times.add(start_time)
        elif action_type == "rotation":
            # 回転運動: 開始、中間、終了時刻を記録
            if start_time not in added_times:
                frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                frames.append(frame_start)
                added_times.add(start_time)
            # 中間時刻（動作の途中）
            mid_time = (start_time + end_time) / 2.0
            if mid_time not in added_times:
                frame_mid = calculate_first_locomotion_frame(robot_id, mid_time)
                frames.append(frame_mid)
                added_times.add(mid_time)
            if end_time not in added_times:
                frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                frames.append(frame_end)
                added_times.add(end_time)
        elif action_type == "straight":
            # 並行移動: 開始、中間、終了時刻を記録
            if start_time not in added_times:
                frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                frames.append(frame_start)
                added_times.add(start_time)
            # 中間時刻（動作の途中）
            mid_time = (start_time + end_time) / 2.0
            if mid_time not in added_times:
                frame_mid = calculate_first_locomotion_frame(robot_id, mid_time)
                frames.append(frame_mid)
                added_times.add(mid_time)
            if end_time not in added_times:
                frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                frames.append(frame_end)
                added_times.add(end_time)
        elif action_type == "pause":
            # 停止中: 開始、中間、終了時刻を記録（適切な間隔で）
            if start_time not in added_times:
                frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                frames.append(frame_start)
                added_times.add(start_time)
            # 停止中の間隔（1秒間隔）
            pause_interval = 1.0
            t = start_time + pause_interval
            while t < end_time:
                if t not in added_times:
                    frame = calculate_first_locomotion_frame(robot_id, t)
                    frames.append(frame)
                    added_times.add(t)
                t += pause_interval
            if end_time not in added_times:
                frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                frames.append(frame_end)
                added_times.add(end_time)
    
    # フレームを時間順にソート
    frames.sort(key=lambda f: f["time"])
    
    # Part 2: 螺旋運動（30-50秒）
    # 螺旋運動は連続的な動きなので、0.5秒間隔で生成
    # 注意: パラメータは衝突チェック後に調整される可能性があるため、
    # generate_all_robots_json()で全ロボットのパラメータを事前に計算・検証する
    # ここでは一時的にパラメータを計算（後で上書きされる可能性がある）
    params = calculate_spiral_params(robot_id)
    t = 30.5
    while t <= SPIRAL_END_TIME:
        frame = calculate_spiral_frame(robot_id, t, params)
        frames.append(frame)
        t += FRAME_INTERVAL
    
    return frames


def generate_all_robots_json() -> Dict[int, List[Dict[str, Any]]]:
    """
    全ロボットのJSONフレームを生成（衝突チェック付き）
    
    Returns:
        ロボットID → フレームリストの辞書
    """
    global MOVE_TO_CENTER_VECTOR
    
    # 動作7の移動ベクトルを事前に計算
    calculate_move_to_center_vector()
    
    # 全ロボットの螺旋パラメータを計算
    all_spiral_params = {}
    for robot_id in range(NUM_ROBOTS):
        all_spiral_params[robot_id] = calculate_spiral_params(robot_id)
    
    # 衝突チェックとパラメータ調整
    print("\n螺旋運動の衝突チェックを実行中...")
    all_spiral_params = validate_spiral_collisions(all_spiral_params)
    print("衝突チェック完了\n")
    
    # 全ロボットのJSONフレームを生成
    all_frames = {}
    for robot_id in range(NUM_ROBOTS):
        frames = []
        
        # Part 1: 最初のロコモーション（0-30秒）
        # 動作シーケンスの定義（開始時刻、終了時刻、動作タイプ）
        action_sequence = [
            (0.0, 0.0, "initial"),           # 初期ポジション
            (0.0, 0.5, "rotation"),          # 回転運動（90°）
            (0.5, 3.5, "pause"),             # 停止
            (3.5, 6.25, "straight"),         # 並行移動
            (6.25, 9.25, "pause"),           # 停止
            (9.25, 11.75, "straight"),       # 長方形移動
            (11.75, 14.75, "pause"),         # 停止
            (14.75, 17.5, "straight"),       # 初期ポジションに戻る
            (17.5, 20.5, "pause"),           # 停止
            (20.5, 21.25, "rotation"),       # 135°回転
            (21.25, 23.0, "pause"),          # 停止（短縮）
            (23.0, 26.5, "straight"),        # 中心への移動（延長: 3.5秒、700mmまで可能）
            (26.5, 28.5, "pause"),           # 停止
            (28.5, 28.75, "rotation"),       # 45°回転
            (28.75, 30.0, "pause"),          # 停止
        ]
        
        # 各動作のフレームを生成（重複を避けるため、既に追加された時刻を記録）
        added_times = set()
        
        for start_time, end_time, action_type in action_sequence:
            if action_type == "initial":
                if start_time not in added_times:
                    frame = calculate_first_locomotion_frame(robot_id, start_time)
                    frames.append(frame)
                    added_times.add(start_time)
            elif action_type == "rotation":
                if start_time not in added_times:
                    frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                    frames.append(frame_start)
                    added_times.add(start_time)
                mid_time = (start_time + end_time) / 2.0
                if mid_time not in added_times:
                    frame_mid = calculate_first_locomotion_frame(robot_id, mid_time)
                    frames.append(frame_mid)
                    added_times.add(mid_time)
                if end_time not in added_times:
                    frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                    frames.append(frame_end)
                    added_times.add(end_time)
            elif action_type == "straight":
                if start_time not in added_times:
                    frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                    frames.append(frame_start)
                    added_times.add(start_time)
                mid_time = (start_time + end_time) / 2.0
                if mid_time not in added_times:
                    frame_mid = calculate_first_locomotion_frame(robot_id, mid_time)
                    frames.append(frame_mid)
                    added_times.add(mid_time)
                if end_time not in added_times:
                    frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                    frames.append(frame_end)
                    added_times.add(end_time)
            elif action_type == "pause":
                if start_time not in added_times:
                    frame_start = calculate_first_locomotion_frame(robot_id, start_time)
                    frames.append(frame_start)
                    added_times.add(start_time)
                pause_interval = 1.0
                t = start_time + pause_interval
                while t < end_time:
                    if t not in added_times:
                        frame = calculate_first_locomotion_frame(robot_id, t)
                        frames.append(frame)
                        added_times.add(t)
                    t += pause_interval
                if end_time not in added_times:
                    frame_end = calculate_first_locomotion_frame(robot_id, end_time)
                    frames.append(frame_end)
                    added_times.add(end_time)
        
        # フレームを時間順にソート
        frames.sort(key=lambda f: f["time"])
        
        # Part 2: 螺旋運動（30-50秒）
        # 衝突チェック済みのパラメータを使用
        spiral_params = all_spiral_params[robot_id]
        t = 30.5
        while t <= SPIRAL_END_TIME:
            frame = calculate_spiral_frame(robot_id, t, spiral_params)
            frames.append(frame)
            t += FRAME_INTERVAL
        
        all_frames[robot_id] = frames
    
    return all_frames


def calculate_move_to_center_vector() -> Tuple[float, float]:
    """
    動作5終了後の長方形の中心を計算し、フィールド中心への移動ベクトルを返す
    30秒時点で形成の中心がフィールド中心に来るように計算
    
    Returns:
        (dx, dy): フィールド中心への移動ベクトル
    """
    global MOVE_TO_CENTER_VECTOR
    
    # 動作5終了後（20.5秒時点）の全ロボットの位置を正確に計算
    # calculate_first_locomotion_frameを使って実際の位置を計算（フィールド制限を含む）
    robot_positions_20_5 = []
    for robot_id in range(NUM_ROBOTS):
        frame_20_5 = calculate_first_locomotion_frame(robot_id, 20.5)
        x_after = frame_20_5["position"]["x"]
        y_after = frame_20_5["position"]["y"]
        robot_positions_20_5.append((robot_id, x_after, y_after))
    
    # 長方形の中心を計算（平均座標）
    center_x_20_5 = sum(x for _, x, y in robot_positions_20_5) / len(robot_positions_20_5)
    center_y_20_5 = sum(y for _, x, y in robot_positions_20_5) / len(robot_positions_20_5)
    
    # 30秒時点で形成の中心がフィールド中心に来るようにする
    # 各ロボットの相対位置を保持したまま、形成全体を移動
    target_center_x = FIELD_CENTER_X
    target_center_y = FIELD_CENTER_Y
    
    # 移動ベクトルを計算
    dx_formation = target_center_x - center_x_20_5
    dy_formation = target_center_y - center_y_20_5
    
    # 動作7の移動時間と速度を考慮（23.0-26.5秒、200mm/sec）
    movement_time = 26.5 - 23.0  # 3.5秒
    max_movement_distance = STRAIGHT_SPEED * movement_time  # 700mm
    
    # 必要な移動距離
    required_distance = math.sqrt(dx_formation**2 + dy_formation**2)
    
    # 移動可能距離を超える場合は、最大距離まで移動
    if required_distance > max_movement_distance:
        scale = max_movement_distance / required_distance if required_distance > 0.1 else 1.0
        dx = dx_formation * scale
        dy = dy_formation * scale
        print(f"警告: 移動距離が不足します。必要な距離: {required_distance:.1f}mm, 可能な距離: {max_movement_distance:.1f}mm")
        print(f"  スケール: {scale:.3f} で調整します")
    else:
        dx = dx_formation
        dy = dy_formation
    
    print(f"動作5終了後の長方形の中心: ({center_x_20_5:.1f}, {center_y_20_5:.1f})")
    print(f"フィールド中心: ({FIELD_CENTER_X}, {FIELD_CENTER_Y})")
    print(f"形成移動ベクトル: ({dx_formation:.1f}, {dy_formation:.1f}), 距離: {required_distance:.1f}mm")
    print(f"適用移動ベクトル: ({dx:.1f}, {dy:.1f}), 距離: {math.sqrt(dx**2 + dy**2):.1f}mm")
    
    MOVE_TO_CENTER_VECTOR = (dx, dy)
    return (dx, dy)


def main():
    """メイン処理: 30台分のJSONファイルを生成"""
    # 出力ディレクトリを作成
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    print(f"JSONファイル生成を開始します（{NUM_ROBOTS}台、0-50秒）...")
    
    # 全ロボットのJSONフレームを生成（衝突チェック付き）
    all_frames = generate_all_robots_json()
    
    # JSONファイルに出力
    for robot_id in range(NUM_ROBOTS):
        frames = all_frames[robot_id]
        output_file = OUTPUT_DIR / f"robot_{robot_id:02d}.json"
        with open(output_file, "w", encoding="utf-8") as f:
            json.dump({"frames": frames}, f, indent=2, ensure_ascii=False)
        
        print(f"  {output_file}: {len(frames)}フレーム生成完了")
    
    print(f"\n完了: {NUM_ROBOTS}台分のJSONファイルを生成しました")
    print(f"出力ディレクトリ: {OUTPUT_DIR.absolute()}")
    
    # 衝突チェック結果のサマリーを出力
    collision_summary_file = Path("docs/collision_check_summary.md")
    collision_summary_file.parent.mkdir(exist_ok=True)
    with open(collision_summary_file, "w", encoding="utf-8") as f:
        f.write("# 螺旋運動の衝突チェック結果\n\n")
        f.write(f"生成日時: {Path(__file__).stat().st_mtime}\n\n")
        f.write("## 衝突チェック設定\n\n")
        f.write(f"- 最小安全距離: {COLLISION_SAFE_DISTANCE}mm\n")
        f.write(f"- 警告閾値: {COLLISION_WARNING_DISTANCE}mm\n")
        f.write(f"- 緊急閾値: {COLLISION_URGENT_DISTANCE}mm\n\n")
        f.write("## 結果\n\n")
        f.write("衝突チェックは実行されました。詳細はコンソール出力を参照してください。\n")


if __name__ == "__main__":
    main()

