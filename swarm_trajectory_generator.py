#!/usr/bin/env python3
"""
50-120秒の群ロボット動作の軌跡生成器（オフライン生成）

生命らしさを厳密に定義した数理モデル：
- 相関ランダムウォーク（Correlated Random Walk）
- 恒常性（Homeostasis）システム
- エントロピーベースの収束防止
- 局所最小値からの脱出
- 個体差の導入

Boidモデルは使用しない。衝突回避と自律性のみで実現。
"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ============================================================================
# 型定義
# ============================================================================

Point = Tuple[float, float]  # (x, y) in mm
Heading = float  # 角度 in degrees [0, 360)


@dataclass
class RobotState:
    """ロボットの状態（厳密な定義）"""
    robot_id: int
    position: Point  # mm
    heading: Heading  # degrees
    velocity: Point  # mm/s (vx, vy)
    
    # 相関ランダムウォーク用の状態
    current_heading_rad: float = 0.0  # 現在の進行方向（ラジアン）
    heading_persistence_time: float = 0.0  # 方向を保持する残り時間（秒）
    last_heading_change_time: float = 0.0  # 最後に方向を変えた時刻（秒）
    
    # 恒常性（Homeostasis）システム
    homeostasis_energy: float = 0.5  # エネルギー値 [0, 1]
    homeostasis_target: float = 0.7  # 目標エネルギー値 [0, 1]
    homeostasis_decay_rate: float = 0.01  # エネルギー減衰率（秒^-1）
    homeostasis_threshold: float = 0.3  # 探索行動を促進する偏差閾値
    
    # エントロピー計算用
    position_history: List[Tuple[float, Point]] = field(default_factory=list)  # (time, position)
    entropy: float = 0.5  # エントロピー値 [0, 1]
    entropy_history: List[float] = field(default_factory=list)
    
    # 局所最小値検出
    last_position: Point = (0.0, 0.0)
    stuck_timer: float = 0.0  # 同じ場所に留まっている時間（秒）
    stuck_threshold: float = 2.0  # 2秒間動かなければ脱出モード
    
    # 個体差
    speed_category: str = "moderate"  # "fast", "moderate", "slow"
    preferred_speed: float = 150.0  # mm/s
    speed_bias: float = 1.0  # 個体差による速度倍率
    
    # 彷徨（Wandering）行動
    wandering_mode: bool = False
    wandering_timer: float = 0.0
    wandering_direction_rad: float = 0.0
    
    # 動的速度変化
    current_speed: float = 0.0  # 現在の実際の速度（段階的に変化）
    acceleration_rate: float = 50.0  # mm/s^2
    deceleration_rate: float = 60.0  # mm/s^2
    
    # 優先順位システム（衝突回避時の譲る動作）
    priority: float = 0.5  # 0.0-1.0、大きい方が優先
    base_preferred_speed: float = 0.0  # 元の好ましい速度（減速時に復元用）


@dataclass
class HumanZone:
    """人間領域の定義（時刻付き）"""
    start_time: float  # 開始時刻（秒）
    end_time: float  # 終了時刻（秒）
    center: Point  # 中心座標（mm）
    radius: float  # 半径（mm）
    strength: float  # ポテンシャル強度


# ============================================================================
# フィールド定数
# ============================================================================

FIELD_MIN_X = 34.0
FIELD_MAX_X = 949.0
FIELD_MIN_Y = 35.0
FIELD_MAX_Y = 898.0

ROBOT_LENGTH_MM = 72.0
ROBOT_WIDTH_MM = 32.0
ROBOT_DIAGONAL_MM = math.sqrt(ROBOT_LENGTH_MM**2 + ROBOT_WIDTH_MM**2)  # ≈ 78.8mm

# 安全マージン: 各ロボットの占有矩形を縦横 +10mm 拡張（20mmから10mmに縮小）
SAFETY_MARGIN_MM = 10.0
ROBOT_SAFE_RADIUS_MM = ROBOT_DIAGONAL_MM / 2.0 + SAFETY_MARGIN_MM  # ≈ 49.4mm

# 衝突回避の閾値（実際のサイズに合わせて調整）
EMERGENCY_STOP_DISTANCE_MM = ROBOT_DIAGONAL_MM  # 78.8mm（toioの対角線長）
MIN_SEPARATION_DISTANCE_MM = ROBOT_DIAGONAL_MM + 2.0  # 80.8mm（最小分離距離、5mmから2mmに縮小）
SAFE_DISTANCE_MM = 85.0  # 通常の安全距離（80mmから85mmに調整）

# 速度制限（ground_rules.mdより）
MAX_SPEED_MM_PER_SEC = 160.0  # 速度上限を抑える（200mm/sの80%）
DEFAULT_SPEED_MM_PER_SEC = 100.0  # デフォルト速度

# 境界バッファ（端っこに寄りすぎないように緩和）
BOUNDARY_BUFFER_MM = 15.0  # 通常の境界バッファ
# セーフティーゾーン（外側10mmは入らないようにする）
SAFETY_ZONE_MM = 10.0  # 外側10mmのセーフティーゾーン
# 境界からの反発力
BOUNDARY_REPULSION_DISTANCE_MM = 100.0  # 境界から100mm以内で反発力が働く
BOUNDARY_REPULSION_GAIN = 2000.0  # 境界反発力のゲイン

# ============================================================================
# 生命らしさのパラメータ（厳密な定義）
# ============================================================================

# 相関ランダムウォーク
PERSISTENCE_TIME_BASE = 5.0  # 方向を保持する時間（秒）
PERSISTENCE_ANGLE_STD = 0.25  # 角度変化の標準偏差（ラジアン）
SHARP_TURN_PROBABILITY = 0.01  # 急旋回の確率（1%）
SHARP_TURN_ANGLE = math.pi / 2  # 急旋回の角度範囲（±90度）

# 恒常性システム
HOMEOSTASIS_TARGET_BASE = 0.7  # 目標エネルギー値
HOMEOSTASIS_DECAY_BASE = 0.01  # エネルギー減衰率（秒^-1）
HOMEOSTASIS_THRESHOLD_BASE = 0.3  # 探索行動を促進する偏差閾値

# エントロピー計算
ENTROPY_HISTORY_SIZE = 10  # エントロピーの履歴サイズ
POSITION_HISTORY_SIZE = 30  # 位置履歴のサイズ（約0.5秒、0.5秒間隔想定）
STATIONARY_THRESHOLD_MM_PER_SEC = 30.0  # 静止判定の閾値（mm/s）
STATIONARY_TIME_THRESHOLD_SEC = 1.0  # 静止時間の閾値（秒）

# 速度分布（個体差）
SPEED_DISTRIBUTION = {
    "fast": 0.2,      # 20%が速い
    "moderate": 0.6,  # 60%が中程度
    "slow": 0.2       # 20%が遅い
}

SPEED_RANGES = {
    "fast": (120.0, 150.0),      # mm/s（適度な速度に調整）
    "moderate": (70.0, 100.0),  # mm/s（適度な速度に調整）
    "slow": (50.0, 70.0)        # mm/s（適度な速度に調整）
}

# 人間との相互作用
PREFERRED_DISTANCE_MIN_MM = 400.0  # 40cm
PREFERRED_DISTANCE_MAX_MM = 700.0  # 70cm
HUMAN_AVOIDANCE_DISTANCE_MM = 400.0  # 40cm以内は強制的に離れる
# 人間の動きの予測
HUMAN_PREDICTION_TIME = 4.0  # 4秒前から予測を開始
HUMAN_PREDICTION_STRENGTH_MIN = 0.3  # 予測時の最小強度（30%）
HUMAN_PREDICTION_STRENGTH_MAX = 1.0  # 実際の強度（100%）

# 彷徨行動
WANDERING_PROBABILITY = 0.005  # 彷徨モードに入る確率（フレームごと、0.5秒間隔想定）
WANDERING_DURATION_MIN = 2.0  # 彷徨継続時間の最小値（秒）
WANDERING_DURATION_MAX = 7.0  # 彷れ継続時間の最大値（秒）

# ============================================================================
# 数学的関数（厳密な定義）
# ============================================================================


def normalize_heading(deg: float) -> Heading:
    """角度を[0, 360)の範囲に正規化"""
    return deg % 360.0


def normalize_angle_rad(rad: float) -> float:
    """角度を[-π, π]の範囲に正規化"""
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def distance(p1: Point, p2: Point) -> float:
    """2点間の距離を計算（mm）"""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.hypot(dx, dy)


def clamp_position(pos: Point) -> Point:
    """
    位置をフィールド境界内にクリップ（境界バッファ + セーフティーゾーンを考慮）
    
    セーフティーゾーン: 外側10mmは入らないようにする
    """
    # 境界バッファ + セーフティーゾーン
    total_buffer = BOUNDARY_BUFFER_MM + SAFETY_ZONE_MM
    x = max(FIELD_MIN_X + total_buffer, 
            min(FIELD_MAX_X - total_buffer, pos[0]))
    y = max(FIELD_MIN_Y + total_buffer,
            min(FIELD_MAX_Y - total_buffer, pos[1]))
    return (x, y)


def clamp_position_loose(pos: Point) -> Point:
    """
    位置をフィールド境界内にクリップ（緩い境界バッファ、初期状態用）
    
    初期状態用: セーフティーゾーンのみ適用（10mm）
    """
    # 初期状態用はセーフティーゾーンのみ
    x = max(FIELD_MIN_X + SAFETY_ZONE_MM, 
            min(FIELD_MAX_X - SAFETY_ZONE_MM, pos[0]))
    y = max(FIELD_MIN_Y + SAFETY_ZONE_MM,
            min(FIELD_MAX_Y - SAFETY_ZONE_MM, pos[1]))
    return (x, y)


def gaussian_random() -> float:
    """ガウス乱数を生成（Box-Muller変換）"""
    u1 = random.random()
    u2 = random.random()
    return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def assign_speed_category(robot_id: int, total_count: int) -> str:
    """
    速度カテゴリを割り当て（20%速い、60%中程度、20%遅い）
    
    厳密な定義: ロボットIDに基づいて決定論的に割り当て
    """
    ratio = robot_id / total_count
    if ratio < 0.2:
        return "fast"
    elif ratio < 0.8:
        return "moderate"
    else:
        return "slow"


def calculate_preferred_speed(robot_id: int, speed_category: str) -> float:
    """
    好ましい速度を計算（個体差を含む）
    
    厳密な定義: 速度カテゴリとロボットIDに基づいて決定論的に計算
    """
    min_speed, max_speed = SPEED_RANGES[speed_category]
    
    # ロボットIDに基づいた決定論的な乱数生成
    rng = random.Random(robot_id)
    base_speed = min_speed + rng.random() * (max_speed - min_speed)
    
    # 個体差によるバイアス（0.85-1.15倍に縮小、速度を抑える）
    speed_bias = 0.85 + rng.random() * 0.3
    return base_speed * speed_bias


# ============================================================================
# 相関ランダムウォーク（Correlated Random Walk）
# ============================================================================


def update_correlated_random_walk(
    state: RobotState,
    dt: float,
    current_time: float,
) -> None:
    """
    相関ランダムウォークを更新
    
    厳密な定義:
    - 現在の進行方向を一定時間（persistence_time）保持
    - 時間が経過したら、前の方向から小さな角度変化（ガウス分布、標準偏差persistence_angle）
    - 急旋回は低確率（sharp_turn_probability）で±90度
    
    数学的定義:
    - θ(t+dt) = θ(t) + N(0, σ²) if persistence_time <= 0
    - θ(t+dt) = θ(t) if persistence_time > 0
    - ただし、急旋回の場合は θ(t+dt) = θ(t) + U(-π/2, π/2)
    """
    state.heading_persistence_time -= dt
    state.last_heading_change_time += dt
    
    # 急旋回チェック（低確率）
    should_sharp_turn = False
    if random.random() < SHARP_TURN_PROBABILITY * dt:
        should_sharp_turn = True
        turn_angle = (random.random() - 0.5) * 2 * SHARP_TURN_ANGLE
        state.current_heading_rad += turn_angle
        state.heading_persistence_time = PERSISTENCE_TIME_BASE * (1.5 + random.random() * 1.0)
        state.last_heading_change_time = 0.0
    
    # 方向を変更する必要があるかチェック
    if state.heading_persistence_time <= 0 and not should_sharp_turn:
        # 相関ランダムウォーク: 前の方向から小さな角度変化
        angle_change = gaussian_random() * PERSISTENCE_ANGLE_STD
        state.current_heading_rad += angle_change
        state.current_heading_rad = normalize_angle_rad(state.current_heading_rad)
        
        # 方向保持時間を更新（個体差を含む）
        persistence_variation = 0.5 + random.random() * 1.0  # 0.5-1.5倍
        state.heading_persistence_time = PERSISTENCE_TIME_BASE * persistence_variation
        state.last_heading_change_time = 0.0


# ============================================================================
# 恒常性（Homeostasis）システム
# ============================================================================


def calculate_entropy(
    state: RobotState,
    neighbors: List[RobotState],
    current_time: float,
) -> float:
    """
    エントロピーを計算（近傍ロボットの位置の分散度）
    
    厳密な定義:
    - エントロピー = 近傍ロボットの位置の分散度を正規化した値 [0, 1]
    - 分散度が高い = エントロピーが高い = 分散している
    - 分散度が低い = エントロピーが低い = 収束している
    
    数学的定義:
    - μ = (1/N) Σ p_i  (近傍ロボットの平均位置)
    - σ² = (1/N) Σ ||p_i - μ||²  (分散)
    - entropy = min(1.0, σ² / max_variance)
    """
    if not neighbors:
        return 0.5  # デフォルト値
    
    # 平均位置を計算
    sum_x = sum(n.position[0] for n in neighbors)
    sum_y = sum(n.position[1] for n in neighbors)
    avg_x = sum_x / len(neighbors)
    avg_y = sum_y / len(neighbors)
    
    # 分散を計算
    variance = 0.0
    for neighbor in neighbors:
        dx = neighbor.position[0] - avg_x
        dy = neighbor.position[1] - avg_y
        variance += dx * dx + dy * dy
    variance /= len(neighbors)
    
    # エントロピーを正規化（0-1の範囲）
    max_variance = 100000.0  # 最大分散（フィールドサイズに基づく）
    entropy = min(1.0, variance / max_variance)
    
    return entropy


def update_homeostasis(
    state: RobotState,
    neighbors: List[RobotState],
    dt: float,
    current_time: float,
) -> None:
    """
    恒常性（Homeostasis）システムを更新
    
    厳密な定義:
    - エネルギーが時間とともに減衰（homeostasis_decay_rate）
    - 目標値（homeostasis_target）からの偏差が閾値（homeostasis_threshold）を超えると探索行動を促進
    - エントロピーが低い（収束している）場合、パラメータを変更して収束を防ぐ
    
    数学的定義:
    - dE/dt = -decay_rate * E
    - if |E - target| > threshold: 探索行動を促進
    - if entropy < 0.3: パラメータをランダムに変更
    """
    # エントロピーを計算
    state.entropy = calculate_entropy(state, neighbors, current_time)
    state.entropy_history.append(state.entropy)
    if len(state.entropy_history) > ENTROPY_HISTORY_SIZE:
        state.entropy_history.pop(0)
    
    # 位置履歴を更新
    state.position_history.append((current_time, state.position))
    if len(state.position_history) > POSITION_HISTORY_SIZE:
        state.position_history.pop(0)
    
    # 静止検出
    if len(state.position_history) >= 2:
        oldest_time, oldest_pos = state.position_history[0]
        newest_time, newest_pos = state.position_history[-1]
        time_diff = newest_time - oldest_time
        
        if time_diff > 0.1:  # 0.1秒以上
            distance_moved = distance(oldest_pos, newest_pos)
            avg_speed = distance_moved / time_diff
            
            if avg_speed < STATIONARY_THRESHOLD_MM_PER_SEC:
                # 静止している
                state.stuck_timer += dt
                
                if state.stuck_timer > STATIONARY_TIME_THRESHOLD_SEC:
                    # 強制的に移動を促進
                    state.wandering_mode = True
                    state.wandering_timer = WANDERING_DURATION_MIN + random.random() * (
                        WANDERING_DURATION_MAX - WANDERING_DURATION_MIN
                    )
                    state.wandering_direction_rad = random.random() * 2 * math.pi
                    state.homeostasis_energy = 0.1  # エネルギーを急激に下げる
                    state.stuck_timer = 0.0
            else:
                state.stuck_timer = 0.0
    
    # エントロピーが低い場合（収束している）、パラメータを変更
    if len(state.entropy_history) > 0:
        avg_entropy = sum(state.entropy_history) / len(state.entropy_history)
        
        if avg_entropy < 0.3:
            # 収束を防ぐためにパラメータをランダムに変更
            variation = (random.random() - 0.5) * 0.1
            state.homeostasis_target = max(0.1, min(1.0, state.homeostasis_target + variation))
            state.homeostasis_decay_rate = max(0.001, min(0.02, state.homeostasis_decay_rate + variation * 0.001))
    
    # エネルギーを減衰
    state.homeostasis_energy -= state.homeostasis_decay_rate * dt
    state.homeostasis_energy = max(0.0, min(1.0, state.homeostasis_energy))
    
    # 目標値からの偏差を計算
    deviation = state.homeostasis_target - state.homeostasis_energy
    
    # 偏差が大きい場合は探索行動を促進
    if abs(deviation) > state.homeostasis_threshold:
        state.wandering_mode = True
        state.wandering_timer = WANDERING_DURATION_MIN + random.random() * (
            WANDERING_DURATION_MAX - WANDERING_DURATION_MIN
        )
        state.wandering_direction_rad = random.random() * 2 * math.pi
    
    # 彷徨中はエネルギーを回復
    if state.wandering_mode:
        recovery_rate = 0.01 + random.random() * 0.02
        state.homeostasis_energy += recovery_rate * dt
        state.homeostasis_energy = min(1.0, state.homeostasis_energy)


# ============================================================================
# 局所最小値からの脱出
# ============================================================================


def check_local_minimum(
    state: RobotState,
    dt: float,
) -> bool:
    """
    局所最小値に留まっているかチェック
    
    厳密な定義:
    - 2秒間20mm未満の移動で局所最小値と判定
    - 局所最小値の場合は強制的に方向を変更
    
    数学的定義:
    - if ||p(t) - p(t-2)|| < 20mm and t - t_last_move > 2.0: 局所最小値
    """
    move_distance = distance(state.position, state.last_position)
    
    if move_distance < 20.0:  # 20mm未満の移動
        state.stuck_timer += dt
    else:
        state.stuck_timer = 0.0
        state.last_position = state.position
    
    return state.stuck_timer > state.stuck_threshold


def escape_local_minimum(state: RobotState) -> None:
    """
    局所最小値から脱出
    
    厳密な定義:
    - 完全にランダムな方向に変更
    - 方向保持時間を長く設定（探索を促進）
    """
    state.current_heading_rad = random.random() * 2 * math.pi
    state.heading_persistence_time = PERSISTENCE_TIME_BASE * (1.5 + random.random() * 1.0)
    state.stuck_timer = 0.0
    state.last_position = state.position


# ============================================================================
# 人間領域のポテンシャル場
# ============================================================================


def compute_human_potential_gradient(
    position: Point,
    human_zones: List[HumanZone],
    current_time: float,
) -> Point:
    """
    人間領域のポテンシャル場の勾配を計算（4秒前から予測）
    
    厳密な定義:
    - 人間領域を負のポテンシャル（反発力）として定義
    - 4秒前から予測を開始し、時間が近づくにつれて強度を増やす
    - 予測時の強度は通常の30-100%の範囲で線形補間
    
    数学的定義:
    - φ(x, y) = Σ strength * exp(-||p - c|| / radius)
    - 予測時間: t_pred = zone.start_time - current_time
    - 予測強度: strength_pred = min(1.0, max(0.3, 1.0 - t_pred / 4.0))
    - ∇φ = Σ strength * strength_pred * exp(-||p - c|| / radius) * (p - c) / ||p - c||
    """
    gradient_x = 0.0
    gradient_y = 0.0
    
    for zone in human_zones:
        # 現在の時刻が人間領域の期間内か、4秒前から予測範囲内か
        prediction_start = zone.start_time - HUMAN_PREDICTION_TIME
        
        if not (prediction_start <= current_time < zone.end_time):
            continue
        
        # 予測強度を計算（時間が近づくにつれて強度を増やす）
        if current_time < zone.start_time:
            # 予測期間中: 時間が近づくにつれて強度を増やす（30% → 100%）
            time_until_start = zone.start_time - current_time
            prediction_ratio = 1.0 - (time_until_start / HUMAN_PREDICTION_TIME)
            prediction_ratio = max(0.0, min(1.0, prediction_ratio))  # 0.0-1.0にクランプ
            
            # 線形補間: 30% → 100%
            strength_multiplier = HUMAN_PREDICTION_STRENGTH_MIN + (
                prediction_ratio * (HUMAN_PREDICTION_STRENGTH_MAX - HUMAN_PREDICTION_STRENGTH_MIN)
            )
        else:
            # 実際の期間中: 通常の強度（100%）
            strength_multiplier = HUMAN_PREDICTION_STRENGTH_MAX
        
        dx = position[0] - zone.center[0]
        dy = position[1] - zone.center[1]
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            continue
        
        # ポテンシャルの勾配を計算（指数減衰）
        decay_factor = math.exp(-dist / (zone.radius * 0.3))
        # 予測強度を適用（やりすぎないように）
        strength = zone.strength * decay_factor * strength_multiplier
        
        gradient_x += strength * (dx / dist)
        gradient_y += strength * (dy / dist)
    
    return (gradient_x, gradient_y)


# ============================================================================
# 衝突回避システム
# ============================================================================


def compute_priority(robot_id: int) -> float:
    """
    優先順位を計算（0.0-1.0、大きい方が優先）
    
    方法: ロボットIDに基づいた決定論的乱数
    - 同じIDなら常に同じ優先度
    - デッドロックを防ぐためにランダム性を持たせる
    """
    rng = random.Random(robot_id)
    return rng.random()


def compute_collision_avoidance_force(
    state: RobotState,
    other_robots: List[RobotState],
) -> Point:
    """
    優先順位を考慮した衝突回避力を計算（多層的）
    
    厳密な定義:
    1. 優先順位が低いロボット（譲る側）: 速度を下げる + 回避方向に移動
    2. 優先順位が高いロボット（優先側）: 通常の反発力
    3. 緊急停止（71mm以下）: 強力な反発力 + 減速
    
    数学的定義:
    - F_repulsion = Σ (gain * (1/dist - 1/safe_dist) * (p_i - p_j) / ||p_i - p_j||)
    - 優先度が低い場合: F_yield = F_repulsion * 0.8, v_preferred *= 0.5
    - 緊急停止時: F_emergency = F_repulsion * 2.0, v_preferred *= 0.15
    """
    force_x = 0.0
    force_y = 0.0
    
    REPULSION_GAIN = 6000.0
    SEPARATION_FORCE = 3000.0
    
    # 元の速度を保存（初回のみ）
    if state.base_preferred_speed == 0.0:
        state.base_preferred_speed = state.preferred_speed
    
    # 速度を復元（衝突がない場合は元の速度に戻す）
    state.preferred_speed = state.base_preferred_speed
    
    for other in other_robots:
        if other.robot_id == state.robot_id:
            continue
        
        dx = state.position[0] - other.position[0]
        dy = state.position[1] - other.position[1]
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            continue
        
        # 優先順位を比較
        priority_self = state.priority
        priority_other = other.priority
        
        # 優先度が低い場合は譲る（減速 + 回避）
        is_yielding = priority_self < priority_other
        
        # 緊急停止（71mm以下）
        if dist < EMERGENCY_STOP_DISTANCE_MM:
            if is_yielding:
                # 譲る側: 速度を15%に減速 + 回避方向に移動
                state.preferred_speed = min(state.preferred_speed, state.base_preferred_speed * 0.15)
                strength = SEPARATION_FORCE * 1.5 * (1.0 / dist - 1.0 / EMERGENCY_STOP_DISTANCE_MM)
            else:
                # 優先側: 通常の強力な反発力
                strength = SEPARATION_FORCE * 2.0 * (1.0 / dist - 1.0 / EMERGENCY_STOP_DISTANCE_MM)
            force_x += strength * (dx / dist)
            force_y += strength * (dy / dist)
        # 最小分離距離（76mm以下）
        elif dist < MIN_SEPARATION_DISTANCE_MM:
            if is_yielding:
                # 譲る側: 速度を50%に減速 + 回避方向に移動
                state.preferred_speed = min(state.preferred_speed, state.base_preferred_speed * 0.5)
                strength = SEPARATION_FORCE * 0.8 * (1.0 / dist - 1.0 / MIN_SEPARATION_DISTANCE_MM)
            else:
                # 優先側: 通常の反発力
                strength = SEPARATION_FORCE * (1.0 / dist - 1.0 / MIN_SEPARATION_DISTANCE_MM)
            force_x += strength * (dx / dist)
            force_y += strength * (dy / dist)
        # 通常の安全距離（80mm以下）
        elif dist < SAFE_DISTANCE_MM:
            if is_yielding:
                # 譲る側: 速度を70%に減速 + 弱い回避
                state.preferred_speed = min(state.preferred_speed, state.base_preferred_speed * 0.7)
                strength = REPULSION_GAIN * 0.6 * (1.0 / dist - 1.0 / SAFE_DISTANCE_MM)
            else:
                # 優先側: 通常の反発力
                strength = REPULSION_GAIN * (1.0 / dist - 1.0 / SAFE_DISTANCE_MM)
            force_x += strength * (dx / dist)
            force_y += strength * (dy / dist)
    
    return (force_x, force_y)


def compute_predictive_avoidance_force(
    state: RobotState,
    other_robots: List[RobotState],
    dt: float,
) -> Point:
    """
    予測的衝突回避力を計算
    
    厳密な定義:
    - 0.5秒、1秒、2秒先の位置を予測
    - 予測位置が重なる場合に反発力を計算
    - 予測時間が長いほど不確実性を大きくする
    
    数学的定義:
    - p_future(t+Δt) = p(t) + v(t) * Δt
    - if ||p_future_i - p_future_j|| < 2 * radius: 反発力を計算
    """
    force_x = 0.0
    force_y = 0.0
    
    PREDICTION_TIMES = [0.5, 1.0, 2.0]  # 秒
    PREDICTION_RADIUS = 76.0  # mm
    REPULSION_GAIN = 1500.0
    
    for other in other_robots:
        if other.robot_id == state.robot_id:
            continue
        
        for pred_time in PREDICTION_TIMES:
            # 将来位置を予測
            future_x_self = state.position[0] + state.velocity[0] * pred_time
            future_y_self = state.position[1] + state.velocity[1] * pred_time
            future_x_other = other.position[0] + other.velocity[0] * pred_time
            future_y_other = other.position[1] + other.velocity[1] * pred_time
            
            dx = future_x_self - future_x_other
            dy = future_y_self - future_y_other
            dist = math.hypot(dx, dy)
            
            # 不確実性を考慮（予測時間が長いほど大きく）
            uncertainty = 1.0 + pred_time * 0.2
            combined_radius = PREDICTION_RADIUS * uncertainty * 2.0
            
            if dist < combined_radius and dist > 1e-3:
                # 反発力を計算
                time_weight = 1.0 / (1.0 + pred_time)  # 予測時間が長いほど重みを小さく
                overlap = combined_radius - dist
                strength = REPULSION_GAIN * time_weight * (overlap / combined_radius)
                
                force_x += strength * (dx / dist)
                force_y += strength * (dy / dist)
    
    return (force_x, force_y)


def compute_boundary_repulsion_force(
    state: RobotState,
) -> Point:
    """
    境界からの反発力を計算
    
    厳密な定義:
    - フィールドの端から100mm以内で反発力が働く
    - 端に近づくほど強くなる
    - 右端に溜まりにくくする
    
    数学的定義:
    - dist_to_boundary = min(x - min_x, max_x - x, y - min_y, max_y - y)
    - if dist_to_boundary < BOUNDARY_REPULSION_DISTANCE_MM:
    -   F = gain * (1/dist - 1/repulsion_distance) * direction_to_center
    """
    force_x = 0.0
    force_y = 0.0
    
    # 境界からの距離を計算
    total_buffer = BOUNDARY_BUFFER_MM + SAFETY_ZONE_MM
    min_x = FIELD_MIN_X + total_buffer
    max_x = FIELD_MAX_X - total_buffer
    min_y = FIELD_MIN_Y + total_buffer
    max_y = FIELD_MAX_Y - total_buffer
    
    # 各境界からの距離
    dist_to_left = state.position[0] - min_x
    dist_to_right = max_x - state.position[0]
    dist_to_top = state.position[1] - min_y
    dist_to_bottom = max_y - state.position[1]
    
    # 左端からの反発力
    if dist_to_left < BOUNDARY_REPULSION_DISTANCE_MM:
        strength = BOUNDARY_REPULSION_GAIN * (1.0 / (dist_to_left + 1.0) - 1.0 / BOUNDARY_REPULSION_DISTANCE_MM)
        force_x += strength  # 右方向に押す
    
    # 右端からの反発力（重要：右端に溜まりにくくする）
    if dist_to_right < BOUNDARY_REPULSION_DISTANCE_MM:
        strength = BOUNDARY_REPULSION_GAIN * (1.0 / (dist_to_right + 1.0) - 1.0 / BOUNDARY_REPULSION_DISTANCE_MM)
        force_x -= strength  # 左方向に押す
    
    # 上端からの反発力
    if dist_to_top < BOUNDARY_REPULSION_DISTANCE_MM:
        strength = BOUNDARY_REPULSION_GAIN * (1.0 / (dist_to_top + 1.0) - 1.0 / BOUNDARY_REPULSION_DISTANCE_MM)
        force_y += strength  # 下方向に押す
    
    # 下端からの反発力
    if dist_to_bottom < BOUNDARY_REPULSION_DISTANCE_MM:
        strength = BOUNDARY_REPULSION_GAIN * (1.0 / (dist_to_bottom + 1.0) - 1.0 / BOUNDARY_REPULSION_DISTANCE_MM)
        force_y -= strength  # 上方向に押す
    
    return (force_x, force_y)


# ============================================================================
# 自律性システム（Boidモデルなし）
# ============================================================================


def compute_autonomous_velocity(
    state: RobotState,
    human_zones: List[HumanZone],
    current_time: float,
    dt: float,
) -> Point:
    """
    自律的な速度を計算（Boidモデルなし）
    
    厳密な定義:
    - 相関ランダムウォークによる方向決定
    - 人間領域のポテンシャル場による回避
    - 恒常性システムによる探索促進
    - 彷徨行動
    
    数学的定義:
    - v_autonomy = preferred_speed * (cos(θ), sin(θ))
    - v_human = -α * ∇φ_human
    - v_final = w_autonomy * v_autonomy + w_human * v_human
    - w_autonomy = 0.7, w_human = 0.3
    """
    # 相関ランダムウォークを更新
    update_correlated_random_walk(state, dt, current_time)
    
    # 彷徨行動を更新
    if state.wandering_mode:
        state.wandering_timer -= dt
        if state.wandering_timer <= 0:
            state.wandering_mode = False
            state.wandering_direction_rad = random.random() * 2 * math.pi
        else:
            # 彷徨中はランダムな方向変化
            angle_change = (random.random() - 0.5) * 0.5  # ±0.25ラジアン
            state.wandering_direction_rad += angle_change
    else:
        # 一定確率で彷徨モードに入る
        if random.random() < WANDERING_PROBABILITY * dt * 2.0:  # 0.5秒間隔想定
            state.wandering_mode = True
            state.wandering_timer = WANDERING_DURATION_MIN + random.random() * (
                WANDERING_DURATION_MAX - WANDERING_DURATION_MIN
            )
            state.wandering_direction_rad = random.random() * 2 * math.pi
    
    # 目標方向を決定
    if state.wandering_mode:
        # 彷徨モード
        target_heading_rad = state.wandering_direction_rad
    else:
        # 相関ランダムウォーク
        target_heading_rad = state.current_heading_rad
    
    # 自律的な速度ベクトル
    autonomy_vx = math.cos(target_heading_rad) * state.preferred_speed
    autonomy_vy = math.sin(target_heading_rad) * state.preferred_speed
    
    # 人間領域のポテンシャル場による回避
    human_gradient = compute_human_potential_gradient(
        state.position, human_zones, current_time
    )
    human_vx = -0.9 * human_gradient[0]  # α = 0.9
    human_vy = -0.9 * human_gradient[1]
    
    # 合成（自律性70%、人間回避30%）
    final_vx = 0.7 * autonomy_vx + 0.3 * human_vx
    final_vy = 0.7 * autonomy_vy + 0.3 * human_vy
    
    return (final_vx, final_vy)


# ============================================================================
# 速度の段階的変化（物理的制約）
# ============================================================================


def update_velocity_smoothly(
    state: RobotState,
    target_velocity: Point,
    dt: float,
) -> None:
    """
    速度を段階的に変化させる（物理的制約を反映）
    
    厳密な定義:
    - 加速度・減速度を考慮した段階的な速度変化
    - 実際のロボットの物理的制約を反映
    
    数学的定義:
    - dv/dt = (v_target - v_current) / τ
    - ただし、|dv/dt| <= acceleration_rate or deceleration_rate
    """
    target_vx, target_vy = target_velocity
    current_vx, current_vy = state.velocity
    
    vx_diff = target_vx - current_vx
    vy_diff = target_vy - current_vy
    speed_diff = math.hypot(vx_diff, vy_diff)
    
    if speed_diff > 1.0:
        # 加速または減速
        rate = state.acceleration_rate if speed_diff > 0 else state.deceleration_rate
        max_change = rate * dt
        change = min(speed_diff, max_change)
        scale = change / speed_diff if speed_diff > 0 else 0.0
        
        state.velocity = (
            current_vx + vx_diff * scale,
            current_vy + vy_diff * scale,
        )
    else:
        state.velocity = target_velocity
    
    # 速度制限
    speed = math.hypot(state.velocity[0], state.velocity[1])
    if speed > MAX_SPEED_MM_PER_SEC:
        scale = MAX_SPEED_MM_PER_SEC / speed
        state.velocity = (state.velocity[0] * scale, state.velocity[1] * scale)


# ============================================================================
# メインの更新ループ
# ============================================================================


def update_robot_state(
    state: RobotState,
    all_robots: List[RobotState],
    human_zones: List[HumanZone],
    dt: float,
    current_time: float,
) -> None:
    """
    ロボットの状態を更新（1タイムステップ）
    
    厳密な定義:
    1. 局所最小値チェック
    2. 恒常性システムの更新
    3. 自律的な速度計算（Boidモデルなし）
    4. 衝突回避力の計算
    5. 速度の合成と段階的変化
    6. 位置の更新
    """
    # 近傍ロボットを取得（50cm以内）
    neighbors = [
        r for r in all_robots
        if r.robot_id != state.robot_id
        and distance(state.position, r.position) < 500.0
    ]
    
    # 局所最小値チェック
    if check_local_minimum(state, dt):
        escape_local_minimum(state)
    
    # 恒常性システムの更新
    update_homeostasis(state, neighbors, dt, current_time)
    
    # 自律的な速度計算（Boidモデルなし）
    autonomous_velocity = compute_autonomous_velocity(
        state, human_zones, current_time, dt
    )
    
    # 衝突回避力の計算
    collision_force = compute_collision_avoidance_force(state, all_robots)
    predictive_force = compute_predictive_avoidance_force(state, all_robots, dt)
    boundary_force = compute_boundary_repulsion_force(state)
    
    # 力を速度に変換（重み付け合成）
    collision_vx = collision_force[0] * dt * 0.5  # ダンピング係数
    collision_vy = collision_force[1] * dt * 0.5
    predictive_vx = predictive_force[0] * dt * 0.3
    predictive_vy = predictive_force[1] * dt * 0.3
    boundary_vx = boundary_force[0] * dt * 0.4  # 境界反発力（控えめに）
    boundary_vy = boundary_force[1] * dt * 0.4
    
    # 最終的な目標速度
    target_vx = autonomous_velocity[0] + collision_vx + predictive_vx + boundary_vx
    target_vy = autonomous_velocity[1] + collision_vy + predictive_vy + boundary_vy
    
    # 速度を段階的に変化
    update_velocity_smoothly(state, (target_vx, target_vy), dt)
    
    # 位置を更新
    new_x = state.position[0] + state.velocity[0] * dt
    new_y = state.position[1] + state.velocity[1] * dt
    state.position = clamp_position((new_x, new_y))
    
    # 向きを更新（速度ベクトルの方向）
    speed = math.hypot(state.velocity[0], state.velocity[1])
    if speed > 1.0:
        state.heading = normalize_heading(
            math.degrees(math.atan2(state.velocity[1], state.velocity[0])) + 90.0
        )


# ============================================================================
# JSONフレーム生成
# ============================================================================


def make_frame(
    time_sec: float,
    position: Point,
    heading: Heading,
    use_position: bool,
    use_rotation: bool,
    use_loose_clamp: bool = False,
) -> Dict[str, object]:
    """JSONフレームを作成（ground_rules.mdの仕様に準拠）"""
    # 初期状態の場合は緩いクリッピングを使用
    if use_loose_clamp:
        pos_clamped = clamp_position_loose(position)
    else:
        pos_clamped = clamp_position(position)
    frame: Dict[str, object] = {
        "time": round(time_sec, 3),
        "x": round(pos_clamped[0], 3),
        "y": round(pos_clamped[1], 3),
        "rotation": round(normalize_heading(heading), 3),
        "use_position": use_position,
        "use_rotation": use_rotation,
        "sound": None,
    }
    if use_position:
        frame["stop_distance"] = 20.0
    if use_rotation:
        frame["angle_tolerance"] = 5.0
    return frame


# ============================================================================
# メインの軌跡生成
# ============================================================================


def generate_swarm_trajectories(
    initial_states: Dict[int, Tuple[Point, Heading]],
    human_zones: List[HumanZone],
    start_time: float = 50.0,
    end_time: float = 120.0,
    dt: float = 0.5,
) -> Dict[int, List[Dict[str, object]]]:
    """
    50-120秒の群ロボット動作の軌跡を生成
    
    厳密な定義:
    - オフライン軌跡生成（リアルタイムシミュレーションではない）
    - 各時刻での位置・姿勢を計算（衝突回避を含む）
    - JSONフレームとして出力
    
    Args:
        initial_states: ロボットID → (位置, 向き) の辞書
        human_zones: 人間領域の定義（時刻付き）
        start_time: 開始時刻（秒）
        end_time: 終了時刻（秒）
        dt: タイムステップ（秒）
    
    Returns:
        ロボットID → フレームリストの辞書
    """
    # ロボット状態を初期化
    robots: List[RobotState] = []
    for robot_id, (pos, heading) in initial_states.items():
        speed_category = assign_speed_category(robot_id, len(initial_states))
        preferred_speed = calculate_preferred_speed(robot_id, speed_category)
        
        # 個体差によるパラメータ変動
        rng = random.Random(robot_id)
        homeostasis_energy = 0.5 + rng.random() * 0.5
        homeostasis_target = 0.6 + rng.random() * 0.3
        homeostasis_decay = 0.005 + rng.random() * 0.01
        homeostasis_threshold = 0.2 + rng.random() * 0.2
        
        # 初期速度（ランダムな方向）
        initial_heading_rad = math.radians(heading - 90.0)  # toio座標系から数学座標系へ
        initial_vx = math.cos(initial_heading_rad) * preferred_speed
        initial_vy = math.sin(initial_heading_rad) * preferred_speed
        
        # 優先順位を計算
        priority = compute_priority(robot_id)
        
        state = RobotState(
            robot_id=robot_id,
            position=pos,
            heading=heading,
            velocity=(initial_vx, initial_vy),
            current_heading_rad=initial_heading_rad,
            heading_persistence_time=PERSISTENCE_TIME_BASE * (0.5 + rng.random() * 1.0),
            homeostasis_energy=homeostasis_energy,
            homeostasis_target=homeostasis_target,
            homeostasis_decay_rate=homeostasis_decay,
            homeostasis_threshold=homeostasis_threshold,
            speed_category=speed_category,
            preferred_speed=preferred_speed,
            speed_bias=0.8 + rng.random() * 0.4,
            current_speed=preferred_speed,
            acceleration_rate=40.0 + rng.random() * 50.0,
            deceleration_rate=50.0 + rng.random() * 60.0,
            last_position=pos,
            priority=priority,
            base_preferred_speed=preferred_speed,
        )
        robots.append(state)
    
    # フレームを生成
    frames: Dict[int, List[Dict[str, object]]] = {rid: [] for rid in initial_states.keys()}
    
    # 最初のフレーム（start_time）は初期状態をそのまま使う（緩いクリッピング）
    current_time = start_time
    for robot in robots:
        frame = make_frame(
            current_time,
            robot.position,
            robot.heading,
            use_position=True,
            use_rotation=True,
            use_loose_clamp=True,  # 初期状態は緩いクリッピング
        )
        frames[robot.robot_id].append(frame)
    
    # 次のタイムステップから更新を開始
    current_time = start_time + dt
    while current_time <= end_time:
        # 各ロボットの状態を更新
        for robot in robots:
            update_robot_state(robot, robots, human_zones, dt, current_time)
        
        # JSONフレームを生成
        for robot in robots:
            # 位置と向きの両方を更新（群の動きでは同時に変化する）
            frame = make_frame(
                current_time,
                robot.position,
                robot.heading,
                use_position=True,
                use_rotation=True,
            )
            frames[robot.robot_id].append(frame)
        
        current_time += dt
    
    return frames


# ============================================================================
# 初期状態の読み込み
# ============================================================================


def load_initial_state(path: Path, timestamp: float) -> Dict[int, Tuple[Point, Heading]]:
    """初期状態を読み込む"""
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    
    initial_states: Dict[int, Tuple[Point, Heading]] = {}
    
    if "sets" in data:
        for entry in data["sets"]:
            robot_id_str = entry["id"]
            robot_id = int(robot_id_str.split("_")[-1])
            
            # 指定時刻に最も近いフレームを探す
            closest_frame = None
            closest_time_diff = float("inf")
            
            for frame in entry["frames"]:
                frame_time = frame["time"]
                time_diff = abs(frame_time - timestamp)
                
                if time_diff < closest_time_diff and frame_time <= timestamp:
                    closest_time_diff = time_diff
                    closest_frame = frame
            
            if closest_frame:
                x = closest_frame.get("x", closest_frame.get("position", {}).get("x", 0.0))
                y = closest_frame.get("y", closest_frame.get("position", {}).get("y", 0.0))
                rotation = closest_frame.get("rotation", closest_frame.get("angle", 0.0))
                
                initial_states[robot_id] = ((x, y), rotation)
    
    return initial_states


def load_human_zones(path: Path) -> List[HumanZone]:
    """人間領域を読み込む"""
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    
    QUADRANT_CENTERS = {
        "Q1": (262.75, 250.75),
        "Q2": (262.75, 682.25),
        "Q3": (720.25, 682.25),
        "Q4": (720.25, 250.75),
        "Center": (491.5, 466.5),
    }
    
    zones: List[HumanZone] = []
    for zone_data in data:
        center = zone_data["center"]
        if isinstance(center, str):
            center = QUADRANT_CENTERS.get(center, QUADRANT_CENTERS["Center"])
        else:
            center = (center["x"], center["y"])
        
        zones.append(
            HumanZone(
                start_time=zone_data["start"],
                end_time=zone_data["end"],
                center=center,
                radius=zone_data.get("radius", 150.0),
                strength=zone_data.get("strength", 1.0),
            )
        )
    
    return zones


# ============================================================================
# メイン
# ============================================================================


def main() -> None:
    parser = argparse.ArgumentParser(
        description="50-120秒の群ロボット動作の軌跡を生成（Boidモデルなし）"
    )
    parser.add_argument(
        "--initial-state",
        type=Path,
        required=True,
        help="初期状態JSONファイル（例: quadrant_output/all_robots.json）",
    )
    parser.add_argument(
        "--initial-timestamp",
        type=float,
        default=50.0,
        help="初期状態を抽出する時刻（秒、デフォルト: 50.0）",
    )
    parser.add_argument(
        "--human-zones",
        type=Path,
        required=True,
        help="人間領域JSONファイル（例: data/human_zones.json）",
    )
    parser.add_argument(
        "--start-time",
        type=float,
        default=50.0,
        help="開始時刻（秒、デフォルト: 50.0）",
    )
    parser.add_argument(
        "--end-time",
        type=float,
        default=120.0,
        help="終了時刻（秒、デフォルト: 120.0）",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.5,
        help="タイムステップ（秒、デフォルト: 0.5）",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("swarm_output"),
        help="出力ディレクトリ（デフォルト: swarm_output）",
    )
    args = parser.parse_args()
    
    print("群ロボット動作の軌跡生成を開始...")
    print(f"  初期状態: {args.initial_state}")
    print(f"  初期時刻: {args.initial_timestamp}s")
    print(f"  人間領域: {args.human_zones}")
    print(f"  時間範囲: {args.start_time}s - {args.end_time}s")
    print(f"  タイムステップ: {args.dt}s")
    
    # 初期状態を読み込む
    initial_states = load_initial_state(args.initial_state, args.initial_timestamp)
    print(f"  読み込んだロボット数: {len(initial_states)}")
    
    # 人間領域を読み込む
    human_zones = load_human_zones(args.human_zones)
    print(f"  読み込んだ人間領域数: {len(human_zones)}")
    
    # 軌跡を生成
    frames = generate_swarm_trajectories(
        initial_states,
        human_zones,
        start_time=args.start_time,
        end_time=args.end_time,
        dt=args.dt,
    )
    
    # JSONファイルに出力
    args.output_dir.mkdir(parents=True, exist_ok=True)
    
    set_entries: List[Dict[str, object]] = []
    for robot_id, frame_list in frames.items():
        payload = {"frames": frame_list}
        set_entries.append({"id": f"robot_{robot_id:02d}", "frames": frame_list})
        
        out_path = args.output_dir / f"robot_{robot_id:02d}.json"
        with out_path.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
    
    # 統合ファイルを出力
    all_path = args.output_dir / "all_robots.json"
    with all_path.open("w", encoding="utf-8") as fh:
        json.dump({"sets": set_entries}, fh, ensure_ascii=False, indent=2)
    
    print(f"\n生成完了: {len(frames)}台のロボット")
    print(f"  総フレーム数: {sum(len(f) for f in frames.values())}")
    print(f"  出力ディレクトリ: {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()

