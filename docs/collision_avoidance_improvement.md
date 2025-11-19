# 衝突回避システムの改善案

## 現在の問題点

1. **対称的な反発力**: 両方のロボットが同じ強さで反発するため、デッドロックが起こりやすい
2. **優先順位がない**: どちらが譲るべきかが決まっていない
3. **人間の動きが定義されていない**: 人間領域は静的なポテンシャル場として扱われている

## 改善案

### 1. 優先順位システム（Priority-based Yielding）

#### 1.1 優先順位の決定方法

**方法A: ロボットIDベース**
- 小さいIDが優先（例: robot_00 > robot_01 > ...）
- シンプルで決定論的

**方法B: 相対速度ベース**
- 速度が大きい方が優先（動いている方が優先）
- より自然な動作

**方法C: ランダム（推奨）**
- 各ロボットがランダムに優先度を決定
- デッドロックを防ぐ

#### 1.2 実装方法

```python
def compute_priority(state: RobotState, other: RobotState) -> float:
    """
    優先順位を計算（0.0-1.0、大きい方が優先）
    
    方法A: IDベース
    return 1.0 - (state.robot_id / total_count)
    
    方法B: 速度ベース
    speed_self = math.hypot(state.velocity[0], state.velocity[1])
    speed_other = math.hypot(other.velocity[0], other.velocity[1])
    return 1.0 if speed_self > speed_other else 0.0
    
    方法C: ランダム（推奨）
    rng = random.Random(state.robot_id)
    return rng.random()
    """
    # 方法C: ランダム（決定論的）
    rng = random.Random(state.robot_id)
    return rng.random()
```

#### 1.3 譲る動作の実装

```python
def compute_collision_avoidance_force_with_priority(
    state: RobotState,
    other_robots: List[RobotState],
) -> Point:
    """
    優先順位を考慮した衝突回避力
    
    優先度が低いロボット（譲る側）:
    - 速度を下げる（減速）
    - 方向を変える（回避）
    - 反発力を弱める
    
    優先度が高いロボット（優先側）:
    - 通常の反発力
    - 速度を維持
    """
    force_x = 0.0
    force_y = 0.0
    
    for other in other_robots:
        if other.robot_id == state.robot_id:
            continue
        
        dx = state.position[0] - other.position[0]
        dy = state.position[1] - other.position[1]
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            continue
        
        # 優先順位を計算
        priority_self = compute_priority(state, other)
        priority_other = compute_priority(other, state)
        
        # 優先度が低い場合は譲る（減速 + 回避）
        if priority_self < priority_other:
            # 譲る側: 速度を下げる + 方向を変える
            if dist < EMERGENCY_STOP_DISTANCE_MM:
                # 緊急停止: 速度を15%に減速
                state.preferred_speed *= 0.15
                # 回避方向に移動
                avoidance_strength = SEPARATION_FORCE * 1.5
            else:
                # 通常の回避: 速度を50%に減速
                state.preferred_speed *= 0.5
                avoidance_strength = SEPARATION_FORCE * 0.8
        else:
            # 優先側: 通常の反発力
            if dist < EMERGENCY_STOP_DISTANCE_MM:
                avoidance_strength = SEPARATION_FORCE * 2.0
            else:
                avoidance_strength = SEPARATION_FORCE
        
        if dist < SAFE_DISTANCE_MM:
            strength = avoidance_strength * (1.0 / dist - 1.0 / SAFE_DISTANCE_MM)
            force_x += strength * (dx / dist)
            force_y += strength * (dy / dist)
    
    return (force_x, force_y)
```

### 2. 人間の動きの定義

#### 2.1 現在の問題

- `human_zones.json`では、時刻付きで位置が変わるが、これは「瞬間移動」
- 人間の速度、軌跡が定義されていない
- 予測的な回避ができない

#### 2.2 改善案

**方法A: 速度ベクトルを追加**
```json
{
  "start": 80.0,
  "end": 85.0,
  "center_start": "Q2",
  "center_end": "Q4",
  "radius": 150.0,
  "strength": 1.0,
  "velocity": {"x": 50.0, "y": 0.0}  // mm/s
}
```

**方法B: 軌跡を定義**
```json
{
  "start": 80.0,
  "end": 85.0,
  "trajectory": [
    {"time": 80.0, "center": "Q2"},
    {"time": 82.5, "center": "Center"},
    {"time": 85.0, "center": "Q4"}
  ],
  "radius": 150.0,
  "strength": 1.0
}
```

#### 2.3 予測的な回避

```python
def compute_human_potential_gradient_with_motion(
    position: Point,
    human_zones: List[HumanZone],
    current_time: float,
) -> Point:
    """
    人間の動きを考慮したポテンシャル場
    
    人間の将来位置を予測:
    - p_future(t+Δt) = p_current + velocity * Δt
    - 予測位置でのポテンシャルを計算
    """
    gradient_x = 0.0
    gradient_y = 0.0
    
    PREDICTION_TIME = 1.0  # 1秒先を予測
    
    for zone in human_zones:
        if not (zone.start_time <= current_time < zone.end_time):
            continue
        
        # 人間の現在位置
        current_center = zone.center
        
        # 人間の将来位置を予測（速度がある場合）
        if hasattr(zone, 'velocity'):
            future_center = (
                current_center[0] + zone.velocity[0] * PREDICTION_TIME,
                current_center[1] + zone.velocity[1] * PREDICTION_TIME,
            )
        else:
            future_center = current_center
        
        # 現在位置と将来位置の両方でポテンシャルを計算
        for center in [current_center, future_center]:
            dx = position[0] - center[0]
            dy = position[1] - center[1]
            dist = math.hypot(dx, dy)
            
            if dist < 1e-3:
                continue
            
            # ポテンシャルの勾配を計算
            decay_factor = math.exp(-dist / (zone.radius * 0.3))
            strength = zone.strength * decay_factor * 0.5  # 重みを半分に
            
            gradient_x += strength * (dx / dist)
            gradient_y += strength * (dy / dist)
    
    return (gradient_x, gradient_y)
```

## 実装の優先順位

1. **優先順位システム（最優先）**: デッドロックを防ぐために必須
2. **人間の動きの定義**: より自然な回避動作のため

## 推奨実装

### 優先順位システム（方法C: ランダム）

```python
# RobotStateに追加
priority: float = 0.5  # 0.0-1.0

# 初期化時
rng = random.Random(robot_id)
state.priority = rng.random()

# 衝突回避時
if state.priority < other.priority:
    # 譲る側: 減速 + 回避
    state.preferred_speed *= 0.5
    avoidance_strength = SEPARATION_FORCE * 0.8
else:
    # 優先側: 通常の反発力
    avoidance_strength = SEPARATION_FORCE * 2.0
```

### 人間の動き（簡易版）

```python
# HumanZoneに追加
velocity: Optional[Point] = None  # mm/s

# 人間領域の読み込み時
if "center_start" in zone_data and "center_end" in zone_data:
    duration = zone_data["end"] - zone_data["start"]
    if duration > 0:
        start_center = QUADRANT_CENTERS[zone_data["center_start"]]
        end_center = QUADRANT_CENTERS[zone_data["center_end"]]
        velocity = (
            (end_center[0] - start_center[0]) / duration,
            (end_center[1] - start_center[1]) / duration,
        )
```


