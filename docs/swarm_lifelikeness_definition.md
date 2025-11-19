# 群ロボット動作の生命らしさの厳密な定義

## 概要

50-120秒の「群ロボット動作」において、生命らしさを厳密に数理モデルとして定義する。Boidモデルは使用せず、衝突回避と自律性のみで実現する。

## 基本方針

1. **オフライン軌跡生成**: リアルタイムシミュレーションではなく、離散的な時刻（0.5秒間隔）で位置・姿勢を計算
2. **20台のロボット**: 各ロボットが独立した状態を持つ
3. **人間のポジションに基づく回避**: 時刻付きの人間領域をポテンシャル場として定義
4. **生命らしさの厳密な定義**: 数理モデルとして明確に定義

---

## 1. 相関ランダムウォーク（Correlated Random Walk）

### 1.1 定義

**相関ランダムウォーク**は、完全にランダムな動きではなく、前の方向から小さな角度変化をするランダムウォークである。

### 1.2 数学的定義

現在の進行方向を `θ(t)`、時刻を `t` とする。

```
θ(t+dt) = θ(t) + N(0, σ²)  if persistence_time <= 0
θ(t+dt) = θ(t)              if persistence_time > 0
```

ただし：
- `persistence_time`: 方向を保持する時間（秒）
- `σ`: 角度変化の標準偏差（ラジアン）
- `N(0, σ²)`: 平均0、分散σ²の正規分布

### 1.3 パラメータ

- `PERSISTENCE_TIME_BASE = 5.0` 秒: 方向を保持する時間
- `PERSISTENCE_ANGLE_STD = 0.25` ラジアン: 角度変化の標準偏差
- `SHARP_TURN_PROBABILITY = 0.01`: 急旋回の確率（1%）
- `SHARP_TURN_ANGLE = π/2`: 急旋回の角度範囲（±90度）

### 1.4 実装

```python
def update_correlated_random_walk(state: RobotState, dt: float, current_time: float):
    state.heading_persistence_time -= dt
    
    if state.heading_persistence_time <= 0:
        # 前の方向から小さな角度変化
        angle_change = gaussian_random() * PERSISTENCE_ANGLE_STD
        state.current_heading_rad += angle_change
        state.heading_persistence_time = PERSISTENCE_TIME_BASE * (0.5 + random.random() * 1.0)
```

---

## 2. 恒常性（Homeostasis）システム

### 2.1 定義

**恒常性**は、生物が内部環境を一定に保とうとする性質である。ロボットの場合、エネルギー値が目標値から外れると探索行動を促進する。

### 2.2 数学的定義

エネルギー値を `E(t)`、目標値を `E_target`、減衰率を `decay_rate` とする。

```
dE/dt = -decay_rate * E
```

偏差が閾値を超えると探索行動を促進：

```
if |E(t) - E_target| > threshold:
    探索行動を促進（彷徨モードに入る）
```

### 2.3 パラメータ

- `HOMEOSTASIS_TARGET_BASE = 0.7`: 目標エネルギー値 [0, 1]
- `HOMEOSTASIS_DECAY_BASE = 0.01` 秒^-1: エネルギー減衰率
- `HOMEOSTASIS_THRESHOLD_BASE = 0.3`: 探索行動を促進する偏差閾値

### 2.4 個体差

各ロボットで以下のパラメータが異なる：
- `homeostasis_energy`: 0.5-1.0（ランダム）
- `homeostasis_target`: 0.6-0.9（ランダム）
- `homeostasis_decay_rate`: 0.005-0.015（ランダム）
- `homeostasis_threshold`: 0.2-0.4（ランダム）

---

## 3. エントロピーベースの収束防止

### 3.1 定義

**エントロピー**は、近傍ロボットの位置の分散度を測定する。エントロピーが低い（収束している）場合、パラメータを変更して収束を防ぐ。

### 3.2 数学的定義

近傍ロボットの位置を `p_i` (i=1,2,...,N)、平均位置を `μ` とする。

```
μ = (1/N) Σ p_i
σ² = (1/N) Σ ||p_i - μ||²
entropy = min(1.0, σ² / max_variance)
```

エントロピーが低い場合（`entropy < 0.3`）、パラメータをランダムに変更：

```
if avg_entropy < 0.3:
    homeostasis_target += random_variation
    homeostasis_decay_rate += random_variation * 0.001
```

### 3.3 実装

```python
def calculate_entropy(state: RobotState, neighbors: List[RobotState], current_time: float) -> float:
    if not neighbors:
        return 0.5
    
    # 平均位置を計算
    avg_x = sum(n.position[0] for n in neighbors) / len(neighbors)
    avg_y = sum(n.position[1] for n in neighbors) / len(neighbors)
    
    # 分散を計算
    variance = sum(
        (n.position[0] - avg_x)**2 + (n.position[1] - avg_y)**2
        for n in neighbors
    ) / len(neighbors)
    
    # エントロピーを正規化
    max_variance = 100000.0
    entropy = min(1.0, variance / max_variance)
    
    return entropy
```

---

## 4. 局所最小値からの脱出

### 4.1 定義

**局所最小値**は、一定時間（2秒）20mm未満の移動で判定される。局所最小値に留まっている場合、強制的に方向を変更する。

### 4.2 数学的定義

位置を `p(t)`、最後の位置を `p_last` とする。

```
if ||p(t) - p_last|| < 20mm and (t - t_last_move) > 2.0秒:
    局所最小値と判定
    θ(t+dt) = random() * 2π  (完全にランダムな方向に変更)
```

### 4.3 実装

```python
def check_local_minimum(state: RobotState, dt: float) -> bool:
    move_distance = distance(state.position, state.last_position)
    
    if move_distance < 20.0:  # 20mm未満
        state.stuck_timer += dt
    else:
        state.stuck_timer = 0.0
        state.last_position = state.position
    
    return state.stuck_timer > 2.0  # 2秒間動かない
```

---

## 5. 個体差の導入

### 5.1 速度分布

**厳密な定義**: ロボットIDに基づいて決定論的に速度カテゴリを割り当て

```
if robot_id / total_count < 0.2:
    speed_category = "fast"      # 20%
elif robot_id / total_count < 0.8:
    speed_category = "moderate"  # 60%
else:
    speed_category = "slow"      # 20%
```

速度範囲：
- `fast`: 200-250 mm/s
- `moderate`: 120-180 mm/s
- `slow`: 80-120 mm/s

### 5.2 個体パラメータの変動

各ロボットで以下のパラメータが異なる（ロボットIDに基づいた決定論的乱数）：
- `preferred_speed`: 速度カテゴリ内でランダム
- `speed_bias`: 0.8-1.2倍
- `homeostasis_energy`: 0.5-1.0
- `homeostasis_target`: 0.6-0.9
- `acceleration_rate`: 40-90 mm/s²
- `deceleration_rate`: 50-110 mm/s²

---

## 6. 人間領域のポテンシャル場

### 6.1 定義

**ポテンシャル場**は、人間領域を負のポテンシャル（反発力）として定義する。

### 6.2 数学的定義

人間領域の中心を `c`、半径を `r`、強度を `strength` とする。

```
φ(x, y) = Σ strength * exp(-||p - c|| / (r * 0.3))
∇φ = Σ strength * exp(-||p - c|| / (r * 0.3)) * (p - c) / ||p - c||
v_human = -α * ∇φ
```

ただし：
- `α = 0.9`: 勾配係数
- `exp(-dist / (radius * 0.3))`: 指数減衰

### 6.3 実装

```python
def compute_human_potential_gradient(position: Point, human_zones: List[HumanZone], current_time: float) -> Point:
    gradient_x = 0.0
    gradient_y = 0.0
    
    for zone in human_zones:
        if not (zone.start_time <= current_time < zone.end_time):
            continue
        
        dx = position[0] - zone.center[0]
        dy = position[1] - zone.center[1]
        dist = math.hypot(dx, dy)
        
        if dist < 1e-3:
            continue
        
        decay_factor = math.exp(-dist / (zone.radius * 0.3))
        strength = zone.strength * decay_factor
        
        gradient_x += strength * (dx / dist)
        gradient_y += strength * (dy / dist)
    
    return (gradient_x, gradient_y)
```

---

## 7. 衝突回避システム（多層的）

### 7.1 緊急停止（71mm以下）

**厳密な定義**: toioの最長辺（71mm）以下で緊急停止

```
if dist < 71mm:
    v = v * 0.15  (速度を15%に減速)
    F_repulsion = separation_force * 2.0 * (1/dist - 1/71mm)
```

### 7.2 予測的衝突回避

**厳密な定義**: 0.5秒、1秒、2秒先の位置を予測し、重なる場合に反発力を計算

```
p_future(t+Δt) = p(t) + v(t) * Δt
if ||p_future_i - p_future_j|| < 2 * radius * (1 + uncertainty * Δt):
    F_repulsion = repulsion_gain * time_weight * overlap
```

ただし：
- `time_weight = 1.0 / (1.0 + Δt)`: 予測時間が長いほど重みを小さく
- `uncertainty = 0.2`: 不確実性の成長率

### 7.3 通常の反発力

**厳密な定義**: 80mm以下で通常の反発力

```
if dist < 80mm:
    F_repulsion = repulsion_gain * (1/dist - 1/80mm)
```

---

## 8. 速度の段階的変化（物理的制約）

### 8.1 定義

**段階的な速度変化**は、実際のロボットの物理的制約を反映する。

### 8.2 数学的定義

目標速度を `v_target`、現在速度を `v_current`、加速度を `a` とする。

```
dv/dt = (v_target - v_current) / τ
ただし、|dv/dt| <= acceleration_rate or deceleration_rate
```

### 8.3 実装

```python
def update_velocity_smoothly(state: RobotState, target_velocity: Point, dt: float):
    vx_diff = target_velocity[0] - state.velocity[0]
    vy_diff = target_velocity[1] - state.velocity[1]
    speed_diff = math.hypot(vx_diff, vy_diff)
    
    if speed_diff > 1.0:
        rate = state.acceleration_rate if speed_diff > 0 else state.deceleration_rate
        max_change = rate * dt
        change = min(speed_diff, max_change)
        scale = change / speed_diff
        
        state.velocity = (
            state.velocity[0] + vx_diff * scale,
            state.velocity[1] + vy_diff * scale,
        )
```

---

## 9. 彷徨（Wandering）行動

### 9.1 定義

**彷徨行動**は、一定確率でランダムな方向に移動する行動である。

### 9.2 数学的定義

彷徨モードに入る確率を `wandering_probability`、継続時間を `wandering_duration` とする。

```
if random() < wandering_probability * dt:
    wandering_mode = true
    wandering_direction = random() * 2π
    wandering_timer = wandering_duration
```

### 9.3 パラメータ

- `WANDERING_PROBABILITY = 0.005`: 彷徨モードに入る確率（フレームごと）
- `WANDERING_DURATION_MIN = 2.0` 秒: 最小継続時間
- `WANDERING_DURATION_MAX = 7.0` 秒: 最大継続時間

---

## 10. 統合的な速度計算

### 10.1 定義

最終的な速度は、自律性と人間回避を重み付けして合成する。

### 10.2 数学的定義

```
v_autonomy = preferred_speed * (cos(θ), sin(θ))
v_human = -α * ∇φ_human
v_final = 0.7 * v_autonomy + 0.3 * v_human
```

ただし：
- `v_autonomy`: 相関ランダムウォークによる自律的な速度
- `v_human`: 人間領域のポテンシャル場による回避速度
- 重み: 自律性70%、人間回避30%

### 10.3 衝突回避力の追加

```
v_final = v_final + F_collision * dt * 0.5 + F_predictive * dt * 0.3
```

---

## 11. 物理的制約の遵守

### 11.1 速度制限

```
if ||v|| > 200 mm/s:
    v = v * (200 / ||v||)
```

### 11.2 境界条件

```
if x < 34 + 25 or x > 949 - 25:
    x = clamp(x, 34 + 25, 949 - 25)
if y < 35 + 25 or y > 898 - 25:
    y = clamp(y, 35 + 25, 898 - 25)
```

### 11.3 座標の連続性

```
||p(t+dt) - p(t)|| <= ||v(t)|| * dt
```

連続するフレーム間の距離は、速度上限と時間差から導かれる最大移動量以下でなければならない。

---

## 12. Boidモデルを排除する理由

### 12.1 問題点

1. **重なりまくり**: Boidモデルは本質的にロボットを集約させる傾向がある
2. **変な動き**: 強すぎる相互作用により、予測困難な動きになる
3. **計算コスト**: 全ロボット間の相互作用を計算する必要がある

### 12.2 代替アプローチ

Boidモデルの代わりに：
- **相関ランダムウォーク**: 自然な動きを実現
- **恒常性システム**: 探索を促進
- **エントロピーベースの収束防止**: 群全体が同じ場所に集約することを防ぐ
- **個体差の導入**: 群としての協調性を弱める

---

## 13. 実装の流れ

### 13.1 1タイムステップの処理

```
1. 近傍ロボットを取得（50cm以内）
2. 局所最小値チェック
3. 恒常性システムの更新
4. エントロピー計算
5. 相関ランダムウォークを更新
6. 自律的な速度計算（Boidモデルなし）
7. 人間領域のポテンシャル場を計算
8. 衝突回避力を計算（緊急停止、予測回避、通常の反発力）
9. 速度を合成（自律性70%、人間回避30%、衝突回避力）
10. 速度を段階的に変化（物理的制約）
11. 位置を更新
12. 向きを更新（速度ベクトルの方向）
```

### 13.2 JSONフレーム生成

各タイムステップ（0.5秒間隔）で、以下のフレームを生成：

```json
{
  "time": 50.0,
  "x": 491.5,
  "y": 466.5,
  "rotation": 180.0,
  "use_position": true,
  "use_rotation": true,
  "stop_distance": 20.0,
  "angle_tolerance": 5.0,
  "sound": null
}
```

---

## 14. 検証項目

### 14.1 座標の連続性

```
||p(t+dt) - p(t)|| <= MAX_SPEED * dt
```

### 14.2 速度制限

```
||v(t)|| <= 200 mm/s
```

### 14.3 境界条件

```
34 + 25 <= x <= 949 - 25
35 + 25 <= y <= 898 - 25
```

### 14.4 衝突回避

```
||p_i(t) - p_j(t)|| >= ROBOT_DIAGONAL_MM (71mm)
```

### 14.5 人間領域の回避

```
||p(t) - human_center|| >= human_radius (200mm)
```

---

## まとめ

生命らしさを厳密に定義した数理モデル：

1. **相関ランダムウォーク**: 前の方向から小さな角度変化
2. **恒常性システム**: エネルギーが減衰し、探索を促進
3. **エントロピーベースの収束防止**: 群全体が同じ場所に集約することを防ぐ
4. **局所最小値からの脱出**: 2秒間動かない場合、強制的に方向変更
5. **個体差の導入**: 速度分布と個体パラメータの変動
6. **人間領域のポテンシャル場**: 指数減衰による反発力
7. **多層的な衝突回避**: 緊急停止、予測回避、通常の反発力
8. **物理的制約の遵守**: 速度制限、境界条件、座標の連続性

**Boidモデルは使用しない**。これらの数理モデルの組み合わせにより、生命らしい動きを実現する。

