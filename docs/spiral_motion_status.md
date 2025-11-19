# 螺旋運動の実装状況

## 1. 基本情報

### 1.1 タイムライン
- **開始時刻**: 30.0秒（`generate_json.py`）または 32.5秒（`quadrant_json_generator.py`）
- **終了時刻**: 50.0秒
- **継続時間**: 20秒または17.5秒
- **ロボット数**: 30台（ID: 0-29）

### 1.2 動作の目的
- 中心から螺旋状に拡散する動き
- 集団の連動感を保ちながら、徐々に分散
- 50秒時点で分散度75%以上を達成

### 1.3 設計思想
- **宇宙軌道のメタファー**: 第一宇宙速度（小さな軌道）→ 第二宇宙速度（軌道脱出・拡散）
- **放射状拡散モデル**: 全ロボットが同じ角速度で回転し、外側のロボットが速く移動
- **加速的な拡大**: 最初はゆっくり、だんだん早く大規模になっていく

## 2. 座標系と初期状態

### 2.1 フィールド座標系
- **原点**: フィールドの左上角
- **X軸**: 右方向が正（34mm → 949mm）
- **Y軸**: 下方向が正（35mm → 898mm）
- **フィールド中心**: (491.5mm, 466.5mm)

### 2.2 角度の定義
- **0度**: 上向き（Y軸の負の方向）
- **90度**: 右向き（X軸の正の方向）
- **180度**: 下向き（Y軸の正の方向）**初期向き**
- **270度**: 左向き（X軸の負の方向）
- **角度の正の方向**: 反時計回り（toioの仕様に従う）

### 2.3 初期配置（30秒時点）
- **仮定**: 0-30秒の動作で相対位置関係（グリッド構造）は維持される
- **初期半径範囲**: 約**82mm ～ 559mm**（フィールド中心からの距離）
  - **最小半径**: 約82mm（中央付近のロボット、例: ID 14, 15）
  - **最大半径**: 約559mm（四隅のロボット、例: ID 0, 5, 24, 29）

## 3. 実装の状況

### 3.1 実装ファイル

#### `generate_json.py`（30-50秒）
- **開始時刻**: 30.0秒
- **実装関数**:
  - `calculate_spiral_params()`: 初期パラメータ計算
  - `calculate_spiral_frame()`: 各時刻でのフレーム計算
- **特徴**:
  - 設計ドキュメント（`archive/spiral_motion_design.md`）に基づく実装
  - 中心からの距離に応じて角速度と拡大加速度を調整
  - 衝突回避のためのパラメータ調整機能あり

#### `quadrant_json_generator.py`（32.5-50秒）
- **開始時刻**: 32.5秒
- **実装関数**:
  - `build_spiral_initial_states()`: 初期状態の構築
  - `generate_spiral_frames()`: フレーム生成
- **特徴**:
  - より動的な角速度制御（接線速度制限あり）
  - 45秒以降にノイズを付与（ランダムウォーク）
  - イージング関数による滑らかな半径拡大

### 3.2 パラメータの違い

| パラメータ | `generate_json.py` | `quadrant_json_generator.py` |
|-----------|-------------------|----------------------------|
| **開始時刻** | 30.0秒 | 32.5秒 |
| **角速度** | 31-40deg/sec（距離に応じて変化） | 動的（最大45deg/sec、接線速度制限あり） |
| **半径拡大** | 加速的線形拡大 `r(t) = r0 + a × dt²/2` | イージング関数 `f(t) = progress^1.1` |
| **最終半径** | `min(300 + (r0 - 82) × 0.3, 380)` | `min(400, r0 + (400 - r_max) × (r0 / r_max))` |
| **ノイズ** | なし | 45秒以降にランダムウォーク（半径±5mm、角度±2°） |
| **接線速度制限** | なし（最大232mm/sec） | 160mm/sec以下に制限 |

## 4. 設計仕様（確定版）

### 4.1 基本パラメータ（`archive/spiral_motion_design.md`より）

#### 角速度
- **基準角速度**: **35deg/sec**（0.611 rad/sec）**確定**
- **全ロボット共通**: 連動感を保つため、全ロボットが同じ角速度で回転
- **安全範囲**: 100deg/sec以下（転倒リスクを最小化）
- **1回転時間**: 10.3秒（360° ÷ 35deg/sec）

#### 回転方向
- **推奨**: **反時計回り**（toioの角度定義に合わせる）
- **全ロボット統一**: 統一された回転方向で回転

#### 半径拡大
- **初期半径**: 各ロボットの30秒時点での位置からフィールド中心までの距離
- **最終半径**: `r_max_i = min(300 + (r₀_i - 82) × 0.3, 380)`
  - **r_base**: 300mm（最小最終半径）
  - **r_min**: 82mm（最小初期半径）
  - **k**: 0.3（拡大係数）
  - **r_max_max**: 380mm（速度制限により制約）
- **範囲**: **300mm ～ 380mm**

#### 拡大加速度
- **計算式**: `a_i = 2 × (r_max_i - r₀_i) / 400`
- **範囲**: 約**0.4mm/sec² ～ 1.1mm/sec²**（ロボットにより異なる）
- **時間変化**: `r(t) = r₀ + a × (t - 30)² / 2`
  - **30秒時点**: `dr/dt = 0`（最初はゆっくり）
  - **50秒時点**: `dr/dt = a × 20`（最後は早く広がる）

### 4.2 速度制限と安全性

#### 最大線速度
- **50秒時点での最大線速度**: 約**232mm/sec**（外側のロボット）
- **toioの直進速度制限**: 200mm/sec（載せ物あり）
- **許容性**: 短時間（20秒間のみ）かつ加速的拡大のため許容（実テスト推奨）

#### 拡大加速度の安全性
- **拡大加速度範囲**: 約**0.4 ～ 1.1 mm/sec²**
- **評価**: 非常に小さな加速度であり、**急加速による転倒リスクは極めて低い**

#### 衝突回避
- **最小安全距離**: 100mm（toioサイズ72mm + 安全マージン28mm）
- **初期配置**: 列間隔163mm、行間隔191mmで、安全距離100mmを満たしている
- **螺旋運動の性質**: 全ロボットが同じ角速度で回転し、半径のみが拡大するため、相対的な位置関係が維持され、衝突リスクは低減

### 4.3 分散度の目標
- **目標**: 50秒時点で分散度**75%以上**
- **達成方法**: 最終半径300mm ～ 380mmの設定により、フィールド全体に75%以上散らばる

## 5. 実装の詳細

### 5.1 `generate_json.py`の実装

#### 初期パラメータ計算
```python
def calculate_spiral_params(robot_id: int) -> Tuple[float, float, float, float, float]:
    # 30秒時点での位置を計算
    frame_30 = calculate_first_locomotion_frame(robot_id, 30.0)
    x_i = frame_30["position"]["x"]
    y_i = frame_30["position"]["y"]
    
    # 初期半径と角度
    dx_i = x_i - FIELD_CENTER_X
    dy_i = y_i - FIELD_CENTER_Y
    r0_i = math.sqrt(dx_i**2 + dy_i**2)
    theta0_i_rad = math.atan2(dy_i, dx_i)
    theta0_i_deg = normalize_angle(math.degrees(theta0_i_rad))
    
    # 中心からの距離に基づいて分類
    if r0_i < 150.0:
        # 中心近く: 角速度を小さく
        omega_base = 31.0 + (robot_id % 3) * 0.5  # 31-32deg/sec
        a_multiplier = 0.85 + (robot_id % 5) * 0.02
    elif r0_i < 300.0:
        # 中間距離: 基準値
        omega_base = 35.0 + ((robot_id % 3) - 1) * 0.5  # 34-36deg/sec
        a_multiplier = 1.0 + ((robot_id % 5) - 2) * 0.02
    else:
        # 外側: 角速度を大きく
        omega_base = 38.0 + (robot_id % 3) * 0.5  # 38-40deg/sec
        a_multiplier = 1.1 + (robot_id % 5) * 0.02
    
    # 最終半径と拡大加速度
    r_max_i = min(R_BASE + (r0_i - R_MIN) * K, R_MAX_MAX)
    a_i = 2.0 * (r_max_i - r0_i) / 400.0 * a_multiplier
    
    return r0_i, theta0_i_deg, r_max_i, a_i, omega_base
```

#### フレーム計算
```python
def calculate_spiral_frame(robot_id: int, t: float, params: Tuple[float, float, float, float, float]) -> Dict[str, Any]:
    r0, theta0_deg, r_max, a, omega_deg_per_sec = params
    
    dt = t - SPIRAL_START_TIME  # 30.0秒からの経過時間
    
    # 半径の計算（加速的な線形拡大）
    r_t = r0 + a * dt**2 / 2.0
    
    # 回転角度の計算（反時計回り）
    theta_t_deg = normalize_angle(theta0_deg + omega_deg_per_sec * dt)
    theta_t_rad = math.radians(theta_t_deg)
    
    # 位置の計算（極座標から直交座標へ）
    x_t = FIELD_CENTER_X + r_t * math.cos(theta_t_rad)
    y_t = FIELD_CENTER_Y + r_t * math.sin(theta_t_rad)
    
    # フィールド境界チェック
    x_t = max(FIELD_MIN_X, min(FIELD_MAX_X, x_t))
    y_t = max(FIELD_MIN_Y, min(FIELD_MAX_Y, y_t))
    
    # 向きの計算（接線方向 = 回転角度 + 90°）
    rotation_t = normalize_angle(theta_t_deg + 90.0)
    
    return {
        "time": round(t, 1),
        "use_position": True,
        "use_rotation": True,
        "position": {"x": round(x_t, 1), "y": round(y_t, 1)},
        "rotation": round(rotation_t, 1),
        "sound": None
    }
```

### 5.2 `quadrant_json_generator.py`の実装

#### 主な定数
```python
SPIRAL_START_TIME = 32.5
SPIRAL_END_TIME = 50.0
SPIRAL_DT = 0.5
SPIRAL_MAX_RADIUS = 400.0
SPIRAL_MAX_TANGENTIAL_SPEED = 160.0  # mm/s (80% of 200)
SPIRAL_MAX_OMEGA_DEG = 45.0
SPIRAL_FINAL_OMEGA_DEG = 10.0
SPIRAL_RADIUS_EASE_EXP = 1.1
SPIRAL_NOISE_START_BASE = 45.0
SPIRAL_NOISE_RADIUS_STEP = 1.5  # mm per step
SPIRAL_NOISE_RADIUS_MAX = 5.0  # mm
SPIRAL_NOISE_THETA_STEP_DEG = 0.4  # deg per step
SPIRAL_NOISE_THETA_MAX_DEG = 2.0  # deg
SPIRAL_SPIN_RAMP_DURATION = 1.5  # seconds to ramp up angular speed
```

#### 特徴
- **動的角速度制御**: 接線速度を160mm/sec以下に制限
- **イージング関数**: `progress^1.1`による滑らかな半径拡大
- **ノイズ付与**: 45秒以降にランダムウォークを追加（決定論的）
- **角速度減衰**: 45秒以降、角速度を10deg/secまで減衰

## 6. 時間経過のフェーズ

### 6.1 第一宇宙速度フェーズ（30-35秒）
- **メタファー**: 小さな軌道を周回（地球の周り）
- **角速度**: 35deg/sec（一定）
- **半径拡大**: 最小限（`dr/dt ≈ 0-4.5 mm/sec`）
- **線速度**: 122-129mm/sec程度
- **目的**: 小さな軌道を維持する回転運動を開始、集団の連動感を確立

### 6.2 加速フェーズ（35-45秒）
- **メタファー**: 第一宇宙速度から第二宇宙速度へ加速
- **角速度**: 35deg/sec（一定）
- **半径拡大**: 加速中（`dr/dt = 4.5-13.5 mm/sec`）
- **線速度**: 129-184mm/sec
- **目的**: だんだん広がっていく動きを実現、軌道が拡大

### 6.3 第二宇宙速度フェーズ（45-50秒）
- **メタファー**: 第二宇宙速度に到達、軌道から脱出
- **角速度**: 35deg/sec（一定、または減衰）
- **半径拡大**: 最大速度（`dr/dt = 13.5-18 mm/sec`）
- **線速度**: 184-232mm/sec（最大）
- **目的**: 大きく広がり、50秒時点で分散度75%以上を達成

## 7. 実装の課題と改善点

### 7.1 実装の不一致
- **問題**: `generate_json.py`と`quadrant_json_generator.py`で異なるアプローチを採用
- **影響**: どちらの実装を使用するか明確でない
- **推奨**: 設計ドキュメント（`archive/spiral_motion_design.md`）に基づく`generate_json.py`を基準とする

### 7.2 パラメータの調整
- **角速度**: 設計では35deg/sec（全ロボット共通）だが、実装では距離に応じて31-40deg/secに変化
- **開始時刻**: 設計では30秒だが、`quadrant_json_generator.py`では32.5秒
- **推奨**: 設計仕様に合わせて統一

### 7.3 速度制限の検証
- **最大線速度**: 232mm/secは200mm/secの制限を約16%超過
- **推奨**: 実テストで動作確認し、必要に応じて角速度や最大半径を調整

### 7.4 衝突回避の検証
- **現状**: 理論的には衝突回避が確保される
- **推奨**: シミュレーションまたは実テストで衝突を確認

## 8. 検証項目

### 8.1 計算結果の検証
- [ ] 初期半径の範囲: 82mm ～ 559mm の範囲内か
- [ ] 最終半径の範囲: 300mm ～ 380mm の範囲内か
- [ ] 座標の範囲: フィールド内（X: 34-949mm, Y: 35-898mm）か
- [ ] 角度の範囲: 0°～360° の範囲内か
- [ ] 時間間隔: 0.5秒間隔で連続しているか

### 8.2 安全性の検証
- [ ] 最大線速度: 200mm/sec以下（または許容範囲内）
- [ ] ロボット間距離: 100mm以上（全時刻、全ペア）
- [ ] フィールド境界: 全フレームで境界内
- [ ] 転倒リスク: 角速度と加速度が安全範囲内

### 8.3 分散度の検証
- [ ] 50秒時点での分散度: 75%以上
- [ ] 螺旋パターンの維持: 初期角度の分布が維持されているか

## 9. 関連ドキュメント

- **設計ドキュメント**: `archive/spiral_motion_design.md`（確定版）
- **実装仕様**: `archive/spiral_implementation.md`
- **速度計算**: `archive/spiral_calculation.md`
- **ロコモーション整理**: `docs/locomotion_phase_summary.md`
- **実装状況**: `docs/implementation_status.md`

## 10. まとめ

### 10.1 実装状況
- ✅ **設計ドキュメント**: 確定版が存在（`archive/spiral_motion_design.md`）
- ✅ **実装**: 2つの実装が存在（`generate_json.py`、`quadrant_json_generator.py`）
- ⚠️ **不一致**: 実装間でパラメータとアプローチが異なる

### 10.2 推奨事項
1. **実装の統一**: 設計ドキュメントに基づく`generate_json.py`を基準とする
2. **パラメータの調整**: 設計仕様（角速度35deg/sec、開始時刻30秒）に合わせる
3. **実テスト**: 速度制限と衝突回避を実機で検証
4. **ドキュメント更新**: 実装の違いを明確化し、使用する実装を決定

### 10.3 確定パラメータ
- ✅ **角速度**: 35deg/sec（全ロボット共通）
- ✅ **回転方向**: 反時計回り
- ✅ **半径拡大**: 加速的線形拡大
- ✅ **最終半径**: `r_max_i = min(300 + (r₀_i - 82) × 0.3, 380)`
- ✅ **分散度**: 75%以上（50秒時点）
- ✅ **衝突回避**: 確保される（理論的）

