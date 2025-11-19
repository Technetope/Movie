# JSONファイル生成の実装仕様（0-50秒）

## 概要
30台のtoioロボットの0-50秒の動作をJSONファイルとして生成する実装仕様。
- **0-30秒**: 最初のロコモーション（集団行動、fast→stop→fastパターン）
- **30-50秒**: 螺旋運動（連続的な回転と拡散）

**入力**: ロボットID (0-29) と 初期位置（グリッド配置から計算可能）  
**出力**: 連続したJSONファイル（0-50秒の時系列フレーム）  
**計算方法**: 数列的な計算で効率的に生成

## 入力仕様

### 初期状態（0秒時点）
各ロボットID `i` (0-29)について、グリッド配置から初期位置を計算：

```python
# 定数（グリッド配置）
GRID_START_X = 84       # mm（左上のX座標）
GRID_START_Y = 85       # mm（左上のY座標）
GRID_COL_SPACING = 163  # mm（列間隔）
GRID_ROW_SPACING = 191  # mm（行間隔）

# 初期座標の計算
col_i = i % 6           # 列番号（0-5）
row_i = i // 6          # 行番号（0-4）
x_i_0 = GRID_START_X + col_i * GRID_COL_SPACING
y_i_0 = GRID_START_Y + row_i * GRID_ROW_SPACING
rotation_i_0 = 180.0    # 初期向き（下向き）
```

**出力**: 各ロボットの `(x_i_0, y_i_0, rotation_i_0)`

### 30秒時点での状態（螺旋運動開始時）
- **向き**: 180°（全ロボット共通、45°回転後の最終状態）
- **位置**: 0-30秒の動作後の位置（グリッド配置とほぼ同じ相対位置）
- **仮定**: 相対位置維持モデル（グリッド構造は維持）

## 実装に必要な計算式（数列的計算）

### Part 1: 最初のロコモーション（0-30秒）のJSONフレーム生成

#### 動作シーケンス（0-30秒）

| # | 動作 | 速度 | 移動距離/角度 | 移動時間 | 停止時間 | 開始時刻 | 終了時刻 | 停止後時刻 |
|---|------|------|---------------|----------|----------|----------|----------|------------|
| 1 | 初期ポジション | - | - | 0秒 | - | 0.0秒 | 0.0秒 | - |
| 2 | 回転運動 | 180deg/sec | 90°（r=270°） | 0.5秒 | 3.0秒 | 0.0秒 | 0.5秒 | 3.5秒 |
| 3 | 並行移動 | 200mm/sec | 550mm | 2.75秒 | 3.0秒 | 3.5秒 | 6.25秒 | 9.25秒 |
| 4 | 長方形移動（縦） | 200mm/sec | 500mm | 2.5秒 | 3.0秒 | 9.25秒 | 11.75秒 | 14.75秒 |
| 5 | 初期ポジションに戻る | 200mm/sec | 550mm | 2.75秒 | 3.0秒 | 14.75秒 | 17.5秒 | 20.5秒 |
| 6 | 135°回転 | 180deg/sec | 135° | 0.75秒 | 3.0秒 | 20.5秒 | 21.25秒 | 24.25秒 |
| 7 | 位置調整移動 | 200mm/sec | 250mm | 1.25秒 | 3.0秒 | 24.25秒 | 25.5秒 | 28.5秒 |
| 8 | 45°回転 | 180deg/sec | 45°（r=180°） | 0.25秒 | - | 28.5秒 | 28.75秒 | - |

#### JSONフレーム生成（0-30秒）

0.5秒間隔でフレームを生成（0.0, 0.5, 1.0, ..., 28.5, 28.75秒）。

各時刻での位置・向きの計算：

```python
# 定数
STRAIGHT_SPEED = 200.0   # mm/sec（直進速度）
ROTATION_SPEED = 180.0   # deg/sec（回転速度）
FRAME_INTERVAL = 0.5     # 秒

# 初期状態（各ロボットIDから計算）
def get_initial_position(robot_id: int) -> Tuple[float, float, float]:
    """初期位置を計算"""
    col = robot_id % 6
    row = robot_id // 6
    x = 84 + col * 163
    y = 85 + row * 191
    rotation = 180.0
    return x, y, rotation

# 0-30秒の動作計算
def calculate_first_locomotion_frame(robot_id: int, t: float) -> Frame:
    """最初のロコモーション（0-30秒）のフレームを計算"""
    x_init, y_init, rotation_init = get_initial_position(robot_id)
    
    # 各動作の時刻に応じて位置・向きを計算
    # （詳細は実装時に各動作の計算式を適用）
    # 例: 回転運動、並行移動、長方形移動、etc.
    
    return Frame(
        time=t,
        use_position=True,
        use_rotation=True,
        position={"x": x, "y": y},
        rotation=rotation,
        sound=None
    )
```

**注意**: 動作の詳細計算（回転方向、移動方向など）は実装時に各動作シーケンスに基づいて実装する。

### Part 2: 螺旋運動（30-50秒）のJSONフレーム生成

### Step 1: 初期パラメータの計算（30台分、一度だけ計算）

各ロボットID `i` (0-29)について、初期パラメータを計算：

```python
# 定数
FIELD_CENTER_X = 491.5
FIELD_CENTER_Y = 466.5
GRID_START_X = 84
GRID_START_Y = 85
GRID_COL_SPACING = 163
GRID_ROW_SPACING = 191
R_BASE = 300.0
R_MIN = 82.0
K = 0.3
R_MAX_MAX = 380.0

# 各ロボットの初期座標（数列的計算）
col_i = i % 6
row_i = i // 6
x_i = GRID_START_X + col_i * GRID_COL_SPACING
y_i = GRID_START_Y + row_i * GRID_ROW_SPACING

# 初期半径と角度
dx_i = x_i - FIELD_CENTER_X
dy_i = y_i - FIELD_CENTER_Y
r0_i = math.sqrt(dx_i**2 + dy_i**2)
theta0_i_rad = math.atan2(dy_i, dx_i)
theta0_i_deg = math.degrees(theta0_i_rad)
theta0_i_deg = (theta0_i_deg + 360) % 360  # 0°～360°に正規化

# 最終半径と拡大加速度
r_max_i = min(R_BASE + (r0_i - R_MIN) * K, R_MAX_MAX)
a_i = 2.0 * (r_max_i - r0_i) / 400.0
```

**出力**: 各ロボットの `(r0_i, theta0_i_deg, r_max_i, a_i)`

### Step 2: 各時刻での位置・向きの計算（0.5秒間隔、41フレーム）

各時刻 `t` (30.0, 30.5, 31.0, ..., 49.5, 50.0秒)について、各ロボット `i` の位置・向きを計算：

```python
# 定数
OMEGA_DEG_PER_SEC = 35.0  # deg/sec
OMEGA_RAD_PER_SEC = math.radians(OMEGA_DEG_PER_SEC)

# 時間の経過
dt = t - 30.0  # 30秒からの経過時間

# 半径の計算（加速的な線形拡大）
r_i_t = r0_i + a_i * dt**2 / 2.0

# 回転角度の計算（反時計回り）
theta_i_t_deg = theta0_i_deg + OMEGA_DEG_PER_SEC * dt
theta_i_t_rad = math.radians(theta_i_t_deg)

# 位置の計算
x_i_t = FIELD_CENTER_X + r_i_t * math.cos(theta_i_t_rad)
y_i_t = FIELD_CENTER_Y + r_i_t * math.sin(theta_i_t_rad)

# 向きの計算（接線方向）
rotation_i_t = theta_i_t_deg + 90.0
rotation_i_t = (rotation_i_t + 360) % 360  # 0°～360°に正規化
```

**出力**: 各時刻・各ロボットの `(x_i_t, y_i_t, rotation_i_t)`

### Step 3: JSONフレームの生成

各時刻 `t`、各ロボット `i` について、JSONフレームを生成：

```python
frame = {
    "time": t,
    "use_position": True,
    "use_rotation": True,
    "position": {
        "x": round(x_i_t, 1),
        "y": round(y_i_t, 1)
    },
    "rotation": round(rotation_i_t, 1),
    "sound": None
}
```

## 実装の構造

### データ構造

```python
# ロボットごとの初期パラメータ
RobotParams = {
    "robot_id": int,
    "r0": float,           # 初期半径（mm）
    "theta0": float,       # 初期角度（度、0°～360°）
    "r_max": float,        # 最終半径（mm）
    "a": float             # 拡大加速度（mm/sec²）
}

# JSONフレーム
Frame = {
    "time": float,
    "use_position": bool,
    "use_rotation": bool,
    "position": {"x": float, "y": float},
    "rotation": float,
    "sound": str | None
}
```

### 実装アルゴリズム（統合版）

```python
def generate_robot_json(robot_id: int) -> List[Frame]:
    """
    ロボットIDから0-50秒のJSONフレームを生成（最初のロコモーション + 螺旋運動）
    
    Args:
        robot_id: ロボットID (0-29)
    
    Returns:
        frames: JSONフレームのリスト（0-50秒、0.5秒間隔）
    """
    frames = []
    
    # Part 1: 最初のロコモーション（0-30秒）
    for t in range(0, 300, 5):  # 0.0, 0.5, ..., 28.5, 28.75秒（0.5秒間隔）
        t_sec = t / 10.0
        if t_sec > 28.75:  # 最終フレーム（28.75秒）
            if t_sec <= 29.0:
                frame = calculate_first_locomotion_frame(robot_id, t_sec)
                frames.append(frame)
            break
        frame = calculate_first_locomotion_frame(robot_id, t_sec)
        frames.append(frame)
    
    # Part 2: 螺旋運動（30-50秒）
    params = calculate_spiral_params(robot_id)
    for t in range(300, 505, 5):  # 30.0, 30.5, ..., 50.0秒（0.5秒間隔）
        t_sec = t / 10.0
        frame = calculate_spiral_frame(robot_id, t_sec, params)
        frames.append(frame)
    
    return frames

def calculate_first_locomotion_frame(robot_id: int, t: float) -> Frame:
    """最初のロコモーション（0-30秒）のフレームを計算"""
    # 上記のPart 1の計算を実装
    # 各動作シーケンスに基づいて位置・向きを計算
    pass

def calculate_spiral_params(robot_id: int) -> RobotParams:
    """螺旋運動の初期パラメータを計算"""
    # 上記のStep 1の計算を実装
    pass

def calculate_spiral_frame(robot_id: int, t: float, params: RobotParams) -> Frame:
    """螺旋運動（30-50秒）の時刻tでのフレームを計算"""
    # 上記のStep 2とStep 3の計算を実装
    pass
```

## 実装に必要な定数

```python
# フィールド定数
FIELD_CENTER_X = 491.5  # mm
FIELD_CENTER_Y = 466.5  # mm

# グリッド定数（初期配置）
GRID_START_X = 84       # mm（左上のX座標）
GRID_START_Y = 85       # mm（左上のY座標）
GRID_COL_SPACING = 163  # mm（列間隔）
GRID_ROW_SPACING = 191  # mm（行間隔）

# 最初のロコモーション定数（0-30秒）
FIRST_LOCOMOTION_START_TIME = 0.0    # 秒
FIRST_LOCOMOTION_END_TIME = 28.75    # 秒
STRAIGHT_SPEED = 200.0                # mm/sec（直進速度）
ROTATION_SPEED = 180.0                # deg/sec（回転速度）
PAUSE_TIME = 3.0                      # 秒（停止時間）

# 螺旋運動定数（30-50秒）
SPIRAL_START_TIME = 30.0   # 秒
SPIRAL_END_TIME = 50.0     # 秒
FRAME_INTERVAL = 0.5       # 秒（フレーム間隔）
OMEGA_DEG_PER_SEC = 35.0   # deg/sec（角速度）

# 最終半径パラメータ（螺旋運動）
R_BASE = 300.0    # mm（最小最終半径）
R_MIN = 82.0      # mm（最小初期半径、推定）
K = 0.3           # 拡大係数
R_MAX_MAX = 380.0 # mm（最大最終半径、速度制限により制約）
```

## 出力JSONファイル仕様

### ファイル名
- `robot_00.json`, `robot_01.json`, ..., `robot_29.json`
- ロボットIDは2桁のゼロパディング

### JSON構造（0-50秒の連続フレーム）

```json
{
  "frames": [
    {
      "time": 0.0,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 84.0, "y": 85.0},
      "rotation": 180.0,
      "sound": null
    },
    {
      "time": 0.5,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 84.0, "y": 85.0},
      "rotation": 270.0,
      "sound": null
    },
    ...（0-30秒の最初のロコモーション）...,
    {
      "time": 28.75,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 247.0, "y": 467.0},
      "rotation": 180.0,
      "sound": null
    },
    {
      "time": 30.0,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 247.0, "y": 467.0},
      "rotation": 180.0,
      "sound": null
    },
    {
      "time": 30.5,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 248.2, "y": 468.1},
      "rotation": 197.5,
      "sound": null
    },
    ...（30-50秒の螺旋運動）...,
    {
      "time": 50.0,
      "use_position": true,
      "use_rotation": true,
      "position": {"x": 320.5, "y": 380.2},
      "rotation": 730.0,
      "sound": null
    }
  ]
}
```

### フレーム仕様
- **時間間隔**: 0.5秒間隔（0.0, 0.5, 1.0, ..., 49.5, 50.0秒）
- **総フレーム数**: 約101フレーム（0-30秒: 約58フレーム、30-50秒: 41フレーム）
- **use_position**: 常に `true`（位置指令を反映）
- **use_rotation**: 常に `true`（回転指令を反映）
- **sound**: 常に `null`（音は80-95秒で別途追加）

### 精度
- **座標**: 小数点以下1桁（mm単位）
- **角度**: 小数点以下1桁（度単位）
- **時間**: 小数点以下1桁（秒単位）

## 計算式まとめ

### Part 1: 最初のロコモーション（0-30秒）の計算式

#### 初期位置計算式

| 計算項目 | 式 | 説明 |
|---------|---|------|
| 列番号 | `col_i = i % 6` | ロボットIDから列を計算 |
| 行番号 | `row_i = i // 6` | ロボットIDから行を計算 |
| 初期X座標 | `x_i = 84 + col_i × 163` | グリッド配置 |
| 初期Y座標 | `y_i = 85 + row_i × 191` | グリッド配置 |
| 初期向き | `rotation_i = 180.0` | 下向き |

#### 動作計算式（各動作シーケンスに基づく）

各動作の時刻に応じて位置・向きを計算：
- **回転運動**: `rotation(t) = rotation_init + 90 × (t / 0.5)`（0.0-0.5秒）
- **並行移動**: `position(t) = position_init + direction × 200 × (t - t_start)`（3.5-6.25秒）
- **長方形移動**: `position(t) = position_init + direction × 200 × (t - t_start)`（9.25-11.75秒）
- etc.（詳細は実装時に各動作シーケンスに基づいて実装）

### Part 2: 螺旋運動（30-50秒）の計算式

#### 初期パラメータ計算式（数列的計算）

| 計算項目 | 式 | 説明 |
|---------|---|------|
| 列番号 | `col_i = i % 6` | ロボットIDから列を計算 |
| 行番号 | `row_i = i // 6` | ロボットIDから行を計算 |
| 初期X座標 | `x_i = 84 + col_i × 163` | グリッド配置 |
| 初期Y座標 | `y_i = 85 + row_i × 191` | グリッド配置 |
| 初期半径 | `r0_i = √((x_i - 491.5)² + (y_i - 466.5)²)` | 中心からの距離 |
| 初期角度 | `theta0_i = atan2(y_i - 466.5, x_i - 491.5)` | 中心からの角度（ラジアン） |
| 初期角度（度） | `theta0_i_deg = degrees(theta0_i)` | 度に変換 |
| 初期角度正規化 | `theta0_i_deg = (theta0_i_deg + 360) % 360` | 0°～360° |
| 最終半径 | `r_max_i = min(300 + (r0_i - 82) × 0.3, 380)` | 速度制限を考慮 |
| 拡大加速度 | `a_i = 2 × (r_max_i - r0_i) / 400` | 20秒で到達 |

### 時刻tでの位置・向き計算式

| 計算項目 | 式 | 説明 |
|---------|---|------|
| 経過時間 | `dt = t - 30.0` | 30秒からの経過 |
| 半径 | `r_i(t) = r0_i + a_i × dt² / 2` | 加速的拡大 |
| 回転角度 | `theta_i(t) = theta0_i + 35 × dt` | 反時計回り、度単位 |
| 回転角度（ラジアン） | `theta_i_rad = radians(theta_i_deg)` | ラジアン変換 |
| X座標 | `x_i(t) = 491.5 + r_i(t) × cos(theta_i_rad)` | 極座標から直交座標へ |
| Y座標 | `y_i(t) = 466.5 + r_i(t) × sin(theta_i_rad)` | 極座標から直交座標へ |
| 向き | `rotation_i(t) = (theta_i(t) + 90) % 360` | 接線方向 |

## 実装の注意点

### 1. 数列的な計算の効率化
- **初期パラメータ**: 30台分を一度だけ計算（O(30) = 定数時間）
- **軌跡計算**: 各ロボットについて41フレームを計算（O(30 × 41) = O(1230)）

### 2. 浮動小数点誤差への対応
- 角度の正規化: `% 360` 演算で範囲を保証
- 座標・角度の丸め: 小数点以下1桁に丸める

### 3. JSONフォーマットの整合性
- 全てのフレームで `use_position = true`, `use_rotation = true`
- `sound` は30-50秒では `null`（音は80-95秒で追加）

## 実装の確認事項

### 計算結果の検証
1. **初期半径の範囲**: 82mm ～ 559mm の範囲内か
2. **最終半径の範囲**: 300mm ～ 380mm の範囲内か
3. **座標の範囲**: フィールド内（X: 34-949mm, Y: 35-898mm）か
4. **角度の範囲**: 0°～360° の範囲内か
5. **時間間隔**: 0.5秒間隔で連続しているか

### パフォーマンス
- 30台分のJSONファイル生成が数秒以内に完了すること

