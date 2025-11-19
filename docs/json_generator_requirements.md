# JSON Generator 要件定義（First/Final Phase）

本仕様は `docs/ground_rules.md` と `docs/locomotion_phase_summary.md` に準拠し、約 0–32.5 秒（First Phase）、32.5–50 秒（Spiral Phase）、130–150 秒（Final Phase）の toio 群制御 JSON を自動生成するための要件をまとめる。対象は象限ベースの 5 行 × 4 列フォーメーション（20 台）で、残り 10 台の扱いは別途定義する。

---

## 1. 入出力
- **入力**（CLI もしくは設定ファイルで指定可能にすること）  
  - `rows`, `cols`: グリッド寸法（既定 5 × 4）。  
  - `gap_x`, `gap_y` [mm]: toio 同士の横/縦空白（既定 `gap_x = 10.7`, `gap_y = 18`）。  
  - `active_robot_ids`: JSON を生成する `robot_id` のリスト（20 件必須）。  
  - `output_dir`: 書き出し先ディレクトリ。  
  - `phases`: `first`, `final` を個別/同時に指定できるようにする。
- **出力**  
  - 各 `robot_id` ごとの JSON ファイル (`robot_{id:02d}.json`)。  
  - JSON スキーマは `docs/ground_rules.md` §9 の `frames` 配列（`time/t`, `up`, `ur`, `x`, `y`, `rotation/angle`, `stop_distance/sd`, `angle_tolerance/at`, `sound`）に厳密準拠させ、`position` オブジェクトは使わない。  
  - 追加で全ロボット分をひとまとめにした `all_robots.json` を `{ "sets": [ { "id": "robot_00", "frames": [...] }, ... ] }` 形式で出力する。

---

## 2. 座標・フォーメーション定義
- フィールド座標は Position ID (34–949, 35–898)。  
- 象限中心:
  | ラベル | 座標 (mm) |
  |--------|-----------|
  | Q1 (左上) | (262.75, 250.75) |
  | Q2 (左下) | (262.75, 682.25) |
  | Q3 (右下) | (720.25, 682.25) |
  | Q4 (右上) | (720.25, 250.75) |
  | Center    | (491.5, 466.5)   |
- フォーメーション計算:
  - セル幅 `cell_x = 32 + gap_x`、セル高さ `cell_y = 72 + gap_y`。  
  - 群長方形の幅/高さ = `32 + (cols-1)*cell_x`, `72 + (rows-1)*cell_y`。  
  - 各ロボットの相対オフセットは `((col - (cols-1)/2)*cell_x, (row - (rows-1)/2)*cell_y)` を中心に加算する（Y 方向は下向きを正とする）。  
  - `active_robot_ids` とフォーメーションスロットの対応は row-major をデフォルトとし、外部から差し替え可能にする。

---

## 3. タイムライン（First / Spiral / Final）

### First Phase (0–30 秒)
| 区間 | 時間帯 [s] | 種別 | 内容 |
|------|-----------|------|------|
| 1 | 0.0–1.0 | Rotation | Q2 で 90°（右向き）へ回頭 |
| 2 | 1.0–4.0 | Translate | Q2 → Q3（横 457.5 mm, 3 s） |
| 3 | 4.0–6.0 | Pause | 停止 |
| 4 | 6.0–7.0 | Rotation | 0°（上向き）へ回頭 |
| 5 | 7.0–10.0 | Translate | Q3 → Q4 |
| 6 | 10.0–12.0 | Pause | 停止 |
| 7 | 12.0–13.0 | Rotation | 270°（左向き）へ回頭 |
| 8 | 13.0–16.0 | Translate | Q4 → Q1 |
| 9 | 16.0–18.0 | Pause | 停止 |
| 10 | 18.0–19.0 | Rotation | 180°（下向き）へ回頭 |
| 11 | 19.0–22.0 | Translate | Q1 → Q2 |
| 12 | 22.0–24.0 | Pause | 停止 |
| 13 | 24.0–25.0 | Rotation | 中心方向へ回頭（ベクトル Q2→Center） |
| 14 | 25.0–28.0 | Translate | Q2 → Center |
| 15 | 28.0–30.0 | Pause | 中心滞留 |
| 16 | 30.0–31.0 | Rotation | 初期姿勢（180°）へ戻す |
| 17 | 31.0–32.5 | Pause | バッファ（位置保持） |

### Spiral Phase (32.5–50 秒)
- **初期状態**: 32.5 s 時点で 5×4 フォーメーションの各ロボットがフィールド中心 (491.5, 466.5) に配置され、`use_position=true`/`use_rotation=true` のフレームで静止姿勢を指示する。
- **半径カーブ**: 初期半径 `r0` を中心距離、最大半径 `r_max ≈ 191 mm` とし、`r_target = min(400, r0 + (400 - r_max) * (r0 / r_max))`。`f(t) = ((t-32.5)/17.5)^{1.1}` を用いて `r(t) = r0 + (r_target - r0) * f(t)` で外向きに拡散。
- **角速度**: 接線速度 160 mm/s を上限としつつ `ω_fast = ω_cap * (r0 / r_max)`（`ω_cap = min(160 / r(t), 45°/s)`）で中心ほど角速度を抑制。45 s〜50 s で `ω` を 10°/s へ線形ブレンドし、さらに 1.5 s の立ち上げランプを掛けて急激な回転を抑える。
- **ノイズ**: 45 s 以降、`t_noise = 45 + ((robot_id % 5) - 2) × 0.2` で開始時刻をずらしたランダムウォークを付与（半径 ±5 mm、角度 ±2° を上限に 0.5 s ごとに ±1.5 mm / ±0.4° を積算）。
- **出力**: 32.5–50 s を 0.5 s 刻みでサンプリングし、全フレームで `use_position=true` / `use_rotation=true` とする。

### Final Phase (130–150 秒)
| 区間 | 時間帯 [s] | 種別 | 内容 |
|------|-----------|------|------|
| 1 | 130.0–130.5 | Rotation | Center で Q3 方向へ |
| 2 | 130.5–135.5 | Translate | Center → Q3 |
| 3 | 135.5–136.0 | Rotation | Q4 方向へ |
| 4 | 136.0–141.0 | Translate | Q3 → Q4 |
| 5 | 141.0–141.5 | Rotation | Q1 方向へ |
| 6 | 141.5–146.5 | Translate | Q4 → Q1 |
| 7 | 146.5–147.0 | Rotation | Q2 方向へ |
| 8 | 147.0–150.0 | Translate | Q1 → Q2（5 s 中 3 s を移動、残り 2 s は静止可） |

---

## 4. フレーム生成ルール
- **コマンド発行タイミング**  
  - translate: 区間開始時刻に 1 フレームのみ発行し、`x`/`y` に移動完了時の目標座標を入れる（`use_position=true`, `stop_distance=20.0` を付与）。  
  - rotation: 区間開始時刻に 1 フレームのみ発行し、`rotation` に到達角度を入れる（`use_rotation=true`, `angle_tolerance=5.0` を付与）。  
  - pause: 状態変化が無い区間だが、必要に応じて `use_position=false`, `use_rotation=false` の監視フレームを開始時刻に打つ。必要であれば `use_position` と `use_rotation` を同時に `true` にしてもよい（中心移動中に姿勢を合わせたい場合など）。
- **回転角**: 0°=上, 90°=右, 180°=下, 270°=左。中心移動前の回転は `atan2` で算出し四捨五入。
- **速度チェック**: 各 translate 区間で `distance / duration <= 200`（First）または `<=105`（Final）を assert。  
- **境界/瞬間移動検証**:
  - 生成後、隣接フレーム間の距離が `speed_limit * Δt` を超えないことを自動チェック。  
  - すべての `position` がフィールド矩形 (34–949, 35–898) 内にあることを検証。

---

## 5. JSON Generator 実装要件
1. **構成**: `quadrant_json_generator.py`（仮）として Python 3.10 以上。`argparse` で CLI 引数を受け取る。
2. **手順**:
   - フォーメーションオフセット計算 → 象限中心の軌跡生成 → 各ロボットへ適用。  
   - フェーズごとにセンターの線形補間を行い、指示時刻リストを自動生成。  
   - 32.5–50 s は `generate_spiral_frames(initial_states)` のような専用関数で処理し、初期状態（中心整列）から半径/角度/ノイズの方程式を適用して 0.5 s 刻みのフレームを生成する。  
   - 検証（境界・速度・時間ソート）を行い、違反時は例外。
   - `validate_json.py` で 32.5 s 基準フレームの存在、31–50 s の速度/回転制限、0.5 s グリッド、ロボット間距離（閾値 20 mm）を自動チェックし、違反があればレポートする。
3. **出力**: 指定ディレクトリに per-robot JSON。`--summary` 指定時にはシーケンス要約（CSV/Markdown）も出力可。
4. **拡張性**:  
   - 将来的に `phases` に 30–50 秒や 50–120 秒を追加できるよう、フェーズを抽象化（Segment のリストを辞書に保持）。  
   - `active_robot_ids` は任意順序を許容し、フォーメーションスロットとの結び付けをログに残す。

---

この要件を満たすジェネレーターを実装し、`docs/ground_rules.md` および `docs/locomotion_phase_summary.md` の整合を常に維持すること。

